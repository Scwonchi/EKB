// Распознавание нейросетью на ESP32-S3 + LD2410C (дрон сел, моторы выключены)
// + координаты GPS GY-NEO6MV2 и направление по компасу GY-27.
//
// Рядом со скетчем: nn_core.h, model.h (создаёт train.py), nav_core.h, nav_hw.h.
// Подключение:
//   LD2410C:    VCC -> 5V,  GND -> GND, TX -> GPIO18, RX -> GPIO17
//   GY-NEO6MV2: VCC -> 5V,  GND -> GND, TX -> GPIO16, RX -> GPIO15
//   GY-27:      VCC -> 3V3, GND -> GND, SDA -> GPIO8, SCL -> GPIO9
// Компас ставится стрелкой X туда же, куда смотрит радар, и настраивается скетчем nav_test
// (настройки хранятся в ESP и подхватываются здесь автоматически).
// Монитор порта 115200. Команды навигации те же, что в nav_test (i — список).
//
// Работа: кадры радара берутся 10 раз в секунду, копятся в окно 10 с.
// Раз в секунду сеть оценивает окно. Тревога — если вероятность «человек есть»
// выше ALARM_P в CONFIRM окнах подряд; отбой — если ниже CLEAR_P столько же окон.

#include <Arduino.h>
#include "nn_core.h"
#include "nav_hw.h"
#include "report.h"
#include <TelemetryLink.h>     // библиотека TelemetryProto (проект telemetry, папка esp/libraries)

// ---------- Связь с сервером (protobuf) ----------
// Пустой WIFI_SSID — связь выключена, работает только монитор порта.
const char *WIFI_SSID   = "drone-net";
const char *WIFI_PASS   = "12345678";
const char *SERVER_HOST = "192.168.4.1";     // ESP-шлюз (esp/gateway); при сервере прямо на ПК — IP ПК
const uint16_t SERVER_PORT = 9000;
const uint32_t FIRMWARE_VERSION = 3;
TelemetryLink telem;

#define RADAR_RX   18
#define RADAR_TX   17
#define RADAR_BAUD 256000
#define SAMPLE_MS  (1000 / NN_FS)

const float ALARM_P = 0.7f;
const float CLEAR_P = 0.3f;
const int   CONFIRM = 3;

HardwareSerial Radar(1);

// ---------- Радар ----------
uint8_t mv[9] = {0}, stE[9] = {0};
uint16_t radarDistCm = 0;          // дистанция цели по данным самого радара
bool engMode = false, haveFrame = false;
uint32_t lastFrameMs = 0, lastEngTry = 0, frames = 0;

void sendCmd(uint16_t cmd, const uint8_t *val, uint8_t vlen) {
  uint16_t len = 2 + vlen;
  uint8_t f[32]; int i = 0;
  f[i++] = 0xFD; f[i++] = 0xFC; f[i++] = 0xFB; f[i++] = 0xFA;
  f[i++] = len & 0xFF; f[i++] = len >> 8;
  f[i++] = cmd & 0xFF; f[i++] = cmd >> 8;
  for (int k = 0; k < vlen; k++) f[i++] = val[k];
  f[i++] = 0x04; f[i++] = 0x03; f[i++] = 0x02; f[i++] = 0x01;
  Radar.write(f, i);
  Radar.flush();
  delay(100);
}

void enableEngineering() {
  const uint8_t on[2] = {0x01, 0x00};
  sendCmd(0x00FF, on, 2);
  sendCmd(0x0062, nullptr, 0);
  sendCmd(0x00FE, nullptr, 0);
}

static const uint8_t HDR[4] = {0xF4, 0xF3, 0xF2, 0xF1};
uint8_t buf[64];
uint8_t pos = 0;
uint16_t frameLen = 0;

void handleFrame() {
  if (buf[7] != 0xAA) return;
  frames++;
  if (buf[6] != 0x01) { engMode = false; return; }
  engMode = true;
  radarDistCm = buf[8] ? (buf[15] | (buf[16] << 8)) : 0;   // 0 — радар цели не видит
  uint8_t nM = min(buf[17] + 1, 9);
  uint8_t nS = min(buf[18] + 1, 9);
  for (int g = 0; g < 9; g++) mv[g]  = g < nM ? buf[19 + g] : 0;
  for (int g = 0; g < 9; g++) stE[g] = g < nS ? buf[19 + nM + g] : 0;
  haveFrame = true;
  lastFrameMs = millis();
}

void readRadar() {
  while (Radar.available()) {
    uint8_t b = Radar.read();
    if (pos < 4) {
      if (b == HDR[pos]) buf[pos++] = b;
      else if (b == HDR[0]) { buf[0] = b; pos = 1; }
      else pos = 0;
      continue;
    }
    buf[pos++] = b;
    if (pos == 6) {
      uint16_t len = buf[4] | (buf[5] << 8);
      if (len < 13 || len > 50) { pos = 0; continue; }
      frameLen = 6 + len + 4;
    }
    if (pos >= 6 && pos == frameLen) {
      if (buf[frameLen - 4] == 0xF8 && buf[frameLen - 1] == 0xF5) handleFrame();
      pos = 0;
    }
  }
}

// ---------- Донесение об обнаружении (структура Report — в report.h) ----------
Report makeReport(uint8_t type, int cls, float conf) {
  static uint16_t id = 0;
  if (type == 1) id++;
  const nav::State &s = nav::st;
  Report r = {};
  r.type = type; r.alarmId = id; r.cls = cls; r.confidence = (uint8_t)(conf * 100);
  r.distCm = radarDistCm;
  r.heading10 = 0xFFFF;
  if (s.gpsFresh) {
    r.flags |= 1;
    r.droneLat = (int32_t)(s.gps.lat * 1e7); r.droneLon = (int32_t)(s.gps.lon * 1e7);
    r.sats = s.gps.sats; r.hdop10 = (uint8_t)min(255.0f, s.gps.hdop * 10);
  }
  bool headOk = !isnan(s.heading) && s.calibrated && !s.magDisturbed;
  if (headOk) { r.flags |= 2; r.heading10 = (uint16_t)(s.heading * 10); }
  if (s.magDisturbed) r.flags |= 8;
  if (s.gpsFresh && headOk && radarDistCm > 0) {
    double la, lo;
    nav::offsetPoint(s.gps.lat, s.gps.lon, s.heading, radarDistCm / 100.0f, la, lo);
    r.tgtLat = (int32_t)(la * 1e7); r.tgtLon = (int32_t)(lo * 1e7);
    r.flags |= 4;
  }
  return r;
}

void printReport(const Report &r) {
  Serial.printf(">>> ТРЕВОГА №%u: человек (%s), уверенность %u%%\n",
                r.alarmId, NN_LABELS[r.cls], r.confidence);
  if (r.flags & 1)
    Serial.printf("    Дрон:   %.6f, %.6f (спутников %u, HDOP %.1f)\n",
                  r.droneLat / 1e7, r.droneLon / 1e7, r.sats, r.hdop10 / 10.0);
  else
    Serial.println("    Дрон:   координат нет (GPS без фиксации)");
  if (r.flags & 2)
    Serial.printf("    Радар смотрит на %.0f° (%s)\n", r.heading10 / 10.0, nav::compassName(r.heading10 / 10.0f));
  else
    Serial.printf("    Направление неизвестно (%s)\n",
                  (r.flags & 8) ? "магнитная помеха" : nav::st.calibrated ? "нет данных компаса" : "компас не откалиброван");
  if (r.distCm) Serial.printf("    Дистанция до цели ~%.1f м\n", r.distCm / 100.0);
  if (r.flags & 4)
    Serial.printf("    Примерная точка цели: %.6f, %.6f  (±%.0f м по GPS, направление ±60° — конус радара)\n",
                  r.tgtLat / 1e7, r.tgtLon / 1e7, max(3.0, r.hdop10 / 10.0 * 2.5));
  // Машиночитаемая строка для программы оператора
  Serial.printf("ALARM,%u,%s,%u,%u,%.7f,%.7f,%u,%.1f,%.1f,%.7f,%.7f,%u\n",
                r.alarmId, NN_LABELS[r.cls], r.confidence, r.distCm,
                r.droneLat / 1e7, r.droneLon / 1e7, r.sats, r.hdop10 / 10.0,
                r.heading10 == 0xFFFF ? -1.0 : r.heading10 / 10.0,
                r.tgtLat / 1e7, r.tgtLon / 1e7, r.flags);
}

// ---------- Сообщения на сервер ----------
bool  alarmOn = false;
int   lastClass = -1;          // последний ответ сети
float lastPPerson = 0;

telemetry_v1_DetectionClass classFromLabel(int cls) {
  if (cls < 0) return telemetry_v1_DetectionClass_DETECTION_CLASS_UNSPECIFIED;
  const char *l = NN_LABELS[cls];
  if (!strcmp(l, "pusto"))       return telemetry_v1_DetectionClass_DETECTION_CLASS_EMPTY;
  if (!strcmp(l, "dvizhetsya"))  return telemetry_v1_DetectionClass_DETECTION_CLASS_MOVING;
  if (!strcmp(l, "nepodvizhen")) return telemetry_v1_DetectionClass_DETECTION_CLASS_STATIONARY;
  if (!strcmp(l, "pomeha"))      return telemetry_v1_DetectionClass_DETECTION_CLASS_INTERFERENCE;
  return telemetry_v1_DetectionClass_DETECTION_CLASS_UNSPECIFIED;
}

// Снимок telemetry.v1.Telemetry: полное состояние, раз в секунду (сеть отвечает раз в секунду).
// Целей с координатами у LD2410C нет — список tracks пустой.
void publishTelemetry() {
  const nav::State &s = nav::st;
  telemetry_v1_Telemetry t = telemetry_v1_Telemetry_init_zero;
  t.firmware_version = FIRMWARE_VERSION;
  t.uptime_s = millis() / 1000;
  t.radar_ok = engMode && millis() - lastFrameMs < 1000;
  t.detection_class = classFromLabel(lastClass);
  t.person_probability = lastPPerson;
  if (s.gpsFresh) {
    t.has_position = true;
    t.position.lat = s.gps.lat; t.position.lon = s.gps.lon;
    t.has_hdop = true; t.hdop = s.gps.hdop;
  }
  t.satellites = s.gps.sats;
  t.heading_valid = !isnan(s.heading) && s.calibrated && !s.magDisturbed;
  if (t.heading_valid) { t.has_heading_deg = true; t.heading_deg = s.heading; }
  t.rssi_dbm = WiFi.RSSI();
  t.target_count = alarmOn ? 1 : 0;
  t.alarm_active = alarmOn;
  telem.publish(t);
}

// ---------- Окно и сеть ----------
static float ring[NN_WIN][NN_CH];
static float win[NN_WIN][NN_CH];
static float feat[NN_FEAT];
int ringPos = 0, filled = 0, sinceRun = 0;

int above = 0, below = 0;

void runNet() {
  for (int n = 0; n < NN_WIN; n++)                       // от старых к новым
    memcpy(win[n], ring[(ringPos + n) % NN_WIN], sizeof(win[n]));

  uint32_t t0 = micros();
  nn::features(win, feat);
  float prob[NN_CLASSES];
  int best = nn::predict(feat, prob);
  uint32_t dt = micros() - t0;

  float pPerson = 0;
  int bestPerson = best;
  for (int j = 0; j < NN_CLASSES; j++)
    if (NN_IS_PERSON[j]) {
      pPerson += prob[j];
      if (!NN_IS_PERSON[bestPerson] || prob[j] > prob[bestPerson]) bestPerson = j;
    }

  lastClass = best;
  lastPPerson = pPerson;

  if (pPerson >= ALARM_P) { above++; below = 0; }
  else if (pPerson <= CLEAR_P) { below++; above = 0; }
  else { above = 0; below = 0; }

  if (!alarmOn && above >= CONFIRM) {
    alarmOn = true;
    Report r = makeReport(1, bestPerson, pPerson);
    printReport(r);
  } else if (alarmOn && below >= CONFIRM) {
    alarmOn = false;
    Report r = makeReport(2, best, 1 - pPerson);
    Serial.printf(">>> Отбой тревоги №%u: человека нет\n", r.alarmId);
  }

  const nav::State &s = nav::st;
  Serial.printf("[%s] %-12s", alarmOn ? "ТРЕВОГА" : "  норма", NN_LABELS[best]);
  for (int j = 0; j < NN_CLASSES; j++) Serial.printf(" %s=%.2f", NN_LABELS[j], prob[j]);
  Serial.printf(" | %lu мкс | GPS %s | курс %s\n", dt,
                s.gpsFresh ? String(s.gps.sats).c_str() : "—",
                isnan(s.heading) ? "—" : (String(s.heading, 0) + "°").c_str());
}

void setup() {
  Serial.begin(115200);
  Radar.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX, RADAR_TX);
  nav::begin();
  delay(300);
  enableEngineering();
  Serial.printf("Сеть: %d классов, окно %d с. Накапливаю данные...\n", NN_CLASSES, NN_WIN / NN_FS);
  nav::printSettings();
  if (WIFI_SSID[0]) {
    telem.begin({WIFI_SSID, WIFI_PASS, SERVER_HOST, SERVER_PORT});
    Serial.printf("Связь: Wi-Fi «%s», сервер %s:%u\n", WIFI_SSID, SERVER_HOST, SERVER_PORT);
  }
}

void loop() {
  readRadar();
  nav::poll();
  static uint32_t lastTelemetry = 0;
  if (WIFI_SSID[0] && millis() - lastTelemetry >= 1000) { lastTelemetry = millis(); publishTelemetry(); }

  static String in;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (in.length()) { nav::command(in); in = ""; } }
    else in += c;
  }

  if (!engMode && frames > 0 && millis() - lastEngTry > 3000) {
    lastEngTry = millis();
    enableEngineering();
  }

  static uint32_t nextSample = 0;
  uint32_t now = millis();
  if ((int32_t)(now - nextSample) < 0) return;
  if (nextSample && now - nextSample > 500) { filled = 0; sinceRun = 0; }  // был перерыв (калибровка)
  nextSample = now + SAMPLE_MS;

  if (!haveFrame || !engMode || now - lastFrameMs > 1000) {
    static uint32_t lastWarn = 0;
    if (now - lastWarn > 2000) { lastWarn = now; Serial.println("Нет данных от радара"); }
    filled = 0; sinceRun = 0;                            // окно должно быть непрерывным
    return;
  }

  for (int g = 0; g < 9; g++) { ring[ringPos][g] = mv[g]; ring[ringPos][9 + g] = stE[g]; }
  ringPos = (ringPos + 1) % NN_WIN;
  if (filled < NN_WIN) filled++;
  sinceRun++;

  if (filled == NN_WIN && sinceRun >= NN_STEP) {
    sinceRun = 0;
    runNet();
  }
}
