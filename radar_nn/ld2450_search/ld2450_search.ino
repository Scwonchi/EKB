// Поиск людей: ESP32-S3 + LD2450 + GPS + компас + связь с сервером
// (снимки telemetry.v1.Telemetry ~5 раз в секунду, без ответов сервера).
//
// Рядом со скетчем: ld2450.h, tracker.h, nav_core.h, nav_hw.h.
// Библиотека TelemetryProto (проект telemetry) — в папке библиотек Arduino.
//
// Подключение:
//   LD2450:     5V -> 5V,  GND -> GND, TX -> GPIO18, RX -> GPIO17
//   GY-NEO6MV2: VCC -> 5V,  GND -> GND, TX -> GPIO16, RX -> GPIO15
//   Компас:     VCC -> 3V3, GND -> GND, SDA -> GPIO8, SCL -> GPIO9
//
// LD2450 даёт координаты до 3 целей, поэтому известны и дистанция, и угол:
// азимут на человека = курс радара по компасу + угол цели.
//
// Монитор порта 115200, «Новая строка». Команды:
//   bg     — запомнить фон: 20 с, в зоне радара никого нет
//   bgoff  — забыть фон
//   rx     — сменить знак угла (если цель справа показывается слева)
//   tracks — печатать все треки вкл/выкл
//   и команды компаса/GPS (i — список)

#include <Arduino.h>
#include <Preferences.h>
#include "ld2450.h"
#include "tracker.h"
#include "nav_hw.h"
#include <TelemetryLink.h>

// ---------- Настройки ----------
#define RADAR_RX   18
#define RADAR_TX   17
#define RADAR_BAUD 256000

const uint32_t CLEAR_MS = 5000;         // отбой, если людей нет столько, мс

// Связь с сервером. Пустой WIFI_SSID — связь выключена.
const char *WIFI_SSID   = "drone-net";
const char *WIFI_PASS   = "12345678";
const char *SERVER_HOST = "192.168.4.1";     // ESP-шлюз (esp/gateway); при сервере прямо на ПК — IP ПК
const uint16_t SERVER_PORT = 9000;
const uint32_t FIRMWARE_VERSION = 3;

HardwareSerial Radar(1);
ld2450::Parser parser;
trk::Tracker tracker;
TelemetryLink telem;

int8_t angleSign = 1;                   // хранится во флеш-памяти
bool printTracks = false;
uint32_t lastFrameMs = 0;

bool alarmOn = false;
uint32_t alarmId = 0, lastPersonMs = 0;

// ---------- Радар ----------
void radarWrite(const uint8_t *b, int n) { Radar.write(b, n); Radar.flush(); }
void waitMs(int ms) { delay(ms); }

bool radarOk() { return parser.frames > 0 && millis() - lastFrameMs < 1000; }

// Угол цели относительно оси радара, ° (+ по часовой, если смотреть по направлению радара)
float targetAngle(const trk::Track &t) {
  return angleSign * atan2f(-t.x, t.y) * nav::DEG;
}

bool headingOk() {
  const nav::State &s = nav::st;
  return !isnan(s.heading) && s.calibrated && !s.magDisturbed;
}

void loadRadarSettings() {
  Preferences p;
  p.begin("radar", true);
  angleSign = p.getChar("asign", 1) < 0 ? -1 : 1;
  p.end();
}

void saveRadarSettings() {
  Preferences p;
  p.begin("radar", false);
  p.putChar("asign", angleSign);
  p.end();
}

telemetry_v1_DetectionClass kindToClass(const trk::Track *t) {
  if (!t) return telemetry_v1_DetectionClass_DETECTION_CLASS_EMPTY;
  return t->kind == trk::KIND_MOVING ? telemetry_v1_DetectionClass_DETECTION_CLASS_MOVING
                                     : telemetry_v1_DetectionClass_DETECTION_CLASS_STATIONARY;
}

// ---------- Снимок для сервера (telemetry.v1.Telemetry) ----------
// Полное состояние ~5 раз в секунду; сервер сам замечает начало и конец тревоги по alarm_active.
const uint32_t TELEMETRY_MS = 200;

void publishTelemetry() {
  const nav::State &s = nav::st;
  const trk::Track *b = tracker.learning() ? nullptr : tracker.best();
  telemetry_v1_Telemetry t = telemetry_v1_Telemetry_init_zero;
  t.firmware_version = FIRMWARE_VERSION;
  t.uptime_s = millis() / 1000;
  t.radar_ok = radarOk();
  t.detection_class = kindToClass(b);
  t.person_probability = b ? b->hitRatio() : 0;
  if (s.gpsFresh) {
    t.has_position = true;
    t.position.lat = s.gps.lat; t.position.lon = s.gps.lon;
    t.has_hdop = true; t.hdop = s.gps.hdop;
  }
  t.satellites = s.gps.sats;
  t.heading_valid = headingOk();
  if (t.heading_valid) { t.has_heading_deg = true; t.heading_deg = s.heading; }
  t.rssi_dbm = WiFi.RSSI();
  t.target_count = tracker.learning() ? 0 : tracker.personCount();
  if (!tracker.learning())
    for (auto &tr : tracker.tracks) {
      if (!tr.used || t.tracks_count >= 6) continue;
      telemetry_v1_Track &o = t.tracks[t.tracks_count++];
      o.id = tr.id;
      o.distance_m = tr.range();
      o.angle_deg = targetAngle(tr);
      o.speed_mps = tr.speed;
      o.confirmed = tr.confirmed;
      o.detection_class = tr.confirmed ? kindToClass(&tr) : telemetry_v1_DetectionClass_DETECTION_CLASS_UNSPECIFIED;
      o.hit_ratio = tr.hitRatio();
    }
  t.learning_background = tracker.learning();
  t.background_cells = tracker.backgroundCells();
  t.alarm_active = alarmOn;      // держится не меньше CLEAR_MS (5 с) — сервер не пропустит между снимками
  telem.publish(t);
}

// ---------- Тревога (в монитор порта; на сервер она уходит в снимках) ----------
void reportAlarm(const trk::Track &tr) {
  const nav::State &s = nav::st;
  float dist = tr.range(), ang = targetAngle(tr);
  bool head = headingOk();
  float bearing = head ? nav::wrap360(s.heading + ang) : NAN;

  Serial.printf(">>> ТРЕВОГА №%u: человек %s, дистанция %.1f м, угол %+.0f° от оси радара, людей в зоне %d\n",
                alarmId, tr.kind == trk::KIND_MOVING ? "движется" : "неподвижен", dist, ang, tracker.personCount());
  if (s.gpsFresh) Serial.printf("    Устройство: %.6f, %.6f (спутников %d, HDOP %.1f)\n", s.gps.lat, s.gps.lon, s.gps.sats, s.gps.hdop);
  else Serial.println("    Устройство: координат нет (GPS без фиксации)");
  if (head) Serial.printf("    Азимут на человека: %.0f° (%s), радар смотрит на %.0f°\n", bearing, nav::compassName(bearing), s.heading);
  else Serial.printf("    Азимут неизвестен (%s)\n", s.magChip == nav::MAG_NONE ? "компас не найден" :
                     !s.calibrated ? "компас не откалиброван" : "магнитная помеха");
  if (s.gpsFresh && head) {
    double la, lo;
    nav::offsetPoint(s.gps.lat, s.gps.lon, bearing, dist, la, lo);
    Serial.printf("    Точка человека: %.6f, %.6f (±%.0f м по GPS, ±%.0f м по углу)\n", la, lo,
                  fmaxf(3.0f, s.gps.hdop * 2.5f), dist * 0.1f + 0.2f);
  }
}

void reportCleared() {
  Serial.printf(">>> Отбой тревоги №%u: людей не видно %lu с\n", alarmId, (unsigned long)(CLEAR_MS / 1000));
}

void updateAlarm() {
  uint32_t now = millis();
  const trk::Track *b = tracker.learning() ? nullptr : tracker.best();
  if (b) lastPersonMs = now;
  if (!alarmOn && b) { alarmOn = true; alarmId++; reportAlarm(*b); }
  else if (alarmOn && !b && now - lastPersonMs > CLEAR_MS) { alarmOn = false; reportCleared(); }
}

// ---------- Вывод и команды ----------
void printStatus() {
  const nav::State &s = nav::st;
  uint32_t now = millis();
  if (!radarOk()) { Serial.println("Нет данных от радара (TX радара -> GPIO18, питание 5 В)"); return; }
  if (tracker.learning()) {
    Serial.printf("Запоминаю фон: ещё %lu с, в зоне никого не должно быть\n", (unsigned long)(tracker.learnLeftMs(now) / 1000 + 1));
    return;
  }
  const trk::Track *b = tracker.best();
  Serial.printf("[%s] людей %d", alarmOn ? "ТРЕВОГА" : "  норма", tracker.personCount());
  if (b) Serial.printf(" | ближайший надёжный: %.1f м, %+.0f°, %s", b->range(), targetAngle(*b),
                       b->kind == trk::KIND_MOVING ? "движется" : "неподвижен");
  char link[160];
  if (!WIFI_SSID[0]) snprintf(link, sizeof(link), "выкл");
  else if (telem.connected) snprintf(link, sizeof(link), "есть, снимков %lu, обрывов %lu, исправлено %lu",
                                     (unsigned long)telem.sentCount, (unsigned long)telem.reconnects, (unsigned long)telem.fixedCount);
  else snprintf(link, sizeof(link), "%s", telem.wifiUp ? "Wi-Fi есть, сервер не принимает подключение" : "нет Wi-Fi");
  Serial.printf(" | GPS %s | курс %s | фон %s | связь %s\n",
                s.gpsFresh ? "есть" : "нет",
                headingOk() ? (String(s.heading, 0) + "°").c_str() : "—",
                tracker.backgroundCells() ? (String(tracker.backgroundCells()) + " кл.").c_str() : "нет",
                link);
  if (printTracks)
    for (auto &t : tracker.tracks)
      if (t.used)
        Serial.printf("    трек %lu: x %+.2f y %.2f м, %.1f м, %+.0f°, v %.2f м/с, видно %.0f%%, %s%s\n",
                      (unsigned long)t.id, t.x, t.y, t.range(), targetAngle(t), t.speed, t.hitRatio() * 100,
                      t.confirmed ? "ЧЕЛОВЕК" : "проверка", t.confirmed ? (t.kind == trk::KIND_MOVING ? ", движется" : ", неподвижен") : "");
}

void command(String s) {
  s.trim();
  if (s == "bg") {
    tracker.startBackground(millis());
    Serial.println("Запоминаю фон 20 с — отойдите из зоны радара");
  } else if (s == "bgoff") {
    tracker.clearBackground();
    Serial.println("Фон забыт");
  } else if (s == "rx") {
    angleSign = -angleSign; saveRadarSettings();
    Serial.printf("Знак угла: %+d\n", angleSign);
  } else if (s == "tracks") {
    printTracks = !printTracks;
  } else if (!nav::command(s)) {
    Serial.println("Команды: bg | bgoff | rx | tracks | i — компас и GPS");
  }
}

// ---------- Основное ----------
void setup() {
  Serial.begin(115200);
  Radar.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX, RADAR_TX);
  delay(300);
  ld2450::setMultiTarget(radarWrite, waitMs);
  loadRadarSettings();
  nav::begin();
  Serial.println("Поиск людей: LD2450 + GPS + компас");
  nav::printSettings();
  if (WIFI_SSID[0]) {
    telem.begin({WIFI_SSID, WIFI_PASS, SERVER_HOST, SERVER_PORT});
    Serial.printf("Связь: Wi-Fi «%s», сервер %s:%u\n", WIFI_SSID, SERVER_HOST, SERVER_PORT);
  }
  Serial.println("Совет: после посадки дайте команду bg (20 с никого в зоне) — радар запомнит фон");
}

void loop() {
  while (Radar.available()) {
    if (parser.feed(Radar.read())) {
      lastFrameMs = millis();
      tracker.update(parser.t, 3, lastFrameMs);
      updateAlarm();
    }
  }
  if (!radarOk()) { tracker.clearTracks(); updateAlarm(); }   // радар замолчал — треки недействительны

  nav::poll();
  static uint32_t lastTelemetry = 0;
  if (WIFI_SSID[0] && millis() - lastTelemetry >= TELEMETRY_MS) { lastTelemetry = millis(); publishTelemetry(); }

  static String in;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (in.length()) { command(in); in = ""; } }
    else in += c;
  }

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 1000) { lastPrint = millis(); printStatus(); }
}
