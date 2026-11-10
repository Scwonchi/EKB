// Разведчик: радар LD2450 + гироскоп MPU-6050 (+ камера, + компас по желанию).
// Секторное сканирование с остановками, карта обнаружения, тревога с направлением и
// дистанцией, видеопоток с камеры (IP-камера, MJPEG), журнал событий миссии.
// Связь: снимки telemetry.v1.Telemetry ~5 раз в секунду через ретранслятор на носителе
// (запасной вариант — Wi-Fi роутер напрямую) на сервер iot-go.
//
// Устройство не управляет полётом: его поворачивает носитель (сейчас — пилот DJI Flip).
// Остановка = курс по гироскопу не меняется ~1 с; на остановке радар слушает 3 с
// (светодиод горит — держать, погас — поворачивать). Когда остановки закрыли весь круг,
// сканирование завершено — дальше устройство работает как неподвижный датчик.
//
// Рядом со скетчем: mission.h, scan.h, tracker.h, ld2450.h, camera.h, stream.h, gyro.h, nav_core.h, nav_hw.h.
// Видео: http://<адрес разведчика>/ (через ретранслятор — http://<ретранслятор>:8000/<номер>/), см. stream.h.
// Библиотека TelemetryProto (проект telemetry) — в папке библиотек Arduino.
//
// ДВА ВАРИАНТА УСТРОЙСТВА — выбираются сами по плате в Arduino IDE:
//
// ===== А. XIAO ESP32S3 Sense — с камерой (основной, для полёта) =====
// Плата «XIAO_ESP32S3», PSRAM: «OPI PSRAM» (без неё камера не запустится).
//   LD2450:   5V -> 5V, GND -> GND, TX радара -> D7 (GPIO44), RX радара -> D6 (GPIO43)
//   MPU-6050: VCC -> 3V3, GND -> GND, SDA -> D4 (GPIO5), SCL -> D5 (GPIO6); компас — туда же
//   Светодиод — жёлтый на плате (GPIO21). Схема: wiring_xiao.svg
//
// ===== Б. Обычная ESP32-S3 (DevKitC и клоны) — без камеры =====
// Плата «ESP32S3 Dev Module». Тревога, направление и дистанция — без фотоснимка.
//   LD2450:   5V -> 5V, GND -> GND, TX радара -> GPIO13, RX радара -> GPIO14
//   MPU-6050: VCC -> 3V3, GND -> GND, SDA -> GPIO18, SCL -> GPIO17; компас — туда же
//   Плата-носитель под YD-ESP32-S3 / DevKitC-1: radar_nn/hardware/scout_carrier.
//   Светодиод — RGB на плате (GPIO48; у YD-ESP32-S3 замкнуть перемычку «RGB»). Схема: wiring_esp32s3.svg
//
// Обе платы: кнопка BOOT коротко — начать сканирование через START_DELAY_MS
// (время взлететь); удержание 2 с — калибровка компаса, а если его нет — гироскопа.
// Компас не обязателен: без него «Калибровка компаса невозможна» — это нормально.
// При включении 2 с модуль должен лежать неподвижно — калибровка гироскопа.
// Поддерживается и ESP32-CAM AI-Thinker: раскомментируйте VARIANT_ESP32CAM (wiring_esp32cam.svg).
//
// Монитор порта 115200, «Новая строка». Команды:
//   diag  — сводка: выводы, что на шине I2C, гироскоп, компас, радар, связь
//   findi2c — найти, на каких выводах на самом деле отвечают гироскоп и компас
//   findradar — нет данных радара: проверить выводы (не перепутаны ли TX/RX) и скорость
//   start — начать сканирование сейчас     done  — закончить сканирование
//   reset — сбросить карту, ждать начала    fps 10 — кадров видео в секунду
//   sep   — отметить «отделение от носителя» zone — отметить «вход в зону»
//   log   — весь журнал событий             map   — карта обнаружения
//   g — калибровка гироскопа (2 с неподвижно), gs — знак курса, gi — состояние гироскопа
//   bg / bgoff / rx / tracks — как в ld2450_search; i — команды компаса
//   nn    — нейросеть: какая модель, режим; nn gate / nn advise / nn off / nn 0.6 — порог
//   rec   — запись данных для обучения (строки R и F, их сохраняет tools/record_tracks.py)
//   Трек дрона (odom.h): home 55.751244 37.618423 90 — точка старта и куда смотрит дрон (азимут);
//   az 90 — только азимут старта; odo — где дрон, сколько пролетел и повернулся; odo reset — старт здесь;
//   move 1.5 [0.3] — дрон сдвинулся на 1,5 м вперёд [и 0,3 м вправо] (пока нет датчика перемещения)

// ---------- Вариант устройства ----------
// Выбирается по плате; чтобы задать вручную, раскомментируйте одну строку:
// #define VARIANT_XIAO_SENSE
// #define VARIANT_ESP32S3_NOCAM
// #define VARIANT_ESP32CAM

#include <Arduino.h>                       // здесь: нужны ARDUINO_* и CONFIG_IDF_TARGET_*
#if !defined(VARIANT_XIAO_SENSE) && !defined(VARIANT_ESP32S3_NOCAM) && !defined(VARIANT_ESP32CAM)
#if defined(ARDUINO_XIAO_ESP32S3)
#define VARIANT_XIAO_SENSE
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#define VARIANT_ESP32S3_NOCAM
#else
#error "Плата не ESP32-S3: выберите «XIAO_ESP32S3» или «ESP32S3 Dev Module» (или VARIANT_ESP32CAM вручную)"
#endif
#endif

#if defined(VARIANT_XIAO_SENSE)
#define VARIANT_NAME "XIAO ESP32S3 Sense, с камерой"
#define BOARD_XIAO_S3_SENSE                // выводы камеры (camera.h)
#define RADAR_RX   44                      // D7, сюда TX радара
#define RADAR_TX   43                      // D6, сюда RX радара
#define I2C_SDA    5                       // D4
#define I2C_SCL    6                       // D5
#define STATUS_LED 21                      // жёлтый светодиод, горит при LOW
#define STATUS_LED_ON LOW
#define FLASH_LED  -1
#elif defined(VARIANT_ESP32S3_NOCAM)
#define VARIANT_NAME "ESP32-S3, без камеры"
#define NO_CAMERA
#define RADAR_RX   13                      // выводы — как на плате-носителе (hardware/scout_carrier)
#define RADAR_TX   14
#define I2C_SDA    18
#define I2C_SCL    17
#define STATUS_LED RGB_BUILTIN             // RGB-светодиод платы (горит белым)
#define STATUS_LED_ON HIGH
#define FLASH_LED  -1
#elif defined(VARIANT_ESP32CAM)
#define VARIANT_NAME "ESP32-CAM AI-Thinker, с камерой"
#define BOARD_AI_THINKER
#define RADAR_RX   13                      // RX радара не подключён: GPIO12/2 мешают загрузке
#define RADAR_TX   -1
#define I2C_SDA    15
#define I2C_SCL    14
#define BUTTON_PIN -1                      // кнопки нет: старт через AUTO_START_MS, калибровка — команды g, c
#define STATUS_LED 33                      // красный светодиод, горит при LOW
#define STATUS_LED_ON LOW
#define FLASH_LED  4
#endif
#ifndef BUTTON_PIN
#define BUTTON_PIN 0                       // кнопка BOOT
#endif
#define CAL_BUTTON -1                      // кнопку обрабатывает скетч, а не nav_hw.h
#define NAV_NO_GPS                         // в помещении GPS не нужен
#define RADAR_BAUD 256000

#include <Arduino.h>
#include <Preferences.h>
#include <TelemetryLink.h>              // раньше mission.h: из библиотеки берётся telemetry.pb.h
#if !defined(TELEMETRY_LINK_VERSION) || TELEMETRY_LINK_VERSION < 5
#error "Старая библиотека TelemetryProto в папке библиотек Arduino. Распакуйте свежий telemetry.zip и замените её папкой telemetry/esp/libraries/TelemetryProto (лучше ссылкой, см. README telemetry)."
#endif
#include "mission.h"
#include "nav_hw.h"
#include "gyro.h"
#include "camera.h"
#include "stream.h"
#include "odom.h"

// ---------- Настройки ----------
// Основная сеть — ретранслятор на носителе (esp/relay). Запасная — Wi-Fi роутер напрямую,
// если разведчик до него достаёт: тогда данные идут прямо на сервер iot-go.
// Если 15 с ничего не уходит, разведчик переключается на другую сеть. Пустой WIFI_SSID — связь выключена.
const char *WIFI_SSID    = "scout-net";      // ретранслятор на Mavic
const char *WIFI_PASS    = "12345678";
const char *SERVER_HOST  = "192.168.5.1";
const uint16_t SERVER_PORT = 9000;
const char *WIFI_SSID2   = "my-router";      // роутер (2,4 ГГц); "" — без запасной сети
const char *WIFI_PASS2   = "router-password";
const char *SERVER_HOST2 = "bazzite.local";  // ноутбук с iot-go: IP в сети роутера или «имя.local»
const uint16_t SERVER_PORT2 = 8080;          // TCP-порт устройств iot-go — как SERVER_PORT в relay.ino
// Вместо роутера запасной может быть наземный ESP-шлюз по USB: "drone-net", "12345678", "192.168.4.1", 9000.
// Рукопожатие с сервером: при подключении Syn (MAC разведчика и адрес камеры), сервер отвечает Ack.
//   HS_AUTO   — нет Ack за 3 с — сервер старый (без Syn/Ack), снимки всё равно идут. Работает с обоими.
//   HS_STRICT — строго по протоколу iot-go: нет Ack — переподключение.   HS_OFF — старый протокол.
const uint8_t SERVER_HANDSHAKE = TelemetryLink::HS_AUTO;

const uint32_t START_DELAY_MS = 10000;  // после нажатия BOOT: время взлететь и занять точку
#if BUTTON_PIN < 0
const uint32_t AUTO_START_MS  = 30000;  // кнопки нет: старт через 30 с после включения (0 — только командой)
#else
const uint32_t AUTO_START_MS  = 0;      // >0 — начать сканирование само через столько мс после включения
#endif
const uint32_t TELEMETRY_MS = 200;

HardwareSerial Radar(1);
ld2450::Parser parser;
mission::Mission m;
TelemetryLink telem;

bool printTracks = false;
bool recMode = false;                    // запись данных для обучения нейросети (команда rec)
uint32_t recFrames = 0;
char synCamera[128] = "";                // адрес камеры, ушедший серверу в последнем Syn
uint32_t lastFrameMs = 0, startAtMs = 0;

void say(const char *s) { Serial.println(s); }

// ---------- Радар и курс ----------
void radarWrite(const uint8_t *b, int n) { Radar.write(b, n); Radar.flush(); }
void waitMs(int ms) { delay(ms); }
uint32_t radarBytes = 0;                 // сколько байт вообще пришло от радара
bool radarOk() { return parser.frames > 0 && millis() - lastFrameMs < 1000; }

odom::Odom od;                           // трек дрона: поворот по гироскопу, сдвиг — от источника перемещения

// Абсолютный курс, если ему можно верить: компас (откалиброван, нет помехи), иначе —
// азимут старта, заданный оператором (home / az), плюс поворот по гироскопу; иначе NAN
float absHeading() {
  const nav::State &s = nav::st;
  if (!isnan(s.heading) && s.calibrated && !s.magDisturbed) return s.heading;
  return isnan(gyro::value()) ? NAN : od.azimuth();
}

// Точка старта и азимут — в памяти ESP: задали один раз на месте, после перезагрузки те же
void loadHome() {
  Preferences p; p.begin("odom", true);
  od.homeSet = p.getBool("set", false);
  od.homeLat = p.getDouble("lat", 0); od.homeLon = p.getDouble("lon", 0);
  od.homeAz = p.getFloat("az", NAN);
  p.end();
}
void saveHome() {
  Preferences p; p.begin("odom", false);
  p.putBool("set", od.homeSet); p.putDouble("lat", od.homeLat); p.putDouble("lon", od.homeLon);
  p.putFloat("az", od.homeAz);
  p.end();
}

void printOdom() {
  double lat, lon;
  Serial.printf("Трек: от старта x %+.2f м (вправо), y %+.2f м (вперёд), до старта %.2f м; пройдено %.2f м\n",
                od.x, od.y, od.distanceFromStart(), od.path);
  Serial.printf("  поворот от старта %+.0f° (%.1f оборота), всего повёрнуто %.0f°; курс по гироскопу %s\n", od.yaw,
                od.yaw / 360.0f, od.turned, isnan(gyro::value()) ? "НЕТ (гироскоп не откалиброван)" : "есть");
  if (!isnan(od.homeAz)) Serial.printf("  азимут старта %.0f°, дрон сейчас смотрит на %.0f°\n", od.homeAz, od.azimuth());
  else Serial.println("  азимут старта не задан (az 90): трек верный по форме, но на карте повёрнут");
  if (od.latLon(lat, lon))
    Serial.printf("  на карте: %.7f, %.7f (уходит на сервер в position)\n  https://yandex.ru/maps/?pt=%.7f,%.7f&z=20\n", lat, lon, lon, lat);
  else Serial.println("  точка старта не задана (home ШИРОТА ДОЛГОТА АЗИМУТ) — координаты на сервер не уходят");
  Serial.println("  источник перемещения: нет датчика — только команда move (оптический поток — позже)");
  Serial.print("  последние точки (x;y м, поворот°):");
  for (int i = 0; i < od.trailN && i < 8; i++) {
    const odom::Point &q = od.trail[(od.trailPos - 1 - i + odom::Odom::TRAIL * 2) % odom::Odom::TRAIL];
    Serial.printf(" (%.1f;%.1f %+.0f°)", q.x, q.y, q.yaw);
  }
  Serial.println();
}

void odomCommand(String s) {
  s.trim();
  uint32_t now = millis();
  if (s == "odo") printOdom();
  else if (s == "odo reset") { od.reset(now); od.heading(gyro::value(), now); say("Трек начат заново: старт здесь"); }
  else if (s.startsWith("move")) {
    float f = 0, r = 0;
    if (sscanf(s.c_str() + 4, "%f %f", &f, &r) < 1) { say("move ВПЕРЁД [ВПРАВО] — метры, например move 1.5 или move 0 -0.5"); return; }
    od.move(f, r, now);
    Serial.printf("Сдвиг %.2f м вперёд, %.2f м вправо -> x %+.2f, y %+.2f, пройдено %.2f м\n", f, r, od.x, od.y, od.path);
  } else if (s.startsWith("az")) {
    float a;
    if (sscanf(s.c_str() + 2, "%f", &a) == 1) { od.homeAz = odom::wrap360(a); saveHome(); }
    Serial.printf("Азимут старта: %s\n", isnan(od.homeAz) ? "не задан" : String(od.homeAz, 0).c_str());
  } else if (s == "home off") { od.homeSet = false; od.homeAz = NAN; saveHome(); say("Точка старта забыта: координаты на сервер не уходят"); }
  else if (s.startsWith("home")) {
    double la, lo; float a;
    int n = sscanf(s.c_str() + 4, "%lf %lf %f", &la, &lo, &a);
    if (n >= 2 && la >= -90 && la <= 90 && lo >= -180 && lo <= 180) {
      od.homeSet = true; od.homeLat = la; od.homeLon = lo;
      if (n == 3) od.homeAz = odom::wrap360(a);
      saveHome();
      od.reset(now); od.heading(gyro::value(), now);
    } else if (n > 0) { say("home ШИРОТА ДОЛГОТА [АЗИМУТ], например home 55.751244 37.618423 90"); return; }
    if (od.homeSet) Serial.printf("Старт: %.7f, %.7f, азимут %s; трек начат отсюда\n", od.homeLat, od.homeLon,
                                  isnan(od.homeAz) ? "не задан" : String(od.homeAz, 0).c_str());
    else say("Точка старта не задана: home ШИРОТА ДОЛГОТА [АЗИМУТ]");
  }
}
// Курс для остановок: гироскоп; без него — компас (годится и неоткалиброванный:
// важно лишь, что курс меняется при повороте)
float scanHeading() { float g = gyro::value(); return isnan(g) ? nav::st.heading : g; }

void loadRadarSettings() { Preferences p; p.begin("radar", true); m.angleSign = p.getChar("asign", 1) < 0 ? -1 : 1; p.end(); }
void saveRadarSettings() { Preferences p; p.begin("radar", false); p.putChar("asign", m.angleSign); p.end(); }

const char *phaseName();
const char *nnModeName(trk::NnMode md);

// ---------- Состояние для страницы камеры (/status) ----------
size_t statusJson(char *b, size_t cap) {
  char head[16] = "null";
  float sh = scanHeading();
  if (!isnan(sh)) snprintf(head, sizeof head, "\"%.0f°\"", sh);
  int n = snprintf(b, cap, "{\"alarm\":%s,\"phase\":\"%s\",\"people\":%d,\"map\":%d,\"heading\":%s,\"radar\":%s,\"targets\":[",
                   m.alarmOn ? "true" : "false", phaseName(), m.tracker.personCount(), m.scanner.nDet, head,
                   radarOk() ? "true" : "false");
  bool first = true;
  for (auto &t : m.tracker.tracks) {
    if (!t.used || n >= (int)cap - 80) continue;
    n += snprintf(b + n, cap - n, "%s{\"d\":%.2f,\"a\":%.1f,\"confirmed\":%s}", first ? "" : ",", t.range(),
                  m.targetAngle(t), t.confirmed ? "true" : "false");
    first = false;
  }
  n += snprintf(b + n, cap - n, "]}");
  return n < (int)cap ? n : cap - 1;
}

// Светодиод для пилота: горит — остановка, держать; не горит — поворачивать;
// медленно мигает — ждёт начала; часто мигает — сканирование завершено, есть люди на карте
void updateLed(uint32_t now) {
  if (STATUS_LED < 0) return;
  bool on;
  switch (m.scanner.phase()) {
    case scan::SCANNING: on = m.scanner.dwelling(); break;
    case scan::WATCHING: on = m.scanner.nDet ? (now / 150) % 2 : true; break;
    default: on = (now / 700) % 2; break;
  }
  digitalWrite(STATUS_LED, on ? STATUS_LED_ON : !STATUS_LED_ON);
}

// ---------- Вывод и команды ----------
const char *phaseName() {
  switch (m.scanner.phase()) {
    case scan::STANDBY: return startAtMs ? "старт скоро" : "ожидание";
    case scan::SCANNING: return m.scanner.dwelling() ? "остановка" : "поворот";
    default: return "наблюдение";
  }
}

void printStatus() {
  char head[24] = "—";
  float sh = scanHeading();
  if (!isnan(sh)) snprintf(head, sizeof head, "%.0f° (%s)", sh, isnan(gyro::value()) ? "компас" : "гироскоп");
  if (!radarOk()) Serial.print("НЕТ ДАННЫХ РАДАРА | ");
  Serial.printf("[%s%s] остановок %lu, охват %.0f°, курс %s (отн. %.0f°), людей на карте %d, в обзоре %d",
                m.alarmOn ? "ТРЕВОГА, " : "", phaseName(), (unsigned long)m.scanner.stops(), m.scanner.coveredDeg(),
                head, m.scanner.relHeading(), m.scanner.nDet, m.tracker.personCount());
  if (startAtMs) Serial.printf(", старт через %ld с", (long)((int32_t)(startAtMs - millis()) / 1000 + 1));
  Serial.printf(", трек %+.1f;%+.1f м, пройдено %.1f м, поворот %+.0f°", od.x, od.y, od.path, od.yaw);
  if (!od.homeSet) Serial.print(" (home не задан: координат на сервере нет)");
  if (isnan(absHeading())) Serial.print(" (курса от севера нет: az)");
  if (!WIFI_SSID[0]) Serial.println(" | связь выкл");
  else if (telem.connected)
    Serial.printf(" | связь: %s, снимков %lu, кадров видео %lu, обрывов %lu\n", telem.network ? "запасная сеть" : "ретранслятор",
                  (unsigned long)telem.sentCount, (unsigned long)stream::frames, (unsigned long)telem.reconnects);
  else Serial.printf(" | связь: %s\n", telem.wifiUp ? "Wi-Fi есть, сервер не принимает" : "нет Wi-Fi");
  if (printTracks)
    for (auto &t : m.tracker.tracks)
      if (t.used)
        Serial.printf("    трек %lu: %.1f м, %+.0f°, видно %.0f%%, сеть %s, %s\n", (unsigned long)t.id, t.range(), m.targetAngle(t),
                      t.hitRatio() * 100, t.nnProb < 0 ? "—" : String((int)(t.nnProb * 100 + 0.5f)).c_str(),
                      t.confirmed ? "ЧЕЛОВЕК" : "проверка");
}

uint32_t quietUntil = 0;                 // после команды строка состояния молчит — ответ не уезжает

// Сводка для диагностики: всё нужное в нескольких строках
void printDiag() {
  Serial.println("\n===== diag =====");
  Serial.printf("Вариант: %s\n", VARIANT_NAME);
  Serial.printf("Выводы: радар TX -> GPIO%d, радар RX <- GPIO%d, I2C SDA GPIO%d, SCL GPIO%d\n",
                RADAR_RX, RADAR_TX, I2C_SDA, I2C_SCL);
  Serial.print("I2C:");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      const char *what = a == 0x68 || a == 0x69 ? "гироскоп MPU-6050" : a == 0x1E ? "компас HMC5883L" :
                         a == 0x0D ? "компас QMC5883L" : a == 0x2C ? "компас QMC5883P" : "?";
      Serial.printf(" 0x%02X (%s)", a, what);
      n++;
    }
  }
  Serial.println(n ? "" : " НИЧЕГО — команда findi2c найдёт, куда подключены модули");
  gyro::printState();
  Serial.printf("Компас: %s\n", nav::st.magChip == nav::MAG_NONE ? "нет (не обязателен)" :
                nav::st.calibrated ? "есть, откалиброван" : "есть, НЕ откалиброван (удержать BOOT 2 с или c)");
  Serial.printf("Радар: %s, байт %lu, кадров %lu, битых %lu\n", radarOk() ? "данные идут" :
                radarBytes ? "байты есть, кадров нет — команда findradar" : "НЕТ НИ ОДНОГО БАЙТА — команда findradar", (unsigned long)radarBytes,
                (unsigned long)parser.frames, (unsigned long)parser.badFrames);
  Serial.printf("Трек: старт %s, азимут %s, x %+.2f y %+.2f м, пройдено %.2f м, поворот %+.0f° (odo — подробно)\n",
                od.homeSet ? "задан" : "НЕ задан (home)", isnan(od.homeAz) ? "не задан (az)" : String(od.homeAz, 0).c_str(),
                od.x, od.y, od.path, od.yaw);
  Serial.printf("Нейросеть: %s, %s, порог %.2f\n", tnn::available() ? (tnn::synthetic() ? "обучена на синтетике" : "обучена на записях") : "нет",
                nnModeName(m.tracker.p.nnMode), m.tracker.p.nnThreshold);
  if (cam::ok) {
    IPAddress ip = WiFi.localIP();
    Serial.printf("Камера: есть, видео http://%s/ (через ретранслятор — список на http://scout-relay.local:8000/), "
                  "%s, кадров отправлено %lu, %d к/с\n", ip.toString().c_str(),
                  stream::watching ? "смотрят" : "никто не смотрит", (unsigned long)stream::frames, stream::fpsLimit);
  } else Serial.println("Камера: нет");
  if (!WIFI_SSID[0]) Serial.println("Связь: выключена");
  else {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    Serial.printf("Связь: Wi-Fi %s, сервер %s, сеть %s, снимков %lu\n", telem.wifiUp ? "есть" : "НЕТ",
                  telem.connected ? "подключён" : "НЕТ", telem.network ? WIFI_SSID2 : WIFI_SSID,
                  (unsigned long)telem.sentCount);
    Serial.printf("  рукопожатие: %s; MAC %02x:%02x:%02x:%02x:%02x:%02x (по нему сервер узнаёт разведчика)",
                  telem.handshakeText(), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    if (telem.hsResult == TelemetryLink::HSR_REJECTED) Serial.printf(", код отказа %ld", (long)telem.ackCode);
    Serial.printf("\n  камера в Syn: %s\n", synCamera[0] ? synCamera : "нет");
  }
  Serial.println("================ (строка состояния вернётся через 15 с)\n");
}

// Поиск, на каких выводах на самом деле висят I2C-модули (частые ошибки: старая схема 8/9 или
// 41/42, перепутаны SDA и SCL). Выводы радара, USB и BOOT не трогаем.
void findI2C() {
#if defined(VARIANT_ESP32S3_NOCAM)
  static const uint8_t pairs[][2] = {{18, 17}, {17, 18}, {8, 9}, {9, 8}, {41, 42}, {42, 41}, {5, 6}, {6, 5},
                                     {4, 5}, {15, 16}, {16, 15}, {1, 2}, {2, 1}, {38, 39}, {39, 38}, {47, 21}, {21, 47}};
#elif defined(VARIANT_XIAO_SENSE)                // только свободные выводы XIAO, камеру не трогаем
  static const uint8_t pairs[][2] = {{5, 6}, {6, 5}, {1, 2}, {2, 1}, {3, 4}, {4, 3}, {7, 8}, {8, 9}, {9, 8}};
#else
  static const uint8_t pairs[][2] = {{15, 14}, {14, 15}};
#endif
  Serial.println("\n===== поиск I2C на других выводах =====");
  int found = 0;
  for (auto &p : pairs) {
    if (p[0] == RADAR_RX || p[0] == RADAR_TX || p[1] == RADAR_RX || p[1] == RADAR_TX) continue;
    Wire.end();
    Wire.begin(p[0], p[1]);
    Wire.setClock(100000);
    String list;
    for (uint8_t a = 1; a < 127; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) { char b[8]; snprintf(b, sizeof b, " 0x%02X", a); list += b; }
    }
    if (list.length()) {
      found++;
      Serial.printf("SDA GPIO%d, SCL GPIO%d:%s%s\n", p[0], p[1], list.c_str(),
                    p[0] == I2C_SDA && p[1] == I2C_SCL ? "  <- как в прошивке" : "  <- НЕ как в прошивке");
    }
  }
  Wire.end();
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  if (!found) Serial.println("Ни на одной паре выводов I2C-модули не ответили: питание модулей (светодиод на GY-521 горит?), GND, пайка");
  Serial.printf("Прошивка ждёт SDA GPIO%d, SCL GPIO%d\n=====\n", I2C_SDA, I2C_SCL);
}

// ---------- Syn: оборудование на борту ----------

// Мы в сети ретранслятора? Он отвечает на http://<шлюз>:8000/ip строкой «<свой адрес в сети
// роутера> <номер разведчика>». Обычный роутер так не отвечает — тогда мы подключены напрямую.
bool relayInfo(String &relayIp, int &number) {
  WiFiClient c;
  if (!c.connect(WiFi.gatewayIP(), 8000, 400)) return false;
  c.print("GET /ip HTTP/1.0\r\n\r\n");
  String r;
  uint32_t t0 = millis();
  while ((c.connected() || c.available()) && millis() - t0 < 800 && r.length() < 512) {
    while (c.available()) r += (char)c.read();
    delay(2);
  }
  c.stop();
  int k = r.indexOf("\r\n\r\n");
  if (k < 0) return false;
  String body = r.substring(k + 4);
  body.trim();
  int sp = body.indexOf(' ');
  if (sp < 0) return false;
  IPAddress ip;
  relayIp = body.substring(0, sp);
  number = body.substring(sp + 1).toInt();
  return ip.fromString(relayIp) && number > 0;
}

// Вызывается задачей связи перед каждым подключением: адрес камеры всегда текущий.
// У каждого разведчика за ретранслятором свой номер — и своя ссылка на сервере.
void fillSyn(telemetry_v1_Syn &syn, uint8_t network) {
  (void)network;
  synCamera[0] = 0;
  if (!cam::ok) return;
  String relay;
  int n = 0;
  char stream[128], page[128];
  if (relayInfo(relay, n)) {                                   // через ретранслятор
    snprintf(stream, sizeof stream, "http://%s:8000/%d/stream", relay.c_str(), n);
    snprintf(page, sizeof page, "http://%s:8000/%d/", relay.c_str(), n);
  } else {                                                     // напрямую в сети роутера
    String me = WiFi.localIP().toString();
    snprintf(stream, sizeof stream, "http://%s:81/stream", me.c_str());
    snprintf(page, sizeof page, "http://%s/", me.c_str());
  }
  if (telemetry::synAddOnboard(syn, "camera", stream)) strcpy(synCamera, stream);
  telemetry::synAddOnboard(syn, "camera_page", page);
}

// ---------- Нейросеть треков ----------
const char *nnModeName(trk::NnMode md) {
  return md == trk::NN_GATE ? "решает вместе с правилами" : md == trk::NN_ADVISE ? "только оценка (решают правила)" : "выключена";
}

void nnCommand(String a) {
  a.trim();
  trk::Params &p = m.tracker.p;
  if (a == "gate") p.nnMode = trk::NN_GATE;
  else if (a == "advise") p.nnMode = trk::NN_ADVISE;
  else if (a == "off") p.nnMode = trk::NN_OFF;
  else if (a.length() && isdigit(a[0])) p.nnThreshold = constrain(a.toFloat(), 0.05f, 0.95f);
  if (!tnn::available() && p.nnMode != trk::NN_OFF) { p.nnMode = trk::NN_OFF; say("Модели нет — сеть выключена"); }
  Serial.printf("Нейросеть: %s\n  модель: %s\n  режим: %s, порог %.2f\n", tnn::available() ? "есть" : "нет", tnn::info(),
                nnModeName(p.nnMode), p.nnThreshold);
  if (tnn::available() && tnn::synthetic())
    say("  Модель обучена на синтетике: по умолчанию только оценивает. Запишите данные (rec, tools/record_tracks.py) и переобучите");
  say("  nn gate — сеть решает | nn advise — только оценка | nn off | nn 0.6 — порог");
}

// Строки для обучения: R — сырой кадр радара, F — признаки каждого трека (раз в 0,5 с)
void recordFrame(uint32_t ms) {
  Serial.printf("R,%lu", (unsigned long)ms);
  for (int i = 0; i < 3; i++) {
    const ld2450::Target &t = parser.t[i];
    if (t.valid) Serial.printf(",%.3f,%.3f,%.2f", t.x, t.y, t.speed);
    else Serial.print(",,,");
  }
  Serial.println();
  if (++recFrames % 5) return;
  for (auto &tr : m.tracker.tracks) {
    float f[tnn::NF];
    if (!m.tracker.trackFeatures(tr, f)) continue;
    Serial.printf("F,%lu,%lu,%.3f,%.1f,-", (unsigned long)ms, (unsigned long)tr.id, tr.range(), m.targetAngle(tr));
    for (int i = 0; i < tnn::NF; i++) Serial.printf(",%.4f", f[i]);
    Serial.println();
  }
}

// Поиск радара: те ли выводы (TX и RX не перепутаны?) и та ли скорость. Байты есть,
// а кадров нет — не та скорость; байтов нет ни на одной паре — питание 5 В, GND, провод TX радара.
void findRadar() {
  Serial.println("\n===== поиск радара =====");
  struct Opt { int rx, tx; uint32_t baud; };
  Opt opts[] = {{RADAR_RX, RADAR_TX, 256000}, {RADAR_TX, RADAR_RX, 256000},
                {RADAR_RX, RADAR_TX, 115200}, {RADAR_TX, RADAR_RX, 115200}, {RADAR_RX, RADAR_TX, 57600}};
  bool any = false;
  for (auto &o : opts) {
    if (o.rx < 0) continue;
    Radar.end();
    Radar.begin(o.baud, SERIAL_8N1, o.rx, o.tx);
    delay(50);
    while (Radar.available()) Radar.read();
    ld2450::Parser pr;
    uint32_t bytes = 0, t0 = millis();
    while (millis() - t0 < 1200) while (Radar.available()) { bytes++; pr.feed(Radar.read()); }
    Serial.printf("RX GPIO%d, TX GPIO%d, %lu бод: байт %lu, кадров %lu%s\n", o.rx, o.tx, (unsigned long)o.baud,
                  (unsigned long)bytes, (unsigned long)pr.frames,
                  pr.frames ? (o.rx == RADAR_RX && o.baud == RADAR_BAUD ? "  <- РАБОТАЕТ, как в прошивке" : "  <- РАБОТАЕТ, но НЕ как в прошивке") : "");
    any |= bytes > 0;
  }
  Radar.end();
  Radar.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX, RADAR_TX);
  if (!any) Serial.println("Ни одного байта: радару нужно 5 В (не 3,3 В), общий GND, провод TX радара — на RX платы");
  Serial.printf("Прошивка ждёт: TX радара -> GPIO%d, RX радара <- GPIO%d, %d бод\n=====\n", RADAR_RX, RADAR_TX, RADAR_BAUD);
}

void command(String s) {
  s.trim();
  uint32_t now = millis();
  quietUntil = now + 15000;
  if (s == "diag") printDiag();
  else if (s == "findi2c") findI2C();
  else if (s == "findradar") findRadar();
  else if (s == "start") { startAtMs = 0; m.start(now, scanHeading(), absHeading()); }
  else if (s == "done") m.finish(now);
  else if (s == "reset") { m.reset(now); startAtMs = 0; say("Карта сброшена, жду начала"); }
  else if (s.startsWith("fps")) { stream::fpsLimit = constrain((int)s.substring(3).toFloat(), 1, 30); Serial.printf("Видео: %d к/с\n", stream::fpsLimit); }
  else if (s == "sep") m.mark(now, mission::EV_SEPARATED);
  else if (s == "zone") m.mark(now, mission::EV_ZONE_ENTERED);
  else if (s == "log") { m.printLog(); m.printMap(); }
  else if (s == "map") m.printMap();
  else if (s == "bg") { m.tracker.startBackground(now); say("Запоминаю фон 20 с — в зоне радара никого"); }
  else if (s == "bgoff") { m.tracker.clearBackground(); say("Фон забыт"); }
  else if (s == "rx") { m.angleSign = -m.angleSign; saveRadarSettings(); Serial.printf("Знак угла: %+d\n", m.angleSign); }
  else if (s == "tracks") printTracks = !printTracks;
  else if (s.startsWith("odo") || s.startsWith("move") || s.startsWith("home") || s.startsWith("az")) odomCommand(s);
  else if (s == "rec" || s == "rec on" || s == "rec off") {
    recMode = s == "rec" ? !recMode : s == "rec on";
    recFrames = 0;
    if (recMode) Serial.println("# rec on: R,мс,x1,y1,v1,x2,y2,v2,x3,y3,v3 | F,мс,трек,дальность,угол,метка,12 признаков");
    else say("# rec off");
  }
  else if (s.startsWith("nn")) nnCommand(s.substring(2));
  else if (gyro::command(s)) {}
  else if (!nav::command(s))
    say("Команды: diag — сводка | findi2c | findradar | start | done | reset | fps10 | sep | zone | log | map | g | gs | gi | bg | bgoff | rx | tracks | nn | rec | odo | home | az | move | i — компас");
}

// Кнопка BOOT: коротко — старт через START_DELAY_MS; удержание 2 с — калибровка:
// компаса, если он подключён (30 с вращать), иначе гироскопа (2 с не двигать после отпускания)
void pollButton() {
  if (BUTTON_PIN < 0) return;
  static uint32_t downAt = 0;
  static bool longDone = false;
  bool down = digitalRead(BUTTON_PIN) == LOW;
  uint32_t now = millis();
  if (down && !downAt) { downAt = now; longDone = false; }
  if (down && downAt && !longDone && now - downAt >= 2000) {
    longDone = true;
    if (nav::st.magChip != nav::MAG_NONE) nav::calibrate();
    else say("Компаса нет — отпустите кнопку и 2 с не двигайте модуль: калибровка гироскопа");
  }
  if (!down && downAt) {
    if (longDone && nav::st.magChip == nav::MAG_NONE) {
      delay(300);                                   // рука ушла от кнопки
      say(gyro::calibrate() ? "Гироскоп откалиброван, курс = 0" : "Не вышло: модуль двигался, повторите");
    } else if (!longDone && now - downAt < 1000 && m.scanner.phase() != scan::SCANNING) {
      m.reset(now);
      startAtMs = now + START_DELAY_MS;
      Serial.printf("Сканирование начнётся через %lu с\n", (unsigned long)(START_DELAY_MS / 1000));
    }
    downAt = 0;
  }
}

// ---------- Основное ----------
void setup() {
  Serial.begin(115200);
  m.print = say;
  Radar.setRxBufferSize(2048);
  Radar.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX, RADAR_TX);
  delay(300);
  if (RADAR_TX >= 0) ld2450::setMultiTarget(radarWrite, waitMs);
  loadRadarSettings();
  if (STATUS_LED >= 0) { pinMode(STATUS_LED, OUTPUT); digitalWrite(STATUS_LED, !STATUS_LED_ON); }
  nav::begin();                                     // запускает и шину I2C (Wire) для гироскопа
  if (BUTTON_PIN >= 0) pinMode(BUTTON_PIN, INPUT_PULLUP);
  m.boot(millis());
  say("Разведчик: LD2450 + гироскоп, вариант: " VARIANT_NAME);
  Serial.printf("Выводы: радар TX -> GPIO%d, радар RX <- GPIO%d, I2C SDA GPIO%d, SCL GPIO%d\n",
                RADAR_RX, RADAR_TX, I2C_SDA, I2C_SCL);
  say("Калибровка гироскопа: 2 с не двигайте модуль...");
  gyro::begin();
  loadHome();
  od.reset(millis());
  if (!od.homeSet)
    say("ВНИМАНИЕ: точка старта не задана — position (координаты) на сервер не уходит. "
        "Один раз: home ШИРОТА ДОЛГОТА АЗИМУТ, например home 55.751244 37.618423 90");
  if (isnan(od.homeAz) && nav::st.magChip == nav::MAG_NONE)
    say("ВНИМАНИЕ: азимут старта не задан и компаса нет — heading_deg (курс от севера) на сервер не уходит. az 90 или home ... 90");
  gyro::printState();
  cam::flashPin = FLASH_LED;
  nav::printSettings();
  Serial.printf("Нейросеть треков: %s — %s\n", tnn::info(), nnModeName(m.tracker.p.nnMode));
  if (WIFI_SSID[0]) {
    telem.onSyn = fillSyn;
    telem.begin({WIFI_SSID, WIFI_PASS, SERVER_HOST, SERVER_PORT, WIFI_SSID2, WIFI_PASS2, SERVER_HOST2, SERVER_PORT2,
                 SERVER_HANDSHAKE});
    Serial.printf("Связь: «%s» -> %s:%u, запасная «%s» -> %s:%u\n", WIFI_SSID, SERVER_HOST, SERVER_PORT,
                  WIFI_SSID2, SERVER_HOST2, SERVER_PORT2);
  }
  // Камера и HTTP-сервер видео — после запуска Wi-Fi: без сетевого стека httpd падает
#if defined(NO_CAMERA)
  say("Камеры нет в этом варианте: тревога без видео");
#else
  if (!cam::begin()) say("Камера: НЕ НАЙДЕНА (шлейф, плата, PSRAM «OPI PSRAM») — видео не будет");
  else if (!WIFI_SSID[0]) say("Камера: есть, но Wi-Fi выключен — видео не будет");
  else {
    stream::statusFn = statusJson;
    say(stream::begin() ? "Камера: есть, видео — http://<адрес>/ (адрес — в diag)" : "Камера: есть, но HTTP-сервер не запустился");
  }
#endif
  if (AUTO_START_MS) {
    startAtMs = millis() + AUTO_START_MS;
    Serial.printf("Сканирование начнётся само через %lu с (start — сразу)\n", (unsigned long)(AUTO_START_MS / 1000));
  } else if (BUTTON_PIN >= 0) say("BOOT коротко — начать сканирование (start — сразу), удержание 2 с — калибровка");
  else say("Начать сканирование: команда start");
}

void loop() {
  uint32_t now = millis();
  nav::poll();
  gyro::poll();
  od.heading(gyro::value(), now);
  pollButton();
  if (startAtMs && (int32_t)(now - startAtMs) >= 0) { startAtMs = 0; m.start(now, scanHeading(), absHeading()); }

  m.tick(now, scanHeading(), telem.lastAlarmSentMs);

  while (Radar.available())
    if (radarBytes++, parser.feed(Radar.read())) {
      lastFrameMs = millis();
      m.radarFrame(parser.t, 3, lastFrameMs, absHeading());
      if (recMode) recordFrame(lastFrameMs);
    }
  if (!radarOk()) m.radarLost();

  updateLed(now);

  static uint32_t lastTelemetry = 0;
  if (WIFI_SSID[0] && now - lastTelemetry >= TELEMETRY_MS) {
    lastTelemetry = now;
    static telemetry_v1_Telemetry t;                   // ~1,3 КБ — не на стеке
    m.fill(t, now, radarOk(), absHeading());
    double lat, lon;
    if (od.latLon(lat, lon)) { t.has_position = true; t.position.lat = lat; t.position.lon = lon; }   // трек дрона
    t.rssi_dbm = WiFi.RSSI();
    telem.publish(t);
  }

  static String in;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (in.length()) { command(in); in = ""; } }
    else in += c;
  }

  static uint32_t lastPrint = 0;
  if (!recMode && now - lastPrint >= 1000 && (int32_t)(now - quietUntil) >= 0) { lastPrint = now; printStatus(); }
}
