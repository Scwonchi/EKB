// Демонстрация связи ESP32 → сервер без радара и датчиков: снимки telemetry.v1.Telemetry.
// Нужна библиотека TelemetryProto (папка esp/libraries/TelemetryProto, см. README).
//
// Что делает:
//   - подключается к Wi-Fi и серверу, при обрыве переподключается сама;
//   - 5 раз в секунду отправляет снимок Telemetry с условными данными;
//   - нажатие BOOT включает условную тревогу на 10 с: в снимках появляется
//     подтверждённая цель и alarm_active = true, сервер сам создаёт и снимает тревогу.
// Монитор порта 115200.

#include <TelemetryLink.h>

// ---------- Настройки ----------
const char *WIFI_SSID   = "drone-net";
const char *WIFI_PASS   = "12345678";
const char *SERVER_HOST = "192.168.4.1";     // ESP-шлюз; при сервере прямо на ПК — IP ПК
const uint16_t SERVER_PORT = 9000;
const uint32_t FIRMWARE_VERSION = 3;
const int BUTTON = 0;                        // BOOT

TelemetryLink telem;
uint32_t alarmUntil = 0;

void publish() {
  bool alarm = millis() < alarmUntil;
  telemetry_v1_Telemetry t = telemetry_v1_Telemetry_init_zero;
  t.firmware_version = FIRMWARE_VERSION;
  t.uptime_s = millis() / 1000;
  t.radar_ok = false;                                        // в демо радара нет
  t.detection_class = alarm ? telemetry_v1_DetectionClass_DETECTION_CLASS_STATIONARY
                            : telemetry_v1_DetectionClass_DETECTION_CLASS_EMPTY;
  t.person_probability = alarm ? 0.9f : 0.0f;
  t.rssi_dbm = WiFi.RSSI();                                  // has_position = false: GPS нет
  if (alarm) {                                               // условный человек в 3 м, 20° правее оси
    telemetry_v1_Track &o = t.tracks[t.tracks_count++];
    o.id = 1; o.distance_m = 3.0f; o.angle_deg = 20.0f; o.speed_mps = 0.02f;
    o.confirmed = true; o.hit_ratio = 0.9f;
    o.detection_class = telemetry_v1_DetectionClass_DETECTION_CLASS_STATIONARY;
    t.target_count = 1;
  }
  t.alarm_active = alarm;
  telem.publish(t);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(BUTTON, INPUT_PULLUP);
  telem.begin({WIFI_SSID, WIFI_PASS, SERVER_HOST, SERVER_PORT});
  Serial.printf("Сервер %s:%u; BOOT — условная тревога на 10 с\n", SERVER_HOST, SERVER_PORT);
}

void loop() {
  uint32_t now = millis();

  static bool prevBtn = true;
  bool btn = digitalRead(BUTTON);
  if (prevBtn && !btn) { alarmUntil = now + 10000; Serial.println("Условная тревога на 10 с"); }
  prevBtn = btn;

  static uint32_t lastPub = 0;
  if (now - lastPub >= 200) { lastPub = now; publish(); }

  static uint32_t lastLog = 0;
  if (now - lastLog >= 5000) {
    lastLog = now;
    Serial.printf("Wi-Fi %s | сервер %s | отправлено снимков %u, заменено до отправки %u, переподключений %u\n",
                  telem.wifiUp ? "есть" : "нет", telem.connected ? "подключён" : "нет",
                  (unsigned)telem.sentCount, (unsigned)telem.replacedCount, (unsigned)telem.reconnects);
  }
  delay(5);
}
