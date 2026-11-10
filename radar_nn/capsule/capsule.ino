// Капсула с радаром LD2410C: её сбрасывают с носителя (Mavic), она лежит на земле и слушает,
// есть ли рядом человек — в том числе неподвижный или без сознания (LD2410C видит дыхание).
// Данные идут на сервер iot-go через ретранслятор на носителе, как у разведчика.
//
// Плата: ESP32-S3 DevKitC / YD-ESP32-S3, в Arduino IDE — «ESP32S3 Dev Module».
// Рядом со скетчем: ld2410.h, capsule_core.h, capsule_nn.h, model.h (сеть; пишет train.py).
// Нужна библиотека TelemetryProto (проект telemetry, esp/libraries) версии 5 и новее.
//
// Подключение (как радар у разведчика на ESP32-S3 — та же плата-носитель подходит):
//   LD2410C VCC -> 5V, GND -> GND, TX -> GPIO13, RX -> GPIO14  (OUT не нужен)
//   Питание: LiPo 1S + повышающий модуль 5 В на вывод 5V, или пауэрбанк в USB.
//
// Как решает «человек есть» — capsule_core.h: правила по энергиям зон радара (с фоном)
// и нейросеть по окну 10 с, которая отличает человека от куста на ветру.
//
// Монитор порта 115200, «Новая строка». Команды:
//   diag   — сводка: радар, фон, сеть, связь          drop — «капсула упала»: 8 с не решать
//   bg     — запомнить фон 20 с (рядом никого)         bgoff — забыть фон
//   nn     — нейросеть: модель и режим; nn gate / nn advise / nn off / nn 0.6 — порог
//   rec    — запись для обучения: строки m0..m8,s0..s8 (их сохраняет record.py)
//   findradar — нет данных радара: проверить выводы и скорость
//
// Светодиод на плате: синий мигает — успокоение после сброса; голубой — запоминает фон;
// зелёный вспыхивает — всё в порядке; жёлтый — нет связи; красный часто — ТРЕВОГА.

#include <Arduino.h>
#include <Preferences.h>
#include <TelemetryLink.h>
#if !defined(TELEMETRY_LINK_VERSION) || TELEMETRY_LINK_VERSION < 5
#error "Старая библиотека TelemetryProto в папке библиотек Arduino. Замените её папкой telemetry/esp/libraries/TelemetryProto из свежего telemetry.zip (лучше ссылкой, см. README telemetry)."
#endif
#include "capsule_core.h"

// ---------- Выводы ----------
#define RADAR_RX   13                      // сюда TX радара
#define RADAR_TX   14                      // сюда RX радара
#define RADAR_BAUD 256000
#ifdef RGB_BUILTIN
#define STATUS_RGB RGB_BUILTIN
#else
#define STATUS_RGB -1
#endif

// ---------- Связь ----------
// Как у разведчика: основная сеть — ретранслятор на носителе, запасная — роутер напрямую.
// Пустой WIFI_SSID — связь выключена.
const char *WIFI_SSID    = "scout-net";
const char *WIFI_PASS    = "12345678";
const char *SERVER_HOST  = "192.168.5.1";
const uint16_t SERVER_PORT = 9000;
const char *WIFI_SSID2   = "my-router";      // "" — без запасной сети
const char *WIFI_PASS2   = "router-password";
const char *SERVER_HOST2 = "bazzite.local";
const uint16_t SERVER_PORT2 = 8080;          // TCP-порт устройств iot-go
// Рукопожатие: HS_AUTO — работает и с iot-go без Syn/Ack; HS_STRICT — строго; HS_OFF — старый протокол
const uint8_t SERVER_HANDSHAKE = TelemetryLink::HS_AUTO;
const uint32_t TELEMETRY_MS = 200;
const uint32_t FIRMWARE_VERSION = 1;

HardwareSerial Radar(1);
ld2410::Parser parser;
capsule::Capsule cap;
TelemetryLink telem;

uint32_t lastFrameMs = 0, radarBytes = 0, quietUntil = 0;
bool recMode = false;

void say(const char *s) { Serial.println(s); }
void radarWrite(const uint8_t *b, int n) { Radar.write(b, n); Radar.flush(); }
void waitMs(int ms) { delay(ms); }
bool radarOk() { return parser.frames > 0 && millis() - lastFrameMs < 1000 && parser.f.engineering; }

// ---------- Фон в памяти ESP: капсулу настроили на месте — после перезагрузки фон тот же ----------
void saveBackground() {
  Preferences p; p.begin("capsule", false);
  p.putBytes("bgmv", cap.bgMoving(), ld2410::GATES);
  p.putBytes("bgst", cap.bgStatic(), ld2410::GATES);
  p.end();
}
void loadBackground() {
  Preferences p; p.begin("capsule", true);
  uint8_t mv[ld2410::GATES], st[ld2410::GATES];
  if (p.getBytes("bgmv", mv, sizeof mv) == sizeof mv && p.getBytes("bgst", st, sizeof st) == sizeof st)
    cap.setBackground(mv, st);
  p.end();
}
void forgetBackground() { Preferences p; p.begin("capsule", false); p.clear(); p.end(); }

// ---------- Вывод ----------
const char *nnModeName(capsule::NnMode m) {
  return m == capsule::NN_GATE ? "решает вместе с правилами" : m == capsule::NN_ADVISE ? "только оценка (решают правила)" : "выключена";
}

void printStatus() {
  uint32_t now = millis();
  if (!radarOk()) Serial.print(parser.frames && !parser.f.engineering ? "РАДАР НЕ В ИНЖЕНЕРНОМ РЕЖИМЕ | " : "НЕТ ДАННЫХ РАДАРА | ");
  if (cap.settling(now)) Serial.printf("[успокоение, %lu с] ", (unsigned long)(cap.settleLeftMs(now) / 1000 + 1));
  else if (cap.learning()) Serial.printf("[фон, %lu с] ", (unsigned long)(cap.learnLeftMs(now) / 1000 + 1));
  else Serial.printf("[%s] ", cap.alarmOn ? "ТРЕВОГА" : cap.rulesPresent ? "цель, сеть против" : "норма");
  if (cap.ratio > 0) Serial.printf("цель %.1f м, %s, видно %.0f%%", cap.distanceM, cap.moving ? "движется" : "неподвижна", cap.ratio * 100);
  else Serial.print("целей нет");
  if (cap.nnP >= 0) Serial.printf(" | сеть: человек %.0f%% (%s)", cap.nnP * 100, cap.nnLabel());
  if (!WIFI_SSID[0]) Serial.println(" | связь выкл");
  else if (telem.connected) Serial.printf(" | связь: %s, снимков %lu\n", telem.network ? "запасная сеть" : "ретранслятор", (unsigned long)telem.sentCount);
  else Serial.printf(" | связь: %s\n", telem.wifiUp ? "Wi-Fi есть, сервер не принимает" : "нет Wi-Fi");
}

void printDiag() {
  Serial.println("\n===== diag =====");
  Serial.printf("Капсула LD2410C, прошивка %lu\n", (unsigned long)FIRMWARE_VERSION);
  Serial.printf("Радар: TX радара -> GPIO%d, RX радара <- GPIO%d; %s, байт %lu, кадров %lu (обычных %lu), битых %lu\n",
                RADAR_RX, RADAR_TX,
                radarOk() ? "данные идут" : parser.frames ? "кадры есть, но не инженерные (команда eng)" :
                radarBytes ? "байты есть, кадров нет — findradar" : "НИ ОДНОГО БАЙТА — findradar",
                (unsigned long)radarBytes, (unsigned long)parser.frames, (unsigned long)parser.basicFrames,
                (unsigned long)parser.badFrames);
  const ld2410::Frame &f = cap.lastFrame();
  Serial.print("Энергии движения по зонам 0..8:     ");
  for (int g = 0; g < 9; g++) Serial.printf("%4u", f.mv[g]);
  Serial.print("\nЭнергии неподвижности по зонам 0..8: ");
  for (int g = 0; g < 9; g++) Serial.printf("%4u", f.st[g]);
  Serial.println("   (зона = 0,75 м)");
  if (cap.backgroundReady()) {
    Serial.print("Фон движения:                       ");
    for (int g = 0; g < 9; g++) Serial.printf("%4u", cap.bgMoving()[g]);
    Serial.print("\nФон неподвижности:                  ");
    for (int g = 0; g < 9; g++) Serial.printf("%4u", cap.bgStatic()[g]);
    Serial.println();
  } else Serial.println("Фон: не запомнен (команда bg, 20 с рядом никого) — решают состояние радара и сеть");
  Serial.printf("Нейросеть: %s\n  режим: %s, порог %.2f, запусков %lu\n", capsule::modelInfo(), nnModeName(cap.p.nnMode),
                cap.p.nnThreshold, (unsigned long)cap.nnRuns);
  if (!WIFI_SSID[0]) Serial.println("Связь: выключена");
  else {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    Serial.printf("Связь: Wi-Fi %s, сервер %s, сеть %s, снимков %lu\n  рукопожатие: %s; MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
                  telem.wifiUp ? "есть" : "НЕТ", telem.connected ? "подключён" : "НЕТ", telem.network ? WIFI_SSID2 : WIFI_SSID,
                  (unsigned long)telem.sentCount, telem.handshakeText(), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }
  Serial.printf("Тревог с включения: %lu\n================ (строка состояния вернётся через 15 с)\n\n", (unsigned long)cap.alarms);
}

// Те ли выводы и скорость: байты есть, а кадров нет — не та скорость; байтов нет — питание, провод TX
void findRadar() {
  Serial.println("\n===== поиск радара =====");
  struct Opt { int rx, tx; uint32_t baud; };
  Opt opts[] = {{RADAR_RX, RADAR_TX, 256000}, {RADAR_TX, RADAR_RX, 256000}, {RADAR_RX, RADAR_TX, 115200},
                {RADAR_TX, RADAR_RX, 115200}, {RADAR_RX, RADAR_TX, 57600}};
  bool any = false;
  for (auto &o : opts) {
    Radar.end();
    Radar.begin(o.baud, SERIAL_8N1, o.rx, o.tx);
    delay(50);
    while (Radar.available()) Radar.read();
    ld2410::Parser pr;
    uint32_t bytes = 0, t0 = millis();
    while (millis() - t0 < 1200) while (Radar.available()) { bytes++; pr.feed(Radar.read()); }
    Serial.printf("RX GPIO%d, TX GPIO%d, %lu бод: байт %lu, кадров %lu%s\n", o.rx, o.tx, (unsigned long)o.baud,
                  (unsigned long)bytes, (unsigned long)pr.frames,
                  pr.frames ? (o.rx == RADAR_RX && o.baud == RADAR_BAUD ? "  <- РАБОТАЕТ, как в прошивке" : "  <- РАБОТАЕТ, но НЕ как в прошивке") : "");
    any |= bytes > 0;
  }
  Radar.end();
  Radar.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX, RADAR_TX);
  ld2410::enableEngineering(radarWrite, waitMs);
  if (!any) say("Ни одного байта: радару нужно 5 В, общий GND, провод TX радара — на RX платы");
  Serial.println("=====");
}

void nnCommand(String a) {
  a.trim();
  capsule::Params &p = cap.p;
  if (a == "gate") p.nnMode = capsule::NN_GATE;
  else if (a == "advise") p.nnMode = capsule::NN_ADVISE;
  else if (a == "off") p.nnMode = capsule::NN_OFF;
  else if (a.length() && isdigit(a[0])) p.nnThreshold = constrain(a.toFloat(), 0.05f, 0.95f);
  if (!capsule::haveModel() && p.nnMode != capsule::NN_OFF) { p.nnMode = capsule::NN_OFF; say("Модели нет — сеть выключена"); }
  Serial.printf("Нейросеть: %s\n  режим: %s, порог %.2f\n", capsule::modelInfo(), nnModeName(p.nnMode), p.nnThreshold);
  if (capsule::haveModel() && capsule::modelSynthetic())
    say("  Модель обучена на синтетике: по умолчанию только оценивает. Запишите данные (rec, record.py) и переобучите");
  say("  nn gate — сеть решает | nn advise — только оценка | nn off | nn 0.6 — порог");
}

void command(String s) {
  s.trim();
  uint32_t now = millis();
  quietUntil = now + 15000;
  if (s == "diag") printDiag();
  else if (s == "findradar") findRadar();
  else if (s == "drop") { cap.drop(now); say("Капсула «упала»: 8 с не решаю"); }
  else if (s == "bg") { cap.startBackground(now); say("Запоминаю фон 20 с — рядом с капсулой никого"); }
  else if (s == "bgoff") { cap.clearBackground(); forgetBackground(); say("Фон забыт"); }
  else if (s == "eng") { ld2410::enableEngineering(radarWrite, waitMs); say("Радару отправлено «инженерный режим»"); }
  else if (s.startsWith("nn")) nnCommand(s.substring(2));
  else if (s == "rec" || s == "rec on" || s == "rec off") {
    recMode = s == "rec" ? !recMode : s == "rec on";
    if (!recMode) say("# rec off");
  }
  else say("Команды: diag | drop | bg | bgoff | nn | rec | findradar | eng");
}

// ---------- Светодиод ----------
void updateLed(uint32_t now) {
#if STATUS_RGB >= 0
  static uint32_t last = 0;
  if (now - last < 50) return;
  last = now;
  uint8_t r = 0, g = 0, b = 0;
  if (cap.alarmOn) { if ((now / 150) % 2) r = 40; }
  else if (cap.settling(now)) { if ((now / 400) % 2) b = 30; }
  else if (cap.learning()) { g = 15; b = 25; }
  else if (WIFI_SSID[0] && !telem.connected) { if ((now / 1000) % 2) { r = 25; g = 15; } }
  else if (now % 2000 < 100) g = 25;                  // короткая вспышка — экономим батарею
  rgbLedWrite(STATUS_RGB, r, g, b);
#endif
}

// ---------- Основное ----------
void setup() {
  Serial.begin(115200);
  Radar.setRxBufferSize(1024);
  Radar.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX, RADAR_TX);
  delay(300);
  ld2410::enableEngineering(radarWrite, waitMs);
  cap.firmwareVersion = FIRMWARE_VERSION;
  loadBackground();
  cap.boot(millis());
  Serial.printf("\nКапсула LD2410C. Радар: TX -> GPIO%d, RX <- GPIO%d\n", RADAR_RX, RADAR_TX);
  Serial.printf("Фон: %s\n", cap.backgroundReady() ? "из памяти" : "не запомнен (команда bg)");
  Serial.printf("Нейросеть: %s — %s\n", capsule::modelInfo(), nnModeName(cap.p.nnMode));
  if (WIFI_SSID[0]) {
    telem.begin({WIFI_SSID, WIFI_PASS, SERVER_HOST, SERVER_PORT, WIFI_SSID2, WIFI_PASS2, SERVER_HOST2, SERVER_PORT2,
                 SERVER_HANDSHAKE});
    Serial.printf("Связь: «%s» -> %s:%u, запасная «%s» -> %s:%u\n", WIFI_SSID, SERVER_HOST, SERVER_PORT,
                  WIFI_SSID2, SERVER_HOST2, SERVER_PORT2);
  }
  say("Команды: diag | drop | bg | bgoff | nn | rec | findradar | eng");
}

void loop() {
  uint32_t now = millis();
  while (Radar.available()) {
    radarBytes++;
    if (!parser.feed(Radar.read())) continue;
    lastFrameMs = millis();
    if (!parser.f.engineering) continue;              // обычный кадр без энергий — ждём инженерный
    if (cap.frame(parser.f, lastFrameMs)) { saveBackground(); say("Фон запомнен и сохранён"); }
    if (recMode && cap.sampleReady()) { char b[96]; cap.csvLine(b, sizeof b); Serial.println(b); }
  }
  if (!radarOk()) cap.radarLost();

  // Радар шлёт обычные кадры (после сброса питания) — снова включить инженерный режим
  static uint32_t lastEng = 0;
  if (parser.frames && !parser.f.engineering && now - lastEng > 3000) { lastEng = now; ld2410::enableEngineering(radarWrite, waitMs); }

  static uint32_t lastTelemetry = 0;
  if (WIFI_SSID[0] && now - lastTelemetry >= TELEMETRY_MS) {
    lastTelemetry = now;
    static telemetry_v1_Telemetry t;
    cap.fill(t, now, radarOk());
    t.rssi_dbm = WiFi.RSSI();
    telem.publish(t);
  }

  updateLed(now);

  static String in;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (in.length()) { command(in); in = ""; } }
    else in += c;
  }

  static uint32_t lastPrint = 0;
  if (!recMode && now - lastPrint >= 1000 && (int32_t)(now - quietUntil) >= 0) { lastPrint = now; printStatus(); }
}
