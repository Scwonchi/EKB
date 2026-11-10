// Шлюз: ESP32 раздаёт Wi-Fi устройствам с радаром и пересылает их данные на ПК по USB.
//
//   ESP с радаром --Wi-Fi/TCP 192.168.4.1:9000--> ШЛЮЗ --USB--> server.py --serial
//
// Устройства подключаются к точке доступа SSID/PASS и к серверу 192.168.4.1:9000
// (в скетчах устройств: SERVER_HOST = "192.168.4.1"). Шлюз protobuf не разбирает —
// просто пересылает байты каждого клиента; сообщения собирает сервер на ПК.
//
// Плата: любая ESP32/ESP32-S3. Порт ПК: 921600 бод (для встроенного USB скорость не важна).
// В порт шлюз ничего, кроме кадров протокола, не пишет — монитор порта Arduino покажет «мусор»,
// это нормально. Светодиод LED_PIN (если задан) мигает при пересылке данных.

#include <WiFi.h>
#include "gw_link.h"

// ---------- Настройки ----------
const char *AP_SSID = "drone-net";
const char *AP_PASS = "12345678";        // не короче 8 символов
const int   AP_CHANNEL = 6;              // 1, 6 или 11; если пульт Mavic на 2,4 ГГц — переведите его на 5,8 ГГц
const uint16_t PORT = 9000;
const uint32_t USB_BAUD = 921600;
const int MAX_CLIENTS = 6;               // до 10 у точки доступа ESP32
const int LED_PIN = -1;                  // вывод светодиода для индикации (-1 — не использовать)

WiFiServer server(PORT);

struct Slot {
  bool used = false;
  uint8_t id = 0;
  WiFiClient c;
};
Slot slots[MAX_CLIENTS];
uint8_t nextId = 1;

gw::Parser fromPc;
uint8_t out[gw::MAX_PAYLOAD + 8];
uint8_t chunk[512];
uint32_t lastBlink = 0;

void sendPc(uint8_t type, uint8_t client, const uint8_t *data, uint16_t len) {
  size_t n = gw::encode(type, client, data, len, out);
  Serial.write(out, n);
}

void blink() {
  if (LED_PIN < 0) return;
  digitalWrite(LED_PIN, HIGH);
  lastBlink = millis();
}

Slot *findSlot(uint8_t id) {
  for (auto &s : slots) if (s.used && s.id == id) return &s;
  return nullptr;
}

void closeSlot(Slot &s, bool notify) {
  s.c.stop();
  s.used = false;
  if (notify) sendPc(gw::DISCONNECT, s.id, nullptr, 0);
}

void acceptClients() {
  WiFiClient c = server.accept();
  if (!c) return;
  for (auto &s : slots) {
    if (s.used) continue;
    s.used = true;
    s.id = nextId++;
    if (nextId == 0) nextId = 1;            // id 0 не используем
    s.c = c;
    s.c.setNoDelay(true);
    IPAddress ip = c.remoteIP();
    uint8_t a[4] = {ip[0], ip[1], ip[2], ip[3]};
    sendPc(gw::CONNECT, s.id, a, 4);
    return;
  }
  c.stop();                                 // мест нет
}

void pumpClients() {
  for (auto &s : slots) {
    if (!s.used) continue;
    int n;
    while ((n = s.c.available()) > 0) {
      n = s.c.read(chunk, n > (int)sizeof(chunk) ? sizeof(chunk) : n);
      if (n <= 0) break;
      sendPc(gw::DATA, s.id, chunk, n);
      blink();
    }
    if (!s.c.connected()) closeSlot(s, true);
  }
}

void pumpPc() {
  static uint32_t lastByte = 0;
  if (!fromPc.idle() && millis() - lastByte > 100) fromPc.reset();   // обрывок кадра — выбросить
  while (Serial.available()) {
    lastByte = millis();
    if (!fromPc.feed(Serial.read())) continue;
    switch (fromPc.type) {
      case gw::DATA: {
        Slot *s = findSlot(fromPc.client);
        if (s) { s->c.write(fromPc.data, fromPc.len); blink(); }
        else sendPc(gw::DISCONNECT, fromPc.client, nullptr, 0);   // сервер пишет закрытому клиенту
        break;
      }
      case gw::DISCONNECT: {
        Slot *s = findSlot(fromPc.client);
        if (s) closeSlot(*s, false);
        break;
      }
      case gw::RESET:                       // сервер перезапущен: устройства переподключатся
        for (auto &s : slots) if (s.used) closeSlot(s, false);
        break;
      default: break;
    }
  }
}

void setup() {
  Serial.setRxBufferSize(4096);
  Serial.begin(USB_BAUD);
  if (LED_PIN >= 0) pinMode(LED_PIN, OUTPUT);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, 10);
  WiFi.setSleep(false);                     // без энергосбережения — меньше задержка
  server.begin();
  server.setNoDelay(true);
}

void loop() {
  acceptClients();
  pumpClients();
  pumpPc();

  static uint32_t lastStatus = 0;
  uint32_t now = millis();
  if (now - lastStatus >= 2000) {
    lastStatus = now;
    uint8_t st[6];
    uint8_t n = 0;
    for (auto &s : slots) if (s.used) n++;
    uint32_t up = now / 1000;
    st[0] = n; st[1] = up; st[2] = up >> 8; st[3] = up >> 16; st[4] = up >> 24;
    st[5] = WiFi.softAPgetStationNum();     // сколько устройств подключено к Wi-Fi (даже без TCP)
    sendPc(gw::STATUS, 0, st, 6);
  }
  if (LED_PIN >= 0 && lastBlink && now - lastBlink > 30) { digitalWrite(LED_PIN, LOW); lastBlink = 0; }
  delay(1);
}
