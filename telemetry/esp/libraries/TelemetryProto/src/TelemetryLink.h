// Связь ESP32 с сервером: Wi-Fi + TCP + снимки Telemetry (protobuf, nanopb).
//
// Подключение (протокол iot-go, docs/device-protocol.md): кадры [4 байта длины BigEndian][protobuf].
//   1. Устройство -> Syn: MAC (6 байт, постоянный — по нему сервер узнаёт устройство после
//      переподключения) и список оборудования на борту (камера и её адрес) — onSyn.
//   2. Сервер -> Ack: code 0 — принят, иначе отказ (соединение закрывается, повтор через 10 с).
//   3. Устройство -> полные снимки telemetry.v1.Telemetry; сервер больше ничего не шлёт.
// Режимы рукопожатия (Config.handshake):
//   HS_AUTO   — Syn, ждём Ack до ACK_TIMEOUT_MS; Ack не пришёл, а соединение живо — сервер
//               старый (без Syn/Ack), снимки идут дальше в этом же подключении. По умолчанию:
//               прошивка работает и с текущим, и с новым сервером.
//   HS_STRICT — строго по документу: нет Ack — закрыть и переподключиться.
//   HS_OFF    — старый протокол: без Syn, снимки сразу.
// После подключения:
//   - важен только последний снимок: если сеть не успевает, старые снимки
//     заменяются новыми (очередь на 1 место), а не копятся;
//   - потерю связи устройство замечает по ошибке записи в TCP и переподключается;
//   - перед отправкой снимок приводится к ограничениям сервера (telemetry::sanitize):
//     сервер закрывает подключение на первом неверном значении (NaN, курс 360 и т.п.).
//
// Две сети: основная (например, ретранслятор на носителе) и запасная (роутер или наземный
// шлюз напрямую). Если за SWITCH_MS ни один снимок не ушёл, связь переключается на другую сеть.
//
// Вся сетевая работа идёт в отдельной задаче FreeRTOS на ядре 0, поэтому
// переподключения не тормозят loop() (радар, трекер).
//
//   TelemetryLink telem;
//   telem.onSyn = fillOnboards;  // по желанию: void fillOnboards(telemetry_v1_Syn &s, uint8_t network)
//   telem.begin({"scout-net", "12345678", "192.168.5.1", 9000,
//                "my-router", "password", "bazzite.local", 9000});   // запасная сеть — по желанию
//   Имена «компьютер.local» разрешаются через mDNS.
//   telemetry_v1_Telemetry t = telemetry_v1_Telemetry_init_zero;
//   ... заполнить ...
//   telem.publish(t);            // не блокирует; отправится последний снимок
#pragma once
// Версия библиотеки: скетчи проверяют её и просят обновить старую копию.
// 3 — запасная сеть со своим портом (Config.port2); 4 — схема telemetry.v1 поля 1–16, без фото;
// 5 — рукопожатие Syn/Ack (synack.proto), onSyn, Config.handshake.
#define TELEMETRY_LINK_VERSION 5
#include <Arduino.h>
#include <WiFi.h>
#include "telemetry_codec.h"

class TelemetryLink {
 public:
  enum Handshake : uint8_t { HS_AUTO = 0, HS_STRICT = 1, HS_OFF = 2 };
  // Чем кончилось последнее рукопожатие
  enum HsResult : uint8_t { HSR_NONE = 0, HSR_OK, HSR_NO_ACK, HSR_REJECTED, HSR_TIMEOUT, HSR_CLOSED, HSR_BAD, HSR_OFF };
  typedef void (*SynFn)(telemetry_v1_Syn &syn, uint8_t network);

  struct Config {
    const char *ssid;
    const char *password;
    const char *host;            // IP или имя сервера («имя.local» — через mDNS)
    uint16_t    port;
    const char *ssid2;           // запасная сеть; не указана (nullptr) или "" — нет
    const char *password2;
    const char *host2;
    uint16_t    port2;           // порт в запасной сети; 0 — как port
    uint8_t     handshake = HS_AUTO;
  };

  static const uint32_t SWITCH_MS = 15000;
  // Сколько ждать Ack. iot-go отвечает после регистрации устройства (до server_tcp.read_timeout,
  // по умолчанию 30 с), обычно сразу. HS_AUTO ждёт 3 с и дальше всё равно шлёт снимки (сервер
  // прочтёт их после Ack); HS_STRICT без Ack переподключается — ему ждать дольше.
  static const uint32_t ACK_TIMEOUT_MS = 3000;
  static const uint32_t ACK_TIMEOUT_STRICT_MS = 10000;

  // Заполнить оборудование в Syn (MAC уже стоит). Вызывается из задачи связи перед каждым
  // подключением, поэтому адрес камеры в нём всегда текущий.
  SynFn onSyn = nullptr;

  // Состояние для вывода (обновляется задачей связи)
  volatile bool     wifiUp = false;
  volatile bool     connected = false;
  volatile uint8_t  network = 0;          // 0 — основная сеть, 1 — запасная
  volatile uint32_t sentCount = 0;        // отправлено снимков
  volatile uint32_t replacedCount = 0;    // снимков заменено более новыми до отправки
  volatile uint32_t reconnects = 0;
  volatile uint32_t fixedCount = 0;       // снимков, в которых пришлось исправить значения
  volatile uint32_t lastAlarmSentMs = 0;  // millis(), когда ушёл снимок с alarm_active=true
  volatile uint8_t  hsResult = HSR_NONE;
  volatile int32_t  ackCode = 0;          // код последнего отказа
  volatile uint32_t rejects = 0;          // отказов сервера (Ack с code ≠ 0)

  const char *handshakeText() const {
    switch (hsResult) {
      case HSR_OK: return "Syn/Ack — принят";
      case HSR_NO_ACK: return "сервер не ответил Ack — старый протокол, снимки идут";
      case HSR_REJECTED: return "сервер ОТКАЗАЛ (Ack code ≠ 0)";
      case HSR_TIMEOUT: return "нет Ack за 10 с";
      case HSR_CLOSED: return "сервер закрыл подключение после Syn";
      case HSR_BAD: return "ответ сервера не разобран";
      case HSR_OFF: return "без рукопожатия (старый протокол)";
      default: return "ещё не было";
    }
  }

  void begin(const Config &cfg) {
    cfg_ = cfg;
    box_ = xQueueCreate(1, sizeof(telemetry_v1_Telemetry));
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    startNetwork(0);
    xTaskCreatePinnedToCore(taskEntry, "telemetry", 8192, this, 1, &task_, 0);
  }

  // Положить новый снимок (не блокирует). Неотправленный старый снимок заменяется.
  void publish(const telemetry_v1_Telemetry &t) {
    if (!box_) return;
    if (uxQueueMessagesWaiting(box_)) replacedCount = replacedCount + 1;
    xQueueOverwrite(box_, &t);
  }

 private:
  Config cfg_{};
  QueueHandle_t box_ = nullptr;
  TaskHandle_t task_ = nullptr;
  WiFiClient client_;
  telemetry_v1_Telemetry msg_;
  uint32_t nextConnect_ = 0, backoff_ = 1000, lastProgress_ = 0;

  static void taskEntry(void *self) { static_cast<TelemetryLink *>(self)->run(); }

  bool hasBackup() const { return cfg_.ssid2 && cfg_.ssid2[0]; }
  const char *host() const { return network ? cfg_.host2 : cfg_.host; }
  uint16_t port() const { return network && cfg_.port2 ? cfg_.port2 : cfg_.port; }

  // Syn -> Ack. true — можно слать снимки
  bool handshake() {
    if (cfg_.handshake == HS_OFF) { hsResult = HSR_OFF; return true; }
    static telemetry_v1_Syn syn;                // 640 байт — не на стеке задачи
    uint8_t mac[6];
    WiFi.macAddress(mac);
    telemetry::synInit(syn, mac);
    if (onSyn) onSyn(syn, network);
    static uint8_t buf[telemetry::MAX_SYN];
    size_t n = telemetry::encodeSyn(syn, buf, sizeof buf);
    if (!n || !writeAll(buf, n)) { hsResult = HSR_CLOSED; return false; }
    telemetry::FrameReader fr(64);
    uint32_t t0 = millis(), wait = cfg_.handshake == HS_STRICT ? ACK_TIMEOUT_STRICT_MS : ACK_TIMEOUT_MS;
    while (millis() - t0 < wait) {
      while (client_.available()) {
        telemetry::FrameReader::Result r = fr.feed((uint8_t)client_.read());
        if (r == telemetry::FrameReader::ERROR) { hsResult = HSR_BAD; return false; }
        if (r != telemetry::FrameReader::FRAME_READY) continue;
        telemetry_v1_Ack a;
        if (!fr.decodeAck(a)) { hsResult = HSR_BAD; return false; }
        if (a.code == 0) { hsResult = HSR_OK; return true; }
        ackCode = (int32_t)a.code;
        rejects = rejects + 1;
        hsResult = HSR_REJECTED;
        return false;
      }
      if (!client_.connected()) { hsResult = HSR_CLOSED; return false; }
      delay(10);
    }
    if (cfg_.handshake == HS_AUTO) { hsResult = HSR_NO_ACK; return true; }
    hsResult = HSR_TIMEOUT;
    return false;
  }

  void startNetwork(uint8_t n) {
    network = n;
    lastProgress_ = millis();
    WiFi.disconnect();
    if (n) WiFi.begin(cfg_.ssid2, cfg_.password2);
    else WiFi.begin(cfg_.ssid, cfg_.password);
  }

  void drop() {
    client_.stop();
    if (connected) reconnects = reconnects + 1;
    connected = false;
  }

  // write() может записать меньше, чем просили: дописываем остаток
  bool writeAll(const uint8_t *p, size_t n) {
    uint32_t start = millis();
    while (n) {
      size_t w = client_.write(p, n);
      if (w == 0) {
        if (!client_.connected() || millis() - start > 5000) return false;
        delay(5);
        continue;
      }
      p += w; n -= w;
      start = millis();
    }
    return true;
  }

  static bool sink(void *self, const uint8_t *d, size_t n) { return static_cast<TelemetryLink *>(self)->writeAll(d, n); }

  void send() {
    bool alarm = msg_.alarm_active;
    int fixed = 0;
    size_t n = telemetry::writeTelemetry(msg_, sink, this, &fixed);
    if (fixed) fixedCount = fixedCount + 1;
    if (n) {
      sentCount = sentCount + 1;
      lastProgress_ = millis();
      if (alarm) lastAlarmSentMs = millis();
    } else {
      drop();                       // недописанный кадр испортил бы поток: только переподключение
    }
  }

  void run() {
    for (;;) {
      wifiUp = WiFi.status() == WL_CONNECTED;
      if (connected && !client_.connected()) drop();

      // Долго ничего не уходит — пробуем другую сеть
      if (hasBackup() && millis() - lastProgress_ > SWITCH_MS) {
        drop();
        startNetwork(network ? 0 : 1);
        nextConnect_ = millis() + 1000;
        backoff_ = 1000;
      }

      if (!connected && wifiUp && (int32_t)(millis() - nextConnect_) >= 0) {
        bool ok = client_.connect(host(), port(), 2000);
        if (ok) {
          client_.setNoDelay(true);
          ok = handshake();
          if (!ok) client_.stop();
        }
        if (ok) {
          connected = true;
          backoff_ = 1000;
          xQueueReset(box_);              // старый снимок не отправляем: следующий будет свежим
        } else if (hsResult == HSR_REJECTED) {
          nextConnect_ = millis() + 10000;              // отказ — не долбим сервер
        } else {
          nextConnect_ = millis() + backoff_;
          backoff_ = backoff_ >= 5000 ? 10000 : backoff_ * 2;   // 1, 2, 4, 8, 10, 10... с
        }
      }

      // Ждём новый снимок до 50 мс; без связи снимок просто пропадает — следующий будет свежее
      if (xQueueReceive(box_, &msg_, pdMS_TO_TICKS(50)) == pdTRUE && connected) send();

      // После Ack сервер по контракту ничего не присылает; если что-то пришло — выбрасываем
      while (connected && client_.available()) client_.read();
    }
  }
};
