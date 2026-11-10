// Ретранслятор на носителе (Mavic): ESP32 раздаёт Wi-Fi разведчикам и капсулам и пересылает их
// TCP-подключения через Wi-Fi роутер на ноутбук оператора. Телеметрия всех устройств одинаковая
// (Syn/Ack, Telemetry); камеры — только у разведчиков, номера видео получают только они.
//
//   разведчик --Wi-Fi «scout-net»--> РЕТРАНСЛЯТОР 192.168.5.1:9000 --Wi-Fi роутера--> сервер iot-go
//
// Данные идут прямо в TCP-порт устройств сервера (SERVER_PORT). Порт смотрите в конфигурации
// iot-go: configs/config.yaml, раздел server_tcp (в репозитории по умолчанию 9000,
// а 8080 — server_http, веб-API).
//
// Каждое подключение разведчика ретранслятор превращает в своё подключение к серверу и
// пересылает байты в обе стороны, protobuf не разбирает: Syn разведчика уходит серверу,
// Ack сервера — обратно разведчику. Сервер узнаёт разведчика по MAC из Syn, а не по адресу
// (все разведчики для него приходят с адреса ретранслятора).
//
// Видео с нескольких разведчиков. Каждый разведчик — IP-камера (http://<разведчик>/ и
// :81/stream) в сети «scout-net», куда ноутбук напрямую не попадает. Ретранслятор даёт каждому
// разведчику номер (1, 2, ...) и раздаёт их камеры через один порт в сети роутера:
//   http://<ретранслятор>:8000/            — список разведчиков со ссылками
//   http://<ретранслятор>:8000/2/          -> разведчик №2 :80 (страница, /capture, /status)
//   http://<ретранслятор>:8000/2/stream    -> разведчик №2 :81 (поток MJPEG)
// Один порт, а не пара портов на каждого: у ESP32 всего 16 сокетов на всё (lwIP).
// Номер закреплён за адресом разведчика в «scout-net» (точка доступа выдаёт одному и тому же
// разведчику тот же адрес), пока ретранслятор включён.
//
// Адрес камеры для Syn: разведчик спрашивает http://192.168.5.1:8000/ip, ретранслятор отвечает
// «<свой адрес в сети роутера> <номер разведчика>», и в Syn уходит
// http://<ретранслятор>:8000/<номер>/stream — у каждого разведчика на сервере своя камера.
// Адрес ретранслятора — в мониторе порта или http://scout-relay.local:8000/ (mDNS).
//
// Нет связи с сервером — ретранслятор отказывает разведчику в подключении; разведчик через
// 15 с без отправок сам переключится на запасную сеть (WIFI_SSID2 в его скетче — например,
// тот же роутер напрямую), если до неё достаёт.
//
// У ESP32 один радиоканал на обе сети: при включении ретранслятор ищет роутер и
// поднимает свою сеть на его канале. Роутер должен работать на 2,4 ГГц (5 ГГц ESP32 не видит).
// Плата: любая ESP32 / ESP32-S3 / ESP32-C3; лучше с внешней антенной.
// Монитор порта 115200 — состояние раз в 2 с (на носителе порт не нужен).

#include <WiFi.h>
#include <ESPmDNS.h>

// ---------- Настройки ----------
const char *AP_SSID     = "scout-net";      // сеть для разведчиков (как WIFI_SSID в rescue_scout)
const char *AP_PASS     = "12345678";
const char *ROUTER_SSID = "my-router";      // Wi-Fi роутера, 2,4 ГГц
const char *ROUTER_PASS = "router-password";
// Ноутбук с iot-go: IP в сети роутера (закрепите его в роутере: «резервирование DHCP»)
// или имя «компьютер.local» — у Bazzite по умолчанию «bazzite.local» (работает через avahi).
const char *SERVER_HOST = "bazzite.local";
const uint16_t SERVER_PORT = 8080;          // TCP-порт устройств iot-go (server_tcp.port)
const uint16_t LISTEN_PORT = 9000;          // порт для разведчиков (SERVER_PORT в rescue_scout)
const uint16_t VIDEO_PORT = 8000;           // видео всех разведчиков: /<номер>/..., список — /
const int MAX_SCOUTS = 4;                   // разведчиков с номерами
// Пар подключений (телеметрия разведчика или зритель видео): по 2 сокета на пару, а всего
// сокетов 16, из них 2 — порты 9000 и 8000. Один слот всегда держится свободным для телеметрии.
const int MAX_PAIRS = 6;
const int LED_PIN = -1;                     // светодиод: горит — есть роутер и сервер (-1 — нет)

IPAddress apIp(192, 168, 5, 1), apMask(255, 255, 255, 0);
WiFiServer server(LISTEN_PORT);
WiFiServer web(VIDEO_PORT);                 // видео разведчиков, список, /ip для Syn

struct Scout { IPAddress ip; uint32_t seenMs = 0; };
Scout scouts[MAX_SCOUTS];                   // номер разведчика = индекс + 1

struct Pair {
  bool used = false;
  bool video = false;                       // false — телеметрия (разведчик -> сервер), true — видео (ноутбук -> разведчик)
  WiFiClient down, up;                      // кто подключился и куда переслали
  uint32_t bytesUp = 0, bytesDown = 0, startedMs = 0;
};
Pair pairs[MAX_PAIRS];
uint8_t buf[1460];
uint32_t totalUp = 0, refused = 0, opened = 0;
IPAddress serverIp;                         // разрешённый адрес сервера (0 — ещё не найден)
uint32_t lastResolve = 0;
bool serverReachable = false;

// ---------- Адрес сервера ----------
bool isLocalName(const char *h) {
  size_t n = strlen(h);
  return n > 6 && !strcmp(h + n - 6, ".local");
}

// IP сервера: число — сразу, «имя.local» — через mDNS (повтор не чаще раза в 5 с)
bool resolveServer() {
  if (serverIp != IPAddress((uint32_t)0)) return true;
  if (serverIp.fromString(SERVER_HOST)) return true;
  if (millis() - lastResolve < 5000 && lastResolve) return false;
  lastResolve = millis();
  if (isLocalName(SERVER_HOST)) {
    String name(SERVER_HOST);
    name.remove(name.length() - 6);
    IPAddress ip = MDNS.queryHost(name, 2000);
    if (ip != IPAddress((uint32_t)0)) {
      serverIp = ip;
      Serial.printf("[%lu] сервер %s = %s\n", (unsigned long)(millis() / 1000), SERVER_HOST, ip.toString().c_str());
      return true;
    }
    Serial.printf("[%lu] имя %s не найдено (avahi на ноутбуке, брандмауэр mDNS) — лучше указать IP\n",
                  (unsigned long)(millis() / 1000), SERVER_HOST);
    return false;
  }
  if (WiFi.hostByName(SERVER_HOST, serverIp)) return true;   // обычное имя через DNS роутера
  serverIp = IPAddress((uint32_t)0);
  return false;
}

// ---------- Пары подключений ----------
void closePair(Pair &p, const char *why) {
  Serial.printf("[%lu] %s %s: %s, передано %lu байт за %lu с\n", (unsigned long)(millis() / 1000),
                p.video ? "видео для" : "разведчик", p.down.remoteIP().toString().c_str(), why,
                (unsigned long)(p.video ? p.bytesDown : p.bytesUp), (unsigned long)((millis() - p.startedMs) / 1000));
  p.down.stop();
  p.up.stop();
  p.used = false;
}

void acceptScouts() {
  WiFiClient c = server.accept();
  if (!c) return;
  Pair *slot = nullptr;
  for (auto &p : pairs) if (!p.used) { slot = &p; break; }
  if (!slot || WiFi.status() != WL_CONNECTED || !resolveServer()) { refused++; c.stop(); return; }
  WiFiClient up;
  if (!up.connect(serverIp, SERVER_PORT, 1500)) {
    refused++;
    serverReachable = false;
    Serial.printf("[%lu] разведчик %s: сервер %s:%u не отвечает, отказ (сервер запущен? порт открыт в брандмауэре?)\n",
                  (unsigned long)(millis() / 1000), c.remoteIP().toString().c_str(),
                  serverIp.toString().c_str(), SERVER_PORT);
    if (isLocalName(SERVER_HOST)) serverIp = IPAddress((uint32_t)0);   // вдруг ноутбук сменил адрес
    c.stop();
    return;
  }
  serverReachable = true;
  c.setNoDelay(true);
  up.setNoDelay(true);
  slot->used = true;
  slot->video = false;
  slot->bytesDown = 0;
  scoutNumber(c.remoteIP(), false);            // номер выдаётся только устройствам с камерой (запрос /ip)
  slot->down = c;
  slot->up = up;
  slot->bytesUp = 0;
  slot->startedMs = millis();
  opened++;
  int num = scoutNumber(c.remoteIP(), false);
  if (num) Serial.printf("[%lu] разведчик №%d %s подключён к серверу %s:%u\n", (unsigned long)(millis() / 1000),
                         num, c.remoteIP().toString().c_str(), serverIp.toString().c_str(), SERVER_PORT);
  else Serial.printf("[%lu] устройство без камеры %s (капсула?) подключено к серверу %s:%u\n", (unsigned long)(millis() / 1000),
                     c.remoteIP().toString().c_str(), serverIp.toString().c_str(), SERVER_PORT);
}

// ---------- Номера разведчиков ----------
bool inScoutNet(IPAddress ip) { return ip[0] == apIp[0] && ip[1] == apIp[1] && ip[2] == apIp[2]; }

// Номер разведчика с этим адресом (1..MAX_SCOUTS); assign — выдать новый, если ещё нет. 0 — нет номера
int scoutNumber(IPAddress ip, bool assign) {
  for (int i = 0; i < MAX_SCOUTS; i++)
    if (scouts[i].ip == ip) { scouts[i].seenMs = millis(); return i + 1; }
  if (!assign || !inScoutNet(ip)) return 0;
  int k = -1;
  for (int i = 0; i < MAX_SCOUTS && k < 0; i++) if (scouts[i].ip == IPAddress((uint32_t)0)) k = i;
  if (k < 0) {                                   // все заняты — отдать номер того, кого дольше всех не было
    k = 0;
    for (int i = 1; i < MAX_SCOUTS; i++) if (scouts[i].seenMs < scouts[k].seenMs) k = i;
    Serial.printf("[%lu] номер %d переходит от %s к %s (разведчиков больше %d)\n", (unsigned long)(millis() / 1000), k + 1,
                  scouts[k].ip.toString().c_str(), ip.toString().c_str(), MAX_SCOUTS);
  }
  scouts[k].ip = ip;
  scouts[k].seenMs = millis();
  Serial.printf("[%lu] разведчик %s получил номер %d, видео — http://%s:%u/%d/\n", (unsigned long)(millis() / 1000),
                ip.toString().c_str(), k + 1, WiFi.localIP().toString().c_str(), VIDEO_PORT, k + 1);
  return k + 1;
}

bool scoutOnline(int n) {                        // есть телеметрия от разведчика n
  for (auto &p : pairs) if (p.used && !p.video && p.down.remoteIP() == scouts[n - 1].ip) return true;
  return false;
}

// ---------- HTTP на порту 8000 ----------
void reply(WiFiClient &c, const char *status, const char *type, const String &body, const char *extra = "") {
  c.printf("HTTP/1.1 %s\r\nContent-Type: %s\r\nContent-Length: %u\r\n%sCache-Control: no-store\r\n"
           "Connection: close\r\n\r\n", status, type, (unsigned)body.length(), extra);
  c.print(body);
  c.stop();
}

String indexPage() {
  String h = "<!doctype html><meta charset=utf-8><meta name=viewport content='width=device-width'>"
             "<title>Разведчики</title><style>body{font:16px system-ui;margin:16px;background:#111;color:#eee}"
             "a{color:#74c0fc}td{padding:6px 12px 6px 0}</style><h2>Разведчики на ретрансляторе</h2><table>";
  int n = 0;
  for (int i = 0; i < MAX_SCOUTS; i++) {
    if (scouts[i].ip == IPAddress((uint32_t)0)) continue;
    n++;
    h += "<tr><td><b>№" + String(i + 1) + "</b></td><td>" + scouts[i].ip.toString() + "</td><td>" +
         (scoutOnline(i + 1) ? "на связи" : "нет телеметрии") + "</td><td><a href='/" + String(i + 1) +
         "/'>страница</a></td><td><a href='/" + String(i + 1) + "/stream'>поток MJPEG</a></td></tr>";
  }
  if (!n) h += "<tr><td>Разведчиков пока не было</td></tr>";
  return h + "</table><p>Адрес потока для VLC: http://" + WiFi.localIP().toString() + ":" + String(VIDEO_PORT) +
         "/&lt;номер&gt;/stream</p>";
}

// Запрос на 8000: /ip — разведчику для Syn, / — список, /<номер>/... — к камере разведчика
void acceptWeb() {
  WiFiClient c = web.accept();
  if (!c) return;
  String head;                                   // первая строка запроса (и что пришло следом)
  uint32_t t0 = millis();
  while (c.connected() && millis() - t0 < 500 && head.indexOf("\r\n") < 0 && head.length() < 800) {
    while (c.available() && head.length() < 800) head += (char)c.read();
    if (head.indexOf("\r\n") < 0) delay(2);
  }
  int e = head.indexOf("\r\n");
  int s1 = head.indexOf(' '), s2 = e > 0 ? head.lastIndexOf(' ', e) : -1;
  if (e < 0 || s1 < 0 || s2 <= s1) { c.stop(); return; }
  String path = head.substring(s1 + 1, s2);

  if (path == "/ip") {                           // разведчик: мой адрес в сети роутера и его номер
    int n = scoutNumber(c.remoteIP(), true);
    String body = WiFi.status() == WL_CONNECTED && n ? WiFi.localIP().toString() + " " + String(n) : "";
    reply(c, "200 OK", "text/plain", body);
    return;
  }
  if (path == "/" || path == "/index.html") { reply(c, "200 OK", "text/html; charset=utf-8", indexPage()); return; }

  int k = 1, n = 0;                              // /<номер>[/остаток]
  while (k < (int)path.length() && isDigit(path[k])) n = n * 10 + (path[k++] - '0');
  if (k == 1 || n < 1 || n > MAX_SCOUTS || (k < (int)path.length() && path[k] != '/' && path[k] != '?')) {
    reply(c, "404 Not Found", "text/plain; charset=utf-8", "Нет такого адреса. Список разведчиков: /\r\n");
    return;
  }
  if (k == (int)path.length()) {                 // «/2» -> «/2/»: относительные ссылки страницы
    reply(c, "302 Found", "text/plain", "", ("Location: " + path + "/\r\n").c_str());
    return;
  }
  IPAddress ip = scouts[n - 1].ip;
  if (ip == IPAddress((uint32_t)0)) {
    reply(c, "503 Service Unavailable", "text/plain; charset=utf-8", "Разведчика с этим номером ещё не было\r\n");
    return;
  }
  String rest = path.substring(k);
  if (rest[0] == '?') rest = "/" + rest;
  uint16_t port = rest.startsWith("/stream") ? 81 : 80;

  Pair *slot = nullptr;
  int free = 0;
  for (auto &p : pairs) if (!p.used) { free++; if (!slot) slot = &p; }
  if (free < 2) {                                // последний слот — для телеметрии
    reply(c, "503 Service Unavailable", "text/plain; charset=utf-8",
          "Ретранслятор занят: слишком много зрителей. Закройте лишние окна с видео\r\n");
    return;
  }
  WiFiClient up;
  if (!up.connect(ip, port, 1500)) {
    Serial.printf("[%lu] видео: разведчик №%d %s:%u не отвечает (камера есть? прошивка с видео?)\n",
                  (unsigned long)(millis() / 1000), n, ip.toString().c_str(), port);
    reply(c, "502 Bad Gateway", "text/plain; charset=utf-8", "Разведчик не отвечает\r\n");
    return;
  }
  // Первая строка — уже с адресом разведчика: «GET /2/stream HTTP/1.1» -> «GET /stream HTTP/1.1»
  String first = head.substring(0, s1 + 1) + rest + head.substring(s2, e);
  up.print(first);
  up.print(head.substring(e));                   // остаток заголовков, что уже прочитан
  c.setNoDelay(true);
  up.setNoDelay(true);
  slot->used = true;
  slot->video = true;
  slot->down = c;
  slot->up = up;
  slot->bytesUp = slot->bytesDown = 0;
  slot->startedMs = millis();
  if (port == 81)
    Serial.printf("[%lu] видео: %s -> разведчик №%d %s\n", (unsigned long)(millis() / 1000),
                  c.remoteIP().toString().c_str(), n, ip.toString().c_str());
}

// Переслать всё доступное из a в b; false — запись не удалась
bool pump(WiFiClient &a, WiFiClient &b, uint32_t *count) {
  int n;
  while ((n = a.available()) > 0) {
    n = a.read(buf, n > (int)sizeof(buf) ? sizeof(buf) : n);
    if (n <= 0) break;
    const uint8_t *p = buf;
    size_t left = n;
    uint32_t t0 = millis();
    while (left) {                                  // write() может записать часть
      size_t w = b.write(p, left);
      if (w == 0) {
        if (!b.connected() || millis() - t0 > 3000) return false;
        delay(2);
        continue;
      }
      p += w; left -= w;
    }
    if (count) *count += n;
  }
  return true;
}

void pumpPairs() {
  for (auto &p : pairs) {
    if (!p.used) continue;
    uint32_t before = p.bytesUp;
    if (!pump(p.down, p.up, &p.bytesUp)) { closePair(p, p.video ? "ошибка записи разведчику" : "ошибка записи на сервер"); continue; }
    totalUp += p.bytesUp - before;
    if (!pump(p.up, p.down, &p.bytesDown)) { closePair(p, p.video ? "зритель отключился" : "ошибка записи разведчику"); continue; }
    if (!p.down.connected()) closePair(p, p.video ? "зритель отключился" : "разведчик отключился");
    else if (!p.up.connected()) closePair(p, p.video ? "разведчик закрыл видео" : "сервер закрыл подключение");
  }
}

// ---------- Запуск ----------
// Канал роутера: точка доступа для разведчиков сразу поднимается на нём
int findRouterChannel() {
  Serial.printf("Ищу роутер «%s»...\n", ROUTER_SSID);
  int n = WiFi.scanNetworks(false, false, false, 300);
  int ch = 0, best = -127;
  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) == ROUTER_SSID && WiFi.RSSI(i) > best) { best = WiFi.RSSI(i); ch = WiFi.channel(i); }
  }
  WiFi.scanDelete();
  if (ch) Serial.printf("Роутер найден: канал %d, %d дБм\n", ch, best);
  else Serial.printf("Роутер «%s» не найден (2,4 ГГц? имя?) — сеть разведчиков на канале 6, буду искать дальше\n", ROUTER_SSID);
  return ch;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  if (LED_PIN >= 0) pinMode(LED_PIN, OUTPUT);
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  int ch = findRouterChannel();
  WiFi.softAPConfig(apIp, apIp, apMask);
  WiFi.softAP(AP_SSID, AP_PASS, ch ? ch : 6, 0, MAX_PAIRS + 2);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ROUTER_SSID, ROUTER_PASS);
  MDNS.begin("scout-relay");
  server.begin();
  server.setNoDelay(true);
  web.begin();
  MDNS.addService("http", "tcp", VIDEO_PORT);
  Serial.printf("Ретранслятор: сеть «%s» %s:%u -> роутер «%s» -> сервер %s:%u\n", AP_SSID,
                apIp.toString().c_str(), LISTEN_PORT, ROUTER_SSID, SERVER_HOST, SERVER_PORT);
}

void loop() {
  acceptScouts();
  acceptWeb();
  pumpPairs();

  static bool wasUp = false;
  bool up = WiFi.status() == WL_CONNECTED;
  if (up && !wasUp) {
    IPAddress me = WiFi.localIP();
    Serial.printf("[%lu] подключён к роутеру: адрес %s, канал %d; видео разведчиков — http://%s:%u/\n",
                  (unsigned long)(millis() / 1000), me.toString().c_str(), WiFi.channel(), me.toString().c_str(),
                  VIDEO_PORT);
    if (me[0] == apIp[0] && me[1] == apIp[1] && me[2] == apIp[2])
      Serial.println("  ВНИМАНИЕ: сеть роутера совпадает с сетью разведчиков 192.168.5.x — смените apIp "
                     "здесь и SERVER_HOST в rescue_scout");
    serverIp = IPAddress((uint32_t)0);              // разрешить имя заново в новой сети
    resolveServer();
  }
  if (!up && wasUp) for (auto &p : pairs) if (p.used) closePair(p, "нет связи с роутером");
  wasUp = up;
  if (LED_PIN >= 0) digitalWrite(LED_PIN, up && serverReachable ? HIGH : LOW);

  static uint32_t last = 0;
  if (millis() - last >= 2000) {
    last = millis();
    int n = 0;
    for (auto &p : pairs) n += p.used;
    if (up)
      Serial.printf("[%lu] роутер: %d дБм, канал %d | сервер %s | разведчиков в сети %d, подключено %d | "
                    "передано %lu КБ, отказов %lu\n",
                    (unsigned long)(last / 1000), WiFi.RSSI(), WiFi.channel(),
                    serverIp == IPAddress((uint32_t)0) ? "адрес не найден" : serverIp.toString().c_str(),
                    WiFi.softAPgetStationNum(), n, (unsigned long)(totalUp / 1024), (unsigned long)refused);
    else
      Serial.printf("[%lu] НЕТ СВЯЗИ С РОУТЕРОМ «%s», разведчиков в сети %d\n", (unsigned long)(last / 1000),
                    ROUTER_SSID, WiFi.softAPgetStationNum());
  }
  delay(1);
}
