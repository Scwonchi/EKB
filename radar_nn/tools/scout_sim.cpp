// Имитатор разведчика на компьютере: тот же код, что в прошивке (mission.h, scan.h, tracker.h,
// кодек nanopb), но радар, компас и поворот носителя — по сценарию.
//
// Сценарий: носитель делает остановки через 45° (держит 4,5 с, поворачивает ~60°/с),
// курс для остановок — гироскоп (уход 0,05°/с, шум 0,3°), азимут от севера — компас (шум ±1,5°). В помещении два неподвижных человека и один движущийся,
// плюс ложные точки радара в 15% кадров. Время обнаружения и карта печатаются,
// в конце — проверка карты (направления ±10°, дистанции ±0,4 м).
//
// Сборка (из папки radar_nn/tools):
//   L=../../telemetry/esp/libraries/TelemetryProto/src
//   g++ -std=c++17 -O2 -I../rescue_scout -I$L -I../../telemetry/tools scout_sim.cpp $L/pb_*.c $L/telemetry/v1/*.pb.c -o scout_sim
// Запуск:
//   ./scout_sim                         — без сервера, ускоренно, только проверка алгоритма
//   ./scout_sim empty                   — то же без людей: проверка на ложные тревоги
//   ./scout_sim 127.0.0.1 9000          — в реальном времени шлёт снимки серверу (iot-go, server.py) по TCP,
//                                         при подключении — Syn/Ack, как прошивка (MAC 02:5c:00:00:00:01)
//   ./scout_sim gate  (или empty gate)  — нейросеть треков решает вместе с правилами (nn gate)
//   ./scout_sim off                     — без нейросети, только правила
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <time.h>
#include "mission.h"
#include "telemetry_codec.h"
#include "pc_link.h"

struct Person { float bearing, dist; bool moving; };
static const Person PEOPLE[] = {{100, 3.0f, false}, {250, 2.0f, true}, {262, 4.5f, false}};
static const float START_HEADING = 37;                  // курс носителя в начале (от севера)

static int sock = -1;
static bool sinkWrite(void *, const uint8_t *d, size_t n) {
  while (n) { ssize_t w = write(sock, d, n); if (w <= 0) return false; d += w; n -= (size_t)w; }
  return true;
}
static void say(const char *s) { printf("%s\n", s); }

// Курс носителя по сценарию: 8 остановок через 45°, на каждой 4,5 с, поворот 0,75 с
static float headingAt(uint32_t ms) {
  const uint32_t T0 = 1500, HOLD = 4500, TURN = 750, STEP = HOLD + TURN;
  if (ms < T0) return START_HEADING;
  uint32_t k = (ms - T0) / STEP, in = (ms - T0) % STEP;
  if (k >= 8) return START_HEADING + 8 * 45;           // замкнули круг — сел, смотрит туда же
  float h = START_HEADING + k * 45;
  if (in > HOLD) h += 45.0f * (in - HOLD) / TURN;
  return h;
}

int main(int argc, char **argv) {
  bool empty = false;
  int nnMode = -1;
  std::vector<const char *> pos;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "empty")) empty = true;
    else if (!strcmp(argv[i], "gate")) nnMode = trk::NN_GATE;
    else if (!strcmp(argv[i], "off")) nnMode = trk::NN_OFF;
    else pos.push_back(argv[i]);
  }
  bool live = pos.size() >= 2;
  if (live) {
    argv[1] = (char *)pos[0]; argv[2] = (char *)pos[1];
    const uint8_t mac[6] = {0x02, 0x5c, 0x00, 0x00, 0x00, 0x01};
    static telemetry_v1_Syn ob;
    telemetry::synAddOnboard(ob, "camera", "http://127.0.0.1:81/stream");
    sock = pclink::connect(argv[1], atoi(argv[2]), mac, &ob);
    if (sock < 0) return 1;
  }

  std::mt19937 rng(7);
  std::normal_distribution<float> n01(0, 1);
  std::uniform_real_distribution<float> u01(0, 1);

  mission::Mission m;
  m.print = say;
  if (nnMode >= 0) m.tracker.p.nnMode = (trk::NnMode)nnMode;
  printf("Нейросеть треков: %s, режим %s\n", tnn::info(),
         m.tracker.p.nnMode == trk::NN_GATE ? "решает" : m.tracker.p.nnMode == trk::NN_ADVISE ? "оценка" : "выключена");
  m.boot(0);

  uint32_t alarmSentMs = 0;
  uint32_t firstDetectMs = 0;
  const uint32_t END = 50000;
  for (uint32_t now = 0; now <= END; now += 20) {           // цикл loop() каждые 20 мс
    float h = headingAt(now) + 1.5f * n01(rng) * 0.5f;                               // компас
    float gy = scan::wrap360(headingAt(now) - START_HEADING + 0.05f * now / 1000 + 0.3f * n01(rng));  // гироскоп
    if (now == 300) m.mark(now, mission::EV_SEPARATED);       // отметки оператора/носителя
    if (now == 900) m.mark(now, mission::EV_ZONE_ENTERED);
    if (now == 1000) m.start(now, gy, scan::wrap360(h));
    m.tick(now, gy, alarmSentMs);

    if (now % 100 == 0) {                                    // кадр радара 10 раз в секунду
      ld2450::Target t[3];
      int k = 0;
      for (const Person &p : PEOPLE) {
        if (empty) break;
        float ang = scan::angDiff(p.bearing, headingAt(now));
        if (fabsf(ang) > 60 || k >= 3 || u01(rng) < 0.15f) continue;   // вне сектора или пропуск кадра
        float d = p.dist + (p.moving ? 0.35f * sinf(now / 900.0f) : 0) + 0.03f * n01(rng);
        float a = (ang + 1.0f * n01(rng)) / 57.29578f;
        t[k].x = -d * sinf(a); t[k].y = d * cosf(a); t[k].speed = p.moving ? 0.4f : 0.0f; t[k].valid = true; k++;
      }
      if (k < 3 && u01(rng) < 0.15f) {                       // ложная точка
        float d = 0.8f + 4.5f * u01(rng), a = (u01(rng) * 120 - 60) / 57.29578f;
        t[k].x = -d * sinf(a); t[k].y = d * cosf(a); t[k].valid = true; k++;
      }
      m.radarFrame(t, 3, now, scan::wrap360(h));
      if (m.scanner.nDet && !firstDetectMs) firstDetectMs = now;
    }

    if (now % 200 == 0) {                                    // снимок состояния 5 раз в секунду
      static telemetry_v1_Telemetry t;
      m.fill(t, now, true, scan::wrap360(h));
      t.rssi_dbm = -58;
      bool alarm = t.alarm_active;
      if (live) {
        if (!telemetry::writeTelemetry(t, sinkWrite, nullptr)) { printf("ошибка отправки\n"); return 1; }
      } else {
        uint8_t buf[telemetry::MAX_SNAPSHOT];
        if (!telemetry::encodeTelemetry(t, buf, sizeof buf)) { printf("ошибка кодирования\n"); return 1; }
      }
      if (alarm) alarmSentMs = now ? now : 1;
    }
    if (live) { struct timespec ts{0, 20 * 1000 * 1000}; nanosleep(&ts, nullptr); }
  }

  if (firstDetectMs) printf("\nПервое обнаружение через %.1f с после начала сканирования\n", (firstDetectMs - 1000) / 1000.0);
  m.printMap();
  // Проверка: каждый человек найден один раз, направление ±10°, дистанция ±0,4 м
  if (empty) {
    printf(m.scanner.nDet == 0 && !m.alarmOn ? "ИТОГ: ложных тревог нет\n" : "ИТОГ: ЛОЖНАЯ ТРЕВОГА, на карте %d\n", m.scanner.nDet);
    return m.scanner.nDet == 0 ? 0 : 1;
  }
  int ok = 0;
  for (const Person &p : PEOPLE) {
    float rel = scan::wrap360(p.bearing - START_HEADING);
    int hits = 0;
    for (int i = 0; i < m.scanner.nDet; i++) {
      const scan::Detection &d = m.scanner.det[i];
      if (fabsf(scan::angDiff(d.relBearing, rel)) <= 10 && fabsf(d.distance - p.dist) <= 0.4f) hits++;
    }
    printf("человек %.0f°/%.1f м (отн. %.0f°): %s\n", p.bearing, p.dist, rel, hits == 1 ? "найден" : hits ? "ДВАЖДЫ" : "НЕ НАЙДЕН");
    ok += hits == 1;
  }
  bool good = ok == 3 && m.scanner.nDet == 3;
  printf(good ? "ИТОГ: карта верна\n" : "ИТОГ: ошибка, лишних или пропущенных: %d\n", m.scanner.nDet - ok);
  if (live) close(sock);
  return good ? 0 : 1;
}
