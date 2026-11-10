// Имитатор капсулы с LD2410C: тот же код, что в прошивке (capsule/ld2410.h — разбор байтов
// радара, capsule_core.h — правила и нейросеть), а кадры радара — по сценам.
//
// Сцены (лес, капсула на земле; у каждой случайные дистанции, сила сигнала, ветер):
//   пусто      — фон земли и травы, порывы ветра (короткие всплески движения);
//   помеха     — куст или ветка качается на ветру в одной зоне: долгое движение, проходит правила;
//   неподвижен — человек стоит, сидит или лежит без сознания: только дыхание;
//   движется   — человек ходит туда-обратно с остановками.
//
// Сборка (из папки radar_nn/tools):
//   L=../../telemetry/esp/libraries/TelemetryProto/src
//   g++ -std=c++17 -O2 -I../capsule -I$L -I../../telemetry/tools capsule_sim.cpp $L/pb_*.c $L/telemetry/v1/*.pb.c -o capsule_sim
// Запуск:
//   ./capsule_sim                    — по 20 сцен каждого вида: сколько тревог по правилам и с сетью
//   ./capsule_sim synth ../data_capsule_sim 40 — записи для train.py (формат record.py, 60 с на сцену)
//   ./capsule_sim 127.0.0.1 9000     — в реальном времени на сервер: Syn/Ack, затем пусто -> человек -> пусто
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <sys/stat.h>
#include <time.h>
#include "capsule_core.h"
#include "telemetry_codec.h"
#include "pc_link.h"

enum Scene { EMPTY = 0, BUSH, STILL, WALK, NSCENES };
static const char *SCENE_RU[NSCENES] = {"пусто", "помеха (куст)", "человек неподвижен", "человек движется"};
static const char *SCENE_DIR[NSCENES] = {"pusto", "pomeha", "nepodvizhen", "dvizhetsya"};

static std::mt19937 rng(1);
static float U(float a, float b) { return std::uniform_real_distribution<float>(a, b)(rng); }
static float N(float s) { return std::normal_distribution<float>(0, s)(rng); }
static bool P(float p) { return U(0, 1) < p; }
static uint8_t clip(float v) { return v < 0 ? 0 : v > 100 ? 100 : (uint8_t)v; }

// Пороги радара по умолчанию (зоны 0–1 неподвижности не учитывают — там земля и корпус)
static const float THR_M[9] = {50, 50, 40, 30, 20, 15, 15, 15, 15};
static const float THR_S[9] = {101, 101, 40, 40, 30, 30, 20, 20, 20};

struct SceneGen {
  Scene kind;
  float baseSt[9], noiseMv;
  float gustRate; int gustLeft = 0, gustGate = 0; float gustAmp = 0;
  // помеха
  int bushGate; float bushF, bushAmp; bool bushOn = true;
  // человек
  float r, v, amp, breathF, breathAmp, pause = 0;
  bool lying;

  explicit SceneGen(Scene k) : kind(k) {
    for (int g = 0; g < 9; g++) baseSt[g] = g == 0 ? U(35, 55) : g == 1 ? U(20, 40) : U(4, 14);
    noiseMv = U(3, 7);
    gustRate = U(0.005f, 0.03f);
    bushGate = (int)U(1, 6); bushF = U(1.2f, 3.0f); bushAmp = U(25, 45);
    lying = P(0.4f);
    r = U(1.6f, 5.2f);
    v = (P(0.5f) ? 1 : -1) * U(0.4f, 1.2f);
    amp = lying ? U(25, 42) : U(40, 70);
    breathF = U(0.2f, 0.4f); breathAmp = U(4, 9);
  }

  ld2410::Frame frame(float t) {
    float mv[9], st[9];
    for (int g = 0; g < 9; g++) { mv[g] = U(0, noiseMv); st[g] = baseSt[g] + N(2.5f); }
    // порывы ветра — у всех сцен (лес)
    if (gustLeft > 0) { gustLeft--; mv[gustGate] += gustAmp * U(0.5f, 1.0f); if (gustGate + 1 < 9) mv[gustGate + 1] += gustAmp * 0.4f; }
    else if (P(gustRate)) { gustLeft = (int)U(3, 15); gustGate = (int)U(1, 6); gustAmp = U(15, 40); }

    if (kind == BUSH) {
      if (P(0.02f)) bushOn = !bushOn ? true : P(0.7f);              // ветер то стихает, то снова
      if (bushOn) {
        mv[bushGate] += bushAmp * (0.6f + 0.4f * fabsf(sinf(6.283f * bushF * t))) + N(4);
        if (bushGate + 1 < 9) mv[bushGate + 1] += 0.4f * bushAmp * fabsf(sinf(6.283f * bushF * t + 1));
        st[bushGate] += 12 + N(3);
      }
    } else if (kind == STILL) {
      int k = (int)(r / ld2410::GATE_M);
      float b = sinf(6.283f * breathF * t);
      st[k] += amp + breathAmp * b + N(2);
      if (k > 0) st[k - 1] += 0.35f * amp;
      if (k < 8) st[k + 1] += 0.35f * amp;
      mv[k] += (lying ? 8 : 15) + (lying ? 5 : 9) * fabsf(b) + N(3);
      if (!lying && P(0.004f)) pause = U(0.5f, 1.5f);               // переступил, повернулся
      if (pause > 0) { pause -= 0.1f; mv[k] += U(30, 60); }
    } else if (kind == WALK) {
      if (pause > 0) pause -= 0.1f;
      else {
        r += v * 0.1f;
        if (r < 1.0f || r > 5.6f) { v = -v; r += 2 * v * 0.1f; }
        if (P(0.01f)) pause = U(1, 3);
      }
      int k = (int)(r / ld2410::GATE_M);
      float a = pause > 0 ? U(10, 25) : U(55, 90);
      mv[k] += a;
      if (k > 0) mv[k - 1] += 0.4f * a;
      if (k < 8) mv[k + 1] += 0.4f * a;
      st[k] += 25 + N(4);
    }
    // Решение самого радара: превышение порога в какой-нибудь зоне
    ld2410::Frame f;
    f.engineering = true;
    int bm = -1, bs = -1; float em = 0, es = 0;
    for (int g = 0; g < 9; g++) {
      f.mv[g] = clip(mv[g]); f.st[g] = clip(st[g]);
      if (f.mv[g] - THR_M[g] > em) { em = f.mv[g] - THR_M[g]; bm = g; }
      if (f.st[g] - THR_S[g] > es) { es = f.st[g] - THR_S[g]; bs = g; }
    }
    f.state = (bm >= 0 ? 1 : 0) | (bs >= 0 ? 2 : 0);
    if (bm >= 0) { f.movDistCm = bm * 75 + 37; f.movEnergy = f.mv[bm]; }
    if (bs >= 0) { f.statDistCm = bs * 75 + 37; f.statEnergy = f.st[bs]; }
    f.detDistCm = bm >= 0 ? f.movDistCm : f.statDistCm;
    return f;
  }
};

// Через байты, как на плате: кадр -> encode -> Parser
static ld2410::Parser parser;
static bool viaBytes(const ld2410::Frame &f, ld2410::Frame &out) {
  uint8_t b[64];
  int n = ld2410::encode(f, b);
  bool got = false;
  for (int i = 0; i < n; i++) got |= parser.feed(b[i]);
  out = parser.f;
  return got;
}

// Одна сцена 60 с: была ли тревога (после успокоения), через сколько, дистанция
struct Result { bool alarm; float firstS; float dist; };
static Result run(Scene k, capsule::NnMode md, float *trueR = nullptr) {
  SceneGen sg(k);
  if (trueR) *trueR = sg.r;
  capsule::Capsule c;
  c.p.nnMode = md;
  c.boot(0);
  Result r{false, -1, 0};
  for (uint32_t ms = 0; ms < 70000; ms += 100) {
    ld2410::Frame f;
    if (!viaBytes(sg.frame(ms / 1000.0f), f)) continue;
    c.frame(f, ms);
    if (c.alarmOn && !r.alarm) { r.alarm = true; r.firstS = (ms - c.p.settleMs) / 1000.0f; r.dist = c.distanceM; }
  }
  return r;
}

static int evaluate() {
  printf("Модель: %s\n", capsule::modelInfo());
  printf("По 20 сцен каждого вида, 60 с после успокоения капсулы:\n");
  printf("  сцена: тревог только по правилам | с сетью (nn gate)\n");
  bool ok = true;
  for (int k = 0; k < NSCENES; k++) {
    int cnt[2] = {0, 0}; float tsum = 0; int tn = 0;
    for (int m = 0; m < 2; m++) {
      if (m == 1 && !capsule::haveModel()) break;
      rng.seed(1000 + k * 100);                       // одни и те же сцены для обоих режимов
      for (int i = 0; i < 20; i++) {
        Result r = run((Scene)k, m ? capsule::NN_GATE : capsule::NN_OFF);
        cnt[m] += r.alarm;
        if (m == 1 && r.alarm) { tsum += r.firstS; tn++; }
      }
    }
    char b2[32];
    if (capsule::haveModel()) snprintf(b2, sizeof b2, "%d/20", cnt[1]); else snprintf(b2, sizeof b2, "нет модели");
    printf("  %s: %d/20 | %s", SCENE_RU[k], cnt[0], b2);
    if (tn) printf(" первая тревога в среднем через %.1f с", tsum / tn);
    printf("\n");
    int got = capsule::haveModel() ? cnt[1] : cnt[0];
    if (k == EMPTY && got > 1) ok = false;
    if ((k == STILL || k == WALK) && got < 18) ok = false;
  }
  printf(ok ? "ИТОГ: люди найдены, ложных тревог нет\n" : "ИТОГ: есть ошибки\n");
  return ok ? 0 : 1;
}

static int synth(const char *dir, int perClass) {
  mkdir(dir, 0755);
  rng.seed(42);                                       // не те сцены, что в проверке
  for (int k = 0; k < NSCENES; k++) {
    std::string d = std::string(dir) + "/" + SCENE_DIR[k];
    mkdir(d.c_str(), 0755);
    for (int i = 0; i < perClass; i++) {
      SceneGen sg((Scene)k);
      char path[512];
      snprintf(path, sizeof path, "%s/s%03d.csv", d.c_str(), i);
      FILE *o = fopen(path, "w");
      if (!o) { perror(path); return 1; }
      fprintf(o, "m0,m1,m2,m3,m4,m5,m6,m7,m8,s0,s1,s2,s3,s4,s5,s6,s7,s8\n");
      for (int n = 0; n < 600; n++) {
        ld2410::Frame f = sg.frame(n / 10.0f);
        for (int g = 0; g < 9; g++) fprintf(o, "%u,", f.mv[g]);
        for (int g = 0; g < 9; g++) fprintf(o, g < 8 ? "%u," : "%u\n", f.st[g]);
      }
      fclose(o);
    }
  }
  FILE *m = fopen((std::string(dir) + "/SYNTHETIC").c_str(), "w");
  if (m) { fprintf(m, "Синтетические записи tools/capsule_sim.cpp — модель на них только оценивает (nn advise)\n"); fclose(m); }
  printf("Синтетика: по %d сцен (60 с) в %s/{pusto,pomeha,nepodvizhen,dvizhetsya}\n", perClass, dir);
  return 0;
}

static int sock = -1;
static bool sinkWrite(void *, const uint8_t *d, size_t n) { return pclink::writeAll(sock, d, n); }

static int live(const char *host, int port) {
  const uint8_t mac[6] = {0x02, 0xca, 0x00, 0x00, 0x00, 0x01};
  sock = pclink::connect(host, port, mac);            // без оборудования: у капсулы нет камеры
  if (sock < 0) return 1;
  capsule::Capsule c;
  c.boot(0);
  printf("Сценарий: 8 с успокоение, 20 с пусто, 40 с человек сидит (2,6 м), 30 с пусто\n");
  rng.seed(5);
  SceneGen empty(EMPTY), still(STILL);
  still.r = 2.6f; still.lying = false; still.amp = 55;
  bool was = false;
  for (uint32_t ms = 0; ms <= 98000; ms += 100) {
    Scene k = ms >= 28000 && ms < 68000 ? STILL : EMPTY;
    ld2410::Frame f;
    if (viaBytes((k == STILL ? still : empty).frame(ms / 1000.0f), f)) c.frame(f, ms);
    if (c.alarmOn != was) { printf("[%5.1f с] %s, %.1f м\n", ms / 1000.0, c.alarmOn ? "ТРЕВОГА" : "отбой", c.distanceM); was = c.alarmOn; }
    if (ms % 200 == 0) {
      static telemetry_v1_Telemetry t;
      c.fill(t, ms, true);
      t.rssi_dbm = -60;
      if (!telemetry::writeTelemetry(t, sinkWrite, nullptr)) { printf("ошибка отправки\n"); return 1; }
    }
    struct timespec ts{0, 100 * 1000 * 1000};
    nanosleep(&ts, nullptr);
  }
  close(sock);
  return 0;
}

int main(int argc, char **argv) {
  if (argc >= 3 && !strcmp(argv[1], "synth")) return synth(argv[2], argc > 3 ? atoi(argv[3]) : 40);
  if (argc >= 3) return live(argv[1], atoi(argv[2]));
  return evaluate();
}
