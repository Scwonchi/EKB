// Данные для нейросети треков (rescue_scout/track_nn.h). Тот же трекер и те же признаки,
// что в прошивке.
//
// Сборка (из папки radar_nn/tools):
//   g++ -std=c++17 -O2 -I../rescue_scout track_data.cpp -o track_data
// Режимы:
//   ./track_data synth ../data_tracks 160   — синтетические записи: 160 сцен по 40 с
//        (люди, призраки-отражения, шум, вибрирующие предметы, покачивание дрона)
//        в data_tracks/sim_chelovek и data_tracks/sim_pusto, формат как у record_tracks.py,
//        метка каждого трека известна точно
//   ./track_data replay запись.csv > новая.csv — пересчитать строки F из строк R
//        (если изменились признаки в track_nn.h — старые записи не пропадают)
//   ./track_data check track_check.csv      — сеть на C++ даёт те же числа, что train_tracks.py
//   ./track_data eval                       — новые синтетические сцены: сколько ложных «людей»
//        и пропусков даёт трекер только по правилам и с сетью
//
// Синтетика нужна, чтобы проверить весь путь и иметь стартовую модель. Настоящий LD2450
// ведёт себя иначе — перед соревнованием обучите сеть на своих записях (README).
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>
#include <sys/stat.h>
#include "tracker.h"

static std::mt19937 rng(2450);
static float U(float a, float b) { return std::uniform_real_distribution<float>(a, b)(rng); }
static float N(float s) { return std::normal_distribution<float>(0, s)(rng); }
static bool P(float p) { return U(0, 1) < p; }
static const float D2R = 0.0174533f;

// Строки F — как печатает прошивка (recordFrame в rescue_scout.ino)
static void writeF(FILE *o, uint32_t ms, const trk::Track &t, const float *f, const char *label) {
  float ang = atan2f(-t.x, t.y) * 57.29578f;
  fprintf(o, "F,%u,%u,%.3f,%.1f,%s", ms, t.id, t.range(), ang, label);
  for (int i = 0; i < tnn::NF; i++) fprintf(o, ",%.4f", f[i]);
  fprintf(o, "\n");
}
static void writeR(FILE *o, uint32_t ms, const ld2450::Target *t) {
  fprintf(o, "R,%u", ms);
  for (int i = 0; i < 3; i++)
    if (t[i].valid) fprintf(o, ",%.3f,%.3f,%.2f", t[i].x, t[i].y, t[i].speed);
    else fprintf(o, ",,,");
  fprintf(o, "\n");
}

// ---------------- Синтетическая сцена ----------------
struct Person {
  float x, y, vx = 0, vy = 0;
  bool moving;
  float pd, sr, sl;              // вероятность отметки, шум по дальности и вбок
  float pauseLeft = 0;
  bool ghost; float gdr, gda, gpd, gs;   // призрак: +дальность, +угол, вероятность, шум
  float lastSpeed = 0;
};
struct Clutter { float x, y, pd, amp; };

static void polar(float r, float aDeg, float &x, float &y) { x = -r * sinf(aDeg * D2R); y = r * cosf(aDeg * D2R); }

struct Stats { int falseConf = 0, trueConf = 0, windows = 0, found = 0; };

static void synthScene(FILE *o, int people, bool scanStops, trk::NnMode mode = trk::NN_OFF, Stats *st = nullptr) {
  std::vector<Person> ps;
  for (int i = 0; i < people; i++) {
    Person p;
    p.moving = P(0.45f);
    polar(U(0.8f, 5.5f), U(-50, 50), p.x, p.y);
    p.pd = p.moving ? U(0.8f, 0.98f) : U(0.45f, 0.95f);
    p.sr = U(0.02f, 0.07f); p.sl = U(0.03f, 0.10f);
    if (p.moving) { float v = U(0.25f, 1.2f), h = U(0, 6.283f); p.vx = v * cosf(h); p.vy = v * sinf(h); }
    p.ghost = P(0.55f);
    p.gdr = U(0.6f, 2.2f); p.gda = U(10, 40) * (P(0.5f) ? 1 : -1);
    p.gpd = U(0.15f, 0.65f); p.gs = U(0.10f, 0.30f);
    ps.push_back(p);
  }
  std::vector<Clutter> cl;
  int nc = P(0.4f) ? (P(0.3f) ? 2 : 1) : 0;
  for (int i = 0; i < nc; i++) { Clutter c; polar(U(0.8f, 5.5f), U(-55, 55), c.x, c.y); c.pd = U(0.3f, 0.85f); c.amp = U(0.08f, 0.45f); cl.push_back(c); }
  float noiseRate = U(0.05f, 0.30f);
  float yawAmp = U(0, 2.5f), yawT = U(1.5f, 5), shift = U(0, 0.04f);
  bool burst = false; float bx = 0, by = 0; int burstLeft = 0;

  trk::Tracker tr;
  tr.p.nnMode = mode;                         // при сборе данных сеть не нужна (NN_OFF)
  uint32_t frame = 0;
  std::vector<uint32_t> seen;                 // для оценки: уже подтверждённые треки
  std::vector<bool> foundInWin(people, false);
  const float DT = 0.1f;
  for (uint32_t ms = 0; ms < 40000; ms += 100, frame++) {
    float t = ms / 1000.0f;
    if (scanStops && ms % 4000 == 0) {        // как на остановках сканирования
      if (st && ms) for (int i = 0; i < people; i++) { st->windows++; st->found += foundInWin[i]; foundInWin[i] = false; }
      tr.clearTracks();
    }
    // Движение людей
    for (auto &p : ps) {
      if (!p.moving) continue;
      if (p.pauseLeft > 0) { p.pauseLeft -= DT; continue; }
      if (P(0.01f)) p.pauseLeft = U(0.5f, 2.5f);
      if (P(0.02f)) { float v = hypotf(p.vx, p.vy), h = atan2f(p.vy, p.vx) + N(0.8f); p.vx = v * cosf(h); p.vy = v * sinf(h); }
      p.x += p.vx * DT; p.y += p.vy * DT;
      float r = hypotf(p.x, p.y), a = atan2f(-p.x, p.y) / D2R;
      if (r < 0.7f || r > 5.8f || fabsf(a) > 55) { p.x -= 2 * p.vx * DT; p.y -= 2 * p.vy * DT; p.vx = -p.vx; p.vy = -p.vy; }
    }
    // Покачивание дрона: поворот и сдвиг всей сцены
    float yaw = yawAmp * sinf(6.283f * t / yawT) * D2R + N(0.3f) * D2R;
    float cy = cosf(yaw), sy = sinf(yaw), ox = shift * sinf(t * 1.3f), oy = shift * cosf(t * 0.9f);
    auto view = [&](float x, float y, float &vx, float &vy) { vx = cy * x - sy * y + ox; vy = sy * x + cy * y + oy; };

    struct Pt { float x, y, s; int pri; };
    std::vector<Pt> cand;
    std::vector<std::pair<float, float>> truth;
    for (auto &p : ps) {
      float vx, vy; view(p.x, p.y, vx, vy);
      truth.push_back({vx, vy});
      float r = hypotf(vx, vy);
      float radial = (p.moving && p.pauseLeft <= 0) ? (p.vx * vx + p.vy * vy) / fmaxf(r, 0.1f) : 0;
      float spd = p.moving && p.pauseLeft <= 0 ? radial + N(0.05f) : (P(0.7f) ? 0 : N(0.05f));
      p.lastSpeed = spd;
      if (P(p.pd)) {
        float rr = r + N(p.sr), a = atan2f(-vx, vy) + N(p.sl) / fmaxf(r, 0.5f);
        cand.push_back({-rr * sinf(a), rr * cosf(a), spd, 0});
      }
      if (p.ghost && P(p.gpd)) {
        float a = atan2f(-vx, vy) / D2R + p.gda + N(4), rr = r + p.gdr + N(p.gs);
        float gx, gy; polar(rr, a, gx, gy);
        cand.push_back({gx + N(p.gs * 0.5f), gy, P(0.5f) ? spd * U(0.4f, 1.1f) : 0, 1});
      }
    }
    for (auto &c : cl)
      if (P(c.pd)) {
        float vx, vy; view(c.x, c.y, vx, vy);
        cand.push_back({vx + N(0.02f), vy + N(0.02f), P(0.25f) ? 0 : (P(0.5f) ? c.amp : -c.amp) * U(0.5f, 1.2f), 1});
      }
    if (burstLeft > 0) { burstLeft--; cand.push_back({bx + N(0.15f), by + N(0.15f), N(0.2f), 2}); }
    else if (P(noiseRate * 0.15f)) { polar(U(0.5f, 6), U(-60, 60), bx, by); burstLeft = (int)U(2, 12); burst = true; }
    if (P(noiseRate)) { float x, y; polar(U(0.4f, 6), U(-60, 60), x, y); cand.push_back({x, y, P(0.5f) ? 0 : N(0.3f), 2}); }
    (void)burst;
    // У LD2450 три слота: сначала люди, остальное — как повезёт
    std::shuffle(cand.begin(), cand.end(), rng);
    std::stable_sort(cand.begin(), cand.end(), [](const Pt &a, const Pt &b) { return a.pri < b.pri; });
    ld2450::Target tg[3];
    for (int i = 0; i < 3 && i < (int)cand.size(); i++) { tg[i].x = cand[i].x; tg[i].y = cand[i].y; tg[i].speed = cand[i].s; tg[i].valid = true; }

    tr.update(tg, 3, ms);
    if (st) {
      for (auto &k : tr.tracks) {
        if (!k.used || !k.confirmed) continue;
        int near = -1;
        for (int i = 0; i < (int)truth.size(); i++)
          if (hypotf(truth[i].first - k.x, truth[i].second - k.y) < 0.7f) near = i;
        if (near >= 0) foundInWin[near] = true;
        if (std::find(seen.begin(), seen.end(), k.id) != seen.end()) continue;
        seen.push_back(k.id);
        (near >= 0 ? st->trueConf : st->falseConf)++;
      }
      continue;
    }
    writeR(o, ms, tg);
    if (frame % 5) continue;
    for (auto &k : tr.tracks) {
      float f[tnn::NF];
      if (!tr.trackFeatures(k, f)) continue;
      bool person = false;
      for (auto &q : truth) if (hypotf(q.first - k.x, q.second - k.y) < 0.7f) person = true;
      writeF(o, ms, k, f, person ? "1" : "0");
    }
  }
}

static int synth(const char *dir, int scenes) {
  std::string d = dir;
  mkdir(d.c_str(), 0755);
  mkdir((d + "/sim_chelovek").c_str(), 0755);
  mkdir((d + "/sim_pusto").c_str(), 0755);
  for (int i = 0; i < scenes; i++) {
    int people = i % 2 ? 0 : (P(0.35f) ? 2 : 1);
    char path[512];
    snprintf(path, sizeof path, "%s/%s/s%03d.csv", dir, people ? "sim_chelovek" : "sim_pusto", i);
    FILE *o = fopen(path, "w");
    if (!o) { perror(path); return 1; }
    fprintf(o, "# source=synth label=%s people=%d\n", people ? "sim_chelovek" : "sim_pusto", people);
    synthScene(o, people, i % 4 < 2);
    fclose(o);
  }
  printf("Синтетика: %d сцен в %s/sim_chelovek и %s/sim_pusto\n", scenes, dir, dir);
  return 0;
}

// ---------------- Сравнение: только правила / правила + сеть ----------------
static int eval(int scenes) {
#if !TNN_HAVE_MODEL
  printf("Нет track_model.h\n"); return 1;
#else
  printf("Новые сцены (%d, сканирование с остановками по 4 с), порог сети %.2f:\n", scenes, trk::Params().nnThreshold);
  for (trk::NnMode md : {trk::NN_OFF, trk::NN_GATE}) {
    rng.seed(777);                            // одни и те же сцены для обоих режимов, не те, что для обучения
    Stats st;
    for (int i = 0; i < scenes; i++) synthScene(nullptr, i % 2 ? 0 : (P(0.35f) ? 2 : 1), true, md, &st);
    printf("  %-18s людей найдено на остановках %5.1f%% (%d/%d), ложных «людей» %d, настоящих подтверждений %d\n",
           md == trk::NN_OFF ? "только правила:" : "правила + сеть:", 100.0 * st.found / st.windows, st.found, st.windows,
           st.falseConf, st.trueConf);
  }
  return 0;
#endif
}

// ---------------- Пересчёт признаков из записи ----------------
static int replay(const char *path) {
  FILE *in = fopen(path, "r");
  if (!in) { perror(path); return 1; }
  trk::Tracker tr;
  tr.p.nnMode = trk::NN_OFF;
  char line[1024];
  uint32_t frame = 0;
  while (fgets(line, sizeof line, in)) {
    if (line[0] == '#') { fputs(line, stdout); continue; }
    if (line[0] != 'R') continue;
    fputs(line, stdout);
    // R,ms,x1,y1,v1,x2,y2,v2,x3,y3,v3 — пустые поля у пустых слотов
    std::vector<std::string> c;
    std::string cur;
    for (char *p = line; *p && *p != '\n' && *p != '\r'; p++) { if (*p == ',') { c.push_back(cur); cur.clear(); } else cur += *p; }
    c.push_back(cur);
    if (c.size() < 11) continue;
    uint32_t ms = strtoul(c[1].c_str(), nullptr, 10);
    ld2450::Target tg[3];
    for (int i = 0; i < 3; i++)
      if (!c[2 + i * 3].empty()) { tg[i].x = atof(c[2 + i * 3].c_str()); tg[i].y = atof(c[3 + i * 3].c_str()); tg[i].speed = atof(c[4 + i * 3].c_str()); tg[i].valid = true; }
    tr.update(tg, 3, ms);
    if (++frame % 5) continue;
    for (auto &k : tr.tracks) {
      float f[tnn::NF];
      if (tr.trackFeatures(k, f)) writeF(stdout, ms, k, f, "-");
    }
  }
  fclose(in);
  return 0;
}

// ---------------- Сверка сети с train_tracks.py ----------------
static int check(const char *path) {
#if !TNN_HAVE_MODEL
  printf("Нет track_model.h\n"); return 1;
#else
  FILE *in = fopen(path, "r");
  if (!in) { perror(path); return 1; }
  char line[1024];
  int n = 0; float worst = 0;
  while (fgets(line, sizeof line, in)) {
    float f[tnn::NF], want;
    char *p = line;
    bool ok = true;
    for (int i = 0; i < tnn::NF && ok; i++) { f[i] = strtof(p, &p); ok = *p == ','; p++; }
    if (!ok) continue;
    want = strtof(p, nullptr);
    float got = tnn::predict(f);
    worst = fmaxf(worst, fabsf(got - want));
    n++;
  }
  fclose(in);
  printf("Сверка сети: %d строк, наибольшее расхождение с Python %.2e — %s\n", n, worst, n && worst < 1e-4f ? "совпадает" : "ОШИБКА");
  return n && worst < 1e-4f ? 0 : 1;
#endif
}

int main(int argc, char **argv) {
  if (argc >= 3 && !strcmp(argv[1], "synth")) return synth(argv[2], argc > 3 ? atoi(argv[3]) : 160);
  if (argc >= 3 && !strcmp(argv[1], "replay")) return replay(argv[2]);
  if (argc >= 3 && !strcmp(argv[1], "check")) return check(argv[2]);
  if (argc >= 2 && !strcmp(argv[1], "eval")) return eval(argc > 2 ? atoi(argv[2]) : 200);
  fprintf(stderr, "track_data synth DIR [сцен] | replay FILE | check track_check.csv\n");
  return 2;
}
