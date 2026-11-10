// Нейросеть на борту: «этот трек LD2450 — человек или ложная цель».
// Не зависит от Arduino — тот же код работает в имитаторе и в инструментах обучения.
//
// LD2450 кроме людей выдаёт ложные точки: отражения от стен (призраки рядом с человеком),
// шум отдельных кадров, вибрирующие предметы (вентилятор, штора). Правила трекера
// (живёт ≥ 2 с, виден ≥ 50% кадров) отсекают короткий шум, но не устойчивые призраки.
// Сеть смотрит на поведение трека за последние 2 с (20 кадров) — 12 признаков — и даёт
// вероятность «человек». Сеть: полносвязная 12 → 16 → 8 → 1 (ReLU, на выходе сигмоида),
// 353 веса, расчёт — доли миллисекунды на ESP32-S3.
//
// Веса — track_model.h, его пишет tools/train_tracks.py. Признаки считаются только здесь:
// устройство в режиме записи (команда rec) печатает их готовыми, поэтому обучение на ПК
// видит ровно то же, что сеть в полёте.
#pragma once
#include <math.h>
#include <stdint.h>

#if __has_include("track_model.h")
#include "track_model.h"
#define TNN_HAVE_MODEL 1
#else
#define TNN_HAVE_MODEL 0
#endif

namespace tnn {

static const int WIN = 20;          // окно, кадров (2 с при 10 кадрах/с)
static const int MIN_FRAMES = 10;   // раньше сеть не считает: мало данных
static const int NF = 12;           // признаков

inline const char *featureName(int i) {
  static const char *n[NF] = {"дальность", "доля_кадров", "разброс_по_дальности", "разброс_вбок",
                              "скорость", "разброс_скорости", "доля_нулевой_скорости", "смещение",
                              "шаг", "самый_длинный_пропуск", "смены_знака_скорости", "до_соседа"};
  return i >= 0 && i < NF ? n[i] : "?";
}

// Сырые точки трека за последние WIN кадров (пропуск — hit = false)
struct History {
  float x[WIN], y[WIN], s[WIN];
  bool  hit[WIN];
  int   n = 0, pos = 0;
  void clear() { n = pos = 0; }
  void push(bool h, float px, float py, float ps) {
    hit[pos] = h; x[pos] = px; y[pos] = py; s[pos] = ps;
    pos = (pos + 1) % WIN;
    if (n < WIN) n++;
  }
  int at(int i) const { return (pos - n + i + WIN) % WIN; }   // i = 0 — самый старый
};

// 12 признаков. nearestM — расстояние до ближайшего другого трека (призраки ходят рядом с человеком)
inline void features(const History &h, float nearestM, float *f) {
  for (int i = 0; i < NF; i++) f[i] = 0;
  int hits = 0, miss = 0, maxMiss = 0, zero = 0, signCh = 0, steps = 0;
  float sr = 0, sr2 = 0, sa = 0, sa2 = 0, ss = 0, ss2 = 0, stepSum = 0;
  float fx = 0, fy = 0, lx = 0, ly = 0, lastSign = 0;
  for (int i = 0; i < h.n; i++) {
    int k = h.at(i);
    if (!h.hit[k]) { miss++; if (miss > maxMiss) maxMiss = miss; continue; }
    miss = 0;
    float r = sqrtf(h.x[k] * h.x[k] + h.y[k] * h.y[k]);
    float a = atan2f(h.x[k], h.y[k]);
    float s = fabsf(h.s[k]);
    sr += r; sr2 += r * r; sa += a; sa2 += a * a; ss += s; ss2 += s * s;
    if (s < 0.01f) zero++;
    else {
      float sg = h.s[k] > 0 ? 1.0f : -1.0f;
      if (lastSign != 0 && sg != lastSign) signCh++;
      lastSign = sg;
    }
    if (hits) { stepSum += hypotf(h.x[k] - lx, h.y[k] - ly); steps++; }
    else { fx = h.x[k]; fy = h.y[k]; }
    lx = h.x[k]; ly = h.y[k];
    hits++;
  }
  if (!hits || !h.n) return;
  float mr = sr / hits, ma = sa / hits, ms = ss / hits;
  f[0] = mr;
  f[1] = (float)hits / h.n;
  f[2] = sqrtf(fmaxf(0, sr2 / hits - mr * mr));
  f[3] = sqrtf(fmaxf(0, sa2 / hits - ma * ma)) * mr;
  f[4] = ms;
  f[5] = sqrtf(fmaxf(0, ss2 / hits - ms * ms));
  f[6] = (float)zero / hits;
  f[7] = hypotf(lx - fx, ly - fy);
  f[8] = steps ? stepSum / steps : 0;
  f[9] = (float)maxMiss / h.n;
  f[10] = (float)signCh / hits;
  f[11] = fminf(nearestM, 3.0f);
}

inline bool available() { return TNN_HAVE_MODEL; }

#if TNN_HAVE_MODEL
static_assert(TNN_NF == NF, "track_model.h от другой версии признаков — переобучите tools/train_tracks.py");

// Вероятность «человек», 0..1
inline float predict(const float *feat) {
  float x[NF], h1[TNN_H1], h2[TNN_H2];
  for (int i = 0; i < NF; i++) x[i] = (feat[i] - TNN_MU[i]) / TNN_SD[i];
  for (int j = 0; j < TNN_H1; j++) {
    float s = TNN_B1[j];
    for (int i = 0; i < NF; i++) s += x[i] * TNN_W1[i * TNN_H1 + j];
    h1[j] = s > 0 ? s : 0;
  }
  for (int j = 0; j < TNN_H2; j++) {
    float s = TNN_B2[j];
    for (int i = 0; i < TNN_H1; i++) s += h1[i] * TNN_W2[i * TNN_H2 + j];
    h2[j] = s > 0 ? s : 0;
  }
  float z = TNN_B3;
  for (int i = 0; i < TNN_H2; i++) z += h2[i] * TNN_W3[i];
  return 1.0f / (1.0f + expf(-z));
}
inline bool synthetic() { return TNN_SYNTHETIC; }
inline const char *info() { return TNN_INFO; }
#else
inline float predict(const float *) { return 1.0f; }
inline bool synthetic() { return true; }
inline const char *info() { return "модели нет (track_model.h не найден)"; }
#endif

}  // namespace tnn
