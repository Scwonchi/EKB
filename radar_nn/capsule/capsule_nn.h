// Нейросеть капсулы: признаки окна и прямой проход — точная копия window_features() и
// numpy_forward() из train.py (та же сеть, что в ld2410c_nn). Не зависит от Arduino.
// model.h пишет train.py (--out capsule/model.h). Нет model.h — капсула работает по правилам.
#pragma once
#include <math.h>
#if __has_include("model.h")
#include "model.h"
#define CAPSULE_HAVE_MODEL 1
#ifndef NN_SYNTHETIC
#define NN_SYNTHETIC 0
#endif
#ifndef NN_INFO
#define NN_INFO "train.py"
#endif
#else
#define CAPSULE_HAVE_MODEL 0
#endif

#if CAPSULE_HAVE_MODEL

namespace nn {

static float cosT[NN_WIN], sinT[NN_WIN];
static bool tablesReady = false;

inline void initTables() {
  for (int n = 0; n < NN_WIN; n++) {
    cosT[n] = cosf(2.0f * (float)M_PI * n / NN_WIN);
    sinT[n] = sinf(2.0f * (float)M_PI * n / NN_WIN);
  }
  tablesReady = true;
}

// win[t][c] — окно NN_WIN кадров по NN_CH каналов (старые кадры первыми)
inline void features(const float win[][NN_CH], float *out) {
  if (!tablesReady) initTables();
  int f = 0;
  float d[NN_WIN];
  for (int c = 0; c < NN_CH; c++) {
    float mean = 0;
    for (int n = 0; n < NN_WIN; n++) mean += win[n][c];
    mean /= NN_WIN;
    float var = 0;
    for (int n = 0; n < NN_WIN; n++) { d[n] = win[n][c] - mean; var += d[n] * d[n]; }
    var /= NN_WIN;

    out[f++] = mean;
    out[f++] = sqrtf(var);

    for (int b = 0; b < NN_NBANDS; b++) {
      float sum = 0;
      for (int k = NN_BAND_LO[b]; k <= NN_BAND_HI[b]; k++) {
        float re = 0, im = 0;
        for (int n = 0; n < NN_WIN; n++) {
          int idx = (k * n) % NN_WIN;
          re += d[n] * cosT[idx];
          im -= d[n] * sinT[idx];
        }
        sum += (re * re + im * im) / NN_WIN;
      }
      out[f++] = log1pf(sum);
    }
  }
}

// Возвращает номер класса, prob[] — вероятности
inline int predict(const float *feat, float *prob) {
  float x[NN_FEAT], h1[NN_H1], h2[NN_H2];
  for (int i = 0; i < NN_FEAT; i++) x[i] = (feat[i] - NN_MU[i]) / NN_SD[i];

  for (int j = 0; j < NN_H1; j++) {
    float s = NN_B1[j];
    for (int i = 0; i < NN_FEAT; i++) s += x[i] * NN_W1[i * NN_H1 + j];
    h1[j] = s > 0 ? s : 0;
  }
  for (int j = 0; j < NN_H2; j++) {
    float s = NN_B2[j];
    for (int i = 0; i < NN_H1; i++) s += h1[i] * NN_W2[i * NN_H2 + j];
    h2[j] = s > 0 ? s : 0;
  }
  float mx = -1e30f;
  for (int j = 0; j < NN_CLASSES; j++) {
    float s = NN_B3[j];
    for (int i = 0; i < NN_H2; i++) s += h2[i] * NN_W3[i * NN_CLASSES + j];
    prob[j] = s;
    if (s > mx) mx = s;
  }
  float tot = 0;
  for (int j = 0; j < NN_CLASSES; j++) { prob[j] = expf(prob[j] - mx); tot += prob[j]; }
  int best = 0;
  for (int j = 0; j < NN_CLASSES; j++) { prob[j] /= tot; if (prob[j] > prob[best]) best = j; }
  return best;
}

}  // namespace nn
#endif  // CAPSULE_HAVE_MODEL
