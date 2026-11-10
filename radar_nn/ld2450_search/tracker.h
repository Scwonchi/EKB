// Сопровождение целей LD2450 и решение «здесь человек».
// Не зависит от Arduino — проверяется на компьютере.
//
// Радар сам выдаёт до 3 целей, но в отдельных кадрах бывают ложные точки
// (отражения, корпус дрона), а неподвижный человек иногда пропадает на 1–2 с.
// Поэтому:
//   1. Точки кадра привязываются к трекам (ближайший трек в пределах gateM).
//   2. Трек подтверждается как человек, если живёт ≥ confirmMs и виден
//      в ≥ minHitRatio кадров. Пропуски до missTimeoutMs трек переживает.
//   3. Карта фона: 20 с при пустой зоне — клетки, где радар «видит» что-то
//      постоянно (мебель, стены, свои конструкции), потом игнорируются.
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "ld2450.h"

namespace trk {

struct Params {
  float    gateM = 0.8f;          // радиус привязки точки к треку, м
  uint32_t missTimeoutMs = 3000;  // трек живёт без точек столько, мс
  uint32_t confirmMs = 2000;      // подтверждение не раньше, мс
  float    minHitRatio = 0.5f;    // доля кадров с точкой
  float    minRangeM = 0.3f;      // ближе — корпус дрона и провода
  float    maxRangeM = 6.0f;      // дальше радар ненадёжен
  float    movingSpeed = 0.15f;   // м/с — «движется»
  float    movingSpread = 0.4f;   // м — разброс положения за 3 с, тоже «движется»
  float    bgThreshold = 0.05f;   // клетка — фон, если занята > 5% кадров обучения
};

enum Kind { KIND_NONE = 0, KIND_MOVING = 1, KIND_STATIONARY = 2 };

struct Track {
  bool     used = false;
  uint32_t id = 0;
  float    x = 0, y = 0;          // сглаженное положение, м
  float    speed = 0;             // сглаженный модуль скорости, м/с
  uint32_t firstMs = 0, lastMs = 0;
  uint32_t hits = 0, frames = 0;
  bool     confirmed = false;
  Kind     kind = KIND_NONE;
  static const int HIST = 30;     // ~3 с при 10 кадрах/с
  float    hx[HIST], hy[HIST];
  int      hn = 0, hpos = 0;

  float range() const { return sqrtf(x * x + y * y); }
  float hitRatio() const { return frames ? (float)hits / frames : 0; }
};

class Tracker {
 public:
  static const int MAX_TRACKS = 6;
  // Карта фона: X от −6 до 6 м, Y от 0 до 6 м, клетка 0,25 м
  static const int GX = 48, GY = 24;
  static constexpr float CELL = 0.25f, X0 = -6.0f;

  Params p;
  Track tracks[MAX_TRACKS];

  // ----- Фон -----
  void startBackground(uint32_t now, uint32_t durationMs = 20000) {
    memset(bgCount_, 0, sizeof(bgCount_));
    bgFrames_ = 0;
    bgUntil_ = now + durationMs;
    learning_ = true;
    clearTracks();
  }
  void clearBackground() { memset(clutter_, 0, sizeof(clutter_)); bgCells_ = 0; }
  bool learning() const { return learning_; }
  int  backgroundCells() const { return bgCells_; }
  uint32_t learnLeftMs(uint32_t now) const { return learning_ && (int32_t)(bgUntil_ - now) > 0 ? bgUntil_ - now : 0; }

  bool inClutter(float x, float y) const {
    int cx, cy;
    return cell(x, y, cx, cy) && clutter_[cy][cx];
  }

  // ----- Обработка кадра -----
  void update(const ld2450::Target *t, int n, uint32_t now) {
    if (learning_) {
      for (int i = 0; i < n; i++) {
        int cx, cy;
        if (t[i].valid && cell(t[i].x, t[i].y, cx, cy) && bgCount_[cy][cx] < 65535) bgCount_[cy][cx]++;
      }
      bgFrames_++;
      if ((int32_t)(now - bgUntil_) >= 0) finishBackground();
      return;
    }

    for (auto &tr : tracks) if (tr.used) tr.frames++;

    bool taken[MAX_TRACKS] = {false};
    for (int i = 0; i < n; i++) {
      const ld2450::Target &d = t[i];
      if (!d.valid) continue;
      float r = sqrtf(d.x * d.x + d.y * d.y);
      if (r < p.minRangeM || r > p.maxRangeM || inClutter(d.x, d.y)) continue;

      int best = -1; float bestD = p.gateM;
      for (int k = 0; k < MAX_TRACKS; k++) {
        if (!tracks[k].used || taken[k]) continue;
        float dd = hypotf(tracks[k].x - d.x, tracks[k].y - d.y);
        if (dd < bestD) { bestD = dd; best = k; }
      }
      if (best < 0) best = newTrack(d, now);
      if (best < 0) continue;
      taken[best] = true;
      Track &tr = tracks[best];
      if (tr.hits) {                                  // сглаживание
        tr.x += 0.5f * (d.x - tr.x);
        tr.y += 0.5f * (d.y - tr.y);
        tr.speed += 0.3f * (fabsf(d.speed) - tr.speed);
      }
      tr.hits++;
      tr.lastMs = now;
      tr.hx[tr.hpos] = tr.x; tr.hy[tr.hpos] = tr.y;
      tr.hpos = (tr.hpos + 1) % Track::HIST;
      if (tr.hn < Track::HIST) tr.hn++;
    }

    for (auto &tr : tracks) {
      if (!tr.used) continue;
      if (now - tr.lastMs > p.missTimeoutMs) { tr.used = false; continue; }
      if (!tr.confirmed && now - tr.firstMs >= p.confirmMs && tr.hitRatio() >= p.minHitRatio)
        tr.confirmed = true;
      tr.kind = (tr.speed > p.movingSpeed || spread(tr) > p.movingSpread) ? KIND_MOVING : KIND_STATIONARY;
    }
  }

  // Подтверждённые люди
  int personCount() const {
    int c = 0;
    for (auto &tr : tracks) if (tr.used && tr.confirmed) c++;
    return c;
  }

  // Самый надёжный подтверждённый трек (дольше всех виден), nullptr — людей нет
  const Track *best() const {
    const Track *b = nullptr;
    for (auto &tr : tracks)
      if (tr.used && tr.confirmed && (!b || tr.hits > b->hits)) b = &tr;
    return b;
  }

  void clearTracks() { for (auto &tr : tracks) tr.used = false; }

 private:
  uint16_t bgCount_[GY][GX];
  bool     clutter_[GY][GX] = {};
  uint32_t bgFrames_ = 0, bgUntil_ = 0, nextId_ = 1;
  bool     learning_ = false;
  int      bgCells_ = 0;

  static bool cell(float x, float y, int &cx, int &cy) {
    cx = (int)floorf((x - X0) / CELL);
    cy = (int)floorf(y / CELL);
    return cx >= 0 && cx < GX && cy >= 0 && cy < GY;
  }

  void finishBackground() {
    learning_ = false;
    memset(clutter_, 0, sizeof(clutter_));
    if (!bgFrames_) return;
    bool raw[GY][GX];
    for (int y = 0; y < GY; y++)
      for (int x = 0; x < GX; x++) raw[y][x] = bgCount_[y][x] > p.bgThreshold * bgFrames_;
    bgCells_ = 0;
    for (int y = 0; y < GY; y++)                      // расширяем на соседние клетки
      for (int x = 0; x < GX; x++) {
        bool c = false;
        for (int dy = -1; dy <= 1 && !c; dy++)
          for (int dx = -1; dx <= 1 && !c; dx++) {
            int yy = y + dy, xx = x + dx;
            if (yy >= 0 && yy < GY && xx >= 0 && xx < GX && raw[yy][xx]) c = true;
          }
        clutter_[y][x] = c;
        if (c) bgCells_++;
      }
  }

  int newTrack(const ld2450::Target &d, uint32_t now) {
    for (int k = 0; k < MAX_TRACKS; k++) {
      if (tracks[k].used) continue;
      Track &tr = tracks[k];
      tr = Track();
      tr.used = true; tr.id = nextId_++;
      tr.x = d.x; tr.y = d.y; tr.speed = fabsf(d.speed);
      tr.firstMs = tr.lastMs = now;
      tr.frames = 1;
      return k;
    }
    return -1;
  }

  static float spread(const Track &tr) {
    if (tr.hn < 5) return 0;
    float mx = 0, my = 0;
    for (int i = 0; i < tr.hn; i++) { mx += tr.hx[i]; my += tr.hy[i]; }
    mx /= tr.hn; my /= tr.hn;
    float m = 0;
    for (int i = 0; i < tr.hn; i++) m = fmaxf(m, hypotf(tr.hx[i] - mx, tr.hy[i] - my));
    return m;
  }
};

}  // namespace trk
