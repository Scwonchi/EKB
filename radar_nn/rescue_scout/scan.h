// Секторное сканирование и карта обнаружения. Не зависит от Arduino — проверяется на ПК.
//
// Устройство не управляет полётом: его поворачивает носитель (пилот DJI Flip, позже —
// автопилот микродрона). Поэтому сканер только наблюдает курс:
//   - «остановка» — курс не меняется больше чем на stillDeg за stillMs;
//   - на остановке радар слушает dwellMs; данные радара в повороте не используются
//     (цели размазываются), треки сбрасываются;
//   - каждая остановка закрывает сектор ±coverHalfDeg вокруг своего курса;
//     круг закрыт полностью — сканирование завершено, устройство переходит
//     в наблюдение (неподвижный датчик);
//   - люди, найденные на разных остановках, сводятся в карту обнаружения:
//     одно и то же направление (±mergeDeg) и дистанция (±mergeM) — один человек.
//
// Углы карты относительные: от курса в начале сканирования, по часовой стрелке.
// Для них важно только, чтобы курс правильно менялся при повороте, — абсолютная
// точность компаса (в помещении она плохая) не нужна.
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>

namespace scan {

enum Phase { STANDBY = 1, SCANNING = 2, WATCHING = 3 };
enum Kind { MOVING = 1, STATIONARY = 2 };

struct Params {
  float    stillDeg = 5;          // курс «стоит», если за stillMs ушёл меньше, °
  uint32_t stillMs = 1000;        // столько курс должен стоять, чтобы началась остановка, мс
  uint32_t dwellMs = 3000;        // столько радар слушает на остановке, мс
  float    coverHalfDeg = 50;     // полуширина сектора, который закрывает остановка, °
  float    newStopDeg = 25;       // новая остановка — если курс ушёл от прошлой хотя бы на столько, °
  float    mergeDeg = 20;         // один человек: направление ближе, °
  float    mergeM = 0.8f;         //                и дистанция ближе, м
  uint32_t timeoutMs = 120000;    // сканирование заканчивается не позже, мс
};

struct Detection {
  uint32_t id = 0;
  float    distance = 0;          // м
  float    relBearing = 0;        // ° от курса в начале сканирования, [0, 360)
  float    absBearing = NAN;      // ° от севера (если курс достоверен)
  Kind     kind = STATIONARY;
  uint32_t stopIndex = 0;         // на какой остановке найден (с 1)
  uint32_t seen = 0;              // на скольких остановках подтверждён
  uint32_t firstMs = 0;
  uint32_t lastStop_ = 0;
};

// События для журнала (что произошло на этом шаге)
struct Step {
  bool stopStarted = false;       // началась остановка stops()
  bool stopEnded = false;         // остановка закончилась (dwell прошёл или поворот)
  bool completed = false;         // сканирование завершено
};

inline float wrap360(float a) { a = fmodf(a, 360.0f); if (a < 0) a += 360.0f; return a >= 360.0f ? 0 : a; }
inline float angDiff(float a, float b) { float d = wrap360(a - b); return d > 180 ? d - 360 : d; }  // a-b в (-180, 180]

class Scanner {
 public:
  static const int MAX_DET = 12;
  static const int BINS = 72;     // покрытие круга по 5°

  Params p;
  Detection det[MAX_DET];
  int nDet = 0;

  Phase phase() const { return phase_; }
  uint32_t stops() const { return stops_; }
  bool dwelling() const { return dwelling_; }
  // Радар слушает: идёт остановка (в наблюдении — всегда)
  bool listening() const { return phase_ == WATCHING || (phase_ == SCANNING && dwelling_); }
  float startHeading() const { return startHeading_; }
  // Текущий курс относительно начала сканирования
  float relHeading() const { return rel_; }
  uint32_t elapsedMs(uint32_t now) const { return phase_ == STANDBY ? 0 : (phase_ == SCANNING ? now : endMs_) - startMs_; }

  float coveredDeg() const {
    int n = 0;
    for (int i = 0; i < BINS; i++) n += covered_[i];
    return n * (360.0f / BINS);
  }

  void reset() {
    phase_ = STANDBY; stops_ = 0; dwelling_ = false; nDet = 0; nextId_ = 1;
    memset(covered_, 0, sizeof(covered_)); memset(map_, 0, sizeof(map_));
  }

  // Начать сканирование. heading — текущий курс, °; NAN — компаса нет:
  // тогда остановки идут подряд по времени, а направления — только относительно оси радара.
  void start(uint32_t now, float heading) {
    reset();
    phase_ = SCANNING;
    startMs_ = now;
    noHeading_ = isnan(heading);
    startHeading_ = noHeading_ ? NAN : wrap360(heading);
    rel_ = 0;
    anchor_ = 0; anchorMs_ = now;
    lastStopRel_ = NAN;
  }

  // Закончить сканирование досрочно (команда оператора)
  void finish(uint32_t now) { if (phase_ == SCANNING) { endDwell(); phase_ = WATCHING; endMs_ = now; } }

  // Вызывать часто (каждый цикл loop) с текущим курсом
  Step update(uint32_t now, float heading) {
    Step s;
    if (phase_ == STANDBY) return s;
    if (!noHeading_ && !isnan(heading)) rel_ = wrap360(heading - startHeading_);
    if (phase_ == WATCHING) return s;

    if (!noHeading_ && fabsf(angDiff(rel_, anchor_)) > p.stillDeg) { anchor_ = rel_; anchorMs_ = now; }
    bool still = noHeading_ || now - anchorMs_ >= p.stillMs;

    if (dwelling_) {
      if (!still) { endDwell(); s.stopEnded = true; }                  // начали поворачивать раньше времени
      else if (now - dwellStart_ >= p.dwellMs) {
        endDwell(); s.stopEnded = true;
        markCovered(stopRel_);
        if (noHeading_) { /* без компаса: одна «остановка» = весь обзор радара */ }
      }
    } else if (still && (noHeading_ ? stops_ == 0 : (isnan(lastStopRel_) || fabsf(angDiff(rel_, lastStopRel_)) >= p.newStopDeg))) {
      dwelling_ = true; dwellStart_ = now;
      stops_++; stopRel_ = rel_; lastStopRel_ = rel_;
      memset(map_, 0, sizeof(map_));
      s.stopStarted = true;
    }

    bool full = noHeading_ ? (stops_ > 0 && !dwelling_) : coveredDeg() >= 359.9f;
    if (full || now - startMs_ >= p.timeoutMs) {
      if (dwelling_) { endDwell(); s.stopEnded = true; }
      phase_ = WATCHING; endMs_ = now;
      s.completed = true;
    }
    return s;
  }

  // Подтверждённая цель трекера в момент, когда радар слушает.
  // trackId — номер трека (в пределах остановки), angle — от оси радара, ° (+ по часовой).
  // absHeading — достоверный курс от севера или NAN.
  // Возвращает индекс в det (или -1), isNew — человек появился в карте впервые.
  int observe(uint32_t now, uint32_t trackId, float dist, float angle, Kind kind, float absHeading, bool &isNew) {
    isNew = false;
    if (!listening()) return -1;
    float rb = wrap360((phase_ == WATCHING ? rel_ : stopRel_) + angle);
    int k = lookup(trackId);
    if (k < 0) {                                                      // новый трек на этой остановке
      for (int i = 0; i < nDet; i++)
        if (fabsf(angDiff(det[i].relBearing, rb)) <= p.mergeDeg && fabsf(det[i].distance - dist) <= p.mergeM) { k = i; break; }
      if (k < 0) {
        if (nDet >= MAX_DET) return -1;
        k = nDet++;
        Detection &d = det[k];
        d = Detection();
        d.id = nextId_++;
        d.stopIndex = stops_ ? stops_ : 1;
        d.firstMs = now;
        d.distance = dist;
        d.relBearing = rb;
        isNew = true;
      }
      remember(trackId, k);
    }
    Detection &d = det[k];
    uint32_t stopNo = phase_ == WATCHING ? 0xFFFFFFFF : stops_;
    if (d.lastStop_ != stopNo) { d.lastStop_ = stopNo; d.seen++; }
    d.distance += 0.3f * (dist - d.distance);                          // сглаживание
    d.relBearing = wrap360(d.relBearing + 0.3f * angDiff(rb, d.relBearing));
    d.kind = kind;
    d.absBearing = isnan(absHeading) ? NAN : wrap360(absHeading + angle);
    return k;
  }

 private:
  Phase    phase_ = STANDBY;
  uint32_t startMs_ = 0, endMs_ = 0, anchorMs_ = 0, dwellStart_ = 0, stops_ = 0, nextId_ = 1;
  float    startHeading_ = NAN, rel_ = 0, anchor_ = 0, stopRel_ = 0, lastStopRel_ = NAN;
  bool     dwelling_ = false, noHeading_ = false;
  uint8_t  covered_[BINS] = {};
  struct Map { uint32_t track; int8_t det; };
  Map      map_[8];                                                   // трек остановки -> человек карты

  void endDwell() { dwelling_ = false; memset(map_, 0, sizeof(map_)); }

  void markCovered(float center) {
    for (int i = 0; i < BINS; i++) {
      float binCenter = (i + 0.5f) * (360.0f / BINS);
      if (fabsf(angDiff(binCenter, center)) <= p.coverHalfDeg) covered_[i] = 1;
    }
  }

  int lookup(uint32_t track) const {
    for (auto &m : map_) if (m.track == track && track) return m.det;
    return -1;
  }
  void remember(uint32_t track, int k) {
    for (auto &m : map_) if (!m.track) { m.track = track; m.det = (int8_t)k; return; }
  }
};

}  // namespace scan
