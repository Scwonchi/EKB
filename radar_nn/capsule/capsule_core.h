// Логика капсулы без привязки к железу: кадры LD2410C -> правила + нейросеть -> тревога ->
// снимок telemetry.v1.Telemetry. Скетч только подаёт кадры радара и отправляет снимок;
// на компьютере тот же код гоняет tools/capsule_sim.cpp.
//
// Капсулу сбрасывают с носителя, она лежит на земле и слушает. LD2410C не даёт угла — только
// дистанцию (зоны по 0,75 м до 6 м) и энергии движения и неподвижности по зонам.
//
// Правила (работают всегда):
//   1. Отметка кадра — радар видит цель (своё состояние ≠ 0) и, если запомнен фон, энергия
//      хотя бы в одной зоне выше фона на margin. Фон (команда bg, 20 с никого рядом) —
//      трава, ветки, земля: то, что радар «видит» и без людей.
//   2. Доля отметок за 3 с ≥ onRatio дольше confirmMs — человек есть; ≤ offRatio дольше
//      clearMs — ушёл. Одиночные всплески (ветер, птица) не проходят.
//   3. Первые settleMs после включения (и после команды drop) ничего не решается: капсула
//      падает, качается, рядом ещё висит носитель.
// Нейросеть (capsule_nn.h, обучается train.py): окно 10 с, классы по папкам записей
// (pusto, nepodvizhen, dvizhetsya, pomeha). Вероятность «человек» = сумма классов-людей.
//   NN_GATE   — тревога = правила И сеть ≥ nnThreshold: куст на ветру, который проходит
//               правила (долгое движение в одной зоне), сеть отличает по ритму энергии;
//   NN_ADVISE — сеть только оценивает (в телеметрию), решают правила;
//   NN_OFF    — сеть не считается.
#pragma once
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "ld2410.h"
#include "capsule_nn.h"
#include "telemetry/v1/telemetry.pb.h"

namespace capsule {

enum NnMode { NN_OFF = 0, NN_ADVISE = 1, NN_GATE = 2 };

inline bool haveModel() { return CAPSULE_HAVE_MODEL; }
inline bool modelSynthetic() {
#if CAPSULE_HAVE_MODEL
  return NN_SYNTHETIC;
#else
  return true;
#endif
}
inline const char *modelInfo() {
#if CAPSULE_HAVE_MODEL
  return NN_INFO;
#else
  return "модели нет (capsule/model.h не найден — обучите train.py)";
#endif
}
// Сеть на записях с капсулы — решает; на синтетике — только оценивает
inline NnMode defaultNnMode() { return !haveModel() ? NN_OFF : modelSynthetic() ? NN_ADVISE : NN_GATE; }

struct Params {
  uint32_t settleMs = 8000;      // после включения/сброса не решать, мс
  uint32_t confirmMs = 2000;     // доля отметок высокая столько — человек, мс
  uint32_t clearMs = 5000;       // доля низкая столько — ушёл, мс
  float    onRatio = 0.6f;
  float    offRatio = 0.2f;
  int      margin = 10;          // на сколько энергия должна быть выше фона (0..100)
  float    maxRangeM = 6.0f;
  uint32_t bgMs = 20000;
  NnMode   nnMode = defaultNnMode();
  float    nnThreshold = 0.5f;   // сеть пропускает тревогу
  float    nnClear = 0.3f;       // и снимает разрешение ниже этого
};

class Capsule {
 public:
  static const int RING = 30;    // 3 с при 10 кадрах/с
  Params p;
  uint32_t firmwareVersion = 1;

  // Состояние (для вывода и снимка)
  bool     alarmOn = false;
  bool     rulesPresent = false;
  bool     moving = false;
  float    distanceM = 0;        // сглаженная дистанция цели
  float    ratio = 0;            // доля отметок за 3 с
  float    nnP = -1;             // вероятность «человек» по сети, −1 — не считалась
  int      nnClass = -1;
  uint32_t alarms = 0, trackId = 0;
  uint32_t nnRuns = 0, nnMicros = 0;

  void boot(uint32_t now) { settleUntil_ = now + p.settleMs; }
  void drop(uint32_t now) { settleUntil_ = now + p.settleMs; resetDecision(); }   // снова «упала»
  bool settling(uint32_t now) const { return (int32_t)(settleUntil_ - now) > 0; }
  uint32_t settleLeftMs(uint32_t now) const { return settling(now) ? settleUntil_ - now : 0; }

  // ----- Фон -----
  void startBackground(uint32_t now) {
    memset(bgMv_, 0, sizeof bgMv_); memset(bgSt_, 0, sizeof bgSt_);
    bgUntil_ = now + p.bgMs; learning_ = true; bgReady_ = false; bgFrames_ = 0;
    resetDecision();
  }
  void clearBackground() { bgReady_ = false; learning_ = false; }
  bool learning() const { return learning_; }
  bool backgroundReady() const { return bgReady_; }
  uint32_t learnLeftMs(uint32_t now) const { return learning_ && (int32_t)(bgUntil_ - now) > 0 ? bgUntil_ - now : 0; }
  // Зон, где фон заметный (для background_cells)
  int backgroundCells() const {
    if (!bgReady_) return 0;
    int n = 0;
    for (int g = 0; g < ld2410::GATES; g++) n += (bgMv_[g] > 15) + (bgSt_[g] > 15);
    return n;
  }
  void setBackground(const uint8_t *mv, const uint8_t *st) {   // из памяти ESP
    memcpy(bgMv_, mv, sizeof bgMv_); memcpy(bgSt_, st, sizeof bgSt_); bgReady_ = true; learning_ = false;
  }
  const uint8_t *bgMoving() const { return bgMv_; }
  const uint8_t *bgStatic() const { return bgSt_; }

  // ----- Кадр радара -----
  // Возвращает true, если закончилось запоминание фона (скетч сохраняет фон)
  bool frame(const ld2410::Frame &f, uint32_t now) {
    last_ = f;
    haveFrame_ = true;
    sample(f, now);
    if (learning_) {
      for (int g = 0; g < ld2410::GATES; g++) {
        if (f.mv[g] > bgMv_[g]) bgMv_[g] = f.mv[g];
        if (f.st[g] > bgSt_[g]) bgSt_[g] = f.st[g];
      }
      bgFrames_++;
      if ((int32_t)(now - bgUntil_) >= 0) { learning_ = false; bgReady_ = bgFrames_ > 0; return true; }
      return false;
    }
    bool hit = isHit(f);
    hits_[hpos_] = hit; movs_[hpos_] = hit && movingFrame(f);
    hpos_ = (hpos_ + 1) % RING;
    if (hn_ < RING) hn_++;
    int h = 0, mvn = 0;
    for (int i = 0; i < hn_; i++) { h += hits_[i]; mvn += movs_[i]; }
    ratio = hn_ ? (float)h / hn_ : 0;
    if (hit) {
      float d = (movingFrame(f) ? f.movDistCm : f.statDistCm) / 100.0f;
      if (d <= 0) d = f.detDistCm / 100.0f;
      distanceM = distanceM <= 0 ? d : distanceM + 0.3f * (d - distanceM);
    }
    moving = h && (float)mvn / h > 0.3f;

    if (settling(now)) { resetDecision(); return false; }
    if (ratio >= p.onRatio) {
      absentSince_ = 0;
      if (!presentSince_) presentSince_ = now;
      if (!rulesPresent && now - presentSince_ >= p.confirmMs) { rulesPresent = true; trackId++; }
    } else if (ratio <= p.offRatio) {
      if (!rulesPresent) presentSince_ = 0;
      else {
        if (!absentSince_) absentSince_ = now;
        if (now - absentSince_ >= p.clearMs) { rulesPresent = false; presentSince_ = 0; absentSince_ = 0; }
      }
    }
    decide();
    return false;
  }

  // Радар пропал: окно сети и решение — заново
  void radarLost() { hn_ = 0; ringFill_ = 0; ratio = 0; haveFrame_ = false; }

  // ----- Снимок для сервера (rssi заполняет скетч) -----
  void fill(telemetry_v1_Telemetry &t, uint32_t now, bool radarOk) const {
    t = telemetry_v1_Telemetry_init_zero;
    t.firmware_version = firmwareVersion;
    t.uptime_s = now / 1000;
    t.radar_ok = radarOk;
    if (settling(now) || learning_) t.detection_class = telemetry_v1_DetectionClass_DETECTION_CLASS_UNSPECIFIED;
    else if (alarmOn) t.detection_class = moving ? telemetry_v1_DetectionClass_DETECTION_CLASS_MOVING
                                                 : telemetry_v1_DetectionClass_DETECTION_CLASS_STATIONARY;
    else if (rulesPresent || nnSaysInterference()) t.detection_class = telemetry_v1_DetectionClass_DETECTION_CLASS_INTERFERENCE;
    else t.detection_class = telemetry_v1_DetectionClass_DETECTION_CLASS_EMPTY;
    float pr = nnP >= 0 && p.nnMode != NN_OFF ? nnP : ratio;
    t.person_probability = pr < 0 ? 0 : pr > 1 ? 1 : pr;
    t.heading_valid = false;                 // капсула лежит как упала: направления нет
    t.target_count = alarmOn ? 1 : 0;
    if (radarOk && !learning_ && ratio > 0 && distanceM > 0) {
      // У LD2410C нет угла: одна цель — дистанция, угол 0 (от оси радара неизвестен)
      telemetry_v1_Track &o = t.tracks[t.tracks_count++];
      o.id = trackId + 1;
      o.distance_m = distanceM;
      o.angle_deg = 0;
      o.speed_mps = 0;
      o.confirmed = alarmOn;
      o.detection_class = alarmOn ? t.detection_class : telemetry_v1_DetectionClass_DETECTION_CLASS_UNSPECIFIED;
      o.hit_ratio = ratio;
    }
    t.learning_background = learning_;
    t.background_cells = backgroundCells();
    t.alarm_active = alarmOn;
  }

  // Строка для записи данных обучения (формат ld2410c_logger / record.py): m0..m8,s0..s8
  int csvLine(char *b, int cap) const {
    int n = 0;
    for (int g = 0; g < ld2410::GATES; g++) n += snprintf(b + n, cap - n, "%u,", last_.mv[g]);
    for (int g = 0; g < ld2410::GATES; g++) n += snprintf(b + n, cap - n, g < 8 ? "%u," : "%u", last_.st[g]);
    return n;
  }
  bool sampleReady() const { return sampled_; }      // в этом кадре был отсчёт окна (10 Гц)
  const ld2410::Frame &lastFrame() const { return last_; }

  const char *nnLabel() const {
#if CAPSULE_HAVE_MODEL
    return nnClass >= 0 ? NN_LABELS[nnClass] : "—";
#else
    return "—";
#endif
  }

 private:
  ld2410::Frame last_;
  bool     haveFrame_ = false, sampled_ = false;
  uint8_t  bgMv_[ld2410::GATES] = {0}, bgSt_[ld2410::GATES] = {0};
  bool     bgReady_ = false, learning_ = false;
  uint32_t bgUntil_ = 0, bgFrames_ = 0, settleUntil_ = 0;
  bool     hits_[RING] = {false}, movs_[RING] = {false};
  int      hpos_ = 0, hn_ = 0;
  uint32_t presentSince_ = 0, absentSince_ = 0, lastSample_ = 0;
  bool     nnOk_ = false;
#if CAPSULE_HAVE_MODEL
  float    ring_[NN_WIN][NN_CH];
  float    win_[NN_WIN][NN_CH];
  float    feat_[NN_FEAT];
#endif
  int      ringPos_ = 0, ringFill_ = 0, sinceRun_ = 0;

  void resetDecision() {
    rulesPresent = false; presentSince_ = absentSince_ = 0;
    if (alarmOn) alarmOn = false;
  }

  bool movingFrame(const ld2410::Frame &f) const { return (f.state & 1) && f.movEnergy >= f.statEnergy / 2; }

  bool isHit(const ld2410::Frame &f) const {
    if (!f.state) return false;
    float d = (movingFrame(f) ? f.movDistCm : f.statDistCm) / 100.0f;
    if (d > p.maxRangeM) return false;
    if (!bgReady_) return true;
    for (int g = 0; g < ld2410::GATES; g++)
      if (f.mv[g] > bgMv_[g] + p.margin || f.st[g] > bgSt_[g] + p.margin) return true;
    return false;
  }

  bool nnSaysInterference() const {
#if CAPSULE_HAVE_MODEL
    return nnClass >= 0 && p.nnMode != NN_OFF && !strcmp(NN_LABELS[nnClass], "pomeha") && nnP < p.nnClear;
#else
    return false;
#endif
  }

  void decide() {
    bool nnOk = true;
    if (p.nnMode == NN_GATE && haveModel()) {
      if (nnP >= p.nnThreshold) nnOk_ = true;
      else if (nnP >= 0 && nnP <= p.nnClear) nnOk_ = false;
      nnOk = nnOk_;
    }
    bool on = rulesPresent && nnOk;
    if (on && !alarmOn) alarms++;
    alarmOn = on;
  }

  // Отсчёт окна сети 10 раз в секунду (кадры радара идут чаще или реже — берём по времени)
  void sample(const ld2410::Frame &f, uint32_t now) {
    sampled_ = false;
    if (lastSample_ && now - lastSample_ < 95) return;
    if (lastSample_ && now - lastSample_ > 500) { ringFill_ = 0; sinceRun_ = 0; }   // разрыв — окно заново
    lastSample_ = now;
    sampled_ = true;
#if CAPSULE_HAVE_MODEL
    for (int g = 0; g < 9; g++) { ring_[ringPos_][g] = f.mv[g]; ring_[ringPos_][9 + g] = f.st[g]; }
    ringPos_ = (ringPos_ + 1) % NN_WIN;
    if (ringFill_ < NN_WIN) ringFill_++;
    if (p.nnMode == NN_OFF || ringFill_ < NN_WIN || ++sinceRun_ < NN_STEP) return;
    sinceRun_ = 0;
    for (int n = 0; n < NN_WIN; n++) memcpy(win_[n], ring_[(ringPos_ + n) % NN_WIN], sizeof win_[n]);
    nn::features(win_, feat_);
    float prob[NN_CLASSES];
    nnClass = nn::predict(feat_, prob);
    float pp = 0;
    for (int j = 0; j < NN_CLASSES; j++) if (NN_IS_PERSON[j]) pp += prob[j];
    nnP = pp;
    nnRuns++;
#else
    (void)f;
#endif
  }
};

}  // namespace capsule
