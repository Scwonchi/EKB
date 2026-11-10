// Логика разведчика без привязки к железу: радар -> трекер -> сканирование -> карта
// обнаружения -> тревога, журнал событий, снимок telemetry.v1.Telemetry (поля 1–16).
// Скетч (rescue_scout.ino) только подаёт данные датчиков и отправляет результат;
// на компьютере тот же код гоняет имитатор tools/scout_sim.cpp.
#pragma once
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include "ld2450.h"
#include "tracker.h"
#include "scan.h"
#include "telemetry/v1/telemetry.pb.h"

namespace mission {

// События миссии — журнал на устройстве (монитор порта, команда log). На сервер уходит
// только снимок telemetry.v1.Telemetry (поля 1–16), событий в нём нет.
enum Ev : uint8_t {
  EV_BOOT = 1, EV_SEPARATED, EV_ZONE_ENTERED, EV_SCAN_STARTED, EV_STOP_STARTED, EV_PERSON_DETECTED,
  EV_ALARM_RAISED, EV_ALARM_SENT, EV_ALARM_CLEARED, EV_SCAN_COMPLETED
};

inline const char *eventName(Ev t) {
  switch (t) {
    case EV_BOOT: return "включение";
    case EV_SEPARATED: return "отделение от носителя";
    case EV_ZONE_ENTERED: return "вход в зону";
    case EV_SCAN_STARTED: return "начало сканирования";
    case EV_STOP_STARTED: return "остановка";
    case EV_PERSON_DETECTED: return "обнаружен человек";
    case EV_ALARM_RAISED: return "тревога";
    case EV_ALARM_SENT: return "тревога передана";
    case EV_ALARM_CLEARED: return "отбой тревоги";
    case EV_SCAN_COMPLETED: return "сканирование завершено";
    default: return "?";
  }
}

struct Event { uint32_t seq, ms; Ev type; uint32_t ref; float value; };

class Mission {
 public:
  static const int LOG_MAX = 64;

  trk::Tracker tracker;
  scan::Scanner scanner;
  int8_t   angleSign = 1;
  uint32_t firmwareVersion = 4;
  void   (*print)(const char *) = nullptr;   // вывод сообщений (Serial или printf)

  bool     alarmOn = false;
  float    absStartHeading = NAN;     // азимут от севера в начале сканирования (если известен)
  Event    evlog[LOG_MAX];
  uint32_t evSeq = 0;

  // ----- Команды -----
  void boot(uint32_t now) { addEvent(now, EV_BOOT, firmwareVersion); }
  void mark(uint32_t now, Ev e, uint32_t ref = 0, float v = 0) { addEvent(now, e, ref, v); }

  // heading — курс для остановок (гироскоп или компас; NAN — нет ни того, ни другого),
  // absHeading — азимут от севера, если ему можно верить (иначе NAN)
  void start(uint32_t now, float heading, float absHeading) {
    tracker.clearTracks();
    scanner.start(now, heading);
    absStartHeading = absHeading;
    addEvent(now, EV_SCAN_STARTED, 0, isnan(absHeading) ? 0 : absHeading);
    if (isnan(scanner.startHeading())) out("Нет ни гироскопа, ни компаса: одна остановка по оси радара, направления — только от оси");
  }
  void finish(uint32_t now) {
    if (scanner.phase() != scan::SCANNING) return;
    scanner.finish(now);
    completed(now);
  }
  void reset(uint32_t now) { scanner.reset(); clearAlarm(now); }

  // ----- Данные -----
  float targetAngle(const trk::Track &t) const { return angleSign * atan2f(-t.x, t.y) * 57.29578f; }

  // Кадр радара. absHeading — достоверный курс от севера или NAN.
  void radarFrame(const ld2450::Target *t, int n, uint32_t now, float absHeading) {
    // В повороте данные радара размазаны — не используем. В ожидании — только показываем цели.
    if (scanner.phase() == scan::SCANNING && !scanner.listening()) return;
    tracker.update(t, n, now);
    if (scanner.listening() && !tracker.learning()) observe(now, absHeading);
  }

  void radarLost() { tracker.clearTracks(); }

  // Каждый цикл: курс для остановок; когда снимок с тревогой записан в подключение (millis)
  void tick(uint32_t now, float scanHeading, uint32_t alarmSentMs) {
    scan::Step st = scanner.update(now, scanHeading);
    if (st.stopEnded || st.stopStarted) tracker.clearTracks();          // новый ракурс — треки заново
    if (st.stopStarted) addEvent(now, EV_STOP_STARTED, scanner.stops(), scanner.relHeading());
    if (st.completed) completed(now);

    // Тревога держится, пока на карте есть люди: оператор должен подтвердить каждого.
    // Снимается только сбросом миссии (reset) — переход в ожидание.
    if (scanner.phase() == scan::STANDBY) clearAlarm(now);
    if (alarmOn && !alarmSentLogged_ && alarmSentMs && (int32_t)(alarmSentMs - alarmRaisedMs_) >= 0) {
      alarmSentLogged_ = true;
      addEvent(now, EV_ALARM_SENT);
    }
  }

  // ----- Снимок состояния для сервера (поля связи — rssi — заполняет скетч) -----
  void fill(telemetry_v1_Telemetry &t, uint32_t now, bool radarOk, float absHeading) {
    const trk::Track *b = tracker.best();
    t = telemetry_v1_Telemetry_init_zero;
    t.firmware_version = firmwareVersion;
    t.uptime_s = now / 1000;
    t.radar_ok = radarOk;
    // Вероятность «человек» — от нейросети, если она включена; без сети — доля кадров с точкой
    if (b) { t.detection_class = cls(b->kind == trk::KIND_MOVING); t.person_probability = b->nnProb >= 0 ? b->nnProb : b->hitRatio(); }
    else if (alarmOn && scanner.nDet) { t.detection_class = cls(scanner.det[0].kind == scan::MOVING); t.person_probability = 0.9f; }
    else t.detection_class = telemetry_v1_DetectionClass_DETECTION_CLASS_EMPTY;
    t.heading_valid = !isnan(absHeading);
    if (t.heading_valid) { t.has_heading_deg = true; t.heading_deg = absHeading; }
    t.target_count = tracker.learning() ? 0 : tracker.personCount();
    if (!tracker.learning())
      for (auto &tr : tracker.tracks) {
        if (!tr.used || t.tracks_count >= 6) continue;
        telemetry_v1_Track &o = t.tracks[t.tracks_count++];
        o.id = tr.id;
        o.distance_m = tr.range();
        o.angle_deg = targetAngle(tr);
        o.speed_mps = tr.speed;
        o.confirmed = tr.confirmed;
        o.detection_class = tr.confirmed ? cls(tr.kind == trk::KIND_MOVING) : telemetry_v1_DetectionClass_DETECTION_CLASS_UNSPECIFIED;
        o.hit_ratio = tr.hitRatio();
      }
    t.learning_background = tracker.learning();
    t.background_cells = tracker.backgroundCells();
    t.alarm_active = alarmOn;
  }

  // ----- Вывод -----
  void printEvent(const Event &e) const {
    char b[160];
    int n = snprintf(b, sizeof b, "[%7.3f с] событие %lu: %s", e.ms / 1000.0, (unsigned long)e.seq, eventName(e.type));
    if (e.ref) n += snprintf(b + n, sizeof b - n, ", №%lu", (unsigned long)e.ref);
    if (e.value != 0) snprintf(b + n, sizeof b - n, ", %.1f", e.value);
    out(b);
  }
  void printLog() const {
    uint32_t first = evSeq > LOG_MAX ? evSeq - LOG_MAX + 1 : 1;
    for (uint32_t s = first; s <= evSeq; s++) printEvent(evlog[(s - 1) % LOG_MAX]);
  }
  void printMap() const {
    char b[200];
    if (!scanner.nDet) { out("    карта обнаружения пуста"); return; }
    for (int i = 0; i < scanner.nDet; i++) {
      const scan::Detection &d = scanner.det[i];
      char az[24] = "";
      if (!isnan(d.absBearing)) snprintf(az, sizeof az, ", азимут %.0f°", d.absBearing);
      snprintf(b, sizeof b, "    №%lu: %.1f м, %.0f° от начала%s, %s, остановка %lu, подтверждён на %lu остановк.",
               (unsigned long)d.id, d.distance, d.relBearing, az, d.kind == scan::MOVING ? "движется" : "неподвижен",
               (unsigned long)d.stopIndex, (unsigned long)d.seen);
      out(b);
    }
  }

 private:
  bool     alarmSentLogged_ = false;
  uint32_t alarmRaisedMs_ = 0;

  static telemetry_v1_DetectionClass cls(bool moving) {
    return moving ? telemetry_v1_DetectionClass_DETECTION_CLASS_MOVING : telemetry_v1_DetectionClass_DETECTION_CLASS_STATIONARY;
  }

  void out(const char *s) const { if (print) print(s); }

  void addEvent(uint32_t now, Ev type, uint32_t ref = 0, float value = 0) {
    Event &e = evlog[evSeq % LOG_MAX];
    e = {++evSeq, now, type, ref, value};
    printEvent(e);
  }

  void completed(uint32_t now) {
    addEvent(now, EV_SCAN_COMPLETED, scanner.nDet, scanner.coveredDeg());
    printMap();
  }

  void raiseAlarm(uint32_t now, int k) {
    if (alarmOn) return;
    alarmOn = true; alarmSentLogged_ = false; alarmRaisedMs_ = now;
    addEvent(now, EV_ALARM_RAISED, k >= 0 ? scanner.det[k].id : 0);
  }

  void clearAlarm(uint32_t now) {
    if (!alarmOn) return;
    alarmOn = false;
    addEvent(now, EV_ALARM_CLEARED);
  }

  // Подтверждённые цели -> карта обнаружения, тревога
  void observe(uint32_t now, float absHeading) {
    for (auto &tr : tracker.tracks) {
      if (!tr.used || !tr.confirmed) continue;
      float ang = targetAngle(tr);
      bool isNew;
      int k = scanner.observe(now, tr.id, tr.range(), ang,
                              tr.kind == trk::KIND_MOVING ? scan::MOVING : scan::STATIONARY, absHeading, isNew);
      if (k < 0) continue;
      scan::Detection &d = scanner.det[k];
      if (isNew) {
        addEvent(now, EV_PERSON_DETECTED, d.id, d.distance);
        char b[160];
        snprintf(b, sizeof b, ">>> Человек №%lu: %.1f м, %+.0f° от оси, %.0f° от начала сканирования, %s",
                 (unsigned long)d.id, d.distance, ang, d.relBearing, tr.kind == trk::KIND_MOVING ? "движется" : "неподвижен");
        out(b);
        raiseAlarm(now, k);
      }
    }
  }
};

}  // namespace mission
