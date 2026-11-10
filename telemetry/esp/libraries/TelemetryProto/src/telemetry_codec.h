// Кадры protobuf в потоке TCP: [длина тела — 4 байта, uint32 BigEndian][сообщение].
// Формат сервера iot-go (internal/transport/iot/server.go, readFrame). Кадры в обе стороны одинаковые.
// Подключение: устройство -> Syn (MAC, оборудование), сервер -> Ack (code 0 — принят),
// дальше устройство -> Telemetry, Telemetry, ...
// Проверочные векторы:
//   Telemetry{uptime_s: 1}            -> 00 00 00 02 10 01
//   Syn{mac: 02:11:22:33:44:55}       -> 00 00 00 08 0A 06 02 11 22 33 44 55
//   Ack{code: 0} -> 00 00 00 00,  Ack{code: 1} -> 00 00 00 02 08 01
// Не зависит от Arduino — проверяется и на компьютере.
//
// Снимок с 6 целями — до 279 байт тела (telemetry_v1_Telemetry_size).
#pragma once
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "telemetry/v1/telemetry.pb.h"
#include "telemetry/v1/synack.pb.h"

namespace telemetry {

static const size_t HEADER = 4;
static const size_t MAX_SNAPSHOT = telemetry_v1_Telemetry_size + HEADER;   // наибольший кадр снимка
static const size_t MAX_FRAME = 64 * 1024;       // наибольшее тело для FrameReader (как у iot-go)
static const size_t MAX_SYN = telemetry_v1_Syn_size + HEADER;
static const size_t MAX_ACK = telemetry_v1_Ack_size + HEADER;

// ---------- Приведение значений к ограничениям сервера ----------
static inline bool finite_(float v) { return !isnan(v) && !isinf(v); }
static inline float clamp01_(float v) { return !finite_(v) || v < 0 ? 0 : (v > 1 ? 1 : v); }
// Угол в [0, 360); false — не число
static inline bool wrap360_(float &h) {
  if (!finite_(h)) return false;
  if (h >= 0 && h < 360) return true;
  h = fmodf(h, 360.0f);
  if (h < 0) h += 360.0f;
  if (h >= 360.0f) h = 0;                               // -1e-7 + 360 округляется до 360
  return true;
}

// Приводит снимок к ограничениям сервера (domain.Telemetry.Validate в iot-go).
// Сервер закрывает подключение на первом же неверном снимке, поэтому одна NaN от датчика без этой проверки дала бы
// бесконечные переподключения. Возвращает число исправленных полей.
inline int sanitize(telemetry_v1_Telemetry &t) {
  int fixed = 0;
  auto fix = [&](bool bad) { if (bad) fixed++; return bad; };
  auto angle = [&](float &h) { float o = h; bool ok = wrap360_(h); if (!ok || o != h) fixed++; return ok; };

  float p = clamp01_(t.person_probability);
  if (fix(p != t.person_probability)) t.person_probability = p;

  if (t.has_position && fix(isnan(t.position.lat) || isinf(t.position.lat) || t.position.lat < -90 || t.position.lat > 90 ||
                            isnan(t.position.lon) || isinf(t.position.lon) || t.position.lon < -180 || t.position.lon > 180))
    t.has_position = false;                               // нет достоверных координат = нет фиксации

  if (t.has_hdop && fix(!finite_(t.hdop) || t.hdop < 0)) t.has_hdop = false;

  if (t.has_heading_deg && !angle(t.heading_deg)) t.has_heading_deg = false;
  if (t.heading_valid && fix(!t.has_heading_deg)) t.heading_valid = false;

  if (fix(t.tracks_count > 6)) t.tracks_count = 6;
  pb_size_t k = 0;
  for (pb_size_t i = 0; i < t.tracks_count; i++) {
    telemetry_v1_Track o = t.tracks[i];
    if (fix(!finite_(o.distance_m) || !finite_(o.angle_deg))) continue;   // цель без положения — выбросить
    if (fix(o.distance_m < 0)) o.distance_m = -o.distance_m;
    float v = finite_(o.speed_mps) ? fabsf(o.speed_mps) : 0;
    if (fix(v != o.speed_mps)) o.speed_mps = v;
    float h = clamp01_(o.hit_ratio);
    if (fix(h != o.hit_ratio)) o.hit_ratio = h;
    t.tracks[k++] = o;
  }
  t.tracks_count = k;

  return fixed;
}

// ---------- Кодирование в буфер ----------
inline size_t encodeFrame(const pb_msgdesc_t *fields, const void *msg, uint8_t *buf, size_t cap) {
  if (cap < HEADER) return 0;
  pb_ostream_t os = pb_ostream_from_buffer(buf + HEADER, cap - HEADER);
  if (!pb_encode(&os, fields, msg)) return 0;
  uint32_t n = (uint32_t)os.bytes_written;
  buf[0] = n >> 24; buf[1] = n >> 16; buf[2] = n >> 8; buf[3] = n;
  return HEADER + n;
}

// Кодирует снимок в кадр, предварительно приведя его к ограничениям сервера.
inline size_t encodeTelemetry(const telemetry_v1_Telemetry &m, uint8_t *buf, size_t cap, int *fixed = nullptr) {
  telemetry_v1_Telemetry t = m;
  int f = sanitize(t);
  if (fixed) *fixed = f;
  return encodeFrame(telemetry_v1_Telemetry_fields, &t, buf, cap);
}

// ---------- Syn: первый кадр каждого подключения ----------
inline void synInit(telemetry_v1_Syn &s, const uint8_t mac[6]) {
  s = telemetry_v1_Syn_init_zero;
  memcpy(s.mac, mac, 6);                         // 6 сырых байт, не строка «02:11:...»
}
// Добавить оборудование. false — список полон или name/url не помещаются (тогда не добавляется)
inline bool synAddOnboard(telemetry_v1_Syn &s, const char *name, const char *url) {
  if (s.onboards_count >= sizeof(s.onboards) / sizeof(s.onboards[0])) return false;
  telemetry_v1_Syn_OnboardDevice &o = s.onboards[s.onboards_count];
  if (strlen(name) >= sizeof(o.name) || strlen(url) >= sizeof(o.url)) return false;
  strcpy(o.name, name);
  strcpy(o.url, url);
  s.onboards_count++;
  return true;
}
inline size_t encodeSyn(const telemetry_v1_Syn &s, uint8_t *buf, size_t cap) {
  return encodeFrame(telemetry_v1_Syn_fields, &s, buf, cap);
}
// Тело кадра -> Ack. false — не разбирается
inline bool decodeAck(const uint8_t *body, size_t n, telemetry_v1_Ack &a) {
  a = telemetry_v1_Ack_init_zero;
  pb_istream_t is = pb_istream_from_buffer(body, n);
  return pb_decode(&is, telemetry_v1_Ack_fields, &a);
}

// ---------- Кодирование прямо в поток (без промежуточного буфера) ----------
// sink(ctx, data, n) должен записать все n байт или вернуть false.
typedef bool (*Sink)(void *ctx, const uint8_t *data, size_t n);

struct BufferedSink_ {
  Sink sink; void *ctx;
  uint8_t buf[1024];
  size_t n = 0;
  bool ok = true;
  bool put(const uint8_t *p, size_t len) {
    while (len && ok) {
      size_t k = sizeof(buf) - n < len ? sizeof(buf) - n : len;
      memcpy(buf + n, p, k); n += k; p += k; len -= k;
      if (n == sizeof(buf)) flush();
    }
    return ok;
  }
  void flush() { if (n && ok) ok = sink(ctx, buf, n); n = 0; }
};

inline bool streamWrite_(pb_ostream_t *os, const pb_byte_t *b, size_t n) {
  return ((BufferedSink_ *)os->state)->put(b, n);
}

// Кодирует (после sanitize) и пишет кадр в sink кусками по 1 КБ.
// t изменяется (sanitize). Возвращает размер кадра или 0 при ошибке.
inline size_t writeTelemetry(telemetry_v1_Telemetry &t, Sink sink, void *ctx, int *fixed = nullptr) {
  int f = sanitize(t);
  if (fixed) *fixed = f;
  size_t body = 0;
  if (!pb_get_encoded_size(&body, telemetry_v1_Telemetry_fields, &t) || body > MAX_FRAME) return 0;
  BufferedSink_ bs;
  bs.sink = sink; bs.ctx = ctx;
  uint8_t h[4] = {(uint8_t)(body >> 24), (uint8_t)(body >> 16), (uint8_t)(body >> 8), (uint8_t)body};
  bs.put(h, 4);
  pb_ostream_t os;
  os.callback = &streamWrite_;
  os.state = &bs;
  os.max_size = SIZE_MAX;
  os.bytes_written = 0;
#ifndef PB_NO_ERRMSG
  os.errmsg = nullptr;
#endif
  if (!pb_encode(&os, telemetry_v1_Telemetry_fields, &t)) return 0;
  bs.flush();
  return bs.ok ? HEADER + body : 0;
}

// ---------- Сборка кадров из потока байт (приёмник на C/C++, проверки на ПК) ----------
// Подаём байты по одному; когда кадр собран, feed() возвращает FRAME_READY.
class FrameReader {
 public:
  enum Result { NEED_MORE, FRAME_READY, ERROR };

  explicit FrameReader(size_t cap = MAX_FRAME) : cap_(cap) { buf_ = (uint8_t *)malloc(cap); }
  ~FrameReader() { free(buf_); }
  FrameReader(const FrameReader &) = delete;
  FrameReader &operator=(const FrameReader &) = delete;

  Result feed(uint8_t b) {
    if (ready_) reset();
    if (hdrGot_ < HEADER) {
      len_ = (len_ << 8) | b;
      if (++hdrGot_ < HEADER) return NEED_MORE;
      if (len_ > cap_ || !buf_) { reset(); errors++; return ERROR; }
      got_ = 0;
      if (len_ == 0) { ready_ = true; return FRAME_READY; }   // пустое сообщение — допустимо
      return NEED_MORE;
    }
    buf_[got_++] = b;
    if (got_ == len_) { ready_ = true; return FRAME_READY; }
    return NEED_MORE;
  }

  bool decode(const pb_msgdesc_t *fields, void *msg) const {
    if (!ready_) return false;
    pb_istream_t is = pb_istream_from_buffer(buf_, len_);
    return pb_decode(&is, fields, msg);
  }

  bool decodeTelemetry(telemetry_v1_Telemetry &m) const {
    m = telemetry_v1_Telemetry_init_zero;
    return decode(telemetry_v1_Telemetry_fields, &m);
  }

  bool decodeAck(telemetry_v1_Ack &a) const { return ready_ && telemetry::decodeAck(buf_, len_, a); }

  size_t frameLength() const { return len_; }
  void reset() { hdrGot_ = 0; len_ = 0; got_ = 0; ready_ = false; }

  uint32_t errors = 0;

 private:
  uint8_t *buf_ = nullptr;
  size_t   cap_;
  bool     ready_ = false;
  uint32_t len_ = 0, got_ = 0, hdrGot_ = 0;
};

}  // namespace telemetry
