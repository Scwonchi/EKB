// Разбор кадров радара HLK-LD2410C (24 ГГц, присутствие человека, 9 зон дальности по 0,75 м).
// Не зависит от Arduino — проверяется на компьютере (tools/capsule_sim.cpp).
//
// Кадр данных: F4 F3 F2 F1 | длина (2 байта LE) | тип | AA | данные | 55 00 | F8 F7 F6 F5
//   тип 0x02 — обычный (13 байт), 0x01 — инженерный (35 байт): ещё и энергии по зонам.
// Данные: состояние (0 — никого, 1 — движение, 2 — неподвижная цель, 3 — оба), дистанция и энергия
// движущейся цели, дистанция и энергия неподвижной, дистанция обнаружения; в инженерном —
// номера последних зон и энергии движения и неподвижности по зонам 0..8 (0..100).
#pragma once
#include <stdint.h>
#include <string.h>

namespace ld2410 {

static const int GATES = 9;
static const float GATE_M = 0.75f;       // ширина зоны по умолчанию

struct Frame {
  bool     engineering = false;
  uint8_t  state = 0;                    // 0 — никого, 1 — движение, 2 — неподвижный, 3 — оба
  uint16_t movDistCm = 0, statDistCm = 0, detDistCm = 0;
  uint8_t  movEnergy = 0, statEnergy = 0;
  uint8_t  mv[GATES] = {0}, st[GATES] = {0};   // энергии по зонам (только инженерный)
};

class Parser {
 public:
  Frame f;
  uint32_t frames = 0, badFrames = 0, basicFrames = 0;

  // true — собран новый кадр (в f)
  bool feed(uint8_t b) {
    static const uint8_t HDR[4] = {0xF4, 0xF3, 0xF2, 0xF1};
    if (pos_ < 4) {
      if (b == HDR[pos_]) buf_[pos_++] = b;
      else if (b == HDR[0]) { buf_[0] = b; pos_ = 1; }
      else pos_ = 0;
      return false;
    }
    buf_[pos_++] = b;
    if (pos_ == 6) {
      uint16_t len = buf_[4] | (buf_[5] << 8);
      if (len < 13 || len > 50) { pos_ = 0; badFrames++; return false; }
      need_ = 6 + len + 4;
    }
    if (pos_ < 6 || pos_ < need_) return false;
    pos_ = 0;
    if (buf_[need_ - 4] != 0xF8 || buf_[need_ - 1] != 0xF5 || buf_[7] != 0xAA) { badFrames++; return false; }
    return parse();
  }

 private:
  uint8_t buf_[64];
  int pos_ = 0, need_ = 0;

  bool parse() {
    Frame n;
    n.engineering = buf_[6] == 0x01;
    n.state = buf_[8];
    n.movDistCm = buf_[9] | (buf_[10] << 8);
    n.movEnergy = buf_[11];
    n.statDistCm = buf_[12] | (buf_[13] << 8);
    n.statEnergy = buf_[14];
    n.detDistCm = buf_[15] | (buf_[16] << 8);
    if (n.engineering) {
      int nM = buf_[17] + 1, nS = buf_[18] + 1;
      if (nM > GATES) nM = GATES;
      if (nS > GATES) nS = GATES;
      if (19 + nM + nS > need_ - 4) { badFrames++; return false; }
      for (int g = 0; g < nM; g++) n.mv[g] = buf_[19 + g];
      for (int g = 0; g < nS; g++) n.st[g] = buf_[19 + nM + g];
    } else basicFrames++;
    f = n;
    frames++;
    return true;
  }
};

// Кадр в байты — для имитатора и проверок (тот же формат, что шлёт радар в инженерном режиме)
inline int encode(const Frame &f, uint8_t *out) {
  int i = 0;
  const uint8_t h[4] = {0xF4, 0xF3, 0xF2, 0xF1}, t[4] = {0xF8, 0xF7, 0xF6, 0xF5};
  memcpy(out, h, 4); i = 4;
  int lenPos = i; i += 2;
  out[i++] = f.engineering ? 0x01 : 0x02;
  out[i++] = 0xAA;
  out[i++] = f.state;
  out[i++] = f.movDistCm & 0xFF; out[i++] = f.movDistCm >> 8; out[i++] = f.movEnergy;
  out[i++] = f.statDistCm & 0xFF; out[i++] = f.statDistCm >> 8; out[i++] = f.statEnergy;
  out[i++] = f.detDistCm & 0xFF; out[i++] = f.detDistCm >> 8;
  if (f.engineering) {
    out[i++] = GATES - 1; out[i++] = GATES - 1;
    for (int g = 0; g < GATES; g++) out[i++] = f.mv[g];
    for (int g = 0; g < GATES; g++) out[i++] = f.st[g];
    out[i++] = 0; out[i++] = 0;                       // освещённость, выход OUT — не используются
  }
  out[i++] = 0x55; out[i++] = 0x00;
  uint16_t len = i - 6;
  out[lenPos] = len & 0xFF; out[lenPos + 1] = len >> 8;
  memcpy(out + i, t, 4); i += 4;
  return i;
}

// Команда: FD FC FB FA | длина | команда | значение | 04 03 02 01
template <typename Writer>
inline void sendCommand(Writer write, uint16_t cmd, const uint8_t *val = nullptr, uint8_t vlen = 0) {
  uint8_t f[32]; int i = 0;
  uint16_t len = 2 + vlen;
  f[i++] = 0xFD; f[i++] = 0xFC; f[i++] = 0xFB; f[i++] = 0xFA;
  f[i++] = len & 0xFF; f[i++] = len >> 8;
  f[i++] = cmd & 0xFF; f[i++] = cmd >> 8;
  for (int k = 0; k < vlen; k++) f[i++] = val[k];
  f[i++] = 0x04; f[i++] = 0x03; f[i++] = 0x02; f[i++] = 0x01;
  write(f, i);
}

// Включить инженерный режим (энергии по зонам — вход правил и нейросети)
template <typename Writer, typename Delay>
inline void enableEngineering(Writer write, Delay delayMs) {
  const uint8_t on[2] = {0x01, 0x00};
  sendCommand(write, 0x00FF, on, 2); delayMs(100);    // войти в настройку
  sendCommand(write, 0x0062);        delayMs(100);    // инженерный режим
  sendCommand(write, 0x00FE);        delayMs(100);    // выйти из настройки
}

}  // namespace ld2410
