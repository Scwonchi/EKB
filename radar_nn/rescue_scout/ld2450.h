// Разбор кадров радара HLK-LD2450 (24 ГГц, до 3 целей с координатами).
// Не зависит от Arduino — проверяется на компьютере.
//
// Кадр данных: AA FF 03 00 | 3 цели по 8 байт | 55 CC  (30 байт, ~10 раз в секунду)
// Цель: X (мм), Y (мм), скорость (см/с), разрешение (мм). У X, Y и скорости
// старший бит — знак: 1 — плюс, 0 — минус. Пустой слот — все нули.
#pragma once
#include <stdint.h>
#include <string.h>

namespace ld2450 {

struct Target {
  float x = 0, y = 0;     // м: X — вбок, Y — вперёд от радара
  float speed = 0;        // м/с: + удаляется, − приближается
  bool  valid = false;
};

inline int16_t decodeSigned(uint8_t lo, uint8_t hi) {
  int16_t v = (int16_t)(((hi & 0x7F) << 8) | lo);
  return (hi & 0x80) ? v : -v;
}

class Parser {
 public:
  Target t[3];
  uint32_t frames = 0, badFrames = 0;

  // Возвращает true, когда собран новый кадр (цели в t[])
  bool feed(uint8_t b) {
    static const uint8_t HDR[4] = {0xAA, 0xFF, 0x03, 0x00};
    if (pos_ < 4) {
      if (b == HDR[pos_]) buf_[pos_++] = b;
      else if (b == HDR[0]) { buf_[0] = b; pos_ = 1; }
      else pos_ = 0;
      return false;
    }
    buf_[pos_++] = b;
    if (pos_ < 30) return false;
    pos_ = 0;
    if (buf_[28] != 0x55 || buf_[29] != 0xCC) { badFrames++; return false; }
    for (int i = 0; i < 3; i++) {
      const uint8_t *p = buf_ + 4 + i * 8;
      bool empty = true;
      for (int k = 0; k < 8; k++) if (p[k]) { empty = false; break; }
      t[i] = Target();
      if (empty) continue;
      t[i].x = decodeSigned(p[0], p[1]) / 1000.0f;
      t[i].y = decodeSigned(p[2], p[3]) / 1000.0f;
      t[i].speed = decodeSigned(p[4], p[5]) / 100.0f;
      t[i].valid = t[i].x != 0 || t[i].y != 0;
    }
    frames++;
    return true;
  }

 private:
  uint8_t buf_[30];
  int pos_ = 0;
};

// Команда радару: FD FC FB FA | длина | команда | значение | 04 03 02 01
// writer — функция, которая отправляет байты в порт радара
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

// Включить режим нескольких целей (по умолчанию он и так включён — на всякий случай)
template <typename Writer, typename Delay>
inline void setMultiTarget(Writer write, Delay delayMs) {
  const uint8_t on[2] = {0x01, 0x00};
  sendCommand(write, 0x00FF, on, 2);  delayMs(100);   // войти в режим настройки
  sendCommand(write, 0x0090);         delayMs(100);   // несколько целей
  sendCommand(write, 0x00FE);         delayMs(100);   // выйти из режима настройки
}

}  // namespace ld2450
