// Протокол шлюз ↔ ПК по USB (последовательный порт). Не зависит от Arduino.
//
// Кадр: A5 | тип | клиент | длина (2 байта, младший первым) | данные | CRC-8
//   CRC-8 (полином 0x07) считается по байтам от «тип» до конца данных.
//
// Шлюз -> ПК:  CONNECT (данные: IPv4 клиента, 4 байта), DATA (байты от клиента),
//              DISCONNECT, STATUS (данные: число клиентов, 1 байт; время работы, с, 4 байта; станций Wi-Fi, 1 байт)
// ПК -> шлюз:  DATA (байты клиенту), DISCONNECT (закрыть клиента), RESET (закрыть всех)
//
// Шлюз не разбирает protobuf: он только пересылает байтовые потоки клиентов,
// а кадры [4 байта длины][сообщение] собирает уже сервер.
#pragma once
#include <stdint.h>
#include <stddef.h>

namespace gw {

enum Type : uint8_t { CONNECT = 1, DATA = 2, DISCONNECT = 3, STATUS = 4, RESET = 5 };
static const uint8_t SYNC = 0xA5;
static const uint16_t MAX_PAYLOAD = 1024;

inline uint8_t crc8(uint8_t crc, uint8_t b) {
  crc ^= b;
  for (int i = 0; i < 8; i++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  return crc;
}

// Записывает кадр в out (нужно len + 6 байт), возвращает размер
inline size_t encode(uint8_t type, uint8_t client, const uint8_t *data, uint16_t len, uint8_t *out) {
  size_t i = 0;
  out[i++] = SYNC;
  out[i++] = type; out[i++] = client;
  out[i++] = len & 0xFF; out[i++] = len >> 8;
  for (uint16_t k = 0; k < len; k++) out[i++] = data[k];
  uint8_t c = 0;
  for (size_t k = 1; k < i; k++) c = crc8(c, out[k]);
  out[i++] = c;
  return i;
}

class Parser {
 public:
  uint8_t type = 0, client = 0;
  uint16_t len = 0;
  uint8_t data[MAX_PAYLOAD];
  uint32_t errors = 0;

  // Сбросить недособранный кадр (например, после паузы в потоке)
  void reset() { st_ = 0; }
  bool idle() const { return st_ == 0; }

  // true — кадр собран (type, client, data, len)
  bool feed(uint8_t b) {
    switch (st_) {
      case 0: if (b == SYNC) { st_ = 1; crc_ = 0; } return false;
      case 1: type = b; crc_ = crc8(crc_, b); st_ = 2; return false;
      case 2: client = b; crc_ = crc8(crc_, b); st_ = 3; return false;
      case 3: len = b; crc_ = crc8(crc_, b); st_ = 4; return false;
      case 4:
        len |= (uint16_t)b << 8; crc_ = crc8(crc_, b);
        if (len > MAX_PAYLOAD) { errors++; st_ = 0; return false; }
        got_ = 0; st_ = len ? 5 : 6; return false;
      case 5:
        data[got_++] = b; crc_ = crc8(crc_, b);
        if (got_ == len) st_ = 6;
        return false;
      default:
        st_ = 0;
        if (b != crc_) { errors++; return false; }
        return true;
    }
  }

 private:
  int st_ = 0;
  uint16_t got_ = 0;
  uint8_t crc_ = 0;
};

}  // namespace gw
