// Донесение об обнаружении. Вынесено в отдельный файл: Arduino IDE вставляет
// объявления функций в начало скетча, и тип должен быть известен уже там.
#pragma once
#include <stdint.h>

struct __attribute__((packed)) Report {
  uint8_t  type;          // 1 — тревога, 2 — отбой
  uint16_t alarmId;
  uint8_t  cls;           // номер класса сети
  uint8_t  confidence;    // %
  uint16_t distCm;        // 0 — неизвестна
  int32_t  droneLat, droneLon;   // градусы * 1e7
  uint8_t  sats;
  uint8_t  hdop10;        // HDOP * 10
  uint16_t heading10;     // азимут радара * 10, 0xFFFF — неизвестен
  int32_t  tgtLat, tgtLon;       // оценка точки цели, градусы * 1e7
  uint8_t  flags;         // бит0 GPS, бит1 курс, бит2 точка цели, бит3 магнитная помеха
};
