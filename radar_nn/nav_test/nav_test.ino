// Проверка и настройка GPS GY-NEO6MV2 и компаса GY-27 на ESP32-S3.
// Рядом со скетчем: nav_core.h, nav_hw.h. Монитор порта 115200, «Новая строка» (NL).
//
// Подключение:
//   GY-NEO6MV2: VCC -> 5V, GND -> GND, TX -> GPIO16, RX -> GPIO15
//   GY-27:      VCC -> 3V3, GND -> GND, SDA -> GPIO8, SCL -> GPIO9
//
// ПОРЯДОК НАСТРОЙКИ (один раз, настройки сохраняются в ESP):
//  1. Плата лежит ровно компонентами вверх: az ≈ +1 g (иначе a3), наклон и крен ≈ 0.
//     Поднимите край со стрелкой X — НАКЛОН должен стать положительным (иначе a1).
//     Опустите правый край (если смотреть по стрелке X) — КРЕН положительный (иначе a2).
//  2. Калибровка: команда c (или удержание BOOT 2 с) и 30 с вращения во все стороны.
//     Делать в окончательной сборке — с аккумулятором, радаром и проводами на своих местах.
//  3. Медленно поворачивайте плату по часовой стрелке (вид сверху) — КУРС должен расти
//     0 → 90 → 180 → 270. Если убывает — m2.
//  4. Направьте стрелку X на север по компасу телефона — магн. курс X ≈ 0.
//     Ошибка ~180° — m1 и m2; ошибка ~90° — ms, затем снова шаг 2.
//  5. Не поворачивая по горизонтали, наклоните плату на 20–30° вперёд и вбок. Курс должен
//     меняться не больше чем на 5–10°. Если «уплывает» сильно — m3.
//  6. Склонение для своего места: d11.5 (проверить: magnetic-declination.com).
//  7. Если радар смотрит не туда же, куда стрелка X компаса — o<угол> (по часовой).

#include "nav_hw.h"

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n=== Проверка GPS и компаса ===");
  nav::begin();
  nav::i2cScan();
  nav::printSettings();
}

void loop() {
  nav::poll();

  static String in;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (in.length()) { if (!nav::command(in)) Serial.println("Неизвестная команда, i — список"); in = ""; } }
    else in += c;
  }

  static uint32_t last = 0;
  if (millis() - last < 500) return;
  last = millis();
  const nav::State &s = nav::st;

  // GPS
  if (!s.gpsAlive)
    Serial.print("GPS: НЕТ ДАННЫХ (TX модуля -> GPIO16?)");
  else if (!s.gpsFresh)
    Serial.printf("GPS: поиск спутников, видно %d (нужно небо над головой, холодный старт до 1–5 мин)", s.gps.sats);
  else
    Serial.printf("GPS: %.6f, %.6f  выс %.0f м  спутн %d  HDOP %.1f  %s UTC",
                  s.gps.lat, s.gps.lon, s.gps.alt, s.gps.sats, s.gps.hdop, s.gps.time);
  if (s.gps.badChecksum) Serial.printf("  [битых строк %lu]", s.gps.badChecksum);
  Serial.println();

  // Компас
  if (s.magChip == nav::MAG_NONE) { Serial.println("Компас: магнитометр НЕ НАЙДЕН (команда scan — список I2C)\n"); return; }
  float raw[3] = {0, 0, 0};
  nav::readMagBoard(raw);
  Serial.printf("Компас: курс радара %5.1f° (%s), магн. курс X %5.1f°",
                s.heading, nav::compassName(s.heading), s.magHeading);
  if (s.accOk) Serial.printf(" | наклон %+5.1f° крен %+5.1f° | az %+.2f g", s.pitch, s.roll, nav::accF[2]);
  else Serial.print(" | акселерометра нет: наклон не компенсируется");
  if (!s.calibrated) Serial.print(" | НЕ ОТКАЛИБРОВАН");
  else if (s.magDisturbed) Serial.printf(" | МАГНИТНАЯ ПОМЕХА (поле %.0f%% от нормы)", s.magNorm * 100);
  Serial.printf("\n        сырые оси платы: X %6.0f Y %6.0f Z %6.0f", raw[0], raw[1], raw[2]);
  if (fabsf(raw[0]) > 30000 || fabsf(raw[1]) > 30000 || fabsf(raw[2]) > 30000)
    Serial.print("  <- близко к пределу: рядом магнит, железо или динамик?");
  Serial.println("\n");
}
