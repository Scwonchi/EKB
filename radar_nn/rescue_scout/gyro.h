// Курс по гироскопу MPU-6050 (GY-521): поворот вокруг вертикали с момента включения.
//
// Компас рядом с моторами и в здании врёт на десятки градусов; гироскопу магнитное поле
// безразлично. Для секторного сканирования нужен как раз поворот относительно начала,
// а не север, поэтому гироскоп — основной источник курса, компас — только для азимута
// от севера (если он есть и откалиброван).
//
// Вертикаль берётся по акселерометру, поэтому плату можно крепить в любом положении:
// скорость поворота = проекция вектора угловой скорости на направление «вверх».
// Смещение нуля гироскопа измеряется при включении — первые 2 с модуль должен лежать
// неподвижно (на земле, до взлёта). Уход после этого — 1–3° за минуту сканирования.
//
// Курс растёт при повороте по часовой стрелке, если смотреть сверху. Если наоборот —
// команда gs (знак запоминается).
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>

namespace gyro {

static uint8_t addr = 0;              // 0x68 или 0x69 (AD0 к 3V3); 0 — не найден
static bool ok = false, calibrated = false;
static float bias[3] = {0, 0, 0};     // смещение нуля, °/с
static float up[3] = {0, 0, 1};       // направление «вверх» в осях платы (по акселерометру)
static float heading = 0;             // курс с момента калибровки, ° [0, 360)
static float rate = 0;                // скорость поворота, °/с (+ по часовой)
static int8_t sign = -1;              // MPU: +ω вокруг «вверх» = против часовой → курс с минусом
static uint32_t lastUs = 0, errors = 0;
static const float GYRO_LSB = 65.5f;  // ±500 °/с
static const float ACC_LSB = 8192.0f; // ±4 g

inline bool wr(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(addr); Wire.write(reg); Wire.write(v);
  return Wire.endTransmission() == 0;
}

// a — ускорение, g; w — угловая скорость, °/с (без смещения)
inline bool readRaw(float a[3], float w[3]) {
  Wire.beginTransmission(addr); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, 14) != 14) return false;
  int16_t v[7];
  for (int i = 0; i < 7; i++) { int hi = Wire.read(); int lo = Wire.read(); v[i] = (int16_t)((hi << 8) | lo); }
  for (int i = 0; i < 3; i++) { a[i] = v[i] / ACC_LSB; w[i] = v[4 + i] / GYRO_LSB; }
  return true;
}

inline void loadSettings() { Preferences p; p.begin("gyro", true); sign = p.getChar("sign", -1) > 0 ? 1 : -1; p.end(); }
inline void saveSettings() { Preferences p; p.begin("gyro", false); p.putChar("sign", sign); p.end(); }

// Смещение нуля: среднее за ~1,5 с неподвижности. false — модуль двигался.
inline bool calibrate() {
  float s[3] = {0, 0, 0}, sa[3] = {0, 0, 0}, mx = 0;
  int n = 0;
  float a[3], w[3];
  float first[3] = {0, 0, 0};
  for (int i = 0; i < 300; i++) {
    if (readRaw(a, w)) {
      if (!n) for (int k = 0; k < 3; k++) first[k] = w[k];
      for (int k = 0; k < 3; k++) { s[k] += w[k]; sa[k] += a[k]; mx = fmaxf(mx, fabsf(w[k] - first[k])); }
      n++;
    }
    delay(5);
  }
  if (n < 200 || mx > 3.0f) return false;           // качали — не годится
  float g = 0;
  for (int k = 0; k < 3; k++) { bias[k] = s[k] / n; up[k] = sa[k] / n; g += up[k] * up[k]; }
  g = sqrtf(g);
  if (g < 0.5f) return false;
  for (int k = 0; k < 3; k++) up[k] /= g;
  heading = 0;
  lastUs = micros();
  calibrated = true;
  return true;
}

inline bool begin() {
  loadSettings();
  for (uint8_t a : {(uint8_t)0x68, (uint8_t)0x69}) {
    Wire.beginTransmission(a); Wire.write(0x75);
    if (Wire.endTransmission(false) != 0 || Wire.requestFrom((int)a, 1) != 1) continue;
    uint8_t who = Wire.read();
    if (who == 0x68 || who == 0x70 || who == 0x72 || who == 0x98) { addr = a; break; }   // MPU-6050/6500/клоны
  }
  if (!addr) return ok = false;
  ok = wr(0x6B, 0x01)                  // проснуться, тактирование от гироскопа X
    && wr(0x1A, 0x03)                  // фильтр 44 Гц — вибрация моторов
    && wr(0x1B, 0x08)                  // гироскоп ±500 °/с
    && wr(0x1C, 0x08)                  // акселерометр ±4 g
    && wr(0x19, 0x04);                 // 200 отсчётов/с
  delay(50);
  if (ok) {
    for (int i = 0; i < 5 && !calibrate(); i++) delay(300);
  }
  return ok;
}

// Вызывать часто (каждый loop). Интегрирует поворот вокруг вертикали.
inline void poll() {
  if (!ok || !calibrated) return;
  uint32_t now = micros();
  if (now - lastUs < 4000) return;                  // ~250 Гц хватает
  float a[3], w[3];
  if (!readRaw(a, w)) { errors++; return; }
  float dt = (now - lastUs) * 1e-6f;
  lastUs = now;
  if (dt > 0.1f) dt = 0.1f;                          // пропуск (долгий loop) — не переинтегрировать
  // Вертикаль медленно подстраивается по акселерометру (дрон наклоняется при манёврах)
  float g = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  if (g > 0.8f && g < 1.2f) {
    float n = 0;
    for (int k = 0; k < 3; k++) { up[k] += 0.02f * (a[k] / g - up[k]); n += up[k] * up[k]; }
    n = sqrtf(n);
    for (int k = 0; k < 3; k++) up[k] /= n;
  }
  float wz = 0;
  for (int k = 0; k < 3; k++) wz += (w[k] - bias[k]) * up[k];
  rate = sign * wz;
  heading = fmodf(heading + rate * dt, 360.0f);
  if (heading < 0) heading += 360.0f;
}

// Курс, °; NAN — гироскопа нет
inline float value() { return ok && calibrated ? heading : NAN; }

inline void printState() {
  if (!ok) { Serial.println("Гироскоп: MPU-6050 НЕ НАЙДЕН (SDA/SCL, питание)"); return; }
  Serial.printf("Гироскоп: MPU-6050 0x%02X, %s, смещение %.2f %.2f %.2f °/с, вверх %.2f %.2f %.2f, знак %+d, курс %.1f°\n",
                addr, calibrated ? "откалиброван" : "НЕ откалиброван (двигали при включении — команда g)",
                bias[0], bias[1], bias[2], up[0], up[1], up[2], sign, heading);
}

// Команды: g — перекалибровать (2 с неподвижно), gs — сменить знак, gi — состояние
inline bool command(const String &s) {
  if (s == "g") { Serial.println(calibrate() ? "Гироскоп откалиброван, курс = 0" : "Не вышло: модуль двигался, повторите"); return true; }
  if (s == "gs") { sign = -sign; saveSettings(); Serial.printf("Знак курса гироскопа: %+d\n", sign); return true; }
  if (s == "gi") { printState(); return true; }
  return false;
}

}  // namespace gyro
