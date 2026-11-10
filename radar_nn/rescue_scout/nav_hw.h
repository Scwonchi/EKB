// GPS GY-NEO6MV2 + компас GY-27 (ADXL345 + HMC5883L или QMC5883L) на ESP32-S3.
// Настройки (знаки осей, склонение, калибровка) хранятся во флеш-памяти ESP (Preferences)
// и общие для всех скетчей: настроили в nav_test — работает и в ld2410c_nn.
//
// Подключение:
//   Выводы ниже — только по умолчанию (для nav_test, ld2450_search, ld2410c_nn).
//   В rescue_scout выводы задаются в начале скетча для каждого варианта платы.
//   GY-NEO6MV2: VCC -> 5V, GND -> GND, TX -> GPIO16, RX -> GPIO15
//   GY-27:      VCC -> 3V3, GND -> GND, SDA -> GPIO8, SCL -> GPIO9
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include "nav_core.h"

#ifndef GPS_RX
#define GPS_RX 16      // сюда TX модуля GPS
#endif
#ifndef GPS_TX
#define GPS_TX 15      // сюда RX модуля GPS (можно не подключать)
#endif
#ifndef GPS_BAUD
#define GPS_BAUD 9600
#endif
#ifndef I2C_SDA
#define I2C_SDA 8
#endif
#ifndef I2C_SCL
#define I2C_SCL 9
#endif
#ifndef CAL_BUTTON
#define CAL_BUTTON 0   // кнопка BOOT: удержание 2 с — калибровка компаса
#endif

namespace nav {

enum MagChip { MAG_NONE = 0, MAG_HMC5883L = 1, MAG_QMC5883L = 2, MAG_QMC5883P = 3 };

struct Settings {
  int8_t magSign[3] = {1, 1, 1};
  bool   magSwap = false;           // поменять X и Y магнитометра
  int8_t accSign[3] = {1, 1, 1};
  bool   accSwap = false;
  float  declination = 11.5f;       // магнитное склонение, ° (+ восточное). Москва ~ +11..12
  float  mountOffset = 0;           // угол от стрелки X компаса до направления радара, ° по часовой
  bool   tiltComp = true;
  MagCal cal;
  float  calRadius = 0;             // 0 — калибровки нет
  uint8_t calChip = MAG_NONE;
};

struct State {
  GpsData gps;
  bool     gpsFresh = false;        // координаты есть и обновлялись < 3 с назад
  uint32_t gpsAgeMs = 0;
  bool     gpsAlive = false;        // от модуля вообще приходят данные
  MagChip  magChip = MAG_NONE;
  bool     accOk = false;
  float    heading = NAN;           // истинный азимут направления радара, °
  float    magHeading = NAN;        // магнитный курс стрелки X, °
  float    pitch = 0, roll = 0;
  float    magNorm = 0;             // длина откалиброванного вектора / ожидаемая (≈1)
  bool     magDisturbed = false;    // рядом металл/магнит — курсу доверять нельзя
  bool     calibrated = false;
};

static Settings cfg;
static State st;
static NmeaParser nmea;
static HardwareSerial GpsSerial(2);
static uint8_t magAddr = 0, accAddr = 0;
static float magF[3] = {0, 0, 0}, accF[3] = {0, 0, 1};
static bool filtInit = false;
static uint32_t lastGgaMs = 0, lastGpsByteMs = 0, lastSensorMs = 0;

// ---------- Хранение настроек ----------
inline void saveSettings() {
  Preferences p;
  p.begin("nav", false);
  p.putBytes("cfg", &cfg, sizeof(cfg));
  p.end();
}

inline void loadSettings() {
  Preferences p;
  p.begin("nav", true);
  if (p.getBytesLength("cfg") == sizeof(cfg)) p.getBytes("cfg", &cfg, sizeof(cfg));
  p.end();
}

// ---------- I2C ----------
inline bool i2cWrite(uint8_t a, uint8_t reg, uint8_t v) {
  Wire.beginTransmission(a); Wire.write(reg); Wire.write(v);
  return Wire.endTransmission() == 0;
}

inline bool i2cRead(uint8_t a, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(a); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(a, n) != n) return false;
  for (int i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

inline bool present(uint8_t a) {
  Wire.beginTransmission(a);
  return Wire.endTransmission() == 0;
}

inline void detectSensors() {
  uint8_t id[3] = {0, 0, 0};
  st.magChip = MAG_NONE;
  if (present(0x1E)) {                // HMC5883L (у клонов ID бывает не «H43» — принимаем любой)
    magAddr = 0x1E; st.magChip = MAG_HMC5883L;
    if (!(i2cRead(0x1E, 0x0A, id, 3) && id[0] == 'H' && id[1] == '4' && id[2] == '3'))
      Serial.printf("Магнитометр 0x1E: нестандартный ID %02X %02X %02X, пробую как HMC5883L\n", id[0], id[1], id[2]);
    i2cWrite(magAddr, 0x00, 0x70);   // 8 усреднений, 15 Гц
    i2cWrite(magAddr, 0x01, 0x20);   // ±1,3 Гс
    i2cWrite(magAddr, 0x02, 0x00);   // непрерывный режим
  } else if (present(0x0D)) {
    magAddr = 0x0D; st.magChip = MAG_QMC5883L;
    i2cWrite(magAddr, 0x0B, 0x01);   // период SET/RESET
    i2cWrite(magAddr, 0x09, 0x1D);   // непрерывный, 200 Гц, ±8 Гс, OSR 512
  } else if (present(0x2C)) {          // QMC5883P — на новых дешёвых платах
    magAddr = 0x2C; st.magChip = MAG_QMC5883P;
    if (i2cRead(0x2C, 0x00, id, 1) && id[0] != 0x80)
      Serial.printf("Устройство 0x2C: ID %02X (у QMC5883P должен быть 80) — возможно, другая микросхема\n", id[0]);
    i2cWrite(magAddr, 0x29, 0x06);   // знаки осей (по документации)
    i2cWrite(magAddr, 0x0B, 0x08);   // SET/RESET вкл, ±8 Гс
    i2cWrite(magAddr, 0x0A, 0xCD);   // нормальный режим, 200 Гц
  }
  st.accOk = false;
  for (uint8_t a : {(uint8_t)0x53, (uint8_t)0x1D}) {
    if (present(a) && i2cRead(a, 0x00, id, 1) && id[0] == 0xE5) {
      accAddr = a; st.accOk = true;
      i2cWrite(a, 0x31, 0x08);         // полное разрешение, ±2 g
      i2cWrite(a, 0x2C, 0x0A);         // 100 Гц
      i2cWrite(a, 0x2D, 0x08);         // измерение
      break;
    }
  }
  st.calibrated = cfg.calRadius > 0 && cfg.calChip == st.magChip;
}

// Сырые значения в осях микросхемы
inline bool readMagChip(float r[3]) {
  uint8_t b[6];
  if (st.magChip == MAG_HMC5883L) {
    if (!i2cRead(magAddr, 0x03, b, 6)) return false;
    r[0] = (int16_t)(b[0] << 8 | b[1]);      // X
    r[2] = (int16_t)(b[2] << 8 | b[3]);      // Z (порядок X, Z, Y)
    r[1] = (int16_t)(b[4] << 8 | b[5]);      // Y
    return r[0] != -4096 && r[1] != -4096 && r[2] != -4096;   // -4096 — переполнение
  }
  if (st.magChip == MAG_QMC5883L || st.magChip == MAG_QMC5883P) {
    if (!i2cRead(magAddr, st.magChip == MAG_QMC5883L ? 0x00 : 0x01, b, 6)) return false;
    r[0] = (int16_t)(b[1] << 8 | b[0]);
    r[1] = (int16_t)(b[3] << 8 | b[2]);
    r[2] = (int16_t)(b[5] << 8 | b[4]);
    return true;
  }
  return false;
}

inline bool readAccChip(float g[3]) {
  uint8_t b[6];
  if (!st.accOk || !i2cRead(accAddr, 0x32, b, 6)) return false;
  for (int i = 0; i < 3; i++) g[i] = (int16_t)(b[2 * i + 1] << 8 | b[2 * i]) * 0.0039f;
  return true;
}

inline void mapAxes(float v[3], const int8_t sign[3], bool swap) {
  if (swap) { float t = v[0]; v[0] = v[1]; v[1] = t; }
  for (int i = 0; i < 3; i++) v[i] *= sign[i];
}

// Сырые показания магнитометра в осях платы (до калибровки)
inline bool readMagBoard(float r[3]) {
  if (!readMagChip(r)) return false;
  mapAxes(r, cfg.magSign, cfg.magSwap);
  return true;
}

// ---------- Обновление ----------
inline void updateSensors() {
  float m[3], a[3];
  bool okM = readMagBoard(m);
  bool okA = readAccChip(a);
  if (okA) mapAxes(a, cfg.accSign, cfg.accSwap);
  if (!okM) { st.heading = NAN; return; }

  float mc[3];
  cfg.cal.apply(m, mc);
  const float k = filtInit ? 0.25f : 1.0f;   // сглаживание
  for (int i = 0; i < 3; i++) {
    magF[i] += k * (mc[i] - magF[i]);
    if (okA) accF[i] += k * (a[i] - accF[i]);
  }
  filtInit = true;

  tilt(accF, st.pitch, st.roll);
  st.magHeading = magHeading(magF, accF, cfg.tiltComp && st.accOk);
  st.heading = wrap360(st.magHeading + cfg.declination + cfg.mountOffset);

  float n = sqrtf(magF[0] * magF[0] + magF[1] * magF[1] + magF[2] * magF[2]);
  st.magNorm = cfg.calRadius > 0 ? n / cfg.calRadius : 0;
  st.magDisturbed = st.calibrated && fabsf(st.magNorm - 1) > 0.25f;
}

inline void begin() {
  loadSettings();
#ifndef NAV_NO_GPS                              // NAV_NO_GPS — модуля GPS нет (в помещении)
  GpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
#endif
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  detectSensors();
  if (CAL_BUTTON >= 0) pinMode(CAL_BUTTON, INPUT_PULLUP);   // -1 — кнопки нет (ESP32-CAM)
}

inline void calibrate(uint32_t ms = 30000);

// Вызывать в loop() как можно чаще
inline void poll() {
#ifndef NAV_NO_GPS
  while (GpsSerial.available()) {
    char c = GpsSerial.read();
    lastGpsByteMs = millis();
    if (nmea.feed(c) && nmea.ggaUpdated) { nmea.ggaUpdated = false; lastGgaMs = millis(); }
  }
#endif
  uint32_t now = millis();
  st.gps = nmea.d;
  st.gpsAlive = lastGpsByteMs && now - lastGpsByteMs < 2000;
  st.gpsAgeMs = lastGgaMs ? now - lastGgaMs : UINT32_MAX;
  st.gpsFresh = st.gps.fix && st.gpsAgeMs < 3000;

  if (now - lastSensorMs >= 50) {               // компас 20 раз в секунду
    lastSensorMs = now;
    if (st.magChip == MAG_NONE && now > 3000 && (now / 50) % 100 == 0) detectSensors();  // повторный поиск
    updateSensors();
  }

  static uint32_t pressedAt = 0;                // удержание BOOT 2 с — калибровка
  if (CAL_BUTTON >= 0 && digitalRead(CAL_BUTTON) == LOW) {
    if (!pressedAt) pressedAt = now;
    else if (now - pressedAt > 2000) { pressedAt = 0; calibrate(); }
  } else pressedAt = 0;
}

// ---------- Калибровка ----------
inline void calibrate(uint32_t ms) {
  if (st.magChip == MAG_NONE) {
    Serial.println("Калибровка компаса невозможна: магнитометр не найден (команда scan — что есть на шине I2C)");
    return;
  }
  Serial.printf("\n=== КАЛИБРОВКА КОМПАСА %lu с ===\n", ms / 1000);
  Serial.println("Вращайте устройство (в сборе, с аккумулятором) во всех направлениях:");
  Serial.println("полные обороты вокруг вертикали, затем на боку, вверх ногами, «восьмёркой».");
  MagCalCollector c; c.reset();
  uint32_t t0 = millis(), lastP = 0;
  while (millis() - t0 < ms) {
    float r[3];
    if (readMagBoard(r)) c.add(r);
    if (millis() - lastP > 1000) {
      lastP = millis();
      Serial.printf("  %2lu с: X %.0f..%.0f  Y %.0f..%.0f  Z %.0f..%.0f\n", (millis() - t0) / 1000,
                    c.mn[0], c.mx[0], c.mn[1], c.mx[1], c.mn[2], c.mx[2]);
    }
    delay(20);
  }
  MagCal cal; float radius = 0;
  float q = c.result(cal, &radius);
  if (c.n < 100 || q < 0.4f) {
    Serial.printf("Калибровка НЕ сохранена: вращали недостаточно (качество %.2f, нужно ≥ 0,4). Повторите.\n", q);
    return;
  }
  cfg.cal = cal; cfg.calRadius = radius; cfg.calChip = st.magChip;
  saveSettings();
  st.calibrated = true; filtInit = false;
  Serial.printf("Калибровка сохранена. Качество %.2f (%s)\n", q, q > 0.7f ? "хорошо" : "удовлетворительно");
}

// ---------- Команды из монитора порта ----------
inline void printSettings() {
  Serial.println("\n--- Настройки навигации ---");
  Serial.printf("Магнитометр: %s, акселерометр: %s\n",
                st.magChip == MAG_HMC5883L ? "HMC5883L" : st.magChip == MAG_QMC5883L ? "QMC5883L" :
                st.magChip == MAG_QMC5883P ? "QMC5883P" : "НЕ НАЙДЕН",
                st.accOk ? "ADXL345" : "НЕ НАЙДЕН");
  Serial.printf("Знаки магнитометра X%+d Y%+d Z%+d%s; акселерометра X%+d Y%+d Z%+d%s\n",
                cfg.magSign[0], cfg.magSign[1], cfg.magSign[2], cfg.magSwap ? ", X<->Y" : "",
                cfg.accSign[0], cfg.accSign[1], cfg.accSign[2], cfg.accSwap ? ", X<->Y" : "");
  Serial.printf("Склонение %.1f°, поправка установки %.1f°, компенсация наклона %s\n",
                cfg.declination, cfg.mountOffset, cfg.tiltComp ? "вкл" : "выкл");
  Serial.printf("Калибровка: %s\n", st.calibrated ? "есть" : "НЕТ (команда c или удержание BOOT 2 с)");
  Serial.println("Команды: c — калибровка | m1 m2 m3 — сменить знак оси X/Y/Z магнитометра | ms — поменять X,Y");
  Serial.println("         a1 a2 a3 / as — то же для акселерометра | d11.5 — склонение | o0 — поправка установки");
  Serial.println("         t — компенсация наклона вкл/выкл | i — настройки | scan — поиск I2C | regs — регистры | reset\n");
}

inline void i2cScan() {
  Serial.print("I2C устройства:");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) if (present(a)) { Serial.printf(" 0x%02X", a); n++; }
  Serial.println(n ? "" : " НЕТ — проверьте SDA/SCL, питание 3V3 и общий GND");
  Serial.println("  магнитометр: 0x1E (HMC5883L), 0x0D (QMC5883L), 0x2C (QMC5883P); акселерометр 0x53 (ADXL345); гироскоп MPU-6050 0x68");
}

// Дамп регистров магнитометра — для диагностики
inline void dumpRegs() {
  if (st.magChip == MAG_NONE) { Serial.println("Магнитометр не найден"); return; }
  uint8_t b[16] = {0};
  uint8_t n = st.magChip == MAG_HMC5883L ? 13 : 12;
  bool ok = i2cRead(magAddr, 0x00, b, n);
  Serial.printf("Регистры 0x%02X%s:", magAddr, ok ? "" : " (ошибка чтения)");
  for (int i = 0; i < n; i++) Serial.printf(" %02X", b[i]);
  if (st.magChip == MAG_QMC5883P && i2cRead(magAddr, 0x29, b, 1)) Serial.printf(" | 29h=%02X", b[0]);
  Serial.println();
}

// Возвращает true, если строка была навигационной командой
inline bool command(String s) {
  s.trim();
  if (!s.length()) return false;
  bool changed = true;
  if (s == "c") { calibrate(); return true; }
  else if (s == "i") { printSettings(); return true; }
  else if (s == "scan") { i2cScan(); detectSensors(); printSettings(); return true; }
  else if (s == "regs") { dumpRegs(); return true; }
  else if (s == "reset") { cfg = Settings(); st.calibrated = false; }
  else if (s == "ms") {                         // калибровку пересчитываем, повторять не нужно
    cfg.magSwap = !cfg.magSwap;
    float t = cfg.cal.off[0]; cfg.cal.off[0] = cfg.cal.off[1]; cfg.cal.off[1] = t;
    t = cfg.cal.scl[0]; cfg.cal.scl[0] = cfg.cal.scl[1]; cfg.cal.scl[1] = t;
    int8_t g = cfg.magSign[0]; cfg.magSign[0] = cfg.magSign[1]; cfg.magSign[1] = g;
  }
  else if (s == "as") {
    cfg.accSwap = !cfg.accSwap;
    int8_t g = cfg.accSign[0]; cfg.accSign[0] = cfg.accSign[1]; cfg.accSign[1] = g;
  }
  else if (s.length() == 2 && s[0] == 'm' && s[1] >= '1' && s[1] <= '3') {
    int i = s[1] - '1';
    cfg.magSign[i] *= -1;
    cfg.cal.off[i] *= -1;
  }
  else if (s.length() == 2 && s[0] == 'a' && s[1] >= '1' && s[1] <= '3') cfg.accSign[s[1] - '1'] *= -1;
  else if (s[0] == 'd' && s.length() > 1) cfg.declination = s.substring(1).toFloat();
  else if (s[0] == 'o' && s.length() > 1) cfg.mountOffset = s.substring(1).toFloat();
  else if (s == "t") cfg.tiltComp = !cfg.tiltComp;
  else changed = false;
  if (changed) {
    saveSettings(); filtInit = false;
    printSettings();
  }
  return changed;
}

}  // namespace nav
