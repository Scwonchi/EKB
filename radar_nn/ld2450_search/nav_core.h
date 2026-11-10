// Навигационная математика без привязки к Arduino (проверяется и на компьютере):
//   - разбор NMEA от GPS (GGA, RMC);
//   - курс по магнитометру с компенсацией наклона по акселерометру;
//   - калибровка магнитометра (смещение и масштаб по осям);
//   - точка на заданном расстоянии и азимуте от исходной.
//
// Оси платы: X — вперёд (стрелка X на модуле, туда же смотрит радар), Y — влево, Z — вверх.
#pragma once
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

namespace nav {

static const float DEG = 180.0f / (float)M_PI;

// ======================= GPS (NMEA) =======================
struct GpsData {
  bool   fix = false;        // есть координаты (GGA quality > 0 и RMC 'A')
  double lat = 0, lon = 0;   // градусы, + север / + восток
  float  alt = 0;            // м над уровнем моря
  int    sats = 0;
  float  hdop = 99;
  int    quality = 0;        // 0 нет, 1 GPS, 2 DGPS
  char   time[9] = "--:--:--";  // UTC
  uint32_t sentences = 0, badChecksum = 0;
};

class NmeaParser {
 public:
  GpsData d;
  bool ggaUpdated = false;   // выставляется при каждом корректном GGA

  // Подаём по одному символу; возвращает true, когда разобрано корректное предложение
  bool feed(char c) {
    if (c == '$') { len_ = 0; in_ = true; return false; }
    if (!in_) return false;
    if (c == '\r' || c == '\n') { in_ = false; return finish(); }
    if (len_ < (int)sizeof(line_) - 1) line_[len_++] = c; else in_ = false;
    return false;
  }

 private:
  char line_[100];
  int  len_ = 0;
  bool in_ = false;

  static int hex(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
  }

  // ddmm.mmmm + полушарие -> градусы
  static double coord(const char *v, const char *hemi) {
    if (!*v) return NAN;
    double x = atof(v);
    int dd = (int)(x / 100);
    double r = dd + (x - dd * 100) / 60.0;
    if (*hemi == 'S' || *hemi == 'W') r = -r;
    return r;
  }

  bool finish() {
    line_[len_] = 0;
    char *star = strchr(line_, '*');
    if (!star || star[1] == 0 || star[2] == 0) return false;
    uint8_t sum = 0;
    for (char *p = line_; p < star; p++) sum ^= (uint8_t)*p;
    int h1 = hex(star[1]), h2 = hex(star[2]);
    if (h1 < 0 || h2 < 0 || sum != (uint8_t)(h1 * 16 + h2)) { d.badChecksum++; return false; }
    *star = 0;

    const char *f[20]; int n = 0;
    f[n++] = line_;
    for (char *p = line_; *p && n < 20; p++) if (*p == ',') { *p = 0; f[n++] = p + 1; }
    if (strlen(f[0]) != 5) return false;
    const char *type = f[0] + 2;              // GPGGA / GNGGA -> "GGA"

    if (!strcmp(type, "GGA") && n >= 10) {
      d.quality = atoi(f[6]);
      d.sats = atoi(f[7]);
      d.hdop = *f[8] ? (float)atof(f[8]) : 99;
      d.alt = (float)atof(f[9]);
      if (strlen(f[1]) >= 6) {
        memcpy(d.time, f[1], 2); d.time[2] = ':';
        memcpy(d.time + 3, f[1] + 2, 2); d.time[5] = ':';
        memcpy(d.time + 6, f[1] + 4, 2); d.time[8] = 0;
      }
      double la = coord(f[2], f[3]), lo = coord(f[4], f[5]);
      if (d.quality > 0 && !isnan(la) && !isnan(lo)) { d.lat = la; d.lon = lo; d.fix = true; }
      else d.fix = false;
      ggaUpdated = true;
      d.sentences++;
      return true;
    }
    if (!strcmp(type, "RMC") && n >= 7) {
      if (*f[2] != 'A') d.fix = false;
      d.sentences++;
      return true;
    }
    d.sentences++;
    return true;
  }
};

// ======================= Компас =======================
// Калибровка магнитометра: смещение (hard iron) и масштаб по осям (простая soft iron)
struct MagCal {
  float off[3] = {0, 0, 0};
  float scl[3] = {1, 1, 1};
  void apply(const float raw[3], float out[3]) const {
    for (int i = 0; i < 3; i++) out[i] = (raw[i] - off[i]) * scl[i];
  }
};

// Сбор минимумов/максимумов при вращении
struct MagCalCollector {
  float mn[3], mx[3];
  uint32_t n = 0;
  void reset() { for (int i = 0; i < 3; i++) { mn[i] = 1e30f; mx[i] = -1e30f; } n = 0; }
  void add(const float r[3]) {
    for (int i = 0; i < 3; i++) { if (r[i] < mn[i]) mn[i] = r[i]; if (r[i] > mx[i]) mx[i] = r[i]; }
    n++;
  }
  // Возвращает качество 0..1 (минимальный размах оси / максимальный). < 0.5 — вращали мало.
  // radius — ожидаемая длина откалиброванного вектора (для обнаружения магнитных помех)
  float result(MagCal &c, float *radius = nullptr) const {
    float rng[3], avg = 0, rmax = 0, rmin = 1e30f;
    for (int i = 0; i < 3; i++) {
      rng[i] = (mx[i] - mn[i]) / 2;
      c.off[i] = (mx[i] + mn[i]) / 2;
      avg += rng[i] / 3;
      if (rng[i] > rmax) rmax = rng[i];
      if (rng[i] < rmin) rmin = rng[i];
    }
    for (int i = 0; i < 3; i++) c.scl[i] = rng[i] > 1e-6f ? avg / rng[i] : 1;
    if (radius) *radius = avg;
    return rmax > 0 ? rmin / rmax : 0;
  }
};

// Наклон из акселерометра (оси платы). pitch > 0 — нос вверх, roll > 0 — правый борт вниз
inline void tilt(const float acc[3], float &pitchDeg, float &rollDeg) {
  float fx = acc[0], fy = -acc[1], fz = -acc[2];          // перевод в оси «вперёд-вправо-вниз»
  rollDeg  = atan2f(-fy, -fz) * DEG;
  pitchDeg = atan2f(fx, sqrtf(fy * fy + fz * fz)) * DEG;
}

// Магнитный курс оси X (0 = магнитный север, по часовой), градусы 0..360.
// mag — откалиброванный магнитометр, acc — акселерометр, оба в осях платы.
inline float magHeading(const float mag[3], const float acc[3], bool tiltComp) {
  float mx = mag[0], my = -mag[1], mz = -mag[2];
  float xh = mx, yh = my;
  if (tiltComp) {
    float fx = acc[0], fy = -acc[1], fz = -acc[2];
    float phi = atan2f(-fy, -fz);
    float th  = atan2f(fx, sqrtf(fy * fy + fz * fz));
    xh = mx * cosf(th) + my * sinf(phi) * sinf(th) + mz * cosf(phi) * sinf(th);
    yh = my * cosf(phi) - mz * sinf(phi);
  }
  float h = atan2f(-yh, xh) * DEG;
  return (h < 0 ? h + 360 : h) + 0.0f;    // + 0.0f убирает «-0»
}

inline float wrap360(float a) {
  a = fmodf(a, 360.0f);
  return (a < 0 ? a + 360 : a) + 0.0f;
}

inline const char *compassName(float deg) {
  static const char *n[8] = {"С", "СВ", "В", "ЮВ", "Ю", "ЮЗ", "З", "СЗ"};
  return n[(int)((wrap360(deg) + 22.5f) / 45.0f) % 8];
}

// Точка на расстоянии distM по азимуту bearingDeg (для десятков метров плоская Земля точна)
inline void offsetPoint(double lat, double lon, float bearingDeg, float distM,
                        double &outLat, double &outLon) {
  const double R = 6371000.0;
  double b = bearingDeg / DEG;
  outLat = lat + (distM * cos(b) / R) * DEG;
  outLon = lon + (distM * sin(b) / (R * cos(lat / DEG))) * DEG;
}

}  // namespace nav
