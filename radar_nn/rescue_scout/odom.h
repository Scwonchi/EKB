// Одометрия разведчика: на сколько повернулся дрон и где он относительно точки старта —
// для трека передвижения на сервере. Не зависит от Arduino — проверяется на ПК (tools/odom_test.cpp).
//
// Поворот — гироскоп (gyro.h): курс непрерывный, +370° = полтора оборота по часовой.
// Перемещение — сдвиг в осях дрона (вперёд, вправо), его даёт источник перемещения:
//   - датчик оптического потока (MTF-01, Matek 3901-L0X — когда появится): move() каждый отсчёт;
//   - сейчас датчика нет: команда move в мониторе порта (проверка на столе, ручная отметка
//     «пролетел 2 м вперёд»). Без источника точка остаётся на старте, меняется только курс.
// По одному гироскопу и акселерометру расстояние не посчитать: ошибка двойного интегрирования
// за 10–20 с вырастает до метров.
//
// Координаты: x — вправо, y — вперёд от направления дрона на старте, метры. Для сервера
// они переводятся в широту и долготу от точки старта (home): так трек рисуется в поле
// position схемы iot-go без новых полей. Азимут старта (куда смотрел дрон) задаёт оператор
// или компас; без него трек верен по форме, но повёрнут.
#pragma once
#include <math.h>
#include <stdint.h>

namespace odom {

static const double EARTH_R = 6371000.0;
inline float wrap360(float a) { a = fmodf(a, 360.0f); if (a < 0) a += 360.0f; return a >= 360.0f ? 0 : a; }

struct Point { float x, y, yaw; uint32_t ms; };

class Odom {
 public:
  static const int TRAIL = 64;         // последние точки трека (для вывода в монитор порта)

  double x = 0, y = 0;                 // м от старта: вправо, вперёд (оси старта)
  float  yaw = 0;                      // поворот от старта, °, по часовой, непрерывный
  float  path = 0;                     // пройдено, м
  float  turned = 0;                   // повёрнуто всего (сумма модулей), °
  bool   haveYaw = false;
  Point  trail[TRAIL];
  int    trailN = 0, trailPos = 0;

  // Точка старта на карте (home) и куда смотрел дрон на старте
  bool   homeSet = false;
  double homeLat = 0, homeLon = 0;
  float  homeAz = NAN;                 // азимут от севера, NAN — неизвестен

  void reset(uint32_t now) {
    x = y = 0; yaw = 0; path = 0; turned = 0; haveYaw = false; last_ = NAN;
    trailN = trailPos = 0;
    addPoint(now, true);
  }

  // Курс гироскопа [0, 360) или NAN; разворачиваем в непрерывный поворот
  void heading(float h, uint32_t now) {
    if (isnan(h)) return;
    if (!isnan(last_)) {
      float d = wrap360(h - last_);
      if (d > 180) d -= 360;
      yaw += d;
      turned += fabsf(d);
    }
    last_ = h;
    haveYaw = true;
    addPoint(now, false);
  }

  // Сдвиг в осях дрона за отсчёт: вперёд и вправо, м (от источника перемещения)
  void move(float fwd, float right, uint32_t now) {
    float a = yaw * 0.017453293f;
    x += fwd * sinf(a) + right * cosf(a);
    y += fwd * cosf(a) - right * sinf(a);
    path += sqrtf(fwd * fwd + right * right);
    addPoint(now, false);
  }

  float distanceFromStart() const { return (float)sqrt(x * x + y * y); }

  // Азимут дрона от севера: азимут старта + поворот. NAN — азимут старта неизвестен
  float azimuth() const { return isnan(homeAz) ? NAN : wrap360(homeAz + fmodf(yaw, 360.0f)); }

  // Широта и долгота текущей точки. false — точка старта не задана
  bool latLon(double &lat, double &lon) const {
    if (!homeSet) return false;
    float az = isnan(homeAz) ? 0 : homeAz;      // без азимута — «вперёд на старте» = север
    double a = az * 0.017453292519943295;
    double north = y * cos(a) - x * sin(a);
    double east = y * sin(a) + x * cos(a);
    lat = homeLat + north / EARTH_R * 57.29577951308232;
    lon = homeLon + east / (EARTH_R * cos(homeLat * 0.017453292519943295)) * 57.29577951308232;
    return true;
  }

 private:
  float last_ = NAN;

  // В трек — точка, если сдвинулись на 0,2 м или повернулись на 15°
  void addPoint(uint32_t now, bool force) {
    if (!force && trailN) {
      const Point &p = trail[(trailPos - 1 + TRAIL) % TRAIL];
      if (hypot(x - p.x, y - p.y) < 0.2 && fabsf(yaw - p.yaw) < 15) return;
    }
    trail[trailPos] = {(float)x, (float)y, yaw, now};
    trailPos = (trailPos + 1) % TRAIL;
    if (trailN < TRAIL) trailN++;
  }
};

}  // namespace odom
