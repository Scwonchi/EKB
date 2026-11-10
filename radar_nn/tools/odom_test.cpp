// Проверка одометрии (rescue_scout/odom.h): квадрат 2×2 м с поворотами направо, полный круг на месте,
// перевод в широту/долготу — против той же формулы, что в server.py (offset_point).
//   g++ -std=c++17 -I../rescue_scout odom_test.cpp -o odom_test && ./odom_test
#include <cstdio>
#include <cmath>
#include "odom.h"

static int bad = 0;
static void check(const char *what, double got, double want, double tol) {
  bool ok = fabs(got - want) <= tol;
  bad += !ok;
  printf("%s %s: %.6f (ждали %.6f)\n", ok ? "ок    " : "ОШИБКА", what, got, want);
}

int main() {
  odom::Odom o;
  o.reset(0);
  uint32_t t = 0;
  float h = 350;                                       // гироскоп показывает 350° на старте
  o.heading(h, t);
  for (int side = 0; side < 4; side++) {
    for (int i = 0; i < 20; i++) o.move(0.1f, 0, t += 100);          // 2 м вперёд
    for (int i = 0; i < 9; i++) { h = odom::wrap360(h + 10); o.heading(h, t += 100); }   // 90° направо
  }
  check("квадрат: x в конце", o.x, 0, 1e-3);
  check("квадрат: y в конце", o.y, 0, 1e-3);
  check("пройдено, м", o.path, 8, 1e-3);
  check("повернулся, °", o.yaw, 360, 1e-3);

  o.reset(0);
  h = 10; o.heading(h, 0);
  for (int i = 0; i < 20; i++) o.move(0.1f, 0, 100 * i);          // 2 м вперёд
  for (int i = 0; i < 9; i++) { h = odom::wrap360(h + 10); o.heading(h, 0); }
  for (int i = 0; i < 10; i++) o.move(0.1f, 0, 0);                 // 1 м направо
  check("Г-путь: x", o.x, 1, 1e-3);
  check("Г-путь: y", o.y, 2, 1e-3);
  for (int i = 0; i < 72; i++) { h = odom::wrap360(h - 5); o.heading(h, 0); }   // полный круг против часовой
  check("круг против часовой: поворот", o.yaw, 90 - 360, 1e-3);

  // Широта/долгота: старт в Москве, дрон смотрел на восток (90°), точка (x=1, y=2) —
  // 2 м на восток и 1 м на юг
  o.homeSet = true; o.homeLat = 55.751244; o.homeLon = 37.618423; o.homeAz = 90;
  double lat, lon;
  o.latLon(lat, lon);
  // server.py offset_point: lat + deg(d*cos(b)/R), lon + deg(d*sin(b)/(R*cos(lat)))
  double R = 6371000.0, dE = 2, dN = -1;
  check("широта", lat, 55.751244 + dN / R * 180 / M_PI, 1e-9);
  check("долгота", lon, 37.618423 + dE / (R * cos(55.751244 * M_PI / 180)) * 180 / M_PI, 1e-9);
  check("азимут дрона: 90 + (−270) = 180", o.azimuth(), 180, 1e-3);
  printf(bad ? "ИТОГ: ошибок %d\n" : "ИТОГ: одометрия верна\n", bad);
  return bad != 0;
}
