// «ESP на компьютере» (Linux/macOS/WSL): тот же код nanopb и те же кадры, что в прошивке.
// Подключается с рукопожатием Syn/Ack (MAC 02:11:22:33:44:55, камера на борту), затем шлёт
// снимки telemetry.v1.Telemetry: 3 с без людей, 6 с с человеком (alarm_active), 6 с снова без.
// Сервер должен показать тревогу и через 5 с после ухода человека — отбой.
//
// Сборка (одной строкой, из папки tools):
//   g++ -std=c++17 -I../esp/libraries/TelemetryProto/src fake_device.cpp
//       ../esp/libraries/TelemetryProto/src/pb_*.c
//       ../esp/libraries/TelemetryProto/src/telemetry/v1/*.pb.c -o fake_device
// Запуск: ./fake_device 127.0.0.1 9000 [auto|strict|off]   (рукопожатие: как в TelemetryLink.h)
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <cstring>
#include "pc_link.h"

int main(int argc, char **argv) {
  const char *host = argc > 1 ? argv[1] : "127.0.0.1";
  int port = argc > 2 ? atoi(argv[2]) : 9000;
  int hs = argc > 3 ? (!strcmp(argv[3], "strict") ? 1 : !strcmp(argv[3], "off") ? 2 : 0) : 0;
  const uint8_t mac[6] = {0x02, 0x11, 0x22, 0x33, 0x44, 0x55};
  static telemetry_v1_Syn ob;
  telemetry::synAddOnboard(ob, "camera", "http://127.0.0.1:81/stream");
  int sock = pclink::connect(host, port, mac, &ob, hs);
  if (sock < 0) return 1;

  bool alarm = false;
  int lastPersonTick = -1000;
  for (int tick = 0; tick < 75; tick++) {               // 15 с по 200 мс
    bool person = tick >= 15 && tick < 45;
    if (person) { alarm = true; lastPersonTick = tick; }
    else if (alarm && tick - lastPersonTick > 25) alarm = false;   // 5 с без людей — отбой

    telemetry_v1_Telemetry t = telemetry_v1_Telemetry_init_zero;
    t.firmware_version = 3;
    t.uptime_s = tick / 5;
    t.radar_ok = true;
    t.detection_class = person ? telemetry_v1_DetectionClass_DETECTION_CLASS_STATIONARY
                               : telemetry_v1_DetectionClass_DETECTION_CLASS_EMPTY;
    t.person_probability = person ? 0.9f : 0;
    t.has_position = true; t.position.lat = 55.751244; t.position.lon = 37.618423;
    t.satellites = 8; t.has_hdop = true; t.hdop = 1.1f;
    t.heading_valid = true; t.has_heading_deg = true; t.heading_deg = 90;
    t.rssi_dbm = -61;
    if (person) {
      telemetry_v1_Track &o = t.tracks[t.tracks_count++];
      o.id = 1; o.distance_m = 3.2f; o.angle_deg = 25; o.speed_mps = 0.02f;
      o.confirmed = tick >= 25; o.hit_ratio = 0.85f;
      o.detection_class = o.confirmed ? telemetry_v1_DetectionClass_DETECTION_CLASS_STATIONARY
                                      : telemetry_v1_DetectionClass_DETECTION_CLASS_UNSPECIFIED;
      t.target_count = o.confirmed ? 1 : 0;
    }
    t.background_cells = 14;
    t.alarm_active = alarm && tick >= 25;

    uint8_t buf[telemetry::MAX_SNAPSHOT];
    size_t n = telemetry::encodeTelemetry(t, buf, sizeof buf);
    if (!n || write(sock, buf, n) != (ssize_t)n) { printf("ошибка отправки\n"); return 1; }
    if (tick % 5 == 0) printf("t=%2d с: снимок %zu байт, людей %u, alarm_active=%d\n", tick / 5, n, t.target_count, t.alarm_active);
    usleep(200000);
  }
  close(sock);
  printf("готово\n");
}
