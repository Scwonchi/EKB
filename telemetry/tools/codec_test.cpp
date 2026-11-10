// Проверочные векторы протокола (docs/device-protocol.md): кодек прошивки даёт те же байты,
// что приведены в документации iot-go.
//   g++ -std=c++17 -I../esp/libraries/TelemetryProto/src codec_test.cpp ../esp/libraries/TelemetryProto/src/pb_*.c
//       ../esp/libraries/TelemetryProto/src/telemetry/v1/*.pb.c -o codec_test && ./codec_test
#include <cstdio>
#include <string>
#include "telemetry_codec.h"

static int bad = 0;
static void check(const char *name, const uint8_t *b, size_t n, const char *want) {
  std::string got;
  char h[4];
  for (size_t i = 0; i < n; i++) { snprintf(h, sizeof h, "%s%02X", i ? " " : "", b[i]); got += h; }
  bool ok = got == want;
  bad += !ok;
  printf("%s %s\n   %s\n", ok ? "ок    " : "ОШИБКА", name, got.c_str());
  if (!ok) printf("   ждали %s\n", want);
}

int main() {
  uint8_t buf[512];
  telemetry_v1_Telemetry t = telemetry_v1_Telemetry_init_zero;
  t.uptime_s = 1;
  check("Telemetry{uptime_s: 1}", buf, telemetry::encodeTelemetry(t, buf, sizeof buf), "00 00 00 02 10 01");

  telemetry_v1_Syn s;
  const uint8_t mac[6] = {0x02, 0x11, 0x22, 0x33, 0x44, 0x55};
  telemetry::synInit(s, mac);
  check("Syn{mac: 02:11:22:33:44:55}", buf, telemetry::encodeSyn(s, buf, sizeof buf), "00 00 00 08 0A 06 02 11 22 33 44 55");

  telemetry_v1_Ack a = telemetry_v1_Ack_init_zero;
  check("Ack{code: 0}", buf, telemetry::encodeFrame(telemetry_v1_Ack_fields, &a, buf, sizeof buf), "00 00 00 00");
  a.code = 1;
  check("Ack{code: 1}", buf, telemetry::encodeFrame(telemetry_v1_Ack_fields, &a, buf, sizeof buf), "00 00 00 02 08 01");

  // Разбор Ack так же, как в TelemetryLink (по одному байту)
  const uint8_t ack1[] = {0, 0, 0, 2, 8, 1}, ack0[] = {0, 0, 0, 0};
  for (auto pr : {std::make_pair(ack0, 4), std::make_pair(ack1, 6)}) {
    telemetry::FrameReader fr(64);
    telemetry::FrameReader::Result r = telemetry::FrameReader::NEED_MORE;
    for (int i = 0; i < pr.second; i++) r = fr.feed(pr.first[i]);
    telemetry_v1_Ack d;
    bool ok = r == telemetry::FrameReader::FRAME_READY && fr.decodeAck(d) && d.code == (pr.second == 6 ? 1 : 0);
    bad += !ok;
    printf("%s разбор Ack code %d\n", ok ? "ок    " : "ОШИБКА", pr.second == 6 ? 1 : 0);
  }

  // Полный пример снимка из iot-device-client-tutorial.md (97 байт). В документе пропущен один FF
  // в rssi_dbm = -55: varint отрицательного int32 — 10 байт (C9 + восемь FF + 01), а там семь FF,
  // 92 байта тела при заголовке 5D = 93. Здесь — верные байты (их же даёт Python protobuf).
  t = telemetry_v1_Telemetry_init_zero;
  t.firmware_version = 3; t.uptime_s = 42; t.radar_ok = true;
  t.detection_class = telemetry_v1_DetectionClass_DETECTION_CLASS_MOVING; t.person_probability = 0.88f;
  t.has_position = true; t.position.lat = 56.8389; t.position.lon = 60.6057;
  t.satellites = 10; t.has_hdop = true; t.hdop = 0.9f;
  t.heading_valid = true; t.has_heading_deg = true; t.heading_deg = 42;
  t.rssi_dbm = -55; t.target_count = 1;
  t.tracks_count = 1;
  telemetry_v1_Track &o = t.tracks[0];
  o.id = 1; o.distance_m = 4.5f; o.angle_deg = -12; o.speed_mps = 0.7f; o.confirmed = true;
  o.detection_class = telemetry_v1_DetectionClass_DETECTION_CLASS_MOVING; o.hit_ratio = 0.95f;
  t.background_cells = 24; t.alarm_active = true;
  check("полный снимок из документации", buf, telemetry::encodeTelemetry(t, buf, sizeof buf),
        "00 00 00 5D 08 03 10 2A 18 01 20 02 2D AE 47 61 3F 32 12 09 A5 4E 40 13 61 6B 4C 40 "
        "11 F6 97 DD 93 87 4D 4E 40 38 0A 45 66 66 66 3F 48 01 55 00 00 28 42 58 C9 "
        "FF FF FF FF FF FF FF FF 01 60 01 6A 1A 08 01 15 00 00 90 40 1D 00 00 40 C1 25 "
        "33 33 33 3F 28 01 30 02 3D 33 33 73 3F 78 18 80 01 01");
  printf(bad ? "\nИТОГ: ошибок %d\n" : "\nИТОГ: все векторы совпали\n", bad);
  return bad != 0;
}
