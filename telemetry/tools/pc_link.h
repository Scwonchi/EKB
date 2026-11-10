// Подключение к серверу с компьютера (Linux/macOS/WSL): TCP + рукопожатие Syn/Ack —
// тот же порядок, что в TelemetryLink.h на ESP32. Для имитаторов fake_device и scout_sim.
#pragma once
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <poll.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include "telemetry_codec.h"

namespace pclink {

static bool writeAll(int fd, const uint8_t *p, size_t n) {
  while (n) { ssize_t w = write(fd, p, n); if (w <= 0) return false; p += w; n -= (size_t)w; }
  return true;
}

// handshake: 0 — авто (нет Ack за 3 с — старый сервер, продолжаем), 1 — строго, 2 — без Syn.
// Возвращает сокет или -1.
static int connect(const char *host, int port, const uint8_t mac[6], const telemetry_v1_Syn *onboards = nullptr,
                   int handshake = 0) {
  addrinfo hints{}, *res = nullptr;
  hints.ai_family = AF_INET; hints.ai_socktype = SOCK_STREAM;
  char ps[8]; snprintf(ps, sizeof ps, "%d", port);
  if (getaddrinfo(host, ps, &hints, &res) || !res) { fprintf(stderr, "не найден %s\n", host); return -1; }
  int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (::connect(fd, res->ai_addr, res->ai_addrlen)) { perror("connect"); freeaddrinfo(res); close(fd); return -1; }
  freeaddrinfo(res);
  if (handshake == 2) { printf("подключено к %s:%d, без рукопожатия\n", host, port); return fd; }

  static telemetry_v1_Syn syn;
  telemetry::synInit(syn, mac);
  if (onboards)
    for (pb_size_t i = 0; i < onboards->onboards_count; i++)
      telemetry::synAddOnboard(syn, onboards->onboards[i].name, onboards->onboards[i].url);
  uint8_t buf[telemetry::MAX_SYN];
  size_t n = telemetry::encodeSyn(syn, buf, sizeof buf);
  if (!n || !writeAll(fd, buf, n)) { printf("Syn не отправлен\n"); close(fd); return -1; }

  telemetry::FrameReader fr(64);
  pollfd pf{fd, POLLIN, 0};
  int left = handshake == 1 ? 10000 : 3000;                // строго — ждём дольше, как TelemetryLink
  while (left > 0) {
    if (poll(&pf, 1, 100) > 0) {
      uint8_t c;
      if (read(fd, &c, 1) != 1) { printf("сервер закрыл подключение после Syn\n"); close(fd); return -1; }
      telemetry::FrameReader::Result r = fr.feed(c);
      if (r == telemetry::FrameReader::ERROR) { printf("неверный кадр ответа\n"); close(fd); return -1; }
      if (r != telemetry::FrameReader::FRAME_READY) continue;
      telemetry_v1_Ack a;
      if (!fr.decodeAck(a)) { printf("Ack не разобран\n"); close(fd); return -1; }
      if (a.code) { printf("сервер отказал: Ack code %lld\n", (long long)a.code); close(fd); return -1; }
      printf("подключено к %s:%d, Syn/Ack — принят (MAC %02x:%02x:%02x:%02x:%02x:%02x)\n", host, port,
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      return fd;
    }
    left -= 100;
  }
  if (handshake == 1) { printf("нет Ack за 10 с\n"); close(fd); return -1; }
  printf("подключено к %s:%d, сервер не ответил Ack — старый протокол, шлю снимки\n", host, port);
  return fd;
}

}  // namespace pclink
