// IP-камера разведчика: видеопоток MJPEG по HTTP (esp_http_server, как в примере CameraWebServer).
//
//   http://<адрес>/          страница: видео + состояние радара (тревога, люди, дистанция, угол)
//   http://<адрес>:81/stream поток MJPEG — открывается в браузере, VLC, OBS, <img src=...>
//   http://<адрес>/capture   один кадр JPEG
//   http://<адрес>/status    состояние радара в JSON (страница опрашивает его 2 раза в секунду)
//
// Через ретранслятор на носителе: http://<ретранслятор>:8000/ — он пробрасывает 8000 -> 80
// и 8001 -> 81 последнего подключившегося разведчика (страница сама берёт порт потока «+1»).
//
// Поток отдаётся одному зрителю за раз: каждый лишний зритель удваивает трафик через
// ретранслятор, а телеметрии нужен свой кусок канала.
#pragma once
#include <Arduino.h>
#include "camera.h"

#if !defined(NO_CAMERA)
#include "esp_http_server.h"

namespace stream {

static int fpsLimit = 10;                     // кадров в секунду не больше
static volatile uint32_t frames = 0;          // отправлено кадров с включения
static volatile bool watching = false;        // сейчас кто-то смотрит поток
static httpd_handle_t pageServer = nullptr, streamServer = nullptr;

// Скетч заполняет JSON с состоянием (тревога, цели) — для страницы
typedef size_t (*StatusFn)(char *buf, size_t cap);
static StatusFn statusFn = nullptr;

static const char PAGE[] = R"HTML(<!doctype html><html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Разведчик</title>
<style>
body{margin:0;background:#111;color:#eee;font:15px system-ui,sans-serif}
#v{display:block;width:100%;max-width:960px;margin:0 auto;background:#000;min-height:200px}
#bar{max-width:960px;margin:0 auto;padding:8px 12px;display:flex;gap:16px;flex-wrap:wrap;align-items:center}
#al{padding:4px 10px;border-radius:6px;background:#2b8a3e;font-weight:700}
#al.on{background:#e03131;animation:b 1s infinite}@keyframes b{50%{opacity:.6}}
#t{max-width:960px;margin:0 auto;padding:0 12px 12px;color:#bbb}a{color:#74c0fc}
</style></head><body>
<img id="v" alt="видео с разведчика">
<div id="bar"><span id="al">нет данных</span><span id="st"></span><a id="cap" href="capture" target="_blank">кадр JPEG</a></div>
<div id="t"></div>
<script>
// Напрямую страница — http://<разведчик>/, поток — порт 81. Через ретранслятор — http://<ретранслятор>:8000/<номер>/,
// поток — там же, <номер>/stream (ретранслятор сам отправит его на порт 81 нужного разведчика)
const base = location.pathname.replace(/[^/]*$/, '');
const direct = base === '/';
const su = direct ? location.protocol + '//' + location.hostname + ':81/stream' : base + 'stream';
const v = document.getElementById('v');
function start() { v.src = su + '?t=' + Date.now(); }
v.onerror = () => setTimeout(start, 1500);
start();
async function poll() {
  try {
    const s = await (await fetch(base + 'status', {cache: 'no-store'})).json();
    const al = document.getElementById('al');
    al.textContent = s.alarm ? 'ТРЕВОГА' : 'норма';
    al.className = s.alarm ? 'on' : '';
    document.getElementById('st').textContent =
      `${s.phase} · людей в обзоре ${s.people} · на карте ${s.map} · курс ${s.heading} · радар ${s.radar ? 'ок' : 'НЕТ'}`;
    document.getElementById('t').innerHTML = (s.targets || []).map(t =>
      `${t.confirmed ? '<b>человек</b>' : 'цель'} — ${t.d.toFixed(1)} м, ${t.a > 0 ? '+' : ''}${t.a.toFixed(0)}° от оси`).join('<br>');
  } catch (e) { document.getElementById('al').textContent = 'нет связи'; }
}
setInterval(poll, direct ? 500 : 1000); poll();     // через ретранслятор реже: каждый опрос — подключение
</script></body></html>)HTML";

inline esp_err_t onPage(httpd_req_t *r) {
  httpd_resp_set_type(r, "text/html; charset=utf-8");
  return httpd_resp_send(r, PAGE, sizeof(PAGE) - 1);
}

inline esp_err_t onStatus(httpd_req_t *r) {
  char buf[768];
  size_t n = statusFn ? statusFn(buf, sizeof buf) : snprintf(buf, sizeof buf, "{}");
  httpd_resp_set_type(r, "application/json");
  httpd_resp_set_hdr(r, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(r, "Cache-Control", "no-store");
  return httpd_resp_send(r, buf, n);
}

inline esp_err_t onCapture(httpd_req_t *r) {
  camera_fb_t *fb = cam::capture();
  if (!fb) { httpd_resp_send_500(r); return ESP_FAIL; }
  httpd_resp_set_type(r, "image/jpeg");
  httpd_resp_set_hdr(r, "Content-Disposition", "inline; filename=scout.jpg");
  httpd_resp_set_hdr(r, "Access-Control-Allow-Origin", "*");
  esp_err_t e = httpd_resp_send(r, (const char *)fb->buf, fb->len);
  cam::release(fb);
  return e;
}

#define STREAM_BOUNDARY "scoutframe"
inline esp_err_t onStream(httpd_req_t *r) {
  httpd_resp_set_type(r, "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY);
  httpd_resp_set_hdr(r, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(r, "Cache-Control", "no-store");
  watching = true;
  esp_err_t e = ESP_OK;
  char head[96];
  uint32_t next = millis();
  while (e == ESP_OK) {
    int32_t wait = (int32_t)(next - millis());
    if (wait > 0) delay(wait);
    next = millis() + 1000 / (fpsLimit > 0 ? fpsLimit : 1);
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { delay(50); continue; }
    int hn = snprintf(head, sizeof head, "\r\n--" STREAM_BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                      (unsigned)fb->len);
    e = httpd_resp_send_chunk(r, head, hn);
    if (e == ESP_OK) e = httpd_resp_send_chunk(r, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (e == ESP_OK) frames = frames + 1;
  }
  watching = false;
  return e;
}

// Запуск: страница на порту 80, поток на 81 (отдельные серверы — поток не мешает странице)
inline bool begin() {
  if (!cam::ok) return false;
  httpd_config_t c = HTTPD_DEFAULT_CONFIG();
  c.server_port = 80;
  c.ctrl_port = 32768;
  c.max_open_sockets = 4;
  c.lru_purge_enable = true;
  httpd_uri_t page = {"/", HTTP_GET, onPage, nullptr};
  httpd_uri_t status = {"/status", HTTP_GET, onStatus, nullptr};
  httpd_uri_t capture = {"/capture", HTTP_GET, onCapture, nullptr};
  if (httpd_start(&pageServer, &c) != ESP_OK) return false;
  httpd_register_uri_handler(pageServer, &page);
  httpd_register_uri_handler(pageServer, &status);
  httpd_register_uri_handler(pageServer, &capture);

  httpd_config_t c2 = HTTPD_DEFAULT_CONFIG();
  c2.server_port = 81;
  c2.ctrl_port = 32769;
  c2.max_open_sockets = 2;
  c2.lru_purge_enable = true;                 // новый зритель вытесняет зависшего старого
  httpd_uri_t st = {"/stream", HTTP_GET, onStream, nullptr};
  if (httpd_start(&streamServer, &c2) != ESP_OK) return false;
  httpd_register_uri_handler(streamServer, &st);
  return true;
}

}  // namespace stream

#else   // вариант без камеры
namespace stream {
typedef size_t (*StatusFn)(char *buf, size_t cap);
static StatusFn statusFn = nullptr;
static int fpsLimit = 0;
static volatile uint32_t frames = 0;
static volatile bool watching = false;
inline bool begin() { return false; }
}  // namespace stream
#endif
