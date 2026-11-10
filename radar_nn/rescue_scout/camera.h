// Камера ESP32 / ESP32-S3 (OV2640 / OV3660) через драйвер esp32-camera (входит в ядро ESP32 для Arduino).
// Снимок JPEG 640×480 ~20–40 КБ; если вышел больше MAX_JPEG — качество понижается.
//
// Плата выбирается в rescue_scout.ino (BOARD_AI_THINKER, BOARD_XIAO_S3_SENSE, BOARD_ESP32S3_EYE_PINS).
// Для камеры нужна PSRAM: у «AI Thinker ESP32-CAM» включена сама, у ESP32-S3 — «OPI PSRAM».
#pragma once
#include <Arduino.h>

#if defined(NO_CAMERA)
// Вариант без камеры: те же функции, снимков нет
struct camera_fb_t { uint8_t *buf; size_t len; size_t width, height; };
namespace cam {
static bool ok = false, flashOn = false;
static int flashPin = -1;
inline bool begin() { return false; }
inline camera_fb_t *capture() { return nullptr; }
inline void release(camera_fb_t *) {}
}  // namespace cam
#else
#include "esp_camera.h"

namespace cam {

#if defined(BOARD_AI_THINKER)
// ESP32-CAM AI-Thinker (OV2640)
static const int PWDN = 32, RESET = -1, XCLK = 0, SIOD = 26, SIOC = 27;
static const int D7 = 35, D6 = 34, D5 = 39, D4 = 36, D3 = 21, D2 = 19, D1 = 18, D0 = 5;
static const int VSYNC = 25, HREF = 23, PCLK = 22;
#elif defined(BOARD_XIAO_S3_SENSE)
// Seeed Studio XIAO ESP32S3 Sense
static const int PWDN = -1, RESET = -1, XCLK = 10, SIOD = 40, SIOC = 39;
static const int D7 = 48, D6 = 11, D5 = 12, D4 = 14, D3 = 16, D2 = 18, D1 = 17, D0 = 15;
static const int VSYNC = 38, HREF = 47, PCLK = 13;
#elif defined(BOARD_ESP32S3_EYE_PINS)
// Freenove ESP32-S3-WROOM CAM, ESP32-S3-EYE и их клоны («ESP32-S3 CAM» с разъёмом OV2640)
static const int PWDN = -1, RESET = -1, XCLK = 15, SIOD = 4, SIOC = 5;
static const int D7 = 16, D6 = 17, D5 = 18, D4 = 12, D3 = 10, D2 = 8, D1 = 9, D0 = 11;
static const int VSYNC = 6, HREF = 7, PCLK = 13;
#else
#error "Выберите плату: BOARD_AI_THINKER, BOARD_XIAO_S3_SENSE или BOARD_ESP32S3_EYE_PINS"
#endif

static const size_t MAX_JPEG = 58000;
static bool ok = false;
static int quality = 12;                 // 10 — лучше и больше, 30 — хуже и меньше
// Вспышка: вывод светодиода (у ESP32-CAM — GPIO4) или -1. Включается на время снимка.
static int flashPin = -1;
static bool flashOn = true;

inline bool begin() {
  camera_config_t c = {};
  c.pin_pwdn = PWDN; c.pin_reset = RESET; c.pin_xclk = XCLK;
  c.pin_sccb_sda = SIOD; c.pin_sccb_scl = SIOC;
  c.pin_d7 = D7; c.pin_d6 = D6; c.pin_d5 = D5; c.pin_d4 = D4;
  c.pin_d3 = D3; c.pin_d2 = D2; c.pin_d1 = D1; c.pin_d0 = D0;
  c.pin_vsync = VSYNC; c.pin_href = HREF; c.pin_pclk = PCLK;
  c.xclk_freq_hz = 20000000;
  c.ledc_timer = LEDC_TIMER_0;
  c.ledc_channel = LEDC_CHANNEL_0;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_VGA;
  c.jpeg_quality = quality;
  c.fb_count = psramFound() ? 2 : 1;
  c.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;
  ok = esp_camera_init(&c) == ESP_OK;
  if (flashPin >= 0) { pinMode(flashPin, OUTPUT); digitalWrite(flashPin, LOW); }
  return ok;
}

// Свежий кадр. Вернуть его обязательно через release(). nullptr — камеры нет или ошибка.
inline camera_fb_t *capture() {
  if (!ok) return nullptr;
  bool fl = flashPin >= 0 && flashOn;
  if (fl) { digitalWrite(flashPin, HIGH); delay(120); }  // экспозиция успевает подстроиться
  camera_fb_t *fb = esp_camera_fb_get();                 // кадр мог лежать в буфере — выбрасываем
  if (fb) esp_camera_fb_return(fb);
  fb = esp_camera_fb_get();
  if (fl) digitalWrite(flashPin, LOW);
  if (fb && fb->len > MAX_JPEG) {                        // слишком большой — следующий хуже качеством
    esp_camera_fb_return(fb);
    sensor_t *s = esp_camera_sensor_get();
    if (s) { quality = quality + 6 > 40 ? 40 : quality + 6; s->set_quality(s, quality); }
    fb = esp_camera_fb_get();
    if (fb && fb->len > MAX_JPEG) { esp_camera_fb_return(fb); fb = nullptr; }
  }
  return fb;
}

inline void release(camera_fb_t *fb) { if (fb) esp_camera_fb_return(fb); }

}  // namespace cam
#endif  // NO_CAMERA
