// Регистратор данных LD2410C для обучения нейросети.
// Ровно 10 раз в секунду печатает строку из 18 чисел через запятую:
//   m0..m8 — энергия движения по зонам дальности (0,75 м каждая)
//   s0..s8 — энергия неподвижной цели по тем же зонам
// Эти строки принимает record.py на компьютере.
//
// Подключение LD2410C -> ESP32-S3: VCC->5V, GND->GND, TX->GPIO18, RX->GPIO17.
// Порт 115200. Монитор порта должен быть закрыт, когда работает record.py.

#include <Arduino.h>

#define RADAR_RX    18
#define RADAR_TX    17
#define RADAR_BAUD  256000
#define SAMPLE_MS   100        // 10 Гц — должно совпадать с FS в train.py

HardwareSerial Radar(1);

uint8_t mv[9] = {0}, stE[9] = {0};
bool engMode = false, haveFrame = false;
uint32_t lastFrameMs = 0, lastEngTry = 0, frames = 0;

void sendCmd(uint16_t cmd, const uint8_t *val, uint8_t vlen) {
  uint16_t len = 2 + vlen;
  uint8_t f[32]; int i = 0;
  f[i++] = 0xFD; f[i++] = 0xFC; f[i++] = 0xFB; f[i++] = 0xFA;
  f[i++] = len & 0xFF; f[i++] = len >> 8;
  f[i++] = cmd & 0xFF; f[i++] = cmd >> 8;
  for (int k = 0; k < vlen; k++) f[i++] = val[k];
  f[i++] = 0x04; f[i++] = 0x03; f[i++] = 0x02; f[i++] = 0x01;
  Radar.write(f, i);
  Radar.flush();
  delay(100);
}

void enableEngineering() {
  const uint8_t on[2] = {0x01, 0x00};
  sendCmd(0x00FF, on, 2);
  sendCmd(0x0062, nullptr, 0);
  sendCmd(0x00FE, nullptr, 0);
}

static const uint8_t HDR[4] = {0xF4, 0xF3, 0xF2, 0xF1};
uint8_t buf[64];
uint8_t pos = 0;
uint16_t frameLen = 0;

void handleFrame() {
  if (buf[7] != 0xAA) return;
  frames++;
  if (buf[6] != 0x01) { engMode = false; return; }
  engMode = true;
  uint8_t nM = min(buf[17] + 1, 9);
  uint8_t nS = min(buf[18] + 1, 9);
  for (int g = 0; g < 9; g++) mv[g]  = g < nM ? buf[19 + g] : 0;
  for (int g = 0; g < 9; g++) stE[g] = g < nS ? buf[19 + nM + g] : 0;
  haveFrame = true;
  lastFrameMs = millis();
}

void readRadar() {
  while (Radar.available()) {
    uint8_t b = Radar.read();
    if (pos < 4) {
      if (b == HDR[pos]) buf[pos++] = b;
      else if (b == HDR[0]) { buf[0] = b; pos = 1; }
      else pos = 0;
      continue;
    }
    buf[pos++] = b;
    if (pos == 6) {
      uint16_t len = buf[4] | (buf[5] << 8);
      if (len < 13 || len > 50) { pos = 0; continue; }
      frameLen = 6 + len + 4;
    }
    if (pos >= 6 && pos == frameLen) {
      if (buf[frameLen - 4] == 0xF8 && buf[frameLen - 1] == 0xF5) handleFrame();
      pos = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Radar.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX, RADAR_TX);
  delay(300);
  enableEngineering();
}

void loop() {
  readRadar();

  if (!engMode && frames > 0 && millis() - lastEngTry > 3000) {
    lastEngTry = millis();
    enableEngineering();
  }

  static uint32_t nextSample = 0;
  uint32_t now = millis();
  if ((int32_t)(now - nextSample) >= 0) {
    nextSample = now + SAMPLE_MS;
    if (!haveFrame || !engMode || now - lastFrameMs > 1000) return;  // нет свежих данных — молчим
    char line[96]; int p = 0;
    for (int g = 0; g < 9; g++) p += snprintf(line + p, sizeof(line) - p, "%u,", mv[g]);
    for (int g = 0; g < 9; g++) p += snprintf(line + p, sizeof(line) - p, g < 8 ? "%u," : "%u", stE[g]);
    Serial.println(line);
  }
}
