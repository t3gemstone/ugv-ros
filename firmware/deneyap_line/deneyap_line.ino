// Deneyap Mini v2 / Kart 1A v2: 8-channel analog line sensor -> T3 Gemstone O1.
//
// Sensor D1..D8 -> A0..A7. Select the board you are using in Deneyap Kart IDE:
//   Mini v2 (ESP32-S2): GPIO 7, 8, 9, 10, 11, 12, 13, 16.
//   Kart 1A v2 (ESP32-S3): GPIO 4, 5, 6, 7, 15, 16, 17, 18.
// The compiled .bin files of the two boards are not interchangeable.
// One line every ~20 ms:  {"ugv_adc":1,"seq":<counter>,"mv":[D1,...,D8]}
//   UART0: TX GPIO43 -> T3 Gemstone O1 pin 10, RX GPIO44 <- pin 8, 115200 8N1.
//   Mini v2: TX=D1, RX=D0. Kart 1A v2: TX=D2, RX=D3.
//   USB CDC (the same lines for debugging; skipped when not connected)
// Upload and wiring: firmware/deneyap_line/README.md; https://docs.t3gemstone.org/en/projects/ugv-ros/hardware
//
// Arduino board package: deneyap:esp32 2.1.1.
// Board: "Deneyap Mini v2" OR "Deneyap Kart 1A v2" (whichever model you have).
// USB CDC On Boot: Enabled. If the board doesn't come up for uploading, hold
// BUTTON, press and release RESET, then release BUTTON.

#include <Arduino.h>

const uint8_t SENSOR_PINS[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
const uint32_t BAUD = 115200;
const uint32_t PERIOD_MS = 20;  // 50 frames/s
const int SAMPLES = 4;          // samples averaged per channel

uint32_t sequence = 0;

void setup() {
  Serial.begin(BAUD);                         // USB CDC
  Serial0.begin(BAUD, SERIAL_8N1, RX, TX);    // UART0: GPIO44 RX, GPIO43 TX
  analogReadResolution(12);
  for (uint8_t pin : SENSOR_PINS) {
    pinMode(pin, INPUT);
    analogSetPinAttenuation(pin, ADC_11db);   // 0 .. ~3.1 V
  }
}

void loop() {
  uint32_t started = millis();
  uint32_t mv[8] = {};
  for (int round = 0; round < SAMPLES; ++round) {
    for (int ch = 0; ch < 8; ++ch) {
      mv[ch] += analogReadMilliVolts(SENSOR_PINS[ch]);
    }
  }
  char frame[128];
  int length = snprintf(frame, sizeof(frame),
      "{\"ugv_adc\":1,\"seq\":%lu,\"mv\":[%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu]}\n",
      (unsigned long)sequence++,
      (unsigned long)(mv[0] / SAMPLES), (unsigned long)(mv[1] / SAMPLES),
      (unsigned long)(mv[2] / SAMPLES), (unsigned long)(mv[3] / SAMPLES),
      (unsigned long)(mv[4] / SAMPLES), (unsigned long)(mv[5] / SAMPLES),
      (unsigned long)(mv[6] / SAMPLES), (unsigned long)(mv[7] / SAMPLES));
  if (length > 0 && length < (int)sizeof(frame)) {
    Serial0.write((const uint8_t *)frame, length);
    if (Serial) {
      Serial.write((const uint8_t *)frame, length);
    }
  }
  uint32_t elapsed = millis() - started;
  if (elapsed < PERIOD_MS) {
    delay(PERIOD_MS - elapsed);
  }
}
