#include "led.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "config.h"

namespace {

bool s_ok = false;
bool s_track = false;
uint8_t s_mac[6];
float s_rssi = 0;         // the target's level: per-frame EMA
uint32_t s_seenMs = 0;    // last frame from the target (0 = none yet)
uint32_t s_blinkMs = 0;   // last new-network blink (0 = none)
float s_level = 0;        // output 0..1, smoothed

void write(float v) {
  const int duty = (int)lroundf(v * 255.0f);
  ledcWrite(cfg::kLedPin, cfg::kLedActiveLow ? 255 - duty : duty);
}

}  // namespace

void ledBegin() {
  s_ok = ledcAttach(cfg::kLedPin, 5000, 8);
  if (s_ok) write(0);
}

void ledTrack(const uint8_t *mac) {
  s_track = mac != nullptr;
  if (mac != nullptr) memcpy(s_mac, mac, 6);
  s_seenMs = 0;
}

void ledOnFrame(const uint8_t *a2, int8_t rssi, uint32_t nowMs) {
  if (!s_track || memcmp(a2, s_mac, 6) != 0) return;
  s_rssi = s_seenMs ? s_rssi + (rssi - s_rssi) * 0.4f : (float)rssi;
  s_seenMs = nowMs ? nowMs : 1;
}

void ledNewNetwork(uint32_t nowMs) {
  // the boot sweep finds every network at once: no blinks then, and at most one blink per 400 ms
  if (nowMs < cfg::kLedQuietBootMs || (s_blinkMs && nowMs - s_blinkMs < 400)) return;
  s_blinkMs = nowMs ? nowMs : 1;
}

void ledTick(uint32_t nowMs) {
  if (!s_ok) return;
  float target;
  if (s_track) {
    if (s_seenMs && nowMs - s_seenMs < cfg::kLedLostMs) {
      float x = (s_rssi - cfg::kLedRssiMin) / (float)(cfg::kLedRssiMax - cfg::kLedRssiMin);
      x = x < 0 ? 0 : x > 1 ? 1 : x;
      target = 0.03f + 0.97f * powf(x, 2.2f);  // even steps to the eye; never fully dark while tracking
    } else {
      target = ((nowMs / 150) & 1) ? 0.3f : 0.0f;  // target not heard: quick flash
    }
  } else {
    const float ph = (float)(nowMs % 3200) / 3200.0f;
    target = 0.015f + 0.05f * (0.5f - 0.5f * cosf(ph * 6.2831853f));  // dim slow breath: listening
    if (s_blinkMs && nowMs - s_blinkMs < 90) target = 0.7f;            // a new network
  }
  s_level = s_level * 0.7f + target * 0.3f;  // the original sketch's smoothing, at the same 10 ms step
  write(s_level);
}
