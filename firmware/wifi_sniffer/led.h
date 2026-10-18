// Onboard user LED (one colour). Tracking (L,<ch>,<mac>): brightness follows that source's RSSI, like the
// original LED sketch. Scanning: a dim slow breath, plus a short blink when a new network turns up. Loop task only.
#pragma once

#include <stdint.h>

void ledBegin();
void ledTrack(const uint8_t *mac);  // nullptr = stop tracking (back to the breath)
void ledOnFrame(const uint8_t *a2, int8_t rssi, uint32_t nowMs);
void ledNewNetwork(uint32_t nowMs);
void ledTick(uint32_t nowMs);       // every cfg::kLedTickMs
