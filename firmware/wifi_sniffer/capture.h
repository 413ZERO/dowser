// Capture: radio set-up and the promiscuous callback. The callback only copies a few bytes into
// lock-free rings (see ring.h) and counts; all parsing and printing happens in the loop task.
#pragma once

#include <atomic>
#include <stdint.h>

#include "esp_err.h"

#include "config.h"
#include "ring.h"

// One per accepted management / data frame (header fields only).
struct FrameRec {
  uint8_t a1[6];  // receiver
  uint8_t a2[6];  // transmitter = the source ID
  uint8_t a3[6];  // BSSID for management frames
  uint8_t fc0;    // Frame Control byte 0: type bits 2-3, subtype bits 4-7
  uint8_t fc1;    // Frame Control byte 1: ToDS 0x01, FromDS 0x02
  int8_t rssi;    // rx_ctrl.rssi, dBm, clamped to -127..0 (the protocol range)
  int8_t noise;   // rx_ctrl.noise_floor, dBm
  uint8_t ch;     // rx_ctrl.channel: the channel the radio was tuned to
  uint8_t pad;
  uint16_t len;   // rx_ctrl.sig_len: the whole MPDU as received (on the C6 this counts the 4 FCS bytes)
};

// Bounded copy of a management body, only for frames that carry elements we parse:
// beacon 0x80, probe response 0x50, probe request 0x40, (re)assoc request 0x00 / 0x20,
// and vendor action frames 0xD0 from Espressif's OUI (ESP-NOW).
struct IeRec {
  uint8_t src[6];      // addr2
  uint8_t a3[6];       // addr3 (BSSID)
  uint8_t fc0;
  uint8_t ch;
  int8_t rssi;
  uint8_t exact;       // 1: bodyLen comes from rx_ctrl.dump_len (FCS excluded for sure); 0: from sig_len (FCS unknown)
  uint16_t bodyLen;    // frame length - 24: everything after the MAC header
  uint16_t n;          // bytes copied into body (= min(bodyLen, kIeMaxBytes))
  uint8_t body[cfg::kIeMaxBytes];
};

struct CaptureCounters {
  std::atomic<uint32_t> frames{0};     // accepted: rx_state OK, mgmt / data, >= 24 bytes
  std::atomic<uint32_t> ringDrop{0};   // frame ring full: frame lost
  std::atomic<uint32_t> ieCopied{0};   // mgmt bodies queued for parsing
  std::atomic<uint32_t> ieDrop{0};     // IE ring full: body not parsed (the frame itself still counted)
  std::atomic<uint32_t> badState{0};   // rx_state != 0 (CRC error etc.), dropped
  std::atomic<uint32_t> runt{0};       // shorter than a MAC header, dropped
  std::atomic<int32_t> noiseSum{0};
  std::atomic<uint32_t> noiseN{0};
  // Frame length source (answers FIRMWARE-fields.md's FCS [verify] item; printed by V).
  std::atomic<uint32_t> lenDump{0};    // dump_len used (0 < dump_len <= sig_len)
  std::atomic<uint32_t> lenDumpFcs{0}; // ... and sig_len - dump_len == 4: sig_len counts the FCS
  std::atomic<uint32_t> lenSig{0};     // dump_len missing / implausible: sig_len used, FCS position voted on
};

extern SpscRing<FrameRec, cfg::kFrameRingSlots> g_frameRing;
extern SpscRing<IeRec, cfg::kIeRingSlots> g_ieRing;
extern CaptureCounters g_cap;

// STA mode without a connection (station config cleared in RAM, never connect / scan), power save
// off, country GB 1-13 (manual), promiscuous MGMT|DATA on `firstChannel`. On failure returns the
// error and sets *failedStep.
esp_err_t captureBegin(uint8_t firstChannel, const char **failedStep);

esp_err_t captureSetChannel(uint8_t ch);
