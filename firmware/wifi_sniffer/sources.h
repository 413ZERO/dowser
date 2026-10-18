// Source table: one entry per transmitter address (addr2), bounded, LRU eviction.
// Aggregates P lines per 25 ms window and keeps the identity behind M lines. Loop task only.
#pragma once

#include <stdint.h>

#include "capture.h"
#include "ieparse.h"

struct Source {
  bool used;
  uint8_t mac[6];
  uint32_t firstSeen; // millis()
  uint32_t lastSeen;

  // current window
  uint16_t n;
  int32_t rssiSum;
  int8_t rssiMax;
  uint8_t aggCh;
  uint32_t bytes;
  bool aggHasBssid;
  uint8_t aggBssid[6];

  // identity
  bool isAp;
  bool hasBssid;  // client: its AP
  uint8_t bssid[6];
  uint8_t dsCh;   // AP: DS Parameter Set channel (0 = not seen)
  uint8_t rxCh;   // channel it is heard best on (fallback for "ch")
  int8_t rxChRssi;
  uint32_t rxChAt;
  bool sawProbeResp;
  bool hasSsid;
  bool hidden;
  Str32 ssid;
  bool capSeen;   // AP: capability field read (privacy is meaningful)
  bool privacy, rsn, vht, he, extcap, interworking, espnow, p2p, hasHt, ht40;
  uint8_t htStreams;
  uint8_t nVie;
  uint32_t vie[cfg::kMaxVendorIes];
  WpsInfo wps;
  P2pDevInfo p2pDev;

  // M bookkeeping
  bool metaDirty;
  bool metaResend;  // send even if unchanged (V resync, periodic refresh)
  bool metaSent;
  uint32_t metaHash;
  uint32_t metaAt;
};

struct TableStats {
  uint32_t active;          // entries in use
  uint32_t tableDrops;      // frames with no free entry (all entries busy in the current window)
  uint32_t pFramesDropped;  // frames inside P lines that did not fit the USB buffer
  uint32_t evictions;
  uint32_t metaSent;
};

void sourcesOnFrame(const FrameRec &f, uint32_t nowMs);
void sourcesOnIe(const IeRec &r, uint32_t nowMs);
// Close the window: M lines for new / changed sources first, then one P line per source heard.
void sourcesFlush(uint32_t nowMs);
// Queue the M line of every known source again (sent over the next windows, kMetaPerFlush at a time).
// Called on V: a page that (re)connects to a board that kept running has missed the first M lines.
void sourcesResendMeta();
TableStats sourcesStats();
// Channels with an AP heard in the last `forMs` at `minRssi` or stronger: bit ch set. Drives the hop round.
uint16_t sourcesBusyChannels(uint32_t nowMs, uint32_t forMs, int8_t minRssi);
