// Management-body parser: Information Elements, WPS and P2P attributes, ESP-NOW. Pure functions over
// a copied body; runs in the loop task. Never forwards WPS UUID-E (0x1047) or Serial Number (0x1042).
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "config.h"

struct Str32 {
  uint8_t len = 0;
  uint8_t b[cfg::kStrMax];
  void assign(const uint8_t *p, size_t n);
  bool same(const Str32 &o) const;
};

struct WpsInfo {
  bool hasType = false;  // Primary Device Type 0x1054
  uint16_t cat = 0, sub = 0;
  Str32 name;     // Device Name 0x1011
  Str32 mfr;      // Manufacturer 0x1021
  Str32 model;    // Model Name 0x1023
  Str32 modelNo;  // Model Number 0x1024
  bool any() const { return hasType || name.len || mfr.len || model.len || modelNo.len; }
};

struct P2pDevInfo {  // P2P attribute 13 (Device Info)
  bool has = false;
  uint16_t cat = 0, sub = 0;
  Str32 name;
};

struct IeInfo {
  bool hasCap = false;  // beacon / probe response: capability field read
  bool privacy = false; // capability bit 4
  bool hasSsid = false; // SSID element (id 0) present
  bool hidden = false;  // ... with length 0 or only zero bytes
  Str32 ssid;
  uint8_t dsCh = 0;     // DS Parameter Set (id 3); 0 = absent
  bool hasHt = false;   // id 45
  uint8_t htStreams = 0;
  bool ht40 = false;
  bool rsn = false;           // id 48
  bool interworking = false;  // id 107
  bool extcap = false;        // id 127
  bool vht = false;           // id 191
  bool he = false;            // id 255, ext 35
  bool p2p = false;           // vendor 50:6F:9A type 9
  bool espnow = false;        // action frame, category 127, Espressif OUI, vendor element type 4
  uint8_t nVie = 0;
  uint32_t vie[cfg::kMaxVendorIes];  // (OUI << 8) | type, unique
  WpsInfo wps;
  P2pDevInfo p2pDev;
};

// fc0 = Frame Control byte 0. body = bytes after the 24-byte MAC header, n = bytes available,
// bodyLen = frame length - 24 (n < bodyLen when the copy was cut). exact = bodyLen is known to exclude
// the FCS (from rx_ctrl.dump_len); otherwise it came from sig_len and may end with the 4 FCS bytes.
// Returns false if fc0 is not parsed.
bool parseMgmtBody(uint8_t fc0, const uint8_t *body, uint16_t n, uint16_t bodyLen, bool exact, IeInfo &out);

struct FcsStats {
  uint32_t votesIncluded;  // sig_len frames: element list ends 4 bytes before the body end
  uint32_t votesExcluded;  // sig_len frames: element list ends exactly at the body end
  uint32_t exactEndOk;     // dump_len frames: element list ends exactly at the body end (dump_len is right)
  uint32_t exactEndOff;    // dump_len frames: it does not (malformed frame, or dump_len means something else)
};
FcsStats fcsStats();
