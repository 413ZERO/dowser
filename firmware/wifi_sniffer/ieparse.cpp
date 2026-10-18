#include "ieparse.h"

#include <string.h>

namespace {

uint32_t s_fcsIncluded = 0;
uint32_t s_fcsExcluded = 0;
uint32_t s_exactEndOk = 0;
uint32_t s_exactEndOff = 0;

// Concatenation buffers for WPS / P2P data split over several vendor elements (hostap does the same).
uint8_t s_wpsBuf[512];
uint8_t s_p2pBuf[512];

inline uint16_t be16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
inline uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

inline void append(uint8_t *dst, uint16_t &dn, size_t cap, const uint8_t *src, size_t sn) {
  const size_t room = cap - dn;
  const size_t k = sn < room ? sn : room;
  memcpy(dst + dn, src, k);
  dn = (uint16_t)(dn + k);
}

// Where the element list starts inside the management body (after the fixed fields).
int elementStart(uint8_t fc0) {
  switch (fc0) {
    case 0x80:  // beacon: timestamp 8, interval 2, capability 2
    case 0x50:  // probe response: same
      return 12;
    case 0x40:  // probe request: elements only
      return 0;
    case 0x00:  // association request: capability 2, listen interval 2
      return 4;
    case 0x20:  // reassociation request: + current AP 6
      return 10;
    default:
      return -1;
  }
}

// Where the element list ends. A cut copy ends where the copy ends. A body measured by dump_len ends
// at bodyLen. A body measured by sig_len may or may not carry the 4 FCS bytes at its end: a
// well-formed element list ends exactly at the body end, or exactly 4 bytes before it.
uint16_t elementEnd(const uint8_t *d, uint16_t start, uint16_t n, uint16_t bodyLen, bool lenExact) {
  if (n < bodyLen) return n;
  bool hitMinus4 = false;
  uint32_t p = start;
  while (p + 2 <= bodyLen) {
    if (bodyLen >= 4 && p == (uint32_t)bodyLen - 4) hitMinus4 = true;
    const uint32_t next = p + 2 + d[p + 1];
    if (next > bodyLen) break;
    p = next;
  }
  const bool exact = (p == bodyLen);
  if (lenExact) {  // diagnostics only: does dump_len really end where the elements end?
    if (exact) s_exactEndOk++;
    else s_exactEndOff++;
    return bodyLen;
  }
  if (exact && !hitMinus4) {
    s_fcsExcluded++;
    return bodyLen;
  }
  if (hitMinus4 && !exact) {
    s_fcsIncluded++;
    return (uint16_t)(bodyLen - 4);
  }
  // Ambiguous or malformed: follow the majority; before any votes assume the FCS is included.
  if (s_fcsExcluded > s_fcsIncluded) return bodyLen;
  return bodyLen >= 4 ? (uint16_t)(bodyLen - 4) : bodyLen;
}

void addVendor(IeInfo &out, uint32_t key) {
  for (uint8_t i = 0; i < out.nVie; i++) {
    if (out.vie[i] == key) return;
  }
  if (out.nVie < cfg::kMaxVendorIes) out.vie[out.nVie++] = key;
}

// WPS attributes: type u16 BE, length u16 BE, value.
void parseWps(const uint8_t *d, uint16_t n, WpsInfo &w) {
  uint32_t p = 0;
  while (p + 4 <= n) {
    const uint16_t t = be16(d + p);
    const uint16_t l = be16(d + p + 2);
    if (p + 4 + l > n) break;
    const uint8_t *v = d + p + 4;
    switch (t) {
      case 0x1054:  // Primary Device Type: category u16, OUI 00 50 F2 04, subcategory u16
        if (l >= 8) {
          w.hasType = true;
          w.cat = be16(v);
          w.sub = be16(v + 6);
        }
        break;
      case 0x1011: w.name.assign(v, l); break;
      case 0x1021: w.mfr.assign(v, l); break;
      case 0x1023: w.model.assign(v, l); break;
      case 0x1024: w.modelNo.assign(v, l); break;
      default: break;  // 0x1047 UUID-E and 0x1042 Serial Number are skipped on purpose
    }
    p += 4 + l;
  }
}

// P2P attributes: id u8, length u16 LE, value. Device Info (13): device address 6, config methods 2,
// primary device type 8, number of secondary types 1, secondary types 8 x n, then the device name as
// a WPS TLV (0x1011, BE length).
void parseP2p(const uint8_t *d, uint16_t n, P2pDevInfo &out) {
  uint32_t p = 0;
  while (p + 3 <= n) {
    const uint8_t id = d[p];
    const uint16_t l = le16(d + p + 1);
    if (p + 3 + l > n) break;
    const uint8_t *v = d + p + 3;
    if (id == 13 && l >= 17) {
      out.has = true;
      out.cat = be16(v + 8);
      out.sub = be16(v + 14);
      const uint32_t q = 17u + 8u * v[16];
      if (q + 4 <= l && be16(v + q) == 0x1011) {
        uint32_t nl = be16(v + q + 2);
        if (q + 4 + nl > l) nl = l - (q + 4);
        out.name.assign(v + q + 4, nl);
      }
    }
    p += 3 + l;
  }
}

}  // namespace

void Str32::assign(const uint8_t *p, size_t n) {
  if (n > cfg::kStrMax) n = cfg::kStrMax;
  memcpy(b, p, n);
  len = (uint8_t)n;
}

bool Str32::same(const Str32 &o) const { return len == o.len && memcmp(b, o.b, len) == 0; }

bool parseMgmtBody(uint8_t fc0, const uint8_t *d, uint16_t n, uint16_t bodyLen, bool exact, IeInfo &out) {
  out = IeInfo();

  if (fc0 == 0xD0) {  // ESP-NOW: category 127, OUI 18 FE 34, 4 random bytes, element 221 len OUI 18 FE 34 type 4
    if (n >= 14 && d[0] == 127 && d[1] == 0x18 && d[2] == 0xFE && d[3] == 0x34 && d[8] == 221 &&
        d[10] == 0x18 && d[11] == 0xFE && d[12] == 0x34 && d[13] == 0x04) {
      out.espnow = true;
      return true;
    }
    return false;
  }

  const int start = elementStart(fc0);
  if (start < 0 || n < (uint16_t)start) return false;
  if (fc0 == 0x80 || fc0 == 0x50) {
    out.hasCap = true;
    out.privacy = (d[10] & 0x10) != 0;  // capability LE16 at body[10], bit 4
  }
  const uint16_t end = elementEnd(d, (uint16_t)start, n, bodyLen, exact);

  uint16_t wpsN = 0, p2pN = 0;
  uint32_t p = (uint32_t)start;
  while (p + 2 <= end) {
    const uint8_t id = d[p];
    const uint8_t l = d[p + 1];
    if (p + 2 + l > end) break;
    const uint8_t *v = d + p + 2;
    switch (id) {
      case 0:  // SSID: first one only
        if (!out.hasSsid) {
          out.hasSsid = true;
          bool allZero = true;
          for (uint8_t i = 0; i < l; i++) {
            if (v[i] != 0) {
              allZero = false;
              break;
            }
          }
          out.hidden = (l == 0) || allZero;
          if (!out.hidden) out.ssid.assign(v, l);
        }
        break;
      case 3:  // DS Parameter Set: the AP's real channel
        if (l >= 1 && out.dsCh == 0 && v[0] >= 1 && v[0] <= 14) out.dsCh = v[0];
        break;
      case 45:  // HT Capabilities: cap info LE16 (bit 1 = 40 MHz), A-MPDU 1, MCS set from byte 3
        if (l >= 7) {
          out.hasHt = true;
          out.ht40 = (v[0] & 0x02) != 0;
          uint8_t streams = 0;
          for (int i = 3; i < 7; i++) {
            if (v[i]) streams++;
          }
          out.htStreams = streams;
        }
        break;
      case 48: out.rsn = true; break;
      case 107: out.interworking = true; break;
      case 127: out.extcap = true; break;
      case 191: out.vht = true; break;
      case 255:
        if (l >= 1 && v[0] == 35) out.he = true;
        break;
      case 221:  // Vendor Specific: OUI 3 + type 1 + data
        if (l >= 4) {
          const uint32_t key = ((uint32_t)v[0] << 24) | ((uint32_t)v[1] << 16) | ((uint32_t)v[2] << 8) | v[3];
          addVendor(out, key);
          if (key == 0x0050F204) {
            append(s_wpsBuf, wpsN, sizeof(s_wpsBuf), v + 4, l - 4);
          } else if (key == 0x506F9A09) {
            out.p2p = true;
            append(s_p2pBuf, p2pN, sizeof(s_p2pBuf), v + 4, l - 4);
          }
        }
        break;
      default:
        break;
    }
    p += 2 + l;
  }
  if (wpsN) parseWps(s_wpsBuf, wpsN, out.wps);
  if (p2pN) parseP2p(s_p2pBuf, p2pN, out.p2pDev);
  return true;
}

FcsStats fcsStats() { return FcsStats{s_fcsIncluded, s_fcsExcluded, s_exactEndOk, s_exactEndOff}; }
