#include "sources.h"

#include <Arduino.h>

#include <math.h>
#include <string.h>

#include "led.h"
#include "out.h"

namespace {

Source s_tab[cfg::kMaxSources];
TableStats s_stats = {};
size_t s_lastHit = 0;
IeInfo s_ie;           // parse scratch
char s_metaLine[2304]; // worst case M line is ~1.7 KB (six 32-byte strings, each byte escaped to 6)

inline bool isGroup(const uint8_t *m) { return (m[0] & 0x01) != 0; }
inline bool isZero(const uint8_t *m) { return (m[0] | m[1] | m[2] | m[3] | m[4] | m[5]) == 0; }
inline bool usable(const uint8_t *m) { return !isGroup(m) && !isZero(m); }
inline bool sameMac(const uint8_t *a, const uint8_t *b) { return memcmp(a, b, 6) == 0; }

Source *find(const uint8_t *mac) {
  Source &hint = s_tab[s_lastHit];
  if (hint.used && sameMac(hint.mac, mac)) return &hint;
  for (size_t i = 0; i < cfg::kMaxSources; i++) {
    if (s_tab[i].used && sameMac(s_tab[i].mac, mac)) {
      s_lastHit = i;
      return &s_tab[i];
    }
  }
  return nullptr;
}

// Existing entry, a free one, or the least recently heard entry with nothing pending in this window.
Source *findOrCreate(const uint8_t *mac, uint32_t now) {
  if (Source *s = find(mac)) return s;
  Source *slot = nullptr;
  for (Source &e : s_tab) {
    if (!e.used) {
      slot = &e;
      break;
    }
  }
  if (slot == nullptr) {
    for (Source &e : s_tab) {
      if (e.n != 0) continue;
      if (slot == nullptr || (int32_t)(e.lastSeen - slot->lastSeen) < 0) slot = &e;
    }
    if (slot == nullptr) return nullptr;
    s_stats.evictions++;
  }
  *slot = Source();
  slot->used = true;
  memcpy(slot->mac, mac, 6);
  slot->firstSeen = now;
  slot->lastSeen = now;
  slot->metaDirty = true;
  s_lastHit = (size_t)(slot - s_tab);
  return slot;
}

void markClientsDirty(const uint8_t *apMac) {
  for (Source &e : s_tab) {
    if (e.used && !e.isAp && e.hasBssid && sameMac(e.bssid, apMac)) e.metaDirty = true;
  }
}

void setAp(Source &s) {
  if (s.isAp) return;
  s.isAp = true;
  ledNewNetwork(millis());
  s.hasBssid = false;  // "bssid" is the client's AP; an AP is its own BSSID
  s.metaDirty = true;
}

void linkClient(Source &c, const uint8_t *ap) {
  if (c.isAp || !usable(ap) || sameMac(ap, c.mac)) return;
  if (c.hasBssid && sameMac(c.bssid, ap)) return;
  memcpy(c.bssid, ap, 6);
  c.hasBssid = true;
  c.metaDirty = true;
}

// The channel a source is heard best on. While hopping, a strong station also leaks into the
// neighbouring channels, so a new channel has to beat the current one by 3 dB (or the current one
// has gone quiet for 5 s). APs prefer their DS Parameter Set channel over this.
void noteRxChannel(Source &s, uint8_t ch, int8_t rssi, uint32_t now) {
  if (ch < 1 || ch > 14) return;
  if (s.rxCh == 0) {
    s.rxCh = ch;
    s.rxChRssi = rssi;
    s.rxChAt = now;
    s.metaDirty = true;
    return;
  }
  if (ch == s.rxCh) {
    if (rssi > s.rxChRssi) s.rxChRssi = rssi;
    s.rxChAt = now;
    return;
  }
  if ((int)rssi > (int)s.rxChRssi + 3 || (uint32_t)(now - s.rxChAt) > 5000) {
    s.rxCh = ch;
    s.rxChRssi = rssi;
    s.rxChAt = now;
    s.metaDirty = true;
  }
}

uint8_t metaChannel(const Source &s) {
  if (s.isAp) return s.dsCh ? s.dsCh : s.rxCh;
  if (s.hasBssid) {
    const Source *ap = find(s.bssid);
    if (ap != nullptr && ap->dsCh) return ap->dsCh;
  }
  return s.rxCh;
}

void quotedMac(LineBuf &o, const uint8_t *m) {
  o.ch('"');
  o.mac(m);
  o.ch('"');
}

void buildMetaJson(const Source &s, LineBuf &o) {
  JsonObj j(o);
  j.key("kind").str(s.isAp ? "\"ap\"" : "\"client\"");
  if (s.isAp && s.hasSsid) j.key("ssid").jstr(s.ssid.b, s.ssid.len);
  if (!s.isAp && s.hasBssid) quotedMac(j.key("bssid"), s.bssid);
  j.key("ch").u32(metaChannel(s));
  j.key("la").boolean((s.mac[0] & 0x02) != 0);
  j.key("oui");
  o.ch('"');
  o.ouiUpper(s.mac);
  o.ch('"');
  if (s.isAp && s.hasSsid) j.key("hidden").boolean(s.hidden);
  if (s.wps.any()) {
    j.key("wps");
    JsonObj w(o);
    if (s.wps.hasType) {
      w.key("cat").u32(s.wps.cat);
      w.key("sub").u32(s.wps.sub);
    }
    if (s.wps.name.len) w.key("name").jstr(s.wps.name.b, s.wps.name.len);
    if (s.wps.mfr.len) w.key("mfr").jstr(s.wps.mfr.b, s.wps.mfr.len);
    if (s.wps.model.len) w.key("model").jstr(s.wps.model.b, s.wps.model.len);
    if (s.wps.modelNo.len) w.key("modelNo").jstr(s.wps.modelNo.b, s.wps.modelNo.len);
    w.close();
  }
  j.key("p2p").boolean(s.p2p);
  j.key("vie");
  o.ch('[');
  for (uint8_t i = 0; i < s.nVie; i++) {
    if (i) o.ch(',');
    const uint32_t v = s.vie[i];
    o.ch('"');
    o.hex2((uint8_t)(v >> 24));
    o.hex2((uint8_t)(v >> 16));
    o.hex2((uint8_t)(v >> 8));
    o.ch(':');
    o.hex2((uint8_t)v);
    o.ch('"');
  }
  o.ch(']');
  if (s.isAp) j.key("beaconOnly").boolean(!s.sawProbeResp);

  // Optional extras in classifier.js meta naming; present only when seen.
  if (s.isAp && s.capSeen) j.key("privacy").boolean(s.privacy);
  if (s.rsn) j.key("rsn").boolean(true);
  if (s.hasHt) {
    j.key("ht");
    JsonObj h(o);
    h.key("streams").u32(s.htStreams);
    h.key("ht40").boolean(s.ht40);
    h.close();
  }
  if (s.vht) j.key("vht").boolean(true);
  if (s.he) j.key("he").boolean(true);
  if (s.extcap) j.key("extcap").boolean(true);
  if (s.interworking) j.key("interworking").boolean(true);
  if (s.espnow) j.key("espnow").boolean(true);
  if (s.p2pDev.has) {
    j.key("p2pdev");
    JsonObj p(o);
    p.key("cat").u32(s.p2pDev.cat);
    p.key("sub").u32(s.p2pDev.sub);
    if (s.p2pDev.name.len) p.key("name").jstr(s.p2pDev.name.b, s.p2pDev.name.len);
    p.close();
  }
  j.close();
}

// Returns true when a line went to the wire (sent or dropped), false when nothing had changed.
bool emitMeta(Source &s, uint32_t now) {
  LineBuf o(s_metaLine, sizeof(s_metaLine));
  o.str("M,");
  o.mac(s.mac);
  o.ch(',');
  const size_t jsonStart = o.size();
  buildMetaJson(s, o);
  const size_t jsonEnd = o.size();
  o.ch('\n');
  if (!o.ok()) {  // cannot happen with the buffer sized for the worst case; never send half a line
    s.metaDirty = false;
    s.metaResend = false;
    return false;
  }
  const uint32_t h = fnv1a(o.data() + jsonStart, jsonEnd - jsonStart);
  if (s.metaSent && h == s.metaHash && !s.metaResend) {
    s.metaDirty = false;
    return false;
  }
  if (!wireSend(o)) return true;  // USB buffer full: stay dirty, retry next window
  s.metaSent = true;
  s.metaHash = h;
  s.metaAt = now;
  s.metaDirty = false;
  s.metaResend = false;
  s_stats.metaSent++;
  return true;
}

inline void orFlag(bool &dst, bool src, bool &changed) {
  if (src && !dst) {
    dst = true;
    changed = true;
  }
}

inline void mergeStr(Str32 &dst, const Str32 &src, bool &changed) {
  if (src.len && !dst.same(src)) {
    dst = src;
    changed = true;
  }
}

}  // namespace

void sourcesOnFrame(const FrameRec &f, uint32_t now) {
  const uint8_t ftype = (f.fc0 >> 2) & 0x03;  // 0 mgmt, 2 data
  if (ftype != 0 && ftype != 2) return;
  if (!usable(f.a2)) return;
  const bool toDs = (f.fc1 & 0x01) != 0;
  const bool fromDs = (f.fc1 & 0x02) != 0;

  // The AP this frame belongs to, from the header alone.
  const uint8_t *bssid = nullptr;
  if (ftype == 0) {
    if (f.fc0 != 0x40 && usable(f.a3)) bssid = f.a3;  // probe requests: wildcard / directed, not "belonging"
  } else if (toDs && !fromDs) {
    bssid = f.a1;  // client -> AP
  } else if (!toDs && fromDs) {
    bssid = f.a2;  // AP -> client
  } else if (!toDs && !fromDs) {
    if (usable(f.a3)) bssid = f.a3;  // IBSS / direct link
  }                                   // ToDS + FromDS: WDS / mesh, no single BSSID
  if (bssid != nullptr && !usable(bssid)) bssid = nullptr;

  Source *s = findOrCreate(f.a2, now);
  if (s == nullptr) {
    s_stats.tableDrops++;
    return;
  }
  s->lastSeen = now;

  if (s->n == 0) {
    s->rssiSum = 0;
    s->rssiMax = f.rssi;
    s->bytes = 0;
    s->aggHasBssid = false;
  }
  if (s->n < 0xFFFF) {  // n, sum and bytes stay consistent (avg = sum / n) even at the cap
    s->n++;
    s->rssiSum += f.rssi;
    s->bytes += f.len;
  }
  if (f.rssi > s->rssiMax) s->rssiMax = f.rssi;
  s->aggCh = f.ch;
  if (bssid != nullptr) {
    memcpy(s->aggBssid, bssid, 6);
    s->aggHasBssid = true;
  }

  // Role: beacons / probe responses, or the BSSID side of a FromDS data frame.
  if (ftype == 0 && (f.fc0 == 0x80 || f.fc0 == 0x50)) setAp(*s);
  else if (ftype == 2 && fromDs && !toDs) setAp(*s);

  noteRxChannel(*s, f.ch, f.rssi, now);

  // Client -> AP links.
  if (!s->isAp) {
    if (ftype == 2 && toDs && !fromDs) {
      linkClient(*s, f.a1);
    } else if (ftype == 0 && f.fc0 != 0x40 && bssid != nullptr && !sameMac(bssid, f.a2)) {
      linkClient(*s, bssid);  // auth / (re)assoc / action / deauth from a station: addr3 = its BSSID
    }
  }
  if (ftype == 2 && fromDs && !toDs && usable(f.a1)) {
    Source *c = find(f.a1);
    if (c != nullptr) linkClient(*c, f.a2);
  }
}

void sourcesOnIe(const IeRec &r, uint32_t now) {
  if (!usable(r.src)) return;
  if (!parseMgmtBody(r.fc0, r.body, r.n, r.bodyLen, r.exact != 0, s_ie)) return;
  Source *s = findOrCreate(r.src, now);
  if (s == nullptr) return;  // the frame itself was already counted by sourcesOnFrame
  const IeInfo &ie = s_ie;
  const bool apFrame = (r.fc0 == 0x80 || r.fc0 == 0x50);
  bool d = false;

  if (apFrame) setAp(*s);
  if (r.fc0 == 0x50 && !s->sawProbeResp) {
    s->sawProbeResp = true;
    d = true;
  }
  if (apFrame && ie.hasSsid) {
    if (!s->hasSsid) {
      s->hasSsid = true;
      d = true;
    }
    if (r.fc0 == 0x80 && s->hidden != ie.hidden) {  // only beacons say whether the SSID is hidden
      s->hidden = ie.hidden;
      d = true;
    }
    if (!ie.hidden && !s->ssid.same(ie.ssid)) {  // a probe response may reveal a hidden SSID
      s->ssid = ie.ssid;
      d = true;
    }
  }
  if (apFrame && ie.dsCh && ie.dsCh != s->dsCh) {
    s->dsCh = ie.dsCh;
    d = true;
    markClientsDirty(s->mac);
  }
  if (apFrame && ie.hasCap && (!s->capSeen || s->privacy != ie.privacy)) {
    s->capSeen = true;
    s->privacy = ie.privacy;
    d = true;
  }
  if ((r.fc0 == 0x00 || r.fc0 == 0x20) && usable(r.a3)) linkClient(*s, r.a3);

  // Merge-only: a frame that lacks a field never clears it (beacons carry less WPS than probe responses).
  orFlag(s->rsn, ie.rsn, d);
  orFlag(s->interworking, ie.interworking, d);
  orFlag(s->extcap, ie.extcap, d);
  orFlag(s->vht, ie.vht, d);
  orFlag(s->he, ie.he, d);
  orFlag(s->p2p, ie.p2p, d);
  orFlag(s->espnow, ie.espnow, d);
  if (ie.hasHt && (!s->hasHt || s->htStreams != ie.htStreams || s->ht40 != ie.ht40)) {
    s->hasHt = true;
    s->htStreams = ie.htStreams;
    s->ht40 = ie.ht40;
    d = true;
  }
  for (uint8_t i = 0; i < ie.nVie; i++) {
    bool known = false;
    for (uint8_t k = 0; k < s->nVie; k++) {
      if (s->vie[k] == ie.vie[i]) {
        known = true;
        break;
      }
    }
    if (!known && s->nVie < cfg::kMaxVendorIes) {
      s->vie[s->nVie++] = ie.vie[i];
      d = true;
    }
  }
  if (ie.wps.hasType && (!s->wps.hasType || s->wps.cat != ie.wps.cat || s->wps.sub != ie.wps.sub)) {
    s->wps.hasType = true;
    s->wps.cat = ie.wps.cat;
    s->wps.sub = ie.wps.sub;
    d = true;
  }
  mergeStr(s->wps.name, ie.wps.name, d);
  mergeStr(s->wps.mfr, ie.wps.mfr, d);
  mergeStr(s->wps.model, ie.wps.model, d);
  mergeStr(s->wps.modelNo, ie.wps.modelNo, d);
  if (ie.p2pDev.has && (!s->p2pDev.has || s->p2pDev.cat != ie.p2pDev.cat || s->p2pDev.sub != ie.p2pDev.sub)) {
    s->p2pDev.has = true;
    s->p2pDev.cat = ie.p2pDev.cat;
    s->p2pDev.sub = ie.p2pDev.sub;
    d = true;
  }
  mergeStr(s->p2pDev.name, ie.p2pDev.name, d);
  if (d) s->metaDirty = true;
}

void sourcesFlush(uint32_t now) {
  // 1. Identity first: sources never announced, then changed / resent ones (changes rate-limited per
  //    source). A source heard in the last minute whose M is a minute old repeats it, so a line lost
  //    on the host side (read error, page reload) is repaired without a V.
  uint8_t budget = cfg::kMetaPerFlush;
  for (int pass = 0; pass < 2 && budget > 0; pass++) {
    for (Source &s : s_tab) {
      if (budget == 0) break;
      if (!s.used) continue;
      if (s.metaSent && !s.metaResend && (uint32_t)(now - s.metaAt) >= cfg::kMetaRefreshMs &&
          (uint32_t)(now - s.lastSeen) < cfg::kMetaRefreshMs) {
        s.metaResend = true;
      }
      if (!s.metaDirty && !s.metaResend) continue;
      if (s.metaSent != (pass == 1)) continue;
      if (s.metaSent && !s.metaResend && (uint32_t)(now - s.metaAt) < cfg::kMetaMinIntervalMs) continue;
      if (emitMeta(s, now)) budget--;
    }
  }

  // 2. One P line per source heard in this window. A source whose first M is still queued keeps
  //    aggregating (up to kFirstMetaHoldMs) so the page normally sees M before the first P.
  char buf[96];
  for (Source &s : s_tab) {
    if (!s.used || s.n == 0) continue;
    if (!s.metaSent && (uint32_t)(now - s.firstSeen) < cfg::kFirstMetaHoldMs) continue;
    LineBuf o(buf, sizeof(buf));
    o.str("P,");
    o.u32(now);
    o.ch(',');
    o.mac(s.mac);
    o.ch(',');
    o.u32(s.n);
    o.ch(',');
    o.i32((int32_t)lroundf((float)s.rssiSum / (float)s.n));
    o.ch(',');
    o.i32(s.rssiMax);
    o.ch(',');
    o.u32(s.aggCh);
    o.ch(',');
    o.u32(s.bytes);
    o.ch(',');
    if (s.aggHasBssid) o.mac(s.aggBssid);
    else o.ch('-');
    o.ch('\n');
    if (!wireSend(o)) s_stats.pFramesDropped += s.n;
    s.n = 0;
  }
}

void sourcesResendMeta() {
  for (Source &s : s_tab) {
    if (s.used && s.metaSent) s.metaResend = true;  // never-sent ones are dirty already
  }
}

uint16_t sourcesBusyChannels(uint32_t now, uint32_t forMs, int8_t minRssi) {
  uint16_t m = 0;
  for (const Source &s : s_tab) {
    if (!s.used || !s.isAp || (uint32_t)(now - s.lastSeen) > forMs || s.rxChRssi < minRssi) continue;
    const uint8_t ch = s.dsCh ? s.dsCh : s.rxCh;
    if (ch >= cfg::kChanFirst && ch <= cfg::kChanLast) m |= (uint16_t)(1u << ch);
  }
  return m;
}

TableStats sourcesStats() {
  TableStats t = s_stats;
  t.active = 0;
  for (const Source &s : s_tab) {
    if (s.used) t.active++;
  }
  return t;
}
