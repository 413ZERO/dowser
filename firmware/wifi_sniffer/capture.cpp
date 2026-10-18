#include "capture.h"

#include <string.h>

#include <WiFi.h>

#include "esp_wifi.h"

SpscRing<FrameRec, cfg::kFrameRingSlots> g_frameRing;
SpscRing<IeRec, cfg::kIeRingSlots> g_ieRing;
CaptureCounters g_cap;

namespace {

// Per (MAC, subtype) time of the last body copy. Touched only by the Wi-Fi task.
struct ThrottleSlot {
  uint8_t mac[6];
  uint8_t fc0;
  uint8_t used;
  uint32_t lastUs;
};
ThrottleSlot s_throttle[cfg::kIeThrottleSlots];

constexpr std::memory_order kRelaxed = std::memory_order_relaxed;

// Management frames whose body we parse. p points at Frame Control, len = sig_len (>= 24).
inline bool carriesElements(const uint8_t *p, uint32_t len) {
  switch (p[0]) {
    case 0x80:  // beacon
    case 0x50:  // probe response
    case 0x40:  // probe request
    case 0x00:  // association request
    case 0x20:  // reassociation request
      return true;
    case 0xD0:  // action: only vendor-specific (127) from Espressif's OUI, i.e. ESP-NOW candidates
      return len >= 28 && p[24] == 127 && p[25] == 0x18 && p[26] == 0xFE && p[27] == 0x34;
    default:
      return false;
  }
}

// The slot for this (MAC, subtype), or nullptr if its body was copied less than kIeThrottleUs ago.
inline ThrottleSlot *throttleSlot(const uint8_t *mac, uint8_t fc0, uint32_t nowUs) {
  uint32_t h = 2166136261u;  // FNV-1a
  for (int i = 0; i < 6; i++) h = (h ^ mac[i]) * 16777619u;
  h = (h ^ fc0) * 16777619u;
  ThrottleSlot *s = &s_throttle[h & (cfg::kIeThrottleSlots - 1)];
  if (s->used && s->fc0 == fc0 && memcmp(s->mac, mac, 6) == 0 &&
      (uint32_t)(nowUs - s->lastUs) < cfg::kIeThrottleUs) {
    return nullptr;
  }
  return s;
}

// Runs in the Wi-Fi driver task: copy, count, return. No printing, no allocation, no locks.
void rxCallback(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA) return;
  const wifi_promiscuous_pkt_t *pkt = static_cast<const wifi_promiscuous_pkt_t *>(buf);
  const wifi_pkt_rx_ctrl_t &rx = pkt->rx_ctrl;
  if (rx.rx_state != 0) {
    g_cap.badState.fetch_add(1, kRelaxed);
    return;
  }
  // Frame length without the FCS. The C6's rx_ctrl (esp_wifi_he_types.h, MAC version 2 branch) has
  // sig_len = "the length of the reception MPDU" (an MPDU ends with the 4-byte FCS) and dump_len = "the
  // length of the reception MPDU excluding the FCS". Use dump_len when it is plausible: it is also the
  // tighter bound on what payload[] holds. Otherwise fall back on sig_len and let ieparse vote.
  const uint32_t sigLen = rx.sig_len;
  const uint32_t dumpLen = rx.dump_len;
  const bool exact = dumpLen != 0 && dumpLen <= sigLen;
  const uint32_t len = exact ? dumpLen : sigLen;
  if (len < 24) {
    g_cap.runt.fetch_add(1, kRelaxed);
    return;
  }
  g_cap.frames.fetch_add(1, kRelaxed);
  if (exact) {
    g_cap.lenDump.fetch_add(1, kRelaxed);
    if (sigLen - dumpLen == 4) g_cap.lenDumpFcs.fetch_add(1, kRelaxed);
  } else {
    g_cap.lenSig.fetch_add(1, kRelaxed);
  }

  const uint8_t *p = pkt->payload;
  const int8_t rssi = rx.rssi > 0 ? (int8_t)0 : rx.rssi < -127 ? (int8_t)-127 : (int8_t)rx.rssi;
  const int8_t noise = rx.noise_floor;
  const uint8_t ch = rx.channel;
  g_cap.noiseSum.fetch_add(noise, kRelaxed);
  g_cap.noiseN.fetch_add(1, kRelaxed);

  FrameRec *f = g_frameRing.claim();
  if (f == nullptr) {
    g_cap.ringDrop.fetch_add(1, kRelaxed);
    return;
  }
  memcpy(f->a1, p + 4, 6);
  memcpy(f->a2, p + 10, 6);
  memcpy(f->a3, p + 16, 6);
  f->fc0 = p[0];
  f->fc1 = p[1];
  f->rssi = rssi;
  f->noise = noise;
  f->ch = ch;
  f->pad = 0;
  f->len = (uint16_t)sigLen;
  g_frameRing.publish();

  if (type != WIFI_PKT_MGMT || !carriesElements(p, len)) return;
  const uint32_t nowUs = rx.timestamp;
  ThrottleSlot *slot = throttleSlot(p + 10, p[0], nowUs);
  if (slot == nullptr) return;  // parsed this source's body < 1 s ago
  IeRec *r = g_ieRing.claim();
  if (r == nullptr) {
    g_cap.ieDrop.fetch_add(1, kRelaxed);
    return;
  }
  const uint32_t bodyLen = len - 24;
  const uint32_t n = bodyLen < cfg::kIeMaxBytes ? bodyLen : cfg::kIeMaxBytes;
  memcpy(r->src, p + 10, 6);
  memcpy(r->a3, p + 16, 6);
  r->fc0 = p[0];
  r->ch = ch;
  r->rssi = rssi;
  r->exact = exact ? 1 : 0;
  r->bodyLen = (uint16_t)bodyLen;
  r->n = (uint16_t)n;
  memcpy(r->body, p + 24, n);
  g_ieRing.publish();
  g_cap.ieCopied.fetch_add(1, kRelaxed);

  memcpy(slot->mac, p + 10, 6);
  slot->fc0 = p[0];
  slot->used = 1;
  slot->lastUs = nowUs;
}

}  // namespace

esp_err_t captureBegin(uint8_t firstChannel, const char **failedStep) {
  esp_err_t err = ESP_OK;
#define SNIFF_TRY(call)        \
  do {                         \
    err = (call);              \
    if (err != ESP_OK) {       \
      *failedStep = #call;     \
      return err;              \
    }                          \
  } while (0)

  // No network, no credentials: keep the driver config in RAM (nothing written to NVS), never connect.
  // No WiFi.disconnect(): we are never connected, and it is the one call that raises
  // STA_DISCONNECTED, whose first-time handler in the Arduino core (STA.cpp, `first_connect`) calls
  // esp_wifi_connect() for any reason other than ASSOC_LEAVE, which would scan (probe requests).
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  WiFi.setSleep(false);
  if (!WiFi.mode(WIFI_STA)) {
    *failedStep = "WiFi.mode(WIFI_STA)";
    return ESP_FAIL;
  }
  // esp_wifi_init() still loads a station config an older sketch saved in NVS (nvs_enable stays on).
  // Blank it in RAM only (storage is RAM after persistent(false)), so nothing could ever join that
  // network. Best effort: sniffing does not depend on it.
  wifi_config_t blank = {};
  (void)esp_wifi_set_config(WIFI_IF_STA, &blank);
  SNIFF_TRY(esp_wifi_set_ps(WIFI_PS_NONE));

  wifi_country_t country = {};
  memcpy(country.cc, "GB", 3);
  country.schan = cfg::kChanFirst;
  country.nchan = cfg::kChanLast - cfg::kChanFirst + 1;
  country.max_tx_power = 20;  // read-only on set; the sniffer never transmits on purpose
  country.policy = WIFI_COUNTRY_POLICY_MANUAL;
  SNIFF_TRY(esp_wifi_set_country(&country));

  wifi_promiscuous_filter_t filter = {};
  filter.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA;
  SNIFF_TRY(esp_wifi_set_promiscuous_filter(&filter));
  SNIFF_TRY(esp_wifi_set_promiscuous_rx_cb(&rxCallback));
  SNIFF_TRY(esp_wifi_set_promiscuous(true));
  SNIFF_TRY(esp_wifi_set_channel(firstChannel, WIFI_SECOND_CHAN_NONE));
#undef SNIFF_TRY
  *failedStep = nullptr;
  return ESP_OK;
}

esp_err_t captureSetChannel(uint8_t ch) {
  return esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
}
