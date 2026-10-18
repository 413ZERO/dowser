/*
 * wifi_sniffer: passive 2.4 GHz "signal finder" for the Seeed XIAO ESP32C6.
 *
 * Listens in promiscuous mode (management + data frames, channels 1-13, country GB) without joining
 * any network, and streams newline-terminated ASCII lines over native USB to a Chrome page
 * (Web Serial). Protocol and limits: README.md in this folder.
 *
 *   ESP32 -> page   H heartbeat (1 Hz) · P per-source aggregate (25 ms) · M source metadata · I info
 *                   · D dwell ended（0.3.0：每次驻留结束 / 锁定时每 ≥ 110 ms 一行，见 endDwell()）
 *   page -> ESP32   L,<ch> lock · U unlock (hop) · V version / config
 *
 * Board: FQBN esp32:esp32:XIAO_ESP32C6 (its initVariant() powers the RF switch and keeps GPIO14
 * WIFI_ANT_CONFIG LOW = onboard ceramic antenna). This sketch never touches GPIO3 / GPIO14.
 *
 * Files: capture.* (radio + promiscuous callback -> lock-free rings), ieparse.* (elements, WPS, P2P),
 * sources.* (source table, P / M lines), out.* (line building, non-blocking USB writes).
 */
#include <Arduino.h>
#include <WiFi.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_idf_version.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_system.h"

#include "capture.h"
#include "config.h"
#include "ieparse.h"
#include "led.h"
#include "out.h"
#include "sources.h"

// ---------------------------------------------------------------------------------------------
// USB reset guard. Unconfirmed esp-idf issue #19223: heap corruption after a USB line-state reset
// while promiscuous. If this boot was caused by the USB-Serial/JTAG peripheral (host toggled
// DTR/RTS), do one clean software restart before Wi-Fi starts. The marker lives in RTC memory that is
// not cleared by a software reset, so a second USB-reset boot right after it does not loop.
static RTC_NOINIT_ATTR uint32_t s_usbRestartMarker;
static constexpr uint32_t kUsbRestartMarker = 0x19223A5Au;
static esp_reset_reason_t s_bootReason = ESP_RST_UNKNOWN;
static bool s_restartedAfterUsbReset = false;

static void usbResetGuard() {
  s_bootReason = esp_reset_reason();
  const bool usbReset = (s_bootReason == ESP_RST_USB || s_bootReason == ESP_RST_JTAG);
  if (usbReset && s_usbRestartMarker != kUsbRestartMarker) {
    s_usbRestartMarker = kUsbRestartMarker;
    esp_restart();  // does not return
  }
  s_restartedAfterUsbReset = (s_usbRestartMarker == kUsbRestartMarker);
  s_usbRestartMarker = 0;
}

static const char *resetName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "poweron";
    case ESP_RST_EXT: return "ext";
    case ESP_RST_SW: return "sw";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "int_wdt";
    case ESP_RST_TASK_WDT: return "task_wdt";
    case ESP_RST_WDT: return "wdt";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
    case ESP_RST_USB: return "usb";
    case ESP_RST_JTAG: return "jtag";
    case ESP_RST_EFUSE: return "efuse";
    case ESP_RST_PWR_GLITCH: return "pwr_glitch";
    case ESP_RST_CPU_LOCKUP: return "cpu_lockup";
    default: return "unknown";
  }
}

// ---------------------------------------------------------------------------------------------
// State (loop task only).
static bool s_radioOk = false;
static const char *s_radioStep = nullptr;
static esp_err_t s_radioErr = ESP_OK;
static bool s_locked = false;
static uint8_t s_channel = cfg::kChanFirst;  // channel the radio is on
static uint16_t s_busy = 0;                  // busy channels (bit ch), refreshed each hop
// 0.3.0 跳频（v3）和 D 行的状态
static uint32_t s_curDwell = cfg::kDwellMs;  // 这一格驻留多久：忙信道 kDwellMs，空信道 / 全空顺序扫 kQuietDwellMs
static uint32_t s_dwellStartMs = 0;          // 这次驻留从什么时候开始（D 行的驻留时长）
static uint32_t s_dLines = 0;
static constexpr int64_t kNever = INT64_MIN / 4;
static int64_t s_lastStart[cfg::kChanLast + 1]; // 每个信道最近一次驻留开始的时刻（64 位毫秒，不回绕）；kNever = 没去过
static double s_nextQuiet = 0;               // 下一个空信道格最早什么时候（64 位毫秒）
static uint8_t s_bp = 0;                     // 忙信道轮转：上一次去的忙信道
static uint32_t s_chanErrors = 0;
static uint32_t s_lastHopMs = 0;
static uint32_t s_windowStartMs = 0;
static uint32_t s_lastBeatMs = 0;

static void infof(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void infof(const char *fmt, ...) {
  char text[220];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(text, sizeof(text), fmt, ap);
  va_end(ap);
  wireInfo(text);
}

static int64_t nowMs64() { return (int64_t)(esp_timer_get_time() / 1000); } // 跳频调度用：和 millis() 同一个时基，不回绕

// 两个环清空（帧 -> 源表聚合；管理帧体 -> IE 解析）。0.3.0：loop() 先清环再跳频，D 行之前的 P 行就是这次驻留的全部。
static void drainRings(uint32_t now) {
  for (uint32_t i = 0; i < cfg::kFrameRingSlots; i++) {
    const FrameRec *f = g_frameRing.peek();
    if (f == nullptr) break;
    ledOnFrame(f->a2, f->rssi, now);
    sourcesOnFrame(*f, now);
    g_frameRing.pop();
  }
  for (int i = 0; i < 8; i++) {
    const IeRec *r = g_ieRing.peek();
    if (r == nullptr) break;
    sourcesOnIe(*r, now);
    g_ieRing.pop();
  }
}

// D,<ms>,<信道>,<驻留 ms>：一次驻留结束（这次驻留的 P 行都已经发出去了）。页面（serial-source.js 0.3）用它给样本结案，
// 并给这个信道上"这次驻留一帧都没听到"的 AP 记一次没听到（连续 K 次 = 离开 / 关机）。约 9 行/s、每行约 16 字节。
static void emitDwell(uint32_t now) {
  char buf[40];
  LineBuf o(buf, sizeof(buf));
  o.str("D,");
  o.u32(now);
  o.ch(',');
  o.u32(s_channel);
  o.ch(',');
  o.u32(now - s_dwellStartMs);
  o.ch('\n');
  wireSend(o);
  s_dwellStartMs = now;
  s_dLines++;
}

// 换信道之前：这次驻留的数据先刷出去（sourcesFlush），再发 D 行。
static void endDwell(uint32_t now) {
  if (!cfg::kEmitDwellLines) return;
  sourcesFlush(now);
  s_windowStartMs = now;
  emitDwell(now);
}

static bool tuneTo(uint8_t ch, uint32_t now) {
  endDwell(now);
  if (ch == s_channel) return true;  // 0.3.0：同一个信道不重新调台（不白白多一个切信道空窗）
  if (captureSetChannel(ch) != ESP_OK) {
    s_chanErrors++;
    return false;
  }
  s_channel = ch;
  return true;
}

// 0.3.0 跳频（压力测试 stress-final/firmware.js 的 hopV3，逐行对应）：
//   忙信道（最近 kBusyForMs 内听到过 ≥ kBusyMinRssi 的 AP）轮流去，每格 kDwellMs；
//   空信道按截止时间 lastStart[c] + T − dq 排序（没去过的最急），pace = (T − dq) / 空信道数；
//   到点（now ≥ nextQuiet）、或者"现在去忙信道的话后面的空信道就赶不上截止"时插一个空信道，驻留 dq，
//   然后 nextQuiet = min(now + pace, max(nextQuiet + pace, now + pace − dq))（平均间隔 pace，落后太多不补）；
//   一个忙信道都没有：按截止顺序扫，每格 dq。T = kQuietPeriodMs，dq = kQuietDwellMs。
static void hopNext(uint32_t now) {
  const int64_t t = nowMs64();
  s_busy = sourcesBusyChannels(now, cfg::kBusyForMs, cfg::kBusyMinRssi);
  const int64_t T = cfg::kQuietPeriodMs, dw = cfg::kDwellMs, dq = cfg::kQuietDwellMs;
  auto deadline = [&](uint8_t c) -> int64_t { return s_lastStart[c] == kNever ? kNever : s_lastStart[c] + T - dq; };
  uint8_t quiet[cfg::kChanLast];
  int nq = 0;
  for (uint8_t c = cfg::kChanFirst; c <= cfg::kChanLast; c++) {
    if (s_busy & (1u << c)) continue;
    // 插入排序：截止早的在前，同截止按信道号
    int j = nq++;
    while (j > 0 && (deadline(quiet[j - 1]) > deadline(c) || (deadline(quiet[j - 1]) == deadline(c) && quiet[j - 1] > c))) {
      quiet[j] = quiet[j - 1];
      j--;
    }
    quiet[j] = c;
  }
  const bool anyBusy = nq < cfg::kChanLast;
  bool safe = true;  // 现在去一个忙信道，后面的空信道还都赶得上各自的截止吗
  for (int j = 0; j < nq; j++) {
    if (t + dw + (int64_t)j * dq > deadline(quiet[j])) {
      safe = false;
      break;
    }
  }
  uint8_t ch = 0;
  if (!anyBusy) {  // 全空：顺序扫（每格都要能抓到 beacon）
    ch = quiet[0];
    s_curDwell = (uint32_t)dq;
  } else if (nq > 0 && ((double)t >= s_nextQuiet || !safe)) {
    ch = quiet[0];
    s_curDwell = (uint32_t)dq;
    const double pace = (double)(T - dq) / nq;
    const double a = s_nextQuiet + pace, b = (double)t + pace - (double)dq;
    s_nextQuiet = fmin((double)t + pace, a > b ? a : b);
  } else {
    for (int k = 1; k <= cfg::kChanLast; k++) {
      const uint8_t c = (uint8_t)(((s_bp + k - 1) % cfg::kChanLast) + 1);
      if (s_busy & (1u << c)) {
        ch = c;
        break;
      }
    }
    s_bp = ch;
    s_curDwell = (uint32_t)dw;
  }
  if (ch < cfg::kChanFirst || ch > cfg::kChanLast) return;  // 不会发生（有忙信道时轮转一定找得到）
  s_lastStart[ch] = t;
  tuneTo(ch, now);
}

// ---------------------------------------------------------------------------------------------
static void printVersion() {
  infof("%s %s protocol=%s built=%s_%s", SNIFFER_NAME, SNIFFER_VERSION, SNIFFER_PROTOCOL, __DATE__, __TIME__);
  infof("board=XIAO_ESP32C6 chip=esp32c6 idf=%s arduino=%s", esp_get_idf_version(), ESP_ARDUINO_VERSION_STR);
  infof("antenna=onboard (GPIO14 WIFI_ANT_CONFIG left LOW by the board variant)");
  if (s_radioOk) {
    infof("radio=ok country=GB channels=%u-%u mode=%s ch=%u dwell_ms=%lu window_ms=%lu", cfg::kChanFirst,
          cfg::kChanLast, s_locked ? "lock" : "hop", s_channel, (unsigned long)cfg::kDwellMs,
          (unsigned long)cfg::kWindowMs);
    infof("hop=v3 quiet_dwell_ms=%lu quiet_period_ms=%lu busy_for_ms=%lu busy_min_rssi=%d dwell_lines=%d d_lines=%lu",
          (unsigned long)cfg::kQuietDwellMs, (unsigned long)cfg::kQuietPeriodMs, (unsigned long)cfg::kBusyForMs,
          (int)cfg::kBusyMinRssi, cfg::kEmitDwellLines ? 1 : 0, (unsigned long)s_dLines);
  } else {
    infof("radio=error step=%s err=%s", s_radioStep ? s_radioStep : "?", esp_err_to_name(s_radioErr));
  }
  infof("buffers frame_ring=%lu ie_ring=%lux%u sources=%u usb_tx=%u", (unsigned long)cfg::kFrameRingSlots,
        (unsigned long)cfg::kIeRingSlots, cfg::kIeMaxBytes, (unsigned)cfg::kMaxSources, (unsigned)cfg::kSerialTxBuf);
  infof("reset=%s usb_restart=%d", resetName(s_bootReason), s_restartedAfterUsbReset ? 1 : 0);
  const FcsStats fs = fcsStats();
  infof("fcs dump_len=%lu sig_minus_dump_4=%lu sig_len_only=%lu ie_end_ok=%lu ie_end_off=%lu votes_yes=%lu votes_no=%lu",
        (unsigned long)g_cap.lenDump.load(std::memory_order_relaxed),
        (unsigned long)g_cap.lenDumpFcs.load(std::memory_order_relaxed),
        (unsigned long)g_cap.lenSig.load(std::memory_order_relaxed), (unsigned long)fs.exactEndOk,
        (unsigned long)fs.exactEndOff, (unsigned long)fs.votesIncluded, (unsigned long)fs.votesExcluded);
  const TableStats t = sourcesStats();
  infof("stats sources=%lu evictions=%lu meta_sent=%lu tx_dropped_lines=%lu chan_errors=%lu",
        (unsigned long)t.active, (unsigned long)t.evictions, (unsigned long)t.metaSent,
        (unsigned long)wireDroppedLines(), (unsigned long)s_chanErrors);
}

// aa:bb:cc:dd:ee:ff (case-insensitive) -> 6 bytes; the whole rest of the string must be the address
static bool parseMac(const char *z, uint8_t *mac) {
  for (int i = 0; i < 6; i++) {
    char *end = nullptr;
    const unsigned long v = strtoul(z, &end, 16);
    if (end != z + 2 || v > 0xFF) return false;
    mac[i] = (uint8_t)v;
    z = end;
    if (i < 5) {
      if (*z != ':') return false;
      z++;
    }
  }
  return *z == '\0';
}

static void runCommand(char *c) {
  while (*c == ' ' || *c == '\t') c++;
  size_t n = strlen(c);
  while (n > 0 && (c[n - 1] == ' ' || c[n - 1] == '\t')) c[--n] = '\0';

  if ((c[0] == 'L' || c[0] == 'l') && c[1] == ',') {
    char *end = nullptr;
    long ch = strtol(c + 2, &end, 10);
    uint8_t mac[6];
    bool hasMac = false;  // L,<ch>,<aa:bb:cc:dd:ee:ff>: also track that source on the LED
    if (end != c + 2 && *end == ',') {
      hasMac = parseMac(end + 1, mac);
      if (!hasMac) {
        wireInfo("error L mac must be aa:bb:cc:dd:ee:ff");
        return;
      }
    } else if (end == c + 2 || *end != '\0') {
      ch = -1;
    }
    if (ch < cfg::kChanFirst || ch > cfg::kChanLast) {
      infof("error L needs a channel %u-%u", cfg::kChanFirst, cfg::kChanLast);
      return;
    }
    if (!s_radioOk) {
      wireInfo("error radio not running");
      return;
    }
    s_locked = true;
    ledTrack(hasMac ? mac : nullptr);
    if (tuneTo((uint8_t)ch, millis())) infof("lock ch=%u track=%d", s_channel, hasMac ? 1 : 0);
    else infof("error could not tune to ch=%ld", ch);
    return;
  }
  if ((c[0] == 'U' || c[0] == 'u') && c[1] == '\0') {
    s_locked = false;
    ledTrack(nullptr);
    s_lastHopMs = millis();
    infof("unlock hop from ch=%u", s_channel);
    return;
  }
  if ((c[0] == 'V' || c[0] == 'v') && c[1] == '\0') {
    printVersion();
    sourcesResendMeta();  // the page sends V after opening the port: give it every M it missed
    return;
  }
  wireInfo("error unknown command (use L,<ch> or U or V)");
}

static char s_cmd[32];
static uint8_t s_cmdLen = 0;
static bool s_cmdTooLong = false;

static uint32_t s_lastRxMs = 0;  // last byte from the host (USB self-heal)

static void pollCommands() {
  for (int budget = 64; budget > 0 && Serial.available() > 0; budget--) {
    const int c = Serial.read();
    if (c < 0) break;
    s_lastRxMs = millis();
    if (c == '\n' || c == '\r') {
      if (s_cmdTooLong) {
        wireInfo("error command too long");
      } else if (s_cmdLen > 0) {
        s_cmd[s_cmdLen] = '\0';
        runCommand(s_cmd);
      }
      s_cmdLen = 0;
      s_cmdTooLong = false;
      continue;
    }
    if (s_cmdLen < sizeof(s_cmd) - 1) s_cmd[s_cmdLen++] = (char)c;
    else s_cmdTooLong = true;
  }
}

// ---------------------------------------------------------------------------------------------
static uint32_t perSecond(uint32_t delta, uint32_t elapsedMs) {
  if (elapsedMs == 0) return delta;
  return (uint32_t)(((uint64_t)delta * 1000u + elapsedMs / 2) / elapsedMs);
}

// H,<ms>,<hop|lock>,<ch>,<pkts_per_s>,<dropped_per_s>,<heap>   then optional trailing fields:
//   ,<tx_dropped_lines_per_s>,<ie_dropped_per_s>,<bad_rx_per_s>,<sources>,<min_heap>,<noise_floor_dbm>,<busy_channels>
static void heartbeat(uint32_t now) {
  static uint32_t pFrames = 0, pRing = 0, pIe = 0, pBad = 0, pTx = 0, pPDrop = 0, pTable = 0;
  static uint32_t pNoiseSum = 0, pNoiseN = 0;
  const uint32_t elapsed = now - s_lastBeatMs;
  s_lastBeatMs = now;

  const uint32_t frames = g_cap.frames.load(std::memory_order_relaxed);
  const uint32_t ring = g_cap.ringDrop.load(std::memory_order_relaxed);
  const uint32_t ie = g_cap.ieDrop.load(std::memory_order_relaxed);
  const uint32_t bad = g_cap.badState.load(std::memory_order_relaxed);
  const uint32_t noiseSum = (uint32_t)g_cap.noiseSum.load(std::memory_order_relaxed);
  const uint32_t noiseN = g_cap.noiseN.load(std::memory_order_relaxed);
  const uint32_t tx = wireDroppedLines();
  const TableStats t = sourcesStats();

  const uint32_t dropped = (ring - pRing) + (t.pFramesDropped - pPDrop) + (t.tableDrops - pTable);
  const uint32_t dn = noiseN - pNoiseN;
  const int32_t nf = dn ? (int32_t)(noiseSum - pNoiseSum) / (int32_t)dn : 0;

  char buf[160];
  LineBuf o(buf, sizeof(buf));
  o.str("H,");
  o.u32(now);
  o.ch(',');
  o.str(s_locked ? "lock" : "hop");
  o.ch(',');
  o.u32(s_channel);
  o.ch(',');
  o.u32(perSecond(frames - pFrames, elapsed));
  o.ch(',');
  o.u32(perSecond(dropped, elapsed));
  o.ch(',');
  o.u32(esp_get_free_heap_size());
  o.ch(',');
  o.u32(perSecond(tx - pTx, elapsed));
  o.ch(',');
  o.u32(perSecond(ie - pIe, elapsed));
  o.ch(',');
  o.u32(perSecond(bad - pBad, elapsed));
  o.ch(',');
  o.u32(t.active);
  o.ch(',');
  o.u32(esp_get_minimum_free_heap_size());
  o.ch(',');
  o.i32(nf);
  o.ch(',');
  o.u32((uint32_t)__builtin_popcount(s_busy));  // channels in the hop round (+1 quiet slot)
  o.ch('\n');
  wireSend(o);

  pFrames = frames;
  pRing = ring;
  pIe = ie;
  pBad = bad;
  pTx = tx;
  pPDrop = t.pFramesDropped;
  pTable = t.tableDrops;
  pNoiseSum = noiseSum;
  pNoiseN = noiseN;
}

// ---------------------------------------------------------------------------------------------
void setup() {
  usbResetGuard();                        // before Wi-Fi and before any USB output
  esp_log_level_set("*", ESP_LOG_NONE);   // IDF logs share the USB console: keep the line protocol clean
  wireBegin();
  ledBegin();
  Serial.setDebugOutput(false);

  infof("%s %s protocol=%s board=XIAO_ESP32C6 antenna=onboard", SNIFFER_NAME, SNIFFER_VERSION, SNIFFER_PROTOCOL);
  infof("boot reset=%s usb_restart=%d", resetName(s_bootReason), s_restartedAfterUsbReset ? 1 : 0);

  s_radioErr = captureBegin(cfg::kChanFirst, &s_radioStep);
  s_radioOk = (s_radioErr == ESP_OK);
  s_channel = cfg::kChanFirst;
  for (int c = 0; c <= cfg::kChanLast; c++) s_lastStart[c] = kNever;
  if (s_radioOk) {
    infof("radio ok country=GB channels=%u-%u mode=hop dwell_ms=%lu window_ms=%lu", cfg::kChanFirst, cfg::kChanLast,
          (unsigned long)cfg::kDwellMs, (unsigned long)cfg::kWindowMs);
  } else {
    infof("error radio step=%s err=%s", s_radioStep ? s_radioStep : "?", esp_err_to_name(s_radioErr));
  }
  const uint32_t now = millis();
  s_lastHopMs = s_windowStartMs = s_lastBeatMs = s_dwellStartMs = now;
  enableLoopWDT();  // loop() stuck for the task watchdog timeout (5 s) -> restart (fed after every loop())
}

void loop() {
  const uint32_t now = millis();
  pollCommands();

  drainRings(now);  // 0.3.0：先清环再跳频，D 行之前的 P 行就是这次驻留收到的全部
  if (s_radioOk && !s_locked && (uint32_t)(now - s_lastHopMs) >= s_curDwell) {
    s_lastHopMs = now;
    hopNext(now);
  }
  drainRings(now);

  if ((uint32_t)(now - s_windowStartMs) >= cfg::kWindowMs) {
    s_windowStartMs = now;
    sourcesFlush(now);
    // 锁定时没有"换信道"：每 ≥ kDwellMs 在正常的 25 ms 刷出之后补一行 D（不额外刷出、不重新调台），
    // 聚焦源的 P 行窗口和没有 D 行时完全一样
    if (cfg::kEmitDwellLines && s_locked && (uint32_t)(now - s_dwellStartMs) >= cfg::kDwellMs) emitDwell(now);
  }
  if ((uint32_t)(now - s_lastBeatMs) >= cfg::kHeartbeatMs) heartbeat(now);
  // USB self-heal: after a re-plug the USB TX path can stay stuck (0 bytes to the host) while everything else runs;
  // a software restart brings it back. Only when the host is clearly there (it sent a byte in the last 5 s).
  if (s_lastRxMs && now > 5000 && (uint32_t)(now - s_lastRxMs) < 5000 && (uint32_t)(now - wireLastOkMs()) > cfg::kUsbStuckMs) {
    esp_restart();
  }
  static uint32_t s_lastLedMs = 0;
  if ((uint32_t)(now - s_lastLedMs) >= cfg::kLedTickMs) {
    s_lastLedMs = now;
    ledTick(now);
  }

  vTaskDelay(1);  // 1 ms tick: lets IDLE run; the rings absorb ~170 ms of a busy channel
}
