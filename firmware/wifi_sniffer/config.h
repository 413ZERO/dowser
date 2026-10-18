// wifi_sniffer: compile-time settings shared by every file of the sketch.
#pragma once

#include <stddef.h>
#include <stdint.h>

#define SNIFFER_NAME "wifi_sniffer"
#define SNIFFER_VERSION "0.3.2"
#define SNIFFER_PROTOCOL "2"  // H / P / M / I 行 + D 行（0.3.0 新增：一次驻留结束），见 README.md

namespace cfg {

// Onboard user LED (XIAO ESP32C6: GPIO15, lit when LOW; one colour, so the modes differ by behaviour).
// Tracking (L,<ch>,<mac>): brightness follows that source's RSSI, kLedRssiMin (dim) .. kLedRssiMax (full).
constexpr uint8_t kLedPin = 15;
constexpr bool kLedActiveLow = true;
constexpr int8_t kLedRssiMin = -85;
constexpr int8_t kLedRssiMax = -35;
constexpr uint32_t kUsbStuckMs = 2000;      // host sent a byte in the last 5 s, nothing went out for this long: restart
constexpr uint32_t kLedTickMs = 10;        // same step as the original LED sketch
constexpr uint32_t kLedLostMs = 1500;      // tracking: target silent this long = quick flash
constexpr uint32_t kLedQuietBootMs = 5000; // no new-network blinks during the boot sweep

// Radio. Country GB, 2.4 GHz channels 1-13, manual policy (never learnt from an AP).
constexpr uint8_t kChanFirst = 1;
constexpr uint8_t kChanLast = 13;
constexpr uint32_t kDwellMs = 110;  // 跳频时每个"忙"信道停多久（beacon 每 102.4 ms 一个）；锁定时 D 行的最短间隔
// 0.3.0 跳频（压力测试里叫 v3）：忙信道轮流去；每个"空"信道在 kQuietPeriodMs 内完整停留一次 kQuietDwellMs，
// 按截止时间（最早的先）匀速插进去，所以任何信道上新开的网络一个周期内一定能听到。
// 忙 = 最近 kBusyForMs 内在这个信道上听到过 ≥ kBusyMinRssi 的 AP（更弱的网络页面上本来就藏起来）。一个忙信道都没有时：1→13 顺序扫，每格 kQuietDwellMs。
constexpr uint32_t kBusyForMs = 20000;
constexpr int8_t kBusyMinRssi = -85;
constexpr uint32_t kQuietDwellMs = 125;   // 空信道驻留：≥ beacon 周期 102.4 + 切信道空窗（≤ 12）+ 1 Mbps beacon 2.6 ms + 抖动，去一次就能抓到 beacon
constexpr uint32_t kQuietPeriodMs = 2500; // 每个空信道在这么长时间内完整停留一次
constexpr bool kEmitDwellLines = true;    // 每次驻留结束发一行 D,<ms>,<信道>,<驻留 ms>（页面用来判"离开"和给样本结案）

// Output pacing.
constexpr uint32_t kWindowMs = 25;            // P lines aggregate each source over this window
constexpr uint32_t kHeartbeatMs = 1000;       // H line
constexpr uint32_t kMetaMinIntervalMs = 1000; // after the first M of a source, re-send at most this often
constexpr uint8_t kMetaPerFlush = 8;          // M lines per window at most (boot sweep spreads out)
constexpr uint32_t kFirstMetaHoldMs = 500;    // a new source's P data waits (aggregating) for its first M, at most this long
constexpr uint32_t kMetaRefreshMs = 60000;    // a source heard in the last minute repeats its M this often (lost-line repair)

// USB serial (native USB-Serial/JTAG, "USB CDC On Boot" enabled on the XIAO_ESP32C6 board).
constexpr size_t kSerialTxBuf = 8192;  // one flush (8 M + a P per busy source) must fit: the ISR drains only between host polls
constexpr size_t kSerialRxBuf = 256;

// Capture path: Wi-Fi task -> lock-free rings -> loop task.
constexpr uint32_t kFrameRingSlots = 512;     // power of two; one small record per accepted frame
constexpr uint32_t kIeRingSlots = 32;         // power of two; bounded mgmt-body copies
constexpr uint16_t kIeMaxBytes = 768;         // mgmt body bytes copied per IE record (after the 24-byte header)
constexpr uint32_t kIeThrottleUs = 1000000;   // copy the body of the same (MAC, subtype) at most once a second
constexpr uint32_t kIeThrottleSlots = 128;    // power of two

// Source table.
constexpr size_t kMaxSources = 96;  // LRU eviction when full
constexpr size_t kMaxVendorIes = 12;
constexpr size_t kStrMax = 32;      // every forwarded string is cut to 32 bytes

}  // namespace cfg
