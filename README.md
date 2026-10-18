# Dowser firmware

Firmware for **Dowser**, a thumb-sized WiFi finder. A Seeed XIAO ESP32-C6 listens to the 2.4 GHz networks and devices around it and streams them over USB to a web page, which draws them as a cluster of balls you can follow around a room. The page runs in desktop Chrome or Edge and lives in the Lab section of [zehan.work](https://zehan.work) (Lab → Dowser → Demo Link).

This repo holds the firmware only. Use it as a reference to build your own receiver for the same page.

## Status

| | |
|---|---|
| Board | Seeed Studio XIAO ESP32-C6, onboard ceramic antenna |
| Firmware | 0.3.2, serial protocol 2 |
| Toolchain | Arduino core `esp32:esp32` 3.3.11 (ESP-IDF 5.5.5), arduino-cli 1.5 |
| Size | 989 KB flash (75 %), 119 KB static RAM (36 %) |
| Libraries | none beyond the core |

## What it does

- **Listens only.** Promiscuous mode on management and data frames. It never joins a network, scans or transmits, and needs no password.
- **Hops where the networks are.** Channels with an AP heard in the last 20 s (at −85 dBm or stronger) are visited in turn, 110 ms each. Every other channel gets one 125 ms visit within 2.5 s, so a newly switched-on network shows up within that time. Channels 1–13 (UK).
- **Sums up each source.** Every 25 ms, one line per transmitter: frame count, mean and peak RSSI, channel, bytes and the network it belongs to.
- **Describes each source.** SSID, channel, AP or client and which AP, randomised-MAC flag, and vendor hints (WPS name and model, vendor elements) that the page uses to pick icons. Client probe SSIDs, WPS UUIDs and serial numbers are never forwarded.
- **Locks on demand.** The page can lock the radio to one channel to follow a single network at about 10 beacons a second.
- **Drives the LED.** The yellow user LED breathes dimly while scanning and blinks when a new network appears. While locked on a source, its brightness follows that source's signal.
- **Recovers by itself.** If the host has sent something in the last 5 s but nothing has gone out over USB for 2 s, the board restarts. A loop watchdog restarts it after 5 s of stall.

## Files

`firmware/wifi_sniffer/` is one Arduino sketch.

| File | Role |
|---|---|
| `wifi_sniffer.ino` | setup and loop, channel hopping, commands, heartbeat, USB self-heal |
| `config.h` | every tunable constant |
| `capture.*` | radio setup, promiscuous callback into lock-free rings |
| `ring.h` | single-producer, single-consumer ring |
| `ieparse.*` | beacon and probe-response elements: SSID, channel, WPS, vendor elements |
| `sources.*` | source table (96, LRU), 25 ms aggregation, AP and client inference, `P` and `M` lines |
| `out.*` | allocation-free line building, non-blocking USB writes |
| `led.*` | user LED |

## Build and flash

```sh
arduino-cli core install esp32:esp32@3.3.11
cd firmware/wifi_sniffer
arduino-cli compile --upload -p /dev/cu.usbmodemXXXX --fqbn esp32:esp32:XIAO_ESP32C6 .
arduino-cli monitor -p /dev/cu.usbmodemXXXX   # optional: watch the lines
```

- Use the **XIAO_ESP32C6** board, not a generic C6. Its variant powers the RF switch and selects the onboard antenna. With the generic board the signal is about 17 dB weaker.
- Close the web page first. Only one program can hold the port.
- If the port disappears, hold BOOT while plugging in, upload, then press RESET.

## Use

1. Open the Dowser page in desktop Chrome or Edge. Without a board it runs on simulated data.
2. Plug in the board, press **CONNECT** and pick it in the port list.
3. Click a ball to lock onto that network, then walk around and watch it grow or shrink. Click empty space or press Esc to go back to scanning.

If the page shows NO DATA · 0 BYTES and does not recover, press RESET on the board.

## Serial protocol

The page works with any board that speaks this. Transport is USB CDC serial at 115200 baud (ignored on native USB), ASCII lines ending in `\n`. The page never toggles DTR or RTS, ignores lines it does not understand and accepts extra trailing fields.

**Board → page**

| Line | Sent | Format |
|---|---|---|
| `P` | per source, each 25 ms window it was heard | `P,<ms>,<mac>,<n>,<rssi_avg>,<rssi_max>,<ch>,<bytes>,<bssid or ->` |
| `H` | 1 Hz | `H,<ms>,<hop or lock>,<ch>,<frames_per_s>,<dropped_per_s>,<free_heap>[,…]` |
| `M` | first sighting, on change, after `V`, every 60 s | `M,<mac>,<json>` |
| `D` | end of each dwell | `D,<ms>,<ch>,<dwell_ms>` |
| `I` | boot, replies, errors | `I,<text>` |

- `ms` is the board clock (`millis()`). The page reads a jump back of more than 2 s as a reboot.
- `mac` is lowercase `aa:bb:cc:dd:ee:ff`. RSSI is dBm in −127…0. `ch` is the channel the radio was on. `bssid` is the AP the frame belongs to, `-` if unknown.
- `M` is JSON after the second comma: `kind` (`"ap"` or `"client"`), `ssid`, `ch` (the AP's own channel), `bssid` (a client's AP), `la` (randomised MAC), `hidden`, `wps` `{cat, sub, name, mfr, model, modelNo}`, `p2p`, `vie` (`["0050f2:04", …]`), `beaconOnly`, and optional `privacy`, `rsn`, `ht`, `vht`, `he`, `extcap`, `interworking`, `espnow`, `p2pdev`. The page merges, so missing keys are fine.
- After `L`, `H` must report `lock` and the locked channel, or the page sends `L` again.

**Page → board**

| Command | Effect | Reply |
|---|---|---|
| `L,<ch>[,<mac>]` | lock to channel 1–13; the LED tracks `<mac>` | `I,lock ch=<ch> track=<0 or 1>` |
| `U` | resume hopping | `I,unlock hop from ch=<ch>` |
| `V` | print version and config as `I` lines, then resend every `M` | |

**Minimum for the page:** `P` lines, a 1 Hz `H` line, and accepting `L`, `U` and `V`. `M` gets routers, names and icons right; without it the page guesses AP or client from `bssid`. `D` lets the page tell within about 5 s that a network was switched off; without it the page infers each visit from the channels in `P` lines, a little less reliably.

## Porting to another board (notes for agents)

**Hardware requirements**

- A 2.4 GHz WiFi radio with a promiscuous or monitor mode that delivers management and data frames with **RSSI and channel per frame**, and retunes in a few milliseconds.
- A serial link to the computer. The hosted page only lists **Espressif native-USB ports (USB vendor ID 0x303A)**. ESP32-S2, S3, C3 and C6 boards on the chip's own USB work as they are. Boards behind a CP210x, CH340 or FTDI bridge need their own copy of the page with that filter removed (`filters: []` in `serial-source.js`), and may reset when the port opens because of their DTR/RTS auto-reset circuit. Keep 115200 baud.
- About 75 KB of RAM for the firmware's buffers plus an 8 KB USB transmit buffer.
- Optional: one LED.

Any ESP32 with WiFi fits through the ESP-IDF `esp_wifi_set_promiscuous` API. Radios without a usable monitor mode do not.

**Steps**

1. Keep `sources.*`, `ieparse.*`, `out.*` and `ring.h`. They are plain C++ over the `FrameRec` and `IeRec` records in `capture.h`.
2. Rewrite `capture.*` for your radio. Keep the WiFi config in RAM only, turn on promiscuous mode for management and data frames, and in the callback copy addr1/2/3, frame control, RSSI (clamped to −127…0), channel and length into the frame ring, plus the body of beacons, probe responses, probe requests and (re)association requests into the element ring. Do not allocate or lock in the callback. Expose `captureBegin()` and `captureSetChannel()`.
3. In `wifi_sniffer.ino` and `config.h`, replace the board-specific parts: the board name in `printVersion()`, the USB-reset guard (a workaround for the ESP32-C6 USB-Serial/JTAG), `enableLoopWDT()`, and the LED pin and polarity. Keep the hop scheduler and the 25 ms flush.
4. Set the country and channel range for your region in `capture.cpp` and `config.h` (`kChanFirst`, `kChanLast`).
5. Check with a serial monitor: one `H` per second with frames/s above 0 and the channel moving; `V` prints the version lines and a burst of `M`; `L,<your router's channel>` turns `H` into `lock` and that router's `P` lines arrive continuously.

## Limits

- 2.4 GHz only, channels 1–13. 5 GHz networks are invisible.
- One radio, so hopping is time-shared and each busy channel is heard about once a second. Lock onto one channel for continuous data.
- A strong network leaks into neighbouring channels at lower RSSI. Its true channel is in `M`.
- RSSI tells you closer or further, not a distance.
- SSIDs and device names are forwarded as broadcast. Blur them when you record your screen.

## License

[PolyForm Noncommercial 1.0.0](LICENSE.md). Free for personal projects, study, research and other non-commercial use. Commercial use needs permission, ask via [zehan.work](https://zehan.work).
