#include "out.h"

#include <string.h>

#include <Arduino.h>

#include "config.h"

namespace {
const char kHexLower[] = "0123456789abcdef";
const char kHexUpper[] = "0123456789ABCDEF";
uint32_t s_droppedLines = 0;
uint32_t s_lastOkMs = 0;
}  // namespace

void LineBuf::ch(char c) {
  if (n_ + 1 >= cap_) {  // keep one byte spare for a terminating NUL
    overflow_ = true;
    return;
  }
  b_[n_++] = c;
  b_[n_] = '\0';
}

void LineBuf::str(const char *z) {
  while (*z) ch(*z++);
}

void LineBuf::u32(uint32_t v) {
  char tmp[10];
  int i = 0;
  do {
    tmp[i++] = (char)('0' + v % 10);
    v /= 10;
  } while (v != 0);
  while (i > 0) ch(tmp[--i]);
}

void LineBuf::i32(int32_t v) {
  if (v < 0) {
    ch('-');
    u32((uint32_t)(-(int64_t)v));
  } else {
    u32((uint32_t)v);
  }
}

void LineBuf::hex2(uint8_t v, bool upper) {
  const char *h = upper ? kHexUpper : kHexLower;
  ch(h[v >> 4]);
  ch(h[v & 0x0F]);
}

void LineBuf::mac(const uint8_t *m) {
  for (int i = 0; i < 6; i++) {
    if (i) ch(':');
    hex2(m[i]);
  }
}

void LineBuf::ouiUpper(const uint8_t *m) {
  for (int i = 0; i < 3; i++) {
    if (i) ch(':');
    hex2(m[i], true);
  }
}

void LineBuf::jstr(const uint8_t *p, size_t len) {
  ch('"');
  size_t i = 0;
  while (i < len) {
    const uint8_t b0 = p[i];
    if (b0 < 0x80) {
      if (b0 == '"' || b0 == '\\') {
        ch('\\');
        ch((char)b0);
      } else if (b0 < 0x20 || b0 == 0x7F) {
        str("\\u00");
        hex2(b0);
      } else {
        ch((char)b0);
      }
      i++;
      continue;
    }
    // Multi-byte UTF-8: lead byte decides the length and the allowed range of the 2nd byte.
    size_t need = 0;
    uint8_t lo = 0x80, hi = 0xBF;
    if (b0 >= 0xC2 && b0 <= 0xDF) {
      need = 1;
    } else if (b0 >= 0xE0 && b0 <= 0xEF) {
      need = 2;
      if (b0 == 0xE0) lo = 0xA0;  // no overlongs
      if (b0 == 0xED) hi = 0x9F;  // no surrogates
    } else if (b0 >= 0xF0 && b0 <= 0xF4) {
      need = 3;
      if (b0 == 0xF0) lo = 0x90;
      if (b0 == 0xF4) hi = 0x8F;
    }
    if (need == 0) {  // stray byte
      str("\\u00");
      hex2(b0);
      i++;
      continue;
    }
    if (i + need >= len) {
      // Not enough bytes left. If what is there looks like the start of a valid sequence the string
      // was cut mid-character: drop the tail. Otherwise escape the lead byte and go on.
      bool prefixOk = true;
      for (size_t k = 1; i + k < len; k++) {
        const uint8_t c = p[i + k];
        const uint8_t l = (k == 1) ? lo : 0x80, h = (k == 1) ? hi : 0xBF;
        if (c < l || c > h) prefixOk = false;
      }
      if (prefixOk) break;
      str("\\u00");
      hex2(b0);
      i++;
      continue;
    }
    bool valid = true;
    for (size_t k = 1; k <= need; k++) {
      const uint8_t c = p[i + k];
      const uint8_t l = (k == 1) ? lo : 0x80, h = (k == 1) ? hi : 0xBF;
      if (c < l || c > h) {
        valid = false;
        break;
      }
    }
    if (!valid) {
      str("\\u00");
      hex2(b0);
      i++;
      continue;
    }
    for (size_t k = 0; k <= need; k++) ch((char)p[i + k]);
    i += need + 1;
  }
  ch('"');
}

uint32_t fnv1a(const char *p, size_t n) {
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < n; i++) h = (h ^ (uint8_t)p[i]) * 16777619u;
  return h;
}

void wireBegin() {
  // HWCDC: buffer sizes must be set before begin(). Timeout 0 = never wait for the host.
  Serial.setTxBufferSize(cfg::kSerialTxBuf);
  Serial.setRxBufferSize(cfg::kSerialRxBuf);
  Serial.setTxTimeoutMs(0);
  Serial.begin(115200);  // baud is ignored on native USB
}

bool wireSend(const char *line, size_t n) {
  if (n == 0) return true;
  const int room = Serial.availableForWrite();
  if (room <= 0 || (size_t)room < n) {
    s_droppedLines++;
    return false;
  }
  const size_t w = Serial.write(reinterpret_cast<const uint8_t *>(line), n);
  if (w != n) {
    s_droppedLines++;
    return false;
  }
  s_lastOkMs = millis();
  return true;
}

uint32_t wireLastOkMs() { return s_lastOkMs; }

bool wireSend(const LineBuf &line) {
  if (!line.ok()) {  // never send a truncated line
    s_droppedLines++;
    return false;
  }
  return wireSend(line.data(), line.size());
}

uint32_t wireDroppedLines() { return s_droppedLines; }

bool wireInfo(const char *text) {
  char buf[256];
  LineBuf o(buf, sizeof(buf));
  o.str("I,");
  for (const char *c = text; *c && o.size() + 3 < sizeof(buf); c++) o.ch((*c == '\n' || *c == '\r') ? ' ' : *c);
  o.ch('\n');
  return wireSend(o);
}
