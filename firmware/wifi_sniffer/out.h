// Line building (no printf, no allocation) and the non-blocking serial line sender.
#pragma once

#include <stddef.h>
#include <stdint.h>

// Appends into a fixed buffer. On overflow it stops writing and remembers it (check ok()).
class LineBuf {
 public:
  LineBuf(char *buf, size_t cap) : b_(buf), cap_(cap) {}

  void reset() { n_ = 0; overflow_ = false; }
  void ch(char c);
  void str(const char *z);
  void u32(uint32_t v);
  void i32(int32_t v);
  void hex2(uint8_t v, bool upper = false);
  void mac(const uint8_t *m);             // aa:bb:cc:dd:ee:ff
  void ouiUpper(const uint8_t *m);        // AA:BB:CC
  void boolean(bool v) { str(v ? "true" : "false"); }
  // JSON string literal with quotes. Valid UTF-8 passes through; control characters, quotes and
  // backslashes are escaped; any byte that is not part of valid UTF-8 becomes \u00XX (read as
  // Latin-1); an incomplete sequence at the very end (a string cut at 32 bytes) is dropped.
  void jstr(const uint8_t *p, size_t len);

  const char *data() const { return b_; }
  size_t size() const { return n_; }
  bool ok() const { return !overflow_; }

 private:
  char *b_;
  size_t cap_;
  size_t n_ = 0;
  bool overflow_ = false;
};

// Helper for writing JSON object members: emits the separating comma.
class JsonObj {
 public:
  explicit JsonObj(LineBuf &o) : o_(o) { o_.ch('{'); }
  LineBuf &key(const char *k) {
    if (!first_) o_.ch(',');
    first_ = false;
    o_.ch('"');
    o_.str(k);
    o_.str("\":");
    return o_;
  }
  void close() { o_.ch('}'); }

 private:
  LineBuf &o_;
  bool first_ = true;
};

uint32_t fnv1a(const char *p, size_t n);

// Serial line sender. Writes the whole line or nothing: when the host is not draining the USB
// buffer the line is dropped and counted, so a page that stops reading never stalls the sniffer.
void wireBegin();
uint32_t wireLastOkMs();  // millis() of the last line that went out whole (USB self-heal)
bool wireSend(const char *line, size_t n);
bool wireSend(const LineBuf &line);
uint32_t wireDroppedLines();          // total since boot
bool wireInfo(const char *text);      // "I,<text>\n"
