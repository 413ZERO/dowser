// Single-producer / single-consumer ring of fixed-size records. No locks, no allocation.
// Producer: the Wi-Fi driver task (promiscuous callback). Consumer: the Arduino loop task.
#pragma once

#include <atomic>
#include <stdint.h>

template <typename T, uint32_t N>
class SpscRing {
  static_assert(N != 0 && (N & (N - 1)) == 0, "SpscRing size must be a power of two");

 public:
  // Producer: a free slot to fill, or nullptr when the ring is full. Call publish() after filling it.
  T *claim() {
    const uint32_t h = head_.load(std::memory_order_relaxed);
    const uint32_t t = tail_.load(std::memory_order_acquire);
    if (h - t >= N) return nullptr;
    return &buf_[h & (N - 1)];
  }
  void publish() { head_.store(head_.load(std::memory_order_relaxed) + 1, std::memory_order_release); }

  // Consumer: the oldest record, or nullptr when empty. Call pop() when done with it.
  const T *peek() const {
    const uint32_t t = tail_.load(std::memory_order_relaxed);
    const uint32_t h = head_.load(std::memory_order_acquire);
    if (t == h) return nullptr;
    return &buf_[t & (N - 1)];
  }
  void pop() { tail_.store(tail_.load(std::memory_order_relaxed) + 1, std::memory_order_release); }

  uint32_t used() const {
    return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire);
  }
  static constexpr uint32_t capacity() { return N; }

 private:
  T buf_[N];
  std::atomic<uint32_t> head_{0};
  std::atomic<uint32_t> tail_{0};
};
