// Desktop stand-in: a byte FIFO with the FreeRTOS stream buffer calls logger.h uses.
#pragma once
#include <deque>
#include <cstdint>
#include <cstddef>
struct FakeSB { std::deque<uint8_t> q; size_t cap; };
typedef FakeSB *StreamBufferHandle_t;
typedef int StaticStreamBuffer_t;
static inline StreamBufferHandle_t xStreamBufferCreateStatic(size_t cap, size_t, uint8_t *, StaticStreamBuffer_t *) { auto *b = new FakeSB; b->cap = cap; return b; }
static inline size_t xStreamBufferSend(StreamBufferHandle_t b, const void *d, size_t n, int) {
  if (b->q.size() + n > b->cap) return 0;                   // whole line or nothing (lines are short)
  const uint8_t *p = (const uint8_t *)d; b->q.insert(b->q.end(), p, p + n); return n; }
static inline size_t xStreamBufferReceive(StreamBufferHandle_t b, void *d, size_t n, int) {
  size_t k = n < b->q.size() ? n : b->q.size(); uint8_t *p = (uint8_t *)d;
  for (size_t i = 0; i < k; i++) { p[i] = b->q.front(); b->q.pop_front(); } return k; }
static inline size_t xStreamBufferBytesAvailable(StreamBufferHandle_t b) { return b->q.size(); }
#define pdMS_TO_TICKS(x) (x)
static inline void vTaskDelay(int) {}
static inline int xTaskCreatePinnedToCore(void (*)(void *), const char *, int, void *, int, void *, int) { return 1; }
