// =====================================================================
//  S5 Gauge Pod - double-buffered frames: draw the next frame while the last one is sent
//
//  Two frame buffers in PSRAM. The UI loop draws frame N+1 into one while a task on the
//  other core sends frame N from the other, so a frame costs max(draw, send) instead of
//  draw + send.
//
//  Each buffer keeps its own dirty spans (what its last frame drew), because it is two
//  frames behind: restoring "what this buffer drew last time" and then drawing the new
//  frame leaves it holding exactly face + this frame, whatever the other buffer did.
//
//  Which rows to send: the panel always shows exactly the other buffer's frame (every send
//  brings the panel up to date with the buffer it came from). Both buffers are face + their
//  own frame's drawing, so they can only differ where this frame or the previous one drew.
//  Those spans are compared directly (read-only, so it's safe while the other buffer is
//  still being sent): each row gives the exact first..last pixel that changed. Rows are
//  then merged into a few rectangles (a rectangle costs a fixed setup, so neighbouring rows
//  join one while that's cheaper than starting another) and only those rectangles are sent,
//  not whole 466-pixel rows. The panel wants even start / even size in both directions.
//
//  Plain C++ on top of gauge_render.h, so tools/preview/pipe_test.cpp can check it on a
//  desktop against a simulated panel.
// =====================================================================
#pragma once
#include <string.h>

namespace fp {

static const int W = gr::W, H = gr::H;
static const int MAX_RECTS = 64;
static const int RECT_COST_PX = 200;               // fixed cost of one rectangle, in pixels' worth of sending
static bool wholeRows = false;                     // true = always send full-width rows (fallback)

struct Rect { int16_t x0, x1, y0, y1; };
struct Job {
  uint16_t *buf = nullptr;
  bool      full = false;          // send everything (page / theme / pixel-shift change)
  int       nr = 0;
  Rect      r[MAX_RECTS];          // areas to send, even start and even size
};

static int16_t spanMin[2][gr::H], spanMax[2][gr::H];   // what each buffer's last frame drew
static bool    restoreAll[2] = {true, true};           // buffer's spans are stale: restore the whole face
static int     lastK = -1;                             // buffer of the previous frame

// Draw one frame into buffer k (0/1); other = the other buffer (the panel shows its frame).
// The caller makes sure buffer k isn't being sent. render() draws the dynamic layer.
template <class R>
static void draw(int k, uint16_t *buf, const uint16_t *other, const uint16_t *face, bool full, R render, Job &job) {
  if (full) restoreAll[0] = restoreAll[1] = true;
  bool all = restoreAll[k];
  restoreAll[k] = false;
  memcpy(gr::dMin, spanMin[k], sizeof(gr::dMin));
  memcpy(gr::dMax, spanMax[k], sizeof(gr::dMax));
  gr::target(buf);
  gr::beginFrame(face, all);
  render();
  int16_t r16[16][2];
  gr::endFrame(r16, 16);                               // stops tracking; spans = this frame's drawing
  memcpy(spanMin[k], gr::dMin, sizeof(gr::dMin));
  memcpy(spanMax[k], gr::dMax, sizeof(gr::dMax));

  job.buf = buf;
  job.full = full || lastK < 0 || lastK == k;
  job.nr = 0;
  lastK = k;
  if (job.full) return;
  const int o = k ^ 1;
  Rect cur = {0, -1, 0, -1};
  bool have = false;
  auto emit = [&](Rect q) {
    if (wholeRows) { q.x0 = 0; q.x1 = W - 1; }
    q.x0 &= ~1; if (!(q.x1 & 1)) q.x1++; if (q.x1 > W - 1) q.x1 = W - 1;
    q.y0 &= ~1; if (!(q.y1 & 1)) q.y1++; if (q.y1 > H - 1) q.y1 = H - 1;
    if (job.nr == MAX_RECTS) {                       // out of slots: grow the last one
      Rect &l = job.r[job.nr - 1];
      if (q.x0 < l.x0) l.x0 = q.x0; if (q.x1 > l.x1) l.x1 = q.x1; if (q.y1 > l.y1) l.y1 = q.y1;
      return;
    }
    job.r[job.nr++] = q;
  };
  for (int y = 0; y < H; y++) {
    int a = spanMin[k][y] < spanMin[o][y] ? spanMin[k][y] : spanMin[o][y];
    int b = spanMax[k][y] > spanMax[o][y] ? spanMax[k][y] : spanMax[o][y];
    if (a > b) continue;
    const uint16_t *p = buf + y * W, *q = other + y * W;
    while (a <= b && p[a] == q[a]) a++;              // exact first / last changed pixel
    while (b >= a && p[b] == q[b]) b--;
    if (a > b) continue;
    if (!have) { cur = {(int16_t)a, (int16_t)b, (int16_t)y, (int16_t)y}; have = true; continue; }
    int nx0 = a < cur.x0 ? a : cur.x0, nx1 = b > cur.x1 ? b : cur.x1;
    long merged   = (long)(nx1 - nx0 + 1) * (y - cur.y0 + 1);
    long separate = (long)(cur.x1 - cur.x0 + 1) * (cur.y1 - cur.y0 + 1) + (b - a + 1) + RECT_COST_PX;
    if (y - cur.y1 <= 8 && merged <= separate) { cur.x0 = (int16_t)nx0; cur.x1 = (int16_t)nx1; cur.y1 = (int16_t)y; }
    else { emit(cur); cur = {(int16_t)a, (int16_t)b, (int16_t)y, (int16_t)y}; }
  }
  if (have) emit(cur);
}

// Send a drawn frame. send(buf, x, y, w, h) writes that rectangle of buf (stride W) to the
// panel; x, y, w, h are even. Returns the number of pixels sent.
template <class S>
static long push(const Job &job, S send) {
  if (job.full) { send(job.buf, 0, 0, W, H); return (long)W * H; }
  long px = 0;
  for (int i = 0; i < job.nr; i++) {
    const Rect &q = job.r[i];
    int w = q.x1 - q.x0 + 1, h = q.y1 - q.y0 + 1;
    send(job.buf, q.x0, q.y0, w, h);
    px += (long)w * h;
  }
  return px;
}

}  // namespace fp
