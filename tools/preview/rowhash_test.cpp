#include <stdio.h>
#include <string.h>
#include <math.h>
// Checks the firmware's changed-row push: the simulated panel only receives the rows the
// firmware would send, and must always match the frame buffer.
// build: g++ -O2 -std=c++17 tools/preview/rowhash_test.cpp -o rowhash_test
#include "../../firmware/S5GaugePod_Test/gauge_render.h"
using namespace gr;
static uint16_t faceP[P_COUNT][W*H], fbuf[W*H], panelImg[W*H];
static uint32_t rowHash[H];
static const int LCD_W = W, LCD_H = H;
int main() {
  Cfg k; setRotation(3);
  for (int p = 0; p < P_COUNT; p++) { target(faceP[p]); renderStatic(p, k); }
  long bad = 0, sent = 0, frames = 0;
  int page = 0; bool needFull = true;
  for (int f = 0; f < 600; f++) {
    if (f == 200) { page = 1; needFull = true; } if (f == 400) { page = 3; needFull = true; }
    Data d; demo(f < 300 ? 1.0f + (f % 50) * 0.001f : f * 0.05f, d);   // mostly still, then moving
    d.boostText = d.boost; d.peak = 3.5f; d.maxRpm = 7300; d.maxIat = 134;
    target(fbuf); setShift(0, 0);
    beginFrame(faceP[page], needFull); renderDynamic(page, k, d, f * 33.0f);
    int16_t runs[16][2]; int nr = endFrame(runs, 16);
    auto rowFp = [](int y) { const uint32_t *w = (const uint32_t *)(fbuf + y * LCD_W); uint32_t h = 2166136261u;
                             for (int i = 0; i < LCD_W / 2; i++) { h ^= w[i]; h *= 16777619u; } return h; };
    auto push = [&](int a, int n) { memcpy(panelImg + a * W, fbuf + a * W, n * W * 2); sent += n; };
    if (needFull) { push(0, H); for (int y = 0; y < H; y++) rowHash[y] = rowFp(y); }
    else {
      int y0 = -1, gap = 0, last = -1;
      auto send = [&](int a, int b) { int a0 = a, b0 = b; a &= ~1; if (!(b & 1)) b++; if (b > LCD_H - 1) b = LCD_H - 1;
        if (a < a0) rowHash[a] = rowFp(a); if (b > b0) rowHash[b] = rowFp(b); push(a, b - a + 1); };
      for (int i = 0; i < nr; i++) for (int y = runs[i][0]; y <= runs[i][1]; y++) {
        uint32_t h = rowFp(y);
        if (h == rowHash[y]) { if (y0 >= 0 && ++gap > 6) { send(y0, last); y0 = -1; } continue; }
        rowHash[y] = h; gap = 0; last = y; if (y0 < 0) y0 = y;
      }
      if (y0 >= 0) send(y0, last);
    }
    needFull = false;
    for (int i = 0; i < W*H; i++) bad += panelImg[i] != fbuf[i];
    frames++;
  }
  printf("frames %ld, panel mismatches %ld, avg rows sent %.1f%%\n", frames, bad, 100.0 * sent / frames / H);
  return bad != 0;
}
