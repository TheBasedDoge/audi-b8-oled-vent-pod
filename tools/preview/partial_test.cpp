#include <stdio.h>
#include <string.h>
#ifndef MAXR
#define MAXR 16
#endif
#include "../../firmware/S5GaugePod_Test/gauge_render.h"
// Checks the firmware fast path (restore dirty spans, send changed rows) against a full
// redraw every frame, in all four rotations, with page changes and a pixel shift.
// build: g++ -O2 -std=c++17 tools/preview/partial_test.cpp -o partial_test
using namespace gr;
static uint16_t faceC[2][P_COUNT][W*H], fbA[W*H], fbB[W*H], fbUp[W*H], fbRot[W*H];
int main() {
  Cfg k; k.shiftRpm = 7200; k.redlineRpm = 7400;
  long bad = 0, sentRows = 0, frames = 0;
  for (int rot = 0; rot < 4; rot++) {
    setRotation(rot); setShift(0, 0);
    for (int t = 0; t < 2; t++) { setTheme(t); for (int p = 0; p < P_COUNT; p++) { target(faceC[t][p]); renderStatic(p, k); } }
    // upright reference faces for rotation check
    static uint16_t up[P_COUNT][W*H];
    setRotation(0); setTheme(0); for (int p = 0; p < P_COUNT; p++) { target(up[p]); renderStatic(p, k); }
    setRotation(rot);
    // panel buffer B persists (what the panel shows); A = full redraw reference
    target(fbB); setTheme(0); setShift(0,0); beginFrame(faceC[0][0], true); tracking = false;
    int page = 0, sh = 0;
    for (int f = 0; f < 200; f++) {
      if (f == 80) page = 1; if (f == 120) page = 3; if (f == 160) page = 4;
      bool full = (f == 0 || f == 80 || f == 120 || f == 160 || f == 100);
      if (f == 100) sh = 1;                      // pixel shift step
      int dx = sh ? 1 : 0, dy = sh ? 1 : 0;
      Data d; demo(f * 0.09f, d); d.boostText = d.boost; d.peak = 12.3f; d.maxIat = 134; d.maxRpm = 7300;
      // incremental
      target(fbB); setTheme(0); setShift(dx, dy);
      beginFrame(faceC[0][page], full);
      renderDynamic(page, k, d, f * 33.0f);
      int16_t runs[16][2]; int n = endFrame(runs, MAXR);
      for (int i = 0; i < n; i++) sentRows += runs[i][1] - runs[i][0] + 1;
      // the panel only receives the runs: copy those rows of B into the "panel" image
      static uint16_t panelImg[W*H];
      if (full) memcpy(panelImg, fbB, sizeof(fbB));
      else for (int i = 0; i < n; i++) memcpy(panelImg + runs[i][0]*W, fbB + runs[i][0]*W, (runs[i][1]-runs[i][0]+1)*W*2);
      // full redraw reference (save the incremental path's dirty-row state around it)
      static int16_t sMin[H], sMax[H]; memcpy(sMin, dMin, sizeof(dMin)); memcpy(sMax, dMax, sizeof(dMax));
      int sY0 = prevY0, sY1 = prevY1;
      target(fbA); setShift(dx, dy); beginFrame(faceC[0][page], true); renderDynamic(page, k, d, f * 33.0f); endFrame(runs, 16);
      { long fbad = 0; int ymin = H, ymax = -1; for (int i = 0; i < W*H; i++) if (panelImg[i] != fbA[i]) { fbad++; int yy = i / W; if (yy < ymin) ymin = yy; if (yy > ymax) ymax = yy; }
        long bbad = 0; for (int i = 0; i < W*H; i++) if (fbB[i] != fbA[i]) bbad++;
        if (fbad && rot == 0) printf("rot %d f %d page %d: panel %ld bad rows %d-%d, buffer %ld bad, runs %d\n", rot, f, page, fbad, ymin, ymax, bbad, n);
        bad += fbad; }
      frames++;
      // rotation check: upright full render, rotated by hand, must equal the rotated render (no shift)
      if (f % 50 == 7) {
        setRotation(0); setShift(0,0); target(fbUp); beginFrame(up[page], true); renderDynamic(page, k, d, f * 33.0f); endFrame(runs, 16);
        setRotation(rot); target(fbRot); beginFrame(faceC[0][page], true); renderDynamic(page, k, d, f * 33.0f); endFrame(runs, 16);
        long rb = 0;
        for (int y = 0; y < H; y++) for (int x = 0; x < W; x++) { int px, py; physical(x, y, px, py); if (fbRot[py*W+px] != fbUp[y*W+x]) rb++; }
        if (rb) printf("rot %d frame %d: %ld pixels differ from hand-rotated upright frame\n", rot, f, rb);
        bad += rb;
      }
      memcpy(dMin, sMin, sizeof(dMin)); memcpy(dMax, sMax, sizeof(dMax)); prevY0 = sY0; prevY1 = sY1;
    }
  }
  printf("frames %ld, mismatched pixels %ld, avg rows sent %.0f%% of screen\n", frames, bad, 100.0 * sentRows / frames / H);
  return bad != 0;
}
