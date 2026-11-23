// Checks firmware/S5GaugePod_Test/frame_pipe.h: two frame buffers, frame N+1 drawn while
// frame N is sent. A simulated panel only receives the rows push() sends, in the same order
// as on the ESP32 (draw N+1, then finish sending N). After every send the panel must equal
// a from-scratch render of that frame. Covers moving needles, still screens, page changes,
// theme changes and the burn-in pixel shift.
// build: g++ -O2 -std=c++17 tools/preview/pipe_test.cpp -o pipe_test
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "../../firmware/S5GaugePod_Test/gauge_render.h"
#include "../../firmware/S5GaugePod_Test/frame_pipe.h"
using namespace gr;

static uint16_t faceP[2][P_COUNT][W * H], bufA[W * H], bufB[W * H], panelImg[W * H];
static uint16_t ref[16][W * H];                      // reference renders, by frame number mod 16

int main() {
  Cfg k; setRotation(3);
  for (int t = 0; t < 2; t++) { setTheme(t == 1); for (int p = 0; p < P_COUNT; p++) { target(faceP[t][p]); renderStatic(p, k); } }
  static const int8_t OFF[9][2] = {{0,0},{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1}};
  uint16_t *bufs[2] = {bufA, bufB};
  long bad = 0, sent = 0, frames = 0, checks = 0, rects = 0, oddRects = 0;
  int page = 0, theme = 0, shift = 0, cur = 0;
  bool needFull = true;
  fp::Job jobs[2]; bool pending = false; int pendingIdx = 0, pendingFrame = 0;

  auto finishPush = [&]() {                          // the push task finishing frame N
    if (!pending) return;
    sent += fp::push(jobs[pendingIdx], [&](const uint16_t *b, int x, int y, int w, int h) {
      if ((x | y | w | h) & 1) { if (!oddRects++) printf("odd rectangle %d,%d %dx%d\n", x, y, w, h); }
      for (int r = 0; r < h; r++) memcpy(panelImg + (y + r) * W + x, b + (y + r) * W + x, (size_t)w * 2);
      rects++;
    });
    long diff = 0;
    for (int i = 0; i < W * H; i++) diff += panelImg[i] != ref[pendingFrame & 15][i];
    if (diff) { if (bad == 0) printf("first mismatch at frame %d: %ld px\n", pendingFrame, diff); bad += diff; }
    checks++;
    pending = false;
  };

  for (int f = 0; f < 1500; f++) {
    if (f == 200) { page = 1; needFull = true; }
    if (f == 330) { page = 2; needFull = true; }
    if (f == 331) { page = 3; needFull = true; }      // two changes back to back
    if (f == 500) { theme = 1; needFull = true; }
    if (f == 650) { shift = 4; needFull = true; }
    if (f == 800) { page = 4; needFull = true; }
    if (f == 950) { page = 0; theme = 0; needFull = true; }
    if (f == 1100) { shift = 7; needFull = true; }
    if (f == 1300) { page = 3; needFull = true; }
    if (f == 1350) { page = P_FUEL; needFull = true; }
    if (f == 1400) { page = P_HEAT; needFull = true; }
    if (f == 1450) { page = P_PULL; needFull = true; }
    Data d;
    bool still = (f / 60) % 3 == 1;                  // some stretches with nothing moving
    demo(still ? 2.0f : f * 0.037f, d);
    d.boostText = d.boost; d.peak = 6.5f; d.maxRpm = 7300; d.maxIat = 134; d.status = "";
    d.dotsA = (f % 200) < 40 ? 1.0f : 0;
    d.alertText = (f % 300) > 250 ? "CAT 1 1790\xc2\xb0" "F" : ""; d.alertRed = (f % 600) > 300;   // the banner coming and going
    d.alertTitle = "CATALYST"; d.alertValue = "1790"; d.alertUnit = "\xc2\xb0" "F"; d.alertNote = "BANK 1";
    d.alertBig = (f % 300) > 250 && (f % 300) < 275 ? 1.0f - ((f % 300) - 250) / 25.0f : 0; d.alertPulse = (f % 27) / 27.0f;   // pops up, then the banner
    d.coldOilF = f < 200 ? 140 + f * 0.1f : NOV;
    float now = f * 16.0f;

    // reference: full redraw of this frame into its own buffer
    setTheme(theme == 1); setShift(OFF[shift][0], OFF[shift][1]);
    target(ref[f & 15]); beginFrame(faceP[theme][page], true); renderDynamic(page, k, d, now);
    { int16_t r[16][2]; endFrame(r, 16); }

    // pipeline: draw frame f into bufs[cur] (frame f-1 is still being sent from the other)
    fp::Job &j = jobs[cur];
    fp::draw(cur, bufs[cur], bufs[cur ^ 1], faceP[theme][page], needFull, [&] { renderDynamic(page, k, d, now); }, j);
    needFull = false;
    finishPush();                                   // frame f-1 done
    pending = true; pendingIdx = cur; pendingFrame = f;
    cur ^= 1;
    frames++;
  }
  finishPush();
  printf("frames %ld, checks %ld, panel mismatches %ld px, odd rectangles %ld, avg sent %.1f%% of the screen in %.1f rectangles\n",
         frames, checks, bad, oddRects, 100.0 * sent / frames / (W * H), (double)rects / frames);
  return bad != 0 || oddRects != 0;
}
