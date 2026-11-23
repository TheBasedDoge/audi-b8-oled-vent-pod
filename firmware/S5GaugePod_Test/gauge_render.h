// =====================================================================
//  S5 Gauge Pod - screen renderer (shared)
//
//  This one file draws every page. The firmware calls it on the ESP32, and
//  tools/build_preview.sh compiles the SAME file to WebAssembly for
//  docs/screen-mockup.html, so the browser preview shows the pixels the
//  AMOLED will show (the panel's own colour and brightness aside).
//
//  Plain C++ with no Arduino or libc calls: it writes RGB565 into a 466x466
//  buffer and brings its own maths, anti-aliased shapes and fonts.
// =====================================================================
#pragma once
#include <stdint.h>
// Fonts and the boot badge. The repository ships open ones (Barlow, SIL Open Font License, and an
// original gauge badge). A build for your own car can use others: generate them into private/
// (tools/gen_fonts.py, tools/gen_badge.py), which git ignores, and they're used instead.
// GAUGE_PUBLIC forces the shipped ones (tools/build_preview.sh uses it for the published preview).
#if !defined(GAUGE_PUBLIC) && __has_include("private/gauge_fonts.h")
#include "private/gauge_fonts.h"
#else
#include "gauge_fonts.h"
#endif
#if !defined(GAUGE_PUBLIC) && __has_include("private/gauge_badge.h")
#include "private/gauge_badge.h"
#else
#include "gauge_badge.h"
#endif
#include "pull_sum.h"

// Keep float maths in plain IEEE order on GCC (the ESP32 toolchain would otherwise
// fuse multiply-adds), so the car renders the same pixels as the wasm preview.
// Also build this code at -O2: Arduino compiles at -Os, which is much slower for these pixel loops.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC optimize("O2", "fp-contract=off")
#endif

namespace gr {

static const int   W = 466, H = 466;
static const float C = 233.0f;
static const float PI_F = 3.14159265f, D2R = PI_F / 180.0f, R2D = 180.0f / PI_F;
static const float PSI_TO_INHG = 2.03602f, PSI_TO_BAR = 0.0689476f;

enum Page { P_BOOST = 0, P_OVERVIEW, P_TEMPS, P_ENGINE, P_SYSTEM, P_GFORCE, P_PERF, P_GEARS, P_FUEL, P_HEAT, P_PULL, P_COUNT };

// Everything that changes the static face (from config.h on the car, sliders in the preview)
struct Cfg {
  float minPsi = -14.735f, maxPsi = 30.0f, redPsi = 20.0f;
  // AWE layout: 30 inHg at 6 o'clock, 0 at 9 o'clock, 30 psi at 3 o'clock
  float startDeg = 90.0f, zeroDeg = 180.0f, endDeg = 360.0f;
  float shiftRpm = 7200.0f;                  // shift ring flashes here
  float redlineRpm = 7400.0f;                // tach red zone starts here
  float tachMax = 8000.0f;
  bool  telltale = true, digital = true;
};

// Live values for one frame
struct Data {
  float boost = 0, peak = 0;                 // psi, relative to atmosphere (needle)
  float boostText = 0;                       // same, smoothed more for the digital readout
  bool  obd = false;                         // false = OBD values show "--"
  float rpm = 0, thr = 0, iatF = 0, cltF = 0, batV = 0, timing = 0;
  float oilF = 0;
  float maxIat = 0, maxRpm = 0;
  // where the red peak telltales sit on the Boost / Overview / Engine dials. The pod lets them
  // fall back a few seconds after a pull; NOV = use peak / maxRpm (held until reset)
  float peakNeedle = NOV, rpmNeedle = NOV;
  // the boost the ECU is asking for, psi (NOV = not asking for boost, or no OBD): a green
  // marker on the Boost dial, with a thin arc from the needle up to it while it falls short
  float tgtPsi = NOV;
  int   sysAlert = -1;                       // System item forced to the top by a warning (-1 = none)
  const char *status = "";  bool statusWarn = false;   // small line near the bottom
  const char *toast = "";   float toastA = 0;          // centre message, 0..1 fade
  float dotsA = 0;                                     // page dots, shown after a swipe
  const uint8_t *sysOrder = nullptr;                   // System page: [0] = main item, [1..4] = small cells
  // gear (calculated from RPM / speed): -1 = feature off (no chip drawn), 0 = unknown or shifting, 1..7
  int   gear = -1;
  char  trans = 0;                                     // gearbox program: 'S' or 'M' tags the gear chip (0 / 'D' = none)
  int   driveMode = 0;                                 // drive select (sport differential): 1 comfort, 2 auto, 3 dynamic, 0 = unknown
  // G-meter, car frame: gLat + = accelerating to the right, gLon + = accelerating forward (g)
  float gLat = 0, gLon = 0;
  const float *gTrail = nullptr; int gTrailN = 0;      // past (lat, lon) pairs, oldest first
  float gPkL = 0, gPkR = 0, gPkAcc = 0, gPkBrk = 0;    // peak magnitudes since the last reset
  int   gCal = 0;                                      // 0 = needs a level stop, 1 = needs a straight pull, 2 = ready
  // performance timer (mph): runState 0 = not armed, 1 = armed at a stop, 2 = running
  int   runState = 0;
  float runT060 = 0, runT6130 = 0, runTQm = 0;        // running clocks (0 = that one isn't running)
  float runMph = 0;                                    // speed now, while running
  const char *runMsg = "";                             // why it isn't armed ("STOP TO ARM", "NO SPEED FROM OBD")
  bool  runObd = false;                                // timing from OBD speed alone (no G-meter yet)
  int   epb = -1, brake = -1;                          // parking brake / brake pedal: 1 on, 0 off, -1 unknown
  bool  staged = false;                                // held at a stop with the revs up (launch control)
  float t060 = 0, t60130 = 0, tQm = 0, qmMph = 0;      // last results (0 = none yet)
  float b060 = 0, b60130 = 0, bQm = 0;                 // best this session
  const float *gearPeak = nullptr;                     // [7] peak boost per gear, psi (0 = none)
  // fuel, air-fuel ratio, heat (NOV = no reading)
  float afr1 = NOV, afr2 = NOV, afrCmd = NOV;          // AFR per bank, commanded
  float hpfp = NOV, hpfpSet = NOV, lpfp = NOV;         // fuel pressures, psi
  float idc = NOV;                                     // injector duty, %
  float knock = NOV; int knockCyl = 0;                 // worst cylinder's knock retard (<= 0), which one
  float cat1F = NOV, cat2F = NOV, ambF = NOV, preF = NOV, icPump = NOV;
  float clutchF = NOV, gbxF = NOV, diffF = NOV;         // S tronic clutch oil (7E1 1259), gearbox (125A), sport diff oil (71D 1307), degF
  bool  onBoost = false;                               // boost > ~2 psi right now (lean / fuel colours apply)
  // warning: a big round pop-up in the middle first (alertBig 1 -> 0 as it fades), then a
  // banner across the top of every page while it lasts (empty = none)
  const char *alertText = ""; bool alertRed = false;
  const char *alertTitle = "", *alertValue = "", *alertUnit = "", *alertNote = "";
  float alertBig = 0, alertPulse = 0;                  // pop-up opacity; 0..1 pulse phase of its ring
  // warm-up: oil temperature while it's below about 170 F (NOV = warm, or no reading)
  float coldOilF = NOV;
  const PullSum *pull = nullptr;                       // last pull (Last Pull page)
};

// ---------------------------------------------------------------- maths
static inline float fabs_(float x) { return x < 0 ? -x : x; }
static inline float clampf(float v, float a, float b) { return v < a ? a : (v > b ? b : v); }
static inline float minf(float a, float b) { return a < b ? a : b; }
static inline float maxf(float a, float b) { return a > b ? a : b; }
static inline float sqrt_(float x) { return __builtin_sqrtf(x); }
static inline int   ifloor(float x) { int i = (int)x; return i - (x < (float)i); }
static inline float lerp(float a, float b, float t) { return a + (b - a) * t; }

static float sin_(float x) {                       // deterministic on every target
  const float TAU_F = 6.28318531f;
  x -= TAU_F * (float)ifloor(x / TAU_F + 0.5f);  // -> [-pi, pi]
  if (x > 1.57079633f) x = PI_F - x;
  else if (x < -1.57079633f) x = -PI_F - x;
  float x2 = x * x;
  return x * (1.0f + x2 * (-1.0f / 6 + x2 * (1.0f / 120 + x2 * (-1.0f / 5040 + x2 * (1.0f / 362880 - x2 / 39916800.0f)))));
}
static inline float cos_(float x) { return sin_(x + 1.57079633f); }
static float atan2deg(float y, float x) {          // degrees, -180..180
  float ax = fabs_(x), ay = fabs_(y);
  if (ax == 0 && ay == 0) return 0;
  float z = ax > ay ? ay / ax : ax / ay, z2 = z * z;
  float a = z * (0.99997726f + z2 * (-0.33262347f + z2 * (0.19354346f + z2 * (-0.11643287f + z2 * (0.05265332f + z2 * -0.01172120f)))));
  if (ay > ax) a = 1.57079633f - a;
  if (x < 0) a = PI_F - a;
  if (y < 0) a = -a;
  return a * R2D;
}
static inline float wrap180(float a) { a -= 360.0f * (float)ifloor((a + 180.0f) / 360.0f); return a; }

// ---------------------------------------------------------------- colour + target
struct Col { float r, g, b; };
static inline Col hex(uint32_t h) { return { (float)((h >> 16) & 255), (float)((h >> 8) & 255), (float)(h & 255) }; }
static inline Col mix(Col a, Col b, float t) { return { lerp(a.r, b.r, t), lerp(a.g, b.g, t), lerp(a.b, b.b, t) }; }
static inline Col scale(Col a, float k) { return { a.r * k, a.g * k, a.b * k }; }

static uint16_t *fb = nullptr;
static inline void target(uint16_t *buf) { fb = buf; }

static inline uint16_t pack(Col c) {
  int r = (int)(clampf(c.r, 0, 255) * 31.0f / 255.0f + 0.5f);
  int g = (int)(clampf(c.g, 0, 255) * 63.0f / 255.0f + 0.5f);
  int b = (int)(clampf(c.b, 0, 255) * 31.0f / 255.0f + 0.5f);
  return (uint16_t)((r << 11) | (g << 5) | b);
}
static const uint8_t BAYER[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};
static inline uint16_t packDither(Col c, int x, int y) {     // ordered dither: no banding in gradients
  float d = (BAYER[y & 3][x & 3] + 0.5f) / 16.0f;
  int r = ifloor(clampf(c.r, 0, 255) * 31.0f / 255.0f + d); if (r > 31) r = 31;
  int g = ifloor(clampf(c.g, 0, 255) * 63.0f / 255.0f + d); if (g > 63) g = 63;
  int b = ifloor(clampf(c.b, 0, 255) * 31.0f / 255.0f + d); if (b > 31) b = 31;
  return (uint16_t)((r << 11) | (g << 5) | b);
}
static inline Col unpack(uint16_t v) {
  return { (float)((v >> 11) & 31) * (255.0f / 31), (float)((v >> 5) & 63) * (255.0f / 63), (float)(v & 31) * (255.0f / 31) };
}

// ---------------------------------------------------------------- screen orientation, pixel shift, dirty rows
// Drawing code works in upright "logical" coordinates. pidx() maps to the panel:
//   rotQ    = quarter turns counter-clockwise (DISPLAY_ROTATION in config.h)
//   offX/Y  = burn-in pixel shift
// While a frame is being drawn, every touched pixel widens that row's dirty span, so the
// next frame only restores those spans from the face and only those rows are sent to the panel.
static int rotQ = 0, offX = 0, offY = 0;
static bool tracking = false;
static int16_t dMin[H], dMax[H];
static inline void setRotation(int q) { rotQ = q & 3; }
static inline void setShift(int dx, int dy) { offX = dx; offY = dy; }
static inline void physical(int x, int y, int &px, int &py) {
  switch (rotQ) {
    case 0:  px = x;         py = y;         break;
    case 1:  px = y;         py = W - 1 - x; break;   // 90 deg CCW
    case 2:  px = W - 1 - x; py = H - 1 - y; break;
    default: px = H - 1 - y; py = x;         break;   // 90 deg CW
  }
}
// Touch arrives in panel coordinates; turn it back into logical ones
static inline void logical(int px, int py, int &x, int &y) {
  switch (rotQ) {
    case 0:  x = px;         y = py;         break;
    case 1:  x = W - 1 - py; y = px;         break;
    case 2:  x = W - 1 - px; y = H - 1 - py; break;
    default: x = py;         y = H - 1 - px; break;
  }
}
static inline int pidx(int x, int y) {
  if ((unsigned)x >= (unsigned)W || (unsigned)y >= (unsigned)H) return -1;
  int px, py;
  physical(x, y, px, py);
  px += offX; py += offY;
  if ((unsigned)px >= (unsigned)W || (unsigned)py >= (unsigned)H) return -1;
  if (tracking) { if (px < dMin[py]) dMin[py] = (int16_t)px; if (px > dMax[py]) dMax[py] = (int16_t)px; }
  return py * W + px;
}

// Recording: while recLayer >= 0, blends are captured (see hub()) instead of drawn
static int recLayer = -1;
static void recordBlend(int x, int y, uint16_t s, int a);

// Integer blend in RGB565: a = 0..256
static inline void blendI(int x, int y, uint16_t s, int a) {
  if (a <= 0) return;
  if (recLayer >= 0) { recordBlend(x, y, s, a); return; }
  int i = pidx(x, y);
  if (i < 0) return;
  uint16_t *p = fb + i;
  if (a >= 256) { *p = s; return; }
  int d = *p;
  int dr = d >> 11, dg = (d >> 5) & 63, db = d & 31;
  dr += (((s >> 11) - dr) * a + 128) >> 8;
  dg += ((((s >> 5) & 63) - dg) * a + 128) >> 8;
  db += (((s & 31) - db) * a + 128) >> 8;
  *p = (uint16_t)((dr << 11) | (dg << 5) | db);
}
static inline int a256(float a) { return a <= 0 ? 0 : (a >= 1 ? 256 : (int)(a * 256.0f + 0.5f)); }
static inline void blend(int x, int y, Col c, float a) { blendI(x, y, pack(c), a256(a)); }

// ---------------------------------------------------------------- frame bookkeeping (firmware)
// beginFrame: put back what the last frame drew (or the whole face when full), start tracking.
// endFrame:   rows that changed this frame or were restored, as runs for a partial panel update.
static int prevY0 = H, prevY1 = -1;
static void beginFrame(const uint16_t *face, bool full) {
  int n = offY * W + offX;
  prevY0 = H; prevY1 = -1;
  for (int y = 0; y < H; y++) {
    int x0 = 0, x1 = W - 1;
    if (!full) {
      if (dMax[y] < dMin[y]) { dMin[y] = W; dMax[y] = -1; continue; }
      x0 = dMin[y]; x1 = dMax[y];
      if (y < prevY0) prevY0 = y;
      prevY1 = y;
    }
    int i0 = y * W + x0, i1 = y * W + x1, j0 = i0 - n, j1 = i1 - n;
    if (j0 >= 0 && j1 < W * H) __builtin_memcpy(fb + i0, face + j0, (__SIZE_TYPE__)(x1 - x0 + 1) * 2);
    else for (int i = i0; i <= i1; i++) { int j = i - n; fb[i] = (j >= 0 && j < W * H) ? face[j] : 0; }
    dMin[y] = W; dMax[y] = -1;
  }
  tracking = true;
}
// Fills runs[] with {y0, y1} row ranges to send (gaps under 12 rows merged, starts even, heights even).
static int endFrame(int16_t (*runs)[2], int maxRuns) {
  tracking = false;
  int n = 0, y = 0;
  while (y < H && n < maxRuns) {
    bool d = dMax[y] >= dMin[y] || (y >= prevY0 && y <= prevY1);
    if (!d) { y++; continue; }
    int y0 = y, y1 = y, gap = 0;
    for (y = y + 1; y < H; y++) {
      bool dd = dMax[y] >= dMin[y] || (y >= prevY0 && y <= prevY1);
      if (dd) { y1 = y; gap = 0; } else if (++gap > 12) break;
    }
    y0 &= ~1; if (!(y1 & 1)) y1++; if (y1 > H - 1) y1 = H - 1;
    if (n && y0 <= runs[n - 1][1] + 1) runs[n - 1][1] = (int16_t)y1;
    else { runs[n][0] = (int16_t)y0; runs[n][1] = (int16_t)y1; n++; }
    y = y1 + 1;
  }
  if (n == maxRuns)                                    // out of slots: the last run takes the rest
    for (int yy = H - 1; yy > runs[n - 1][1]; yy--)
      if (dMax[yy] >= dMin[yy] || (yy >= prevY0 && yy <= prevY1)) { runs[n - 1][1] = (int16_t)(yy | 1) < H ? (int16_t)(yy | 1) : (int16_t)(H - 1); break; }
  return n;
}

// ---------------------------------------------------------------- shapes (anti-aliased)
struct Pt { float x, y; };

// Same blend as blendI() on a pixel address (row-walking fast paths below use it)
static inline void blendRaw(uint16_t *p, uint16_t s, int a) {
  if (a >= 256) { *p = s; return; }
  int d = *p;
  int dr = d >> 11, dg = (d >> 5) & 63, db = d & 31;
  dr += (((s >> 11) - dr) * a + 128) >> 8;
  dg += ((((s >> 5) & 63) - dg) * a + 128) >> 8;
  db += (((s & 31) - db) * a + 128) >> 8;
  *p = (uint16_t)((dr << 11) | (dg << 5) | db);
}
// Walks a logical row x, x+1, x+2 ... on the panel: one step is a fixed move in panel
// coordinates for each rotation, so there's no per-pixel coordinate mapping.
struct RowWalk {
  int px, py, sx, sy;
  inline void start(int x, int y) {
    physical(x, y, px, py); px += offX; py += offY;
    switch (rotQ) { case 0: sx = 1; sy = 0; break; case 1: sx = 0; sy = -1; break;
                    case 2: sx = -1; sy = 0; break; default: sx = 0; sy = 1; break; }
  }
  inline void skip(int n) { px += sx * n; py += sy * n; }
  inline uint16_t *here() {                         // address of the current pixel (tracked), or null
    if ((unsigned)px >= (unsigned)W || (unsigned)py >= (unsigned)H) return nullptr;
    if (tracking) { if (px < dMin[py]) dMin[py] = (int16_t)px; if (px > dMax[py]) dMax[py] = (int16_t)px; }
    return fb + py * W + px;
  }
  inline void put(uint16_t s, int a) {             // blend here, then move to the next x
    if (a > 0 && (unsigned)px < (unsigned)W && (unsigned)py < (unsigned)H) {
      if (tracking) { if (px < dMin[py]) dMin[py] = (int16_t)px; if (px > dMax[py]) dMax[py] = (int16_t)px; }
      blendRaw(fb + py * W + px, s, a);
    }
    px += sx; py += sy;
  }
};

// Edge band of a polygon row: N edges' signed distances (16.16 fixed point) stepped one
// pixel at a time. N is a compile-time constant so the distances stay in registers.
#if defined(__GNUC__) && !defined(__clang__)
#define GR_UNROLL _Pragma("GCC unroll 8")
#else
#define GR_UNROLL _Pragma("unroll")
#endif
template <int N>
static inline void polyBand(RowWalk &w, const int32_t *d0, const int32_t *st, int count,
                            uint16_t c5, int full, int32_t fQ, int32_t kQ14) {
  int32_t d[N], s[N];
  GR_UNROLL for (int i = 0; i < N; i++) { d[i] = d0[i]; s[i] = st[i]; }
  for (int j = 0; j < count; j++) {
    int32_t m = d[0];
    GR_UNROLL for (int i = 1; i < N; i++) m = d[i] < m ? d[i] : m;
    GR_UNROLL for (int i = 0; i < N; i++) d[i] += s[i];
    int cov;                                            // 0..256 = clamp(d * k + 0.5)
    if (m >= fQ) cov = 256;
    else if (m <= -fQ) cov = 0;
    else { cov = 128 + (((m >> 4) * kQ14) >> 18); cov = cov < 0 ? 0 : (cov > 256 ? 256 : cov); }
    w.put(c5, (full * cov + 128) >> 8);
  }
}

// Convex polygon. feather 0.5 = crisp AA edge; larger = soft edge (used for shadows).
// Per row: the covered x range is found analytically; the fully covered middle is filled
// without per-pixel maths; in the edge bands each edge's signed distance is stepped along
// the row in fixed point (polyBand).
static void poly(const Pt *p, int n, Col c, float alpha = 1, float feather = 0.5f) {
  float A[8], B[8], Cc[8], area = 0;
  if (n < 3 || n > 8) return;
  float ymin = 1e9f, ymax = -1e9f;
  for (int i = 0; i < n; i++) {
    const Pt &a = p[i], &b = p[(i + 1) % n];
    area += a.x * b.y - b.x * a.y;
    ymin = minf(ymin, a.y); ymax = maxf(ymax, a.y);
  }
  float s = area >= 0 ? 1.0f : -1.0f;
  for (int i = 0; i < n; i++) {
    const Pt &a = p[i], &b = p[(i + 1) % n];
    float dx = b.x - a.x, dy = b.y - a.y, len = sqrt_(dx * dx + dy * dy);
    if (len < 1e-4f) { A[i] = 0; B[i] = 0; Cc[i] = 1e9f; continue; }
    A[i] = -dy / len * s; B[i] = dx / len * s;
    Cc[i] = -(A[i] * a.x + B[i] * a.y);
  }
  int y0 = ifloor(ymin - feather), y1 = ifloor(ymax + feather);
  if (y0 < 0) y0 = 0;
  if (y1 > H - 1) y1 = H - 1;
  float k = 1.0f / (2 * feather);
  uint16_t c5 = pack(c);
  int full = a256(alpha);
  const int32_t fQ = (int32_t)(feather * 65536.0f);              // |d| beyond this: coverage 0 or 1
  const int32_t kQ14 = (int32_t)(k * 16384.0f + 0.5f);
  int32_t step[8];
  for (int i = 0; i < n; i++) step[i] = (int32_t)ifloor(A[i] * 65536.0f + 0.5f);
  for (int y = y0; y <= y1; y++) {
    float py = y + 0.5f, xl = 0, xr = W;
    bool empty = false;
    for (int i = 0; i < n && !empty; i++) {
      float rest = -feather - B[i] * py - Cc[i];        // need A*x > rest
      if (A[i] > 1e-6f) xl = maxf(xl, rest / A[i]);
      else if (A[i] < -1e-6f) xr = minf(xr, rest / A[i]);
      else if (rest >= 0) empty = true;
    }
    if (empty || xl > xr) continue;
    int xa = ifloor(xl - 0.5f), xb = ifloor(xr - 0.5f) + 1;
    if (xa < 0) xa = 0;
    if (xb > W - 1) xb = W - 1;
    if (recLayer >= 0) {                                // recording (hub cache): per-pixel path
      for (int x = xa; x <= xb; x++) {
        float px = x + 0.5f, d = 1e9f;
        for (int i = 0; i < n; i++) d = minf(d, A[i] * px + B[i] * py + Cc[i]);
        blendI(x, y, c5, a256(alpha * clampf(d * k + 0.5f, 0, 1)));
      }
      continue;
    }
    // fully covered middle (every edge at least feather + margin away)
    float il = 0, ir = W;
    for (int i = 0; i < n; i++) {
      float rest = feather + 0.02f - B[i] * py - Cc[i];
      if (A[i] > 1e-6f) il = maxf(il, rest / A[i]);
      else if (A[i] < -1e-6f) ir = minf(ir, rest / A[i]);
      else if (rest > 0) { il = 1; ir = 0; }
    }
    int ia = ifloor(il - 0.5f) + 1, ib = ifloor(ir - 0.5f);
    if (ia < xa) ia = xa;
    if (ib > xb) ib = xb;
    auto band = [&](int x0, int x1) {
      if (x0 > x1) return;
      int32_t d0[8];
      float px0 = x0 + 0.5f;
      for (int i = 0; i < n; i++) d0[i] = (int32_t)ifloor((A[i] * px0 + B[i] * py + Cc[i]) * 65536.0f + 0.5f);
      RowWalk w; w.start(x0, y);
      int cnt = x1 - x0 + 1;
      switch (n) {
        case 3:  polyBand<3>(w, d0, step, cnt, c5, full, fQ, kQ14); break;
        case 4:  polyBand<4>(w, d0, step, cnt, c5, full, fQ, kQ14); break;
        case 5:  polyBand<5>(w, d0, step, cnt, c5, full, fQ, kQ14); break;
        case 6:  polyBand<6>(w, d0, step, cnt, c5, full, fQ, kQ14); break;
        case 7:  polyBand<7>(w, d0, step, cnt, c5, full, fQ, kQ14); break;
        default: polyBand<8>(w, d0, step, cnt, c5, full, fQ, kQ14); break;
      }
    };
    if (ia <= ib) {
      band(xa, ia - 1);
      RowWalk w; w.start(ia, y);
      for (int x = ia; x <= ib; x++) w.put(c5, full);
      band(ib + 1, xb);
    } else band(xa, xb);
  }
}

static inline Pt polar(float deg, float r) { float a = deg * D2R; return { C + r * cos_(a), C + r * sin_(a) }; }

// Radial tick from r0 to r1, w = full width (like a canvas lineWidth)
static void tick(float deg, float r0, float r1, float w, Col c, float alpha = 1) {
  float a = deg * D2R, ca = cos_(a), sa = sin_(a), hx = -sa * w / 2, hy = ca * w / 2;
  Pt q[4] = {{C + r0 * ca + hx, C + r0 * sa + hy}, {C + r1 * ca + hx, C + r1 * sa + hy},
             {C + r1 * ca - hx, C + r1 * sa - hy}, {C + r0 * ca - hx, C + r0 * sa - hy}};
  poly(q, 4, c, alpha);
}

static void rect(float x, float y, float w, float h, Col c, float alpha = 1) {
  Pt q[4] = {{x, y}, {x + w, y}, {x + w, y + h}, {x, y + h}};
  poly(q, 4, c, alpha);
}

// Ring segment between radii r0..r1 and angles a0..a1 (degrees, clockwise from 3 o'clock)
static void arc(float r0, float r1, float a0, float a1, Col c, float alpha = 1, float cx = C, float cy = C) {
  bool full = a1 - a0 >= 359.9f;
  float mid = (a0 + a1) / 2, half = (a1 - a0) / 2;
  // bounding box of the sector
  float bx0 = cx - r1 - 1, bx1 = cx + r1 + 1, by0 = cy - r1 - 1, by1 = cy + r1 + 1;
  if (!full) {
    bx0 = by0 = 1e9f; bx1 = by1 = -1e9f;
    float angs[8]; int na = 0;
    angs[na++] = a0; angs[na++] = a1;
    for (int q = -360; q <= 720; q += 90) if (q > a0 && q < a1 && na < 8) angs[na++] = (float)q;
    for (int i = 0; i < na; i++) for (int j = 0; j < 2; j++) {
      float r = j ? r1 : maxf(r0, 0), a = angs[i] * D2R;
      float x = cx + r * cos_(a), y = cy + r * sin_(a);
      bx0 = minf(bx0, x); bx1 = maxf(bx1, x); by0 = minf(by0, y); by1 = maxf(by1, y);
    }
    bx0 -= 2; by0 -= 2; bx1 += 2; by1 += 2;
  }
  float ro = r1 + 0.5f, ri = r0 - 0.5f;
  uint16_t c5 = pack(c);
  // Angular fast path: a pixel whose direction is within `thr` of the middle angle is fully
  // inside the sector (angular coverage exactly 1), so the atan2 can be skipped. thr is taken
  // at the inner radius (the tightest case) with a margin, so the result is unchanged.
  float cm = cos_(mid * D2R), sm = sin_(mid * D2R), cosThr = 2;   // 2 = never
  if (!full) {
    float rMin = maxf(r0 - 0.5f, 1.0f);
    float thr = half - (0.5f / (rMin * D2R)) - 0.05f;              // degrees
    if (thr > 0 && thr < 179) cosThr = cos_(thr * D2R);
  }
  int ya = ifloor(maxf(by0, cy - ro)), yb = ifloor(minf(by1, cy + ro));
  if (ya < 0) ya = 0;
  if (yb > H - 1) yb = H - 1;
  for (int y = ya; y <= yb; y++) {
    float dy = y + 0.5f - cy;
    if (dy * dy >= ro * ro) continue;
    float xo = sqrt_(ro * ro - dy * dy), xi = (ri > 0 && dy * dy < ri * ri) ? sqrt_(ri * ri - dy * dy) : -1;
    for (int side = 0; side < 2; side++) {
      float xs0, xs1;
      if (xi < 0) { if (side) break; xs0 = cx - xo; xs1 = cx + xo; }
      else if (!side) { xs0 = cx - xo; xs1 = cx - xi; }
      else { xs0 = cx + xi; xs1 = cx + xo; }
      xs0 = maxf(xs0, bx0); xs1 = minf(xs1, bx1);
      int xa = ifloor(xs0 - 0.5f), xb = ifloor(xs1 - 0.5f) + 1;
      if (xa < 0) xa = 0;
      if (xb > W - 1) xb = W - 1;
      for (int x = xa; x <= xb; x++) {
        float dx = x + 0.5f - cx, r = sqrt_(dx * dx + dy * dy);
        float cov = clampf(minf(r0 > 0 ? r - r0 : 1e9f, r1 - r) + 0.5f, 0, 1);
        if (cov <= 0) continue;
        if (!full && !(dx * cm + dy * sm >= r * cosThr)) {
          float d = fabs_(wrap180(atan2deg(dy, dx) - mid));
          cov *= clampf((half - d) * D2R * r + 0.5f, 0, 1);
        }
        blendI(x, y, c5, a256(alpha * cov));
      }
    }
  }
}
static inline void disc(float x, float y, float r, Col c, float alpha = 1) { arc(0, r, 0, 360, c, alpha, x, y); }

// ---------------------------------------------------------------- text
static uint32_t utf8(const char *&s) {
  uint8_t c = (uint8_t)*s++;
  if (c < 0x80) return c;
  if ((c & 0xE0) == 0xC0 && *s) return ((c & 0x1F) << 6) | ((uint8_t)*s++ & 0x3F);
  if ((c & 0xF0) == 0xE0 && s[0] && s[1]) { uint32_t v = ((c & 0x0F) << 12) | (((uint8_t)s[0] & 0x3F) << 6) | ((uint8_t)s[1] & 0x3F); s += 2; return v; }
  return '?';
}
static const GGlyph *glyph(const GFont &f, uint32_t cp) {
  if (cp == '-' ) { for (int i = 0; i < f.n; i++) if (f.g[i].cp == 0x2212) return &f.g[i]; }   // use the real minus sign
  for (int i = 0; i < f.n; i++) if (f.g[i].cp == cp) return &f.g[i];
  return nullptr;
}
static float textWidth(const GFont &f, const char *s, float spacing = 0) {
  float w = 0; int n = 0;
  while (*s) { const GGlyph *g = glyph(f, utf8(s)); if (g) { w += g->adv64 / 64.0f; n++; } }
  return w + (n > 1 ? spacing * (n - 1) : 0);
}
// y = vertical centre of the capitals/digits. align: -1 left, 0 centre, 1 right
static void text(const GFont &f, const char *s, float x, float y, Col c, int align = 0, float spacing = 0, float alpha = 1) {
  float w = textWidth(f, s, spacing);
  float pen = align < 0 ? x : (align == 0 ? x - w / 2 : x - w);
  int base = ifloor(y + f.capH / 2.0f + 0.5f);
  uint16_t c5 = pack(c);
  int av[16];
  for (int i = 0; i < 16; i++) av[i] = a256(alpha * i / 15.0f);
  while (*s) {
    const GGlyph *g = glyph(f, utf8(s));
    if (!g) continue;
    int gx = ifloor(pen + 0.5f) + g->xo, gy = base + g->yo;
    for (int yy = 0; yy < g->h; yy++)
      for (int xx = 0; xx < g->w; xx++) {
        uint32_t ni = g->off + (uint32_t)(yy * g->w + xx);
        uint8_t v = (f.bmp[ni >> 1] >> ((ni & 1) ? 0 : 4)) & 15;
        if (v) blendI(gx + xx, gy + yy, c5, av[v]);
      }
    pen += g->adv64 / 64.0f + spacing;
  }
}
static inline void label(const GFont &f, const char *s, float x, float y, Col c, float alpha = 1) { text(f, s, x, y, c, 0, 1.6f, alpha); }

// ---------------------------------------------------------------- images (RGB565 + 8-bit alpha)
static void image(const GImage &im, float cx, float cy, float alpha = 1) {
  int x0 = ifloor(cx - im.w / 2.0f + 0.5f), y0 = ifloor(cy - im.h / 2.0f + 0.5f);
  for (int y = 0; y < im.h; y++)
    for (int x = 0; x < im.w; x++) {
      uint8_t a = im.a[y * im.w + x];
      if (a) blendI(x0 + x, y0 + y, im.rgb[y * im.w + x], a256(alpha * a / 255.0f));
    }
}

// ---------------------------------------------------------------- number formatting
static char *fmt(char *buf, float v, int dec) {               // like "%.*f"
  char *p = buf;
  float m = 1; for (int i = 0; i < dec; i++) m *= 10;
  long q = (long)(fabs_(v) * m + 0.5f);
  if (v < 0 && q > 0) *p++ = '-';
  long ip = q, frac = 0;
  if (dec) { ip = q / (long)m; frac = q % (long)m; }
  char tmp[12]; int n = 0;
  do { tmp[n++] = (char)('0' + ip % 10); ip /= 10; } while (ip);
  while (n) *p++ = tmp[--n];
  if (dec) { *p++ = '.'; long d = (long)m / 10; while (d) { *p++ = (char)('0' + (frac / d) % 10); d /= 10; } }
  *p = 0;
  return buf;
}
static char *cat(char *dst, const char *a, const char *b = "", const char *c = "") {
  char *p = dst;
  const char *parts[3] = {a, b, c};
  for (int i = 0; i < 3; i++) for (const char *s = parts[i]; *s;) *p++ = *s++;
  *p = 0;
  return dst;
}

// ---------------------------------------------------------------- palette (S-gauge face)
static const Col kWhite  = {244, 244, 241}, kSilver = {201, 204, 209}, kGrey = {154, 159, 166};
static const Col kDim    = {138, 143, 150}, kRed = {242, 22, 62};
static Col kFaint = {23, 25, 28};          // empty tracks; a touch lighter on the black night face
static const Col kAmber  = {255, 176, 32},  kGreen = {60, 207, 122}, kBlack = {0, 0, 0};
static const Col kLabel  = {212, 215, 219}, kUnit  = {143, 148, 155}, kCharge = {43, 90, 61};
static const Col kCold   = {96, 170, 255};          // warm-up cue
static const Col kTarget = {60, 207, 122};          // the ECU's requested boost

// ---------------------------------------------------------------- theme
// Day: charcoal S-gauge face. Night (headlights on): true black AMOLED face, so only
// the scale, numbers and needle light up - less glare, and no lit grey to burn in.
static bool dark = false;
static inline void setTheme(bool night) { dark = night; kFaint = night ? Col{30, 32, 36} : Col{23, 25, 28}; }

static void faceNight() {
  for (int i = 0; i < W * H; i++) fb[i] = 0;
  arc(228.5f, 231.0f, 0, 360, hex(0x2b2e33));                  // faint bezel edge
}

// ---------------------------------------------------------------- the face
// Charcoal dial with fine machined rings, a chrome bezel ring and a thin dark seam.
// The charcoal dial is the same on every page: the firmware hands in a spare buffer so it is
// computed once at boot and copied for the other pages (drawn with the same rotation).
static uint16_t *faceBase = nullptr;
static bool faceBaseOk = false;
static inline void setFaceCache(uint16_t *buf) { faceBase = buf; faceBaseOk = false; }
// The firmware can compute the dial ahead of time on its other core (precomputeFace, below)
// and hand it in already done.
static inline void setFaceCacheReady(uint16_t *buf) { faceBase = buf; faceBaseOk = buf != nullptr; }
template <class S> static void faceCompute(S store);
static void face() {
  if (dark) { faceNight(); return; }
  if (faceBase && faceBaseOk) { __builtin_memcpy(fb, faceBase, (__SIZE_TYPE__)W * H * 2); return; }
  faceCompute([](int x, int y, uint16_t v) { int i = pidx(x, y); if (i >= 0) fb[i] = v; });
  if (faceBase) { __builtin_memcpy(faceBase, fb, (__SIZE_TYPE__)W * H * 2); faceBaseOk = true; }
}
// The same dial straight into buf, touching no shared drawing state (target, pixel shift, dirty
// spans): safe to run on one core while the other draws something else. Only reads the
// rotation, so call setRotation() first. Gives what face() draws with no pixel shift.
static inline void precomputeFace(uint16_t *buf) {
  faceCompute([buf](int x, int y, uint16_t v) { int px, py; physical(x, y, px, py); buf[py * W + px] = v; });
}
template <class S> static void faceCompute(S store) {
  const Col g0 = hex(0x4a4e55), g1 = hex(0x2d3035), g2 = hex(0x141518);
  const Col ch[6] = {hex(0xe3e6ea), hex(0x676c74), hex(0xf1f3f5), hex(0x555a62), hex(0xcfd3d8), hex(0xe3e6ea)};
  const float chs[6] = {0, 0.17f, 0.4f, 0.6f, 0.8f, 1.0f};
  for (int y = 0; y < H; y++) {
    float dy = y + 0.5f - C;
    for (int x = 0; x < W; x++) {
      float dx = x + 0.5f - C, r = sqrt_(dx * dx + dy * dy);
      if (r > 233.5f) { store(x, y, 0); continue; }
      // dial: radial gradient, lit from slightly above centre
      float gy = dy + 40, t = clampf(sqrt_(dx * dx + gy * gy) / 250.0f, 0, 1);
      Col c = t < 0.55f ? mix(g0, g1, t / 0.55f) : mix(g1, g2, (t - 0.55f) / 0.45f);
      if (r > 24 && r < 222) c = scale(c, 1.0f + 0.035f * sin_(r * (2 * PI_F / 5.0f)));   // machined rings
      // dark seam r 222.75..224.25
      c = mix(c, hex(0x070708), clampf(minf(r - 222.75f, 224.25f - r) + 0.5f, 0, 1));
      // chrome ring r 224..232: conic sweep plus a rounded bevel across its width
      float cr = clampf(minf(r - 224.0f, 232.0f - r) + 0.5f, 0, 1);
      if (cr > 0) {
        float u = wrap180(atan2deg(dy, dx) + 60.0f) / 360.0f; if (u < 0) u += 1;
        int k = 0; while (k < 4 && u > chs[k + 1]) k++;
        Col m = mix(ch[k], ch[k + 1], (u - chs[k]) / (chs[k + 1] - chs[k]));
        float s = (r - 228.0f) / 4.0f;
        m = scale(m, 0.72f + 0.28f * (1 - s * s) - 0.10f * s);
        c = mix(c, m, cr);
      }
      c = scale(c, clampf(233.0f - r, 0, 1));              // soft outer edge
      store(x, y, packDither(c, x, y));
    }
  }
}

// Needle with a soft drop shadow. thin = slim telltale shape
static int needleParts = 3;                      // profiling only: 1 = shadow, 2 = body
static void needle(float deg, float len, float tail, float w, Col c, bool thin) {
  float a = deg * D2R, ca = cos_(a), sa = sin_(a), px = -sa, py = ca;
  Pt p[6]; int n;
  if (thin) {
    p[0] = {C + ca * len, C + sa * len};
    p[1] = {C + ca * (len - 26) + px * 3.2f, C + sa * (len - 26) + py * 3.2f};
    p[2] = {C + ca * 30 + px * w, C + sa * 30 + py * w};
    p[3] = {C + ca * 30 - px * w, C + sa * 30 - py * w};
    p[4] = {C + ca * (len - 26) - px * 3.2f, C + sa * (len - 26) - py * 3.2f};
    n = 5;
  } else {
    p[0] = {C + ca * len + px * 1.2f, C + sa * len + py * 1.2f};
    p[1] = {C + ca * len - px * 1.2f, C + sa * len - py * 1.2f};
    p[2] = {C - px * w, C - py * w};
    p[3] = {C - ca * tail - px * w * 0.7f, C - sa * tail - py * w * 0.7f};
    p[4] = {C - ca * tail + px * w * 0.7f, C - sa * tail + py * w * 0.7f};
    p[5] = {C + px * w, C + py * w};
    n = 6;
  }
  Pt sh[6];
  for (int i = 0; i < n; i++) sh[i] = {p[i].x + 3, p[i].y + 4};
  if (needleParts & 1) poly(sh, n, kBlack, 0.55f, 3.0f);
  if (needleParts & 2) poly(p, n, c);
}

static void hubDraw(float r);
// The hub never changes, but costs a square root per pixel. It is drawn once into a small cache
// (shadow, body and rim alpha per pixel, plus body colour) and replayed each frame.
static const int HUB_HALF = 34, HUB_N = 2 * HUB_HALF + 1;
static struct { int r = -1; bool dark = false; uint8_t sh[HUB_N * HUB_N], ha[HUB_N * HUB_N], ra[HUB_N * HUB_N];
                uint16_t hc[HUB_N * HUB_N]; uint16_t rimC = 0; } hubC;
static const int HUB_X0 = (int)C - HUB_HALF, HUB_Y0 = (int)C - HUB_HALF;
static void recordBlend(int x, int y, uint16_t s, int a) {
  int bx = x - HUB_X0, by = y - HUB_Y0;
  if ((unsigned)bx >= (unsigned)HUB_N || (unsigned)by >= (unsigned)HUB_N) return;
  int i = by * HUB_N + bx;
  uint8_t v = (uint8_t)(a > 255 ? 255 : a);
  if (recLayer == 0) hubC.sh[i] = v;
  else if (recLayer == 1) { hubC.ha[i] = v; hubC.hc[i] = s; }
  else { hubC.ra[i] = v; hubC.rimC = s; }
}
static void hub(float r) {
  int ri = (int)(r + 0.5f);
  if (hubC.r != ri || hubC.dark != dark) {
    for (int i = 0; i < HUB_N * HUB_N; i++) { hubC.sh[i] = hubC.ha[i] = hubC.ra[i] = 0; hubC.hc[i] = 0; }
    hubDraw(r);                                   // records into the three layers
    recLayer = -1;
    hubC.r = ri; hubC.dark = dark;
  }
  // One pass: each pixel gets shadow, body, rim in that order (pixels don't interact, so
  // this matches drawing the three layers one after another)
  for (int by = 0; by < HUB_N; by++) {
    RowWalk w; w.start(HUB_X0, HUB_Y0 + by);
    const int row = by * HUB_N;
    for (int bx = 0; bx < HUB_N; bx++, w.skip(1)) {
      int i = row + bx;
      uint8_t a0 = hubC.sh[i], a1 = hubC.ha[i], a2 = hubC.ra[i];
      if (!(a0 | a1 | a2)) continue;
      uint16_t *q = w.here();
      if (!q) continue;
      if (a0) blendRaw(q, 0, a0 == 255 ? 256 : a0);
      if (a1) blendRaw(q, hubC.hc[i], a1 == 255 ? 256 : a1);
      if (a2) blendRaw(q, hubC.rimC, a2 == 255 ? 256 : a2);
    }
  }
}
static void hubDraw(float r) {
  recLayer = 0;
  disc(C, C + 3, r + 3, kBlack, 0.5f);                          // shadow
  recLayer = 1;
  const Col a = hex(0x2a2c30), b = hex(0x050506);
  int y0 = ifloor(C - r - 1), y1 = ifloor(C + r + 1);
  for (int y = y0; y <= y1; y++)
    for (int x = y0; x <= y1; x++) {
      float dx = x + 0.5f - C, dy = y + 0.5f - C, d = sqrt_(dx * dx + dy * dy);
      float cov = clampf(r - d + 0.5f, 0, 1);
      if (cov <= 0) continue;
      float gx = dx + 6, gy = dy + 7, t = clampf((sqrt_(gx * gx + gy * gy) - 2) / (r - 2), 0, 1);
      blend(x, y, mix(a, b, t), cov);
    }
  recLayer = 2;
  arc(r - 2, r, 0, 360, dark ? hex(0x5a5f66) : hex(0x9ea3aa));   // silver rim
}

// ---------------------------------------------------------------- shared bits
// Two linear segments, so 0 psi lands exactly on zeroDeg (the horizontal axis)
static inline float psiDeg(const Cfg &k, float p) {
  if (p < 0) return k.startDeg + clampf((p - k.minPsi) / (0 - k.minPsi), 0, 1) * (k.zeroDeg - k.startDeg);
  return k.zeroDeg + clampf(p / k.maxPsi, 0, 1) * (k.endDeg - k.zeroDeg);
}
// "NOT WARM 142 F" in blue while the oil warms up

// the status line; the warm-up cue takes its place when there's nothing else to say
static void statusAt(const Data &d, float x, float y, float spacing = 1.5f) {
  if (d.status && *d.status) text(C16, d.status, x, y, d.statusWarn ? kAmber : kGrey, 0, spacing * 0.7f);
}
static void status(const Data &d, float y) { statusAt(d, C, y); }
// Boost readout text: psi on boost, inHg with a minus sign on vacuum (the dial's units)
static char *boostStr(char *s, float psi) {
  char v[12];
  bool vac = psi < 0;
  fmt(v, vac ? -psi * PSI_TO_INHG : psi, 1);
  return cat(s, vac ? "-" : "", v);
}

// Gear chip: a small outlined box with the gear number ("-" while shifting / not matched)
static void gearChip(const Data &d, float x, float y, bool caption, bool tabUp = false, bool big = false) {
  if (d.gear < 0) return;
  const float w = big ? 42 : 30, h = big ? 52 : 38, lw = big ? 2 : 1.5f;
  Col frame = hex(big ? 0x6a7079 : 0x5d626a);
  rect(x - w / 2, y - h / 2, w, lw, frame); rect(x - w / 2, y + h / 2 - lw, w, lw, frame);
  rect(x - w / 2, y - h / 2, lw, h, frame); rect(x + w / 2 - lw, y - h / 2, lw, h, frame);
  const GFont &F = big ? R40 : R28;
  char b[4] = {0, 0, 0, 0};
  if (d.gear >= 1 && d.gear <= 9) { b[0] = (char)('0' + d.gear); text(F, b, x, y, kWhite); }
  else text(F, "-", x, y, kDim);
  if (caption) label(C12, "GEAR", x, y + h / 2 + 12, kDim);
  // S (sport program, also Dynamic) or M (manual): a filled tab on the right, or on top
  if (d.trans == 'S' || d.trans == 'M') {
    char t[2] = {d.trans, 0};
    const GFont &T = big ? C24 : C20;
    const float tt = big ? 30 : 22;
    if (tabUp) { rect(x - w / 2, y - h / 2 - tt + lw, w, tt, frame); text(T, t, x, y - h / 2 - tt / 2 + lw, kWhite); }
    else { rect(x + w / 2 - lw, y - h / 2, tt, h, frame); text(T, t, x + w / 2 - lw + tt / 2, y, kWhite); }
  }
}

static void overlays(const Data &d, int page) {
  if (d.dotsA > 0)
    for (int i = 0; i < P_COUNT; i++) {
      const float step = P_COUNT > 8 ? 13.0f : 18.0f;
      float x = C + (2 * i - (P_COUNT - 1)) * step / 2, y = C + 212;
      if (i == page) disc(x, y, 3.5f, kWhite, d.dotsA);
      else arc(2.0f, 3.2f, 0, 360, kDim, d.dotsA, x, y);
    }
  if (d.toast && *d.toast && d.toastA > 0) {
    // a dark pill across the middle; a short word (DYNAMIC, RE-ZERO) in the big caps, a longer line smaller
    bool big = textWidth(C44, d.toast, 1.0f) <= 300;
    float w = big ? 340 : 380, h = big ? 84 : 64, a = d.toastA;
    rect(C - w / 2 + h / 2, C - h / 2, w - h, h, kBlack, 0.96f * a);
    disc(C - w / 2 + h / 2, C, h / 2, kBlack, 0.96f * a); disc(C + w / 2 - h / 2, C, h / 2, kBlack, 0.96f * a);
    arc(h / 2 - 2, h / 2, 90, 270, kSilver, 0.8f * a, C - w / 2 + h / 2, C);
    arc(h / 2 - 2, h / 2, -90, 90, kSilver, 0.8f * a, C + w / 2 - h / 2, C);
    rect(C - w / 2 + h / 2, C - h / 2, w - h, 2, kSilver, 0.8f * a); rect(C - w / 2 + h / 2, C + h / 2 - 2, w - h, 2, kSilver, 0.8f * a);
    text(big ? C44 : C24, d.toast, C, C, kWhite, 0, 1.0f, a);
  }
  if (!d.alertText || !*d.alertText) return;
  Col c = d.alertRed ? kRed : kAmber, ink = d.alertRed ? kWhite : kBlack;
  if (d.alertBig > 0.01f && d.alertTitle && *d.alertTitle) {
    float a = d.alertBig, ph = d.alertPulse;
    disc(C, C, 233, kBlack, 0.78f * a);
    arc(178 + 40 * ph, 184 + 40 * ph, 0, 360, c, a * 0.7f * (1 - ph));
    disc(C, C, 178, c, a);
    bool hasVal = d.alertValue && *d.alertValue;
    text(C44, d.alertTitle, C, hasVal ? C - 92 : C - 22, ink, 0, 1.0f, a);
    if (hasVal) {
      float wv = textWidth(L84, d.alertValue), wu = (d.alertUnit && *d.alertUnit) ? textWidth(C28, d.alertUnit, 1.0f) + 8 : 0;
      float x0 = C - (wv + wu) / 2;
      text(L84, d.alertValue, x0 + wv / 2, C, ink, 0, 0, a);
      if (wu > 0) text(C28, d.alertUnit, x0 + wv + 8, C - 10, ink, -1, 1.0f, a);
    }
    if (d.alertNote && *d.alertNote) text(C24, d.alertNote, C, hasVal ? C + 80 : C + 32, ink, 0, 1.2f, a);
    return;
  }
  // the banner: a solid pill across the top, on every page while the warning lasts
  const float y0 = 24, h = 60, w = 320;
  rect(C - w / 2 + h / 2, y0, w - h, h, c);
  disc(C - w / 2 + h / 2, y0 + h / 2, h / 2, c); disc(C + w / 2 - h / 2, y0 + h / 2, h / 2, c);
  text(textWidth(C28, d.alertText, 1.0f) <= w - 40 ? C28 : C24, d.alertText, C, y0 + h / 2, ink, 0, 1.0f);
}

// ================================================================ BOOST
// The bottom-right quadrant (3 to 6 o'clock) has no scale: badge and readout live there.
static const float BQ_X = C + 92;
static void boostStatic(const Cfg &k) {
  face();
  float dRed = psiDeg(k, k.redPsi), dEnd = psiDeg(k, k.maxPsi);
  if (k.redPsi < k.maxPsi) arc(200, 212, dRed, dEnd, kRed);                     // red zone blocks
  for (int i = 30; i > 0; i--) {                                              // vacuum, inHg
    if ((i % 5) && (i % 2)) continue;
    bool major = i % 10 == 0;
    tick(psiDeg(k, -i / PSI_TO_INHG), major ? 190 : (i % 5 == 0 ? 200 : 206), 214, major ? 3.5f : 1.6f, kSilver);
  }
  for (int p = 0; p <= (int)k.maxPsi; p++) {                                  // boost, psi
    bool major = p % 5 == 0;
    tick(psiDeg(k, (float)p), major ? 190 : 205, 214, major ? 3.5f : 1.6f, kWhite);
  }
  char b[8];
  for (int i = 30; i >= 10; i -= 10) { Pt q = polar(psiDeg(k, -i / PSI_TO_INHG), 165); text(R28, fmt(b, (float)i, 0), q.x, q.y, kSilver); }
  for (int p = 0; p <= (int)k.maxPsi; p += 5) { Pt q = polar(psiDeg(k, (float)p), 163); text(R32, fmt(b, (float)p, 0), q.x, q.y, kWhite); }
  label(C16, "BOOST", C, C - 70, kLabel);
  { Pt q = polar(k.endDeg, 163);   label(C12, "PSI", q.x - 4, q.y + 26, kUnit); }     // under the 30, like the AWE face
  image(BADGE_FACE, BQ_X, C + 26);                                                  // S badge in the free quadrant
}

static void boostDynamic(const Cfg &k, const Data &d) {
  char s[16];
  // one big number (the red telltale needle already shows the peak); under it the gear chip,
  // or the status line while there is one
  if (k.digital) text(L56, boostStr(s, d.boostText), BQ_X, C + 80, d.boostText >= k.redPsi ? kRed : kWhite);
  if (d.status && *d.status) text(C12, d.status, BQ_X, C + 132, d.statusWarn ? kAmber : kDim, 0, 1.0f);
  else gearChip(d, BQ_X, C + 134, false);
  // the ECU's requested boost: an arrow outside the ticks, and the shortfall as a thin arc
  if (has(d.tgtPsi)) {
    float ta = psiDeg(k, d.tgtPsi);
    if (d.tgtPsi > d.boost + 0.3f) arc(216, 219, psiDeg(k, d.boost), ta, kTarget, 0.55f);
    Pt p[3] = {polar(ta, 212), polar(ta - 2.6f, 228), polar(ta + 2.6f, 228)};
    poly(p, 3, kTarget);
  }
  float pn = has(d.peakNeedle) ? d.peakNeedle : d.peak;
  if (k.telltale && pn > 0.2f && pn > d.boost + 0.1f) needle(psiDeg(k, pn), 200, 0, 1.6f, kRed, true);
  needle(psiDeg(k, d.boost), 206, 36, 5.2f, kWhite, false);
  hub(26);
}

// ================================================================ TEMPS
// Bracket arcs on the left (intake after the intercooler) and right (coolant)
static const float T_R = 206, T_W = 14;
static void sideArcStatic(bool left, float warnFrac) {
  float aBot = left ? 140.0f : 40.0f, dir = left ? 1.0f : -1.0f, span = 80;
  float a0 = left ? 140 : -40, a1 = left ? 220 : 40;
  arc(T_R - T_W / 2, T_R + T_W / 2, a0, a1, kFaint);
  for (int i = 0; i <= 4; i++) tick(aBot + dir * span * i / 4, T_R + 9, T_R + 15, 2, kDim);
  tick(aBot + dir * span * warnFrac, T_R - 9, T_R + 15, 2.5f, kRed);
}
static void sideArcFill(bool left, float frac, Col c) {
  frac = clampf(frac, 0, 1);
  if (frac <= 0) return;
  if (left) arc(T_R - T_W / 2, T_R + T_W / 2, 140, 140 + 80 * frac, c);
  else      arc(T_R - T_W / 2, T_R + T_W / 2, 40 - 80 * frac, 40, c);
}
static const float IAT_LO = 40, IAT_HI = 200, IAT_WARN = 140;   // degF
static const float CLT_LO = 100, CLT_HI = 260, CLT_WARN = 230;
// drivetrain. Clutch oil from log_0038 (82 min, 11 pulls, 2 launches): about 200 degF cruising, never
// above 226 in hard driving; a launch adds 30-38 (201 -> 239, 216 -> 246) and it's back near 220 in about
// 45 s. Amber: hotter than one launch leaves it (launches stacked up); red: about three in a row.
// Gearbox and diff are still first guesses.
// from logs with launches. Amber / red.
static const float CLUTCH_WARN = 250, CLUTCH_RED = 275, GBX_WARN = 230, GBX_RED = 255, DIFF_WARN = 250, DIFF_RED = 275;
static const float DT_X = 82;                                    // drivetrain row: column spacing

static const float TX = 90;                           // value columns, either side of the middle
static void tempsStatic(const Cfg &) {
  face();
  sideArcStatic(true, (IAT_WARN - IAT_LO) / (IAT_HI - IAT_LO));
  sideArcStatic(false, (CLT_WARN - CLT_LO) / (CLT_HI - CLT_LO));
  label(C24, "INTAKE \302\260F", C - 84, C - 128, kLabel);
  label(C24, "COOLANT \302\260F", C + 84, C - 128, kLabel);
  rect(C - 1, C - 150, 2, 166, kFaint);
  rect(C - 172, C + 30, 344, 1.5f, kFaint);
  rect(C - 1, C + 40, 2, 64, kFaint);
  label(C20, "CLUTCH \302\260F", C - DT_X, C + 48, kLabel);
  label(C20, "DIFF \302\260F", C + DT_X, C + 48, kLabel);
  label(C20, "GEARBOX \302\260F", C, C + 128, kLabel);
}
static void tempsDynamic(const Cfg &, const Data &d) {
  char v[16];
  // each one on its own: a reading that has stopped arriving shows "--", not its last value
  if (d.obd && has(d.iatF)) {
    Col ic = d.iatF >= IAT_WARN ? kAmber : kWhite;
    sideArcFill(true, (d.iatF - IAT_LO) / (IAT_HI - IAT_LO), ic);
    text(L84, fmt(v, d.iatF, 0), C - TX, C - 62, ic);
  } else text(L84, "--", C - TX, C - 62, kDim);
  if (d.obd && has(d.cltF)) {
    Col cc = d.cltF >= CLT_WARN ? kRed : kWhite;
    sideArcFill(false, (d.cltF - CLT_LO) / (CLT_HI - CLT_LO), cc);
    text(L84, fmt(v, d.cltF, 0), C + TX, C - 62, cc);
  } else text(L84, "--", C + TX, C - 62, kDim);
  const float tv[3] = {d.clutchF, d.gbxF, d.diffF}, warn[3] = {CLUTCH_WARN, GBX_WARN, DIFF_WARN}, red[3] = {CLUTCH_RED, GBX_RED, DIFF_RED};
  for (int i = 0; i < 3; i++) {                      // clutch (left), gearbox (below), diff (right)
    float x = i == 1 ? C : C + (i - 1) * DT_X, y = i == 1 ? C + 164 : C + 84;
    const GFont &F = i == 1 ? L50 : L56;
    if (d.obd && has(tv[i])) text(F, fmt(v, tv[i], 0), x, y, tv[i] >= red[i] ? kRed : tv[i] >= warn[i] ? kAmber : kWhite);
    else text(F, "--", x, y, kDim);
  }
  status(d, C + 204);
}

// ================================================================ ENGINE
static const float E_A0 = 135, E_A1 = 405, E_MAX = 8000;
static const int   SEGS = 24;
static inline float rpmDeg(float r) { return E_A0 + clampf(r / E_MAX, 0, 1) * (E_A1 - E_A0); }
static inline float segA(int i) { return -160.0f + i * (140.0f / SEGS); }

static void engineStatic(const Cfg &k) {
  face();
  for (int i = 0; i < SEGS; i++) arc(205, 215, segA(i), segA(i) + 140.0f / SEGS - 1.6f, kFaint);
  arc(180, 192, rpmDeg(k.redlineRpm), E_A1, kRed);
  for (int r = 0; r <= (int)E_MAX; r += 500) {
    bool major = r % 1000 == 0;
    tick(rpmDeg((float)r), major ? 176 : 186, 194, major ? 3.0f : 1.4f, r >= k.redlineRpm ? kRed : kWhite);
  }
  char b[4];
  for (int r = 0; r <= (int)(E_MAX / 1000); r++) { Pt q = polar(rpmDeg(r * 1000.0f), 154); text(R24, fmt(b, (float)r, 0), q.x, q.y, r * 1000 >= k.redlineRpm ? kRed : kWhite); }
  label(C13, "\xc3\x97 1000 RPM", C, C - 98, kDim);
  image(BADGE_FACE, C, C - 62);
  rect(C - 70, C + 150, 140, 8, kFaint);
}
static void engineDynamic(const Cfg &k, const Data &d, float nowMs) {
  char v[16];
  float rpm = d.rpm;                    // caller leaves this 0 without OBD (the key-on sweep still moves it)
  bool on = rpm >= k.shiftRpm - 1500, flash = rpm >= k.shiftRpm;
  bool blinkOn = ((int)(nowMs / 90.0f)) % 2 == 0;
  for (int i = 0; i < SEGS; i++) {
    float thresh = k.shiftRpm - 1500 + (i + 1) * (1500.0f / SEGS);
    Col c; bool lit = false;
    if (flash) { lit = blinkOn; c = kRed; }
    else if (on && rpm >= thresh) { lit = true; c = i < SEGS / 2 ? kGreen : (i < SEGS * 4 / 5 ? kAmber : kRed); }
    if (lit) arc(205, 215, segA(i), segA(i) + 140.0f / SEGS - 1.6f, c);
  }
  // big RPM, gear chip beside it, throttle as a bar (no small captions)
  float nx = d.gear >= 0 ? C - 22 : C;                 // make room for the gear chip (clear of the red 8)
  if (d.obd) {
    text(L56, fmt(v, (float)((int)(rpm / 10 + 0.5f) * 10), 0), nx, C + 96, kWhite);
    rect(C - 70, C + 150, 140 * clampf(d.thr / 100, 0, 1), 8, kWhite);
  } else text(L56, "--", nx, C + 96, kDim);
  gearChip(d, C + 64, C + 94, false, true);
  status(d, C + 200);
  // peak telltale: highest RPM since the last reset, under the main needle (same as the Boost page)
  float rn = has(d.rpmNeedle) ? d.rpmNeedle : d.maxRpm;
  if (k.telltale && d.obd && rn > 500 && rn > rpm + 20) needle(rpmDeg(rn), 178, 0, 1.6f, kRed, true);
  needle(rpmDeg(rpm), 184, 30, 5.2f, kWhite, false);
  hub(24);
}

// ================================================================ SYSTEM
// Four items. One is the big readout at the top; tap a small one to swap it in.
enum SysItemId { SI_BATTERY = 0, SI_PEAK, SI_TIMING, SI_MAXIAT, SI_COUNT };
static const uint8_t SYS_DEFAULT_ORDER[SI_COUNT] = {SI_BATTERY, SI_PEAK, SI_TIMING, SI_MAXIAT};

struct SysItem {
  const char *label, *bigUnit, *smallUnit, *loTxt, *hiTxt;
  float v, lo, hi, bandLo, bandHi;   // arc range; band = green zone (none if bandLo == bandHi)
  int   dec;
  bool  live;                        // false = no data yet ("--")
  Col   col;                         // value colour (warning colours included)
};
static SysItem sysItem(int id, const Cfg &k, const Data &d) {
  SysItem it{};
  switch (id) {
    case SI_BATTERY:
      it = {"BATTERY V", "V", "", "11", "15", d.batV, 11, 15, 13.2f, 14.8f, 1, d.obd && has(d.batV), kWhite};
      if (has(d.batV) && (d.batV < 12.0f || d.batV > 14.9f)) it.col = kAmber;
      break;
    case SI_PEAK:
      it = {"PEAK PSI", "PSI", "", "0", "30", maxf(d.peak, 0), 0, k.maxPsi, 0, 0, 1, true, kWhite};
      if (d.peak >= k.redPsi) it.col = kRed;
      break;
    case SI_TIMING:
      it = {"TIMING \xc2\xb0", "\xc2\xb0", "", "-10", "40", d.timing, -10, 40, 0, 0, 1, d.obd && has(d.timing), kWhite};
      break;
    default:
      it = {"MAX INTAKE \302\260F", "\302\260F", "", "40", "200", d.maxIat, 40, 200, 0, 0, 0, d.obd, kWhite};
      if (d.maxIat >= IAT_WARN) it.col = kAmber;
      break;
  }
  return it;
}

static const float S_A0 = 200, S_A1 = 340;
static inline float sysDeg(const SysItem &it, float v) { return S_A0 + clampf((v - it.lo) / (it.hi - it.lo), 0, 1) * (S_A1 - S_A0); }
static const int SYS_SMALL = SI_COUNT - 1;                        // small cells: two side by side, one centred below
static inline float sysCellX(int i) { return i == 2 ? C : C + (i % 2 ? 82 : -82); }
static inline float sysCellY(int i) { return C + 28 + (i / 2) * 92; }   // label row; value is 40 px below

// What is actually shown: the saved order, with a warning item (if any) swapped into the top slot
static void sysDisplayOrder(const uint8_t *saved, int alert, uint8_t *out) {
  for (int i = 0; i < SI_COUNT; i++) out[i] = saved ? saved[i] : SYS_DEFAULT_ORDER[i];
  if (alert >= 0 && out[0] != alert)
    for (int i = 1; i < SI_COUNT; i++) if (out[i] == alert) { out[i] = out[0]; out[0] = (uint8_t)alert; break; }
}

// Touch on the System page: the tapped small item becomes the saved top item.
// Returns true if a small cell was hit. alert = item currently promoted by a warning (or -1).
static bool systemTap(uint8_t *order, float x, float y, int alert = -1) {
  uint8_t shown[SI_COUNT];
  sysDisplayOrder(order, alert, shown);
  for (int i = 0; i < SYS_SMALL; i++) {
    if (fabs_(x - sysCellX(i)) < 70 && y > sysCellY(i) - 20 && y < sysCellY(i) + 50) {
      uint8_t id = shown[i + 1];
      for (int j = 1; j < SI_COUNT; j++) if (order[j] == id) { order[j] = order[0]; order[0] = id; break; }
      return true;
    }
  }
  return false;
}

static void systemStatic(const Cfg &) {
  face();
  arc(211, 217, S_A0, S_A1, kFaint);
  rect(C - 170, C + 4, 340, 1.5f, kFaint);
  rect(C - 1, C + 20, 1.5f, 70, kFaint);
}
// Warnings that pull an item to the top of the System page (display only; the saved order is untouched).
//   live:    battery out of range, intake air at or above IAT_WARN right now
//   records: a new peak boost into the red zone, or a new max RPM past the redline, for 10 s
struct SysAlertState { float lastPeak = 0, lastMaxRpm = 0, peakAt = -1e9f, rpmAt = -1e9f; };
static int sysAlert(SysAlertState &a, const Cfg &k, const Data &d, float nowMs) {
  if (d.peak > a.lastPeak + 0.05f && d.peak >= k.redPsi) a.peakAt = nowMs;
  if (d.maxRpm > a.lastMaxRpm + 5 && d.maxRpm >= k.redlineRpm) a.rpmAt = nowMs;
  a.lastPeak = d.peak; a.lastMaxRpm = d.maxRpm;
  if (d.obd && d.batV > 1 && (d.batV < 12.0f || d.batV > 14.9f)) return SI_BATTERY;
  if (d.obd && d.iatF >= IAT_WARN) return SI_MAXIAT;
  if (nowMs - a.peakAt < 10000) return SI_PEAK;
  return -1;
}

static void systemDynamic(const Cfg &k, const Data &d) {
  uint8_t ord[SI_COUNT];
  sysDisplayOrder(d.sysOrder, d.sysAlert, ord);
  char v[16], s[24];
  // main item: label, big number, arc with optional green band and a marker
  SysItem m = sysItem(ord[0], k, d);
  label(C24, m.label, C, C - 144, d.sysAlert == ord[0] ? m.col : kLabel);
  if (m.bandHi > m.bandLo) arc(211, 217, sysDeg(m, m.bandLo), sysDeg(m, m.bandHi), kCharge);
  { Pt q = polar(S_A0 - 8, 212); label(C16, m.loTxt, q.x, q.y, kGrey); }
  { Pt q = polar(S_A1 + 8, 212); label(C16, m.hiTxt, q.x, q.y, kGrey); }
  if (m.live) {
    fmt(v, m.v, m.dec);
    text(L84, v, C - 8, C - 66, m.col);
    text(C20, m.bigUnit, C - 8 + textWidth(L84, v) / 2 + 6, C - 50, kLabel, -1);
    Pt q = polar(sysDeg(m, m.v), 214);
    disc(q.x, q.y, 7, m.col);
  } else text(L84, "--", C - 8, C - 66, kDim);
  // the other three
  for (int i = 0; i < SYS_SMALL; i++) {
    SysItem it = sysItem(ord[i + 1], k, d);
    label(C20, it.label, sysCellX(i), sysCellY(i), kLabel);
    if (it.live) text(L56, cat(s, fmt(v, it.v, it.dec), it.smallUnit), sysCellX(i), sysCellY(i) + 40, it.col);
    else text(L56, "--", sysCellX(i), sysCellY(i) + 40, kDim);
  }
  status(d, C + 196);
}

// ================================================================ OVERVIEW
// Everything at a glance: boost arc across the top, RPM arc across the bottom,
// six readouts in between. Warning colours match the other pages.
static const float OB_A0 = 205, OB_A1 = 335, OR_A0 = 25, OR_A1 = 155, O_R0 = 198, O_R1 = 210;
static const float OIL_WARN = 260;   // degF
static inline float ovBoostDeg(const Cfg &k, float p) { return OB_A0 + clampf((p - k.minPsi) / (k.maxPsi - k.minPsi), 0, 1) * (OB_A1 - OB_A0); }
static inline float ovRpmDeg(float r) { return OR_A1 - clampf(r / E_MAX, 0, 1) * (OR_A1 - OR_A0); }   // fills left to right
static const float OV_X[2] = {C - 88, C + 88}, OV_LY[2] = {C - 70, C + 14}, OV_GY = C + 2;   // OV_GY: the grid's middle (gear chip)
static const char *const OV_L[4] = {"INTAKE \302\260F", "OIL \302\260F", "COOLANT \302\260F", "CLUTCH \302\260F"};

static void overviewStatic(const Cfg &k) {
  face();
  arc(O_R0 - 3, O_R1 + 2, OB_A0, OB_A1, kFaint);
  if (k.redPsi < k.maxPsi) arc(213, 217, ovBoostDeg(k, k.redPsi), OB_A1, kRed);
  for (int p = -10; p <= 30; p += 10) tick(ovBoostDeg(k, (float)p), 213, 221, p == 0 ? 3.0f : 2.0f, p >= k.redPsi ? kRed : kGrey);
  arc(O_R0 - 3, O_R1 + 2, OR_A0, OR_A1, kFaint);
  arc(213, 217, ovRpmDeg(E_MAX), ovRpmDeg(k.redlineRpm), kRed);
  for (int r = 0; r <= (int)E_MAX; r += 1000) tick(ovRpmDeg((float)r), 213, 221, 2.0f, r >= k.redlineRpm ? kRed : kGrey);
  // the grid's cross, with a gap in the middle for the gear chip (its S / M tab sits on the line above)
  rect(C - 1, C - 82, 2, OV_GY - 27 - (C - 82), kFaint);
  rect(C - 1, OV_GY + 29, 2, C + 86 - (OV_GY + 29), kFaint);
  rect(C - 180, OV_GY, 180 - 25, 1.5f, kFaint);
  rect(C + 25, OV_GY, 180 - 25, 1.5f, kFaint);
  for (int i = 0; i < 4; i++) label(C20, OV_L[i], OV_X[i % 2], OV_LY[i / 2], kLabel);
}
static void overviewDynamic(const Cfg &k, const Data &d) {
  char v[16], s[24];
  float z = ovBoostDeg(k, 0), b = ovBoostDeg(k, d.boost);
  if (b > z) arc(O_R0 - 3, O_R1 + 2, z, b, d.boost >= k.redPsi ? kRed : kWhite);
  else if (b < z) arc(O_R0 - 3, O_R1 + 2, b, z, kSilver);
  if (has(d.tgtPsi)) {
    float ta = ovBoostDeg(k, d.tgtPsi);
    Pt p[3] = {polar(ta, 214), polar(ta - 2.8f, 231), polar(ta + 2.8f, 231)};
    poly(p, 3, kTarget);
  }
  { float pn = has(d.peakNeedle) ? d.peakNeedle : d.peak;
    if (k.telltale && pn > 0.2f && pn > d.boost + 0.1f) tick(ovBoostDeg(k, pn), O_R0 - 6, 218, 3.0f, kRed); }
  bool banner = d.alertText && *d.alertText && d.alertBig <= 0.01f;    // the warning banner sits over the top: move down
  text(L70, boostStr(s, d.boostText), C, banner ? C - 108 : C - 132, d.boostText >= k.redPsi ? kRed : kWhite);
  statusAt(d, C, C - 92, 1.0f);
  const float vals[4] = {d.iatF, d.oilF, d.cltF, d.clutchF};
  const float warn[4] = {IAT_WARN, OIL_WARN, CLT_WARN, CLUTCH_WARN};
  for (int i = 0; i < 4; i++) {
    float x = OV_X[i % 2], y = OV_LY[i / 2] + 42;
    if (!d.obd || !has(vals[i])) { text(L56, "--", x, y, kDim); continue; }
    Col c = vals[i] >= warn[i] ? (i == 2 ? kRed : kAmber) : kWhite;
    text(L56, fmt(v, vals[i], 0), x, y, c);
  }
  if (d.rpm > 0) arc(O_R0 - 3, O_R1 + 2, ovRpmDeg(d.rpm), OR_A1, d.rpm >= k.redlineRpm ? kRed : kWhite);
  if (d.obd) text(L70, fmt(v, (float)((int)(d.rpm / 10 + 0.5f) * 10), 0), C, C + 136, d.rpm >= k.shiftRpm ? kRed : kWhite);
  else text(L70, "--", C, C + 136, kDim);
  gearChip(d, C, OV_GY, false, true, true);           // dead centre of the grid, S / M tab on top
}

// ================================================================ G-FORCE
// Friction circle. The dot shows the force you feel: braking pushes it to the top,
// accelerating to the bottom, a left turn to the right. Rings at 0.5 / 1.0 / 1.5 g,
// peaks in each direction as red marks on the axes and numbers at the ends.
static const float GCY = C - 14, GS = 100;            // centre of the circle, pixels per g
static inline Pt gPos(float lat, float lon) {
  float x = -lat * GS, y = lon * GS, r = sqrt_(x * x + y * y), lim = 1.55f * GS;
  if (r > lim) { x *= lim / r; y *= lim / r; }
  return {C + x, GCY + y};
}
static void gforceStatic(const Cfg &) {
  face();
  rect(C - 158, GCY - 0.5f, 316, 1, kFaint);                      // crosshair
  rect(C - 0.5f, GCY - 158, 1, 158 + 100, kFaint);
  for (int i = 1; i <= 3; i++) {
    float r = i * 0.5f * GS;
    arc(r - 0.75f, r + 0.75f, 0, 360, kDim, i == 2 ? 0.6f : 0.3f, C, GCY);
  }
  { Pt q = {C + (GS + 14) * 0.766f, GCY - (GS + 14) * 0.643f}; label(C16, "1G", q.x, q.y, kLabel); }
}
static void gforceDynamic(const Cfg &, const Data &d) {
  char v[12];
  bool ready = d.gCal >= 2;
  // peaks: red marks on the axes, numbers at the ends
  auto mark = [&](float lat, float lon, bool vert) {
    if (lat == 0 && lon == 0) return;
    Pt p = gPos(lat, lon);
    if (vert) rect(p.x - 8, p.y - 1, 16, 2, kRed); else rect(p.x - 1, p.y - 8, 2, 16, kRed);
  };
  if (ready) {                                                     // (no peaks until it knows which way is which)
    mark(0, -d.gPkBrk, true); mark(0, d.gPkAcc, true); mark(-d.gPkL, 0, false); mark(d.gPkR, 0, false);
    text(R40, fmt(v, d.gPkBrk, 2), C, GCY - 178, kWhite);         // braking peak at the top, accelerating at the bottom
    text(R40, fmt(v, d.gPkAcc, 2), C, GCY + 192, kWhite);
    text(R40, fmt(v, d.gPkR, 2), C - 180, GCY, kWhite);            // right turns push the dot left
    text(R40, fmt(v, d.gPkL, 2), C + 180, GCY, kWhite);
  }
  if (!ready) {
    text(L70, "--", C - 6, GCY + 116, kDim);
    label(C24, "CALIBRATING", C, GCY - 26, kAmber);
    label(C16, d.gCal == 0 ? "STOP ON LEVEL GROUND" : "DRIVE STRAIGHT, ACCELERATE", C, GCY + 4, kLabel);
    status(d, C + 200);
    return;
  }
  float comb = sqrt_(d.gLat * d.gLat + d.gLon * d.gLon);
  fmt(v, comb, 2);
  text(L70, v, C - 10, GCY + 116, comb >= 1.0f ? kRed : kWhite);
  text(C20, "G", C - 10 + textWidth(L70, v) / 2 + 6, GCY + 130, kLabel, -1);
  // trail (oldest faint) then the dot
  for (int i = 0; i < d.gTrailN; i++) {
    Pt p = gPos(d.gTrail[2 * i], d.gTrail[2 * i + 1]);
    float a = 0.08f + 0.42f * (i + 1) / (float)d.gTrailN;
    disc(p.x, p.y, 3.2f, kRed, a);
  }
  Pt p = gPos(d.gLat, d.gLon);
  disc(p.x + 2, p.y + 3, 10, kBlack, 0.5f);
  disc(p.x, p.y, 8.5f, kWhite);
  disc(p.x, p.y, 3.0f, kRed);
  status(d, C + 200);
}

// ================================================================ PERF
// 0-60 big at the top; 60-130 and 1/4 mile below. Big numbers, few words.
static void perfStatic(const Cfg &) {
  face();
  label(C24, "0-60 MPH", C, C - 176, kLabel);
  rect(C - 186, C + 4, 372, 1, kFaint);
  rect(C - 1, C + 20, 1, 128, kFaint);
  label(C24, "60-130", C - 92, C + 40, kLabel);
  label(C24, "1/4 MILE", C + 92, C + 40, kLabel);
}
static void halo(Col c);
// Launch readiness, at a stop with the timer armed: the first thing in the way, or READY.
// level 1 = ready (green), 2 = wait (amber); 0 = nothing to say (not stopped, or running).
// From logs 33-38: the clutch oil (7E1 1259) peaks at 216-226 degF in hard driving and a launch adds
// 30-38, cooling about 0.6 degF a second; the intake sits at 113-126 degF at a typical stop.
// Order: oil, clutch, gearbox program, parking brake. Heat soak is a note under READY, not a wait.
static const float LR_CLUTCH = 230, LR_COOL_FPS = 0.6f, LR_OIL = 170, LR_SOAK = 135;   // degF, degF a second
static int launchReady(const Data &d, char *msg, char *sub) {
  char v[12];
  msg[0] = sub[0] = 0;
  if (d.runState != 1) return 0;
  if (d.staged) { cat(msg, "STAGED"); if (d.rpm > 0) cat(sub, fmt(v, (float)((int)(d.rpm / 50 + 0.5f) * 50), 0), " RPM"); return 1; }
  if (has(d.oilF) && d.oilF < LR_OIL) { cat(msg, "WAIT"); cat(sub, "OIL COLD  ", fmt(v, d.oilF, 0), "\302\260F"); return 2; }
  if (has(d.clutchF) && d.clutchF > LR_CLUTCH) {
    int secs = (int)((d.clutchF - LR_CLUTCH) / LR_COOL_FPS / 5 + 0.99f) * 5;      // to the next 5 s
    char a[24], n[8]; cat(msg, "WAIT");
    cat(a, "CLUTCH ", fmt(v, d.clutchF, 0), "\302\260F  ~"); cat(sub, a, fmt(n, (float)secs, 0), " S"); return 2;
  }
  if (d.trans == 'D') { cat(msg, "WAIT"); cat(sub, "SELECT S OR M"); return 2; }
  if (d.epb == 1) { cat(msg, "WAIT"); cat(sub, "PARKING BRAKE ON"); return 2; }
  cat(msg, "READY");
  if (has(d.iatF) && d.iatF >= LR_SOAK) cat(sub, "HEAT SOAK  ", fmt(v, d.iatF, 0), "\302\260F");
  else if (d.b060 > 0) cat(sub, "BEST ", fmt(v, d.b060, 2));
  return 1;
}
// A clock that's running is amber; the last result is white; best since power-on is under it.
static void perfDynamic(const Cfg &, const Data &d) {
  char v[16], s[32];
  if (d.runT060 > 0) text(L84, fmt(v, d.runT060, 2), C, C - 112, kAmber);
  else if (d.t060 > 0) text(L84, fmt(v, d.t060, 2), C, C - 112, kWhite);
  else text(L84, "--.--", C, C - 112, kDim);
  const float my = C - 50, sy2 = C - 16;
  char lm[16], ls[32];
  int lr = launchReady(d, lm, ls);
  if (lr) {                                         // at a stop: READY / WAIT big, why under it, and a ring
    Col c = lr == 1 ? kGreen : kAmber;
    label(C44, lm, C, my, c);
    if (ls[0]) label(C24, ls, C, sy2, lr == 2 ? kWhite : (ls[0] == 'H' ? kAmber : kLabel));
    halo(c);
  }
  else if (d.runState == 2) label(C28, cat(s, fmt(v, d.runMph, 0), " MPH"), C, my + 6, kAmber);
  else if (d.runMsg[0] && d.runMsg[0] == 'N') label(C20, d.runMsg, C, my + 6, kGrey);   // no speed: say so
  else if (d.b060 > 0) label(C24, cat(s, "BEST ", fmt(v, d.b060, 2)), C, my + 6, kLabel);
  else label(C24, d.runMsg[0] ? d.runMsg : "STOP TO ARM", C, my + 6, kGrey);
  const float cx = 92, vy = C + 90, sy = C + 134;
  if (d.runT6130 > 0) text(L56, fmt(v, d.runT6130, 2), C - cx, vy, kAmber);
  else if (d.t60130 > 0) text(L56, fmt(v, d.t60130, 2), C - cx, vy, kWhite);
  else text(L56, "--", C - cx, vy, kDim);
  if (d.b60130 > 0 && d.runT6130 <= 0) label(C20, cat(s, "BEST ", fmt(v, d.b60130, 2)), C - cx, sy, kLabel);
  if (d.runTQm > 0) text(L56, fmt(v, d.runTQm, 2), C + cx, vy, kAmber);
  else if (d.tQm > 0) {
    text(L56, fmt(v, d.tQm, 2), C + cx, vy, kWhite);
    label(C20, cat(s, "@ ", fmt(v, d.qmMph, 0), " MPH"), C + cx, sy, kLabel);
  } else text(L56, "--", C + cx, vy, kDim);
  if (d.runObd) label(C16, "OBD SPEED ONLY", C, C + 172, kGrey);
  status(d, C + 200);
}

// ================================================================ GEARS
// Peak boost reached in each gear at full throttle, as seven rings over the top half (1st
// inside, 7th outside). A ring closes at the red zone (cfg redPsi), so how far round it goes
// reads at a glance; one that reaches the red zone is red. The current gear's ring is white,
// and its peak is the big number below.
static const float GR_R0 = 84, GR_STEP = 17, GR_W = 11, GR_A0 = 180, GR_SW = 180;
static inline float grR(int i) { return GR_R0 + i * GR_STEP; }
static const char *const GR_ORD[8] = {"", "1ST", "2ND", "3RD", "4TH", "5TH", "6TH", "7TH"};
static void gearsStatic(const Cfg &k) {
  face();
  for (int i = 0; i < 7; i++) arc(grR(i) - GR_W / 2, grR(i) + GR_W / 2, GR_A0, GR_A0 + GR_SW, kFaint);
  // scale: five ticks with their psi, outside the rings
  char b[8];
  for (int q = 0; q <= 4; q++) {
    float psi = k.redPsi * q / 4, a = GR_A0 + GR_SW * q / 4;
    Col c = q == 4 ? kRed : kDim;
    tick(a, 196, 205, 2, c);
    Pt t = polar(q == 0 ? 186 : (q == 4 ? 354 : a), 214);
    text(R24, fmt(b, psi, 0), t.x, t.y, q == 4 ? kRed : kLabel);
  }
  { Pt t = polar(338, 196); label(C16, "PSI", t.x - 6, t.y + 30, kLabel); }
  rect(C - 200, C + 3, 400, 1, kFaint);
}
static void gearsDynamic(const Cfg &k, const Data &d) {
  char v[16], s[32];
  int best = 0;
  for (int i = 0; i < 7; i++) {
    float pk = d.gearPeak ? d.gearPeak[i] : 0;
    bool cur = d.gear == i + 1;
    if (pk > 0.05f) {
      if (!best || pk > d.gearPeak[best - 1]) best = i + 1;
      float f = clampf(pk / k.redPsi, 0, 1), r = grR(i);
      arc(r - GR_W / 2, r + GR_W / 2, GR_A0, GR_A0 + GR_SW * f, pk >= k.redPsi ? kRed : (cur ? kWhite : hex(0x7d838c)));
    }
    char g[2] = {(char)('1' + i), 0};
    text(cur ? R24 : R14, g, C - grR(i), C + 20, cur ? kWhite : kLabel);
  }
  // below: this gear's peak, then the highest of all
  int g = d.gear >= 1 && d.gear <= 7 ? d.gear : 0;
  float big = g ? (d.gearPeak ? d.gearPeak[g - 1] : 0) : (best ? d.gearPeak[best - 1] : 0);
  label(C24, g ? cat(s, GR_ORD[g], " GEAR") : "PEAK", C, C + 52, kLabel);
  if (big > 0.05f) {
    fmt(v, big, 1);
    text(L84, v, C - 10, C + 110, big >= k.redPsi ? kRed : kWhite);
    text(C20, "PSI", C - 10 + textWidth(L84, v) / 2 + 8, C + 124, kLabel, -1);
  } else text(L84, "--", C, C + 110, kDim);
  if (best) {
    char t[16];
    if (g) cat(s, "PEAK ", fmt(v, d.gearPeak[best - 1], 1), cat(t, " IN ", GR_ORD[best]));
    else cat(s, "IN ", GR_ORD[best], " GEAR");
    label(C20, s, C, C + 168, kGrey);
  }
  status(d, C + 200);
}


// ================================================================ FUEL
// Air-fuel ratio per bank on the side arcs (cruise sits at 14.7, boost runs richer), with
// the fuel delivery numbers and the worst cylinder's knock retard underneath.
static const float AFR_LO = 9.5f, AFR_HI = 16.5f, AFR_LEAN_BOOST = 13.5f;   // lean on boost from here
static const float HPFP_LOW_FRAC = 0.85f;                                    // actual below 85 % of target
static const float IDC_WARN = 90, KNOCK_WARN = -4.5f;      // as watch.h KNOCK_RED
static inline float afrFrac(float a) { return (a - AFR_LO) / (AFR_HI - AFR_LO); }
// bottom: two cells side by side, one centred below (as on Temps)
static const float FX[3] = {C - DT_X, C + DT_X, C}, FLY[3] = {C + 48, C + 48, C + 128}, FVY[3] = {C + 84, C + 84, C + 164};
static inline const GFont &fvFont(int i) { return i == 2 ? L50 : L56; }
static void bottomStatic(const char *a, const char *b, const char *c) {
  rect(C - 172, C + 30, 344, 1.5f, kFaint);
  rect(C - 1, C + 40, 2, 64, kFaint);
  const char *l[3] = {a, b, c};
  for (int i = 0; i < 3; i++) if (l[i]) label(C20, l[i], FX[i], FLY[i], kLabel);
}

static void fuelStatic(const Cfg &) {
  face();
  float aBotL = 140.0f, aBotR = 40.0f, span = 80;
  arc(T_R - T_W / 2, T_R + T_W / 2, 140, 220, kFaint);
  arc(T_R - T_W / 2, T_R + T_W / 2, -40, 40, kFaint);
  for (int i = 0; i <= 4; i++) { tick(aBotL + span * i / 4, T_R + 9, T_R + 15, 2, kDim); tick(aBotR - span * i / 4, T_R + 9, T_R + 15, 2, kDim); }
  float st = afrFrac(14.7f);
  tick(aBotL + span * st, T_R - 9, T_R + 15, 2.5f, kGrey); tick(aBotR - span * st, T_R - 9, T_R + 15, 2.5f, kGrey);
  label(C24, "AFR BANK 1", C - 80, C - 122, kLabel);
  label(C24, "AFR BANK 2", C + 80, C - 122, kLabel);
  rect(C - 1, C - 140, 2, 160, kFaint);
  bottomStatic("INJ %", nullptr, "FUEL PUMP PSI");   // the knock title is drawn live (it names the cylinder)
}
// lean against what the ECU commands (it runs 14.7 on purpose at light boost); 29.4 is a fuel cut
static Col afrCol(const Data &d, float a) {
  if (!d.onBoost || a >= 20) return kWhite;
  float over = has(d.afrCmd) ? a - d.afrCmd : a - AFR_LEAN_BOOST + 1.0f;
  return over >= 1.5f ? kRed : (over >= 1.0f ? kAmber : kWhite);
}
static void fuelDynamic(const Cfg &, const Data &d) {
  char v[16], s[20];
  const float *afr[2] = {&d.afr1, &d.afr2};
  for (int b = 0; b < 2; b++) {
    float a = *afr[b], x = b ? C + 88 : C - 88;
    if (has(a)) {
      Col c = afrCol(d, a);
      sideArcFill(b == 0, afrFrac(a), c);
      text(L70, fmt(v, a, 1), x, C - 60, c);
    } else text(L70, "--", x, C - 60, kDim);
  }
  // injectors (left), knock (right, the cylinder beside it), fuel pump (below, the low side beside it)
  if (has(d.idc)) text(L56, fmt(v, d.idc, 0), FX[0], FVY[0], d.idc >= IDC_WARN ? kAmber : kWhite);
  else text(L56, "--", FX[0], FVY[0], kDim);
  if (has(d.knock)) {
    Col c = d.knock <= KNOCK_WARN ? kRed : (d.knock <= -1.0f ? kAmber : kWhite);
    text(L56, fmt(v, d.knock, 1), FX[1], FVY[1], c);
    if (d.knock <= -1.0f && d.knockCyl) { char t[] = "KNOCK \xc2\xb0 CYL  "; t[sizeof(t) - 2] = (char)('0' + d.knockCyl); label(C20, t, FX[1], FLY[1], c); }
    else label(C20, "KNOCK \xc2\xb0", FX[1], FLY[1], kLabel);
  } else { text(L56, "--", FX[1], FVY[1], kDim); label(C20, "KNOCK \xc2\xb0", FX[1], FLY[1], kLabel); }
  if (has(d.hpfp)) {
    bool low = d.onBoost && has(d.hpfpSet) && d.hpfpSet >= 1200 && d.hpfp < d.hpfpSet * HPFP_LOW_FRAC;
    fmt(v, d.hpfp, 0);
    text(L50, v, FX[2], FVY[2], low ? kRed : kWhite);
    if (has(d.lpfp) && !(d.status && *d.status)) { char t[12]; cat(s, "LOW ", fmt(t, d.lpfp, 0)); label(C20, s, FX[2], FVY[2] + 38, kGrey); }
  } else text(L50, "--", FX[2], FVY[2], kDim);
  status(d, C + 204);
}

// ================================================================ HEAT
// Both catalytic converters on the side arcs (a known weak spot on the 3.0T when they
// overheat), then the air on its way in: outside, before the supercharger, and the
// intercooler pump that cools it.
static const float CAT_LO = 200, CAT_HI = 1900, CAT_WARN_F = 1800, CAT_RED_F = 1850;   // 982 / 1010 degC (hard pulls reach ~1750)
static inline float catFrac(float f) { return (f - CAT_LO) / (CAT_HI - CAT_LO); }

static void heatStatic(const Cfg &) {
  face();
  sideArcStatic(true, catFrac(CAT_WARN_F));
  sideArcStatic(false, catFrac(CAT_WARN_F));
  label(C24, "CAT 1 \302\260F", C - 80, C - 122, kLabel);
  label(C24, "CAT 2 \302\260F", C + 80, C - 122, kLabel);
  rect(C - 1, C - 140, 2, 160, kFaint);
  bottomStatic("OUTSIDE \302\260F", "PRE-SC \302\260F", "IC PUMP %");
}
static void heatDynamic(const Cfg &, const Data &d) {
  char v[16];
  const float *cats[2] = {&d.cat1F, &d.cat2F};
  for (int b = 0; b < 2; b++) {
    float t = *cats[b], x = b ? C + 88 : C - 88;
    if (has(t)) {
      Col c = t >= CAT_RED_F ? kRed : (t >= CAT_WARN_F ? kAmber : kWhite);
      sideArcFill(b == 0, catFrac(t), c);
      text(t >= 999.5f ? L56 : L70, fmt(v, t, 0), b ? x + 4 : x - 4, C - 60, c);
    } else text(L70, "--", x, C - 60, kDim);
  }
  const float vals[3] = {d.ambF, d.preF, d.icPump};      // outside, before the supercharger, intercooler pump
  for (int i = 0; i < 3; i++)
    if (has(vals[i])) text(fvFont(i), fmt(v, vals[i], 0), FX[i], FVY[i], kWhite); else text(fvFont(i), "--", FX[i], FVY[i], kDim);
  status(d, C + 204);
}

// ================================================================ LAST PULL
// The most recent pull at a glance: peak boost, then what else happened during it, and at
// the bottom how the boost compared with this car's usual boost at the same RPM.
static const float PX[2] = {C - 88, C + 88}, PY[3] = {C - 20, C + 66, C + 152};
// 0 = all good, 1 = something to look at (amber), 2 = something wrong (red)
static int pullSeverity(const PullSum &p) {
  int s = 0;
  auto at = [&](bool red, bool amber) { if (red) s = 2; else if (amber && s < 1) s = 1; };
  at(p.slip, has(p.vsNormal) && p.vsNormal <= -1.5f);
  if (has(p.knock))   at(p.knock <= KNOCK_WARN, p.knock <= -1.0f);
  if (has(p.afrOver)) at(p.afrOver >= 1.5f, p.afrOver >= 1.0f);
  else if (has(p.afrMax)) at(p.afrMax >= AFR_LEAN_BOOST + 0.5f, p.afrMax >= AFR_LEAN_BOOST);
  if (has(p.catMax))  at(p.catMax >= CAT_RED_F, p.catMax >= CAT_WARN_F);
  if (has(p.idcMax))  at(false, p.idcMax >= IDC_WARN);
  if (has(p.hpfpDef) && has(p.hpfpMin)) at(false, p.hpfpDef < -0.15f * (p.hpfpMin - p.hpfpDef));
  return s;
}
// a soft glowing ring round the edge of the screen
static void halo(Col c) {
  arc(227, 233, 0, 360, c, 0.95f);
  arc(221, 227, 0, 360, c, 0.45f);
  arc(214, 221, 0, 360, c, 0.18f);
  arc(205, 214, 0, 360, c, 0.06f);
}
static void pullStatic(const Cfg &) {
  face();
  label(C20, "LAST PULL", C, C - 166, kLabel);
  rect(C - 170, C - 44, 340, 1.5f, kFaint);
  rect(C - 1, C - 30, 2, 160, kFaint);
  static const char *const L[4] = {"KNOCK \xc2\xb0", "LEANEST AFR", "INTAKE RISE \302\260F", "FUEL PUMP PSI"};
  for (int i = 0; i < 4; i++) label(C20, L[i], PX[i % 2], PY[i / 2], kLabel);
}
static void pullDynamic(const Cfg &k, const Data &d) {
  char v[16], s[48];
  const PullSum *p = d.pull;
  if (!p || !p->valid) {
    text(L84, "--", C, C - 104, kDim);
    label(C20, "NO PULL YET", C, C - 62, kGrey);
    status(d, C + 196);
    return;
  }
  fmt(v, has(p->peakPsi) ? p->peakPsi : 0, 1);
  text(L84, v, C - 12, C - 106, p->peakPsi >= k.redPsi ? kRed : kWhite);
  text(C20, "PSI", C - 12 + textWidth(L84, v) / 2 + 8, C - 90, kLabel, -1);
  { char r[12], t[12]; cat(s, "GEARS ", p->gears[0] ? p->gears : "-", "  ");
    cat(s, s, fmt(t, p->durS, 1), " S  ");
    cat(s, s, fmt(r, p->peakRpm, 0), " RPM");
    label(C16, s, C, C - 60, kLabel); }
  auto cell = [&](int i, const char *txt, Col c) { text(L50, txt, PX[i % 2], PY[i / 2] + 40, c); };
  if (has(p->knock)) cell(0, fmt(v, p->knock, 1), p->knock <= KNOCK_WARN ? kRed : (p->knock <= -1.0f ? kAmber : kWhite)); else cell(0, "--", kDim);
  if (has(p->afrMax)) {
    Col c = has(p->afrOver) ? (p->afrOver >= 1.5f ? kRed : p->afrOver >= 1.0f ? kAmber : kWhite)
                            : (p->afrMax >= AFR_LEAN_BOOST + 0.5f ? kRed : (p->afrMax >= AFR_LEAN_BOOST ? kAmber : kWhite));
    cell(1, fmt(v, p->afrMax, 1), c);
  } else cell(1, "--", kDim);
  if (has(p->iat0) && has(p->iatMax)) cell(2, fmt(v, p->iatMax - p->iat0, 0), kWhite); else cell(2, "--", kDim);
  if (has(p->hpfpDef)) cell(3, fmt(v, p->hpfpDef, 0), (has(p->hpfpMin) && p->hpfpDef < -0.15f * (p->hpfpMin - p->hpfpDef)) ? kRed : kWhite); else cell(3, "--", kDim);
  int sev = pullSeverity(*p);
  halo(sev == 2 ? kRed : sev == 1 ? kAmber : kGreen);
  char t[12];
  if (p->slip) label(C24, "BELT SLIP?", C, C + 178, kRed);
  else if (has(p->vsNormal) && p->vsNormal <= -1.5f) label(C24, cat(s, "BOOST LOW  ", fmt(t, p->vsNormal, 1)), C, C + 178, kAmber);
  else if (!has(p->vsNormal)) { char n[4] = {(char)('0' + (p->normalPulls > 9 ? 9 : p->normalPulls)), 0}; label(C16, cat(s, "LEARNING ", n, " OF 3"), C, C + 178, kGrey); }
}

// ================================================================ entry points
// Static layer of a page (draw once, cache, copy in every frame)
// Not warm yet: a calm blue ring round the edge, part of the cached background (not on the Last
// Pull page, which has its own halo). Drawn every frame it would touch every row of the screen and
// make each frame restore and compare all of it; in the background it costs nothing per frame.
// The caller sets coldRing and redraws the backgrounds when it changes (twice a drive).
static bool coldRing = false;
static void renderStatic(int page, const Cfg &k) {
  switch (page) {
    case P_BOOST:  boostStatic(k);  break;
    case P_OVERVIEW: overviewStatic(k); break;
    case P_TEMPS:  tempsStatic(k);  break;
    case P_ENGINE: engineStatic(k); break;
    case P_GFORCE: gforceStatic(k); break;
    case P_PERF:   perfStatic(k);   break;
    case P_GEARS:  gearsStatic(k);  break;
    case P_FUEL:   fuelStatic(k);   break;
    case P_HEAT:   heatStatic(k);   break;
    case P_PULL:   pullStatic(k);   break;
    default:       systemStatic(k); break;
  }
  if (coldRing && page != P_PULL && page != P_PERF) { arc(226, 233, 0, 360, kCold, 0.9f); arc(219, 226, 0, 360, kCold, 0.3f); }   // Perf: OIL COLD says it
}
// Moving parts, drawn over a copy of the static layer
static void renderDynamic(int page, const Cfg &k, const Data &d, float nowMs) {
  switch (page) {
    case P_BOOST:  boostDynamic(k, d);          break;
    case P_OVERVIEW: overviewDynamic(k, d);     break;
    case P_TEMPS:  tempsDynamic(k, d);          break;
    case P_ENGINE: engineDynamic(k, d, nowMs);  break;
    case P_GFORCE: gforceDynamic(k, d);         break;
    case P_PERF:   perfDynamic(k, d);           break;
    case P_GEARS:  gearsDynamic(k, d);          break;
    case P_FUEL:   fuelDynamic(k, d);           break;
    case P_HEAT:   heatDynamic(k, d);           break;
    case P_PULL:   pullDynamic(k, d);           break;
    default:       systemDynamic(k, d);         break;
  }
  overlays(d, page);
}

// ---------------------------------------------------------------- boot splash
// S badge on black. fade 0..1 (fade-in / fade-out), line = small message under it
static void renderSplash(float fade, const char *line = "", bool warn = false, const char *line2 = "", const char *ver = "") {
  for (int i = 0; i < W * H; i++) fb[i] = 0;
  image(BADGE_SPLASH, C, C - 10, fade);
  if (line && *line) text(C12, line, C, C + 120, warn ? kAmber : kDim, 0, 1.5f, fade);
  if (line2 && *line2) text(C12, line2, C, C + 146, kAmber, 0, 1.5f, fade);
  if (ver && *ver) text(C12, ver, C, C + 176, kDim, 0, 1.5f, fade);   // firmware version, low and quiet
}

// ---------------------------------------------------------------- key-on sweep: 0..1..0 over 1.4 s
static float sweep(float s) {
  float f = s < 0.7f ? s / 0.7f : (s < 1.4f ? 1 - (s - 0.7f) / 0.7f : 0);
  return sin_(f * PI_F / 2);
}

// ---------------------------------------------------------------- demo drive cycle (18 s loop)
// g-forces for the demo cycle: launch, shift dip, braking, an S-bend while cruising
static void demoG(float t, float &lat, float &lon) {
  auto rise = [](float x, float a, float b) { return clampf((x - a) / (b - a), 0, 1); };
  lat = 0; lon = 0;
  if (t < 3)            { lon = 0; }
  else if (t < 7.2f)    { lon = 0.48f * rise(t, 3, 3.3f) - 0.12f * rise(t, 3.3f, 7.2f); }
  else if (t < 7.6f)    { lon = 0.05f; }
  else if (t < 10.6f)   { lon = 0.34f - 0.08f * rise(t, 7.6f, 10.6f); }
  else if (t < 13.5f)   { float k = rise(t, 10.6f, 13.5f); lon = -0.9f * sin_(k * PI_F); lat = 0.15f * sin_(k * 2 * PI_F); }
  else if (t < 16.5f)   { float k = rise(t, 13.5f, 16.5f); lat = 0.82f * sin_(k * 2 * PI_F); lon = -0.08f * sin_(k * 4 * PI_F); }
  else                  { lon = -0.25f * rise(t, 16.5f, 17.0f) * (1 - rise(t, 17.2f, 18.0f)); }
  lat += 0.015f * sin_(t * 53.0f); lon += 0.012f * sin_(t * 71.0f);   // road texture
}
// Bench demo on the car and the preview's "Drive cycle": idle, two pulls, lift-off, cruise.
static void demo(float tSec, Data &d) {
  float t = tSec - 18.0f * (float)ifloor(tSec / 18.0f);
  auto rise = [](float x, float a, float b) { return clampf((x - a) / (b - a), 0, 1); };
  float rpm, thr, boost, iat;
  if (t < 3)         { rpm = 750 + sin_(t * 7) * 15; thr = 2; boost = -8.7f + sin_(t * 5) * 0.15f; iat = 98; }
  else if (t < 7.2f) { float k = rise(t, 3, 7.2f); rpm = lerp(2600, 7300, k); thr = 100;
                       boost = lerp(-8.7f, 6 + 6 * k, rise(t, 3, 3.5f)) - 4 * maxf(0, k - 0.85f); iat = 98 + 16 * k; }
  else if (t < 7.6f) { rpm = lerp(7300, 5200, rise(t, 7.2f, 7.6f)); thr = 5; boost = lerp(11.5f, -5, rise(t, 7.2f, 7.4f)); iat = 114; }
  else if (t < 10.6f){ float k = rise(t, 7.6f, 10.6f); rpm = lerp(5200, 6900, k); thr = 100;
                       boost = lerp(-5, 11 + 1.2f * k, rise(t, 7.6f, 8.0f)); iat = 114 + 20 * k; }
  else if (t < 13.5f){ float k = rise(t, 10.6f, 13.5f); rpm = lerp(6900, 1600, k); thr = 0; boost = lerp(12, -11.6f, rise(t, 10.6f, 10.9f)); iat = 134 - 12 * k; }
  else if (t < 16.5f){ rpm = 2000 + sin_(t) * 40; thr = 16; boost = -3.2f + sin_(t * 2) * 0.3f; iat = 122 - 6 * rise(t, 13.5f, 16.5f); }
  else               { rpm = lerp(2000, 750, rise(t, 16.5f, 17.5f)); thr = 2; boost = lerp(-3.2f, -8.7f, rise(t, 16.5f, 17)); iat = 116 - 18 * rise(t, 16.5f, 18); }
  d.boost = boost + 0.04f * sin_(tSec * 97.0f) + 0.03f * sin_(tSec * 61.0f);
  d.rpm = rpm; d.thr = thr; d.iatF = iat;
  d.cltF = 203 + sin_(tSec / 4.0f) * 2;
  d.oilF = 214 + sin_(tSec / 9.0f) * 5;
  d.batV = 14.2f + sin_(tSec / 0.7f) * 0.05f;
  d.timing = boost > 2 ? 24 - boost * 0.6f : (thr < 5 ? 14 : 30);
  d.obd = true;
  // gear, g-forces, timer (same cycle): stop, pull in 2nd, shift, pull in 3rd, braking, cruise in 5th
  d.gear = t < 3 ? 1 : (t < 7.25f ? 2 : (t < 10.6f ? 3 : (t < 13.5f ? (rpm > 3200 ? 3 : 2) : (t < 16.5f ? 5 : 1))));
  if (t > 7.2f && t < 7.35f) d.gear = 0;                           // mid-shift
  d.trans = 'S'; d.driveMode = 3;                                  // Dynamic (gearbox in S)
  demoG(t, d.gLat, d.gLon);
  static float trail[2 * 40];
  for (int i = 0; i < 40; i++) {                                   // last 1.3 s at 30 Hz
    float tt = t - (40 - i) * 0.033f;
    if (tt < 0) tt += 18.0f;
    demoG(tt, trail[2 * i], trail[2 * i + 1]);
  }
  d.gTrail = trail; d.gTrailN = 40;
  d.gPkL = 0.86f; d.gPkR = 0.91f; d.gPkAcc = 0.52f; d.gPkBrk = 0.97f; d.gCal = 2;
  static const float pk[7] = {8.4f, 11.9f, 12.3f, 12.6f, 12.1f, 0, 0};
  d.gearPeak = pk;
  d.b060 = 3.84f; d.b60130 = 9.71f; d.t60130 = 9.88f; d.tQm = 12.41f; d.qmMph = 112; d.bQm = 12.41f;
  // fuel / heat: richer on boost, the pump working harder, cats warming through the pulls
  bool onB = boost > 2;
  d.onBoost = onB;
  d.tgtPsi = boost > 1 ? minf(boost + 3.4f, 17.8f) : NOV;   // the tune asks for more than the pulley gives
  d.afr1 = onB ? 11.6f + 0.2f * sin_(tSec * 3) : 14.7f + 0.15f * sin_(tSec * 5);
  d.afr2 = d.afr1 + 0.1f;
  d.afrCmd = onB ? 11.5f : 14.7f;
  d.hpfpSet = onB ? 2175 : 725; d.hpfp = d.hpfpSet - (onB ? 60 : 10) + 15 * sin_(tSec * 4);
  d.lpfp = onB ? 78 : 72;
  d.idc = onB ? 38 + 4 * boost : 4 + rpm / 1000;
  d.knock = (t > 9.5f && t < 10.6f) ? -2.3f : 0; d.knockCyl = 4;
  d.cat1F = 1180 + 220 * clampf((t - 3) / 8, 0, 1); d.cat2F = d.cat1F - 25;
  d.ambF = 71; d.preF = 92 + 6 * clampf((t - 3) / 8, 0, 1); d.icPump = onB ? 100 : 45;
  d.clutchF = 192 + 9 * clampf((t - 3) / 8, 0, 1); d.gbxF = 174; d.diffF = 118;
  static PullSum ps;
  ps.valid = true; ps.durS = 4.2f; ps.kmh0 = 40; ps.kmh1 = 118; cat(ps.gears, "2-3");
  ps.peakPsi = 12.3f; ps.peakRpm = 7210; ps.obdPeak = 12.1f; ps.tgtPeak = 15.8f; ps.iat0 = 98; ps.iatMax = 114;
  ps.minTiming = 14.5f; ps.knock = -2.3f; ps.knockCyl = 4; ps.hpfpDef = -72; ps.hpfpMin = 2103; ps.lpfpMin = 74;
  ps.afrMax = 11.9f; ps.idcMax = 86; ps.catMax = 1402; ps.vsNormal = 0.3f; ps.normalPulls = 5;
  d.pull = &ps;
  if (t < 3)            { d.runState = 1; d.t060 = 3.91f; }
  else if (t < 7.1f)    { d.runState = 2; d.runT060 = t < 6.92f ? t - 3 : 0; d.runTQm = t - 3; d.runMph = 60.0f * clampf((t - 3) / 3.9f, 0, 1.08f); }
  else                  { d.runState = 0; d.t060 = 3.92f; }
}

}  // namespace gr
