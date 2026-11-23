// =====================================================================
//  S5 Gauge Pod - which gear (calculated from RPM and road speed)
//
//  Nobody has published a gear identifier for the S tronic (0B5) gearbox, and the standard
//  gear PID (01A4) isn't on this car, so the gear is worked out: rpm / (km/h) is fixed for
//  each gear (gear ratio x final drive x tyre size). Each OBD reply with RPM and speed
//  together (010C0D) is matched to the nearest gear.
//
//  Starting ratios (rpm per km/h): gears 1-4 measured from a drive log of this car
//  (2016 S5, 0B5, 19" tyres); 5-7 extrapolated from 4th with Audi's published S tronic
//  spacing (0.915 / 0.745 / 0.617 against 4th's 1.175). They're refined while driving:
//  whenever RPM and speed hold steady for a second at 20 km/h or more, the matching gear's
//  ratio moves a little toward what was measured (kept within 7 % of the starting value,
//  so a wrong match can't drag it away), and the result is saved.
//
//  Shown as "-" (0) when stopped, below 5 km/h, while the clutch slips or during a shift
//  (nothing within 7 %), and a new gear must match twice in a row before it's shown.
//  Plain C++ (no Arduino calls), tested on the desktop by tools/gear_test.cpp.
// =====================================================================
#pragma once
#include <math.h>
#include <stdint.h>

namespace gearx {

static const int N = 7;
static const float DEFAULTS[N] = {116.0f, 69.4f, 45.2f, 32.2f, 25.1f, 20.4f, 16.9f};
static const float TOL = 0.07f;                  // match window (log ratio ~ +/-7 %)

struct Cal { float r[N]; uint16_t n[N]; uint32_t magic; };
static const uint32_t MAGIC = 0x6EA10001;

struct Est {
  Cal  cal;
  int  gear = 0;                                  // shown: 0 = "-", 1..7
  bool dirty = false;                             // calibration changed: save it
  // matching state
  int  cand = 0, candN = 0;
  float lastMatchT = -10;
  // steadiness window (last ~1 s)
  float wT0 = -1, rpmMin = 0, rpmMax = 0, kmhMin = 0, kmhMax = 0, sumX = 0; int wN = 0;
  float lastSave = -1e9f;

  void begin(const Cal *saved) {
    if (saved && saved->magic == MAGIC) {
      cal = *saved;
      for (int i = 0; i < N; i++)                   // a corrupt or far-off value: back to the default
        if (!(cal.r[i] > DEFAULTS[i] * (1 - TOL) && cal.r[i] < DEFAULTS[i] * (1 + TOL))) { cal.r[i] = DEFAULTS[i]; cal.n[i] = 0; }
    } else {
      for (int i = 0; i < N; i++) { cal.r[i] = DEFAULTS[i]; cal.n[i] = 0; }
      cal.magic = MAGIC;
    }
  }

  // nearest gear for a ratio (0 = none within TOL)
  int match(float x) const {
    int best = 0; float bd = 1e9f;
    for (int i = 0; i < N; i++) { float d = fabsf(logf(x / cal.r[i])); if (d < bd) { bd = d; best = i + 1; } }
    return bd < TOL ? best : 0;
  }

  // One OBD reply with RPM and speed from the same moment. t in s.
  void update(float rpm, float kmh, float t) {
    if (kmh < 5 || rpm < 400) { gear = 0; cand = 0; candN = 0; wT0 = -1; return; }
    float x = rpm / kmh;
    int g = match(x);
    if (g && g == cand) candN++; else { cand = g; candN = g ? 1 : 0; }
    if (g && (g == gear || candN >= 2)) { gear = g; lastMatchT = t; }
    else if (!g && t - lastMatchT > 0.4f) gear = 0;  // a moment of no match is a shift: keep the old one briefly

    // learning: a steady second at 20 km/h or more, all in one gear
    if (wT0 < 0 || !g || g != gear) { wT0 = t; rpmMin = rpmMax = rpm; kmhMin = kmhMax = kmh; sumX = 0; wN = 0; }
    rpmMin = fminf(rpmMin, rpm); rpmMax = fmaxf(rpmMax, rpm);
    kmhMin = fminf(kmhMin, kmh); kmhMax = fmaxf(kmhMax, kmh);
    sumX += x; wN++;
    if (g && kmh >= 20 && t - wT0 >= 1.0f && wN >= 4) {
      bool steady = (rpmMax - rpmMin) < 0.03f * rpmMax && (kmhMax - kmhMin) <= 1.0f;
      if (steady) {
        float m = sumX / wN, &r = cal.r[g - 1];
        r += 0.05f * (m - r);
        float lo = DEFAULTS[g - 1] * (1 - TOL * 0.9f), hi = DEFAULTS[g - 1] * (1 + TOL * 0.9f);
        r = fminf(fmaxf(r, lo), hi);
        if (cal.n[g - 1] < 60000) cal.n[g - 1]++;
        if (t - lastSave > 60) { lastSave = t; dirty = true; }   // save at most once a minute
      }
      wT0 = t; rpmMin = rpmMax = rpm; kmhMin = kmhMax = kmh; sumX = 0; wN = 0;
    }
  }
};

}  // namespace gearx
