// =====================================================================
//  S5 Gauge Pod - the watcher: pulls, the car's usual boost, belt slip, warnings
//
//  Fed about 20 times a second with the latest readings (NOV = no fresh reading). It
//    - detects pulls (pedal 80 % or more and on boost; boost alone if there's no pedal
//      reading) and sums each one up (PullSum: peak boost, intake rise, knock, fuel
//      pressure, leanest AFR, injector duty, hottest cat ...)
//    - learns this car's usual boost against RPM at full pedal (500 rpm bins, gears 2 and
//      up), and after each pull says whether boost was as usual or below it: a pulley car
//      can't reach the ECU's target at high RPM anyway, so "below the target" means little,
//      but "below its own usual" means a slipping belt, a leak or heat
//    - flags a belt slip during a pull: boost falling 1.5 psi while RPM climbs 400 in the
//      same gear, foot still down
//    - raises warnings (warning list below). Each has a hold time before it shows and a
//      lower clear level, so it doesn't flicker. The sketch jumps to the warning's page and
//      shows the banner; when it's over it goes back to where you were.
//  Plain C++ (no Arduino calls), tested on the desktop by tools/watch_test.cpp.
// =====================================================================
#pragma once
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "pull_sum.h"

namespace watch {
using gr::NOV; using gr::has;

struct In {
  float t = 0;                                       // s
  float psi = NOV;                                   // sender boost (psi, vacuum negative)
  float rpm = NOV, kmh = NOV, pedal = NOV, thr = NOV; int gear = 0;
  float iat = NOV, timing = NOV, knock = NOV; int knockCyl = 0;
  float hpfp = NOV, hpfpSet = NOV, lpfp = NOV, idc = NOV, afr1 = NOV, afr2 = NOV, afrCmd = NOV;
  float cat1 = NOV, cat2 = NOV, clt = NOV, oil = NOV, bat = NOV, bypass = NOV, lon = NOV;
  float obdPsi = NOV, tgtPsi = NOV;
  float afrAt = NOV, lpfpAt = NOV;                   // when the AFR / low-side reading was taken (s, same clock as t)
  float redPsi = 20;
  float runSec = 0;                                  // engine running this long (s)
};

// ---------------------------------------------------------------- thresholds
static const float CLT_RED = 230, OIL_AMBER = 260, OIL_RED = 280, IAT_AMBER = 150;
static const float BAT_LO = 12.0f, BAT_HI = 15.2f;
// knock: amber at -3 deg of new retard under load, red at -4.5 (a stage 2 car on 93 sees -1 to -3 now and then)
static const float KNOCK_AMBER = -3.0f, KNOCK_RED = -4.5f, KNOCK_LOAD_PSI = 8, HPFP_FRAC = 0.85f, LPFP_LO = 40, IDC_AMBER = 90;
// Lean: against what the ECU commands, once boost has settled. Calibrated on drive logs: at light
// boost the 3.0T runs at 14.7 on purpose, the wideband reads 29.4 in a fuel cut, and the AFR is
// polled about once a second, so the first reading after a tip-in is from before it.
static const float LEAN_OVER = 1.0f;                 // AFR leaner than commanded by this much
static const float LEAN_AFR = 13.5f;                 // ...or, with no commanded value, leaner than this
static const float AFR_SANE = 20.0f;                 // above this: fuel cut / sensor at its limit, not a reading
static const float BOOST_SETTLE_S = 1.0f;            // on boost this long before AFR counts
// Cats: the ECU's modelled temperature. Hard pulls on this car reach about 1750 F (its own
// component protection); amber above that, red well above it.
static const float CAT_AMBER = 1800, CAT_RED = 1850; // degF (982 / 1010 degC)
static const float HPFP_MIN_SET = 1200;              // fuel pump check only under load (at idle it lags its target and it doesn't matter)

// ---------------------------------------------------------------- warnings
enum Id { A_BOOST, A_CLT, A_OIL, A_IAT, A_BAT, A_KNOCK, A_LEAN, A_HPFP, A_LPFP, A_IDC, A_CAT, A_PULL, A_N };
struct Alarm {
  int   page = 0, sev = 0;                           // sev: 0 off, 1 amber, 2 red
  bool  on = false;
  float since = -1, clearSince = -1, onAt = 0;       // condition true since / false since
  char  text[32] = "";                              // one line (banner, web page, serial log)
  // the same, split up for the big pop-up: title, number, unit, a note under it
  char  title[20] = "", value[12] = "", unit[8] = "", note[24] = "";
};
static const float UNDERBOOST = -1.5f;               // psi against the usual boost: flagged

static const int NB = 10;                           // usual-boost bins: 2500-7500 rpm, 500 each
struct Curve { float psi[NB]; uint16_t n[NB]; uint16_t pulls; uint32_t magic; };
static const uint32_t CURVE_MAGIC = 0xB0057001;

struct Watch {
  Alarm a[A_N];
  gr::PullSum last;                                 // last finished pull
  int   pullsDone = 0;                              // count (the sketch notices it changing)
  Curve curve;
  bool  curveDirty = false;

  // page numbers (gauge_render.h Page) set by the sketch
  int pBoost = 0, pOverview = 1, pTemps = 2, pSystem = 4, pFuel = 8, pHeat = 9, pPull = 10;

  // ---- pull in progress
  bool  inPull = false;
  float boostSince = -1;                            // psi > 3 since
  float pullT0 = 0, liftSince = -1;
  gr::PullSum cur;
  float binMax[NB];
  int   lastGear = 0;
  float gearMaxPsi = -99, gearMaxRpm = 0, slipSince = -1;
  // knock: the ECU's retard per cylinder is held and eases off slowly, so a value from a
  // light-load tip-in is still there when boost comes in (seen on the S5: -5.6 deg on cylinder 6
  // at 3000 rpm part throttle, then carried into the pull). Only knock that gets worse under
  // load counts: kRef follows the reading off load, and follows it up (easing off) on load.
  float kRef = 0; bool kRefSet = false, knockNew = false;
  // a shift (RPM falling fast): fuel pressure and AFR samples from it are left out of the pull
  float lastRpm = 0, shiftUntil = -1;
  float shiftFrom = -1;                             // this shift's window: shiftFrom .. shiftUntil
  // AFR: a reading taken during a gear change is left out (the S5 showed 17.2 on bank 1 at a full-throttle
  // 1-2 shift, the ECU cutting fuel for the shift; the next reading was 11.9)
  bool afrInShift(const In &in) const { float a = has(in.afrAt) ? in.afrAt : in.t; return a >= shiftFrom && a <= shiftUntil; }
  // the same for fuel pressure, by when it was read (it comes ~1.3 s apart on boost, so a reading from
  // mid-shift was still held after the shift window: log_0048 had -241 psi from a 2-3 shift, next on target)
  bool fuelInShift(const In &in) const { float a = has(in.lpfpAt) ? in.lpfpAt : in.t; return a >= shiftFrom && a <= shiftUntil; }
  // knock under load needs a steady foot: the pedal past 40 % for a second. Part-throttle tip-ins in the
  // low gears (17-54 % pedal, boost briefly past 8 psi) read -3 to -5.6 on cylinder 6 again and again
  // (logs 36-48), never with the foot held down: most likely the driveline, not knock.
  float pedalHiSince = -1;
  // low side: one low reading isn't enough (a 28 psi dip at a part-throttle tip-in, high side on target)
  float lpLowAt = -1; int lpLowN = 0;
  // lean: the same, two separate readings. AFR is read about once a second on boost, and as boost
  // builds the ECU drops its target (14.3 -> 13.2 in one reply on log 25) before the wideband follows:
  // one reading a point over, then on target. Held on screen for a second, it raised a red warning.
  float leanAt = -1; int leanN = 0;
  // fuel pump short of its target: the same (log_0036: one reading 1261 of 1494 as the target jumped
  // at a 7->4 downshift tip-in, the next 1623 of 1631). Fuel pressures come in one request with the
  // low side, so lpfpAt dates them.
  float hpShortAt = -1; int hpShortN = 0;
  // lifting off: the pedal up, or the sender falling fast (a lift drops ~50 psi/s, a shift's dip
  // much less). Knock read then isn't load knock (log_0036: -5.6 on cylinder 6 at pedal 0).
  float psiRef = 0, psiRefT = -1, psiRate = 0;
  static constexpr float LIFT_PSI_RATE = -20.0f;    // psi a second
  // ...or boost more than LIFT_DROP_PSI under its highest of the last half second (catches a lift on the
  // next sample: log_0047 fell 13 -> 4.6 psi in 0.12 s, between two rate updates; a shift dips 2-3 psi)
  float psiHi = -99, psiHiT = -1;
  static constexpr float LIFT_DROP_PSI = 4.0f;

  void begin(const Curve *saved) {
    if (saved && saved->magic == CURVE_MAGIC) curve = *saved;
    else { for (int i = 0; i < NB; i++) { curve.psi[i] = 0; curve.n[i] = 0; } curve.pulls = 0; curve.magic = CURVE_MAGIC; }
  }

  // the most important warning showing now (-1 = none): red first, then the newest
  int top() const {
    int best = -1;
    for (int i = 0; i < A_N; i++) {
      if (!a[i].on) continue;
      if (best < 0 || a[i].sev > a[best].sev || (a[i].sev == a[best].sev && a[i].onAt > a[best].onAt)) best = i;
    }
    return best;
  }

  // ---- one condition: raise after `hold` s true, clear after `clearHold` s of `stillBad` false
  // pop-up parts for the next judge() call
  char pT[20] = "", pV[12] = "", pU[8] = "", pN[24] = "";
  void pop(const char *title, float v, int dec, const char *unit, const char *note = "") {
    snprintf(pT, sizeof(pT), "%s", title); snprintf(pV, sizeof(pV), "%.*f", dec, v);
    snprintf(pU, sizeof(pU), "%s", unit); snprintf(pN, sizeof(pN), "%s", note);
  }
  void judge(int id, float t, bool bad, bool stillBad, int sev, float hold, float clearHold, int page, const char *txt) {
    Alarm &x = a[id];
    if (!x.on) {
      if (bad) { if (x.since < 0) x.since = t; if (t - x.since >= hold) { x.on = true; x.onAt = t; x.clearSince = -1; } }
      else x.since = -1;
    } else {
      if (stillBad) x.clearSince = -1;
      else { if (x.clearSince < 0) x.clearSince = t; if (t - x.clearSince >= clearHold) { x.on = false; x.since = -1; } }
    }
    if (x.on) {
      if (x.sev == 0 || (bad && sev > x.sev)) x.sev = sev;   // escalates amber -> red while on, never back
      x.page = page;
      if (bad) {
        snprintf(x.text, sizeof(x.text), "%s", txt);
        memcpy(x.title, pT, sizeof(pT)); memcpy(x.value, pV, sizeof(pV)); memcpy(x.unit, pU, sizeof(pU)); memcpy(x.note, pN, sizeof(pN));
      }
    } else x.sev = 0;
  }

  void step(const In &in) {
    char b[32];
    float t = in.t;
    bool running = in.runSec > 0;
    bool onBoost = has(in.psi) && in.psi > 3;
    if (!onBoost) boostSince = -1; else if (boostSince < 0) boostSince = t;
    bool settled = onBoost && t - boostSince >= BOOST_SETTLE_S;
    // real load: well on boost or the pedal near the floor. A 1st-gear tip-in at 5 psi isn't (the S5 shows
    // -5.6 deg on cylinder 6 there again and again: likely the knock sensor hearing the driveline, not knock)
    if (has(in.psi) && (psiRefT < 0 || t - psiRefT >= 0.2f)) {
      if (psiRefT >= 0) psiRate = (in.psi - psiRef) / (t - psiRefT);
      psiRef = in.psi; psiRefT = t;
    }
    if (has(in.psi)) {
      if (psiHiT < 0 || in.psi >= psiHi || t - psiHiT > 0.5f) { psiHi = in.psi; psiHiT = t; }
    }
    bool dropping = has(in.psi) && psiHi - in.psi > LIFT_DROP_PSI;
    bool lifting = (has(in.pedal) && in.pedal < 40) || psiRate < LIFT_PSI_RATE || dropping;
    if (!has(in.pedal) || in.pedal < 40) pedalHiSince = -1; else if (pedalHiSince < 0) pedalHiSince = t;
    bool steadyFoot = !has(in.pedal) || (in.pedal >= 60 && pedalHiSince >= 0 && t - pedalHiSince >= 1.0f);
    bool loaded = ((has(in.psi) && in.psi > KNOCK_LOAD_PSI) || (has(in.pedal) && in.pedal >= 80)) && !lifting && steadyFoot;
    knockNew = false;
    if (has(in.knock)) {
      if (!loaded || !kRefSet) { kRef = in.knock; kRefSet = true; }
      else if (in.knock > kRef) kRef = in.knock;
      knockNew = loaded && in.knock < kRef - 0.2f;
    }
    if (has(in.rpm)) {
      if (in.rpm < lastRpm - 150) { if (t > shiftUntil) shiftFrom = t - 0.3f; shiftUntil = t + 0.5f; }
      lastRpm = in.rpm;
    }
    if (has(in.lpfp)) {                               // separate low readings in a row
      float at = has(in.lpfpAt) ? in.lpfpAt : t;
      if (in.lpfp >= LPFP_LO) lpLowN = 0;
      else if (at != lpLowAt) { lpLowAt = at; lpLowN++; }
    }

    // ---- warnings ------------------------------------------------------------
    { bool bad = has(in.psi) && in.psi >= in.redPsi;
      snprintf(b, sizeof(b), "BOOST %.1f PSI", has(in.psi) ? in.psi : 0.0f);
      pop("BOOST", has(in.psi) ? in.psi : 0.0f, 1, "PSI");
      judge(A_BOOST, t, bad, has(in.psi) && in.psi >= in.redPsi - 1, 2, 0.3f, 2, pBoost, b); }
    { bool bad = running && has(in.clt) && in.clt >= CLT_RED;
      snprintf(b, sizeof(b), "COOLANT %.0f\xc2\xb0" "F", has(in.clt) ? in.clt : 0.0f);
      pop("COOLANT", has(in.clt) ? in.clt : 0.0f, 0, "\xc2\xb0" "F");
      judge(A_CLT, t, bad, has(in.clt) && in.clt >= CLT_RED - 5, 2, 3, 5, pTemps, b); }
    { bool bad = running && has(in.oil) && in.oil >= OIL_AMBER;
      snprintf(b, sizeof(b), "OIL %.0f\xc2\xb0" "F", has(in.oil) ? in.oil : 0.0f);
      pop("OIL", has(in.oil) ? in.oil : 0.0f, 0, "\xc2\xb0" "F");
      judge(A_OIL, t, bad, has(in.oil) && in.oil >= OIL_AMBER - 5, has(in.oil) && in.oil >= OIL_RED ? 2 : 1, 3, 5, pOverview, b); }
    { bool moving = has(in.kmh) && in.kmh > 15;
      bool bad = running && in.runSec > 60 && moving && has(in.iat) && in.iat >= IAT_AMBER;
      snprintf(b, sizeof(b), "INTAKE %.0f\xc2\xb0" "F", has(in.iat) ? in.iat : 0.0f);
      pop("INTAKE", has(in.iat) ? in.iat : 0.0f, 0, "\xc2\xb0" "F");
      judge(A_IAT, t, bad, has(in.iat) && in.iat >= IAT_AMBER - 5, 1, 3, 5, pTemps, b); }
    { bool bad = running && in.runSec > 90 && has(in.bat) && (in.bat < BAT_LO || in.bat > BAT_HI);
      snprintf(b, sizeof(b), "BATTERY %.1f V", has(in.bat) ? in.bat : 0.0f);
      pop("BATTERY", has(in.bat) ? in.bat : 0.0f, 1, "V");
      judge(A_BAT, t, bad, has(in.bat) && (in.bat < BAT_LO + 0.3f || in.bat > BAT_HI - 0.2f), 1, 10, 10, pSystem, b); }
    { bool bad = running && knockNew && in.knock <= KNOCK_AMBER;    // new knock under load only
      if (in.knockCyl) snprintf(b, sizeof(b), "KNOCK %.1f\xc2\xb0 CYL %d", has(in.knock) ? in.knock : 0.0f, in.knockCyl);
      else snprintf(b, sizeof(b), "KNOCK %.1f\xc2\xb0", has(in.knock) ? in.knock : 0.0f);
      { char n[24]; snprintf(n, sizeof(n), in.knockCyl ? "CYLINDER %d" : "", in.knockCyl); pop("KNOCK", has(in.knock) ? in.knock : 0.0f, 1, "\xc2\xb0", n); }
      judge(A_KNOCK, t, bad, knockNew || (loaded && has(in.knock) && in.knock <= -1.0f), in.knock <= KNOCK_RED ? 2 : 1, 0, 3, pFuel, b); }
    { float a1 = has(in.afr1) && in.afr1 < AFR_SANE ? in.afr1 : 0, a2 = has(in.afr2) && in.afr2 < AFR_SANE ? in.afr2 : 0;
      float lean = fmaxf(a1, a2);
      int bank = a2 > a1 ? 2 : 1;
      bool pedalOk = !has(in.pedal) || in.pedal >= 50;
      float over = has(in.afrCmd) ? lean - in.afrCmd : lean - LEAN_AFR + LEAN_OVER;   // no command: 13.5 counts as 1.0 over
      bool overNow = settled && pedalOk && lean > 0 && over >= LEAN_OVER && !afrInShift(in);
      float at = has(in.afrAt) ? in.afrAt : t;
      if (!overNow) leanN = 0;
      else if (at != leanAt) { leanAt = at; leanN++; }
      bool bad = overNow && leanN >= 2;
      snprintf(b, sizeof(b), "LEAN AFR %.1f BANK %d", lean, bank);
      { char n[24]; if (has(in.afrCmd)) snprintf(n, sizeof(n), "TARGET %.1f, BANK %d", in.afrCmd, bank); else snprintf(n, sizeof(n), "BANK %d ON BOOST", bank);
        pop("LEAN", lean, 1, "AFR", n); }
      judge(A_LEAN, t, bad, settled && lean > 0 && over >= LEAN_OVER - 0.4f, 2, 0.6f, 1, pFuel, b); }
    { bool okSet = has(in.hpfp) && has(in.hpfpSet) && in.hpfpSet >= HPFP_MIN_SET && onBoost;
      bool shortNow = running && okSet && in.hpfp < in.hpfpSet * HPFP_FRAC;
      float at = has(in.lpfpAt) ? in.lpfpAt : t;
      if (!shortNow) hpShortN = 0;
      else if (at != hpShortAt) { hpShortAt = at; hpShortN++; }
      bool bad = shortNow && hpShortN >= 2;
      snprintf(b, sizeof(b), "FUEL PUMP %.0f OF %.0f PSI", okSet ? in.hpfp : 0.0f, okSet ? in.hpfpSet : 0.0f);
      { char n[24]; snprintf(n, sizeof(n), "TARGET %.0f", okSet ? in.hpfpSet : 0.0f); pop("FUEL PUMP", okSet ? in.hpfp : 0.0f, 0, "PSI", n); }
      judge(A_HPFP, t, bad, okSet && in.hpfp < in.hpfpSet * 0.92f, 2, 0.6f, 1, pFuel, b); }
    { bool hpShort = has(in.hpfp) && has(in.hpfpSet) && in.hpfpSet >= HPFP_MIN_SET && in.hpfp < in.hpfpSet * 0.92f;
      // at part throttle the ECU runs the low side on demand and it can lag a roll-on for a few seconds
      // (log_0038: 29-36 psi at 35-62 % pedal, high side on target, then 94): only at full pedal, unless
      // the high side falls short too. At 80 % pedal or more it never went under 55 psi.
      bool fullPedal = !has(in.pedal) || in.pedal >= 80;
      bool bad = onBoost && has(in.lpfp) && in.lpfp < LPFP_LO && ((lpLowN >= 2 && fullPedal) || hpShort);   // stays low, or starves the high side
      snprintf(b, sizeof(b), "LOW SIDE FUEL %.0f PSI", has(in.lpfp) ? in.lpfp : 0.0f);
      pop("LOW SIDE FUEL", has(in.lpfp) ? in.lpfp : 0.0f, 0, "PSI");
      judge(A_LPFP, t, bad, has(in.lpfp) && in.lpfp < LPFP_LO + 5, 1, 0.5f, 2, pFuel, b); }
    { bool bad = has(in.idc) && in.idc >= IDC_AMBER;
      snprintf(b, sizeof(b), "INJECTORS %.0f%%", has(in.idc) ? in.idc : 0.0f);
      pop("INJECTORS", has(in.idc) ? in.idc : 0.0f, 0, "%");
      judge(A_IDC, t, bad, has(in.idc) && in.idc >= IDC_AMBER - 5, 1, 0.3f, 2, pFuel, b); }
    { float hot = fmaxf(has(in.cat1) ? in.cat1 : 0, has(in.cat2) ? in.cat2 : 0);
      int bank = has(in.cat2) && (!has(in.cat1) || in.cat2 > in.cat1) ? 2 : 1;
      bool bad = running && hot >= CAT_AMBER;
      snprintf(b, sizeof(b), "CAT %d %.0f\xc2\xb0" "F", bank, hot);
      { char n[24]; snprintf(n, sizeof(n), "BANK %d", bank); pop("CATALYST", hot, 0, "\xc2\xb0" "F", n); }
      judge(A_CAT, t, bad, hot >= CAT_AMBER - 50, hot >= CAT_RED ? 2 : 1, 5, 5, pHeat, b); }
    // A_PULL: set by the pull summary below, shown for 10 s
    if (a[A_PULL].on && t - a[A_PULL].onAt > 10) { a[A_PULL].on = false; a[A_PULL].sev = 0; }

    // ---- pulls ------------------------------------------------------------------
    bool pedalKnown = has(in.pedal);
    bool footDown = pedalKnown ? in.pedal >= 80 : (has(in.psi) && in.psi >= 4);
    bool lifted   = pedalKnown ? in.pedal < 50  : (!has(in.psi) || in.psi < 0);
    if (!inPull) {
      if (running && footDown && has(in.psi) && in.psi >= 2) {
        inPull = true; pullT0 = t; liftSince = -1;
        cur = gr::PullSum();
        cur.kmh0 = has(in.kmh) ? in.kmh : 0;
        cur.iat0 = in.iat;
        for (int i = 0; i < NB; i++) binMax[i] = NOV;
        lastGear = 0; gearMaxPsi = -99; gearMaxRpm = 0; slipSince = -1;
      }
    }
    if (inPull) {
      if (lifted || !running || (has(in.psi) && in.psi < -2)) { if (liftSince < 0) liftSince = t; }
      else liftSince = -1;
      if (liftSince < 0) accumulate(in, pedalKnown);     // not the lift-off tail (it would add the next gear)
      if (liftSince >= 0 && t - liftSince >= 0.4f) finish(in);
    }
  }

  void accumulate(const In &in, bool pedalKnown) {
    gr::PullSum &p = cur;
    if (has(in.psi) && (!has(p.peakPsi) || in.psi > p.peakPsi)) { p.peakPsi = in.psi; p.peakRpm = has(in.rpm) ? in.rpm : 0; }
    auto mx = [](float &dst, float v) { if (has(v) && (!has(dst) || v > dst)) dst = v; };
    auto mn = [](float &dst, float v) { if (has(v) && (!has(dst) || v < dst)) dst = v; };
    mx(p.obdPeak, in.obdPsi); mx(p.tgtPeak, in.tgtPsi); mx(p.iatMax, in.iat); mn(p.minTiming, in.timing);
    // knock that started in this pull (0 = none; a retard held from before doesn't count)
    if (has(in.knock) && !has(p.knock)) { p.knock = 0; p.knockCyl = 0; }
    if (knockNew && in.knock < p.knock) { p.knock = in.knock; p.knockCyl = in.knockCyl; }
    bool shifting = in.t < shiftUntil;
    if (!shifting && !fuelInShift(in)) {
      if (has(in.hpfp) && has(in.hpfpSet) && in.hpfpSet >= HPFP_MIN_SET) mn(p.hpfpDef, in.hpfp - in.hpfpSet);
      mn(p.hpfpMin, in.hpfp);
    }
    mn(p.lpfpMin, in.lpfp);
    // AFR once boost has settled, and only real readings (not 29.4 from a fuel cut before the pull)
    if (!shifting && !afrInShift(in) && has(in.psi) && in.psi > 3 && boostSince >= 0 && in.t - boostSince >= BOOST_SETTLE_S) {
      const float afrs[2] = {in.afr1, in.afr2};
      for (float a : afrs) if (has(a) && a < AFR_SANE) {
        mx(p.afrMax, a);
        if (has(in.afrCmd)) mx(p.afrOver, a - in.afrCmd);
      }
    }
    mx(p.idcMax, in.idc); mx(p.catMax, in.cat1); mx(p.catMax, in.cat2); mn(p.bypassMin, in.bypass); mx(p.lonMax, in.lon);
    if (has(in.kmh)) p.kmh1 = in.kmh;
    // gears used, in order ("2-3-4")
    if (in.gear >= 1 && in.gear <= 7 && in.gear != lastGear) {
      size_t L = strlen(p.gears);
      if (L + 3 < sizeof(p.gears)) { if (L) p.gears[L++] = '-'; p.gears[L++] = (char)('0' + in.gear); p.gears[L] = 0; }
      lastGear = in.gear; gearMaxPsi = -99; gearMaxRpm = 0; slipSince = -1;
    }
    // usual-boost bins and the slip check: full pedal, 2nd gear and up
    bool full = pedalKnown ? in.pedal >= 85 : (has(in.thr) && in.thr >= 80);
    if (full && in.gear >= 2 && has(in.psi) && has(in.rpm) && in.rpm >= 2500 && in.rpm < 7500 && in.psi > 0) {
      int bi = (int)((in.rpm - 2500) / 500);
      if (!has(binMax[bi]) || in.psi > binMax[bi]) binMax[bi] = in.psi;
      if (in.psi > gearMaxPsi) { gearMaxPsi = in.psi; gearMaxRpm = in.rpm; }
      bool falling = in.psi < gearMaxPsi - 1.5f && in.rpm > gearMaxRpm + 400;
      if (falling) { if (slipSince < 0) slipSince = in.t; if (in.t - slipSince >= 0.2f) p.slip = true; }
      else slipSince = -1;
    }
  }

  void finish(const In &in) {
    inPull = false;
    gr::PullSum &p = cur;
    p.durS = in.t - pullT0 - 0.4f;
    if (p.durS < 1.5f || !has(p.peakPsi) || p.peakPsi < 5) return;      // a blip, not a pull
    // against the usual boost: median of the bins that have history
    float dev[NB]; int nd = 0, used = 0;
    for (int i = 0; i < NB; i++) {
      if (!has(binMax[i])) continue;
      used++;
      if (curve.n[i] >= 3) dev[nd++] = binMax[i] - curve.psi[i];
    }
    if (nd >= 2) {
      for (int i = 1; i < nd; i++) for (int j = i; j > 0 && dev[j] < dev[j - 1]; j--) { float x = dev[j]; dev[j] = dev[j - 1]; dev[j - 1] = x; }
      p.vsNormal = nd % 2 ? dev[nd / 2] : 0.5f * (dev[nd / 2 - 1] + dev[nd / 2]);
    }
    // learn (slowly when this pull looked wrong, so a fault doesn't become "usual")
    if (used) {
      bool bad = p.slip || (has(p.vsNormal) && p.vsNormal <= UNDERBOOST);
      for (int i = 0; i < NB; i++) {
        if (!has(binMax[i])) continue;
        if (curve.n[i] == 0) curve.psi[i] = binMax[i];
        else curve.psi[i] += (bad ? 0.05f : 0.25f) * (binMax[i] - curve.psi[i]);
        if (curve.n[i] < 60000) curve.n[i]++;
      }
      if (curve.pulls < 60000) curve.pulls++;
      curveDirty = true;
    }
    p.normalPulls = curve.pulls;
    for (int i = 0; i < NB && i < 10; i++) p.binPsi[i] = binMax[i];
    p.valid = true;
    p.endMs = (uint32_t)(in.t * 1000);
    last = p;
    pullsDone++;
    // a boost problem gets its own warning (amber, on the Last Pull page, 10 s)
    if (p.slip || (has(p.vsNormal) && p.vsNormal <= UNDERBOOST)) {
      Alarm &x = a[A_PULL];
      x.on = true; x.sev = p.slip ? 2 : 1; x.onAt = in.t; x.page = pPull;
      if (p.slip) {
        snprintf(x.text, sizeof(x.text), "%s", "BOOST FELL: BELT SLIP?");
        snprintf(x.title, sizeof(x.title), "BELT SLIP?"); x.value[0] = 0; x.unit[0] = 0; x.note[0] = 0;
      } else {
        snprintf(x.text, sizeof(x.text), "BOOST %.1f PSI LOW", -p.vsNormal);
        snprintf(x.title, sizeof(x.title), "BOOST LOW"); snprintf(x.value, sizeof(x.value), "%.1f", p.vsNormal);
        snprintf(x.unit, sizeof(x.unit), "PSI"); x.note[0] = 0;
      }
    }
  }
};

}  // namespace watch
