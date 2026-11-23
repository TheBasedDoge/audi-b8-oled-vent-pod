// =====================================================================
//  S5 Gauge Pod - performance timers: 0-60 mph, 1/4 mile (from a stop) and 60-130 mph (rolling)
//
//  Speed. OBD road speed comes a few times a second, in whole km/h and a little late, which
//  on its own would make every time wobble by a tenth or two. So between OBD readings the
//  speed is carried forward by the G-meter's forward acceleration (100 times a second), and
//  each OBD reading pulls that estimate back toward the truth, compared with what the
//  estimate said OBD_LAG seconds earlier (when the car actually had that speed). A slow
//  correction also learns the accelerometer's small offset (road slope, the pod's tilt).
//  Distance (1/4 mile) is the integral of that speed.
//
//  Start. Arms after a second at a standstill (OBD 0 km/h). While waiting it learns what the
//  G-meter reads at rest, so a sloped road doesn't count as acceleration. A launch is 0.15 g
//  for 80 ms; the run is timed from the moment the acceleration began rising (0.04 g), not
//  from when it crossed 0.15 g. With "rollout" on, the clock starts after the first foot of
//  travel instead, as drag strips and magazines time it (about 0.2-0.3 s quicker).
//
//  60-130 starts when the speed passes 60 mph while accelerating hard, whether from a stop
//  or rolling, after having been below 58 mph.
//
//  A run is dropped (no time) when you lift (pedal under 50 % for 0.7 s, or no acceleration
//  for a second), the car is still crawling after 3 s, or it takes far too long.
//
//  Without a calibrated G-meter it works from OBD speed alone (straight lines between
//  readings), marked "OBD only": good to a few tenths.
//
//  Plain C++ (no Arduino calls), tested on the desktop by tools/perf_test.cpp.
// =====================================================================
#pragma once
#include <math.h>
#include <stdint.h>

namespace perf {

static const float NOV  = -9999.0f;                     // "no value" (same as gr::NOV)
static inline bool has(float v) { return v > -9000.0f; }
static const float G    = 9.80665f;
static const float MPH  = 0.44704f;                     // m/s per mph
static const float V60  = 60 * MPH, V130 = 130 * MPH, V58 = 58 * MPH, V57 = 57 * MPH;
static const float QUARTER = 402.336f, ROLLOUT = 0.3048f;

enum Kind { K060 = 0, K60130 = 1, KQM = 2 };
static const char *kindName(int k) { return k == K060 ? "0-60" : k == K60130 ? "60-130" : "1/4 mile"; }

struct Result {
  uint8_t kind = 0;
  float   t = 0;                                         // s
  float   mph = 0;                                       // speed at the end (1/4 mile trap speed)
  float   psi = NOV;                                     // peak boost during the run (sender)
  bool    obdOnly = false, rollout = false;
  float   at = 0;                                        // when it finished (the caller's clock, s)
};

struct Timer {
  // ---- settings
  float lag = 0.12f;                                     // OBD speed lag, s
  bool  rollout = false;

  // ---- outputs (for the Perf page)
  int   state = 0;                                       // 0 idle, 1 armed at a stop, 2 running
  float liveT060 = 0, liveT6130 = 0, liveTQm = 0;        // running clocks (0 = that one isn't running)
  float mph = 0;                                         // current estimate, mph
  bool  obdMode = false;                                 // no G-meter: OBD speed only
  float t060 = 0, t6130 = 0, tQm = 0, qmMph = 0;         // last results
  float b060 = 0, b6130 = 0, bQm = 0;                    // best since power-on
  const char *msg = "STOP TO ARM";
  Result q[6]; int qn = 0;                               // finished runs, for the caller to collect
  int   results = 0;                                     // count of all finished runs

  // ---- speed estimate
  float u = 0, off = 0;                                  // v = u + off; u = integrated acceleration (m/s)
  float bias = 0;                                        // accelerometer reading at rest, g
  float dist = 0;                                        // m since the launch
  float hT[64], hU[64]; int hN = 0, hI = 0;              // u history (every 20 ms), for the OBD lag
  float histAt = -1;
  float obdT = -1, obdV = -1, obdPrevT = -1, obdPrevV = -1;  // last two OBD readings (s, m/s)
  float lastT = -1, vPrev = 0, accF = 0;                 // accF = acceleration, lightly filtered (g)
  float zeroSince = -1;                                  // OBD 0 km/h since
  bool  imuMode = false, seeded = false;

  // ---- standing run
  float riseAt = 0, uRise = 0, dRise = 0;                // acceleration above 0.04 g since...
  float trigSince = -1;
  float t0 = 0, tStart = -1, dStart = 0;                 // launch, clock start (after rollout), distance there
  bool  sDone060 = false;
  float sPsi = NOV;
  // ---- rolling run
  bool  rArmed = false, rRun = false;
  float r0 = 0, rPsi = NOV;
  // ---- lift / timeout tracking (shared)
  float lowPedalSince = -1, sNoAcc = -1, rNoAcc = -1;
  bool  departed = false;                                // OBD only: the speed has left 0 since arming

  float v() const { float x = u + off; return x > 0 ? x : 0; }

  void push(int kind, float t, float vEnd, float psi, float at) {
    Result r; r.kind = kind; r.t = t; r.mph = vEnd / MPH; r.psi = psi; r.obdOnly = obdMode; r.rollout = rollout && kind != K60130; r.at = at;
    if (qn < 6) q[qn++] = r;
    results++;
    if (kind == K060)   { t060 = t;  if (!b060 || t < b060) b060 = t; }
    if (kind == K60130) { t6130 = t; if (!b6130 || t < b6130) b6130 = t; }
    if (kind == KQM)    { tQm = t; qmMph = r.mph; if (!bQm || t < bQm) bQm = t; }
  }
  int take(Result *out, int max) { int n = qn < max ? qn : max; for (int i = 0; i < n; i++) out[i] = q[i]; for (int i = n; i < qn; i++) q[i - n] = q[i]; qn -= n; return n; }
  void resetBest() { b060 = b6130 = bQm = 0; }

  float uAt(float t) const {                             // u at an earlier time (linear between history points)
    if (!hN) return u;
    int newest = (hI + 63) % 64;
    if (t >= hT[newest]) return u;
    float pt = hT[newest], pu = hU[newest];
    for (int k = 1; k < hN; k++) {
      int i = (newest - k + 64) % 64;
      if (hT[i] <= t) { float f = (t - hT[i]) / (pt - hT[i] + 1e-6f); return hU[i] + f * (pu - hU[i]); }
      pt = hT[i]; pu = hU[i];
    }
    return pu;                                           // older than the history: the oldest
  }

  // One step, about 100 times a second.
  //   t      s (any steady clock)
  //   lonG   forward acceleration from the calibrated G-meter, unfiltered (NOV = none)
  //   kmh    the latest OBD speed (NOV = none), kmhT = when it arrived (a new value is picked
  //          up when kmhT changes)
  //   pedal  accelerator pedal %, NOV when not fresh
  //   psi    boost now
  void step(float t, float lonG, float kmh, float kmhT, float pedal, float psi) {
    float dt = lastT < 0 ? 0.01f : t - lastT;
    if (dt <= 0) return;
    if (dt > 0.1f) dt = 0.1f;
    lastT = t;
    bool newObd = has(kmh) && kmhT != obdT;
    bool obdFresh = has(kmh) && t - kmhT < 1.5f;
    if (newObd) { obdPrevT = obdT; obdPrevV = obdV; obdT = kmhT; obdV = kmh / 3.6f; }
    if (!obdFresh) {                                     // no speed: nothing can be timed
      state = 0; liveT060 = liveT6130 = liveTQm = 0; rRun = false; rArmed = false; mph = 0;
      msg = "NO SPEED FROM OBD"; zeroSince = -1; return;
    }
    if (obdV < 0.05f) { if (zeroSince < 0) zeroSince = kmhT; } else zeroSince = -1;

    // the mode is fixed while a run is on
    if (state != 2 && !rRun) imuMode = has(lonG);
    obdMode = !imuMode;

    // ---- speed and acceleration
    float acc;                                           // g, offset removed
    vPrev = v();
    if (imuMode) {
      if (!has(lonG)) lonG = bias;                       // lost mid-run: coast on the estimate
      acc = lonG - bias;
      u += acc * G * dt;
      if (t - histAt >= 0.02f) { histAt = t; hT[hI] = t; hU[hI] = u; hI = (hI + 1) % 64; if (hN < 64) hN++; }
      if (newObd) {
        float tq = obdT - lag;
        bool early = state == 2 && tq < t0 + 0.1f;      // before the launch the estimate was held at 0
        if (!seeded) { seeded = true; off = obdV - u; }   // first reading: start from the OBD speed
        else if (!early) {
          float e = obdV - (uAt(tq) + off);
          float kv = fabsf(accF) > 0.15f ? 0.12f : 0.3f; // trust the G-meter more while accelerating hard
          off += kv * e;
          // the offset learns only from small, steady errors (a big one is the estimate catching up)
          if (state != 1 && fabsf(e) < 1.0f) { bias -= 0.004f * e; if (bias > 0.3f) bias = 0.3f; if (bias < -0.3f) bias = -0.3f; }
        }
      }
      if (u + off < 0) off = -u;
    } else {
      // OBD only: straight lines between readings, OBD_LAG back
      float a = 0, vv = obdV;
      if (obdPrevT >= 0 && obdT > obdPrevT) {
        a = (obdV - obdPrevV) / (obdT - obdPrevT);
        float tt = t - lag;
        vv = obdPrevV + a * (tt - obdPrevT);
        if (tt > obdT + 0.3f) { vv = obdV; a = 0; }      // readings stopped: hold
        if (vv < 0) vv = 0;
      }
      u = vv; off = 0;
      acc = a / G;
    }
    float vNow = v();
    mph = vNow / MPH;
    accF += (acc - accF) * (1 - expf(-dt / 0.03f));
    dist += 0.5f * (vPrev + vNow) * dt;

    // ---- lift tracking
    if (has(pedal) && pedal < 50) { if (lowPedalSince < 0) lowPedalSince = t; } else lowPedalSince = -1;
    // lifted: under 50 % for 0.7 s, counted from the start of the run (the pedal is polled slowly
    // before a launch, so the last reading may still say 0 for a moment)
    auto lifted = [&](float since) { return lowPedalSince >= 0 && t - fmaxf(lowPedalSince, since + 0.3f) > 0.7f; };

    // ---- standing run
    bool stopped = zeroSince >= 0 && t - zeroSince > 1.0f;
    if (state == 0) {
      msg = obdMode ? "STOP TO ARM (OBD)" : "STOP TO ARM";
      if (stopped && vNow < 0.3f) { state = 1; riseAt = t; uRise = 0; dRise = 0; trigSince = -1; departed = false; }
    }
    if (state == 1) {
      msg = "READY";
      if (imuMode) {
        if (accF < 0.04f) {                              // still: hold the estimate at 0 and learn the rest reading
          if (fabsf(accF) < 0.04f) bias += (lonG - bias) * (1 - expf(-dt / 0.7f));
          u = 0; off = 0; dist = 0; riseAt = t; uRise = 0; dRise = 0;
        }
      } else if (vNow < 0.05f) { dist = 0; riseAt = t; departed = false; }
      else if (!departed) {                              // OBD only: the car started between the last 0 and this reading
        departed = true;
        riseAt = 0.5f * (obdPrevT + obdT) + lag;         // (on the same delayed timeline as the speed)
        if (riseAt > t) riseAt = t;
      }
      if (accF >= 0.15f) { if (trigSince < 0) trigSince = t; } else trigSince = -1;
      bool launch = (trigSince >= 0 && t - trigSince >= 0.08f) || vNow > 1.5f;
      if (!obdMode && launch && t - riseAt > 0.8f) riseAt = t - 0.8f;   // a slow rise: no earlier than 0.8 s back
      if (launch) {
        state = 2; t0 = riseAt; tStart = rollout ? -1 : t0; dStart = 0; sDone060 = false; sPsi = psi; sNoAcc = -1;
      } else if (obdV > 1.0f) state = 0;                 // it moved off without a launch (creeping)
    }
    if (state == 2) {
      msg = "";
      if (has(psi) && (!has(sPsi) || psi > sPsi)) sPsi = psi;
      if (tStart < 0 && dist >= ROLLOUT) {               // rollout: the clock starts after the first foot
        float f = (ROLLOUT - (dist - 0.5f * (vPrev + vNow) * dt)) / (0.5f * (vPrev + vNow) * dt + 1e-6f);
        tStart = t - dt + dt * (f < 0 ? 0 : f > 1 ? 1 : f); dStart = ROLLOUT;
      }
      float el = t - (tStart >= 0 ? tStart : t0);
      if (!sDone060) liveT060 = el;
      liveTQm = el;
      if (!sDone060 && tStart >= 0 && vPrev < V60 && vNow >= V60) {
        float tc = t - dt + dt * (V60 - vPrev) / (vNow - vPrev);
        float r = tc - tStart;
        if (r > 1.8f && r < 25) push(K060, r, V60, sPsi, t);
        sDone060 = true; liveT060 = 0;
      }
      bool end = false;
      if (tStart >= 0 && dist - dStart >= QUARTER) {
        float over = dist - dStart - QUARTER, stepD = 0.5f * (vPrev + vNow) * dt + 1e-6f;
        float tc = t - dt * (over / stepD);
        float r = tc - tStart;
        if (r > 7 && r < 40) push(KQM, r, vNow, sPsi, t);
        end = true;
      }
      if (accF < 0.0f) { if (sNoAcc < 0) sNoAcc = t; } else if (accF > 0.04f) sNoAcc = -1;
      bool coast = sNoAcc >= 0 && t - sNoAcc > (obdMode ? 1.5f : 1.0f);
      if (lifted(t0) || coast || (t - t0 > 3 && vNow < 4.5f) || t - t0 > 45) end = true;
      if (end) { state = 0; liveT060 = liveTQm = 0; }
    }

    // ---- rolling 60-130
    if (vNow < V58) rArmed = true;
    if (!rRun && rArmed && vPrev < V60 && vNow >= V60 && accF > 0.1f && !(has(pedal) && pedal < 60)) {
      rRun = true; rArmed = false; r0 = t - dt + dt * (V60 - vPrev) / (vNow - vPrev); rPsi = psi; rNoAcc = -1;
    }
    if (rRun) {
      if (has(psi) && (!has(rPsi) || psi > rPsi)) rPsi = psi;
      liveT6130 = t - r0;
      bool end = false;
      if (vPrev < V130 && vNow >= V130) {
        float r = t - dt + dt * (V130 - vPrev) / (vNow - vPrev) - r0;
        if (r > 4 && r < 40) push(K60130, r, V130, rPsi, t);
        end = true;
      }
      if (accF < 0.02f) { if (rNoAcc < 0) rNoAcc = t; } else if (accF > 0.05f) rNoAcc = -1;
      bool coast = rNoAcc >= 0 && t - rNoAcc > (obdMode ? 1.5f : 1.2f);
      if (lifted(r0) || coast || vNow < V57 || t - r0 > 45) end = true;
      if (end) { rRun = false; liveT6130 = 0; }
    }
    if (rRun && state != 2) msg = "";
  }

  int shownState() const { return (state == 2 || rRun) ? 2 : state; }
};

}  // namespace perf
