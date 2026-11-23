// =====================================================================
//  S5 Gauge Pod - G-meter maths: mounting calibration and car-frame g
//
//  The motion sensor sits in the pod at whatever angle the vent holds it, so its axes mean
//  nothing on their own. Two directions are learned, both in the sensor's own axes:
//
//    up       Whenever the car sits still for 2 s (OBD speed 0 if known, gyro quiet,
//             accelerometer steady) the average accelerometer reading is "up" (a resting
//             accelerometer reads +1 g away from the ground). Each stop is blended into a
//             running average, so a parking slope doesn't skew it; a stop that disagrees by
//             more than 6 degrees means the pod was moved, and replaces it outright.
//    forward  While OBD speed is changing (accelerating or braking at 0.12 g or more) and the
//             car isn't turning, the horizontal part of the accelerometer reading points
//             forward (or backward when braking). Averaged over the first few pulls.
//             Until then an assumed forward is used: the sensor axis that points into the
//             dash when the pod faces the driver (config.h IMU_FWD_AXIS).
//
//  Car frame: lon = acceleration forward (+) / braking (-), lat = acceleration to the right (+).
//  Both are low-passed at about 4 Hz for the display; peaks come from the smoothed values.
//
//  Plain C++ (no Arduino calls), tested on the desktop by tools/imu_test.cpp.
// =====================================================================
#pragma once
#include <math.h>

namespace imu {

struct V3 { float x = 0, y = 0, z = 0; };
static inline V3 v3(float x, float y, float z) { V3 r; r.x = x; r.y = y; r.z = z; return r; }
static inline V3 operator+(V3 a, V3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static inline V3 operator-(V3 a, V3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static inline V3 operator*(V3 a, float k) { return v3(a.x * k, a.y * k, a.z * k); }
static inline float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline V3 cross(V3 a, V3 b) { return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }
static inline float norm(V3 a) { return sqrtf(dot(a, a)); }
static inline V3 unit(V3 a) { float n = norm(a); return n > 1e-6f ? a * (1.0f / n) : v3(0, 0, 0); }
static inline V3 horiz(V3 a, V3 up) { return a - up * dot(a, up); }      // part of a at right angles to up

// What's kept in flash between drives
struct Cal {
  V3    up;  bool upOk = false; int upN = 0;          // up, and how many stops it averages
  V3    fwd; int fwdSrc = 0;                          // 0 none, 1 assumed, 2 learned
  float fwdW = 0;                                     // learned evidence so far (g * s)
  V3    gyroBias;
};

enum State { NEED_LEVEL = 0, NEED_FORWARD = 1, READY = 2 };

struct Proc {
  Cal   cal;
  V3    assumedFwd;                                   // sensor axis into the dash (unit)
  // outputs (read by the display)
  float lat = 0, lon = 0;                             // smoothed, g
  float lonRaw = 0;                                   // forward, unsmoothed (performance timers), g
  float pkL = 0, pkR = 0, pkAcc = 0, pkBrk = 0;       // peak magnitudes, g
  bool  dirty = false;                                // calibration changed: save it
  int   events = 0;                                   // count of calibration events (for the log)
  const char *lastEvent = "";

  // stillness tracking
  V3    aEma, wEma, sumA, sumW;
  int   stillN = 0;
  float stillT = 0, maxDev = 0;
  bool  committed = false;
  // forward learning
  V3    aSlow;                                        // ~1 s average, to line up with the OBD speed slope
  V3    fwdSum;
  float spdHist[16]; float spdT[16]; int spdN = 0;    // OBD speed samples (km/h, s)
  float lastKmhT = -1;
  bool  started = false;
  float lastT = 0, refineAt = -1e9f;                  // time of the current sample / last logged refinement

  void begin(const Cal &saved, V3 assumed) {
    cal = saved;
    assumedFwd = unit(assumed);
    if (cal.fwdSrc == 2) fwdSum = horiz(cal.fwd, cal.up) * cal.fwdW;
    if (cal.fwdSrc != 2 && cal.upOk) useAssumed();
  }
  int state() const {
    if (!cal.upOk) return NEED_LEVEL;
    return cal.fwdSrc ? READY : NEED_FORWARD;
  }
  void resetPeaks() { pkL = pkR = pkAcc = pkBrk = 0; }
  void relevel() { cal.upOk = false; cal.upN = 0; committed = false; stillT = 0; stillN = 0; }
  void forget() {
    Cal c; c.gyroBias = cal.gyroBias; cal = c;
    fwdSum = v3(0, 0, 0); committed = false; stillT = 0; stillN = 0; dirty = true;
    note("calibration cleared");
  }

  void note(const char *e) { lastEvent = e; events++; }
  void useAssumed() {
    V3 h = horiz(assumedFwd, cal.up);
    if (norm(h) > 0.3f) { cal.fwd = unit(h); cal.fwdSrc = 1; }   // only if that axis isn't near vertical
    else cal.fwdSrc = 0;
  }

  // One sample. a = accelerometer (g), w = gyro (deg/s), dt in s, t = time in s.
  // kmh < 0 = no OBD speed; kmhT = time of that speed reading (s) - new readings are picked
  // up when kmhT changes.
  void step(V3 a, V3 w, float dt, float t, float kmh, float kmhT) {
    lastT = t;
    if (!started) { aEma = a; aSlow = a; wEma = w; started = true; }
    const float kEma = 1 - expf(-dt / 0.5f), kSlow = 1 - expf(-dt / 1.0f);
    aEma = aEma + (a - aEma) * kEma;
    wEma = wEma + (w - wEma) * kEma;
    aSlow = aSlow + (a - aSlow) * kSlow;
    V3 wc = w - cal.gyroBias;

    // ---- stillness -> up (and the gyro's zero). The gyro's zero isn't known before the first
    //      stop, so "not rotating" means steady (close to its own average), and close to zero
    //      once the zero has been learned.
    float dev = norm(a - aEma), wDev = norm(w - wEma), rot = norm(wc);
    bool moving = (kmh >= 0 && kmh > 0.5f) || wDev > 2.0f || (cal.upOk && rot > 4.0f) || dev > 0.06f;
    if (moving) { stillT = 0; stillN = 0; maxDev = 0; committed = false; sumA = sumW = v3(0, 0, 0); }
    else {
      stillT += dt; stillN++;
      sumA = sumA + a; sumW = sumW + w;
      if (dev > maxDev) maxDev = dev;
      if (stillT >= 2.0f && !committed) { commitStill(); committed = true; }
    }

    // ---- OBD speed slope -> forward
    if (kmh >= 0 && kmhT != lastKmhT) {
      lastKmhT = kmhT;
      if (spdN == 16) { for (int i = 0; i < 15; i++) { spdHist[i] = spdHist[i + 1]; spdT[i] = spdT[i + 1]; } spdN--; }
      spdHist[spdN] = kmh; spdT[spdN] = kmhT; spdN++;
    }
    if (cal.upOk && kmh > 5 && spdN >= 2) learnForward(wc, dt, t);

    // ---- car frame
    if (state() != READY) { lat = lon = lonRaw = 0; return; }
    V3 fw = cal.fwd, right = cross(cal.fwd, cal.up);
    float lo = dot(a, fw), la = dot(a, right);
    lonRaw = lo;
    const float k = 1 - expf(-dt * 2 * 3.14159265f * 4.0f);   // ~4 Hz
    lon += (lo - lon) * k;
    lat += (la - lat) * k;
    if (lon > pkAcc) pkAcc = lon;
    if (-lon > pkBrk) pkBrk = -lon;
    if (lat > pkR) pkR = lat;
    if (-lat > pkL) pkL = -lat;
  }

  void commitStill() {
    V3 upNew = unit(sumA * (1.0f / stillN));
    V3 bias = sumW * (1.0f / stillN);
    cal.gyroBias = cal.upOk ? cal.gyroBias + (bias - cal.gyroBias) * 0.3f : bias;
    if (!cal.upOk) {
      cal.up = upNew; cal.upOk = true; cal.upN = 1;
      note("level learned");
      if (cal.fwdSrc != 2) useAssumed();
      else cal.fwd = unit(horiz(cal.fwd, cal.up));
    } else if (dot(upNew, cal.up) < cosf(6.0f * 3.14159265f / 180.0f)) {
      cal.up = upNew; cal.upN = 1;                       // the pod was moved: start over
      fwdSum = v3(0, 0, 0); cal.fwdW = 0;
      useAssumed();
      note("pod moved: level relearned, forward reset");
    } else {
      int n = cal.upN < 20 ? cal.upN : 20;
      cal.up = unit(cal.up * (float)n + upNew);
      cal.upN = cal.upN + 1;
      cal.fwd = cal.fwdSrc ? unit(horiz(cal.fwd, cal.up)) : cal.fwd;
      // every stop refines it (and on the bench, every time the pod is put down), so it's only
      // logged and saved now and then
      if (lastT - refineAt < 300) return;
      refineAt = lastT;
      note("level refined");
    }
    dirty = true;
  }

  void learnForward(V3 wc, float dt, float t) {
    // speed slope over the last ~1 s of OBD readings, in g
    int j = spdN - 1, i = j;
    while (i > 0 && spdT[j] - spdT[i - 1] < 1.0f) i--;
    if (i == j || t - spdT[j] > 0.6f) return;           // need fresh readings spanning some time
    float span = spdT[j] - spdT[i];
    if (span < 0.4f) return;
    float slope = (spdHist[j] - spdHist[i]) / 3.6f / span / 9.81f;
    if (fabsf(slope) < 0.12f) return;
    float yaw = fabsf(dot(wc, cal.up));
    if (yaw > 3.0f) return;                              // turning
    V3 h = horiz(aSlow, cal.up);
    float hn = norm(h);
    if (hn < 0.6f * fabsf(slope) || hn > 1.6f * fabsf(slope) + 0.05f) return;   // doesn't match the speed change
    fwdSum = fwdSum + h * ((slope > 0 ? 1.0f : -1.0f) * dt);
    cal.fwdW += fabsf(slope) * dt;
    if (cal.fwdW >= 0.8f) {                              // e.g. 4 s at 0.2 g
      V3 f = unit(horiz(fwdSum, cal.up));
      if (norm(f) < 0.5f) return;
      bool first = cal.fwdSrc != 2;
      bool flipped = cal.fwdSrc == 1 && dot(f, cal.fwd) < 0;
      cal.fwd = f; cal.fwdSrc = 2;
      if (cal.fwdW > 6.0f) { fwdSum = fwdSum * (6.0f / cal.fwdW); cal.fwdW = 6.0f; }   // keep adapting slowly
      if (first) { note(flipped ? "forward learned (the assumed axis was backwards)" : "forward learned"); dirty = true; }
      else {
        static float lastSave = -1e9f;                   // refine quietly, save at most once a minute
        if (t - lastSave > 60) { lastSave = t; dirty = true; }
      }
    }
  }
};

}  // namespace imu
