// Checks firmware/S5GaugePod_Test/imu_cal.h with a simulated drive: a pod mounted at an odd
// angle in the vent, engine idle vibration, gyro offset, OBD speed at 5 Hz in whole km/h with
// a little lag. The car-frame g it reports must match what the simulated car is doing.
// build: g++ -O2 -std=c++17 tools/imu_test.cpp -o imu_test
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../firmware/S5GaugePod_Test/imu_cal.h"
using namespace imu;

static int fails = 0;
static void check(bool ok, const char *what, float got, float want) {
  printf("  %-44s %7.3f (want %6.3f)  %s\n", what, got, want, ok ? "ok" : "FAIL");
  if (!ok) fails++;
}
static float rnd() { return (float)rand() / RAND_MAX * 2 - 1; }

// rotation of a vector about a unit axis
static V3 rot(V3 v, V3 k, float deg) {
  float a = deg * 3.14159265f / 180, c = cosf(a), s = sinf(a);
  return v * c + cross(k, v) * s + k * (dot(k, v) * (1 - c));
}

// Pod mounting. Sensor x = car left, y = car up, z = car forward (into the dash) when the pod
// sits square; then tilted/turned the way a vent pod might be. flip = mounted so +z faces the
// driver (the assumed axis is then backwards).
struct Mount {
  V3 ex, ey, ez;                                        // sensor axes in car coordinates
  Mount(float tiltDeg, float turnDeg, float rollDeg, bool flip) {
    ex = v3(0, 1, 0); ey = v3(0, 0, 1); ez = v3(1, 0, 0);
    if (flip) { ex = ex * -1; ez = ez * -1; }
    V3 axes[3] = {ex, ey, ez};
    for (V3 &a : axes) { a = rot(a, v3(0, 1, 0), tiltDeg); a = rot(a, v3(0, 0, 1), turnDeg); a = rot(a, v3(1, 0, 0), rollDeg); }
    ex = axes[0]; ey = axes[1]; ez = axes[2];
  }
  V3 toSensor(V3 c) const { return v3(dot(c, ex), dot(c, ey), dot(c, ez)); }
};

struct Sim {
  Proc p; Mount m; V3 bias = v3(1.7f, -2.4f, 0.9f);
  float t = 0, v = 0;                                   // car speed m/s
  float obdKmh = 0, obdT = -1; bool obd = true;
  float lastObd = -1;
  Sim(const Mount &mm, const Cal &saved = Cal(), bool withObd = true) : m(mm), obd(withObd) { p.begin(saved, v3(0, 0, 1)); }
  // lon/lat in g (lat + = to the right), yaw in deg/s (+ = turning left), for dur seconds
  void drive(float lon, float lat, float yaw, float dur) {
    for (float e = 0; e < dur; e += 0.01f) {
      v += lon * 9.81f * 0.01f; if (v < 0) v = 0;
      t += 0.01f;
      V3 f = v3(lon, -lat, 1.0f);                       // specific force in car coords (x fwd, y left, z up)
      V3 a = m.toSensor(f) + v3(rnd(), rnd(), rnd()) * 0.02f;          // idle / road vibration
      V3 w = m.toSensor(v3(0, 0, yaw)) + bias + v3(rnd(), rnd(), rnd()) * 0.4f;
      if (obd && t - lastObd >= 0.2f) { lastObd = t; obdKmh = floorf(v * 3.6f + 0.5f); obdT = t; }   // 5 Hz, whole km/h
      p.step(a, w, 0.01f, t, obd ? obdKmh : -1, obdT);
      if (p.events) { printf("  [%5.2f s] %s\n", t, p.lastEvent); p.events = 0; }
    }
  }
};

int main() {
  srand(1);
  printf("1. tilted 25 deg, turned 15 deg toward the driver, rolled 8 deg; OBD speed\n");
  Sim s(Mount(-25, 15, 8, false));
  s.drive(0, 0, 0, 3);
  check(s.p.state() == READY, "ready after a 3 s stop (assumed forward)", (float)s.p.state(), 2);
  check(fabsf(s.p.lon) < 0.03f && fabsf(s.p.lat) < 0.03f, "at rest: |lon| + |lat| small", fabsf(s.p.lon) + fabsf(s.p.lat), 0);
  s.drive(0.35f, 0, 0, 5);                              // straight pull
  check(s.p.cal.fwdSrc == 2, "forward learned after one pull", (float)s.p.cal.fwdSrc, 2);
  check(fabsf(s.p.lon - 0.35f) < 0.03f, "pull: lon", s.p.lon, 0.35f);
  check(fabsf(s.p.lat) < 0.03f, "pull: lat", s.p.lat, 0);
  s.drive(0, 0.6f, -18, 4);                             // steady right-hand bend
  check(fabsf(s.p.lat - 0.6f) < 0.03f, "right bend: lat", s.p.lat, 0.6f);
  check(fabsf(s.p.lon) < 0.03f, "right bend: lon", s.p.lon, 0);
  s.drive(-0.8f, 0, 0, 1.5f);                           // hard braking
  check(fabsf(s.p.lon + 0.8f) < 0.03f, "braking: lon", s.p.lon, -0.8f);
  s.drive(0, -0.5f, 15, 3);                             // left bend
  check(fabsf(s.p.lat + 0.5f) < 0.03f, "left bend: lat", s.p.lat, -0.5f);
  check(fabsf(s.p.pkR - 0.6f) < 0.04f && fabsf(s.p.pkL - 0.5f) < 0.04f, "peaks right / left", s.p.pkR + s.p.pkL, 1.1f);
  check(fabsf(s.p.pkBrk - 0.8f) < 0.04f && fabsf(s.p.pkAcc - 0.35f) < 0.04f, "peaks braking / accel", s.p.pkBrk + s.p.pkAcc, 1.15f);
  Cal saved = s.p.cal;

  printf("2. mounted the other way round: the assumed forward is backwards until the first pull\n");
  Sim b(Mount(-20, -10, 0, true));
  b.drive(0, 0, 0, 3);
  b.drive(0.3f, 0, 0, 2);
  check(b.p.lon < 0, "before learning: pull reads as braking", b.p.lon, -0.3f);
  b.drive(0.3f, 0, 0, 5);
  check(fabsf(b.p.lon - 0.3f) < 0.03f, "after learning: lon", b.p.lon, 0.3f);

  printf("3. next drive, calibration from flash, same mounting: ready at once\n");
  Sim c(Mount(-25, 15, 8, false), saved);
  check(c.p.state() == READY && c.p.cal.fwdSrc == 2, "ready at power-on with learned forward", (float)c.p.state(), 2);
  c.drive(0, 0, 0, 0.5f);
  c.drive(0.4f, 0, 0, 2);
  check(fabsf(c.p.lon - 0.4f) < 0.03f, "pull straight away: lon", c.p.lon, 0.4f);

  printf("4. pod moved in the vent between drives (tilt 25 -> 5 deg): relearned at the first stop\n");
  Sim d(Mount(-5, 15, 8, false), saved);
  d.drive(0, 0, 0, 3);
  check(fabsf(d.p.lon) < 0.03f && fabsf(d.p.lat) < 0.03f, "at rest after relearning: |lon| + |lat|", fabsf(d.p.lon) + fabsf(d.p.lat), 0);
  d.drive(0.3f, 0, 0, 6);
  check(fabsf(d.p.lon - 0.3f) < 0.03f, "pull: lon", d.p.lon, 0.3f);

  printf("5. no OBD (bench): level from stillness, assumed forward\n");
  Sim e(Mount(-25, 15, 8, false), Cal(), false);
  e.drive(0, 0, 0, 3);
  check(e.p.state() == READY && e.p.cal.fwdSrc == 1, "ready with assumed forward", (float)e.p.cal.fwdSrc, 1);
  e.drive(0.25f, 0, 0, 1);
  check(fabsf(e.p.lon - 0.25f) < 0.04f, "pull: lon (assumed forward is close)", e.p.lon, 0.25f);

  printf(fails ? "\n%d FAILED\n" : "\nall G-meter tests passed\n", fails);
  return fails != 0;
}
