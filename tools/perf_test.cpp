// Desktop test for firmware/S5GaugePod_Test/perf.h (performance timers).
// A simulated S5 drives runs; the G-meter and OBD speed are made like the real ones (noise,
// whole km/h, irregular and late replies), and the timer's results are checked against the
// simulation's true times.
// build: g++ -std=c++17 -O2 -Ifirmware/S5GaugePod_Test tools/perf_test.cpp -o perf_test && ./perf_test
#include <cstdio>
#include <cmath>
#include <random>
#include <vector>
#include <functional>
#include "perf.h"

static int fails = 0;
static void check(bool ok, const char *what) { printf("  %-66s %s\n", what, ok ? "ok" : "FAIL"); if (!ok) fails++; }

struct Sim {
  // car
  float m = 1900, Pw = 280000, trac = 0.80f * 9.80665f;
  float v = 0, x = 0, t = 0;
  // drive script: pedal % as a function of time; brake (m/s^2) as a function of time
  std::function<float(float)> pedal = [](float) { return 0.f; };
  std::function<float(float)> brake = [](float) { return 0.f; };
  float launchAt = 1e9f, cruiseKmh = -1;                 // cruise holds this speed (pedal ignored) until launchAt
  float shiftUntil = -1; int gear = 1;
  float grade = 0;                                       // road slope as g (uphill +)
  float trueLag = 0.12f;
  bool  imu = true;
  float accScale = 1.0f;                                 // G-meter gain error (pod a little crooked, sensor tolerance)
  // sensor noise
  std::mt19937 rng{7};
  std::normal_distribution<float> n01{0, 1};
  // truth
  std::vector<float> tv, xv, vv;

  float accel(float dt) {
    float p = pedal(t) / 100.0f, a;
    if (cruiseKmh >= 0 && t < launchAt) { a = (cruiseKmh / 3.6f - v) * 2.0f; }
    else {
      float drive = p <= 0 ? 0 : fminf(trac * fminf(1.0f, (t - launchAt) / 0.25f + (v > 1 ? 1 : 0)), p * Pw / (m * fmaxf(v, 3.0f)));
      if (t - launchAt < 0) drive = p * 2.0f;
      // shifts (S tronic): a short dip
      static const float up[] = {18.0f, 29.0f, 41.0f, 54.0f, 66.0f};   // m/s ~ 40, 65, 92, 120, 148 mph
      if (gear <= 5 && v > up[gear - 1]) { gear++; shiftUntil = t + 0.12f; }
      if (t < shiftUntil) drive *= 0.35f;
      a = drive;
    }
    a -= 0.5f * 1.2f * 0.65f * v * v / m + 0.012f * 9.80665f + grade * 9.80665f * (v > 0.01f || a > grade * 9.8f ? 1 : 0);
    a -= brake(t);
    if (v <= 0 && a < 0) a = 0;                          // parked (brake / slope held)
    (void)dt;
    return a;
  }
};

struct Out { std::vector<perf::Result> r; float tru060 = -1, truQm = -1, truQmMph = 0, tru6130 = -1, truRoll060 = -1; };

// Run a scenario for `dur` seconds. Returns results and true times.
static Out run(Sim &s, perf::Timer &tm, float dur, float vibG = 0.02f) {
  Out o;
  const float dt = 0.001f;
  float accNow = 0;
  // OBD: a reply every 80-200 ms, showing the speed trueLag earlier, in whole km/h
  float nextObd = 0.05f, obdKmh = perf::NOV, obdAt = -1;
  float nextPedal = 0.0f, pedalVal = perf::NOV;
  std::uniform_real_distribution<float> gap(0.08f, 0.20f);
  float nextTimer = 0;
  float tStart = -1, xStart = 0, t60up = -1;             // truth: launch, 60 mph
  bool moving = false;
  float roll60 = -1;
  for (int i = 0; s.t < dur; i++) {
    accNow = s.accel(dt);
    float v0 = s.v;
    s.v += accNow * dt; if (s.v < 0) s.v = 0;
    s.x += 0.5f * (v0 + s.v) * dt;
    s.t += dt;
    if (i % 5 == 0) { s.tv.push_back(s.t); s.vv.push_back(s.v); s.xv.push_back(s.x); }
    // truth
    if (!moving && s.v > 0.001f && s.t >= s.launchAt - 0.01f && s.cruiseKmh < 0) { moving = true; tStart = s.t - dt; xStart = s.x - 0.5f * (v0 + s.v) * dt; }
    if (moving && tStart >= 0 && v0 < perf::V60 && s.v >= perf::V60 && o.tru060 < 0) o.tru060 = s.t - tStart;
    if (moving && tStart >= 0 && o.truQm < 0 && s.x - xStart >= perf::QUARTER) { o.truQm = s.t - tStart; o.truQmMph = s.v / perf::MPH; }
    if (v0 < perf::V60 && s.v >= perf::V60) roll60 = s.t;
    if (roll60 > 0 && v0 < perf::V130 && s.v >= perf::V130 && o.tru6130 < 0) o.tru6130 = s.t - roll60;
    (void)t60up;
    // sensors
    if (s.t >= nextObd) {
      float tq = s.t - s.trueLag; float vq = 0;
      for (size_t k = s.tv.size(); k-- > 0;) if (s.tv[k] <= tq) { vq = s.vv[k]; break; }
      obdKmh = roundf(vq * 3.6f); obdAt = s.t; nextObd = s.t + gap(s.rng);
    }
    if (s.t >= nextPedal) { pedalVal = s.pedal(s.t - 0.05f); nextPedal = s.t + 0.25f; }
    if (s.t >= nextTimer) {
      nextTimer += 0.01f;
      // accelerometer: true acceleration + slope + squat (nose up under power) + vibration + noise
      float lon = s.accScale * accNow / 9.80665f + s.grade + 0.015f * accNow / 9.80665f + vibG * sinf(2 * 3.14159f * 31 * s.t) + 0.025f * s.n01(s.rng);
      tm.step(s.t, s.imu ? lon : perf::NOV, obdKmh, obdAt, pedalVal, 5.0f + s.v / 10);
      perf::Result rr[6]; int n = tm.take(rr, 6);
      for (int k = 0; k < n; k++) o.r.push_back(rr[k]);
    }
  }
  return o;
}

static const perf::Result *find(const Out &o, int kind) { for (auto &r : o.r) if (r.kind == kind) return &r; return nullptr; }

static void show(const Out &o) {
  for (auto &r : o.r) printf("    result %-8s %6.2f s  %5.1f mph  %s%s\n", perf::kindName(r.kind), r.t, r.mph, r.obdOnly ? "OBD only " : "", r.rollout ? "rollout" : "");
  printf("    truth  0-60 %.2f  1/4 %.2f @ %.1f  60-130 %.2f\n", o.tru060, o.truQm, o.truQmMph, o.tru6130);
}

static void standing(const char *name, float lagTrue, float grade, bool imu, float tol060, float tolQm, float tol6130, bool rollout = false, float scale = 1.0f) {
  printf("%s\n", name);
  Sim s; s.trueLag = lagTrue; s.grade = grade; s.imu = imu; s.launchAt = 4.0f; s.accScale = scale;
  s.pedal = [](float t) { return t >= 4.0f ? 100.f : 0.f; };
  perf::Timer tm; tm.rollout = rollout;
  Out o = run(s, tm, 25);
  show(o);
  const perf::Result *a = find(o, perf::K060), *q = find(o, perf::KQM), *r = find(o, perf::K60130);
  float roll = 0;
  if (rollout) {                                         // the true rollout time: when the car has moved a foot
    for (size_t k = 0; k < s.xv.size(); k++) if (s.xv[k] >= perf::ROLLOUT) { roll = s.tv[k] - 4.0f; break; }
    printf("    true rollout %.2f s\n", roll);
  }
  char b[96];
  snprintf(b, sizeof b, "0-60 within %.2f s", tol060);    check(a && fabsf(a->t - (o.tru060 - roll)) <= tol060, b);
  snprintf(b, sizeof b, "1/4 mile within %.2f s", tolQm); check(q && fabsf(q->t - (o.truQm - roll)) <= tolQm + (rollout ? 0.05f : 0), b);
  check(q && fabsf(q->mph - o.truQmMph) < 2.0f, "trap speed within 2 mph");
  snprintf(b, sizeof b, "60-130 within %.2f s", tol6130); check(r && fabsf(r->t - o.tru6130) <= tol6130, b);
  check(a && a->obdOnly == !imu, "marked OBD only exactly when there's no G-meter");
}

int main() {
  standing("standing run, G-meter, OBD lag as assumed",     0.12f, 0, true, 0.05f, 0.08f, 0.10f);
  standing("standing run, OBD later than assumed (0.20 s)", 0.20f, 0, true, 0.12f, 0.15f, 0.15f);
  standing("standing run, OBD sooner than assumed (0.06 s)", 0.06f, 0, true, 0.12f, 0.12f, 0.15f);
  standing("standing run uphill (4 % grade)",               0.12f, 0.04f, true, 0.08f, 0.12f, 0.15f);
  standing("standing run, OBD only (no G-meter)",           0.12f, 0, false, 0.25f, 0.30f, 0.30f);
  standing("standing run, G-meter reads 5 % low (pod 18 deg crooked)", 0.12f, 0, true, 0.08f, 0.12f, 0.15f, false, 0.95f);
  standing("standing run with 1 ft rollout",                0.12f, 0, true, 0.06f, 0.10f, 0.10f, true);

  {
    printf("lift at 45 mph: no 0-60, no 1/4 mile\n");
    Sim s; s.launchAt = 4.0f;
    s.pedal = [](float t) { return t >= 4.0f && t < 7.0f ? 100.f : 0.f; };
    perf::Timer tm; Out o = run(s, tm, 15); show(o);
    check(!find(o, perf::K060) && !find(o, perf::KQM), "no results");
  }
  {
    printf("creep away from a stop, then stop again\n");
    Sim s; s.launchAt = 3.0f;
    s.pedal = [](float t) { return t >= 3.0f && t < 5.0f ? 4.f : 0.f; };     // creep
    s.brake = [](float t) { return t > 6.0f ? 2.0f : 0.f; };
    perf::Timer tm; Out o = run(s, tm, 10); show(o);
    check(o.r.empty(), "no results");
  }
  {
    printf("normal drive away (pedal 30 %%), all the way to 70 mph\n");
    Sim s; s.launchAt = 3.0f;
    s.pedal = [](float t) { return t >= 3.0f ? 30.f : 0.f; };
    perf::Timer tm; Out o = run(s, tm, 30); show(o);
    check(o.r.empty(), "no results");
  }
  {
    printf("idling at a light with engine shake (0.08 g at 31 Hz), 20 s\n");
    Sim s; perf::Timer tm; Out o = run(s, tm, 20, 0.08f);
    check(o.r.empty() && tm.state == 1, "no false start, still armed");
  }
  {
    printf("rolling: cruise at 45 mph, then floor it to 135\n");
    Sim s; s.cruiseKmh = 72; s.v = 20; s.launchAt = 6.0f;
    s.pedal = [](float t) { return t >= 6.0f ? 100.f : 15.f; };
    perf::Timer tm; Out o = run(s, tm, 30); show(o);
    const perf::Result *r = find(o, perf::K60130);
    check(!find(o, perf::K060) && !find(o, perf::KQM), "no 0-60 or 1/4 mile from a rolling start");
    check(r && fabsf(r->t - o.tru6130) < 0.12f, "60-130 within 0.12 s");
  }
  {
    printf("rolling, lift at 100 mph: no 60-130\n");
    Sim s; s.cruiseKmh = 72; s.v = 20; s.launchAt = 6.0f;
    s.pedal = [](float t) { return t >= 6.0f && t < 11.5f ? 100.f : 15.f; };
    perf::Timer tm; Out o = run(s, tm, 25); show(o);
    check(!find(o, perf::K60130), "no 60-130");
  }
  {
    printf("no OBD speed\n");
    perf::Timer tm;
    for (int i = 0; i < 300; i++) tm.step(i * 0.01f, 0.0f, perf::NOV, -1, perf::NOV, 0);
    check(tm.state == 0 && tm.msg[0] == 'N', "not armed, says why");
  }
  {
    printf("two runs: best kept\n");
    Sim s; s.launchAt = 4.0f; s.Pw = 240000;
    s.pedal = [](float t) { return (t >= 4.0f && t < 10.0f) || t >= 26.0f ? 100.f : 0.f; };
    s.brake = [](float t) { return t >= 10.0f && t < 20.0f ? 6.0f : 0.f; };
    perf::Timer tm; Out o = run(s, tm, 18); s.Pw = 300000; s.launchAt = 26.0f; Out o2 = run(s, tm, 40);
    int n060 = 0; for (auto &r : o.r) n060 += r.kind == perf::K060; for (auto &r : o2.r) n060 += r.kind == perf::K060;
    printf("    best 0-60 %.2f, last %.2f\n", tm.b060, tm.t060);
    check(n060 == 2 && tm.b060 <= tm.t060 + 1e-4f && tm.b060 > 0, "two 0-60 runs, best is the quicker");
  }
  if (fails) { printf("%d FAILED\n", fails); return 1; }
  printf("all perf timer tests passed\n");
  return 0;
}
