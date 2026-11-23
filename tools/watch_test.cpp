// Desktop test for firmware/S5GaugePod_Test/watch.h: simulated pulls and warnings.
// build: g++ -std=c++17 -O1 tools/watch_test.cpp -o watch_test && ./watch_test
#include <stdio.h>
#include <math.h>
#include "../firmware/S5GaugePod_Test/watch.h"
using namespace watch;

static int fails = 0;
static void expect(bool ok, const char *what) { printf("  %-62s %s\n", what, ok ? "ok" : "FAIL"); if (!ok) fails++; }
static Watch w;
static float T = 0;
static In base() { In in; in.t = T; in.runSec = 300; in.rpm = 800; in.kmh = 40; in.pedal = 15; in.psi = -9; in.gear = 4;
                   in.iat = 105; in.clt = 200; in.oil = 210; in.bat = 13.8f; in.timing = 20; in.afr1 = in.afr2 = 14.7f; in.cat1 = in.cat2 = 1100; return in; }
static void idle(float sec) { for (float e = 0; e < sec; e += 0.05f) { In in = base(); w.step(in); T += 0.05f; } }
// a 2nd-to-3rd gear pull from 3000 rpm; boost rises with rpm. dropPsi = boost below the usual;
// slip = boost falls off in 3rd while rpm keeps climbing
static void pull(float dropPsi = 0, bool slip = false, float afr = 11.8f, float durS = 6) {
  float rpm = 3000; int gear = 2;
  for (float e = 0; e < durS; e += 0.05f) {
    In in = base();
    rpm += (gear == 2 ? 900 : 600) * 0.05f;
    if (gear == 2 && rpm > 6800) { gear = 3; rpm = 4700; }
    float psi = fminf(15, 6 + 1.6f * (rpm - 3000) / 500) - dropPsi;
    if (slip && gear == 3 && rpm > 5400) psi -= (rpm - 5400) / 250;
    in.rpm = rpm; in.gear = gear; in.pedal = 100; in.psi = psi; in.obdPsi = psi - 0.2f; in.tgtPsi = 17.8f;
    in.iat = 105 + e * 2; in.timing = 15; in.knock = 0; in.hpfp = 2100; in.hpfpSet = 2175; in.lpfp = 75; in.idc = 70;
    in.afr1 = afr; in.afr2 = afr + 0.1f; in.cat1 = 1300 + e * 20; in.kmh = 40 + e * 12;
    w.step(in); T += 0.05f;
  }
  idle(1.0f);
}

int main() {
  w.begin(nullptr);
  printf("1. three normal pulls teach the car's usual boost\n");
  idle(5);
  for (int i = 0; i < 3; i++) { pull(); idle(5); }
  expect(w.pullsDone == 3, "three pulls detected");
  expect(w.last.valid && fabsf(w.last.peakPsi - 15) < 0.3f, "peak boost 15 psi");
  printf("     gears \"%s\"\n", w.last.gears);
  expect(strcmp(w.last.gears, "2-3") == 0, "gears 2-3");
  expect(fabsf(w.last.iatMax - w.last.iat0 - 11.9f) < 0.3f, "intake rise ~12 F");
  expect(fabsf(w.last.hpfpDef + 75) < 1 && fabsf(w.last.afrMax - 11.9f) < 0.05f, "fuel pump -75 psi vs target, leanest AFR 11.9");
  expect(!has(w.last.vsNormal) || w.last.normalPulls == 3, "still learning on the first pulls");
  printf("2. a fourth normal pull: as usual\n");
  pull(); idle(5);
  printf("     vs usual %+.2f psi (%d pulls)\n", w.last.vsNormal, w.last.normalPulls);
  expect(has(w.last.vsNormal) && fabsf(w.last.vsNormal) < 0.5f, "boost as usual");
  expect(!w.a[A_PULL].on, "no boost warning");
  printf("3. a pull 2.5 psi down (belt slip / leak / heat)\n");
  pull(2.5f);
  printf("     vs usual %+.2f psi, warning \"%s\"\n", w.last.vsNormal, w.a[A_PULL].text);
  expect(has(w.last.vsNormal) && w.last.vsNormal < -2.0f, "boost below usual");
  expect(w.a[A_PULL].on && w.top() == A_PULL && w.a[A_PULL].page == w.pPull, "warning shown on the Last Pull page");
  idle(11);
  expect(!w.a[A_PULL].on, "and gone after 10 s");
  printf("4. boost falling away in 3rd while RPM climbs\n");
  pull(0, true, 11.8f, 7);
  expect(w.last.slip && w.a[A_PULL].on, "belt slip flagged");
  idle(12);
  printf("5. lean on boost\n");
  pull(0, false, 13.8f);
  expect(w.a[A_LEAN].on || w.a[A_LEAN].clearSince >= 0, "lean warning raised during the pull");
  idle(2);
  expect(!w.a[A_LEAN].on, "cleared after the pull");
  printf("6. hot cats: amber, then red, then clear\n");
  for (float e = 0; e < 8; e += 0.05f) { In in = base(); in.cat1 = 1757; w.step(in); T += 0.05f; }
  expect(!w.a[A_CAT].on, "1757 F (normal after a hard pull on this car): nothing");
  for (float e = 0; e < 6; e += 0.05f) { In in = base(); in.cat1 = 1820; w.step(in); T += 0.05f; }
  expect(w.a[A_CAT].on && w.a[A_CAT].sev == 1 && w.a[A_CAT].page == w.pHeat, "1820 F for 5 s: amber, Heat page");
  for (float e = 0; e < 1; e += 0.05f) { In in = base(); in.cat1 = 1870; w.step(in); T += 0.05f; }
  expect(w.a[A_CAT].sev == 2, "1870 F: red");
  printf("     banner \"%s\"\n", w.a[A_CAT].text);
  idle(6);
  expect(!w.a[A_CAT].on, "cleared 5 s after dropping");
  printf("6b. the false leans from the logs don't warn\n");
  // light boost, the ECU asking for 14.7 (stoichiometric), the wideband reading 14.7
  for (float e = 0; e < 4; e += 0.05f) { In in = base(); in.psi = 6; in.pedal = 45; in.afr1 = in.afr2 = 14.7f; in.afrCmd = 14.6f; w.step(in); T += 0.05f; }
  expect(!w.a[A_LEAN].on, "light boost at 14.7 commanded: no lean");
  idle(2);
  // a full-pedal tip-in with the last reading still from the fuel cut before it (29.4)
  for (float e = 0; e < 3; e += 0.05f) { In in = base(); in.psi = 14; in.pedal = 100; in.afr1 = in.afr2 = e < 1.5f ? 29.4f : 11.8f; in.afrCmd = 12.0f; w.step(in); T += 0.05f; }
  expect(!w.a[A_LEAN].on, "stale 29.4 from the fuel cut: no lean");
  idle(2);
  // and a real one: 13.4 against 12.0 commanded at full boost
  for (float e = 0; e < 3; e += 0.05f) { In in = base(); in.psi = 15; in.pedal = 100; in.afr1 = 13.4f; in.afr2 = 12.0f; in.afrCmd = 12.0f; w.step(in); T += 0.05f; }
  expect(w.a[A_LEAN].on && w.a[A_LEAN].sev == 2, "13.4 against 12.0 commanded at full boost: lean");
  printf("     pop-up \"%s %s %s\" / %s\n", w.a[A_LEAN].title, w.a[A_LEAN].value, w.a[A_LEAN].unit, w.a[A_LEAN].note);
  idle(3);
  // log 25, 90.6 s: boost building, the ECU drops its target 14.3 -> 13.2 and the one AFR reading
  // (held for a second, read once a second) is from before the wideband followed: 14.25. Next: on target.
  { float t0 = T;
    for (float e = 0; e < 4; e += 0.05f) {
      In in = base(); in.psi = 8; in.pedal = 55;
      float at = t0 + (int)e;                                    // a new AFR reading each second
      bool stale = (int)e == 2;
      in.afrAt = at; in.afr1 = stale ? 14.25f : 13.6f; in.afr2 = stale ? 14.0f : 13.6f; in.afrCmd = stale ? 13.15f : 13.7f;
      w.step(in); T += 0.05f;
    }
    expect(!w.a[A_LEAN].on, "one stale reading a point over a just-dropped target: no lean"); }
  idle(3);
  { float t0 = T;                                              // two readings in a row: that's real
    for (float e = 0; e < 4; e += 0.05f) {
      In in = base(); in.psi = 15; in.pedal = 100;
      in.afrAt = t0 + (int)e; in.afr1 = e < 1 ? 12.0f : 13.4f; in.afr2 = 12.0f; in.afrCmd = 12.0f;
      w.step(in); T += 0.05f;
    }
    expect(w.a[A_LEAN].on, "two separate readings 1.4 over: lean"); }
  idle(3);
  // log_0048: a fuel reading taken at a 2-3 shift (1217 of 1458) and held for a second after it: not in the pull
  { int n0 = w.pullsDone; float rpm = 3500, t0 = T;
    for (float e = 0; e < 5; e += 0.05f) {
      In in = base(); in.pedal = 100; in.psi = fminf(15, 5 + e * 5); in.gear = 2;
      rpm = e < 2.5f ? rpm + 900 * 0.05f : (e < 2.55f ? rpm - 1000 : rpm + 600 * 0.05f); in.rpm = rpm;
      bool mid = e >= 2.5f && e < 3.7f;                           // the reading from the shift, held 1.2 s
      in.lpfpAt = mid ? t0 + 2.52f : t0 + e; in.lpfp = 90;
      in.hpfp = mid ? 1217 : 1930; in.hpfpSet = mid ? 1458 : 1935;
      in.t = T; w.step(in); T += 0.05f;
    }
    idle(1.5f);
    expect(w.pullsDone == n0 + 1 && gr::has(w.last.hpfpDef) && w.last.hpfpDef > -50, "fuel reading from a shift, held after it: not the pull's shortfall"); }
  idle(3);
  // log_0038: a 5th-gear roll-on at 35-62 % pedal, low side 29-36 psi for 3 readings, high side on target
  { float t0 = T;
    for (float e = 0; e < 4; e += 0.05f) { In in = base(); in.psi = 10; in.pedal = 55; in.lpfpAt = t0 + (int)e; in.lpfp = 31; in.hpfp = 1900; in.hpfpSet = 1840; w.step(in); T += 0.05f; }
    expect(!w.a[A_LPFP].on, "low side 31 psi at part throttle, high side on target: no warning"); }
  idle(3);
  { float t0 = T;                                              // the same at full pedal: that's a supply problem
    for (float e = 0; e < 4; e += 0.05f) { In in = base(); in.psi = 15; in.pedal = 100; in.lpfpAt = t0 + (int)e; in.lpfp = 31; in.hpfp = 2100; in.hpfpSet = 2150; w.step(in); T += 0.05f; }
    expect(w.a[A_LPFP].on, "low side 31 psi at full pedal, two readings: warning"); }
  idle(4);
  // fuel pump at idle under its (low) target: doesn't matter
  for (float e = 0; e < 3; e += 0.05f) { In in = base(); in.hpfp = 445; in.hpfpSet = 650; w.step(in); T += 0.05f; }
  expect(!w.a[A_HPFP].on, "fuel pump 445 of 650 at idle: no warning");
  idle(3);
  // log_0036, 2870.8 s: a downshift tip-in, the target jumps to 1494 and the one reading is 1261; next 1623 of 1631
  { float t0 = T;
    for (float e = 0; e < 3; e += 0.05f) { In in = base(); in.psi = 12; in.pedal = 66; in.lpfpAt = t0 + (int)e; in.lpfp = 87;
      bool first = (int)e == 1; in.hpfp = first ? 1261 : 1623; in.hpfpSet = first ? 1494 : 1631; w.step(in); T += 0.05f; }
    expect(!w.a[A_HPFP].on, "one fuel pump reading short as the target jumps: no warning"); }
  idle(3);
  { float t0 = T;                                              // short on two readings in a row: real
    for (float e = 0; e < 3; e += 0.05f) { In in = base(); in.psi = 15; in.pedal = 100; in.lpfpAt = t0 + (int)e; in.lpfp = 80; in.hpfp = 1500; in.hpfpSet = 2150; w.step(in); T += 0.05f; }
    expect(w.a[A_HPFP].on && w.a[A_HPFP].sev == 2, "fuel pump 1500 of 2150 on two readings: red"); }
  idle(4);
  // log_0036, 1685.6 s: lifting off from a pull (pedal 0, the sender still at 10 psi and falling): -5.6 on cyl 6
  { In in = base(); in.pedal = 100; in.psi = 15; in.knock = 0; for (int i = 0; i < 20; i++) { in.t = T; w.step(in); T += 0.05f; }
    for (int i = 0; i < 6; i++) { in.t = T; in.pedal = 0; in.psi = 15 - i * 2.5f; in.knock = i >= 2 ? -5.6f : 0; in.knockCyl = 6; w.step(in); T += 0.05f; } }
  expect(!w.a[A_KNOCK].on, "knock read while lifting off: no warning");
  idle(4);
  printf("7. heat-soaked intake at a standstill doesn't warn; on the move it does\n");
  for (float e = 0; e < 5; e += 0.05f) { In in = base(); in.kmh = 0; in.iat = 155; w.step(in); T += 0.05f; }
  expect(!w.a[A_IAT].on, "155 F parked: no warning");
  for (float e = 0; e < 5; e += 0.05f) { In in = base(); in.iat = 155; w.step(in); T += 0.05f; }
  expect(w.a[A_IAT].on && w.a[A_IAT].sev == 1, "155 F moving: amber");
  printf("8. red beats amber\n");
  for (float e = 0; e < 4; e += 0.05f) { In in = base(); in.iat = 155; in.clt = 235; w.step(in); T += 0.05f; }
  expect(w.top() == A_CLT, "coolant (red) on top of intake (amber)");
  idle(8);
  printf("9. a 1 s blip is not a pull\n");
  int n0 = w.pullsDone;
  for (float e = 0; e < 1; e += 0.05f) { In in = base(); in.pedal = 100; in.psi = 8; in.rpm = 3500; in.gear = 3; w.step(in); T += 0.05f; }
  idle(2);
  expect(w.pullsDone == n0, "ignored");
  printf("10. knock: instant, on the Fuel page - when it happens under load\n");
  { In in = base(); in.knock = -5.6f; in.knockCyl = 6; for (int i = 0; i < 40; i++) { in.t = T; w.step(in); T += 0.05f; } }
  expect(!w.a[A_KNOCK].on, "light-load retard (-5.6 cyl 6 at part throttle): no warning");
  { In in = base(); in.knock = -5.6f; in.knockCyl = 6; in.pedal = 100; in.psi = 12; in.rpm = 4000; in.gear = 3;
    for (int i = 0; i < 20; i++) { in.t = T; in.knock = -5.6f + i * 0.05f; w.step(in); T += 0.05f; } }
  expect(!w.a[A_KNOCK].on, "the same retard carried into boost, easing off: no warning");
  { In in = base(); in.knock = 0; for (int i = 0; i < 40; i++) { in.t = T; w.step(in); T += 0.05f; } }   // retard gone, back on part throttle
  { In in = base(); in.pedal = 100; in.psi = 13; in.rpm = 4500; in.gear = 3; in.knock = 0;
    for (int i = 0; i < 25; i++) { in.t = T; w.step(in); T += 0.05f; }   // foot down for over a second first
    in.knock = -4.1f; in.knockCyl = 3; in.t = T; w.step(in); T += 0.05f; }
  printf("     banner \"%s\"\n", w.a[A_KNOCK].text);
  expect(w.a[A_KNOCK].on && w.a[A_KNOCK].sev == 1 && w.a[A_KNOCK].page == w.pFuel, "new knock on boost: -4.1 cyl 3, amber");
  { In in = base(); in.pedal = 100; in.psi = 13; in.rpm = 4600; in.gear = 3; in.knock = -5.2f; in.knockCyl = 3; in.t = T; w.step(in); T += 0.05f; }
  expect(w.a[A_KNOCK].on && w.a[A_KNOCK].sev == 2, "  worse, -5.2: red");
  idle(5);
  // logs 36-48: a part-throttle tip-in in 2nd, the pedal going 20 -> 54 %, boost briefly past 8 psi: cylinder 6 -5.2
  { In in = base(); in.pedal = 20; in.psi = -5; in.rpm = 3800; in.gear = 2; in.knock = 0;
    for (int i = 0; i < 20; i++) { in.t = T; w.step(in); T += 0.05f; }
    for (int i = 0; i < 12; i++) { in.t = T; in.pedal = 54; in.psi = -5 + i * 1.3f; in.knock = i >= 8 ? -5.25f : 0; in.knockCyl = 6; w.step(in); T += 0.05f; } }
  expect(!w.a[A_KNOCK].on, "tip-in at 54 % pedal, -5.2 on cylinder 6: no warning");
  idle(5);
  // log_0047, 746 s: lifting right after a 3-4 shift, the pedal reading still 80 % (it's a second old), boost
  // 13 -> 4.6 psi in a tenth of a second, then -3.4 on cylinder 6
  { In in = base(); in.pedal = 83; in.psi = 15; in.rpm = 5000; in.gear = 3; in.knock = 0;
    for (int i = 0; i < 30; i++) { in.t = T; w.step(in); T += 0.05f; }
    const float drop[6] = {14.7f, 13.2f, 4.6f, 0.2f, -6.1f, -9.5f};
    for (int i = 0; i < 6; i++) { in.t = T; in.pedal = 80; in.psi = drop[i]; in.knock = i >= 2 ? -3.37f : 0; in.knockCyl = 6; w.step(in); T += 0.05f; } }
  expect(!w.a[A_KNOCK].on, "knock as boost falls away at a lift (pedal reading still 80 %): no warning");
  idle(5);
  printf("11. a pull's knock is only what started in it\n");
  { In in = base(); in.knock = -5.6f; in.knockCyl = 6; for (int i = 0; i < 20; i++) { in.t = T; w.step(in); T += 0.05f; } }
  { int n0 = w.pullsDone; float rpm = 3000;
    for (float e = 0; e < 4; e += 0.05f) { In in = base(); rpm += 700 * 0.05f; in.rpm = rpm; in.gear = 2; in.pedal = 100; in.psi = fminf(15, 6 + e * 4);
      in.knock = fminf(0, -5.6f + e * 1.5f); in.knockCyl = 6; in.hpfp = 2100; in.hpfpSet = 2150; in.t = T; w.step(in); T += 0.05f; }
    idle(1.5f);
    expect(w.pullsDone == n0 + 1 && gr::has(w.last.knock) && w.last.knock == 0, "held -5.6 easing off: knock 0 for the pull"); }
  printf("12. fuel pressure dips at a shift are left out\n");
  { float rpm = 3000; int gear = 2;
    for (float e = 0; e < 5; e += 0.05f) { In in = base(); rpm += 800 * 0.05f; if (gear == 2 && rpm > 6500) { gear = 3; rpm = 4800; }
      bool justShifted = gear == 3 && rpm < 5000;
      in.rpm = rpm; in.gear = gear; in.pedal = 100; in.psi = 14; in.knock = 0; in.hpfp = justShifted ? 1150 : 1900; in.hpfpSet = justShifted ? 1500 : 1880; in.t = T; w.step(in); T += 0.05f; }
    idle(1.5f);
    printf("     worst shortfall %.0f psi\n", w.last.hpfpDef);
    expect(gr::has(w.last.hpfpDef) && w.last.hpfpDef > -100, "the -350 at the 2-3 shift isn't counted"); }
  printf(fails ? "%d FAILED\n" : "all watcher tests passed\n", fails);
  return fails != 0;
}
