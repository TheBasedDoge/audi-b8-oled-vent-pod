// Replay a drive log through the warning watcher (firmware/S5GaugePod_Test/watch.h) and print
// every warning it would have raised and every pull summary. For checking limits against real
// drives. build: g++ -std=c++17 -O2 -Ifirmware/S5GaugePod_Test tools/watch_replay.cpp -o watch_replay
// run: ./watch_replay log_0008.csv
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include "watch.h"
using namespace watch;
int main(int argc, char **argv) {
  if (argc < 2) { printf("usage: watch_replay log.csv\n"); return 1; }
  FILE *f = fopen(argv[1], "r"); if (!f) { perror(argv[1]); return 1; }
  char line[1024]; std::vector<std::string> head;
  if (!fgets(line, sizeof line, f)) return 1;
  for (char *p = strtok(line, ",\r\n"); p; p = strtok(nullptr, ",\r\n")) head.push_back(p);
  auto col = [&](const char *n) { for (size_t i = 0; i < head.size(); i++) if (head[i] == n) return (int)i; return -1; };
  Watch w; w.begin(nullptr);
  bool was[A_N] = {false}; int raised = 0; float t0 = -1, runStart = -1;
  while (fgets(line, sizeof line, f)) {
    std::vector<std::string> v; std::string cur;
    for (char *c = line; *c && *c != '\n' && *c != '\r'; c++) { if (*c == ',') { v.push_back(cur); cur.clear(); } else cur += *c; }
    v.push_back(cur);
    if (v.size() == head.size() + 1) v.erase(v.begin() + 16);           // old logs: an extra empty G column
    auto g = [&](const char *n) { int i = col(n); return (i >= 0 && i < (int)v.size() && !v[i].empty()) ? (float)atof(v[i].c_str()) : NOV; };
    In in;
    float ms = g("ms"); if (t0 < 0) t0 = ms;
    in.t = (ms - t0) / 1000;
    in.psi = g("psi"); in.rpm = g("rpm"); in.kmh = g("speed_kmh"); in.pedal = g("pedal_pct"); in.thr = g("throttle_pct");
    float gear = g("gear"); in.gear = has(gear) ? (int)gear : 0;
    in.iat = g("intake_F"); in.timing = g("timing_deg"); in.knock = g("knock_deg"); float kc = g("knock_cyl"); in.knockCyl = has(kc) ? (int)kc : 0;
    in.hpfp = g("hpfp_psi"); in.hpfpSet = g("hpfp_target_psi"); in.lpfp = g("lpfp_psi"); in.idc = g("injector_duty_pct");
    in.afr1 = g("afr_bank1"); in.afr2 = g("afr_bank2"); in.afrCmd = g("afr_target");
    {                                               // the log repeats the last reading: a reading is "taken" when it changes
      static float lastAfr = -1e9f, afrAt = NOV, lastLp = -1e9f, lpAt = NOV;
      if (has(in.afr1) && in.afr1 != lastAfr) { lastAfr = in.afr1; afrAt = in.t; }
      if (has(in.lpfp) && in.lpfp != lastLp) { lastLp = in.lpfp; lpAt = in.t; }
      in.afrAt = afrAt; in.lpfpAt = lpAt;
    }
    in.cat1 = g("cat1_F"); in.cat2 = g("cat2_F"); in.clt = g("coolant_F"); in.oil = g("oil_F"); in.bat = g("battery_V"); in.bypass = g("bypass_deg");
    float map = g("map_kPa"), baro = g("baro_kPa"), tgt = g("map_target_kPa"); if (!has(baro)) baro = 101.3f;
    in.obdPsi = has(map) ? (map - baro) * 0.145038f : NOV; in.tgtPsi = has(tgt) ? (tgt - baro) * 0.145038f : NOV;
    in.redPsi = 20;
    bool running = has(in.rpm) && in.rpm > 400;
    if (!running) runStart = -1; else if (runStart < 0) runStart = in.t;
    in.runSec = runStart >= 0 ? in.t - runStart + 0.01f : 0;
    int pulls = w.pullsDone;
    w.step(in);
    for (int i = 0; i < A_N; i++) {
      if (w.a[i].on && !was[i]) { raised++; printf("%7.1f s  %-5s %s\n", in.t, w.a[i].sev >= 2 ? "RED" : "amber", w.a[i].text); }
      was[i] = w.a[i].on;
    }
    if (w.pullsDone != pulls) {
      const gr::PullSum &p = w.last;
      printf("%7.1f s  pull: %.1f s, gears %s, peak %.1f psi at %.0f rpm, leanest AFR %.1f (%+.1f vs target), hottest cat %.0f F, fuel pump %+.0f psi vs target\n",
             in.t, p.durS, p.gears, p.peakPsi, p.peakRpm, has(p.afrMax) ? p.afrMax : 0, has(p.afrOver) ? p.afrOver : 0, has(p.catMax) ? p.catMax : 0, has(p.hpfpDef) ? p.hpfpDef : 0);
    }
  }
  printf("%d warning(s), %d pull(s)\n", raised, w.pullsDone);
}
