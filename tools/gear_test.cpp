// Replays a drive log (rpm + speed columns) through firmware/S5GaugePod_Test/gear.h and checks
// the gears it reports. Speed is fed only when it changes or every 0.5 s, like the old 2 Hz
// polling (the worst case). With no log given it runs a synthetic drive through all 7 gears.
// build: g++ -std=c++17 -O2 tools/gear_test.cpp -o gear_test && ./gear_test [log.csv]
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include "../firmware/S5GaugePod_Test/gear.h"

static int fails = 0;
static void expect(bool ok, const char *what) { printf("  %-60s %s\n", what, ok ? "ok" : "FAIL"); if (!ok) fails++; }

int main(int argc, char **argv) {
  printf("1. synthetic drive: 1st to 7th and back down, with clutch slip at a standstill\n");
  {
    gearx::Est e; e.begin(nullptr);
    float t = 0; int seen[8] = {0}, wrong = 0;
    for (int g = 1; g <= 7; g++)                    // hold each gear at a few speeds
      for (float kmh = 20 + g * 12; kmh < 30 + g * 22; kmh += 0.5f) {
        float rpm = kmh * gearx::DEFAULTS[g - 1] * 1.02f;   // the car is 2 % off the starting values
        for (int k = 0; k < 3; k++) { e.update(rpm, kmh, t); t += 0.2f; }
        if (e.gear) { seen[e.gear]++; if (e.gear != g) wrong++; }
      }
    int all = 1; for (int g = 1; g <= 7; g++) all &= seen[g] > 0;
    expect(all, "every gear recognised");
    expect(wrong == 0, "never the wrong gear");
    float worst = 0; for (int g = 0; g < 7; g++) worst = fmaxf(worst, fabsf(e.cal.r[g] / (gearx::DEFAULTS[g] * 1.02f) - 1));
    printf("     learned ratios within %.1f %% of the truth\n", worst * 100);
    expect(worst < 0.012f, "ratios learned (2 % offset taken out)");
    e.update(1500, 3, t);
    expect(e.gear == 0, "below 5 km/h: \"-\"");
    e.update(3000, 20, t + 0.2f);                   // 150 rpm per km/h: slipping, matches nothing
    expect(e.gear == 0, "clutch slip (no match): \"-\"");
  }
  if (argc < 2) { printf(fails ? "%d FAILED\n" : "all gear tests passed\n", fails); return fails != 0; }

  printf("2. replay of %s\n", argv[1]);
  FILE *f = fopen(argv[1], "r");
  if (!f) { printf("can't open it\n"); return 1; }
  char line[512];
  if (!fgets(line, sizeof(line), f)) return 1;
  int ci[32], nc = 0, iMs = -1, iRpm = -1, iKmh = -1, iThr = -1;
  for (char *tok = strtok(line, ",\n"); tok; tok = strtok(nullptr, ",\n"), nc++) {
    if (!strcmp(tok, "ms")) iMs = nc;
    if (!strcmp(tok, "rpm")) iRpm = nc;
    if (!strcmp(tok, "speed_kmh")) iKmh = nc;
    if (!strcmp(tok, "throttle_pct")) iThr = nc;
  }
  (void)ci;
  gearx::Est e; e.begin(nullptr);
  int counts[8] = {0}; float lastKmh = -1, lastFeed = -1; int prevGear = 0;
  std::vector<int> seq;
  while (fgets(line, sizeof(line), f)) {
    char *c[32]; int n = 0; char *p = line;
    while (n < 32) { c[n++] = p; p = strchr(p, ','); if (!p) break; *p++ = 0; }
    if (n <= iKmh || !*c[iRpm] || !*c[iKmh]) continue;
    float t = atof(c[iMs]) / 1000, rpm = atof(c[iRpm]), kmh = atof(c[iKmh]);
    if (kmh != lastKmh || t - lastFeed >= 0.5f) { e.update(rpm, kmh, t); lastKmh = kmh; lastFeed = t; }
    counts[e.gear]++;
    if (e.gear && e.gear != prevGear) { seq.push_back(e.gear); prevGear = e.gear; }
    (void)iThr;
  }
  fclose(f);
  printf("     samples per gear: -:%d", counts[0]); for (int g = 1; g <= 7; g++) printf("  %d:%d", g, counts[g]); printf("\n");
  printf("     gear changes: "); for (size_t i = 0; i < seq.size() && i < 60; i++) printf("%d ", seq[i]); printf("\n");
  printf("     learned rpm per km/h:"); for (int g = 0; g < 7; g++) printf(" %.1f(%d)", e.cal.r[g], e.cal.n[g]); printf("\n");
  int jumps = 0; for (size_t i = 1; i < seq.size(); i++) if (abs(seq[i] - seq[i - 1]) > 2) jumps++;
  expect(jumps * 10 <= (int)seq.size(), "gear changes are mostly one or two steps");
  printf(fails ? "%d FAILED\n" : "all gear tests passed\n", fails);
  return fails != 0;
}
