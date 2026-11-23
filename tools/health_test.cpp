// Desktop test for the health trends file (firmware/S5GaugePod_Test/health.h).
// build: g++ -std=c++17 -O2 -Ifirmware/S5GaugePod_Test tools/health_test.cpp -o health_test && ./health_test
#include <cstdio>
#include <cstddef>
#include <cmath>
#include "health.h"

static int fails = 0;
static void check(bool ok, const char *what) { printf("  %-62s %s\n", what, ok ? "ok" : "FAIL"); if (!ok) fails++; }

int main() {
  const char *P = "health_test.bin", *T = "health_test.tmp";
  remove(P);
  printf("record layout (the web page reads these offsets)\n");
  check(offsetof(health::Rec, bins4) == 20 && offsetof(health::Rec, peak10) == 30 && offsetof(health::Rec, peakRpm) == 48 &&
        offsetof(health::Rec, iat0) == 52 && offsetof(health::Rec, hpfpDef) == 62 && offsetof(health::Rec, kmh1) == 72 && sizeof(health::Rec) == 74,
        "offsets as the page expects");

  printf("one pull\n");
  gr::PullSum p;
  p.valid = true; p.durS = 4.2f; strcpy(p.gears, "3-4"); p.peakPsi = 17.3f; p.peakRpm = 6100; p.tgtPeak = 18.0f; p.vsNormal = -0.3f;
  p.knock = -1.5f; p.knockCyl = 4; p.minTiming = 6.0f; p.afrOver = 0.4f; p.afrMax = 12.4f; p.idcMax = 81.0f;
  p.iat0 = 95; p.iatMax = 121; p.hpfpDef = -60; p.hpfpMin = 1590; p.lpfpMin = 72; p.catMax = 1450; p.kmh0 = 60; p.kmh1 = 140; p.slip = false;
  for (int i = 0; i < 10; i++) p.binPsi[i] = i < 2 ? gr::NOV : 12.0f + i * 0.5f;
  health::Rec r = health::make(p, 68, 220, 196, 118636, 0, 5000, 7, false);
  check(r.bins4[0] == -128 && r.bins4[2] == 52 && r.peak10 == 173 && r.knock10 == -15 && r.afrOver100 == 40, "packed values");
  check(r.gear0 == 3 && (r.flags & health::F_UNDATED) && r.odoKm == 118636, "gear, no date yet, odometer");
  gr::PullSum q; q.peakPsi = 10; q.durS = 2;
  health::Rec r2 = health::make(q, gr::NOV, gr::NOV, gr::NOV, 0, 1700000000, 0, 7, true);
  check(r2.amb == health::NONE16 && r2.hpfpDef == health::NONE16 && (r2.flags & health::F_OBDBOOST) && !(r2.flags & health::F_UNDATED), "missing values marked, dated");

  printf("file\n");
  check(health::append(P, T, r) == 1, "first record written");
  check(health::dateBack(P, 7, 1700001000, 65000) == 1, "dated back once the clock arrives");
  { health::Header h; FILE *f = health::openFile(P, h); health::Rec x; fseek(f, sizeof(h), SEEK_SET); if (fread(&x, sizeof(x), 1, f) != 1) x.when = 0; fclose(f);
    check(x.when == 1700001000 - 60 && !(x.flags & health::F_UNDATED), "  to 60 s before now"); }
  check(health::dateBack(P, 8, 1700002000, 65000) == 0, "not a record from another power-on");
  for (int i = 0; i < 700; i++) { health::Rec x = r; x.ms = i; health::append(P, T, x); }
  long n = health::count(P);
  check(n <= health::KEEP_MAX && n >= health::KEEP_TRIM, "trimmed to the newest");
  { health::Header h; FILE *f = health::openFile(P, h); long c = health::countOf(f); health::Rec last; fseek(f, sizeof(h) + (c - 1) * sizeof(health::Rec), SEEK_SET); if (fread(&last, sizeof(last), 1, f) != 1) last.seq = 0; fclose(f);
    check(last.seq == 700 && h.nextSeq == 701, "sequence numbers carry on after a trim"); }
  check(health::setBaseline(P), "baseline set");
  { health::Header h; FILE *f = health::openFile(P, h); fclose(f); check(h.baseSeq == 701, "  starting with the next pull"); }

  printf("power cuts\n");
  {                                                  // cut between removing the old file and renaming the trimmed copy
    long before = health::count(P);
    remove(T); rename(P, T);                         // what's left on the disk then: only the finished copy
    health::Rec x = r; long after = health::append(P, T, x);
    check(before >= health::KEEP_TRIM && after >= health::KEEP_TRIM, "a finished trim copy is put back, nothing lost");
  }
  {                                                  // a header torn by a power cut: the records are set aside, not emptied
    FILE *f = fopen(P, "r+b"); uint32_t junk = 0xFFFFFFFF; fwrite(&junk, 4, 1, f); fclose(f);
    long had = health::sizeOf(P);
    remove("health_test.bad");
    health::Rec x = r; long after = health::append(P, T, x);
    check(after == 1, "a file that isn't ours starts again");
    check(health::sizeOf("health_test.bad") == had, "  and the old one is kept as .bad, whole");
    remove("health_test.bad");
  }
  remove(P); remove(T);
  if (fails) { printf("%d FAILED\n", fails); return 1; }
  printf("all health tests passed\n");
  return 0;
}
