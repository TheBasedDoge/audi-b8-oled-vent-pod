// =====================================================================
//  S5 Gauge Pod - health trends: one small record per pull, kept in a file
//
//  Every pull the watcher sums up (watch.h) is stored as a 74-byte record in
//  /ffat/health.bin (the internal flash; the log clean-up only ever deletes log_*.csv).
//  The web page's Health tab reads the whole file (/health.bin) and charts it: boost at each
//  RPM against a fixed baseline, boost against the ECU's target, fuel pressures, knock, AFR,
//  intake heat, injector duty and belt slip, over miles and weeks. The usual-boost curve in
//  watch.h follows the car (that's what it's for), so a slow drift never shows against it;
//  the baseline here stays put until you set a new one (after a pulley or a tune change).
//
//  Records are little-endian and packed, so the page reads them with a DataView. Plain C
//  stdio (the ESP32 maps /ffat to the FAT partition), so tools/health_test.cpp runs it on a PC.
// =====================================================================
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "pull_sum.h"

namespace health {

static const uint32_t MAGIC = 0x31544C48;          // "HLT1"
static const uint16_t VER = 1;
static const int KEEP_MAX = 600, KEEP_TRIM = 500;   // past 600 records, keep the newest 500

enum : uint8_t { F_SLIP = 1, F_UNDATED = 2, F_OBDBOOST = 4 };

#pragma pack(push, 1)
struct Header {
  uint32_t magic; uint16_t recSize, ver;
  uint32_t baseSeq;                                  // the baseline starts at this record (0 = the first)
  uint32_t nextSeq;
};
struct Rec {
  uint32_t seq, when, odoKm, ms;                     // when = epoch s (0 = no clock yet); ms = millis() then
  uint16_t boot;                                     // power-on count (to date records once the clock arrives)
  uint8_t  flags, gear0;                             // first gear of the pull
  int8_t   bins4[10];                                // psi x 4 per 500 rpm band, 2500-7500 (-128 = none)
  int16_t  peak10, tgt10, vs10, knock10, minTim10;   // psi x 10, psi x 10 (ECU target), psi x 10 (vs usual), deg x 10, deg x 10
  int16_t  afrOver100, afrMax100, idc10, dur10;      // AFR x 100, AFR x 100, % x 10, s x 10
  uint16_t peakRpm;
  uint8_t  knockCyl, pad;
  int16_t  iat0, iatMax, amb, oil, clt;              // degF
  int16_t  hpfpDef, hpfpMin, lpfpMin, catMax;        // psi, psi, psi, degF
  int16_t  kmh0, kmh1;
};
#pragma pack(pop)
static_assert(sizeof(Header) == 16, "header layout");
static_assert(sizeof(Rec) == 74, "record layout (the web page reads it at these offsets)");

static const int16_t NONE16 = -32768;
static inline int16_t i16(float v, float k) {
  if (!gr::has(v)) return NONE16;
  float x = v * k; if (x > 32767) x = 32767; if (x < -32767) x = -32767;
  return (int16_t)(x < 0 ? x - 0.5f : x + 0.5f);
}

// a record from a pull summary and what was around it
static Rec make(const gr::PullSum &p, float ambF, float oilF, float cltF, uint32_t odoKm, uint32_t when,
                uint32_t ms, uint16_t boot, bool obdBoost) {
  Rec r; memset(&r, 0, sizeof(r));
  r.when = when; r.odoKm = odoKm; r.ms = ms; r.boot = boot;
  r.flags = (p.slip ? F_SLIP : 0) | (when ? 0 : F_UNDATED) | (obdBoost ? F_OBDBOOST : 0);
  r.gear0 = (p.gears[0] >= '1' && p.gears[0] <= '7') ? (uint8_t)(p.gears[0] - '0') : 0;
  for (int i = 0; i < 10; i++) {
    float b = p.binPsi[i];
    if (!gr::has(b)) { r.bins4[i] = -128; continue; }
    float x = b * 4; if (x > 127) x = 127; if (x < -127) x = -127;
    r.bins4[i] = (int8_t)(x + 0.5f);
  }
  r.peak10 = i16(p.peakPsi, 10); r.tgt10 = i16(p.tgtPeak, 10); r.vs10 = i16(p.vsNormal, 10);
  r.knock10 = i16(p.knock, 10); r.minTim10 = i16(p.minTiming, 10);
  r.afrOver100 = i16(p.afrOver, 100); r.afrMax100 = i16(p.afrMax, 100); r.idc10 = i16(p.idcMax, 10); r.dur10 = i16(p.durS, 10);
  r.peakRpm = (uint16_t)(p.peakRpm > 0 ? p.peakRpm : 0); r.knockCyl = (uint8_t)p.knockCyl;
  r.iat0 = i16(p.iat0, 1); r.iatMax = i16(p.iatMax, 1); r.amb = i16(ambF, 1); r.oil = i16(oilF, 1); r.clt = i16(cltF, 1);
  r.hpfpDef = i16(p.hpfpDef, 1); r.hpfpMin = i16(p.hpfpMin, 1); r.lpfpMin = i16(p.lpfpMin, 1); r.catMax = i16(p.catMax, 1);
  r.kmh0 = i16(p.kmh0, 1); r.kmh1 = i16(p.kmh1, 1);
  return r;
}

static long sizeOf(const char *path) { FILE *f = fopen(path, "rb"); if (!f) return -1; fseek(f, 0, SEEK_END); long n = ftell(f); fclose(f); return n; }
// A file that doesn't read as ours but holds data (a header torn by a power cut) is set aside as
// <name>.bad, never emptied: the records in it can still be recovered on a computer.
static void keepBad(const char *path) {
  if (sizeOf(path) <= (long)sizeof(Header)) return;
  char bad[96]; snprintf(bad, sizeof(bad), "%s", path);
  char *dot = strrchr(bad, '.'), *slash = strrchr(bad, '/');
  if (!dot || (slash && dot < slash) || strlen(bad) + 5 > sizeof(bad)) return;
  strcpy(dot, ".bad");
  remove(bad); rename(path, bad);
}
// open (creating it with a header if it's missing or not ours); returns the header.
// tmpPath: if the file is missing but a finished trim copy is there (power cut between removing the
// old file and renaming the new one), that copy is put in its place first.
static FILE *openFile(const char *path, Header &h, const char *tmpPath = nullptr) {
  FILE *f = fopen(path, "r+b");
  if (!f && tmpPath && sizeOf(tmpPath) >= (long)sizeof(Header) && rename(tmpPath, path) == 0) f = fopen(path, "r+b");
  if (f && fread(&h, sizeof(h), 1, f) == 1 && h.magic == MAGIC && h.recSize == sizeof(Rec)) return f;
  if (f) { fclose(f); keepBad(path); }
  f = fopen(path, "w+b");
  if (!f) return nullptr;
  h.magic = MAGIC; h.recSize = sizeof(Rec); h.ver = VER; h.baseSeq = 0; h.nextSeq = 0;
  if (fwrite(&h, sizeof(h), 1, f) != 1) { fclose(f); return nullptr; }
  return f;
}
static long countOf(FILE *f) { fseek(f, 0, SEEK_END); long n = (ftell(f) - (long)sizeof(Header)) / (long)sizeof(Rec); return n < 0 ? 0 : n; }
static void putHeader(FILE *f, const Header &h) { fseek(f, 0, SEEK_SET); fwrite(&h, sizeof(h), 1, f); }

// keep the newest KEEP_TRIM records (rewrites the file; about 36 KB)
static bool trim(const char *path, const char *tmpPath) {
  Header h; FILE *f = openFile(path, h);
  if (!f) return false;
  long n = countOf(f);
  if (n <= KEEP_TRIM) { fclose(f); return true; }
  FILE *t = fopen(tmpPath, "wb");
  if (!t) { fclose(f); return false; }
  fwrite(&h, sizeof(h), 1, t);
  fseek(f, (long)sizeof(Header) + (n - KEEP_TRIM) * (long)sizeof(Rec), SEEK_SET);
  Rec r;
  while (fread(&r, sizeof(r), 1, f) == 1) fwrite(&r, sizeof(r), 1, t);
  fclose(f); fclose(t);
  remove(path);
  return rename(tmpPath, path) == 0;
}

// add one record (its seq is set here). Returns the number of records afterwards (-1 = failed).
static long append(const char *path, const char *tmpPath, Rec &r) {
  Header h; FILE *f = openFile(path, h, tmpPath);
  if (!f) return -1;
  r.seq = h.nextSeq++;
  fseek(f, 0, SEEK_END);
  bool ok = fwrite(&r, sizeof(r), 1, f) == 1;
  putHeader(f, h);
  long n = countOf(f);
  fclose(f);
  if (!ok) return -1;
  if (n > KEEP_MAX) { trim(path, tmpPath); n = KEEP_TRIM; }
  return n;
}

// the baseline starts with the next pull (after a pulley, a tune, new plugs...)
static bool setBaseline(const char *path) {
  Header h; FILE *f = openFile(path, h);
  if (!f) return false;
  h.baseSeq = h.nextSeq;
  putHeader(f, h); fclose(f);
  return true;
}

// records from this power-on that had no date: dated back from now
static int dateBack(const char *path, uint16_t boot, uint32_t epochNow, uint32_t msNow) {
  Header h; FILE *f = openFile(path, h);
  if (!f) return 0;
  long n = countOf(f);
  int fixed = 0;
  long from = n > 40 ? n - 40 : 0;                   // only the newest can be from this power-on
  for (long i = from; i < n; i++) {
    long off = (long)sizeof(Header) + i * (long)sizeof(Rec);
    Rec r; fseek(f, off, SEEK_SET);
    if (fread(&r, sizeof(r), 1, f) != 1) break;
    if (!(r.flags & F_UNDATED) || r.boot != boot || r.ms > msNow) continue;
    r.when = epochNow - (msNow - r.ms) / 1000; r.flags &= ~F_UNDATED;
    fseek(f, off, SEEK_SET); fwrite(&r, sizeof(r), 1, f);
    fixed++;
  }
  fclose(f);
  return fixed;
}

static inline long count(const char *path) { Header h; FILE *f = openFile(path, h); if (!f) return 0; long n = countOf(f); fclose(f); return n; }

// ---------------------------------------------------------------- catalyst trend
// /ffat/cats.bin: the same 16-byte header (magic "CAT1"), then one 24-byte record each time the ECU's
// catalyst test result changes (it reruns the test during steady cruise, about once a drive).
static const uint32_t CAT_MAGIC = 0x31544143;      // "CAT1"
enum : uint8_t { CF_B1 = 1, CF_B2 = 2, CF_MONKNOWN = 4, CF_DONE_SINCE = 8, CF_DONE_DRIVE = 16 };
#pragma pack(push, 1)
struct CatRec {
  uint32_t when, odoKm;
  uint16_t val[2], min[2], max[2];                 // bank 1, bank 2 (raw, as the ECU gives them)
  uint8_t  tid[2], flags, pad;
};
#pragma pack(pop)
static_assert(sizeof(CatRec) == 24, "catalyst record layout (the web page reads it at these offsets)");
static bool sameResult(const CatRec &a, const CatRec &b) {
  return (a.flags & 3) == (b.flags & 3) && !memcmp(a.val, b.val, sizeof(a.val)) && !memcmp(a.min, b.min, sizeof(a.min)) &&
         !memcmp(a.max, b.max, sizeof(a.max));
}
// the newest record (false = none yet)
static bool catLast(const char *path, CatRec &out) {
  FILE *f = fopen(path, "rb"); if (!f) return false;
  Header h; bool ok = fread(&h, sizeof(h), 1, f) == 1 && h.magic == CAT_MAGIC && h.recSize == sizeof(CatRec);
  if (ok) { fseek(f, 0, SEEK_END); long n = (ftell(f) - (long)sizeof(h)) / (long)sizeof(CatRec);
            ok = n > 0 && fseek(f, (long)sizeof(h) + (n - 1) * (long)sizeof(CatRec), SEEK_SET) == 0 && fread(&out, sizeof(out), 1, f) == 1; }
  fclose(f); return ok;
}
static bool catAppend(const char *path, const CatRec &r) {
  FILE *f = fopen(path, "r+b");
  Header h;
  if (!f || fread(&h, sizeof(h), 1, f) != 1 || h.magic != CAT_MAGIC || h.recSize != sizeof(CatRec)) {
    if (f) { fclose(f); keepBad(path); }
    f = fopen(path, "w+b"); if (!f) return false;
    h.magic = CAT_MAGIC; h.recSize = sizeof(CatRec); h.ver = 1; h.baseSeq = 0; h.nextSeq = 0;
    fwrite(&h, sizeof(h), 1, f);
  }
  fseek(f, 0, SEEK_END);
  bool ok = fwrite(&r, sizeof(r), 1, f) == 1;
  h.nextSeq++; fseek(f, 0, SEEK_SET); fwrite(&h, sizeof(h), 1, f);
  fclose(f); return ok;
}

}  // namespace health
