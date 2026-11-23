// =====================================================================
//  S5 Gauge Pod - finding an unknown identifier (for example the drive select mode)
//
//  The OBD tab's "Find an identifier" tool asks one ECU for every Mode 22 identifier in a
//  range and keeps the ones it answers (a discovery scan, the first snapshot). After you
//  change something in the car (Comfort -> Dynamic), "read again" re-reads only those, as
//  another labelled snapshot (seconds, not minutes). This file ranks them: an identifier
//  that stays the same across snapshots with the same label, and differs between labels,
//  is what you're looking for. Taking each mode twice (Comfort, Dynamic, Comfort, Dynamic)
//  rules out the values that change on their own (sensors, counters).
//
//  Plain C++ (no Arduino calls), tested on the desktop by tools/scan_test.cpp. The OBD task
//  (obd_ble.h) does the asking.
// =====================================================================
#pragma once
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>

namespace scanx {

static const int NS = 6;                         // snapshots kept
static const int MAXB = 12;                      // bytes kept per answer
static const int MAXE = 1500;                    // identifiers that answer, kept

struct Entry {
  uint16_t did;
  uint8_t  len[NS];                              // bytes in that snapshot's answer (0 = no answer)
  uint8_t  b[NS][MAXB];
};

struct Table {
  Entry   *e = nullptr; int n = 0;               // storage is given by the caller (PSRAM on the pod)
  int      ns = 0;                               // snapshots taken
  char     label[NS][16];
  void clear() { n = 0; ns = 0; }
  // a new snapshot; returns its index (-1 = full)
  int begin(const char *lab) {
    if (ns >= NS) return -1;
    int k = ns++;
    int j = 0;
    for (const char *c = lab; *c && j < 15; c++) label[k][j++] = (char)toupper((unsigned char)*c);
    label[k][j] = 0;
    if (!j) { memcpy(label[k], "SNAP 1", 7); label[k][5] = (char)('1' + k); }
    for (int i = 0; i < n; i++) e[i].len[k] = 0;
    return k;
  }
  // discovery: a new identifier that answered (snapshot k)
  bool add(uint16_t did, int k, const uint8_t *d, int len) {
    if (n >= MAXE) return false;
    Entry &x = e[n++];
    memset(&x, 0, sizeof(x));
    x.did = did;
    set(x, k, d, len);
    return true;
  }
  static void set(Entry &x, int k, const uint8_t *d, int len) {
    if (len > MAXB) len = MAXB;
    if (len < 1) len = 0;
    x.len[k] = (uint8_t)len;
    memcpy(x.b[k], d, len);
  }
};

static bool same(const Entry &x, int a, int b) {
  return x.len[a] && x.len[a] == x.len[b] && !memcmp(x.b[a], x.b[b], x.len[a]);
}

// How well an identifier fits: -1 = no (it changed between snapshots with the same label, or
// it's the same everywhere, or an answer is missing); otherwise higher is better.
//   +100 each label group with two or more snapshots, all the same (proven steady)
//   +50  every label has its own value (all distinct)
//   +20  if only a few bytes differ (a mode is usually one byte)
static int score(const Table &t, const Entry &x, int *bytesDiff = nullptr) {
  if (t.ns < 2) return -1;
  for (int k = 0; k < t.ns; k++) if (!x.len[k]) return -1;
  int s = 0;
  bool groupDone[NS] = {false};
  int firstOf[NS]; int ng = 0;                    // one representative snapshot per label
  for (int k = 0; k < t.ns; k++) {
    if (groupDone[k]) continue;
    int members = 1;
    for (int j = k + 1; j < t.ns; j++) {
      if (strcmp(t.label[j], t.label[k])) continue;
      groupDone[j] = true; members++;
      if (!same(x, j, k)) return -1;              // changed on its own
    }
    if (members >= 2) s += 100;
    firstOf[ng++] = k;
  }
  if (ng < 2) return -1;                          // one label only: nothing to compare
  bool anyDiff = false, allDistinct = true;
  for (int a = 0; a < ng; a++) for (int b = a + 1; b < ng; b++) {
    if (same(x, firstOf[a], firstOf[b])) allDistinct = false; else anyDiff = true;
  }
  if (!anyDiff) return -1;
  if (allDistinct) s += 50;
  // bytes that differ between the first two labels
  int d = 0, k0 = firstOf[0], k1 = firstOf[1];
  int L = x.len[k0] > x.len[k1] ? x.len[k0] : x.len[k1];
  for (int i = 0; i < L; i++) if (i >= x.len[k0] || i >= x.len[k1] || x.b[k0][i] != x.b[k1][i]) d++;
  if (bytesDiff) *bytesDiff = d;
  if (d <= 2) s += 20;
  s -= d;                                         // fewer changed bytes first
  // a number that only moves by a count or two between modes is a sensor wobbling (seen on
  // the S5: 221188 went 15 / 16 with the mode by chance, and flaps in the same mode too)
  if (x.len[k0] == x.len[k1] && x.len[k0] <= 4) {
    long a = 0, b = 0;
    for (int i = 0; i < x.len[k0]; i++) { a = a * 256 + x.b[k0][i]; b = b * 256 + x.b[k1][i]; }
    long dv = a > b ? a - b : b - a;
    // (a small number like 0/1/2/3 is more likely a flag or a mode: those stay)
    if (dv <= 2 && (a < b ? a : b) >= 4) { s -= 200; if (s < 1) s = 1; }   // listed last, never green
  }
  return s;
}

// The best candidates, best first. Returns how many were written to idx.
static int rank(const Table &t, int *idx, int maxOut) {
  int m = 0, sc[64];
  if (maxOut > 64) maxOut = 64;
  for (int i = 0; i < t.n; i++) {
    int s = score(t, t.e[i]);
    if (s < 0) continue;
    int pos;
    if (m < maxOut) pos = m++;
    else if (s > sc[m - 1]) pos = m - 1;
    else continue;
    idx[pos] = i; sc[pos] = s;
    while (pos > 0 && sc[pos - 1] < sc[pos]) {     // bubble up (ties keep identifier order)
      int a = idx[pos]; idx[pos] = idx[pos - 1]; idx[pos - 1] = a;
      int b = sc[pos]; sc[pos] = sc[pos - 1]; sc[pos - 1] = b;
      pos--;
    }
  }
  return m;
}

static void hex(const Entry &x, int k, char *out, int outSize) {
  int o = 0;
  for (int i = 0; i < x.len[k] && o + 3 < outSize; i++) o += snprintf(out + o, outSize - o, "%02X", x.b[k][i]);
  if (!x.len[k]) snprintf(out, outSize, "-");
}

}  // namespace scanx
