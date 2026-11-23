// Desktop test for the identifier finder (firmware/S5GaugePod_Test/scan_logic.h), with the
// reply parser from obd_parse.h. A pretend ECU answers a few hundred identifiers: steady ones,
// sensors that drift, a counter, and one "drive select" byte that follows the mode. The finder
// must put the mode byte first and leave out everything that changes on its own.
// build: g++ -std=c++17 -O2 -Ifirmware/S5GaugePod_Test tools/scan_test.cpp -o scan_test && ./scan_test
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include "obd_parse.h"
#include "scan_logic.h"

static int fails = 0;
static void check(bool ok, const char *what) { printf("  %-62s %s\n", what, ok ? "ok" : "FAIL"); if (!ok) fails++; }

// The pretend ECU: reply text for "22xxxx" in mode m (0 comfort, 1 auto, 2 dynamic)
static std::mt19937 rng(3);
static int counter = 0;
static const uint16_t MODE_DID = 0x15A2, PARTIAL_DID = 0x2210, ECO_DID = 0x1777;
static std::string reply(uint16_t did, int mode) {
  char b[200];
  auto bytes = [&](std::initializer_list<int> v) {
    int o = snprintf(b, sizeof b, "62%04X", did);
    for (int x : v) o += snprintf(b + o, sizeof b - o, "%02X", x & 255);
    return std::string(b);
  };
  if (did % 13 != 0 && did != MODE_DID && did != PARTIAL_DID && did != ECO_DID) return "7F2231";   // not supported
  if (did == MODE_DID) return bytes({0x00, mode == 0 ? 1 : mode == 1 ? 2 : 4});                     // the one we want
  if (did == PARTIAL_DID) return bytes({mode == 2 ? 1 : 0});                                         // only Dynamic differs (shift map)
  if (did == ECO_DID) return bytes({mode == 0 ? 5 : 7, 0x10});                                       // Comfort vs the rest
  if (did % 39 == 0) return bytes({(int)(rng() % 256), (int)(rng() % 256)});                         // a sensor
  if (did % 65 == 0) { counter++; return bytes({counter >> 8, counter}); }                            // a counter
  if (did % 91 == 0) {                                                                                // long, multi-frame, steady
    snprintf(b, sizeof b, "00C\r0:62%04X414243\r1:44454647484950\r", did);
    return b;
  }
  return bytes({0x12, (int)(did & 255)});                                                            // steady
}

int main() {
  // ---- parser on the reply shapes the scan will see
  printf("reply parsing\n");
  uint8_t d[scanx::MAXB];
  check(obdp::parseMode22("62103801\r", 0x1038, d, scanx::MAXB) == 1 && d[0] == 1, "single byte");
  check(obdp::parseMode22("7F2231\r", 0x1038, d, scanx::MAXB) == 0, "not supported (7F 22 31): nothing");
  check(obdp::parseMode22("00C\r0:62105B414243\r1:44454647484950\r", 0x105B, d, scanx::MAXB) == 9 && d[8] == 0x49, "multi-frame, 9 bytes");
  check(obdp::parseMode22("6210FF0102\r", 0x1038, d, scanx::MAXB) == 0, "an answer for another identifier is ignored");

  // ---- a whole session: discovery in Comfort, then Dynamic, Comfort, Dynamic, Auto
  printf("finding the drive select byte\n");
  static scanx::Entry store[scanx::MAXE];
  scanx::Table t; t.e = store;
  int k = t.begin("Comfort");
  for (int did = 0x1000; did <= 0x2FFF; did++) {
    std::string r = reply((uint16_t)did, 0);
    int n = obdp::parseMode22(r.c_str(), (uint16_t)did, d, scanx::MAXB);
    if (n > 0) t.add((uint16_t)did, k, d, n);
  }
  printf("    %d identifiers answered\n", t.n);
  check(t.n > 500 && t.n < scanx::MAXE, "discovery kept the answering ones");
  const int modes[] = {2, 0, 2, 1};
  const char *labels[] = {"dynamic", "COMFORT", "Dynamic", "Auto"};
  for (int s = 0; s < 4; s++) {
    int kk = t.begin(labels[s]);
    for (int i = 0; i < t.n; i++) {
      std::string r = reply(t.e[i].did, modes[s]);
      int n = obdp::parseMode22(r.c_str(), t.e[i].did, d, scanx::MAXB);
      if (n > 0) scanx::Table::set(t.e[i], kk, d, n);
    }
  }
  int idx[40];
  int m = scanx::rank(t, idx, 40);
  printf("    %d candidates:", m);
  for (int i = 0; i < m && i < 6; i++) printf(" 22%04X(%d)", t.e[idx[i]].did, scanx::score(t, t.e[idx[i]]));
  printf("\n");
  check(m >= 1 && t.e[idx[0]].did == MODE_DID, "the mode byte comes first");
  check(m >= 1 && scanx::score(t, t.e[idx[0]]) >= 150, "and scores as steady within, different between modes");
  bool noise = false;
  for (int i = 0; i < m; i++) { uint16_t x = t.e[idx[i]].did; if (x != MODE_DID && x != PARTIAL_DID && x != ECO_DID && (x % 39 == 0 || x % 65 == 0)) noise = true; }
  check(!noise, "no sensor or counter in the list");
  bool partial = false;
  for (int i = 0; i < m; i++) if (t.e[idx[i]].did == PARTIAL_DID) partial = i > 0;
  check(partial, "a byte that only some modes change is listed, below it");
  { scanx::Table w; static scanx::Entry we[2]; w.e = we;          // a sensor wobbling 0x15 / 0x16 with the mode by chance
    int ka = w.begin("A"); uint8_t v15 = 0x15, v16 = 0x16; w.add(0x1188, ka, &v15, 1);
    int kb = w.begin("B"); scanx::Table::set(w.e[0], kb, &v16, 1);
    int kc = w.begin("A"); scanx::Table::set(w.e[0], kc, &v15, 1);
    int kd = w.begin("B"); scanx::Table::set(w.e[0], kd, &v16, 1);
    check(scanx::score(w, w.e[0]) < 100, "a one-count wobble (15 / 16) isn't ranked as steady"); }
  check(!strcmp(t.label[1], "DYNAMIC") && !strcmp(t.label[2], "COMFORT"), "labels match whatever the case");

  // ---- with each mode read only once, drifting values can't be ruled out: the mode byte
  //      must still be in the list
  printf("one reading per mode\n");
  scanx::Table u; static scanx::Entry st2[scanx::MAXE]; u.e = st2;
  int a = u.begin("Comfort");
  for (int did = 0x1000; did <= 0x2FFF; did++) { std::string r = reply((uint16_t)did, 0); int n = obdp::parseMode22(r.c_str(), (uint16_t)did, d, scanx::MAXB); if (n > 0) u.add((uint16_t)did, a, d, n); }
  int b2 = u.begin("Dynamic");
  for (int i = 0; i < u.n; i++) { std::string r = reply(u.e[i].did, 2); int n = obdp::parseMode22(r.c_str(), u.e[i].did, d, scanx::MAXB); if (n > 0) scanx::Table::set(u.e[i], b2, d, n); }
  m = scanx::rank(u, idx, 40);
  bool found = false; for (int i = 0; i < m; i++) if (u.e[idx[i]].did == MODE_DID) found = true;
  check(found, "the mode byte is in the list");

  printf("limits\n");
  scanx::Table v; static scanx::Entry st3[4]; v.e = st3;
  for (int i = 0; i < scanx::NS; i++) v.begin("x");
  check(v.begin("one too many") == -1, "a 7th reading is refused");

  if (fails) { printf("%d FAILED\n", fails); return 1; }
  printf("all identifier finder tests passed\n");
  return 0;
}
