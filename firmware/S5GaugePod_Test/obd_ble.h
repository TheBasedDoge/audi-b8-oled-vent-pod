// =====================================================================
//  S5 Gauge Pod - OBD over a Veepeak BLE (ELM327) adapter
//
//  A background task on core 0:
//    1. scans for the adapter (by saved address, else by name: "VEEPEAK", "OBD", ...)
//    2. connects and finds its serial service (FFF0 / FFE0, or any notify + write pair)
//    3. sets up the ELM327: ATZ, echo/spaces/headers off, CAN 11-bit 500k (ATSP6), fast timing
//    4. reads which PIDs the ECU supports, then polls in a loop:
//         fast:  RPM + speed in one request (010C0D; 010C11 / 010C if the car refuses)
//         slow:  coolant, charge-air temp (0177, else 010F), oil, timing, battery (ATRV),
//                MAP, baro, throttle, wideband AFR, cats, and the Audi 22 groups -
//                each on its own interval, most overdue first
//  Values land in obd::val[] with a timestamp; the display treats them as live for 3 s.
//  If the car stops answering (ignition off) it waits and retries; if the adapter drops,
//  it reconnects.
//
//  Carried over from the S2000 sketch (same Veepeak, known working there):
//    - bonding enabled; if the adapter refuses the notify subscription, pair and retry
//    - disconnect callback, so a dropped link aborts the wait at once
//    - short BLE connection interval (7.5-15 ms): every request is one write + one notify
//    - ELM setup that tolerates a missed reply, ATAT1 during setup then ATAT2
//    - the "1" response-count suffix (010C111): the adapter returns as soon as the ECU
//      answers instead of waiting for more ECUs; switched off if replies start failing
//    - ATI / ATDP captured, and a rolling adapter log (web UI OBD tab + [obd] serial lines)
//  S5 additions: requests go to the engine ECU only (ATSH7E0, falls back to the
//  broadcast 7DF), so the gearbox ECU doesn't answer too; ATSP6 with one ATSP0 retry.
// =====================================================================
#pragma once
#include <NimBLEDevice.h>
#include <stdarg.h>
#include "obd_parse.h"
#include "scan_logic.h"       // the identifier finder
#include <esp_heap_caps.h>
#ifndef OBD_MIN_RSSI
#define OBD_MIN_RSSI -80
#endif

namespace obd {

enum State : uint8_t { S_OFF, S_SCAN, S_CONNECT, S_INIT, S_RUN, S_NOCAR };
static const char *stateName(int s) {
  static const char *n[] = {"off", "searching", "connecting", "starting ELM327", "running", "no reply from car"};
  return (s >= 0 && s <= 5) ? n[s] : "?";
}

enum Field { F_RPM, F_THR, F_CLT, F_CAC, F_OIL, F_IAT, F_TIM, F_BAT, F_MAP, F_BARO, F_KMH, F_MAPCMD, F_KNOCK, F_PEDAL, F_BYPASS,
             F_HPFP, F_HPFPSET, F_LPFP, F_INJ, F_PRESC, F_ICPUMP, F_AFR1, F_AFR2, F_AFRCMD, F_AMB, F_CAT1, F_CAT2,
             F_GSPORT, F_GPROG,           // gearbox: sport/manual flag (221035), program (221026)
             F_GBT1, F_GBT2, F_GBT3,      // gearbox temperatures (221259 / 125A / 125B, A - 40 degC), degF
             F_DMODE,                     // sport differential's drive select setting (71D 22170E): 1 comfort, 2 auto, 3 dynamic
             F_DIFT1, F_DIFT2,            // sport differential temperatures? (71D 221306 / 221307, A - 40 degC), degF
             F_O2B1S2, F_O2B2S2,          // narrowband O2 after the cats, bank 1 / bank 2 (0115 / 0119), V
             F_EPB,                       // parking brake status (752 222B78): 0x33 released, 0x22 applied
             F_BRAKE1, F_BRAKE2,          // the two brake pedal switches (engine 2216CE / 2216CF): 1 pressed
             F_N };
// Odometer (engine 22121C, 4 bytes, km; matched the dash: 118,636 km = 73,717 mi)
static volatile uint32_t odoKm = 0, odoAt = 0;
// The car's own clock, from the steering (712, answers on 77C, 222BC1): the odometer (4 bytes, km),
// then year, month, day, second, minute, hour (26 10 20 / 55 20 18 = 2026-10-20 18:20:55, the
// car's local time). carLocal = that time counted as if it were UTC; carClockAt = millis() then.
static volatile uint32_t carLocal = 0, carClockAt = 0;
static volatile uint16_t carClockN = 0;          // bumped on every new reading
static int64_t daysFromCivil(int y, unsigned m, unsigned d) {   // days since 1970-01-01 (H. Hinnant)
  y -= m <= 2;
  const int64_t era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + (int64_t)doe - 719468;
}
// ECU self-tests (Mode 06): every test the ECU offers, read when the web page asks (a second or two;
// normal polling waits). The catalyst ones (0621 / 0622) are also read every 5 minutes for the
// Health tab's catalyst trend. Monitor status: 0101 (since codes were cleared) and 0141 (this drive),
// bytes A..D; for a petrol engine byte C = tests supported, byte D = tests not yet complete,
// bit 0 catalyst, bit 2 evap, bit 5 O2 sensor, bit 6 O2 heater.
static obdp::TestRec selfTab[48];
static volatile int selfN = 0;
static volatile uint32_t selfAt = 0;
static volatile bool selfReq = false, selfBusy = false;
static uint8_t monSince[4], monDrive[4];
static volatile bool monKnown = false;
static obdp::TestRec catRes[2];                  // latest catalyst results, bank 1 / bank 2
static volatile bool catHave[2] = {false, false};
static volatile uint16_t catN = 0;               // bumped on every catalyst reading
static portMUX_TYPE selfMux = portMUX_INITIALIZER_UNLOCKED;
static inline bool monDone(const uint8_t *m, int bit) { return (m[2] >> bit & 1) && !(m[3] >> bit & 1); }
// the car's local time now (0 = never read)
static inline uint32_t carNow() { return carLocal ? carLocal + (millis() - carClockAt) / 1000 : 0; }
static volatile float    val[F_N];
static volatile uint32_t valAt[F_N];            // millis() of the last reply (0 = never)
// Knock retard per cylinder (deg, 0 = none, negative = timing pulled), from Audi identifiers
// 2213F2..2213F7. val[F_KNOCK] = the worst cylinder recently, knockCyl = which (1-6).
static volatile float    knockDeg[6];
static volatile uint32_t knockAt[6];
static volatile int      knockCyl = 0;
// Called (on the OBD task) with RPM and road speed taken from the same reply, for the gear
static void (*onRpmSpeed)(float rpm, float kmh) = nullptr;
static bool speedInFast = true;                     // speed comes with RPM in the fast group (010C0D)
static inline bool fresh(int f, uint32_t maxAgeMs = 3000) { return valAt[f] && millis() - valAt[f] < maxAgeMs; }
// How old a reading may be and still count, about 2.5 times the slowest it arrives (at cruise or on
// boost, see the poll list and the OBD tab's table). Logs 33-38: the boost schedule and the pauses
// in pulls left several readings slower than the windows they had, so pages flickered to "--", the
// gear chip lost its S / M tab in pulls, and warnings saw gaps. Readings that wait during a pull (up
// to 20 s) get AGE_PAUSED. Everything takes its window from ageFor(), and tools/poll_check.py checks
// the windows against the poll list (run it after changing either).
static const uint32_t AGE_PEDAL = 3000,    // pedal / bypass: 1.1 s cruising, 1.1 s on boost
                      AGE_AFR = 3000,      // AFR and its target: 1.1 s / 0.55 s
                      AGE_TARGET = 5000,   // the ECU's boost target: 2.2 s / 0.55 s
                      AGE_FUEL = 5500,     // fuel pressures: 2.2 s / 1.3 s
                      AGE_TIMING = 8000,   // timing: 1.7 s / 2.6 s (worst gaps 2.0 / 3.2)
                      AGE_MAP = 8000,      // manifold pressure: 1.7 s / 3.3 s
                      AGE_SLOW = 15000,    // intake, injector, knock, cats, oil / pre-SC / IC pump: worst gaps up to ~6 s
                      AGE_PAUSED = 30000,  // coolant, throttle, outside air, gearbox, D / S / M, rear O2 (wait in pulls)
                      AGE_DIFF = 35000,    // sport diff temps: every 10 s, and wait in pulls
                      AGE_BAT = 40000,     // battery: every 10 s, and waits in pulls
                      AGE_BARO = 60000;    // barometer: every ~16 s, and waits in pulls (stale = 101.3 kPa assumed)
static uint32_t ageFor(int f) {                    // the window for each reading (web page, anything generic)
  switch (f) {
    case F_RPM: case F_KMH: return 1500;
    case F_PEDAL: case F_BYPASS: return AGE_PEDAL;
    case F_AFR1: case F_AFR2: case F_AFRCMD: return AGE_AFR;
    case F_MAPCMD: return AGE_TARGET;
    case F_HPFP: case F_HPFPSET: case F_LPFP: return AGE_FUEL;
    case F_TIM: return AGE_TIMING;
    case F_MAP: return AGE_MAP;
    case F_BARO: return AGE_BARO;
    case F_DIFT1: case F_DIFT2: return AGE_DIFF;
    case F_BAT: return AGE_BAT;
    case F_CLT: case F_THR: case F_AMB: case F_GSPORT: case F_GPROG: case F_GBT1: case F_GBT2: case F_GBT3:
    case F_DMODE: case F_O2B1S2: case F_O2B2S2: return AGE_PAUSED;
    case F_EPB: return 10000;
    case F_BRAKE1: case F_BRAKE2: return 4000;
    default: return AGE_SLOW;                      // intake, oil, knock, injector, pre-SC, IC pump, cats
  }
}
// The gearbox program, from the gearbox (7E1), checked on the S5 with the OBD console:
//   221035 = 00 in D, 01 in S or manual; 221026 = 06 in S, 07 in D or manual.
// (Dynamic drive select puts the gearbox in S, so Dynamic shows as S.) 'D', 'S', 'M', or 0 = not known.
// Drive select, as the sport differential (71D) has it: 1 Comfort, 2 Auto, 3 Dynamic (all checked on the S5).
// Individual reads as whatever its sport differential setting is. 0 = not known.
static inline int driveMode() {
  if (!fresh(F_DMODE, ageFor(F_DMODE))) return 0;
  int m = (int)val[F_DMODE];
  return m >= 1 && m <= 3 ? m : 0;
}
// Parking brake, from the electronic parking brake (752, answers on 7BC), 222B78: 0x33 released,
// 0x22 applied (checked on the S5, twice each). 1 = applied, 0 = released, -1 = not known / moving.
static inline int parkBrake() {
  if (!fresh(F_EPB, ageFor(F_EPB))) return -1;
  int v = (int)val[F_EPB];
  return v == 0x22 ? 1 : v == 0x33 ? 0 : -1;
}
// Brake pedal: the engine's two brake switches (2216CE, 2216CF: 01 pressed, 00 released, checked on
// the S5). 1 = pressed (either switch), 0 = released, -1 = not known.
static inline int brakeOn() {
  bool a = fresh(F_BRAKE1, ageFor(F_BRAKE1)), b = fresh(F_BRAKE2, ageFor(F_BRAKE2));
  if (!a && !b) return -1;
  return ((a && val[F_BRAKE1] > 0.5f) || (b && val[F_BRAKE2] > 0.5f)) ? 1 : 0;
}
// D / S / M from the gearbox: 221035 00 = D, 01 = S or M; 221026 06 = S, 07 = M (or D). Adaptive
// cruise in S switches to another shift program (log_0036: minutes of "M" at a steady 60-79 mph with
// the pedal at 0, upshifting to 7th), so only 07 means M; any other program with the sport flag is S.
static inline char transMode() {
  if (!fresh(F_GSPORT, ageFor(F_GSPORT)) || !fresh(F_GPROG, ageFor(F_GPROG))) return 0;
  if (val[F_GSPORT] < 0.5f) return 'D';
  return (int)val[F_GPROG] == 7 ? 'M' : 'S';
}

static volatile State state = S_OFF;
static volatile bool  enabled = true;
static volatile float injRpm = 0;            // RPM when the injector pulse width was read (0 = not known)
static volatile float senderPsi = -99;         // the analog sender's boost (set by the main loop; -99 = none): on boost the moment you are
static volatile bool  mapFast = false;          // boost is coming from OBD (no sender): manifold pressure as often as possible
static volatile float rate = 0;                 // replies per second
static volatile bool  cacFromIat = false;       // true if 0177 isn't supported and intake comes from 010F
static volatile bool  mapFrom70 = false;        // manifold pressure is coming from 0170 (so 010B is skipped)
static uint32_t       supportedMask[4];
static char devName[40] = "", devAddr[24] = "", lastMsg[80] = "";
static char elmVer[32] = "", proto[40] = "";
static volatile bool ecuHeader = false;         // true = ATSH7E0 (engine ECU only)
static volatile bool useSuffix = true;          // "1" response-count suffix on Mode 01 requests
static char nameFilter[48] = "VEEPEAK,OBD,VLINK,ELM";
static char savedAddr[24] = "";                 // remembered adapter (set by the sketch from Preferences)
static volatile bool addrLearned = false;       // sketch saves devAddr when this goes true

// raw console (web UI): one command at a time
static char consoleCmd[48], consoleReply[256];
static volatile bool consolePending = false, consoleDone = false;

// ---------------------------------------------------------------- adapter log
// Every command and reply goes into a ring the web UI shows (OBD tab). Lines logged
// with toSerial also print as "[obd] ..." on the USB serial monitor.
#define OBD_LOG_N   40
#define OBD_LOG_LEN 96
static char logBuf[OBD_LOG_N][OBD_LOG_LEN];
static volatile uint32_t logSeq = 0;            // total lines ever written (ring index = seq % N)
static portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;
static void logf(bool toSerial, const char *fmt, ...) {
  char tmp[OBD_LOG_LEN];
  int n = snprintf(tmp, sizeof(tmp), "%.1f ", millis() / 1000.0f);
  va_list ap; va_start(ap, fmt); vsnprintf(tmp + n, sizeof(tmp) - n, fmt, ap); va_end(ap);
  portENTER_CRITICAL(&logMux);
  strlcpy(logBuf[logSeq % OBD_LOG_N], tmp, OBD_LOG_LEN);
  logSeq = logSeq + 1;
  portEXIT_CRITICAL(&logMux);
  if (toSerial) Serial.printf("[obd] %s\n", tmp + n);
}
// Copy the last lines (oldest first) for the web UI. Returns the count.
static int logCopy(char (*out)[OBD_LOG_LEN], int maxN) {
  portENTER_CRITICAL(&logMux);
  uint32_t seq = logSeq;
  int n = (int)(seq < OBD_LOG_N ? seq : OBD_LOG_N);
  if (n > maxN) n = maxN;
  for (int i = 0; i < n; i++) strlcpy(out[i], logBuf[(seq - n + i) % OBD_LOG_N], OBD_LOG_LEN);
  portEXIT_CRITICAL(&logMux);
  return n;
}

// ---------------------------------------------------------------- transport
static char rxBuf[768];
static int  rxLen = 0;                          // guarded by rxMux
static portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t rxSem = nullptr;       // given on the '>' prompt (or a disconnect)
static NimBLEClient *client = nullptr;
static NimBLERemoteCharacteristic *txChr = nullptr, *rxChr = nullptr;
static bool txResponse = false;
static volatile bool linkLost = false;
static volatile bool verbose = true;            // log every command (off while polling fast)

static void onNotify(NimBLERemoteCharacteristic *, uint8_t *d, size_t n, bool) {
  bool gotPrompt = false;
  portENTER_CRITICAL(&rxMux);
  for (size_t i = 0; i < n; i++) {
    char c = (char)d[i];
    if (c == '>') { gotPrompt = true; continue; }
    if (rxLen < (int)sizeof(rxBuf) - 1) rxBuf[rxLen++] = c;
  }
  portEXIT_CRITICAL(&rxMux);
  if (gotPrompt) xSemaphoreGive(rxSem);
}

class ClientCB : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient *, int reason) override { linkLost = true; if (rxSem) xSemaphoreGive(rxSem); }
};
static ClientCB clientCB;

// Send one command, wait for the '>' prompt. out gets the reply (CR-separated lines,
// prompt removed). On a timeout it waits a little longer for the late prompt so the
// next command doesn't pick up this one's reply.
static bool request(const char *cmd, char *out, size_t outSize, uint32_t timeoutMs = 1000) {
  out[0] = 0;
  if (!client || !client->isConnected() || !txChr || linkLost) return false;
  xSemaphoreTake(rxSem, 0);
  portENTER_CRITICAL(&rxMux); rxLen = 0; portEXIT_CRITICAL(&rxMux);
  char b[64];
  int n = snprintf(b, sizeof(b), "%s\r", cmd);
  if (!txChr->writeValue((const uint8_t *)b, n, txResponse)) { logf(true, "> %s  (write failed)", cmd); return false; }
  bool ok = xSemaphoreTake(rxSem, pdMS_TO_TICKS(timeoutMs)) == pdTRUE && !linkLost;
  portENTER_CRITICAL(&rxMux);
  int L = rxLen; if (L > (int)outSize - 1) L = (int)outSize - 1;
  memcpy(out, rxBuf, L); out[L] = 0;
  portEXIT_CRITICAL(&rxMux);
  if (verbose || !ok) {
    char shown[72]; int k = 0;                    // one-line copy for the log
    for (int i = 0; out[i] && k < (int)sizeof(shown) - 1; i++) shown[k++] = (out[i] == '\r' || out[i] == '\n') ? '|' : out[i];
    shown[k] = 0;
    logf(false, "> %s  < %s%s", cmd, shown, ok ? "" : "  (timeout)");
  }
  if (!ok && !linkLost) xSemaphoreTake(rxSem, pdMS_TO_TICKS(300));   // swallow a late prompt
  return ok;
}

// ---------------------------------------------------------------- connect
static bool nameMatches(const std::string &name) {
  if (name.empty()) return false;
  char up[48]; size_t i = 0;
  for (; i < name.size() && i < sizeof(up) - 1; i++) up[i] = (char)toupper((unsigned char)name[i]);
  up[i] = 0;
  char f[48]; strlcpy(f, nameFilter, sizeof(f));
  for (char *tok = strtok(f, ","); tok; tok = strtok(nullptr, ",")) {
    while (*tok == ' ') tok++;
    for (char *c = tok; *c; c++) *c = (char)toupper((unsigned char)*c);
    if (*tok && strstr(up, tok)) return true;
  }
  return false;
}

static bool pickCharacteristics() {
  txChr = rxChr = nullptr;
  const char *known[][3] = {{"FFF0", "FFF2", "FFF1"}, {"FFE0", "FFE1", "FFE1"}};
  for (auto &k : known) {
    NimBLERemoteService *s = client->getService(k[0]);
    if (!s) continue;
    NimBLERemoteCharacteristic *t = s->getCharacteristic(k[1]), *r = s->getCharacteristic(k[2]);
    if (t && r && r->canNotify() && (t->canWrite() || t->canWriteNoResponse())) { txChr = t; rxChr = r; break; }
  }
  if (!txChr) {                                   // any service with a notify + a write characteristic
    for (NimBLERemoteService *s : client->getServices(true)) {
      std::string u = s->getUUID().toString();
      if (u.find("1800") != std::string::npos || u.find("1801") != std::string::npos || u.find("180a") != std::string::npos) continue;
      NimBLERemoteCharacteristic *t = nullptr, *r = nullptr;
      for (NimBLERemoteCharacteristic *c : s->getCharacteristics(true)) {
        if (!r && c->canNotify()) r = c;
        if (!t && (c->canWrite() || c->canWriteNoResponse())) t = c;
      }
      if (t && r) { txChr = t; rxChr = r; break; }
    }
  }
  if (!txChr) return false;
  txResponse = !txChr->canWriteNoResponse();
  bool sub = rxChr->subscribe(true, onNotify);
  if (!sub) {                                     // adapter wants an encrypted link: pair, then retry
    logf(true, "subscribe refused, pairing...");
    client->secureConnection();
    sub = rxChr->subscribe(true, onNotify);
  }
  if (!sub) logf(true, "could not subscribe to replies");
  return sub;
}

// Adapters given up on this power cycle (connected, but no car ever answered through them:
// typically one plugged into another car nearby). Cleared at restart.
static char ignored[4][24];
static int  nIgnored = 0;
static bool isIgnored(const std::string &a) {
  for (int i = 0; i < nIgnored; i++) if (!strcasecmp(ignored[i], a.c_str())) return true;
  return false;
}

static bool advertisesSerial(const NimBLEAdvertisedDevice *d) {
  return d->isAdvertisingService(NimBLEUUID((uint16_t)0xFFF0)) || d->isAdvertisingService(NimBLEUUID((uint16_t)0xFFE0));
}

static bool connectAdapter() {
  state = S_SCAN;
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(99);
  NimBLEScanResults res = scan->getResults(4000, false);
  // Saved address first (any signal strength). Otherwise the STRONGEST device whose name
  // matches, and only if it's close: in the car the adapter is a metre away (about -50 to
  // -70 dBm), so this won't grab another car's adapter in the garage or a neighbour's.
  // Last resort: something close advertising the FFF0 / FFE0 serial service.
  const NimBLEAdvertisedDevice *pick = nullptr, *bySvc = nullptr;
  int bestRssi = -999, svcRssi = -999, weak = 0;
  static uint32_t lastList = 0;
  bool listNow = !lastList || millis() - lastList > 60000;   // candidate list at most once a minute
  if (listNow) lastList = millis();
  for (int i = 0; i < res.getCount(); i++) {
    const NimBLEAdvertisedDevice *d = res.getDevice(i);
    int rssi = d->getRSSI();
    if (isIgnored(d->getAddress().toString())) continue;
    bool saved = savedAddr[0] && !strcasecmp(d->getAddress().toString().c_str(), savedAddr);
    bool named = nameMatches(d->getName()), svc = advertisesSerial(d);
    if ((named || svc || saved) && listNow)
      logf(true, "seen \"%s\" %s (%s) %d dBm%s%s", d->getName().empty() ? "(no name)" : d->getName().c_str(),
           d->getAddress().toString().c_str(), d->getAddress().getType() == BLE_ADDR_PUBLIC ? "public" : "random",
           rssi, svc ? " FFF0/FFE0" : "", saved ? " SAVED" : "");
    if (saved) { pick = d; bestRssi = 999; break; }
    if (named && rssi >= OBD_MIN_RSSI && rssi > bestRssi) { pick = d; bestRssi = rssi; }
    else if (named) weak++;
    if (!named && svc && rssi >= OBD_MIN_RSSI + 10 && rssi > svcRssi) { bySvc = d; svcRssi = rssi; }
  }
  if (!pick) pick = bySvc;
  if (!pick) {
    if (weak) snprintf(lastMsg, sizeof(lastMsg), "adapter-like device too far away (below %d dBm)", OBD_MIN_RSSI);
    else snprintf(lastMsg, sizeof(lastMsg), "no adapter found (%d BLE devices seen)", res.getCount());
    static uint32_t lastSay = 0;                  // don't flood the serial log while it searches
    if (!lastSay || millis() - lastSay > 30000) { lastSay = millis(); logf(true, "%s", lastMsg); }
    scan->clearResults(); return false;
  }
  strlcpy(devName, pick->getName().empty() ? "(no name)" : pick->getName().c_str(), sizeof(devName));
  strlcpy(devAddr, pick->getAddress().toString().c_str(), sizeof(devAddr));
  NimBLEAddress target = pick->getAddress();
  int pickRssi = pick->getRSSI();
  scan->clearResults();
  logf(true, "picked \"%s\" %s (%d dBm), connecting", devName, devAddr, pickRssi);
  state = S_CONNECT;
  if (!client) {
    client = NimBLEDevice::createClient();
    client->setClientCallbacks(&clientCB, false);
    client->setConnectTimeout(6000);
    // short connection interval (7.5-15 ms): each request is a write + a notify, so a long
    // interval adds tens of ms to every reply
    client->setConnectionParams(6, 12, 0, 300);
  }
  linkLost = false;
  if (!client->connect(target)) { snprintf(lastMsg, sizeof(lastMsg), "connect to %s failed", devAddr); logf(true, "%s", lastMsg); return false; }
  if (!pickCharacteristics()) { snprintf(lastMsg, sizeof(lastMsg), "no usable serial service on %s", devName); logf(true, "%s", lastMsg); client->disconnect(); return false; }
  bool fast = client->updateConnParams(6, 12, 0, 300);   // in case the adapter picked its own
  logf(true, "connected%s, write %s", fast ? " (fast interval requested)" : "", txResponse ? "with response" : "without response");
  return true;
}

// ---------------------------------------------------------------- ELM327 setup
static void trimTo(char *dst, size_t n, const char *src) {   // first non-empty line, trimmed
  while (*src == '\r' || *src == '\n' || *src == ' ') src++;
  size_t k = 0;
  while (src[k] && src[k] != '\r' && src[k] != '\n' && k < n - 1) { dst[k] = src[k]; k++; }
  dst[k] = 0;
}

// Same as the S2000: a missed reply to one setup command isn't fatal (some clones are slow
// after ATZ); only an adapter that answers nothing at all is.
static bool initElm() {
  state = S_INIT;
  char r[256];
  verbose = true;
  request("ATZ", r, sizeof(r), 3000);                 // reset
  vTaskDelay(pdMS_TO_TICKS(300));
  int answered = 0;
  const char *setup[] = {"ATE0", "ATL0", "ATS0", "ATH0", "ATAT1", "ATSP6"};
  for (const char *c : setup) answered += request(c, r, sizeof(r), 1000);
  if (request("ATI", r, sizeof(r), 1000)) trimTo(elmVer, sizeof(elmVer), r);
  if (!answered) { strlcpy(lastMsg, "adapter connected but not answering", sizeof(lastMsg)); logf(true, "%s", lastMsg); return false; }
  logf(true, "adapter %s ready", elmVer[0] ? elmVer : "(no ATI reply)");
  return true;
}

static bool readMask(int range, uint32_t timeoutMs) {
  char r[256], cmd[8];
  snprintf(cmd, sizeof(cmd), "01%02X", range * 0x20);
  if (!request(cmd, r, sizeof(r), timeoutMs) || obdp::classify(r) != obdp::OK) return false;
  obdp::PidVal v[4];
  int n = obdp::parseMode01(r, v, 4);
  for (int i = 0; i < n; i++) if (v[i].pid == range * 0x20) {
    supportedMask[range] = (uint32_t)v[i].b[0] << 24 | (uint32_t)v[i].b[1] << 16 | (uint32_t)v[i].b[2] << 8 | v[i].b[3];
    return true;
  }
  return false;
}

static bool triedAuto = false;                     // one ATSP0 attempt per adapter connection
static bool craSet = false, fcSet = false;         // the adapter's listen address / flow control are set for another module
static bool probeCar() {
  char r[128];
  verbose = true;
  memset(supportedMask, 0, sizeof(supportedMask));
  // The adapter's filters back to normal first. If a switch back from another module (gearbox, sport
  // diff, parking brake, steering) was ever missed, the adapter would still be listening to that
  // module only: no engine answer would get through and the pod would wait for "the car" until the
  // next key cycle.
  request("ATCRA", r, sizeof(r), 1000);             // no address = listen as before (ELM327 1.4b+)
  if (strchr(r, '?')) request("ATAR", r, sizeof(r), 1000);
  request("ATFCSM0", r, sizeof(r), 1000);
  craSet = fcSet = false;
  request("ATSH7DF", r, sizeof(r), 1000);           // broadcast for the first contact
  ecuHeader = false;
  bool ok = readMask(0, 3000);
  if (!ok && !triedAuto) {                          // not CAN 500k 11-bit? let the adapter search once
    triedAuto = true;
    logf(true, "no reply on ATSP6, trying automatic protocol search");
    request("ATSP0", r, sizeof(r), 1000);
    ok = readMask(0, 12000);
    if (!ok) request("ATSP6", r, sizeof(r), 1000);
  }
  if (!ok) return false;
  if (request("ATDP", r, sizeof(r), 1000)) trimTo(proto, sizeof(proto), r);
  // engine ECU only (7E0 -> answers on 7E8): the gearbox doesn't reply as well, so the "1"
  // suffix is safe and each reply comes straight back. 11-bit CAN only.
  if (request("ATDPN", r, sizeof(r), 1000) && (strchr(r, '6') || strchr(r, '8'))) {
    uint32_t keep = supportedMask[0];
    request("ATSH7E0", r, sizeof(r), 1000);
    if (readMask(0, 2000)) ecuHeader = true;
    else { request("ATSH7DF", r, sizeof(r), 1000); supportedMask[0] = keep; }
  }
  for (int range = 1; range < 4 && (supportedMask[range - 1] & 1); range++)   // bit 0 = next range exists
    if (!readMask(range, 2000)) break;
  cacFromIat = !obdp::supported(supportedMask, 0x77);
  request("ATAT2", r, sizeof(r), 1000);             // aggressive adaptive timing from here on
  useSuffix = ecuHeader;                           // with the broadcast the gearbox could answer first
  logf(true, "car OK: %s, %s, PIDs %08lX %08lX %08lX", proto, ecuHeader ? "engine ECU 7E0" : "broadcast 7DF",
       (unsigned long)supportedMask[0], (unsigned long)supportedMask[1], (unsigned long)supportedMask[2]);
  return true;
}

// ---------------------------------------------------------------- polling
// kind: 0 = Mode 01 PID(s), 1 = ATRV, 2 = Mode 22 identifiers on the engine ECU (Audi 3.0T, from
// Audizine's list confirmed on the same engine; dropped after 3 refusals on other cars),
// 3 = Mode 22 on the gearbox (header 7E1 for that request, then back to the engine),
// 4 = one Mode 22 identifier with a 4-byte answer (the odometer),
// 5 = Mode 22 on the sport differential (71D; single-frame answers, so no flow control needed),
// 6 = the car's clock from the steering (712; a 13-byte answer, so with flow control),
// 7 = the catalyst self-test results (Mode 06) and the monitor status,
// 8 = Mode 22 on the parking brake (752; single-frame answers).
// Mode 22 identifiers are asked up to three at a time (one CAN frame holds no more); if the ECU
// won't answer them together, that group falls back to one at a time.
struct Slow {
  const char *cmd; uint8_t pid; uint32_t periodMs; uint32_t last; uint8_t misses; uint8_t kind; uint16_t boostMs;
  uint8_t nd; uint16_t did[3]; uint8_t len[3];       // Mode 22: identifiers and their data lengths
  bool single; uint8_t gfail, shown;                  // asking one at a time / failed groups / raw replies logged
  // measured (OBD tab): how often it actually got an answer, at cruise [0] and on boost [1]
  uint32_t okAt = 0; float gotMs[2] = {0, 0};
  uint8_t failShown = 0;                              // failed replies written to the adapter log (other modules)
  bool dropTold = false;                              // "not answered" logged for this run of misses
};
// What waits while you're on boost: nothing the watcher or the pull summary uses. They catch up
// as soon as the pull ends (the most overdue goes first), and never wait more than PULL_PAUSE_MAX.
static const uint32_t PULL_PAUSE_MAX = 20000;
// Not the brake pedal or the parking brake: they're only read at a standstill (stoppedOnly), and
// a launch-control hold (stopped, pedal down) counts as "on boost" here, which is exactly when
// launch detection needs them (otherwise they went stale after 4 / 10 s of a hold).
static bool pullPaused(const Slow &s) {
  switch (s.kind) {
    case 1: case 3: case 4: case 5: case 6: case 7: return true;    // battery, gearbox, odometer, diff, clock, cat tests
    case 0: return s.pid == 0x05 || s.pid == 0x33 || s.pid == 0x15 || s.pid == 0x11;   // coolant, barometer, rear O2, throttle plate
  }
  return false;
}
// Only read when (nearly) stopped: the brake pedal and the parking brake are for launch detection.
static bool stoppedOnly(const Slow &s) { return s.kind == 8 || (s.kind == 2 && s.did[0] == 0x16CE); }
static uint32_t fastOkAt = 0; static float fastGotMs[2] = {0, 0};   // the RPM + speed request, measured the same way
static inline void gotEma(uint32_t &okAt, float *got, int b, uint32_t now) {
  if (okAt) { float dt = (float)(now - okAt); got[b] = got[b] > 0 ? got[b] + 0.2f * (dt - got[b]) : dt; }
  okAt = now;
}
// Request budget: about 8-9 of these a second (one per round, between the RPM + speed requests;
// the adapter manages about 17 answers a second in all). The periods ask for more than that, so
// everything is shared out in proportion: the next one is the most overdue as a share of its own
// period. On boost (boostMs) the list is re-weighted for the pull: AFR, knock and the boost target
// first, fuel pressure and pedal next; timing, manifold pressure, injector and intake slower (a
// boostMs above periodMs means slower on boost); what nothing uses in a pull waits (pullPaused).
static Slow slowList[] = {
  {"0105", 0x05, 4000, 0, 0, 0, 0},                   // coolant (slow to change)
  {"0177", 0x77, 1500, 0, 0, 0, 3000},                // charge air cooler temp (post-intercooler intake)
  {"010F", 0x0F, 1000, 0, 0, 0, 500},                 // intake air temp (used when 0177 isn't there)
  {"015C", 0x5C, 3000, 0, 0, 0, 0},                   // oil (standard PID; the S5 doesn't have it)
  // (throttle plate 0111 isn't read on its own any more: on the 3.0T it sits nearly wide open whenever
  // the supercharger works, so it says little; the Engine page's bar shows the pedal instead. It still
  // comes with RPM (010C11) on a car where speed can't.)
  {"010E", 0x0E, 1500, 0, 0, 0, 2000},                // timing (System page, pull summary)
  {"ATRV", 0,    10000, 0, 0, 1, 0},                  // battery at the OBD port (slow to change)
  {"0170", 0x70, 1500, 0, 0, 0, 2500},                // manifold pressure (the B8.5 S5 has no 010B): idle trim and the
                                                      // sender's stand-in (every 120 ms then, see mapFast)
  {"01343844", 0x34, 1000, 0, 0, 0, 400},             // AFR bank 1, bank 2 (wideband, lambda x 14.7) and commanded
  {"01463C3D", 0x46, 3000, 0, 0, 0, 4000},            // outside air, catalyst bank 1 and 2
  // pedal (x 0.0977 %), supercharger bypass (x 0.0256 deg)
  {"2210702982", 0, 1000, 0, 0, 2, 800, 2, {0x1070, 0x2982}, {2, 2}},
  // the ECU's boost target (manifold, x 0.0829 hPa) on its own: one frame, so a quick answer, and
  // the target arrow on the Boost / Overview pages runs on it
  {"222029", 0, 2000, 0, 0, 2, 450, 1, {0x2029}, {2}},   // (the arrow only shows on boost)
  // fuel: high-pressure pump actual and target, low-pressure pump (x 2.6534 hPa each)
  {"22130E130F1321", 0, 2000, 0, 0, 2, 1000, 3, {0x130E, 0x130F, 0x1321}, {2, 2, 2}},
  // injector pulse width, cylinder 1 (x 0.004 ms)
  {"2213A5", 0, 4000, 0, 0, 2, 3000, 1, {0x13A5}, {2}},
  // oil (A - 40 degC), intake before the supercharger (A x 0.75 - 48 degC), intercooler pump (x 0.0015 %)
  {"2220A1100311D6", 0, 5000, 0, 0, 2, 4000, 3, {0x20A1, 0x1003, 0x11D6}, {1, 1, 2}},
  // knock retard, cylinders 1-3 and 4-6: (A*256+B) x 0.375 - 48 deg
  // knock: ONE byte each on this car (0x80 = no retard), seen in the adapter log: 6213F2 80 13F3 80 13F4 80
  {"2213F213F313F4", 0, 3000, 0, 0, 2, 550, 3, {0x13F2, 0x13F3, 0x13F4}, {1, 1, 1}},
  {"2213F513F613F7", 0, 3000, 0, 0, 2, 550, 3, {0x13F5, 0x13F6, 0x13F7}, {1, 1, 1}},
  {"010B", 0x0B, 400, 0, 0, 0, 0},                    // manifold pressure, standard PID (used when 0170 isn't there)
  {"0133", 0x33, 15000, 0, 0, 0, 0},                  // barometric pressure (slow to change; waits in pulls)
  {"010D", 0x0D, 500, 0, 0, 0, 0},                    // speed (only when it can't come with RPM)
  // gearbox (7E1): sport/manual flag and shift program, for the D / S / M indicator
  {"2210351026", 0, 2000, 0, 0, 3, 0, 2, {0x1035, 0x1026}, {1, 1}},   // every 2 s: each costs two header switches
  // gearbox (7E1): three temperatures. 1259 is the clutch oil (log_0038: +30-38 degF per standing
  // launch, the other two 2-3), 125A / 125B the gearbox / mechatronic.
  // sensors (A - 40 degC: about 100 degC after a drive). Not confirmed which is which yet.
  {"221259125A125B", 0, 5000, 0, 0, 3, 0, 3, {0x1259, 0x125A, 0x125B}, {1, 1, 1}},
  // (drive select, 71D 22170E, was read here every 3 s: taken out. Each read cost a switch to the diff
  // and back, about 7 % of the adapter's time at cruise, for a pop-up. Still decoded if asked from the
  // console.)
  // two readings that look like the sport differential's temperatures: every 10 s, one at a time
  // (each answer fits one frame; together they wouldn't)
  {"221306", 0, 10000, 0, 0, 5, 0, 2, {0x1306, 0x1307}, {1, 1}, true},
  // the car's clock (steering, 712): at the start of each drive, then every 10 minutes
  {"222BC1", 0, 60000, 0, 0, 6, 0, 1, {0x2BC1}, {10}},
  // brake pedal switches (engine): once a second
  {"2216CE16CF", 0, 1000, 0, 0, 2, 0, 2, {0x16CE, 0x16CF}, {1, 1}},
  // parking brake (752): one byte, every 3 s
  {"222B78", 0, 3000, 0, 0, 8, 0, 1, {0x2B78}, {1}},
  // rear O2 sensors (after the cats), both in one request: every 3 s
  {"011519", 0x15, 3000, 0, 0, 0, 0},                // (the log only)
  // the ECU's catalyst test results (Mode 06) and monitor status: every 5 minutes
  {"0621", 0, 300000, 0, 0, 7, 0},
  // odometer, engine (4 bytes, km): once a minute
  {"22121C", 0, 60000, 0, 0, 4, 0, 1, {0x121C}, {4}},
};

static void knockStore(int cyl, uint16_t raw) {   // cyl 0-5; raw = the one byte A (0x80 = 0 deg)
  float deg = raw * 0.375f - 48.0f;
  if (deg > 1.0f || deg < -30.0f) {               // not a believable retard: ignore, but say so (a few times)
    static int told = 0;
    if (told++ < 6) logf(true, "knock cyl %d: raw 0x%04X = %.1f deg, outside 0 to -30: ignored", cyl + 1, raw, deg);
    return;
  }
  knockDeg[cyl] = deg; knockAt[cyl] = millis();
  float worst = 1; int wc = 0;
  for (int c = 0; c < 6; c++)
    if (knockAt[c] && millis() - knockAt[c] < 4000 && knockDeg[c] < worst) { worst = knockDeg[c]; wc = c + 1; }
  if (wc) { val[F_KNOCK] = worst; valAt[F_KNOCK] = millis(); knockCyl = wc; }
}

// One Mode 22 value (raw = A, or A*256+B) into its field, if it's believable
static bool store22(uint16_t id, uint16_t raw) {
  if (id >= 0x13F2 && id <= 0x13F7) { knockStore(id - 0x13F2, raw); return true; }
  const float HPA_PSI = 2.6534f * 0.0145038f;
  int f = -1; float x = 0, lo = -1e9f, hi = 1e9f;
  switch (id) {
    case 0x20A1: f = F_OIL;    x = (raw - 40) * 1.8f + 32;           lo = -40;  hi = 330;  break;   // degF
    case 0x2029: f = F_MAPCMD; x = raw * 0.00829f;                   lo = 10;   hi = 400;  break;   // kPa abs
    case 0x1070: f = F_PEDAL;  x = raw * 0.0977f;                    lo = 0;    hi = 105;  break;   // %
    case 0x2982: f = F_BYPASS; x = raw * 0.0256f;                    lo = 0;    hi = 110;  break;   // deg
    case 0x130E: f = F_HPFP;   x = raw * HPA_PSI;                    lo = 0;    hi = 4500; break;   // psi
    case 0x130F: f = F_HPFPSET; x = raw * HPA_PSI;                   lo = 0;    hi = 4500; break;
    case 0x1321: f = F_LPFP;   x = raw * HPA_PSI;                    lo = 0;    hi = 300;  break;
    case 0x13A5: f = F_INJ;    x = raw * 0.004f;                     lo = 0;    hi = 60;   break;   // ms
    case 0x1003: f = F_PRESC;  x = (raw * 0.75f - 48) * 1.8f + 32;  lo = -40;  hi = 330;  break;   // degF
    case 0x11D6: f = F_ICPUMP; x = raw * 0.0015f;                    lo = 0;    hi = 101;  break;   // %
    case 0x1035: f = F_GSPORT; x = raw;                              lo = 0;    hi = 1;    break;   // gearbox
    case 0x1026: f = F_GPROG;  x = raw;                              lo = 0;    hi = 255;  break;   // gearbox
    case 0x1259: f = F_GBT1;   x = (raw - 40) * 1.8f + 32;           lo = -40;  hi = 400;  break;   // degF
    case 0x125A: f = F_GBT2;   x = (raw - 40) * 1.8f + 32;           lo = -40;  hi = 400;  break;
    case 0x125B: f = F_GBT3;   x = (raw - 40) * 1.8f + 32;           lo = -40;  hi = 400;  break;
    case 0x170E: f = F_DMODE;  x = raw;                              lo = 0;    hi = 15;   break;   // sport diff
    case 0x1306: f = F_DIFT1;  x = (raw - 40) * 1.8f + 32;           lo = -40;  hi = 400;  break;   // degF
    case 0x1307: f = F_DIFT2;  x = (raw - 40) * 1.8f + 32;           lo = -40;  hi = 400;  break;
    case 0x2B78: f = F_EPB;    x = raw;                              lo = 0;    hi = 255;  break;   // parking brake
    case 0x16CE: f = F_BRAKE1; x = raw;                              lo = 0;    hi = 1;    break;   // brake pedal
    case 0x16CF: f = F_BRAKE2; x = raw;                              lo = 0;    hi = 1;    break;
    default: return false;
  }
  if (x < lo || x > hi) {
    static uint8_t told[F_N]; uint8_t &t = told[f];   // a few times per reading
    if (t++ < 3) logf(true, "22%04X: raw 0x%04X = %.1f is out of range: ignored", id, raw, x);
    return false;
  }
  val[f] = x; valAt[f] = millis();
  if (id == 0x13A5) injRpm = fresh(F_RPM, 400) ? val[F_RPM] : 0;   // duty = pulse x the RPM at that moment
  if (id == 0x1026 || id == 0x170E) {               // gearbox program / drive select: each new value once (decoding)
    static uint32_t seen[2][8];
    uint32_t *m = seen[id == 0x170E]; uint8_t v = (uint8_t)raw;
    if (!(m[v >> 5] & (1u << (v & 31)))) { m[v >> 5] |= 1u << (v & 31); logf(true, "22%04X: new value 0x%02X", id, v); }
  }
  return true;
}

// A Mode 22 group: all its identifiers in one request, or one at a time if the ECU won't
// Mode 22 with the "1" response-count suffix when the whole answer fits one CAN frame (62, then
// each identifier and its data: 7 bytes at most), so the adapter returns the moment it arrives
// instead of waiting out its timeout. If an answer only comes without it, a few times, it's
// turned off for Mode 22 (Mode 01 keeps its own switch).
static volatile bool suffix22 = true;
static uint8_t suffix22Bad = 0;
static bool request22(const char *cmd, char *r, size_t n, uint32_t timeout, int replyBytes, bool sfxOk = true) {
  if (!(sfxOk && useSuffix && suffix22 && replyBytes <= 7)) return request(cmd, r, n, timeout);
  char c[40]; snprintf(c, sizeof(c), "%s1", cmd);
  if (request(c, r, n, timeout) && obdp::classify(r) == obdp::OK) { suffix22Bad = 0; return true; }
  if (!request(cmd, r, n, timeout)) return false;    // once more without it
  if (obdp::classify(r) == obdp::OK && ++suffix22Bad >= 3) {
    suffix22 = false; logf(true, "Mode 22 answers only come without the \"1\" suffix - turning it off for them");
  }
  return true;
}
static int reply22(const Slow &s, int from, int count) { int b = 1; for (int i = from; i < from + count; i++) b += 2 + s.len[i]; return b; }

// Answers too long for one CAN frame (AFR, knock, fuel, the cat temps...): the count suffix is the
// number of frames (the first holds 6 bytes, each after it 7), so the adapter returns as the last
// one arrives instead of waiting out its timeout. Clones don't all count the same way, so it's
// checked two ways: an answer that comes back short is asked again without it (three of those and
// it's off), and every 20th request goes without it to compare times (no faster: off).
static volatile bool suffixMulti = true;
static uint8_t multiBad = 0;
static uint32_t multiCalls = 0;
static float msSfx = 0, msPlain = 0;               // average reply time with / without (OBD tab)
static uint16_t nSfx = 0, nPlain = 0;
static inline int framesFor(int bytes) { return bytes <= 7 ? 1 : 1 + (bytes - 6 + 6) / 7; }
static bool multiPick() { return useSuffix && suffixMulti && (++multiCalls % 20) != 0; }
static void multiTime(bool sfx, uint32_t ms) {
  float &m = sfx ? msSfx : msPlain; uint16_t &n = sfx ? nSfx : nPlain;
  m = n ? m + 0.1f * ((float)ms - m) : (float)ms;
  if (n < 60000) n++;
  if (suffixMulti && nPlain >= 5 && nSfx >= 40 && msSfx > msPlain * 1.05f) {
    suffixMulti = false;
    logf(true, "frame-count suffix: %.0f ms a reply vs %.0f ms without - no faster, turned off", msSfx, msPlain);
  }
}
static void multiShort() {                          // a reply came back short with it, complete without
  if (suffixMulti && ++multiBad >= 3) { suffixMulti = false; logf(true, "frame-count suffix cuts long answers short on this adapter - turned off"); }
}

static bool runGroup22(Slow &s) {
  char r[200];
  uint16_t raw[3];
  int got = 0;
  // the engine and gearbox answer on the powertrain bus; the sport diff and parking brake come through
  // the gateway, slower and less predictable: they're asked plainly (no count suffix)
  const bool sfxOk = s.kind == 2 || s.kind == 3;
  if (!s.single) {
    int bytes = reply22(s, 0, s.nd);
    bool ok, sfx = false;
    uint32_t t0 = millis();
    if (bytes <= 7) ok = request22(s.cmd, r, sizeof(r), 1000, bytes, sfxOk);
    else {
      sfx = sfxOk && multiPick();
      char c[40]; snprintf(c, sizeof(c), sfx ? "%s%d" : "%s", s.cmd, framesFor(bytes));
      ok = request(c, r, sizeof(r), 1000);
    }
    uint32_t dt = millis() - t0;
    if (bytes > 7 && sfx && ok && obdp::classify(r) == obdp::OK && obdp::parseMode22List(r, s.did, s.len, s.nd, raw) < s.nd) {
      char r2[200];                                  // short with the count: once more without it
      if (request(s.cmd, r2, sizeof(r2), 1000) && obdp::classify(r2) == obdp::OK &&
          obdp::parseMode22List(r2, s.did, s.len, s.nd, raw) > obdp::parseMode22List(r, s.did, s.len, s.nd, raw)) {
        multiShort(); strlcpy(r, r2, sizeof(r));
      }
    } else if (bytes > 7 && ok) multiTime(sfx, dt);
    if (s.shown < 2) {                               // the first replies of each group, as they came (OBD tab)
      s.shown++;
      char t[72]; strlcpy(t, ok ? r : "(no reply)", sizeof(t));
      for (char *c = t; *c; c++) if (*c == '\r' || *c == '\n') *c = '|';
      logf(true, "%s -> %s", s.cmd, t);
    }
    if (ok && obdp::classify(r) == obdp::OK) {
      got = obdp::parseMode22List(r, s.did, s.len, s.nd, raw);
      for (int i = 0; i < got; i++) store22(s.did[i], raw[i]);
    }
    if (got == s.nd) { s.gfail = 0; return true; }
    if (s.kind >= 3 && s.failShown < 4) {            // other modules: say what came back (OBD tab)
      s.failShown++;
      char t[72]; strlcpy(t, ok ? r : "(no reply)", sizeof(t));
      for (char *c = t; *c; c++) if (*c == '\r' || *c == '\n') *c = '|';
      logf(true, "%s failed: %s", s.cmd, t);
    }
    if (s.nd > 1 && ++s.gfail >= 2) { s.single = true; logf(true, "%s: the ECU won't answer %d at once - asking one by one", s.cmd, s.nd); }
    if (got) return true;
    if (!s.single) return false;
  }
  bool any = false;
  for (int i = 0; i < s.nd; i++) {
    char c[12]; snprintf(c, sizeof(c), "22%04X", s.did[i]);
    uint8_t b[4];
    int n;
    if (request22(c, r, sizeof(r), 1000, reply22(s, i, 1), sfxOk) && obdp::classify(r) == obdp::OK && (n = obdp::parseMode22(r, s.did[i], b, 4)) >= s.len[i])
      any |= store22(s.did[i], s.len[i] == 1 ? b[0] : (uint16_t)(b[0] << 8 | b[1]));
  }
  return any;
}

static void store(const obdp::PidVal &v) {
  float x = obdp::value(v);
  int f = -1;
  switch (v.pid) {
    case 0x0C: f = F_RPM; break;  case 0x11: f = F_THR; break;  case 0x05: f = F_CLT; break;
    case 0x77: f = F_CAC; break;  case 0x0F: f = F_IAT; break;  case 0x5C: f = F_OIL; break;
    case 0x0E: f = F_TIM; break;  case 0x0B: f = F_MAP; break;  case 0x33: f = F_BARO; break;
    case 0x0D: f = F_KMH; break;
    case 0x34: f = F_AFR1; break;  case 0x38: f = F_AFR2; break;  case 0x44: f = F_AFRCMD; break;
    case 0x46: f = F_AMB; break;   case 0x3C: f = F_CAT1; break;  case 0x3D: f = F_CAT2; break;
    case 0x15: f = F_O2B1S2; break; case 0x19: f = F_O2B2S2; break;
  }
  if (v.pid == 0x70) {                             // boost: actual and commanded manifold pressure
    float k;
    if (obdp::boostActual(v, k))  { val[F_MAP] = k; valAt[F_MAP] = millis(); }
    if (obdp::boostCommand(v, k)) { val[F_MAPCMD] = k; valAt[F_MAPCMD] = millis(); }
    return;
  }
  if (f >= 0) { val[f] = x; valAt[f] = millis(); }
}

static bool runMode01(const char *cmd, uint32_t timeout = 1000, bool suffixOk = true) {
  char r[256], c[24];
  // "010C111": return after one ECU answers. A reply longer than one CAN frame gets the number of
  // frames instead (see framesFor), checked and timed by the multi-frame helpers.
  int bytes = 1, expect = 0;                         // reply size: 41, then each PID and its data
  for (const char *q = cmd + 2; q[0] && q[1]; q += 2) {
    uint8_t pid = (uint8_t)(obdp::hexv(q[0]) * 16 + obdp::hexv(q[1]));
    bytes += 1 + obdp::pidLen(pid);
    if (obdp::supported(supportedMask, pid)) expect++;   // the ECU leaves out what it doesn't have
  }
  bool multi = bytes > 7, sfx = multi ? multiPick() : (useSuffix && suffixOk);
  snprintf(c, sizeof(c), sfx ? "%s%d" : "%s", cmd, framesFor(bytes));
  uint32_t t0 = millis();
  if (!request(c, r, sizeof(r), timeout)) return false;
  uint32_t dt = millis() - t0;
  if (obdp::classify(r) != obdp::OK) return false;
  obdp::PidVal v[6];
  int n = obdp::parseMode01(r, v, 6);
  if (multi && sfx && n < expect) {                  // short with the count: once more without it
    char r2[256]; obdp::PidVal v2[6]; int n2 = 0;
    if (request(cmd, r2, sizeof(r2), timeout) && obdp::classify(r2) == obdp::OK) n2 = obdp::parseMode01(r2, v2, 6);
    if (n2 > n) { multiShort(); n = n2; for (int i = 0; i < n; i++) v[i] = v2[i]; }
  } else if (multi) multiTime(sfx, dt);
  for (int i = 0; i < n; i++) store(v[i]);
  return n > 0;
}

// ---------------------------------------------------------------- other modules
// The engine (7E0) and gearbox (7E1) answer on 7E8 / 7E9, which the adapter listens to anyway.
// Other modules on this car (UDS, the VW way) answer on their address + 0x6A: the gateway asks on
// 710 and answers on 77A, ABS 713 / 77D, the instrument cluster 714 / 77E. For those the adapter
// has to be told where to listen (ATCRA) and where to send the "go on" frame of a long answer
// (ATFCSH). Modules that still use VW's older TP2.0 protocol can't be reached through an ELM327.
static bool isStd(uint16_t h) { return h >= 0x7E0 && h <= 0x7E7; }
static uint16_t replyAddr(uint16_t h) { return isStd(h) ? h + 8 : h + 0x6A; }
static void setTarget(uint16_t h, bool longReplies = true) {
  char c[24], r[48];
  snprintf(c, sizeof(c), "ATSH%03X", h); request(c, r, sizeof(r), 1000);
  if (!isStd(h) || craSet) { snprintf(c, sizeof(c), "ATCRA%03X", replyAddr(h)); request(c, r, sizeof(r), 1000); craSet = true; }
  if (!isStd(h) && longReplies) {
    snprintf(c, sizeof(c), "ATFCSH%03X", h); request(c, r, sizeof(r), 1000);
    request("ATFCSD300000", r, sizeof(r), 1000);
    request("ATFCSM1", r, sizeof(r), 1000); fcSet = true;
  }
}
static void restoreTarget() {
  char r[48];
  request(ecuHeader ? "ATSH7E0" : "ATSH7DF", r, sizeof(r), 1000);
  if (craSet) {
    request("ATCRA", r, sizeof(r), 1000);         // no address = listen as before (ELM327 1.4b+)
    if (strchr(r, '?')) request("ATCRA7E8", r, sizeof(r), 1000);   // an adapter without it: the engine
    else craSet = false;
  }
  if (fcSet) { request("ATFCSM0", r, sizeof(r), 1000); fcSet = false; }
}
static bool parseAddr(const char *c, uint16_t &h) {
  if (!isxdigit((unsigned char)c[0]) || !isxdigit((unsigned char)c[1]) || !isxdigit((unsigned char)c[2])) return false;
  char b[4] = {c[0], c[1], c[2], 0};
  h = (uint16_t)strtol(b, nullptr, 16);
  return h >= 0x600 && h <= 0x7EF && h != 0x7DF;
}

// ---------------------------------------------------------------- ECU self-tests (Mode 06)
static void readMonitors() {
  char r[96]; obdp::PidVal v[2];
  bool a = request("0101", r, sizeof(r), 1000) && obdp::parseMode01(r, v, 2) >= 1 && v[0].pid == 0x01;
  uint8_t s1[4] = {0}, s2[4] = {0};
  if (a) memcpy(s1, v[0].b, 4);
  bool b = request("0141", r, sizeof(r), 1000) && obdp::parseMode01(r, v, 2) >= 1 && v[0].pid == 0x41;
  if (b) memcpy(s2, v[0].b, 4);
  portENTER_CRITICAL(&selfMux);
  if (a) memcpy(monSince, s1, 4);
  if (b) memcpy(monDrive, s2, 4);
  if (a && b) monKnown = true;
  portEXIT_CRITICAL(&selfMux);
}
static bool readCats() {
  char r[200]; obdp::TestRec t[6]; bool any = false;
  for (int bank = 0; bank < 2; bank++) {
    const char *c = bank ? "0622" : "0621";
    int n = request(c, r, sizeof(r), 1000) && obdp::classify(r) == obdp::OK ? obdp::parseMode06(r, t, 6) : 0;
    for (int i = 0; i < n; i++) if (t[i].mid == (bank ? 0x22 : 0x21)) {
      portENTER_CRITICAL(&selfMux); catRes[bank] = t[i]; catHave[bank] = true; portEXIT_CRITICAL(&selfMux);
      any = true; break;
    }
  }
  readMonitors();
  if (any) catN = catN + 1;
  return any;
}
static void runSelfTests() {
  selfBusy = true;
  char r[400];
  uint8_t mids[96]; int nm = 0;
  for (int base = 0; base <= 0xE0; base += 0x20) {            // which monitor IDs this ECU has
    char c[8]; snprintf(c, sizeof(c), "06%02X", base);
    uint32_t m = 0;
    if (!request(c, r, sizeof(r), 1000) || !obdp::parseMode06Mask(r, (uint8_t)base, m)) break;
    for (int k = 0; k < 31 && nm < (int)sizeof(mids); k++) if (m >> (31 - k) & 1) mids[nm++] = (uint8_t)(base + 1 + k);
    if (!(m & 1)) break;                                       // no further range
  }
  static obdp::TestRec tmp[48]; int n = 0;
  for (int i = 0; i < nm && n < 48; i++) {
    char c[8]; snprintf(c, sizeof(c), "06%02X", mids[i]);
    if (request(c, r, sizeof(r), 1500) && obdp::classify(r) == obdp::OK) n += obdp::parseMode06(r, tmp + n, 48 - n);
  }
  readMonitors();
  portENTER_CRITICAL(&selfMux);
  memcpy(selfTab, tmp, sizeof(obdp::TestRec) * n); selfN = n; selfAt = millis();
  for (int i = 0; i < n; i++) if (tmp[i].mid == 0x21 || tmp[i].mid == 0x22) { catRes[tmp[i].mid - 0x21] = tmp[i]; catHave[tmp[i].mid - 0x21] = true; }
  portEXIT_CRITICAL(&selfMux);
  catN = catN + 1;
  logf(true, "self-tests: %d monitors, %d results", nm, n);
  selfBusy = false;
}

// What the console may send while the car is moving: reading only. Anything that changes the car or
// the adapter (clear codes 04, sessions 10, ECU reset 11, writes 2E, actuator tests 2F, routines 31,
// and AT settings, which stay set and can stop the polling) waits until it's stopped. Speed not
// known (no car answering, the bench) counts as stopped. Returns nullptr if it may go, else why not.
static const char *consoleCheck(const char *cmd) {
  if (!(fresh(F_KMH, 3000) && val[F_KMH] > 3)) return nullptr;
  char b[48]; size_t n = 0;
  bool prefixed = strlen(cmd) > 4 && cmd[3] == ':';
  const char *p = prefixed ? cmd + 4 : cmd;
  for (; *p && n < sizeof(b) - 1; p++) if (prefixed || *p != ' ') b[n++] = *p;   // without a prefix it's one command, as typed
  b[n] = 0;
  char *save = nullptr;
  for (char *t = strtok_r(b, " ,", &save); t; t = strtok_r(nullptr, " ,", &save)) {
    bool y = false;
    if (t[0] == 'A' && t[1] == 'T') {
      static const char *ok[] = {"ATRV", "ATI", "ATDP", "ATDPN", "AT@1"};
      for (const char *o : ok) if (!strcmp(t, o)) y = true;
      if (!y) return "Adapter settings (AT...) only while the car is stopped.";
    } else {
      static const char *rd[] = {"01", "02", "03", "06", "07", "09", "0A", "19", "22", "3E"};
      for (const char *o : rd) if (!strncmp(t, o, 2)) y = true;
      if (!y) return "Only read requests (01, 06, 09, 19, 22...) while the car is moving.";
    }
  }
  return nullptr;
}

// "7E1:221035 22126A" asks another module (here the gearbox) one or more commands, then goes
// back to the engine so normal polling carries on. Without a prefix: one command, as typed.
static void serviceConsole() {
  if (!consolePending) return;
  char r[256];
  bool v = verbose; verbose = true;
  const char *c0 = consoleCmd;
  uint16_t addr = 0;
  bool hdr = strlen(c0) > 4 && c0[3] == ':' && parseAddr(c0, addr);
  if (!hdr) {
    if (!request(consoleCmd, r, sizeof(r), 5000)) strlcpy(r, "(no reply)", sizeof(r));
  } else {
    char h[4] = {c0[0], c0[1], c0[2], 0}, cmds[48], one[128];
    strlcpy(cmds, c0 + 4, sizeof(cmds));
    setTarget(addr);
    int o = snprintf(r, sizeof(r), "[%s -> %03X]\r", h, replyAddr(addr));
    char *save = nullptr;
    for (char *tok = strtok_r(cmds, " ,", &save); tok && o < (int)sizeof(r) - 1; tok = strtok_r(nullptr, " ,", &save)) {
      if (!request(tok, one, sizeof(one), 2000)) strlcpy(one, "(no reply)", sizeof(one));
      for (char *c = one; *c; c++) if (*c == '\r' || *c == '\n') *c = ' ';
      o += snprintf(r + o, sizeof(r) - o, "%s  %s\r", tok, one);
    }
    restoreTarget();                              // back to normal polling
  }
  verbose = v;
  for (char *c = r; *c; c++) if (*c == '\r') *c = '\n';
  strlcpy(consoleReply, r, sizeof(consoleReply));
  consolePending = false; consoleDone = true;
}

// ---------------------------------------------------------------- identifier finder
// The OBD tab's "Find an identifier": while it runs, normal polling pauses (the gauges go
// stale). Discovery asks one ECU (header 7E0 engine, 7E1 gearbox, ...) for every identifier
// in a range and keeps those it answers as snapshot 1; "read again" re-reads only those as
// the next labelled snapshot. scan_logic.h ranks them.
enum ScanMode : uint8_t { SC_IDLE, SC_DISCOVER, SC_SNAP, SC_MODULES };
// "Find modules": which addresses answer. A tester-present (3E00) to every request address
// 700-795 (answers on + 0x6A) and 7E0-7E7 (answers on + 8), then each one that answered is
// asked its name (22F197, the system name; else 22F187, the part number).
struct Mod { uint16_t a; char name[26]; };
static Mod mods[40];
static volatile int nMods = 0;
static int modPass = 0;
static const int SWEEP_N = 0x96 + 8;
static uint16_t sweepAddr(int i) { return i < 0x96 ? (uint16_t)(0x700 + i) : (uint16_t)(0x7E0 + (i - 0x96)); }
static scanx::Table scanTab;
static volatile ScanMode scanMode = SC_IDLE;
static volatile int  scanCur = 0, scanDone = 0, scanTotal = 0;   // progress (identifier / entry index)
static volatile uint32_t scanStartMs = 0;
static char scanHdr[8] = "7E0";
static uint16_t scanFrom = 0x1000, scanTo = 0x2FFF;
static int scanSnap = -1;
static char scanMsg[64] = "";
static portMUX_TYPE scanMux = portMUX_INITIALIZER_UNLOCKED;     // the web task reads the table
// requests from the web task
static volatile uint8_t scanReq = 0;             // 1 discover, 2 snapshot, 3 stop, 4 clear, 5 find modules
static char scanReqHdr[8] = "7E0", scanReqLabel[16] = "";
static uint16_t scanReqFrom = 0x1000, scanReqTo = 0x2FFF;

static bool scanAlloc() {
  if (scanTab.e) return true;
  scanTab.e = (scanx::Entry *)heap_caps_malloc(sizeof(scanx::Entry) * scanx::MAXE, MALLOC_CAP_SPIRAM);
  return scanTab.e != nullptr;
}
static void scanHeader(const char *h) {
  uint16_t a;
  if (parseAddr(h, a)) setTarget(a);
}
static void scanFinish(const char *why) {
  restoreTarget();                                // back to normal polling
  scanMode = SC_IDLE;
  strlcpy(scanMsg, why, sizeof(scanMsg));
  logf(true, "identifier finder: %s (%d answering, %d snapshot%s)", why, scanTab.n, scanTab.ns, scanTab.ns == 1 ? "" : "s");
}
// one step: one identifier asked. Returns false when nothing is running.
static bool scanService() {
  uint8_t rq = scanReq;
  if (rq) {
    scanReq = 0;
    if (rq == 3) { if (scanMode != SC_IDLE) scanFinish("stopped"); }
    else if (rq == 4) { portENTER_CRITICAL(&scanMux); scanTab.clear(); portEXIT_CRITICAL(&scanMux); scanMsg[0] = 0; scanMode = SC_IDLE; }
    else if (rq == 5) {
      if (scanMode != SC_IDLE) restoreTarget();
      nMods = 0; modPass = 0; scanCur = 0; scanDone = 0; scanTotal = SWEEP_N; scanStartMs = millis();
      scanMode = SC_MODULES; scanMsg[0] = 0;
      logf(true, "finding modules: 700-795 and 7E0-7E7");
    }
    else if (!scanAlloc()) strlcpy(scanMsg, "no memory for the table", sizeof(scanMsg));
    else if (rq == 1) {
      portENTER_CRITICAL(&scanMux); scanTab.clear(); scanSnap = scanTab.begin(scanReqLabel); portEXIT_CRITICAL(&scanMux);
      strlcpy(scanHdr, scanReqHdr, sizeof(scanHdr)); scanFrom = scanReqFrom; scanTo = scanReqTo;
      scanCur = scanFrom; scanDone = 0; scanTotal = scanTo - scanFrom + 1; scanStartMs = millis();
      scanHeader(scanHdr);
      scanMode = SC_DISCOVER; scanMsg[0] = 0;
      logf(true, "identifier finder: asking %s for 22%04X-22%04X", scanHdr, scanFrom, scanTo);
    } else if (rq == 2) {
      if (!scanTab.n) strlcpy(scanMsg, "scan a range first", sizeof(scanMsg));
      else {
        int k; portENTER_CRITICAL(&scanMux); k = scanTab.begin(scanReqLabel); portEXIT_CRITICAL(&scanMux);
        if (k < 0) strlcpy(scanMsg, "6 snapshots is the limit: clear to start over", sizeof(scanMsg));
        else { scanSnap = k; scanCur = 0; scanDone = 0; scanTotal = scanTab.n; scanStartMs = millis(); scanHeader(scanHdr); scanMode = SC_SNAP; scanMsg[0] = 0; }
      }
    }
  }
  if (scanMode == SC_IDLE) return false;
  if (scanMode == SC_MODULES) {
    char c[24], r[200];
    if (modPass == 0) {
      if (scanCur >= SWEEP_N) {
        if (!nMods) { restoreTarget(); scanMode = SC_IDLE; strlcpy(scanMsg, "no module answered", sizeof(scanMsg)); return true; }
        modPass = 1; scanCur = 0; scanDone = 0; scanTotal = nMods;
        return true;
      }
      uint16_t a = sweepAddr(scanCur);
      snprintf(c, sizeof(c), "ATSH%03X", a); request(c, r, sizeof(r), 1000);
      snprintf(c, sizeof(c), "ATCRA%03X", replyAddr(a)); request(c, r, sizeof(r), 1000); craSet = true;
      bool alive = request("3E00", r, sizeof(r), 300) && obdp::classify(r) == obdp::OK && (strstr(r, "7E00") || strstr(r, "7F3E"));
      if (alive && nMods < (int)(sizeof(mods) / sizeof(mods[0]))) {
        portENTER_CRITICAL(&scanMux); mods[nMods].a = a; mods[nMods].name[0] = 0; nMods = nMods + 1; portEXIT_CRITICAL(&scanMux);
        logf(true, "module at %03X (answers on %03X)", a, replyAddr(a));
      }
    } else {
      if (scanCur >= nMods) {
        restoreTarget(); scanMode = SC_IDLE;
        snprintf(scanMsg, sizeof(scanMsg), "%d module%s answered", (int)nMods, nMods == 1 ? "" : "s");
        logf(true, "finding modules: %s", scanMsg);
        return true;
      }
      Mod &m = mods[scanCur];
      setTarget(m.a);
      uint8_t b[24]; int n = 0;
      const uint16_t ids[2] = {0xF197, 0xF187};
      for (int k = 0; k < 2 && n == 0; k++) {
        snprintf(c, sizeof(c), "22%04X", ids[k]);
        if (request(c, r, sizeof(r), 800) && obdp::classify(r) == obdp::OK) n = obdp::parseMode22(r, ids[k], b, sizeof(b));
      }
      char nm[26]; int j = 0;
      for (int i = 0; i < n && j < (int)sizeof(nm) - 1; i++) if (b[i] >= 0x20 && b[i] < 0x7F) nm[j++] = (char)b[i];
      while (j && nm[j - 1] == ' ') j--;
      nm[j] = 0;
      portENTER_CRITICAL(&scanMux); strlcpy(m.name, j ? nm : "(no name)", sizeof(m.name)); portEXIT_CRITICAL(&scanMux);
    }
    scanCur = scanCur + 1; scanDone = scanDone + 1;
    return true;
  }
  uint16_t did;
  if (scanMode == SC_DISCOVER) { if (scanCur > scanTo) { scanFinish("scan finished"); return true; } did = (uint16_t)scanCur; }
  else { if (scanCur >= scanTab.n) { scanFinish("snapshot finished"); return true; } did = scanTab.e[scanCur].did; }
  char c[12], r[200];
  snprintf(c, sizeof(c), "22%04X", did);
  uint8_t b[scanx::MAXB]; int n = 0;
  if (request(c, r, sizeof(r), 400) && obdp::classify(r) == obdp::OK) n = obdp::parseMode22(r, did, b, scanx::MAXB);
  portENTER_CRITICAL(&scanMux);
  if (scanMode == SC_DISCOVER) { if (n > 0) scanTab.add(did, scanSnap, b, n); }
  else if (n > 0) scanx::Table::set(scanTab.e[scanCur], scanSnap, b, n);
  portEXIT_CRITICAL(&scanMux);
  scanCur = scanCur + 1; scanDone = scanDone + 1;
  return true;
}

// For the OBD tab: 0 = polled, 1 = dropped (the car kept refusing it), 2 = not used on this car
static int pollState(const Slow &s) {
  if (s.kind == 0 && !obdp::supported(supportedMask, s.pid)) return 2;
  if ((s.pid == 0x0F && !cacFromIat) || (s.pid == 0x0B && mapFrom70)) return 2;
  if ((s.pid == 0x0D && speedInFast) || (s.pid == 0x11 && !speedInFast)) return 2;
  return s.misses >= 3 ? 1 : 0;
}

static void pollLoop() {
  state = S_RUN;
  int fails = 0, replies = 0;
  uint32_t quietAt = millis() + 5000;              // log every command for the first 5 s only
  uint32_t rateT = millis();
  bool fastPair = obdp::supported(supportedMask, 0x11);
  speedInFast = obdp::supported(supportedMask, 0x0D);
  int speedMiss = 0;
  for (;;) {
    if (!enabled || !client->isConnected() || linkLost) return;
    if (verbose && (int32_t)(millis() - quietAt) > 0) verbose = false;
    serviceConsole();
    if (selfReq) { selfReq = false; runSelfTests(); }
    if (scanService()) { fails = 0; continue; }     // the identifier finder has the adapter
    // fast group: RPM and road speed together every cycle (about 11 a second, and from the same
    // moment, which the gear needs). The throttle plate is in the slow list: on the 3.0T it
    // says little (the bypass valve controls boost). Without speed-with-RPM: the old pairing.
    bool ok;
    if (speedInFast) {
      uint32_t before = valAt[F_KMH];
      ok = runMode01("010C0D");
      if (ok && valAt[F_KMH] != before) { speedMiss = 0; if (onRpmSpeed) onRpmSpeed(val[F_RPM], val[F_KMH]); }
      else if (++speedMiss >= 4) { speedInFast = false; logf(true, "speed doesn't come with RPM (010C0D): polling it on its own"); }
    } else ok = runMode01(fastPair ? "010C11" : "010C");
    replies += ok;
    fails = ok ? 0 : fails + 1;
    // one slow item: the most overdue that the car supports
    uint32_t now = millis();
    Slow *best = nullptr; float worst = 1.0f;          // due at 1 (a whole period since the last)
    bool moving = fresh(F_KMH, 2000) && val[F_KMH] > 8;
    // on boost (or the pedal well down): the boost-related items speed up. Not the throttle
    // plate: on the 3.0T it sits nearly wide open whenever the supercharger is working.
    bool boosting = senderPsi > 1.0f ||
                    (fresh(F_MAP, 1000) && val[F_MAP] > (fresh(F_BARO, ageFor(F_BARO)) ? val[F_BARO] : 100.0f) + 7.0f) ||
                    (fresh(F_PEDAL, ageFor(F_PEDAL)) && val[F_PEDAL] > 60);
    if (ok) gotEma(fastOkAt, fastGotMs, boosting, now);
    for (Slow &s : slowList) {
      if (s.kind == 0 && !obdp::supported(supportedMask, s.pid)) continue;
      if (s.pid == 0x0F && !cacFromIat) continue;
      if (s.pid == 0x0B && mapFrom70) continue;          // 0170 already gives manifold pressure
      if (s.pid == 0x0D && speedInFast) continue;        // speed comes with RPM already
      if (s.pid == 0x11 && !speedInFast) continue;       // throttle comes with RPM then
      if (s.misses >= 3) {                           // the car keeps saying NO DATA: set it aside
        // ...but not for the whole drive: a reading that has answered before can miss a few in a row
        // (log_0036 / 0038: drive select stopped after a minute and never came back), and a coolant
        // or oil reading that silently stayed away would leave its warning blind. Another module's
        // reading gets another go every 30 s, the engine's own every 60 s; not during a pull (a
        // request that times out holds everything up for a second).
        if (boosting || now - s.last < ((s.kind == 3 || s.kind == 5 || s.kind == 8) ? 30000u : 60000u)) continue;
        s.misses = 2;
      }
      if (moving && stoppedOnly(s)) { s.okAt = 0; continue; }
      if (boosting && pullPaused(s) && now - s.last < PULL_PAUSE_MAX) { s.okAt = 0; continue; }
      uint32_t period = (s.boostMs && boosting) ? s.boostMs : s.periodMs;
      if (mapFast && (s.pid == 0x70 || s.pid == 0x0B)) period = 120;   // the boost gauge runs on it
      float late = (float)(now - s.last) / (float)period;   // as a share of its period, so the shares follow the periods
      if (late >= worst) { worst = late; best = &s; }
    }
    if (best) {
      best->last = now;
      int before = replies;
      // "not answered" goes in the log once per run of misses, not at every retry
      auto firstDrop = [&]() { if (best->misses != 2 || best->dropTold) return false; best->dropTold = true; return true; };
      if (best->kind == 7) {                          // the catalyst self-tests
        bool k = readCats();
        if (!k && firstDrop()) logf(true, "catalyst self-test results (0621 / 0622) not answered: trying again every minute");
        best->misses = k ? 0 : best->misses + 1;
        replies += k;
      } else if (best->kind == 6) {                   // the car's clock (steering module)
        setTarget(0x712);
        char r[200]; uint8_t b[12]; int n = 0;
        if (request(best->cmd, r, sizeof(r), 1500) && obdp::classify(r) == obdp::OK) n = obdp::parseMode22(r, 0x2BC1, b, sizeof(b));
        restoreTarget();
        int yy = 0, mo = 0, dd = 0, ss = 0, mi = 0, hh = 0;
        if (n >= 10) { yy = b[4]; mo = b[5]; dd = b[6]; ss = b[7]; mi = b[8]; hh = b[9]; }
        bool k = n >= 10 && yy >= 20 && yy < 80 && mo >= 1 && mo <= 12 && dd >= 1 && dd <= 31 && hh < 24 && mi < 60 && ss < 60;
        if (k) {
          carLocal = (uint32_t)(daysFromCivil(2000 + yy, mo, dd) * 86400 + hh * 3600 + mi * 60 + ss);
          carClockAt = millis(); carClockN = carClockN + 1;
          best->periodMs = 600000;                       // then every 10 minutes
        } else if (firstDrop()) logf(true, "%s (steering, 712) not answered: trying again every minute", best->cmd);
        best->misses = k ? 0 : best->misses + 1;
        replies += k;
      } else if (best->kind == 4) {                   // one identifier, 4-byte answer (odometer)
        char r[64]; uint8_t b[8]; int n = 0;
        if (request22(best->cmd, r, sizeof(r), 1000, 3 + best->len[0]) && obdp::classify(r) == obdp::OK) n = obdp::parseMode22(r, best->did[0], b, 8);
        bool k = n >= 4;
        if (k) {
          uint32_t v = (uint32_t)b[0] << 24 | (uint32_t)b[1] << 16 | (uint32_t)b[2] << 8 | b[3];
          if (v > 0 && v < 2000000) { odoKm = v; odoAt = millis(); } else k = false;
        }
        if (!k && firstDrop()) logf(true, "%s not answered: trying again every minute", best->cmd);
        best->misses = k ? 0 : best->misses + 1;
        replies += k;
      } else if (best->kind == 2 || best->kind == 3 || best->kind == 5 || best->kind == 8) {
        if (best->kind == 3) setTarget(0x7E1);           // the gearbox, for this one
        if (best->kind == 5) setTarget(0x71D, false);    // the sport differential
        if (best->kind == 8) setTarget(0x752, false);    // the parking brake
        bool k = runGroup22(*best);
        // the sport diff and parking brake (through the gateway) often miss the first request after the
        // switch: logs 33-38 have 221306 (asked first on a visit) missing a lot while 221307 right after
        // it is nearly always there. One more go straight away, before switching back.
        if (!k && (best->kind == 5 || best->kind == 8)) k = runGroup22(*best);
        // the gearbox: while the header is on 7E1, its other reading too if it's at least half due
        // (D / S / M every 2 s and the temperatures every 5 s then share one switch there and back)
        if (best->kind == 3)
          for (Slow &o : slowList) {
            if (&o == best || o.kind != 3 || o.misses >= 3 || (boosting && pullPaused(o))) continue;
            uint32_t op = (o.boostMs && boosting) ? o.boostMs : o.periodMs;
            if ((float)(now - o.last) < 0.5f * (float)op) continue;
            o.last = now;
            int ob = replies;
            bool ok2 = runGroup22(o);
            o.misses = ok2 ? 0 : o.misses + 1; replies += ok2;
            if (replies > ob) gotEma(o.okAt, o.gotMs, boosting, millis());
          }
        if (best->kind != 2) restoreTarget();
        if (!k && best->misses == 2 && !best->dropTold) {   // once, not at every 30 s retry (the adapter log holds 40 lines)
          best->dropTold = true;
          logf(true, best->kind == 2 ? "%s not answered: trying again every minute" : "%s not answered: trying again every 30 s", best->cmd);
        }
        if (k) best->dropTold = false;
        best->misses = k ? 0 : best->misses + 1;
        replies += k;
      } else if (best->kind == 1) {
        char r[64];
        if (request(best->cmd, r, sizeof(r), 1000)) { float v = obdp::parseVolts(r); if (v > 0) { val[F_BAT] = v; valAt[F_BAT] = millis(); replies++; } }
      } else {
        bool k = runMode01(best->cmd, 1000);
        if (!k && best->pid != 0x0F && best->pid != 0x0B && firstDrop()) logf(true, "%s not answered: trying again every minute", best->cmd);
        if (k && best->pid == 0x0D && onRpmSpeed && fresh(F_RPM, 300)) onRpmSpeed(val[F_RPM], val[F_KMH]);
        if (best->pid == 0x70) mapFrom70 = k ? true : (best->misses >= 2 ? false : mapFrom70);
        best->misses = k ? 0 : best->misses + 1;
        replies += k;
      }
      if (replies > before) { best->dropTold = false; gotEma(best->okAt, best->gotMs, boosting, millis()); }
    }
    if (millis() - rateT >= 2000) { rate = replies * 1000.0f / (millis() - rateT); replies = 0; rateT = millis(); }
    if (useSuffix && fails == 3) { useSuffix = false; logf(true, "replies failing with the \"1\" suffix - turning it off"); }
    if (fails >= 8) { state = S_NOCAR; strlcpy(lastMsg, "car stopped answering", sizeof(lastMsg)); logf(true, "%s", lastMsg); verbose = true; return; }
  }
}

// ---------------------------------------------------------------- task
static void task(void *) {
  NimBLEDevice::init("S5-GaugePod");
  NimBLEDevice::setSecurityAuth(true, false, true);   // bonding, no PIN, secure connections (as the S2000)
  rxSem = xSemaphoreCreateBinary();
  for (;;) {
    if (!enabled) { state = S_OFF; if (client && client->isConnected()) client->disconnect(); vTaskDelay(pdMS_TO_TICKS(500)); continue; }
    if (linkLost && client && !client->isConnected()) { linkLost = false; logf(true, "adapter disconnected"); vTaskDelay(pdMS_TO_TICKS(1000)); }
    if (!client || !client->isConnected()) {
      if (!connectAdapter()) { vTaskDelay(pdMS_TO_TICKS(2000)); continue; }
      triedAuto = false;
      if (!initElm()) { client->disconnect(); vTaskDelay(pdMS_TO_TICKS(2000)); continue; }
    }
    // adapter up: wait for the car (ignition on), then poll until it stops answering
    if (!probeCar()) {
      // Remembered adapter: stay connected and wait (ignition off in our own car). Anything
      // else gets 2 minutes for a car to answer, then it's let go and ignored until restart,
      // so the pod doesn't sit on another car's adapter (they take one connection only).
      static uint32_t waitSince = 0; static char waitAddr[24] = "";
      if (strcmp(waitAddr, devAddr)) { strlcpy(waitAddr, devAddr, sizeof(waitAddr)); waitSince = millis(); }
      bool mine = savedAddr[0] && !strcasecmp(savedAddr, devAddr);
      if (!mine && millis() - waitSince > 120000) {
        if (nIgnored < 4) strlcpy(ignored[nIgnored++], devAddr, sizeof(ignored[0]));
        logf(true, "no car answered through %s for 2 min: letting it go (ignored until restart)", devAddr);
        client->disconnect(); waitAddr[0] = 0;
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }
      if (state != S_NOCAR) logf(true, "car not answering (ignition off?) - retrying");
      state = S_NOCAR; strlcpy(lastMsg, "adapter connected, car not answering (ignition off?)", sizeof(lastMsg));
      for (int i = 0; i < 20 && client->isConnected(); i++) { serviceConsole(); vTaskDelay(pdMS_TO_TICKS(100)); }
      continue;
    }
    lastMsg[0] = 0;
    // a car answered through this adapter: it's ours, remember it
    if (strcasecmp(savedAddr, devAddr) != 0) { strlcpy(savedAddr, devAddr, sizeof(savedAddr)); addrLearned = true; }
    for (Slow &s : slowList) { s.misses = 0; s.last = 0; }
    pollLoop();
  }
}

static void begin() { xTaskCreatePinnedToCore(task, "obd", 8192, nullptr, 1, nullptr, 0); }

}  // namespace obd
