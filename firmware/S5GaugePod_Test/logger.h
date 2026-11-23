// =====================================================================
//  S5 Gauge Pod - CSV drive logs, on the microSD card or the internal flash
//
//  Where: the microSD card if one is in the slot (FAT32; 1-bit SDMMC on GPIO 41 CLK,
//  39 CMD, 40 D0, with 38 = D3 held high, as Waveshare's own demo), otherwise the FFat partition in the ESP32's
//  flash (9.9 MB with the "16M Flash (3MB APP/9.9MB FATFS)" partition scheme). The card is
//  looked for at boot and again at the start of each drive. If it stops answering mid-drive
//  the log carries on in the internal flash.
//
//  The sketch calls logx::sample() at S.logHz from the UI loop. Lines go into a 64 KB
//  PSRAM buffer; a low-priority task on core 0 does everything that touches storage
//  (creating files, writing, saving, deleting old drives), so the UI loop never waits on it.
//
//  Internal flash needs care over WHEN it's written: while the flash is erased or written the
//  ESP32 stops BOTH cores from reading PSRAM and flash, where the frame buffers and most of
//  the code live, so everything freezes for that moment. So on the internal flash:
//    - data goes out only in whole 4 KB sectors, lined up with the sectors (one erase each,
//      about every 4 s at 10 Hz: a short hitch while cruising)
//    - nothing is written while boosting or on the throttle (the buffer holds about a
//      minute), unless the buffer is nearly full
//    - the file is saved (size and table) every 60 s and as soon as the engine stops; while the
//      car stands still, every 4 s (the pod loses power with the ignition, and a short hitch at a
//      standstill doesn't show, so a drive loses its last few seconds instead of up to a minute)
//  The SD card is a separate chip on its own bus: writing it freezes nothing, so it's
//  written whenever a sector's worth is ready and saved every 10 s (every 2 s standing still).
//  Space is checked during the drive too (every 30 s): under 600 KB free, the oldest drives go.
//  The internal flash is never formatted on its own once it has worked: a failed mount leaves it
//  alone (the health history lives there) and the Logs tab offers to format it.
//
//  Either way the file is saved as soon as the engine stops (the pod may lose power with
//  the ignition). One file per drive: log_0001.csv, log_0002.csv ... numbered across both
//  places. A drive ends after 15 s of engine off, or with "Stop this log" (the next one then
//  starts at the next engine start). When free space drops under 1.5 MB the oldest drives
//  in that place are deleted.
//
//  Directory listings read names only and never open the files: opening the file that's
//  being written fails, and that used to end the listing early, so no log ever showed up.
// =====================================================================
#pragma once
#include <FFat.h>
#include <SD_MMC.h>
#include <freertos/stream_buffer.h>
#ifndef LOGX_FAKE_FS
#include <sys/stat.h>
#endif

namespace logx {

static const size_t   BUF = 65536, SECTOR = 4096, FORCE_AT = 49152;
static const int      SD_CLK = 41, SD_CMD = 39, SD_D0 = 40, SD_D3 = 38;   // D3 is the card's CS line

// ---------------------------------------------------------------- the two places
struct Store {
  const char *id, *mount, *label;
  fs::FS     *fs;
  bool        sd, ok;
};
static Store flash = {"flash", "/ffat",   "internal flash", &FFat,   false, false};
static Store card  = {"sd",    "/sdcard", "SD card",        &SD_MMC, true,  false};
static Store *cur  = nullptr;                    // where the open file is
static Store *storeById(const String &id) { return id == "sd" ? &card : &flash; }

static uint64_t totalOf(const Store &s) { return !s.ok ? 0 : s.sd ? SD_MMC.totalBytes() : (uint64_t)FFat.totalBytes(); }
static uint64_t usedOf(const Store &s)  { return !s.ok ? 0 : s.sd ? SD_MMC.usedBytes()  : (uint64_t)FFat.usedBytes(); }
static uint64_t freeOf(const Store &s)  { return totalOf(s) - usedOf(s); }

// Free space, refreshed by the log task (asking the card can take a while: the web page and
// the UI loop read these instead)
static volatile uint32_t freeKB[2] = {0, 0};     // [0] flash, [1] card
static void refreshFree() { freeKB[0] = (uint32_t)(freeOf(flash) >> 10); freeKB[1] = (uint32_t)(freeOf(card) >> 10); }

static bool mountCard() {
  SD_MMC.end();
#ifndef LOGX_FAKE_FS
  pinMode(SD_D3, OUTPUT); digitalWrite(SD_D3, HIGH);   // held high: the card starts in SD mode, not SPI
#endif
  card.ok = SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0) && SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_DEFAULT, 5);
  refreshFree();
  if (card.ok) Serial.printf("[log] SD card: %lu MB, %lu MB free\n", (unsigned long)(totalOf(card) >> 20), (unsigned long)(freeKB[1] >> 10));
  return card.ok;
}

// ---------------------------------------------------------------- state
static bool     fsOk = false;                    // somewhere to log to
static volatile bool open = false;               // a file is open (log task)
static char     curName[24] = "";                // "/log_0001.csv"
static volatile uint32_t curBytes = 0, dropped = 0, writes = 0, worstWriteMs = 0;
static char     lastErr[64] = "";
static StreamBufferHandle_t sb = nullptr;
static File     f;

// UI loop -> log task
static volatile bool openReq = false, closeReq = false, syncReq = false, busy = false;
// UI loop only
static volatile bool active = false;             // this drive is being logged
static volatile bool held = false;               // stopped by hand: wait for the engine to stop first
static volatile bool parked = false;             // the car is standing still (set by the sketch): save often
static volatile bool flashBad = false;           // the internal flash was there before but didn't mount (not formatted)
static bool     wasOn = false;
static uint32_t lastActive = 0;

static void err(const char *m) { strlcpy(lastErr, m, sizeof(lastErr)); Serial.printf("[log] %s\n", m); }

// ---------------------------------------------------------------- listing (names only)
static uint32_t logNum(String n) {               // "log_0007.csv" -> 7, anything else 0
  n.toLowerCase();
  return (n.startsWith("log_") && n.endsWith(".csv")) ? (uint32_t)n.substring(4, n.length() - 4).toInt() : 0;
}
static uint32_t fileSize(const Store &s, const String &name) {
  if (open && cur == &s && name == String(curName + 1)) return curBytes;   // the one being written
#ifdef LOGX_FAKE_FS
  return fakeFileSize(s.fs, "/" + name);
#else
  struct stat st;
  String full = String(s.mount) + "/" + name;
  return stat(full.c_str(), &st) == 0 ? (uint32_t)st.st_size : 0;
#endif
}
// When the file was last written (epoch s, UTC; 0 = not known). The FAT file system stamps each write
// with the system clock, which the pod sets from its RTC, the phone or the car's clock, so a saved log
// carries the time its drive ended. Before the clock is set the stamp is 1980: reported as 0.
static uint32_t fileTime(const Store &s, const String &name) {
#ifdef LOGX_FAKE_FS
  (void)s; (void)name; return 0;
#else
  struct stat st;
  String full = String(s.mount) + "/" + name;
  if (stat(full.c_str(), &st) != 0) return 0;
  return st.st_mtime > 1577836800 ? (uint32_t)st.st_mtime : 0;   // after 2020: a real date
#endif
}
static bool fileExists(const Store &s, const String &name) {
#ifdef LOGX_FAKE_FS
  return fakeFileExists(s.fs, "/" + name);
#else
  struct stat st;
  String full = String(s.mount) + "/" + name;
  return stat(full.c_str(), &st) == 0;
#endif
}
static String logName(uint32_t k) { char b[20]; snprintf(b, sizeof(b), "log_%04lu.csv", (unsigned long)k); return String(b); }
static uint32_t nextN = 1;
static const uint32_t PROBE = 400;               // numbers checked by name below nextN (see forEachLog)

// cb(name, number, size) for every log file in a place
// Two ways, merged: reading the folder, and checking the recent numbers by name. On this board
// the older builds' folder listings came back empty while the files were there (opening each
// entry failed), so the names are checked directly too.
template <class F> static void forEachLog(Store &s, F cb) {
  if (!s.ok) return;
  static uint8_t seen[(PROBE + 7) / 8];
  memset(seen, 0, sizeof(seen));
  uint32_t lo = nextN > PROBE ? nextN - PROBE : 1;
  File root = s.fs->open("/");
  if (root) {
    for (int guard = 0; guard < 5000; guard++) {
      bool dir = false;
      String p = root.getNextFileName(&dir);
      if (!p.length()) break;
      if (dir) continue;
      String n = p.substring(p.lastIndexOf('/') + 1);
      uint32_t k = logNum(n);
      if (!k) continue;
      if (k >= lo && k < lo + PROBE) seen[(k - lo) >> 3] |= 1 << ((k - lo) & 7);
      cb(n, k, fileSize(s, n));
    }
  }
  for (uint32_t k = lo; k < nextN && k < lo + PROBE; k++) {
    if (seen[(k - lo) >> 3] & (1 << ((k - lo) & 7))) continue;
    String n = logName(k);
    if (fileExists(s, n)) cb(n, k, fileSize(s, n));
  }
}
static bool isRecording(const Store &s, const String &name) { return (open || openReq) && cur == &s && name == String(curName + 1); }

static void cleanup(Store &s) {
  for (int guard = 0; guard < 200 && freeOf(s) < 1500000; guard++) {
    String oldest; uint32_t oldestN = 0xFFFFFFFF;
    forEachLog(s, [&](const String &n, uint32_t k, uint32_t) { if (k < oldestN && !isRecording(s, n)) { oldestN = k; oldest = n; } });
    if (!oldest.length()) break;
    s.fs->remove("/" + oldest);
    Serial.printf("[log] %s low on space: deleted %s\n", s.label, oldest.c_str());
  }
}

// ---------------------------------------------------------------- writing (log task)
static const char *HEADER =
  "ms,psi,peak_psi,vsig_V,rpm,throttle_pct,intake_F,coolant_F,oil_F,battery_V,timing_deg,map_kPa,baro_kPa,speed_kmh,page,lat_g,lon_g,map_target_kPa,knock_deg,knock_cyl,gear,pedal_pct,bypass_deg,"
  "hpfp_psi,hpfp_target_psi,lpfp_psi,afr_bank1,afr_bank2,afr_target,cat1_F,cat2_F,ambient_F,precharger_F,ic_pump_pct,injector_duty_pct,boost_source,trans,drive_mode,gbx_1259_F,gbx_125A_F,gbx_125B_F,diff_1306_F,diff_1307_F,o2_b1s2_V,o2_b2s2_V,brake,obd_rate,gbx_1035,gbx_1026\n";

static volatile bool reopen = false;             // the card failed mid-drive: carry on in the flash

static void writeOut(const uint8_t *d, size_t n) {
  if (!open || !n) return;
  uint32_t t = millis();
  size_t w = f.write(d, n);
  t = millis() - t;
  if (t > worstWriteMs) worstWriteMs = t;
  writes = writes + 1;
  curBytes = curBytes + w;
  if (w != n) {
    f.close(); open = false;
    if (cur && cur->sd) { err("SD card write failed - carrying on in the internal flash"); card.ok = false; reopen = true; }
    else err("write failed (storage full?) - log closed");
  }
}

static uint32_t lastSync = 0;
static bool     unsynced = false;
static uint32_t lastCardTry = 0;

static void startFile() {
  // the SD card if there is one (looked for again at each new drive), else the internal flash
  if (!card.ok && !reopen && (lastCardTry == 0 || millis() - lastCardTry > 30000)) { lastCardTry = millis(); mountCard(); }
  reopen = false;
  cur = card.ok ? &card : (flash.ok ? &flash : nullptr);
  if (!cur) { err("no SD card and no internal log storage"); return; }
  cleanup(*cur);
  // never write over an older log, whatever the folder listing said
  for (int guard = 0; guard < 1000 && (fileExists(card, logName(nextN)) || fileExists(flash, logName(nextN))); guard++) nextN++;
  snprintf(curName, sizeof(curName), "/log_%04lu.csv", (unsigned long)nextN++);
  f = cur->fs->open(curName, FILE_WRITE);
  if (!f) {
    if (cur->sd) { card.ok = false; err("couldn't create the log on the SD card - using the internal flash"); startFile(); return; }
    err("couldn't create the log file"); curName[0] = 0; return;
  }
  open = true; curBytes = 0; dropped = 0; lastErr[0] = 0;
  writeOut((const uint8_t *)HEADER, strlen(HEADER));   // stays in the file buffer until the first sector
  refreshFree();
  Serial.printf("[log] recording %s on the %s\n", curName + 1, cur->label);
  lastSync = millis(); unsynced = true;
}

// One pass of the log task (every 100 ms). Split out so tools/log_test.cpp can drive it.
static void taskStep() {
  static uint8_t chunk[SECTOR];
  // Space during the drive: the clean-up at the start of a log only makes 1.5 MB of room (about 20
  // minutes), so a longer drive used to fill the internal flash and stop logging. Not while boosting
  // on the internal flash (deleting writes to it).
  static uint32_t lastSpace = 0;
  if (open && cur && (cur->sd || !busy) && millis() - lastSpace > 30000) {
    lastSpace = millis();
    if (freeOf(*cur) < 600000) { cleanup(*cur); refreshFree(); }
  }
  for (int pass = 0; pass < 32; pass++) {          // "continue" = go round again straight away
    bool closing = closeReq, syncing = syncReq || closing;
    if ((openReq || reopen) && !closing) { openReq = false; startFile(); }
    bool sd = cur && cur->sd;
    size_t avail = xStreamBufferBytesAvailable(sb);
    // a whole sector, lined up (the first one is short by the header). The internal flash
    // waits while boosting; the SD card doesn't need to.
    size_t room = SECTOR - (curBytes % SECTOR);
    if (open && avail >= room && (sd || !busy || avail >= FORCE_AT || syncing)) {
      writeOut(chunk, xStreamBufferReceive(sb, chunk, room, 0));
      unsynced = true;
      continue;
    }
    if (syncing) {                                  // engine stopped or log closing: the rest out now
      size_t n;
      while ((n = xStreamBufferReceive(sb, chunk, sizeof(chunk), 0)) > 0) writeOut(chunk, n);
      if (open) { f.flush(); unsynced = false; lastSync = millis(); }
      syncReq = false;
      if (closing) {
        if (open) { f.close(); open = false; Serial.printf("[log] saved %s, %lu KB\n", curName + 1, (unsigned long)(curBytes / 1024)); }
        refreshFree();
        curName[0] = 0;
        openReq = false; reopen = false;            // stopped before it even opened
        closeReq = false;
      }
      continue;
    }
    if (open && unsynced && (sd || !busy) && millis() - lastSync > (sd ? (parked ? 2000u : 10000u) : (parked ? 4000u : 60000u))) {
      f.flush(); unsynced = false; lastSync = millis();
    }
    return;
  }
}
static void task(void *) {
  lastSync = millis();
  for (;;) { vTaskDelay(pdMS_TO_TICKS(100)); taskStep(); }
}

// Highest log number in both places, so names never repeat
static void findNext() {
  uint32_t mx = 0;
  nextN = 1;
  for (Store *s : {&card, &flash}) forEachLog(*s, [&](const String &, uint32_t k, uint32_t) { if (k > mx) mx = k; });
  nextN = mx + 1;
  // and past any numbered files the listing missed (allowing for gaps of up to 50 deleted ones)
  for (uint32_t k = nextN, miss = 0; miss < 50 && k < nextN + 5000; k++) {
    if (fileExists(card, logName(k)) || fileExists(flash, logName(k))) { nextN = k + 1; miss = 0; }
    else miss++;
  }
  int count[2] = {0, 0};
  for (Store *s : {&card, &flash}) forEachLog(*s, [&](const String &, uint32_t, uint32_t) { count[s->sd]++; });
  Serial.printf("[log] found %d log(s) in the internal flash, %d on the SD card; next is %04lu\n", count[0], count[1], (unsigned long)nextN);
}

// Format the internal flash after a failed mount (Logs tab). Only then, and never by itself:
// it holds the health history as well as logs. The pod restarts afterwards.
static bool formatFlash() {
#ifndef LOGX_FAKE_FS
  if (flash.ok || open) return false;
  FFat.end();
  if (!FFat.format()) return false;
  flashBad = false;
  return true;
#else
  return false;
#endif
}

// allowFormat: only until the internal flash has mounted once (the sketch keeps that in Preferences).
// After that a mount that fails (a power cut at a bad moment) is left as it is, not wiped.
static bool begin(bool allowFormat = true) {
  flash.ok = FFat.begin(false);
  if (!flash.ok) flash.ok = FFat.begin(false);    // once more
  if (!flash.ok && allowFormat) flash.ok = FFat.begin(true);   // first use: format it
  else if (!flash.ok) { flashBad = true; err("internal storage didn't mount (left as it is)"); }
  lastCardTry = millis();
  mountCard();
  fsOk = flash.ok || card.ok;
  if (!fsOk) return false;
  static uint8_t *store = (uint8_t *)ps_malloc(BUF + 1);
  static StaticStreamBuffer_t ctl;
  if (!store) { fsOk = false; return false; }
  sb = xStreamBufferCreateStatic(BUF, 1, store, &ctl);
  findNext();
  refreshFree();
  Serial.printf("[log] logging to the %s, next log %04lu%s\n", card.ok ? card.label : flash.label, (unsigned long)nextN,
                card.ok ? "" : " (no SD card found)");
  xTaskCreatePinnedToCore(task, "log", 6144, nullptr, 1, nullptr, 0);
  return true;
}

// ---------------------------------------------------------------- UI loop side
// Close the current log; byHand = "Stop this log": don't start another until the engine has
// stopped and started again.
static void stop(bool byHand = false) {
  if (active) { active = false; closeReq = true; }
  if (byHand) held = true;
}
static void off() { stop(); held = false; }       // logging switched off in the settings

// At the log rate. engineOn decides when a drive starts / ends; boosting = in boost or on the
// throttle right now (internal-flash writes wait for it to pass).
static void sample(bool engineOn, bool boosting, const char *line) {
  if (!fsOk) return;
  uint32_t now = millis();
  busy = boosting;
  if (engineOn) lastActive = now;
  if (wasOn && !engineOn && active) syncReq = true;   // engine just stopped: save what we have
  wasOn = engineOn;
  if (!engineOn && now - lastActive > 15000) { held = false; stop(); return; }
  if (!active) {
    if (!engineOn || held || closeReq || openReq) return;
    active = true; openReq = true;
  }
  size_t L = strlen(line);
  if (xStreamBufferSend(sb, line, L, 0) != L) dropped = dropped + 1;
}

// Where new logs go right now (for the web page)
static const Store &target() { return (open && cur) ? *cur : (card.ok ? card : flash); }

}  // namespace logx
