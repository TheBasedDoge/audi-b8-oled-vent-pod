// Desktop test for firmware/S5GaugePod_Test/logger.h: simulated drives at 10 Hz against fake
// storage (tools/fakes): internal flash and an SD card that can be missing, present, or fail.
// Checks that every line lands in the file in order; that internal-flash writes are whole
// lined-up 4 KB sectors and never happen during a pull (unless the buffer is nearly full);
// that the file is saved regularly and right after the engine stops; that "Stop this log"
// waits for the next engine start; that the log being written shows in the listing; and that
// the SD card is used when present, with a fall-back to the flash if it fails mid-drive.
// build: g++ -std=c++17 -O1 -Itools/fakes tools/log_test.cpp -o log_test && ./log_test
#include "FFat.h"
uint32_t fakeNow = 0;
bool fakeBusyNow = false;
FakeSerial Serial;
FakeFFat FFat;
FakeSD SD_MMC;
#include "../firmware/S5GaugePod_Test/logger.h"

static int fails = 0;
static void expect(bool ok, const char *what) { printf("  %-66s %s\n", what, ok ? "ok" : "FAIL"); if (!ok) fails++; }

static std::string sent;                       // every line handed to the logger for the current file
static uint32_t seq = 0;
static void tick(bool engineOn, bool boosting) {   // 100 ms: one sample, one task pass
  char line[160];
  snprintf(line, sizeof(line), "%u,%.2f,0.00,1.234,3000,45,95,190,210,13.9,12.5,180.0,101,88,0,0.12,-0.30,185.0\n", seq++, boosting ? 12.0 : -9.0);
  bool wasActive = logx::active;
  fakeBusyNow = boosting;
  logx::sample(engineOn, boosting, line);
  if (logx::active && !wasActive) sent.clear();
  if (logx::active) sent += line;
  logx::taskStep();
  fakeNow += 100;
}
static const FakeDisk::F &newest(FakeDisk &d) { static FakeDisk::F none; auto it = d.files.rbegin(); return it == d.files.rend() ? none : it->second; }
static std::string body(const FakeDisk::F &f) { size_t h = f.data.find('\n'); return h == std::string::npos ? "" : f.data.substr(h + 1); }
static int listed(bool recordingOnly) {
  int n = 0;
  for (logx::Store *s : {&logx::card, &logx::flash})
    logx::forEachLog(*s, [&](const String &name, uint32_t, uint32_t) { if (!recordingOnly || logx::isRecording(*s, name)) n++; });
  return n;
}

int main() {
  SD_MMC.d.present = false;                      // start with no card
  logx::flash.ok = true; logx::fsOk = true;
  logx::mountCard();
  logx::sb = xStreamBufferCreateStatic(logx::BUF, 1, nullptr, nullptr);
  FakeDisk &disk = FFat.d;

  printf("0. 23 logs from an older build (one deleted), and reading the folder finds nothing (as on the board)\n");
  for (int k = 1; k <= 23; k++) { char n[24]; snprintf(n, sizeof(n), "/log_%04d.csv", k); disk.files[n].data = "ms,psi\n1,2\n"; disk.files[n].durable = 11; }
  disk.files.erase("/log_0010.csv");             // one deleted: a gap
  disk.listingBroken = true;
  logx::findNext();
  expect(logx::nextN == 24, "next log is 0024, not 0001");
  expect(listed(false) == 22, "all 22 listed (found by name, past the gap)");
  const size_t base = disk.files.size();

  printf("1. no SD card: a 10-minute drive with a 5 s pull every 40 s, then the engine stops\n");
  for (int i = 0; i < 6000; i++) tick(true, (i % 400) < 50);
  size_t writesBusy = 0, misaligned = 0, saves = 0; uint32_t lastSave = 0, worstGap = 0; bool savedBusy = false;
  for (size_t k = 0; k < disk.ops.size(); k++) {
    auto &o = disk.ops[k];
    if (o.kind == 'w') { if (o.busy && k > 0) writesBusy++; /* k 0 = the header, into the file buffer */ if (k > 0 && o.endPos % 4096) misaligned++; }
    else { saves++; if (o.busy) savedBusy = true; if (o.t - lastSave > worstGap) worstGap = o.t - lastSave; lastSave = o.t; }
  }
  expect(disk.files.size() == base + 1 && logx::open && logx::cur == &logx::flash, "one file open, in the internal flash");
  expect(listed(true) == 1, "the log being written shows in the listing");
  expect(writesBusy == 0, "no flash writes during the pulls");
  expect(misaligned == 0, "every write is a whole sector, lined up");
  expect(!savedBusy, "no saves during the pulls");
  expect(worstGap <= 66000, "saved once a minute (or right after a pull that was in the way)");
  printf("     %zu writes (%.1f s apart), %zu saves, worst gap between saves %.1f s, %zu KB\n",
         disk.ops.size() - saves, 600.0 / (disk.ops.size() - saves), saves, worstGap / 1000.0, newest(disk).data.size() / 1024);
  tick(false, false);
  expect(newest(disk).durable == newest(disk).data.size() && body(newest(disk)) == sent, "engine off: everything saved within 0.1 s (power cut safe)");
  for (int i = 0; i < 160; i++) tick(false, false);
  expect(!logx::open && !logx::active, "closed after 15 s of engine off");
  expect(body(newest(disk)) == sent && listed(false) == 23, "file holds every line, in order, and is listed");
  expect(disk.files["/log_0001.csv"].data == "ms,psi\n1,2\n", "the old log_0001 is untouched");

  printf("2. \"Stop this log\" halfway through the next drive\n");
  for (int i = 0; i < 600; i++) tick(true, false);
  logx::stop(true);
  std::string had = sent;
  tick(true, false); tick(true, false);
  expect(disk.files.size() == base + 2 && !logx::open, "second file closed");
  expect(body(newest(disk)) == had && newest(disk).durable == newest(disk).data.size(), "and saved with every line");
  for (int i = 0; i < 300; i++) tick(true, false);
  expect(disk.files.size() == base + 2 && !logx::active, "no new log while the engine keeps running");
  for (int i = 0; i < 160; i++) tick(false, false);
  for (int i = 0; i < 20; i++) tick(true, false);
  expect(disk.files.size() == base + 3 && logx::open, "a new log at the next engine start");

  printf("3. a 90 s pull with no break (the buffer fills)\n");
  uint32_t d0 = logx::dropped; size_t ops0 = disk.ops.size();
  for (int i = 0; i < 900; i++) tick(true, true);
  size_t forced = 0; for (size_t k = ops0; k < disk.ops.size(); k++) if (disk.ops[k].kind == 'w') forced++;
  expect(logx::dropped == d0, "no lines dropped");
  expect(forced > 0, "written anyway once the buffer is nearly full");
  for (int i = 0; i < 170; i++) tick(false, false);
  expect(body(newest(disk)) == sent, "every line saved");

  printf("4. SD card put in: the next drive goes to the card\n");
  SD_MMC.d.present = true;
  fakeNow += 40000;                               // the card is looked for again at most every 30 s
  for (int i = 0; i < 1200; i++) tick(true, (i % 400) < 50);
  FakeDisk &sd = SD_MMC.d;
  size_t sdBusy = 0, sdSaves = 0; uint32_t sdLast = 0, sdGap = 0;
  for (auto &o : sd.ops) { if (o.kind == 'w' && o.busy) sdBusy++; if (o.kind == 's') { sdSaves++; if (o.t - sdLast > sdGap && sdLast) sdGap = o.t - sdLast; sdLast = o.t; } }
  expect(logx::cur == &logx::card && sd.files.size() == 1, "recording on the SD card");
  expect(sd.files.begin()->first == "/log_0027.csv", "numbered after the drives in the flash (log_0027)");
  expect(sdBusy > 0, "the card is written during pulls too (nothing freezes)");
  expect(sdGap <= 10100, "saved every 10 s");
  expect(listed(false) == 26 && listed(true) == 1, "listing shows both places, one recording");

  printf("5. the card fails mid-drive: the log carries on in the flash\n");
  sd.failWrites = true;
  for (int i = 0; i < 300; i++) tick(true, false);
  expect(logx::open && logx::cur == &logx::flash && !logx::card.ok, "now recording in the internal flash");
  for (int i = 0; i < 170; i++) tick(false, false);
  std::string onCard = body(sd.files.begin()->second), onFlash = body(newest(disk));
  expect(newest(disk).data.rfind("ms,psi", 0) == 0, "the new file has its header");
  bool prefix = sent.compare(0, onCard.size(), onCard) == 0;
  bool suffix = onFlash.size() <= sent.size() && sent.compare(sent.size() - onFlash.size(), onFlash.size(), onFlash) == 0;
  expect(prefix && suffix && onFlash.size() > 0, "card file = start of the drive, flash file = the rest");
  expect(onCard.size() + onFlash.size() + 4096 >= sent.size(), "at most the one failed 4 KB write lost");
  printf("     card file %zu KB, flash file %zu KB\n", onCard.size() / 1024, onFlash.size() / 1024);

  printf("6. logging switched off mid-drive\n");
  for (int i = 0; i < 300; i++) tick(true, false);
  std::string had6 = sent;
  logx::off(); tick(true, false);
  expect(!logx::open && body(newest(disk)) == had6, "closed and saved");

  printf("7. standing still: saved every few seconds (the pod loses power with the ignition)\n");
  SD_MMC.d.present = false; logx::card.ok = false;
  for (int i = 0; i < 100; i++) tick(true, false);            // a new log, in the internal flash, driving
  logx::parked = true;
  size_t ops7 = disk.ops.size(); uint32_t last7 = fakeNow, gap7 = 0; size_t saves7 = 0;
  for (int i = 0; i < 300; i++) tick(true, false);            // 30 s at a standstill
  for (size_t k = ops7; k < disk.ops.size(); k++) if (disk.ops[k].kind == 's') { saves7++; if (disk.ops[k].t - last7 > gap7) gap7 = disk.ops[k].t - last7; last7 = disk.ops[k].t; }
  expect(logx::open && logx::cur == &logx::flash && saves7 >= 4 && gap7 <= 9000, "saved at least every other sector (about 4-8 s)");
  expect(newest(disk).data.size() - newest(disk).durable <= 4096, "a power cut now loses under a sector already written");
  printf("     %zu saves in 30 s, worst gap %.1f s\n", saves7, gap7 / 1000.0);
  logx::parked = false;

  printf("8. a long drive with the internal flash nearly full\n");
  disk.files["/log_0002.csv"].data.assign(3000000, 'x'); disk.files["/log_0002.csv"].durable = 3000000;
  disk.total = FFat.usedBytes() + 700000;                     // 700 KB free: about 10 minutes at this rate
  std::string cur8 = logx::curName;
  for (int i = 0; i < 12000; i++) tick(true, (i % 400) < 50);  // 20 minutes
  expect(!disk.files.count("/log_0002.csv"), "the oldest drives are deleted during the drive, not only at its start");
  expect(logx::open && cur8 == logx::curName && logx::lastErr[0] == 0, "the log carries on, no error");
  expect(FFat.totalBytes() - FFat.usedBytes() >= 600000, "room kept free");

  printf(fails ? "%d FAILED\n" : "all logger tests passed\n", fails);
  return fails != 0;
}
