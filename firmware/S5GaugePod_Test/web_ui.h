// =====================================================================
//  S5 Gauge Pod - Wi-Fi access point + web UI
//
//  The pod makes its own network (S.apSsid / S.apPass). Join it from the phone
//  and open http://192.168.4.1 (or http://s5pod.local).
//  The page itself is tools/web/index.html, gzipped into web_page.h.
//  The server runs in its own task on core 0. Anything that has to touch the
//  display or the sensor is handed to the UI loop through the web:: request flags.
//
//  Included from the .ino after its globals, which it reads directly.
// =====================================================================
#pragma once
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <esp_ota_ops.h>
#include "web_page.h"

namespace web {

static WebServer server(80);
static volatile int  reqPage = 0;          // +1 / -1
static volatile bool reqPeak = false, reqRezero = false, reqFaces = false, reqRestart = false, reqLogStop = false;
static volatile bool reqRunsClear = false;
static volatile uint32_t reqClock = 0;               // unix time from the phone, for the UI loop to set
// firmware update over Wi-Fi (the loop shows the progress and closes the log)
static volatile bool updating = false;
static volatile int  updPct = -1;
static bool up = false;

static String jesc(const char *t);
static String jnum(float v, int dec) { char b[24]; dtostrf(v, 0, dec, b); return String(b); }
static String jobd(int f, int dec) { return obd::fresh(f, obd::ageFor(f)) ? jnum(obd::val[f], dec) : String("null"); }

static void sendJson(const String &s) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", s);
}

static void hLive() {
  bool demo = isDemo();
  String s;
  s.reserve(900);
  s += "{\"psi\":"; s += jnum(gPsiFilt, 2);
  s += ",\"text\":"; s += jnum(gPsiText, 2);
  s += ",\"peak\":"; s += jnum(gPeakPsi, 2);
  s += ",\"vsig\":"; s += jnum(gVsensPin * SIG_DIV_RATIO, 3);
  s += ",\"zero\":"; s += jnum(gZero, 3);
  s += ",\"fault\":"; s += gSensorFault ? "true" : "false";
  s += ",\"src\":\""; s += demo ? "demo" : gBoostObd ? "obd" : "sender"; s += "\"";
  { char tm = obd::transMode(); s += ",\"trans\":\""; if (tm) s += tm; s += "\""; }
  s += ",\"dmode\":"; s += obd::driveMode();
  s += ",\"demo\":"; s += demo ? "true" : "false";
  s += ",\"v\":{";
  s += "\"rpm\":" + jobd(obd::F_RPM, 0);
  s += ",\"thr\":" + jobd(obd::F_THR, 0);
  s += ",\"iat\":" + (obd::cacFromIat ? jobd(obd::F_IAT, 0) : jobd(obd::F_CAC, 0));
  s += ",\"clt\":" + jobd(obd::F_CLT, 0);
  s += ",\"oil\":" + (obd::fresh(obd::F_OIL, obd::ageFor(obd::F_OIL)) ? jnum(obd::val[obd::F_OIL] + S.oilTrimF, 0) : String("null"));
  s += ",\"pedal\":" + jobd(obd::F_PEDAL, 0);
  s += ",\"afr1\":" + jobd(obd::F_AFR1, 2) + ",\"afr2\":" + jobd(obd::F_AFR2, 2) + ",\"afrc\":" + jobd(obd::F_AFRCMD, 2);
  s += ",\"hpfp\":" + jobd(obd::F_HPFP, 0) + ",\"hpfps\":" + jobd(obd::F_HPFPSET, 0) + ",\"lpfp\":" + jobd(obd::F_LPFP, 0);
  { float i = idcNow(); s += ",\"idc\":" + (gr::has(i) ? jnum(i, 0) : String("null")); }
  s += ",\"cat1\":" + (obd::fresh(obd::F_CAT1, obd::ageFor(obd::F_CAT1)) ? jnum(obd::val[obd::F_CAT1], 0) : String("null"));
  s += ",\"cat2\":" + (obd::fresh(obd::F_CAT2, obd::ageFor(obd::F_CAT2)) ? jnum(obd::val[obd::F_CAT2], 0) : String("null"));
  s += ",\"amb\":" + (obd::fresh(obd::F_AMB, obd::ageFor(obd::F_AMB)) ? jnum(obd::val[obd::F_AMB], 0) : String("null"));
  s += ",\"pre\":" + (obd::fresh(obd::F_PRESC, obd::ageFor(obd::F_PRESC)) ? jnum(obd::val[obd::F_PRESC], 0) : String("null"));
  s += ",\"icp\":" + (obd::fresh(obd::F_ICPUMP, obd::ageFor(obd::F_ICPUMP)) ? jnum(obd::val[obd::F_ICPUMP], 0) : String("null"));
  s += ",\"clu\":" + (obd::fresh(obd::F_GBT1, obd::ageFor(obd::F_GBT1)) ? jnum(obd::val[obd::F_GBT1], 0) : String("null"));
  s += ",\"gbx\":" + (obd::fresh(obd::F_GBT2, obd::ageFor(obd::F_GBT2)) ? jnum(obd::val[obd::F_GBT2], 0) : String("null"));
  s += ",\"dif\":" + (obd::fresh(obd::F_DIFT2, obd::ageFor(obd::F_DIFT2)) ? jnum(obd::val[obd::F_DIFT2], 0) : String("null"));
  s += ",\"bypass\":" + jobd(obd::F_BYPASS, 1);
  s += ",\"bat\":" + jobd(obd::F_BAT, 2);
  s += ",\"tim\":" + jobd(obd::F_TIM, 1);
  s += ",\"map\":" + jobd(obd::F_MAP, 1);
  s += ",\"mapc\":" + jobd(obd::F_MAPCMD, 1);
  s += ",\"knock\":" + jobd(obd::F_KNOCK, 1);
  s += ",\"kcyl\":"; s += obd::knockCyl;
  s += ",\"gear\":"; s += obd::fresh(obd::F_KMH, 1500) ? String(gearEst.gear) : String("null");
  s += ",\"baro\":" + (obd::fresh(obd::F_BARO, obd::ageFor(obd::F_BARO)) ? jnum(obd::val[obd::F_BARO], 0) : String("null"));
  s += ",\"kmh\":" + jobd(obd::F_KMH, 0);
  s += "},\"obd\":{\"state\":\""; s += obd::stateName(obd::state);
  s += "\",\"rate\":"; s += jnum(obd::rate, 1); s += "}";
  s += ",\"log\":{\"on\":"; s += S.logOn ? "true" : "false";
  s += ",\"open\":"; s += (logx::open || logx::active) ? "true" : "false";
  s += ",\"file\":\""; s += (logx::curName[0] ? logx::curName + 1 : ""); s += "\"";
  s += ",\"bytes\":"; s += logx::curBytes;
  s += ",\"held\":"; s += logx::held ? "true" : "false"; s += "}";
  if (alertOn >= 0) { s += ",\"warn\":{\"text\":\""; s += jesc(gWatch.a[alertOn].text); s += "\",\"red\":"; s += gWatch.a[alertOn].sev >= 2 ? "true" : "false"; s += "}"; }
  s += ",\"pulls\":"; s += pullHistN;
  s += ",\"runs\":"; s += runsVer;
  s += ",\"hv\":"; s += healthVer;
  s += ",\"cold\":"; s += gr::has(gColdOilF) ? jnum(gColdOilF, 0) : String("null");
  s += ",\"clock\":"; s += clockOk ? "true" : "false";
  s += ",\"perf\":{\"fps\":"; s += fps; s += ",\"gap\":"; s += loopGapShown;
  s += ",\"draw\":"; s += jnum(drawMs, 1); s += ",\"push\":"; s += jnum(pushMs, 1); s += "}";
  s += ",\"lights\":"; s += lightsOn ? "true" : "false";
  s += ",\"br\":"; s += brightness;
  s += ",\"page\":"; s += page;
  s += ",\"pages\":"; s += (int)gr::P_COUNT;
  s += ",\"fps\":"; s += fps;
  s += ",\"g\":{\"imu\":"; s += hasIMU ? "true" : "false";
  s += ",\"state\":"; s += hasIMU ? gimu.state() : -1;
  s += ",\"fwd\":"; s += gimu.cal.fwdSrc;
  s += ",\"stops\":"; s += gimu.cal.upN;
  s += ",\"lat\":"; s += jnum(gimu.lat, 2);
  s += ",\"lon\":"; s += jnum(gimu.lon, 2);
  s += ",\"pk\":["; s += jnum(gimu.pkL, 2); s += ","; s += jnum(gimu.pkR, 2); s += ",";
  s += jnum(gimu.pkAcc, 2); s += ","; s += jnum(gimu.pkBrk, 2); s += "]}";
  s += "}";
  sendJson(s);
}

static void hSettingsGet() {
  String s = "{";
  s += "\"vPerPsi\":" + jnum(S.vPerPsi, 4) + ",\"obdTrim\":" + String(S.obdTrim) + ",\"oilTrimF\":" + jnum(S.oilTrimF, 0);
  s += ",\"autoAlert\":" + String(S.autoAlert) + ",\"autoPull\":" + String(S.autoPull) + ",\"autoLaunch\":" + String(S.autoLaunch);
  s += ",\"perfRollout\":" + String(S.perfRollout) + ",\"obdLagMs\":" + jnum(S.obdLagMs, 0);
  s += ",\"trimPsi\":" + jnum(gPsiTrim, 2) + ",\"trimN\":" + String((unsigned)gTrimN);
  s += ",\"redPsi\":" + jnum(S.redPsi, 0) + ",\"shiftRpm\":" + jnum(S.shiftRpm, 0) + ",\"redlineRpm\":" + jnum(S.redlineRpm, 0) + ",\"peakHoldS\":" + jnum(S.peakHoldS, 0);
  s += ",\"telltale\":" + String(S.telltale) + ",\"digital\":" + String(S.digital) + ",\"nightTheme\":" + String(S.nightTheme);
  s += ",\"brightDay\":" + String(S.brightDay) + ",\"brightDimMin\":" + String(S.brightDimMin) + ",\"brightDimMax\":" + String(S.brightDimMax);
  s += ",\"logOn\":" + String(S.logOn) + ",\"logEngineOnly\":" + String(S.logEngineOnly) + ",\"logHz\":" + String(S.logHz);
  s += ",\"obdOn\":" + String(S.obdOn) + ",\"obdName\":\"" + jesc(S.obdName) + "\"";   // escaped: a quote in a name broke the page
  s += ",\"wifiOn\":" + String(S.wifiOn) + ",\"apSsid\":\"" + jesc(S.apSsid) + "\",\"apPass\":\"" + jesc(S.apPass) + "\"";
  {                                              // which firmware is running
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_ota_img_states_t st = ESP_OTA_IMG_UNDEFINED;
    if (run) esp_ota_get_state_partition(run, &st);
    s += ",\"fw\":{\"version\":\"" FW_VERSION "\",\"built\":\"" __DATE__ " " __TIME__ "\",\"slot\":\""; s += run ? run->label : "?";
    s += "\",\"trial\":"; s += st == ESP_OTA_IMG_PENDING_VERIFY ? "true" : "false";
    s += ",\"rolledBack\":"; s += esp_ota_get_last_invalid_partition() ? "true" : "false"; s += "}";
  }
  s += "}";
  sendJson(s);
}

static float argf(const char *k, float def, float lo, float hi) {
  if (!server.hasArg(k)) return def;
  float v = server.arg(k).toFloat();
  return (v >= lo && v <= hi) ? v : def;
}
static bool argb(const char *k, bool def) { return server.hasArg(k) ? server.arg(k) == "1" : def; }

static void hSettingsPost() {
  Settings old = S;
  S.vPerPsi   = argf("vPerPsi", S.vPerPsi, 0.02f, 0.3f);
  S.redPsi    = argf("redPsi", S.redPsi, 5, 30);
  S.shiftRpm  = argf("shiftRpm", S.shiftRpm, 3000, 8000);
  S.redlineRpm = argf("redlineRpm", S.redlineRpm, 3000, 8000);
  S.peakHoldS = argf("peakHoldS", S.peakHoldS, 0, 120);
  S.telltale  = argb("telltale", S.telltale);
  S.digital   = argb("digital", S.digital);
  S.nightTheme = argb("nightTheme", S.nightTheme);
  S.brightDay = (uint8_t)argf("brightDay", S.brightDay, 10, 255);
  S.brightDimMin = (uint8_t)argf("brightDimMin", S.brightDimMin, 5, 255);
  S.brightDimMax = (uint8_t)argf("brightDimMax", S.brightDimMax, 5, 255);
  S.obdTrim   = argb("obdTrim", S.obdTrim);
  S.oilTrimF  = argf("oilTrimF", S.oilTrimF, -30, 30);
  S.autoAlert = argb("autoAlert", S.autoAlert);
  S.autoPull  = argb("autoPull", S.autoPull);
  S.autoLaunch = argb("autoLaunch", S.autoLaunch);
  S.perfRollout = argb("perfRollout", S.perfRollout);
  S.obdLagMs  = argf("obdLagMs", S.obdLagMs, 0, 500);
  S.logOn     = argb("logOn", S.logOn);
  S.logEngineOnly = argb("logEngineOnly", S.logEngineOnly);
  S.logHz     = (uint8_t)argf("logHz", S.logHz, 1, 25);
  S.obdOn     = argb("obdOn", S.obdOn);
  if (server.hasArg("obdName")) strlcpy(S.obdName, server.arg("obdName").c_str(), sizeof(S.obdName));
  S.wifiOn    = argb("wifiOn", S.wifiOn);
  if (server.hasArg("apSsid") && server.arg("apSsid").length() >= 1) strlcpy(S.apSsid, server.arg("apSsid").c_str(), sizeof(S.apSsid));
  if (server.hasArg("apPass") && server.arg("apPass").length() >= 8) strlcpy(S.apPass, server.arg("apPass").c_str(), sizeof(S.apPass));
  settingsSave(prefs);
  obd::enabled = S.obdOn;
  strlcpy(obd::nameFilter, S.obdName, sizeof(obd::nameFilter));
  if (old.redPsi != S.redPsi || old.shiftRpm != S.shiftRpm || old.redlineRpm != S.redlineRpm ||
      old.telltale != S.telltale || old.digital != S.digital) reqFaces = true;
  bool restart = old.wifiOn != S.wifiOn || strcmp(old.apSsid, S.apSsid) || strcmp(old.apPass, S.apPass);
  Serial.printf("[web] settings saved%s\n", restart ? " (Wi-Fi change needs a restart)" : "");
  sendJson(String("{\"ok\":true,\"restart\":") + (restart ? "true" : "false") + "}");
}

static void hCmd() {
  String d = server.arg("do");
  if (d == "peak") reqPeak = true;
  else if (d == "rezero") reqRezero = true;
  else if (d == "page") reqPage = server.arg("n").toInt() < 0 ? -1 : 1;
  else if (d == "logstop") reqLogStop = true;
  else if (d == "forget") { obd::savedAddr[0] = 0; prefs.remove("obdaddr2"); if (obd::client) obd::client->disconnect(); }
  else if (d == "defaults") { settingsClear(prefs); obd::enabled = S.obdOn; reqFaces = true; }
  else if (d == "restart") reqRestart = true;
  else if (d == "glevel") gReqGLevel = true;
  else if (d == "gforget") gReqGForget = true;
  else if (d == "gpeak") gReqGPeak = true;
  else if (d == "runsclear") reqRunsClear = true;
  else if (d == "formatflash") {                 // only offered after the internal flash failed to mount
    if (!logx::formatFlash()) { server.send(400, "application/json", "{\"ok\":false,\"error\":\"Not formatted: the internal storage is working, or the format failed.\"}"); return; }
    Serial.println(F("[log] internal flash formatted from the web page: restarting"));
    reqRestart = true;
  }
  else { server.send(400, "application/json", "{\"ok\":false}"); return; }
  sendJson("{\"ok\":true}");
}

static void hLogs() {
  using namespace logx;
  String s = "{\"fs\":"; s += fsOk ? "true" : "false";
  s += ",\"on\":"; s += S.logOn ? "true" : "false";
  s += ",\"open\":"; s += (logx::open || active) ? "true" : "false";
  s += ",\"held\":"; s += held ? "true" : "false";
  s += ",\"file\":\""; s += (curName[0] ? curName + 1 : ""); s += "\"";
  s += ",\"bytes\":"; s += curBytes;
  s += ",\"dropped\":"; s += dropped;
  s += ",\"worstMs\":"; s += worstWriteMs;
  s += ",\"err\":\""; s += jesc(lastErr); s += "\"";
  s += ",\"where\":\""; s += target().id; s += "\"";
  s += ",\"sd\":"; s += card.ok ? "true" : "false";
  s += ",\"flashBad\":"; s += flashBad ? "true" : "false";
  s += ",\"sdFreeKB\":"; s += freeKB[1];
  s += ",\"flashFreeKB\":"; s += freeKB[0];
  s += ",\"files\":[";
  // both places, newest first. Names only: the file being written is never opened here.
  struct E { String n; uint32_t size, k; const char *st; bool rec; uint32_t t; };
  std::vector<E> list;
  for (Store *st : {&card, &flash})
    forEachLog(*st, [&](const String &n, uint32_t k, uint32_t size) {
      bool rec = isRecording(*st, n);
      list.push_back({n, size, k, st->id, rec, rec ? 0u : fileTime(*st, n)});   // when the drive ended (0 = not known)
    });
  if (logx::open && cur && curName[0]) {                   // make sure the one being recorded is there
    bool have = false;
    for (auto &e : list) if (e.rec) have = true;
    if (!have) list.push_back({String(curName + 1), curBytes, logNum(String(curName + 1)), cur->id, true, 0u});
  }
  std::sort(list.begin(), list.end(), [](const E &a, const E &b) { return a.k > b.k; });
  bool first = true;
  for (auto &e : list) {
    if (!first) s += ",";
    first = false;
    s += "{\"n\":\"" + e.n + "\",\"s\":" + String(e.size) + ",\"at\":\"" + e.st + "\",\"rec\":" + (e.rec ? "true" : "false") + ",\"t\":" + String(e.t) + "}";
  }
  s += "]}";
  sendJson(s);
}

static bool safeName(const String &n) {
  if (!logx::logNum(n) || n.length() > 20) return false;
  for (char c : n) if (!(isalnum((unsigned char)c) || c == '_' || c == '.')) return false;
  return true;
}

static void hLogGet() {
  String n = server.arg("f");
  logx::Store &st = *logx::storeById(server.arg("at"));
  if (!st.ok || !safeName(n)) { server.send(404, "text/plain", "No such log"); return; }
  if (logx::isRecording(st, n)) { server.send(409, "text/plain", "Still recording: stop the log first"); return; }
  File f = st.fs->open("/" + n, FILE_READ);
  if (!f) { server.send(404, "text/plain", "No such log"); return; }
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + n + "\"");
  server.streamFile(f, "text/csv");
  f.close();
}

static void hLogDel() {
  String n = server.arg("f");
  logx::Store &st = *logx::storeById(server.arg("at"));
  bool ok = st.ok && safeName(n) && !logx::isRecording(st, n) && st.fs->remove("/" + n);
  if (ok) logx::refreshFree();
  sendJson(String("{\"ok\":") + (ok ? "true" : "false") + "}");
}

static String jesc(const char *t) {         // JSON string escape (adapter text can hold anything)
  String o;
  for (const char *c = t; *c; c++) {
    if (*c == '"' || *c == '\\') { o += '\\'; o += *c; }
    else if ((uint8_t)*c < 0x20) o += ' ';
    else o += *c;
  }
  return o;
}

static void hObd() {
  String s = "{\"state\":\""; s += obd::stateName(obd::state);
  s += "\",\"msg\":\""; s += jesc(obd::lastMsg);
  s += "\",\"dev\":\""; s += jesc(obd::devName);
  s += "\",\"addr\":\""; s += obd::devAddr;
  s += "\",\"rate\":"; s += jnum(obd::rate, 1);
  s += ",\"cacFromIat\":"; s += obd::cacFromIat ? "true" : "false";
  s += ",\"elm\":\""; s += jesc(obd::elmVer);
  s += "\",\"proto\":\""; s += jesc(obd::proto);
  s += "\",\"ecu\":"; s += obd::ecuHeader ? "true" : "false";
  s += ",\"suffix\":"; s += obd::useSuffix ? "true" : "false";
  s += ",\"suffix22\":"; s += obd::suffix22 ? "true" : "false";
  s += ",\"suffixMulti\":"; s += obd::suffixMulti ? "true" : "false";
  s += ",\"msSfx\":"; s += jnum(obd::msSfx, 0); s += ",\"msPlain\":"; s += jnum(obd::msPlain, 0);
  s += ",\"nSfx\":"; s += (unsigned)obd::nSfx; s += ",\"nPlain\":"; s += (unsigned)obd::nPlain;
  // polling: [request, asked ms, asked ms on boost, got ms at cruise, got ms on boost, state, flags]
  // (state 0 polled / 1 dropped / 2 not used; flags 1 = waits during a pull, 2 = only when stopped)
  s += ",\"poll\":[[\""; s += obd::speedInFast ? "010C0D" : "010C"; s += "\",0,0,";
  s += jnum(obd::fastGotMs[0], 0); s += ","; s += jnum(obd::fastGotMs[1], 0); s += ",0,0]";
  for (const obd::Slow &q : obd::slowList) {
    s += ",[\""; s += q.cmd; s += "\","; s += (unsigned long)q.periodMs; s += ","; s += (unsigned)(q.boostMs ? q.boostMs : 0); s += ",";
    s += jnum(q.gotMs[0], 0); s += ","; s += jnum(q.gotMs[1], 0); s += ","; s += obd::pollState(q); s += ",";
    s += (obd::pullPaused(q) ? 1 : 0) | (obd::stoppedOnly(q) ? 2 : 0); s += "]";
  }
  s += "]";
  // gearbox temperatures (degC, candidates: 221259 / 125A / 125B) and the odometer (km)
  s += ",\"gbt\":[";
  for (int i = 0; i < 3; i++) {
    int f = obd::F_GBT1 + i; if (i) s += ",";
    if (obd::fresh(f, obd::ageFor(f))) s += jnum((obd::val[f] - 32) / 1.8f, 0); else s += "null";
  }
  s += "],\"odo\":"; if (obd::odoAt && millis() - obd::odoAt < 180000) s += (unsigned long)obd::odoKm; else s += "null";
  s += ",\"dift\":[";
  for (int i = 0; i < 2; i++) {
    int f = obd::F_DIFT1 + i; if (i) s += ",";
    if (obd::fresh(f, obd::ageFor(f))) s += jnum((obd::val[f] - 32) / 1.8f, 0); else s += "null";
  }
  s += "],\"dmode\":"; s += obd::driveMode();
  s += ",\"epb\":"; s += obd::parkBrake();
  s += ",\"brake\":"; s += obd::brakeOn();
  s += ",\"car\":"; { uint32_t c = obd::carNow(); if (c && millis() - obd::carClockAt < 1200000) s += (unsigned long)c; else s += "null"; }
  s += ",\"tz\":"; if (tzOff != INT32_MIN) s += tzOff; else s += "null";
  s += ",\"clockSrc\":"; s += clockSrc;
  { char tm = obd::transMode(); s += ",\"trans\":\""; if (tm) s += tm; s += "\""; }
  s += ",\"log\":[";
  {
    static char lines[OBD_LOG_N][OBD_LOG_LEN];
    int n = obd::logCopy(lines, OBD_LOG_N);
    for (int i = 0; i < n; i++) { if (i) s += ","; s += "\""; s += jesc(lines[i]); s += "\""; }
  }
  s += "]";
  s += ",\"pids\":\"";
  for (int pid = 1; pid < 0x80; pid++) if (obdp::supported(obd::supportedMask, pid) && (pid & 0x1F)) { char b[4]; snprintf(b, sizeof(b), "%02X ", pid); s += b; }
  s += "\",\"reply\":";
  if (obd::consoleDone) {
    String r = obd::consoleReply; r.replace("\\", "\\\\"); r.replace("\"", "\\\""); r.replace("\n", "\\n");
    s += "\"" + r + "\"";
    obd::consoleDone = false;
  } else s += "null";
  s += "}";
  sendJson(s);
}

static void hObdCmd() {
  String c = server.arg("c");
  c.toUpperCase();
  if (c.length() < 2 || c.length() > 40) { sendJson("{\"ok\":false,\"error\":\"2 to 40 characters.\"}"); return; }
  if (obd::consolePending) { sendJson("{\"ok\":false,\"error\":\"The adapter is still busy with the last command.\"}"); return; }
  if (const char *why = obd::consoleCheck(c.c_str())) { sendJson(String("{\"ok\":false,\"error\":\"") + why + "\"}"); return; }
  strlcpy(obd::consoleCmd, c.c_str(), sizeof(obd::consoleCmd));
  obd::consoleDone = false;
  obd::consolePending = true;
  sendJson("{\"ok\":true}");
}

static void hRoot() {
  server.sendHeader("Content-Encoding", "gzip");
  server.sendHeader("Cache-Control", "no-cache");
  server.send_P(200, "text/html", (const char *)WEB_PAGE_GZ, WEB_PAGE_GZ_LEN);
}

// ---------------------------------------------------------------- firmware update
// POST /api/update with the app .bin (Arduino IDE: Sketch > Export Compiled Binary, the
// "S5GaugePod_Test.ino.bin"). It's written to the other app slot while this build keeps
// running; the pod restarts into it only if the whole file arrived and checks out. The new
// build is then on trial: if it crashes or hangs before it has run 30 s, the bootloader goes
// back to this one by itself (see otaConfirm in the sketch).
static String updErr;
static size_t updSize = 0, updDone = 0;
static void hUpdateChunk() {
  HTTPUpload &u = server.upload();
  if (u.status == UPLOAD_FILE_START) {
    updErr = ""; updDone = 0; updPct = 0;
    updSize = (size_t)server.header("X-Size").toInt();
    if (obd::fresh(obd::F_KMH, 3000) && obd::val[obd::F_KMH] > 3) updErr = "The car is moving. Update when it's parked.";
    else if (!Update.begin(updSize ? updSize : UPDATE_SIZE_UNKNOWN, U_FLASH)) updErr = String("Not enough room: ") + Update.errorString();
    else { updating = true; Serial.printf("[ota] receiving %s (%u bytes)\n", u.filename.c_str(), (unsigned)updSize); }
  } else if (u.status == UPLOAD_FILE_WRITE) {
    if (updErr.length()) return;
    if (updDone == 0) {                          // first block: an app image, not the merged / bootloader file
      const uint8_t *b = u.buf;
      bool app = u.currentSize >= 36 && b[0] == 0xE9 && b[32] == 0x32 && b[33] == 0x54 && b[34] == 0xCD && b[35] == 0xAB;
      if (!app) { updErr = "That isn't the app firmware. Use S5GaugePod_Test.ino.bin, not the .merged, .bootloader or .partitions file."; Update.abort(); updating = false; return; }
    }
    if (Update.write(u.buf, u.currentSize) != u.currentSize) { updErr = String("Write failed: ") + Update.errorString(); Update.abort(); updating = false; return; }
    updDone += u.currentSize;
    if (updSize) updPct = (int)(updDone * 100 / updSize);
  } else if (u.status == UPLOAD_FILE_END) {
    if (updErr.length()) return;
    if (!Update.end(true)) { updErr = String("The file didn't check out: ") + Update.errorString(); updating = false; }
    else Serial.printf("[ota] %u bytes written and checked\n", (unsigned)updDone);
  } else if (u.status == UPLOAD_FILE_ABORTED) {
    Update.abort(); updErr = "The upload was interrupted. Nothing was changed."; updating = false;
  }
}
static void hUpdateDone() {
  if (updErr.length() || !Update.isFinished()) {
    if (!updErr.length()) updErr = "The upload didn't finish. Nothing was changed.";
    Serial.printf("[ota] failed: %s\n", updErr.c_str());
    updating = false; updPct = -1;
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"" + jesc(updErr.c_str()) + "\"}");
    return;
  }
  sendJson("{\"ok\":true}");
  prefs.remove("otabad");                        // a fresh install: report the next failure again
  updPct = 100;
  reqRestart = true;                             // the loop closes the log, then restarts into the new build
}

// The last pulls (newest first) and how much the usual-boost curve has learned
static void hPulls() {
  auto n = [](float v, int dec) { return gr::has(v) ? jnum(v, dec) : String("null"); };
  String s = "{\"usual\":["; 
  for (int i = 0; i < watch::NB; i++) {
    if (i) s += ",";
    s += "{\"rpm\":" + String(2500 + i * 500) + ",\"psi\":" + (gWatch.curve.n[i] ? jnum(gWatch.curve.psi[i], 1) : String("null")) + ",\"n\":" + String(gWatch.curve.n[i]) + "}";
  }
  s += "],\"learned\":" + String(gWatch.curve.pulls) + ",\"pulls\":[";
  int cnt = pullHistN < 10 ? pullHistN : 10;
  for (int k = 0; k < cnt; k++) {
    const gr::PullSum &p = pullHist[(pullHistN - 1 - k) % 10];
    if (k) s += ",";
    s += "{\"ago\":" + String((millis() - p.endMs) / 1000);
    s += ",\"dur\":" + jnum(p.durS, 1) + ",\"gears\":\"" + String(p.gears) + "\"";
    s += ",\"kmh0\":" + jnum(p.kmh0, 0) + ",\"kmh1\":" + jnum(p.kmh1, 0);
    s += ",\"peak\":" + n(p.peakPsi, 1) + ",\"peakRpm\":" + jnum(p.peakRpm, 0) + ",\"obd\":" + n(p.obdPeak, 1) + ",\"tgt\":" + n(p.tgtPeak, 1);
    s += ",\"iat0\":" + n(p.iat0, 0) + ",\"iatMax\":" + n(p.iatMax, 0) + ",\"minTim\":" + n(p.minTiming, 1);
    s += ",\"knock\":" + n(p.knock, 1) + ",\"kcyl\":" + String(p.knockCyl);
    s += ",\"hpfpDef\":" + n(p.hpfpDef, 0) + ",\"hpfpMin\":" + n(p.hpfpMin, 0) + ",\"lpfpMin\":" + n(p.lpfpMin, 0);
    s += ",\"afrMax\":" + n(p.afrMax, 1) + ",\"afrOver\":" + n(p.afrOver, 1) + ",\"idcMax\":" + n(p.idcMax, 0) + ",\"catMax\":" + n(p.catMax, 0);
    s += ",\"bypassMin\":" + n(p.bypassMin, 1) + ",\"lonMax\":" + n(p.lonMax, 2);
    s += ",\"vsUsual\":" + n(p.vsNormal, 1) + ",\"slip\":" + String(p.slip ? "true" : "false") + "}";
  }
  s += "]}";
  sendJson(s);
}

// Identifier finder (OBD tab): status and the best candidates
static void hScanGet() {
  using namespace obd;
  static int idx[40];
  String s = "{\"mode\":"; s += (int)scanMode;
  s += ",\"hdr\":\""; s += scanHdr; s += "\"";
  char h[8];
  snprintf(h, sizeof(h), "%04X", scanFrom); s += ",\"from\":\""; s += h; s += "\"";
  snprintf(h, sizeof(h), "%04X", scanTo); s += ",\"to\":\""; s += h; s += "\"";
  s += ",\"done\":"; s += scanDone; s += ",\"total\":"; s += scanTotal;
  s += ",\"secs\":"; s += scanMode != SC_IDLE ? (millis() - scanStartMs) / 1000 : 0;
  s += ",\"msg\":\""; s += jesc(scanMsg); s += "\"";
  s += ",\"obd\":\""; s += stateName(state); s += "\"";
  s += ",\"mods\":[";
  {
    Mod ms[40]; int nm;
    portENTER_CRITICAL(&scanMux); nm = nMods; memcpy(ms, (const void *)mods, sizeof(Mod) * nm); portEXIT_CRITICAL(&scanMux);
    for (int i = 0; i < nm; i++) {
      char a[8], r[8]; snprintf(a, sizeof(a), "%03X", ms[i].a); snprintf(r, sizeof(r), "%03X", replyAddr(ms[i].a));
      if (i) s += ",";
      s += "{\"a\":\""; s += a; s += "\",\"r\":\""; s += r; s += "\",\"n\":\""; s += jesc(ms[i].name); s += "\"}";
    }
  }
  s += "]";
  portENTER_CRITICAL(&scanMux);
  int n = scanTab.n, ns = scanTab.ns;
  int m = (scanMode == SC_IDLE && scanTab.e) ? scanx::rank(scanTab, idx, 40) : 0;
  portEXIT_CRITICAL(&scanMux);
  s += ",\"found\":"; s += n;
  s += ",\"snaps\":[";
  for (int k = 0; k < ns; k++) { if (k) s += ","; s += "\""; s += jesc(scanTab.label[k]); s += "\""; }
  s += "],\"cands\":[";
  for (int i = 0; i < m; i++) {
    const scanx::Entry &x = scanTab.e[idx[i]];
    if (i) s += ",";
    snprintf(h, sizeof(h), "%04X", x.did);
    s += "{\"did\":\""; s += h; s += "\",\"score\":"; s += scanx::score(scanTab, x); s += ",\"v\":[";
    for (int k = 0; k < ns; k++) { char b[2 * scanx::MAXB + 2]; scanx::hex(x, k, b, sizeof(b)); if (k) s += ","; s += "\""; s += b; s += "\""; }
    s += "]}";
  }
  s += "]}";
  sendJson(s);
}
static void hScanPost() {
  using namespace obd;
  String d = server.arg("do");
  auto hexArg = [](const char *k, uint16_t def) { if (!server.hasArg(k)) return def; long v = strtol(server.arg(k).c_str(), nullptr, 16); return (v >= 0 && v <= 0xFFFF) ? (uint16_t)v : def; };
  // a scan pauses every reading (and so every warning) for minutes: not while driving
  if ((d == "discover" || d == "snap" || d == "modules") && fresh(F_KMH, 3000) && val[F_KMH] > 3) { sendJson("{\"ok\":false,\"error\":\"Only while the car is stopped: a scan pauses every reading and warning.\"}"); return; }
  if (state != S_RUN && (d == "discover" || d == "snap" || d == "modules")) { sendJson("{\"ok\":false,\"error\":\"the car isn't answering (ignition on, OBD connected?)\"}"); return; }
  strlcpy(scanReqLabel, server.arg("label").c_str(), sizeof(scanReqLabel));
  if (d == "discover") {
    String h = server.arg("hdr"); h.toUpperCase();
    if (h.length() < 3 || h.length() > 6) h = "7E0";
    strlcpy(scanReqHdr, h.c_str(), sizeof(scanReqHdr));
    scanReqFrom = hexArg("from", 0x1000); scanReqTo = hexArg("to", 0x2FFF);
    if (scanReqTo < scanReqFrom) { uint16_t t = scanReqFrom; scanReqFrom = scanReqTo; scanReqTo = t; }
    scanReq = 1;
  } else if (d == "snap") scanReq = 2;
  else if (d == "stop") scanReq = 3;
  else if (d == "clear") scanReq = 4;
  else if (d == "modules") scanReq = 5;
  else { server.send(400, "application/json", "{\"ok\":false}"); return; }
  sendJson("{\"ok\":true}");
}
// every identifier that answered, every snapshot, as CSV (to look at on a computer)
static void hScanCsv() {
  using namespace obd;
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendHeader("Content-Disposition", "attachment; filename=\"ident_scan.csv\"");
  server.send(200, "text/csv", "");
  String line = "header,did";
  for (int k = 0; k < scanTab.ns; k++) { line += ","; line += scanTab.label[k]; }
  line += ",score\n";
  server.sendContent(line);
  for (int i = 0; i < scanTab.n; i++) {
    const scanx::Entry &x = scanTab.e[i];
    char h[8]; snprintf(h, sizeof(h), "%04X", x.did);
    line = String(scanHdr) + "," + h;
    for (int k = 0; k < scanTab.ns; k++) { char b[2 * scanx::MAXB + 2]; scanx::hex(x, k, b, sizeof(b)); line += ","; line += b; }
    line += ","; line += scanx::score(scanTab, x); line += "\n";
    server.sendContent(line);
  }
  server.sendContent("");
}

// The phone's clock (sent by the page when it opens), so runs get a date
static void hTime() {
  uint32_t e = (uint32_t)strtoul(server.arg("s").c_str(), nullptr, 10);
  if (e < 1767225600UL) { server.send(400, "application/json", "{\"ok\":false}"); return; }   // before 2026: not a real time
  reqClock = e;
  sendJson("{\"ok\":true}");
}

// Performance runs: the last 30, newest first, and the best of each (with and without rollout)
// ECU self-tests (Mode 06): GET = the last results, POST = read them now
static void hSelfGet() {
  using namespace obd;
  static obdp::TestRec t[48]; int n; uint8_t ms[4], md[4]; bool mk;
  portENTER_CRITICAL(&selfMux);
  n = selfN; memcpy(t, selfTab, sizeof(obdp::TestRec) * n); memcpy(ms, monSince, 4); memcpy(md, monDrive, 4); mk = monKnown;
  portEXIT_CRITICAL(&selfMux);
  String s = "{\"busy\":"; s += (selfBusy || selfReq) ? "true" : "false";
  s += ",\"age\":"; if (selfAt) s += (millis() - selfAt) / 1000; else s += "null";
  s += ",\"obd\":\""; s += stateName(state); s += "\"";
  s += ",\"mon\":"; if (mk) { char b[64]; snprintf(b, sizeof(b), "{\"since\":[%u,%u,%u,%u],\"drive\":[%u,%u,%u,%u]}", ms[0], ms[1], ms[2], ms[3], md[0], md[1], md[2], md[3]); s += b; } else s += "null";
  s += ",\"tests\":[";
  for (int i = 0; i < n; i++) {
    char b[96]; snprintf(b, sizeof(b), "%s{\"mid\":%u,\"tid\":%u,\"uas\":%u,\"val\":%u,\"min\":%u,\"max\":%u}", i ? "," : "", t[i].mid, t[i].tid, t[i].uas, t[i].val, t[i].min, t[i].max);
    s += b;
  }
  s += "]}";
  sendJson(s);
}
static void hSelfPost() {
  if (obd::state != obd::S_RUN) { sendJson("{\"ok\":false,\"error\":\"the car isn't answering (ignition on, OBD connected?)\"}"); return; }
  obd::selfReq = true;
  sendJson("{\"ok\":true}");
}
static void hCatsBin() {
  File f = FFat.open("/cats.bin", "r");
  if (!f) { server.send(404, "text/plain", "no catalyst results yet"); return; }
  server.sendHeader("Cache-Control", "no-store");
  server.streamFile(f, "application/octet-stream");
  f.close();
}

// Health trends: the whole record file (the page decodes it), and baseline / clear
static void hHealthBin() {
  File f = FFat.open("/health.bin", "r");
  if (!f) { server.send(404, "text/plain", "no pulls yet"); return; }
  server.sendHeader("Cache-Control", "no-store");
  server.streamFile(f, "application/octet-stream");
  f.close();
}
static void hHealthPost() {
  String d = server.arg("do");
  if (d == "baseline") healthReq = 1;
  else if (d == "clear") healthReq = 2;
  else { server.send(400, "application/json", "{\"ok\":false}"); return; }
  sendJson("{\"ok\":true}");
}

static void hRuns() {
  static RunStore c;                             // static: too big for the web task's stack
  portENTER_CRITICAL(&runsMux); c = gRuns; portEXIT_CRITICAL(&runsMux);
  auto one = [](const RunRec &x) {
    String s = "{\"kind\":" + String(x.kind) + ",\"t\":" + jnum(x.t, 2) + ",\"mph\":" + jnum(x.mph, 1);
    s += ",\"psi\":" + (gr::has(x.psi) ? jnum(x.psi, 1) : String("null"));
    s += ",\"amb\":" + (gr::has(x.ambF) ? jnum(x.ambF, 0) : String("null"));
    s += ",\"when\":" + String(x.when) + ",\"ago\":" + String((millis() - x.ms) / 1000);
    s += ",\"thisBoot\":" + String((x.flags & RF_UNDATED) ? "true" : "false");
    s += ",\"log\":" + String(x.logNo) + ",\"obd\":" + String((x.flags & RF_OBD) ? "true" : "false");
    s += ",\"rollout\":" + String((x.flags & RF_ROLLOUT) ? "true" : "false") + "}";
    return s;
  };
  String s = "{\"clock\":"; s += clockOk ? "true" : "false";
  s += ",\"rollout\":"; s += S.perfRollout ? "true" : "false";
  s += ",\"runs\":[";
  for (int i = 0; i < c.n && i < 30; i++) { if (i) s += ","; s += one(c.r[i]); }
  s += "],\"best\":[";
  bool first = true;
  for (int ro = 0; ro < 2; ro++) for (int k = 0; k < 3; k++) if (c.best[ro][k].t > 0) { if (!first) s += ","; first = false; s += one(c.best[ro][k]); }
  s += "]}";
  sendJson(s);
}

static void task(void *) {
  for (;;) { server.handleClient(); vTaskDelay(pdMS_TO_TICKS(3)); }
}

static void begin() {
  if (!S.wifiOn) { Serial.println(F("[web] Wi-Fi off (setting)")); return; }
  WiFi.mode(WIFI_AP);
  WiFi.softAP(S.apSsid, S.apPass);
  WiFi.setSleep(false);
  MDNS.begin("s5pod");
  server.on("/", HTTP_GET, hRoot);
  server.on("/api/live", HTTP_GET, hLive);
  server.on("/api/settings", HTTP_GET, hSettingsGet);
  server.on("/api/settings", HTTP_POST, hSettingsPost);
  server.on("/api/cmd", HTTP_POST, hCmd);
  server.on("/api/logs", HTTP_GET, hLogs);
  server.on("/api/logdel", HTTP_POST, hLogDel);
  server.on("/log", HTTP_GET, hLogGet);
  server.on("/api/obd", HTTP_GET, hObd);
  server.on("/api/obdcmd", HTTP_POST, hObdCmd);
  server.on("/api/update", HTTP_POST, hUpdateDone, hUpdateChunk);
  server.on("/api/pulls", HTTP_GET, hPulls);
  server.on("/api/runs", HTTP_GET, hRuns);
  server.on("/health.bin", HTTP_GET, hHealthBin);
  server.on("/cats.bin", HTTP_GET, hCatsBin);
  server.on("/api/selftest", HTTP_GET, hSelfGet);
  server.on("/api/selftest", HTTP_POST, hSelfPost);
  server.on("/api/health", HTTP_POST, hHealthPost);
  server.on("/api/scan", HTTP_GET, hScanGet);
  server.on("/api/scan", HTTP_POST, hScanPost);
  server.on("/scan.csv", HTTP_GET, hScanCsv);
  server.on("/api/time", HTTP_POST, hTime);
  const char *hdrs[] = {"X-Size"};
  server.collectHeaders(hdrs, 1);
  server.onNotFound([] { server.send(404, "text/plain", "Not here. Open http://192.168.4.1/"); });
  server.begin();
  MDNS.addService("http", "tcp", 80);
  up = true;
  Serial.printf("[web] Wi-Fi \"%s\" up, open http://%s/ or http://s5pod.local/\n", S.apSsid, WiFi.softAPIP().toString().c_str());
  xTaskCreatePinnedToCore(task, "web", 8192, nullptr, 1, nullptr, 0);
}

}  // namespace web
