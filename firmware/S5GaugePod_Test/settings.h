// =====================================================================
//  S5 Gauge Pod - settings you can change from the web UI
//  Defaults come from config.h; anything saved from the web UI is kept in flash
//  (Preferences namespace "gaugepod", keys prefixed "s.") and wins over config.h.
//  "Reset to defaults" in the web UI clears them again.
// =====================================================================
#pragma once
#include <Preferences.h>

struct Settings {
  // sensor
  float   vPerPsi;          // AWE sender slope (calibration)
  bool    obdTrim;          // learn the sender's running offset against the ECU at idle
  float   oilTrimF;         // added to the ECU's oil temp (to match the dash, if wanted)
  bool    autoAlert;        // jump to the page of a value in warning (and come back after)
  bool    autoPull;         // show the Last Pull page for a few seconds after a pull
  bool    autoLaunch;       // go to the Perf page when the car is held for a launch (high steady RPM at a standstill)
  // performance timers
  bool    perfRollout;      // start 0-60 and 1/4 mile after the first foot of travel (drag strip / magazine timing)
  float   obdLagMs;         // how late OBD speed is, ms (the timers line it up with the G-meter)
  // gauge
  float   redPsi, shiftRpm, redlineRpm;
  float   peakHoldS;        // the dials' peak telltales fall back this long after a pull (0 = held until the peak is reset)
  bool    telltale, digital, nightTheme;
  // brightness
  uint8_t brightDay, brightDimMin, brightDimMax;
  // wifi
  bool    wifiOn;
  char    apSsid[24], apPass[24];
  // obd
  bool    obdOn;
  char    obdName[48];
  // logging
  bool    logOn, logEngineOnly;
  uint8_t logHz;
};

static Settings S;

static void settingsDefaults(Settings &s) {
  s.vPerPsi = AWE_V_PER_PSI; s.obdTrim = true; s.oilTrimF = 0; s.autoAlert = true; s.autoPull = true; s.autoLaunch = true;
  s.perfRollout = false; s.obdLagMs = PERF_OBD_LAG_MS;
  s.redPsi = GAUGE_RED_PSI; s.shiftRpm = SHIFT_RPM; s.redlineRpm = REDLINE_RPM; s.peakHoldS = PEAK_HOLD_S;
  s.telltale = SHOW_TELLTALE; s.digital = SHOW_DIGITAL; s.nightTheme = NIGHT_THEME;
  s.brightDay = BRIGHT_DAY; s.brightDimMin = BRIGHT_DIM_MIN; s.brightDimMax = BRIGHT_DIM_MAX;
  s.wifiOn = WIFI_ENABLED;
  strlcpy(s.apSsid, WIFI_AP_SSID, sizeof(s.apSsid));
  strlcpy(s.apPass, WIFI_AP_PASS, sizeof(s.apPass));
  s.obdOn = OBD_ENABLED;
  strlcpy(s.obdName, OBD_BLE_NAMES, sizeof(s.obdName));
  s.logOn = LOG_ENABLED; s.logEngineOnly = LOG_ENGINE_ONLY; s.logHz = LOG_HZ;
}

static void settingsLoad(Preferences &p) {
  settingsDefaults(S);
  S.vPerPsi     = p.getFloat("s.vpp", S.vPerPsi);
  S.obdTrim     = p.getBool("s.trim", S.obdTrim);
  S.oilTrimF    = p.getFloat("s.oilt", S.oilTrimF);
  S.autoAlert   = p.getBool("s.aal", S.autoAlert);
  S.autoPull    = p.getBool("s.apl", S.autoPull);
  S.autoLaunch  = p.getBool("s.alc", S.autoLaunch);
  S.perfRollout = p.getBool("s.rol", S.perfRollout);
  S.obdLagMs    = p.getFloat("s.lag", S.obdLagMs);
  S.redPsi      = p.getFloat("s.red", S.redPsi);
  S.shiftRpm    = p.getFloat("s.shift", S.shiftRpm);
  S.redlineRpm  = p.getFloat("s.redl", S.redlineRpm);
  S.peakHoldS   = p.getFloat("s.phold", S.peakHoldS);
  S.telltale    = p.getBool("s.tell", S.telltale);
  S.digital     = p.getBool("s.digi", S.digital);
  S.nightTheme  = p.getBool("s.night", S.nightTheme);
  S.brightDay   = p.getUChar("s.bday", S.brightDay);
  S.brightDimMin = p.getUChar("s.bmin", S.brightDimMin);
  S.brightDimMax = p.getUChar("s.bmax", S.brightDimMax);
  // Settings version 2 (2026-11): the day and night-max brightness defaults came down a little. A value
  // saved at the old default (255 / 200) follows the new one, once; anything set by hand is kept.
  if (p.getUChar("s.ver", 1) < 2) {
    if (S.brightDay == 255)    { S.brightDay = BRIGHT_DAY;        if (p.isKey("s.bday")) p.putUChar("s.bday", S.brightDay); }
    if (S.brightDimMax == 200) { S.brightDimMax = BRIGHT_DIM_MAX; if (p.isKey("s.bmax")) p.putUChar("s.bmax", S.brightDimMax); }
    p.putUChar("s.ver", 2);
  }
  S.wifiOn      = p.getBool("s.wifi", S.wifiOn);
  if (p.isKey("s.ssid")) p.getString("s.ssid", S.apSsid, sizeof(S.apSsid));
  if (p.isKey("s.pass")) p.getString("s.pass", S.apPass, sizeof(S.apPass));
  S.obdOn       = p.getBool("s.obd", S.obdOn);
  if (p.isKey("s.obdn")) p.getString("s.obdn", S.obdName, sizeof(S.obdName));
  S.logOn       = p.getBool("s.log", S.logOn);
  S.logEngineOnly = p.getBool("s.loge", S.logEngineOnly);
  S.logHz       = p.getUChar("s.loghz", S.logHz);
  // sanity: a bad value must never brick the gauge
  if (!(S.vPerPsi > 0.02f && S.vPerPsi < 0.3f)) S.vPerPsi = AWE_V_PER_PSI;
  if (S.logHz < 1 || S.logHz > 25) S.logHz = LOG_HZ;
  if (!(S.oilTrimF >= -30 && S.oilTrimF <= 30)) S.oilTrimF = 0;
  if (strlen(S.apPass) < 8) strlcpy(S.apPass, WIFI_AP_PASS, sizeof(S.apPass));
}

static void settingsSave(Preferences &p) {
  p.putFloat("s.vpp", S.vPerPsi); p.putBool("s.trim", S.obdTrim); p.putFloat("s.oilt", S.oilTrimF);
  p.putBool("s.aal", S.autoAlert); p.putBool("s.apl", S.autoPull); p.putBool("s.alc", S.autoLaunch);
  p.putBool("s.rol", S.perfRollout); p.putFloat("s.lag", S.obdLagMs);
  p.putFloat("s.red", S.redPsi); p.putFloat("s.shift", S.shiftRpm); p.putFloat("s.redl", S.redlineRpm); p.putFloat("s.phold", S.peakHoldS);
  p.putBool("s.tell", S.telltale); p.putBool("s.digi", S.digital); p.putBool("s.night", S.nightTheme);
  p.putUChar("s.bday", S.brightDay); p.putUChar("s.bmin", S.brightDimMin); p.putUChar("s.bmax", S.brightDimMax);
  p.putBool("s.wifi", S.wifiOn); p.putString("s.ssid", S.apSsid); p.putString("s.pass", S.apPass);
  p.putBool("s.obd", S.obdOn); p.putString("s.obdn", S.obdName);
  p.putBool("s.log", S.logOn); p.putBool("s.loge", S.logEngineOnly); p.putUChar("s.loghz", S.logHz);
}

static void settingsClear(Preferences &p) {
  const char *keys[] = {"s.vpp", "s.trim", "s.oilt", "s.aal", "s.apl", "s.alc", "s.rol", "s.lag", "s.red", "s.shift", "s.redl", "s.phold", "s.tell", "s.digi", "s.night", "s.bday", "s.bmin",
                        "s.bmax", "s.wifi", "s.ssid", "s.pass", "s.obd", "s.obdn", "s.log", "s.loge", "s.loghz"};
  for (const char *k : keys) p.remove(k);
  settingsDefaults(S);
}
