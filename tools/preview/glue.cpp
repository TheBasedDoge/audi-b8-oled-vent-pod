// Thin C API around gauge_render.h for the browser preview (compiled to wasm)
// and for tools/preview/native_test.cpp (compiled natively to render PNGs).
#include "../../firmware/S5GaugePod_Test/gauge_render.h"
using namespace gr;

static uint16_t FB[W * H];
static uint16_t FACES[2][P_COUNT][W * H];     // [day/night][page], like the firmware's PSRAM cache
static bool     faceOk[2][P_COUNT];
static bool     night = false;
static Cfg      cfg;
static Data     dat;
static char     statusBuf[64], toastBuf[64];
static float    demoOut[8];
static uint8_t  sysOrder[SI_COUNT] = {SI_BATTERY, SI_PEAK, SI_TIMING, SI_MAXIAT};
static SysAlertState alertState;

extern "C" {

#ifdef __wasm__
void *memcpy(void *d, const void *s, __SIZE_TYPE__ n) { char *a = (char *)d; const char *b = (const char *)s; while (n--) *a++ = *b++; return d; }
void *memset(void *d, int c, __SIZE_TYPE__ n) { char *a = (char *)d; while (n--) *a++ = (char)c; return d; }
#endif

uint16_t *g_fb() { return FB; }
char *g_status() { return statusBuf; }
char *g_toast() { return toastBuf; }

static void dirty(int page) { faceOk[0][page] = faceOk[1][page] = false; }
void g_cfg(float redPsi, float shiftRpm, float redlineRpm, int telltale, int digital) {
  if (redPsi != cfg.redPsi) { dirty(P_BOOST); dirty(P_OVERVIEW); }
  if (shiftRpm != cfg.shiftRpm || redlineRpm != cfg.redlineRpm) { dirty(P_ENGINE); dirty(P_OVERVIEW); }
  cfg.redPsi = redPsi; cfg.shiftRpm = shiftRpm; cfg.redlineRpm = redlineRpm;
  cfg.telltale = telltale != 0; cfg.digital = digital != 0;
}
void g_theme(int dark) { night = dark != 0; }

void g_set(float boost, float peak, int obd, float rpm, float thr, float iatF, float cltF, float oilF, float batV,
           float timing, float maxIat, float maxRpm, int statusWarn, float toastA, float dotsA) {
  dat.boost = boost; dat.boostText = boost; dat.peak = peak; dat.obd = obd != 0;
  dat.rpm = rpm; dat.thr = thr; dat.iatF = iatF; dat.cltF = cltF; dat.oilF = oilF; dat.batV = batV;
  dat.timing = timing; dat.maxIat = maxIat; dat.maxRpm = maxRpm;
  dat.status = statusBuf; dat.statusWarn = statusWarn != 0;
  dat.toast = toastBuf; dat.toastA = toastA; dat.dotsA = dotsA;
  dat.sysOrder = sysOrder;
}

// Once per frame (like renderFrame() in the .ino), after g_set and before g_render
void g_tick(float nowMs) { dat.sysAlert = sysAlert(alertState, cfg, dat, nowMs); }

void g_render(int page, float nowMs) {
  if (page < 0 || page >= P_COUNT) page = 0;
  int t = night ? 1 : 0;
  setTheme(night);
  if (!faceOk[t][page]) { target(FACES[t][page]); renderStatic(page, cfg); faceOk[t][page] = true; }
  uint16_t *s = FACES[t][page];
  target(FB);
  beginFrame(s, true);                         // same path as the firmware (full copy)
  renderDynamic(page, cfg, dat, nowMs);
  int16_t runs[16][2];
  endFrame(runs, 16);
}

// System page: tap at screen coords (0..465); returns 1 if an item was swapped in
int g_sys_tap(float x, float y) { return systemTap(sysOrder, x, y, dat.sysAlert) ? 1 : 0; }
uint8_t *g_sys_order() { return sysOrder; }

void g_splash(float fade, int warn) { target(FB); renderSplash(fade, statusBuf, warn != 0); }
void g_blank() { for (int i = 0; i < W * H; i++) FB[i] = 0; }

// demo drive cycle -> boost, rpm, thr, iatF, cltF, batV, timing, oilF
float *g_demo(float tSec) {
  Data d; demo(tSec, d);
  demoOut[0] = d.boost; demoOut[1] = d.rpm; demoOut[2] = d.thr; demoOut[3] = d.iatF;
  demoOut[4] = d.cltF; demoOut[5] = d.batV; demoOut[6] = d.timing; demoOut[7] = d.oilF;
  return demoOut;
}

float g_sweep(float s) { return sweep(s); }

// New pages (G-force, Perf) and the gear chip. drive = 1: everything from the demo cycle.
// drive = 0 (manual sliders): lat/lon/gear from the sliders, a trail built from the last
// 40 calls, results and per-gear peaks from the demo. obd = 0: no gear, timer can't arm.
static float trailBuf[2 * 40];
static int   trailN = 0;
void g_extra(int drive, float tSec, float lat, float lon, int gear, int gcal, int obd) {
  Data dd; demo(tSec, dd);
  dat.gTrail = dd.gTrail; dat.gTrailN = dd.gTrailN;
  dat.gPkL = dd.gPkL; dat.gPkR = dd.gPkR; dat.gPkAcc = dd.gPkAcc; dat.gPkBrk = dd.gPkBrk;
  dat.runState = dd.runState; dat.runT060 = dd.runT060; dat.runT6130 = dd.runT6130; dat.runTQm = dd.runTQm; dat.runMph = dd.runMph;
  dat.t060 = dd.t060; dat.t60130 = dd.t60130; dat.tQm = dd.tQm; dat.qmMph = dd.qmMph;
  dat.b060 = dd.b060; dat.b60130 = dd.b60130; dat.bQm = dd.bQm; dat.gearPeak = dd.gearPeak;
  dat.gLat = dd.gLat; dat.gLon = dd.gLon; dat.gear = dd.gear; dat.gCal = gcal;
  dat.trans = dd.trans; dat.driveMode = dd.driveMode;               // gear chip: S tab, DYNAMIC under it
  if (!drive) {
    dat.gLat = lat; dat.gLon = lon; dat.gear = gear; dat.runState = 0;
    if (trailN == 40) { for (int i = 0; i < 78; i++) trailBuf[i] = trailBuf[i + 2]; trailN--; }
    trailBuf[2 * trailN] = lat; trailBuf[2 * trailN + 1] = lon; trailN++;
    dat.gTrail = trailBuf; dat.gTrailN = trailN;
  } else trailN = 0;
  // Fuel / Heat / Last Pull pages: from the demo cycle (manual: the demo's values, rich/lean
  // following the boost slider); nothing without OBD
  dat.afr1 = dd.afr1; dat.afr2 = dd.afr2; dat.afrCmd = dd.afrCmd; dat.hpfp = dd.hpfp; dat.hpfpSet = dd.hpfpSet;
  dat.lpfp = dd.lpfp; dat.idc = dd.idc; dat.knock = dd.knock; dat.knockCyl = dd.knockCyl;
  dat.cat1F = dd.cat1F; dat.cat2F = dd.cat2F; dat.ambF = dd.ambF; dat.preF = dd.preF; dat.icPump = dd.icPump;
  dat.clutchF = dd.clutchF; dat.gbxF = dd.gbxF; dat.diffF = dd.diffF;
  dat.pull = dd.pull; dat.onBoost = dat.boost > 2;
  dat.tgtPsi = dat.boost > 1 ? minf(dat.boost + 3.4f, 17.8f) : NOV;
  if (!drive) { dat.afr1 = dat.onBoost ? 11.6f : 14.7f; dat.afr2 = dat.afr1 + 0.1f; dat.knock = 0; }
  if (!obd) {
    dat.gear = 0; dat.runState = 0; dat.trans = 0; dat.driveMode = 0;
    dat.afr1 = dat.afr2 = dat.afrCmd = dat.hpfp = dat.hpfpSet = dat.lpfp = dat.idc = dat.knock = NOV;
    dat.cat1F = dat.cat2F = dat.ambF = dat.preF = dat.icPump = NOV;
    dat.clutchF = dat.gbxF = dat.diffF = NOV;
    dat.tgtPsi = NOV;
  }
}
// Warning banner across the top of every page (the firmware's warnings), on / red
// a warning: big pop-up (big 0..1, pulse 0..1), then the banner
void g_alert(int on, int red, float big, float pulse) {
  dat.alertText = on ? (red ? "CAT 1 1790\xc2\xb0" "F" : "INTAKE 152\xc2\xb0" "F") : ""; dat.alertRed = red != 0;
  dat.alertTitle = red ? "CATALYST" : "INTAKE"; dat.alertValue = red ? "1790" : "152"; dat.alertUnit = "\xc2\xb0" "F";
  dat.alertNote = red ? "BANK 1" : ""; dat.alertBig = on ? big : 0; dat.alertPulse = pulse;
}

}  // extern "C"
