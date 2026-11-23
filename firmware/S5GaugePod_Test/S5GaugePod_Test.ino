// =====================================================================
//  S5 Gauge Pod firmware (in the car since 2026-10; the sketch keeps its bring-up name)
//  2016 Audi S5 3.0T supercharged, vent-mounted round gauge
//
//  Board : Waveshare ESP32-S3-Touch-AMOLED-1.43 (ESP32-S3R8, 16MB flash, 8MB OPI PSRAM)
//  Sensor: SENSOR_TYPE in config.h
//    SENSOR_AWE     : existing AWE sender, 12 V supply, 0-5 V out -> 68k/100k -> ADS1115 A0
//                     sender 12 V supply (optional, SUPPLY_SENSE_ENABLED) -> 47k/10k -> A1
//    SENSOR_MPX4250 : NXP MPX4250AP, 5 V ratiometric            -> 68k/100k -> A0 / A1
//
//  What's in here (ENGINEERING.md has the full description):
//    1. Display: CO5300 panel (SH8601 or auto-detect: config.h LCD_PANEL). Eleven pages drawn by
//       gauge_render.h - the same code the browser preview runs, so docs/screen-mockup.html
//       shows what this screen shows
//    2. I2C: scans the shared bus; anything missing is shown in amber under the S badge at boot
//    3. Sensor: ADS1115 sampling in its own task (~500 Hz), key-on zero (atmosphere) captured
//       and saved to flash; OBD manifold pressure stands in if the sender fails
//    4. Touch: hold 1 s = reset peak (System page: tap a small item to make it the big one),
//       hold 3 s = re-zero (ENGINE OFF only), swipe = page
//    5. Dimming: follows the dash dimmer via the lighting wire on GPIO1
//    6. OBD over a Veepeak BLE adapter (obd_ble.h), warnings and pull summaries (watch.h),
//       performance timers (perf.h), G-meter (imu.h), drive logs (logger.h), health trends
//       (health.h), Wi-Fi web page and firmware updates (web_ui.h)
//    7. DEMO mode: with no ADS1115 and no OBD, the preview's drive cycle drives every page
//
//  Serial (115200, USB CDC):  p = reset peak   z = re-zero        d = toggle demo    P = draw timing
//                             s = I2C scan     1-9 / n = page     l = night face   h = help
//
//  Arduino IDE settings (Tools menu):
//    Board: "ESP32S3 Dev Module"   USB CDC On Boot: Enabled
//    Flash Size: 16MB              PSRAM: OPI PSRAM
//    Partition Scheme: 16M Flash (3MB APP/9.9MB FATFS)
//  Libraries: "GFX Library for Arduino" (moononournation), "Adafruit ADS1X15"
// =====================================================================

#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_ADS1X15.h>
#include <esp_task_wdt.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include "config.h"
#include "gauge_render.h"     // shared with the browser preview
#include "settings.h"         // web-UI settings (defaults from config.h)
#include "obd_ble.h"          // Veepeak BLE / ELM327
#include "logger.h"           // CSV logs on the FFat partition
#include "frame_pipe.h"       // two frame buffers: draw the next frame while the last one is sent
#include "imu.h"              // QMI8658 motion sensor + G-meter calibration (imu_cal.h)
#include "gear.h"             // gear from RPM / speed
#include "watch.h"            // pulls, usual boost, belt slip, warnings
#include "perf.h"             // 0-60, 60-130, 1/4 mile
#include "health.h"           // one record per pull, for the Health tab
#include <sys/time.h>

// ---------------------------------------------------------------------
//  Globals
// ---------------------------------------------------------------------
static const float   KPA_TO_PSI  = 0.1450377f;
static const float   PSI_TO_INHG = 2.03602f;

Arduino_DataBus *bus    = nullptr;
Arduino_OLED    *panel  = nullptr;
Arduino_Canvas  *gfx    = nullptr;
// Static layer of each page. Day ones are all drawn at boot (8 x 434 KB). Night ones (headlights
// on, black face) are drawn the first time a page is shown at night, into a small pool that keeps
// the most recently used: 8 pages x 2 themes wouldn't fit in the 8 MB PSRAM next to the frame buffers.
// Page backgrounds (434 KB each in PSRAM): pools that keep the most recently used pages, so
// there can be more pages than buffers. A page outside its pool is redrawn when shown (~40 ms).
static const int DAY_SLOTS = 8;
uint16_t        *dayBuf[DAY_SLOTS];
int8_t           dayPage[DAY_SLOTS] = {-1, -1, -1, -1, -1, -1, -1, -1};
uint32_t         dayUsed[DAY_SLOTS];
static const int NIGHT_SLOTS = 4;
uint16_t        *nightBuf[NIGHT_SLOTS];
int8_t           nightPage[NIGHT_SLOTS] = {-1, -1, -1, -1};
uint32_t         nightUsed[NIGHT_SLOTS];
static const uint16_t *pageFace(int night, int p);
gr::Cfg          gcfg;
bool             panelIsSH8601 = false;
uint8_t          panelId = 0;

Adafruit_ADS1115 ads;
Preferences      prefs;

// I2C presence flags
bool hasTouch = false, hasRTC = false, hasIMU = false, hasADS = false;

// G-meter. The sensor task owns gimu (100 Hz); the UI loop reads its outputs and saves the
// calibration when it changes. Requests from touch / web / serial go through the flags.
imu::Proc         gimu;
volatile bool     gReqGLevel = false, gReqGForget = false, gReqGPeak = false;
portMUX_TYPE      imuMux = portMUX_INITIALIZER_UNLOCKED;
imu::Cal          imuCalOut;                       // copy for the loop to save (under imuMux)
volatile bool     imuCalPending = false;
volatile const char *imuEvent = nullptr;          // last calibration event, for the serial log
volatile uint32_t imuErrors = 0;
float             gTrail[2 * 40]; int gTrailN = 0; // last 1.3 s of the dot, oldest first (UI loop)

// Gear: the OBD task feeds gearEst (RPM and speed from one reply); the UI loop reads .gear and
// saves the learned ratios. gearPk = peak boost reached in each gear on the throttle (Gears page).
gearx::Est        gearEst;
portMUX_TYPE      gearMux = portMUX_INITIALIZER_UNLOCKED;
float             gearPk[7] = {0, 0, 0, 0, 0, 0, 0};

// Watcher (UI loop only): pull summaries and warnings. A warning takes the screen to its page
// with a banner, and goes back to where you were when it's over (or when you tap it away).
watch::Watch      gWatch;
int               alertFrom = -1;              // page before a warning took over (-1 = none)
int               alertOn = -1;                // the warning on screen (-1 = none)
bool              alertAck[watch::A_N];        // tapped away: not again until it clears
int               alertAckSev[watch::A_N];     // ... unless it gets worse (amber -> red)
int               pullFrom = -1;               // page before the Last Pull page popped up
uint32_t          pullUntil = 0;
bool              autoPaged = false;           // the current page was chosen by the pod, not you
int               launchFrom = -1;             // page before a launch took the screen to Perf (-1 = none)
uint32_t          engineSince = 0;             // engine running since (0 = not running)
uint32_t          alertPopAt = 0;              // when the warning on screen popped up (big, then the banner)
float             gColdOilF = gr::NOV;         // oil temp while it's still warming up (NOV = warm / unknown)
gr::PullSum       pullHist[10];                // the last pulls, newest at [ (pullHistN - 1) % 10 ] (web page)
int               pullHistN = 0;

// Performance timers: the sensor task steps gPerf 100 times a second (G-meter + OBD speed);
// the UI loop collects finished runs, shows them and keeps the last 30 (Runs tab on the web
// page) plus the best of each, saved to flash. Dates come from the board's clock, which the
// phone sets whenever the web page is open (it doesn't keep time with the power off).
perf::Timer       gPerf;
portMUX_TYPE      perfMux = portMUX_INITIALIZER_UNLOCKED;
struct RunRec {
  uint32_t when;                                 // unix time (0 = date unknown)
  uint32_t ms;                                   // millis() when it finished (to date it later in the same power-on)
  float    t, mph, psi, ambF;                    // s, end speed (trap speed for the 1/4), peak boost, outside air
  uint16_t logNo;                                // the drive log it's in (log_0012.csv = 12; 0 = none)
  uint8_t  kind, flags;
};
enum { RF_OBD = 1, RF_ROLLOUT = 2, RF_UNDATED = 4 };   // RF_UNDATED: from this power-on, waiting for the clock
struct RunStore { uint32_t magic; uint16_t n, pad; RunRec r[30]; RunRec best[2][3]; };   // best[rollout][kind]
static const uint32_t RUNS_MAGIC = 0x52554E31;
RunStore          gRuns;                         // newest first
portMUX_TYPE      runsMux = portMUX_INITIALIZER_UNLOCKED;   // the web task reads it
volatile uint32_t runsVer = 0;                   // bumped on every change (the web page re-reads the list)
bool              runsDirty = false;
bool              gStaged = false;               // held at a stop with the revs up, steady (launch control)
bool              clockOk = false;               // the clock has the real date (kept by the RTC, or from the phone)
uint8_t           clockSrc = 0;                  // 0 not set, 1 RTC, 2 phone, 3 the car's clock
// The car's clock is local time; the pod keeps UTC. The difference is learned whenever the clock
// is right from the RTC or the phone (rounded to 15 min) and saved, so later the car alone can set it.
int32_t           tzOff = INT32_MIN;             // car local - UTC, s (INT32_MIN = not learned yet)
// health trends: pulls wait here until things are calm, then go to /ffat/health.bin
static const char *HEALTH_PATH = "/ffat/health.bin", *HEALTH_TMP = "/ffat/health.tmp";
health::Rec       healthPend[8];
int               healthPendN = 0;
volatile bool     healthDateReq = false;         // the clock just arrived: date this power-on's records
volatile uint8_t  healthReq = 0;                 // from the web page: 1 = new baseline, 2 = clear
volatile uint32_t healthVer = 0;                 // bumped when the file changes
static const char *CATS_PATH = "/ffat/cats.bin";
health::CatRec    catSaved; bool catSavedOk = false, catPend = false;
health::CatRec    catNew;
uint16_t          bootNo = 0;                    // power-on count (records from before the clock are matched by it)

// Shared between sensor task (core 0) and UI loop (core 1).
// 32-bit aligned floats/bools are written atomically on the ESP32.
volatile float    gVsensPin  = 0;     // ADS A0 volts at the pin
volatile float    gVsupPin   = 0;     // ADS A1 volts at the pin
volatile float    gPsiRaw    = 0;     // unfiltered, relative to atmosphere
// Running offset of the sender against the ECU's own manifold pressure, learned at idle
// (settings: obdTrim). Subtracted from every reading. The first drives showed the sender
// reading a steady ~0.35 psi more vacuum than the ECU at idle.
volatile float    gPsiTrim   = 0;
volatile uint32_t gTrimN     = 0;     // idle samples it has learned from
volatile float    gPsiFilt   = 0;     // needle (NEEDLE_ALPHA)
volatile float    gPsiText   = 0;     // digital readout (TEXT_ALPHA)
volatile float    gWinMin = 99, gWinMax = -99;   // signal volts min/max since the last log line (noise check)
volatile bool     gWinReset  = false;
volatile bool     gSensorFault = false;          // signal outside SENSOR_MIN_V..SENSOR_MAX_V for 0.2 s
volatile bool     gBoostObd = false;             // boost is coming from OBD (no ADS1115, or the sender is faulty)
volatile float    gZero      = 0;     // AWE: signal volts at atmosphere. MPX: baro kPa
volatile float    gPeakPsi   = -99;
volatile uint32_t gSamples   = 0;
volatile bool     gDemo      = false;
// demo: asked for (serial d), or no ADS1115 and no OBD manifold pressure to fall back on
static inline bool isDemo() { return gDemo || (!hasADS && !gBoostObd); }
// the boost reading means something (the sender, or OBD standing in for it)
static inline bool boostValid() { return !gDemo && ((hasADS && !gSensorFault) || gBoostObd); }
volatile bool     gReqRezero = false;
volatile bool     gZeroWide = false;       // this re-zero may accept a lower reading: the ECU says the engine is stopped
volatile float    gZeroExpect = 0;         // ...and it must land near this (the zero the ECU's barometer predicts)
volatile uint32_t gZeroGen = 0;            // counts accepted zeros (to pair each one with the barometer)
volatile uint32_t gZeroAt = 0;             // millis() of the last accepted zero
float             gZeroRefV = 0, gZeroRefKpa = 0;   // a zero and the ECU's barometer at the time (0 = not learned yet)
volatile bool     gReqPeakReset = false;
volatile bool     gZeroFromBoot = false;   // true = captured this key cycle, false = saved/default
volatile bool     gZeroDone  = false;      // set when a manual re-zero finishes (for the toast)

// UI state
float    lightV = 0;
bool     lightsOn = false;
uint8_t  brightness = 255;
uint32_t frames = 0, fps = 0, sps = 0;
float    drawMs = 0, waitMs = 0;                 // frame timing (smoothed), shown in the log
volatile float pushMs = 0, rowsPct = 0;          // measured by the push task
volatile int16_t brightReq = -1;                 // brightness for the push task to apply (it owns the panel)
uint32_t loopGapMax = 0, loopGapShown = 0;        // longest time between two loop passes (ms): freezes show here
uint32_t skipped = 0, skippedShown = 0;          // frames with nothing new on screen (not drawn or sent)
uint16_t *bufs[2];                               // frame buffers: [0] = the canvas, [1] = second PSRAM buffer
fp::Job   jobs[2];
QueueHandle_t     pushQ;                         // index of the buffer to send
SemaphoreHandle_t pushIdle;                      // given when the push task has finished a frame
static const int  STAGE_PX = 2048;               // internal-RAM staging for partial-width rectangles
uint16_t         *stage = nullptr;
bool     needFull = true;                   // next frame redraws and sends the whole screen
bool     fastLog = false;                   // serial 'f': log every 20 ms
QueueHandle_t touchQ;
struct TouchEv { uint8_t type; int16_t x, y; };   // 1 tap, 2 long press, 3 next page, 4 previous page, 5 hold (reset)
int      page = START_PAGE;
uint32_t pageAt = 0, toastAt = 0, readyAt = 0, toastMs = 1200;   // millis() of last page change / toast / end of sweep
char     toastMsg[32] = "";
float    maxIat = 0, maxRpm = 0;
uint8_t  sysOrder[gr::SI_COUNT];            // System page: [0] = big item at the top (saved to flash)
gr::SysAlertState sysAlertState;            // warnings that pull an item to the top of the System page
int      sysAlertNow = -1;
bool     forceNight = false;                // serial 'l': night face without the headlight wire (bench)
int      shiftIdx = 0;

// ---------------------------------------------------------------------
//  Crash diagnostics. RTC memory keeps its contents through a reset (not a power cut),
//  so after an unexpected reboot the log and the splash say why and how long it ran.
// ---------------------------------------------------------------------
RTC_NOINIT_ATTR uint32_t rtcMagic, rtcUpMin, rtcMinHeapK, rtcMinPsramK;
RTC_NOINIT_ATTR uint32_t rtcStage, rtcGapMax;      // where the UI loop was (for a watchdog reset), longest pause
static const char *STAGE_NAMES[] = {"?", "serial", "touch", "web requests", "logging", "dimming", "drawing", "log line"};
#define STAGE(n) (rtcStage = (n))
static char lastReset[56] = "";                 // shown on the splash if it wasn't a normal power-on
static uint32_t minHeap = 0xFFFFFFFF;

static const char *resetName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "power on";      case ESP_RST_SW:       return "restart";
    case ESP_RST_PANIC:   return "CRASH";         case ESP_RST_INT_WDT:  return "INT WATCHDOG";
    case ESP_RST_TASK_WDT: return "TASK WATCHDOG"; case ESP_RST_WDT:     return "WATCHDOG";
    case ESP_RST_BROWNOUT: return "BROWNOUT";     case ESP_RST_USB:      return "usb";
    case ESP_RST_EXT:     return "reset button";  case ESP_RST_DEEPSLEEP: return "wake";
    default:              return "unknown";
  }
}

static void bootReport() {
  esp_reset_reason_t r = esp_reset_reason();
  bool had = rtcMagic == 0x5A17C0DE && r != ESP_RST_POWERON;
  if (had)
    Serial.printf("[boot] reset reason: %s, previous run %lu min, lowest free heap %lu KB, PSRAM %lu KB\n",
                  resetName(r), (unsigned long)rtcUpMin, (unsigned long)rtcMinHeapK, (unsigned long)rtcMinPsramK);
  else
    Serial.printf("[boot] reset reason: %s\n", resetName(r));
  if (had && rtcGapMax > 200) Serial.printf("[boot] longest UI pause in the previous run: %lu ms\n", (unsigned long)rtcGapMax);
  if (had && r == ESP_RST_TASK_WDT)
    Serial.printf("[boot] the UI loop was stuck in: %s\n", rtcStage < sizeof(STAGE_NAMES) / sizeof(STAGE_NAMES[0]) ? STAGE_NAMES[rtcStage] : "?");
  if (r == ESP_RST_PANIC || r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT || r == ESP_RST_WDT || r == ESP_RST_BROWNOUT) {
    if (had) snprintf(lastReset, sizeof(lastReset), "LAST RESET: %s AFTER %lu MIN", resetName(r), (unsigned long)rtcUpMin);
    else     snprintf(lastReset, sizeof(lastReset), "LAST RESET: %s", resetName(r));
  }
  rtcMagic = 0x5A17C0DE; rtcUpMin = 0; rtcMinHeapK = 0; rtcMinPsramK = 0; rtcStage = 0; rtcGapMax = 0;
}

// ---------------------------------------------------------------------
//  Small helpers
// ---------------------------------------------------------------------
#if SENSOR_TYPE == SENSOR_AWE
  #define ZERO_DEFAULT AWE_ZERO_DEFAULT_V
  #define ZERO_MIN     AWE_ZERO_MIN_V
  #define ZERO_WIDE_MIN AWE_ZERO_WIDE_MIN_V
  #define ZERO_MAX     AWE_ZERO_MAX_V
  #define ZERO_KEY     "zeroV"
  #define ZERO_UNIT    "V"
  static const char *SENSOR_NAME = "AWE sender";
#else
  #define ZERO_DEFAULT BARO_DEFAULT_KPA
  #define ZERO_MIN     BARO_MIN_KPA
  #define ZERO_WIDE_MIN BARO_MIN_KPA
  #define ZERO_MAX     BARO_MAX_KPA
  #define ZERO_KEY     "baro"
  #define ZERO_UNIT    "kPa"
  static const char *SENSOR_NAME = "MPX4250AP";
#endif

// ---------------------------------------------------------------------
//  Panel ID read (bit-banged, same sequence as Waveshare's demo), only with LCD_PANEL 0.
//  Returns 0x86 for SH8601; CO5300 panels return something else (usually 0xFF).
// ---------------------------------------------------------------------
#if LCD_PANEL == 0
static void bbWrite(uint8_t v) {
  for (int i = 0; i < 8; i++) {
    digitalWrite(PIN_LCD_D0, (v & 0x80) ? HIGH : LOW);
    v <<= 1;
    digitalWrite(PIN_LCD_SCK, LOW);
    digitalWrite(PIN_LCD_SCK, HIGH);
  }
}
static uint8_t bbRead() {
  uint8_t d = 0;
  pinMode(PIN_LCD_D0, INPUT_PULLUP);
  for (int i = 0; i < 8; i++) {
    digitalWrite(PIN_LCD_SCK, LOW);
    delayMicroseconds(1);
    d = (d << 1) | (digitalRead(PIN_LCD_D0) ? 1 : 0);
    digitalWrite(PIN_LCD_SCK, HIGH);
    delayMicroseconds(1);
  }
  pinMode(PIN_LCD_D0, OUTPUT);
  return d;
}
static uint8_t readPanelId() {
  const int pins[] = {PIN_LCD_CS, PIN_LCD_SCK, PIN_LCD_D0, PIN_LCD_D1, PIN_LCD_D2, PIN_LCD_D3, PIN_LCD_RST};
  for (int p : pins) { pinMode(p, OUTPUT); digitalWrite(p, HIGH); }
  digitalWrite(PIN_LCD_RST, HIGH); delay(120);
  digitalWrite(PIN_LCD_RST, LOW);  delay(120);
  digitalWrite(PIN_LCD_RST, HIGH); delay(120);
  digitalWrite(PIN_LCD_CS, LOW);
  bbWrite(0x03); bbWrite(0x00); bbWrite(0xDA); bbWrite(0x00);   // read ID1
  uint8_t id = bbRead();
  digitalWrite(PIN_LCD_CS, HIGH);
  return id;
}
#endif

// ---------------------------------------------------------------------
//  I2C
// ---------------------------------------------------------------------
static bool i2cPing(uint8_t a) { Wire.beginTransmission(a); return Wire.endTransmission() == 0; }

// A device left mid-transfer by a reset (the ESP restarts, the touch chip and ADS don't) can
// hold SDA low and lock the whole bus. Clock it free and send a STOP before Wire starts.
static bool i2cRecover() {
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  pinMode(PIN_I2C_SCL, OUTPUT_OPEN_DRAIN); digitalWrite(PIN_I2C_SCL, HIGH);
  delayMicroseconds(10);
  bool stuck = digitalRead(PIN_I2C_SDA) == LOW;
  for (int i = 0; i < 18 && digitalRead(PIN_I2C_SDA) == LOW; i++) {
    digitalWrite(PIN_I2C_SCL, LOW);  delayMicroseconds(5);
    digitalWrite(PIN_I2C_SCL, HIGH); delayMicroseconds(5);
  }
  pinMode(PIN_I2C_SDA, OUTPUT_OPEN_DRAIN);          // STOP: SDA low -> high while SCL is high
  digitalWrite(PIN_I2C_SDA, LOW); delayMicroseconds(5);
  digitalWrite(PIN_I2C_SDA, HIGH); delayMicroseconds(5);
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  bool ok = digitalRead(PIN_I2C_SDA) == HIGH;
  if (stuck) Serial.printf("[i2c] bus was held low at boot -> %s\n", ok ? "freed" : "STILL STUCK");
  return ok;
}

// The Adafruit library's readADC_SingleEnded() waits forever for the conversion; if the bus
// misbehaves that hangs the caller. This one gives up after timeoutMs.
static bool adsReadBounded(uint8_t ch, float &volts, uint32_t timeoutMs = 20) {
  static const uint16_t MUX[] = {ADS1X15_REG_CONFIG_MUX_SINGLE_0, ADS1X15_REG_CONFIG_MUX_SINGLE_1,
                                 ADS1X15_REG_CONFIG_MUX_SINGLE_2, ADS1X15_REG_CONFIG_MUX_SINGLE_3};
  ads.startADCReading(MUX[ch & 3], false);
  uint32_t t0 = millis();
  while (!ads.conversionComplete()) { if (millis() - t0 > timeoutMs) return false; delayMicroseconds(200); }
  volts = ads.computeVolts(ads.getLastConversionResults());
  return true;
}

// FT3168 setup: normal mode, stay active (no slow monitor mode). Also called after the touch
// chip comes back from a dropout.
static void touchInit() {
  Wire.beginTransmission(ADDR_TOUCH); Wire.write(0x00); Wire.write(0x00); Wire.endTransmission();
  Wire.beginTransmission(ADDR_TOUCH); Wire.write(0x86); Wire.write(0x00); Wire.endTransmission();
}

static void i2cScan() {
  Serial.println(F("[i2c] scanning..."));
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (i2cPing(a)) {
      const char *name = "?";
      if (a == ADDR_TOUCH) name = "FT3168 touch";
      else if (a == ADDR_RTC) name = "PCF85063 RTC";
      else if (a == 0x6A || a == 0x6B) name = "QMI8658 IMU";
      else if (a >= 0x48 && a <= 0x4B) name = "ADS1115";
      Serial.printf("[i2c]   0x%02X  %s\n", a, name);
      n++;
    }
  }
  Serial.printf("[i2c] %d device(s)\n", n);
}

// FT3168: reg 0x02 = touch count, 0x03..0x06 = X/Y (12-bit)
// Returns 1 = finger down (x, y set), 0 = no finger, -1 = I2C read failed (state unknown)
static int touchRead(uint16_t &x, uint16_t &y) {
  Wire.beginTransmission(ADDR_TOUCH);
  Wire.write(0x02);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((uint8_t)ADDR_TOUCH, (uint8_t)5) != 5) return -1;
  uint8_t n = Wire.read();
  uint8_t xh = Wire.read(), xl = Wire.read(), yh = Wire.read(), yl = Wire.read();
  if (n == 0 || n > 2) return 0;
  x = ((xh & 0x0F) << 8) | xl;
  y = ((yh & 0x0F) << 8) | yl;
  return 1;
}

// Raw touch -> logical (upright) screen coordinates
static void mapTouch(uint16_t rx, uint16_t ry, int &x, int &y) {
  int px = rx, py = ry;
  if (TOUCH_SWAP_XY) { int t = px; px = py; py = t; }
  if (TOUCH_FLIP_X) px = LCD_W - 1 - px;
  if (TOUCH_FLIP_Y) py = LCD_H - 1 - py;
  gr::logical(px, py, x, y);
}

// Touch task (core 1, above loop priority): polls every TOUCH_POLL_MS even while a frame is
// being drawn, so a quick swipe is seen as a swipe and not as a tap. Gestures go to touchQ.
// It also runs when the chip didn't answer at boot (it can sit in a low-power state after the
// ESP resets, and only wakes when touched): it keeps trying and sets it up once it answers.
static void touchTask(void *) {
  bool pressing = false, longFired = false, holdFired = false;
  uint32_t t0 = 0, lastSeen = 0;
  int sx = 0, sy = 0, lx = 0, ly = 0;
  uint32_t errSince = 0;
  bool lost = !hasTouch;
  for (;;) {
    uint16_t rx = 0, ry = 0;
    int r = touchRead(rx, ry);
    uint32_t now = millis();
    if (r < 0) {                                    // bus error: keep the current state
      if (!errSince) errSince = now;
      if (!lost && now - errSince > 1000) { lost = true; Serial.println(F("[touch] chip stopped answering, retrying")); }
      vTaskDelay(pdMS_TO_TICKS(lost ? 250 : TOUCH_POLL_MS));
      continue;
    }
    if (lost) { lost = false; hasTouch = true; touchInit(); Serial.println(F("[touch] chip answering again, set up")); }
    errSince = 0;
    if (r == 1) lastSeen = now;
    // only treat the finger as lifted after TOUCH_RELEASE_MS with no touch reported
    bool down = r == 1 || (pressing && now - lastSeen < TOUCH_RELEASE_MS);
    int x = 0, y = 0;
    if (r == 1) mapTouch(rx, ry, x, y);
    if (down && !pressing) {
      pressing = true; longFired = false; holdFired = false; t0 = now; sx = lx = x; sy = ly = y;
      if (TOUCH_DEBUG) Serial.printf("[touch] down raw %u,%u -> screen %d,%d\n", rx, ry, x, y);
    }
    if (r == 1) { lx = x; ly = y; }
    int dx = lx - sx, dy = ly - sy;
    bool moved = abs(dx) > 20 || abs(dy) > 20;
    if (down && !holdFired && !moved && now - t0 >= RESET_HOLD_MS) {     // held still: reset (fires while held, so the toast says when to let go)
      holdFired = true;
      TouchEv e{5, (int16_t)sx, (int16_t)sy}; xQueueSend(touchQ, &e, 0);
    }
    if (down && !longFired && !moved && now - t0 >= LONGPRESS_MS) {
      longFired = true;
      TouchEv e{2, (int16_t)sx, (int16_t)sy}; xQueueSend(touchQ, &e, 0);
    }
    if (!down && pressing) {
      pressing = false;
      if (TOUCH_DEBUG) Serial.printf("[touch] up after %lu ms, moved %d,%d\n", (unsigned long)(now - t0), dx, dy);
      if (!longFired && !holdFired) {
        TouchEv e{0, (int16_t)sx, (int16_t)sy};
        if (abs(dx) > SWIPE_PX && abs(dx) >= abs(dy)) e.type = dx < 0 ? 3 : 4;        // swipe left = next
        else if (abs(dy) > SWIPE_PX && abs(dy) > abs(dx)) e.type = dy < 0 ? 3 : 4;   // swipe up = next
        else if (!moved) e.type = 1;
        if (e.type) xQueueSend(touchQ, &e, 0);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(TOUCH_POLL_MS));
  }
}

// ---------------------------------------------------------------------
//  Sensor maths
// ---------------------------------------------------------------------
static float supplyVolts(float vSupPin) {
  float vs = vSupPin * SUP_DIV_RATIO;
  return (vs < SUPPLY_MIN_V || vs > SUPPLY_MAX_V) ? SUPPLY_NOMINAL : vs;   // A1 not wired / implausible
}

// "Zero quantity" for one sample: AWE = compensated signal volts, MPX = kPa abs
static float zeroQty(float vSensPin, float vSupPin) {
  float vsig = vSensPin * SIG_DIV_RATIO;
  float vs   = supplyVolts(vSupPin);
#if SENSOR_TYPE == SENSOR_AWE
  #if AWE_RATIOMETRIC
  #if !SUPPLY_SENSE_ENABLED
    #warning "AWE_RATIOMETRIC without Divider B uses SUPPLY_NOMINAL (no correction) until OBD voltage is wired in"
  #endif
  vsig *= AWE_SUPPLY_REF_V / vs;
  #else
  (void)vs;
  #endif
  return vsig;
#else
  // MPX4250AP: Vout = Vs * (K1 * P - K0)  ->  P = (Vout / Vs + K0) / K1
  float kpa = (vsig / vs + MPX_K0) / MPX_K1;
  return kpa * CAL_GAIN + CAL_OFFSET_KPA;
#endif
}

// psi relative to atmosphere (negative = vacuum)
static inline float psiFromQty(float q) {
#if SENSOR_TYPE == SENSOR_AWE
  return (q - gZero) / S.vPerPsi;
#else
  return (q - gZero) * KPA_TO_PSI;
#endif
}

static bool readQtyBounded(float &q) {
  float vSens, vSup = SUPPLY_NOMINAL / SUP_DIV_RATIO;
  if (!adsReadBounded(ADS_CH_SENSOR, vSens)) return false;
#if SUPPLY_SENSE_ENABLED
  if (!adsReadBounded(ADS_CH_SUPPLY, vSup)) return false;
#endif
  q = zeroQty(vSens, vSup);
  return true;
}

// Average a burst of samples; accept as the atmosphere zero only if it is plausible and steady
// (a reading taken while cranking or with the engine running moves, and sits low)
#ifndef ZERO_MAX_SPREAD_V
#define ZERO_MAX_SPREAD_V 1e9f
#endif
// wide: the ECU says the engine is stopped, so a steady reading is the atmosphere even when it's
// lower than the usual window (altitude): accepted down to ZERO_WIDE_MIN, but only within
// ZERO_BARO_TOL_V of `expect` (what the ECU's barometer predicts), so a sender or wire that has
// started reading low can't be taken as the atmosphere.
static void captureZero(bool fromBoot, bool wide = false, float expect = 0) {
  if (!hasADS) return;
  const float zmin = wide ? (float)ZERO_WIDE_MIN : (float)ZERO_MIN;
  float sum = 0, lo = 1e9f, hi = -1e9f;
  int got = 0;
  for (int i = 0; i < ZERO_SAMPLES; i++) {
    float v;
    if (!readQtyBounded(v)) continue;
    sum += v; lo = min(lo, v); hi = max(hi, v); got++;
  }
  if (got < ZERO_SAMPLES / 2) {
    Serial.printf("[zero] ADS1115 not answering (%d/%d readings) -> using saved %.3f\n", got, ZERO_SAMPLES, (float)gZero);
    gZeroFromBoot = false;
    if (!fromBoot) gZeroDone = true;
    return;
  }
  float q = sum / got;
#if SENSOR_TYPE == SENSOR_AWE
  bool steady = hi - lo <= ZERO_MAX_SPREAD_V;
#else
  bool steady = true;
#endif
#ifdef ZERO_BARO_TOL_V
  bool agrees = !wide || q >= ZERO_MIN || fabsf(q - expect) <= ZERO_BARO_TOL_V;   // the usual window needs no check
#else
  bool agrees = true;
#endif
  if (q >= zmin && q <= ZERO_MAX && steady && agrees) {
    gZero = q;
    gZeroFromBoot = true;
    gZeroAt = millis(); gZeroGen = gZeroGen + 1;
    if (!fromBoot) gZeroDone = true;
    prefs.putFloat(ZERO_KEY, q);
    Serial.printf("[zero] captured %.3f %s (%s), saved\n", q, ZERO_UNIT, fromBoot ? "key-on" : "manual");
  } else {
    // keep the zero already in use (the saved one, checked at boot, or the default)
    gZeroFromBoot = false;
    if (!fromBoot) gZeroDone = true;
    if (!agrees && steady && q >= zmin && q <= ZERO_MAX)
      Serial.printf("[zero] reading %.3f %s is %.3f from what the ECU's barometer predicts (%.3f): sender or wiring? -> using saved %.3f\n",
                    q, ZERO_UNIT, q - expect, expect, (float)gZero);
    else
      Serial.printf("[zero] reading %.3f %s (spread %.3f) not a steady %.2f-%.2f (engine running or cranking?) -> using saved %.3f\n",
                    q, ZERO_UNIT, hi - lo, zmin, (float)ZERO_MAX, (float)gZero);
  }
}

// ---------------------------------------------------------------------
//  Sensor task (core 0)
//  ADS1115 in continuous mode at 860 SPS on A0, read every 2 ms (~500 Hz, one short I2C read
//  each). Median of MEDIAN_SAMPLES drops single bad readings from loose contacts, then an EMA
//  for the needle and a slower one for the digits. The peak follows the needle value.
// ---------------------------------------------------------------------
static void adsStartContinuous() {
#if !SUPPLY_SENSE_ENABLED
  if (hasADS) ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, /*continuous=*/true);
#endif
}

static float median(float *v, int n) {
  float t[9];
  for (int i = 0; i < n; i++) t[i] = v[i];
  for (int i = 1; i < n; i++) for (int j = i; j > 0 && t[j] < t[j - 1]; j--) { float x = t[j]; t[j] = t[j - 1]; t[j - 1] = x; }
  return t[n / 2];
}

static void sensorTask(void *) {
  uint32_t n = 0, badSince = 0;
  float vSup = SUPPLY_NOMINAL / SUP_DIV_RATIO;   // nominal until the first A1 read
  float hist[9]; int hn = 0;
  const int MED = constrain(MEDIAN_SAMPLES, 1, 9);
  adsStartContinuous();
  for (;;) {
    if (gReqRezero) { gReqRezero = false; bool wide = gZeroWide; gZeroWide = false; if (!gDemo) { captureZero(false, wide, gZeroExpect); adsStartContinuous(); } }
    if (gReqPeakReset) { gReqPeakReset = false; gPeakPsi = gPsiFilt; }
    if (gWinReset) { gWinReset = false; gWinMin = 99; gWinMax = -99; }

    float psi;
    // OBD manifold pressure as boost: the fallback when there's no sender reading
    bool obdMap = obd::fresh(obd::F_MAP, 1500);
    float obdPsi = obdMap ? (obd::val[obd::F_MAP] - (obd::fresh(obd::F_BARO, obd::ageFor(obd::F_BARO)) ? obd::val[obd::F_BARO] : 101.3f)) * 0.145038f : 0;
    bool useObd = false;
    if (gDemo || (!hasADS && !obdMap)) {
      gr::Data dd; gr::demo(millis() / 1000.0f, dd);
      psi = dd.boost;
      gVsensPin = 0; gVsupPin = 0;
      vTaskDelay(pdMS_TO_TICKS(2));
    } else if (!hasADS) {                          // no ADS1115: OBD only
      psi = obdPsi; useObd = true;
      gVsensPin = 0; gVsupPin = 0;
      vTaskDelay(pdMS_TO_TICKS(2));
    } else {
#if SUPPLY_SENSE_ENABLED
      float vSens = 0;
      adsReadBounded(ADS_CH_SENSOR, vSens);
      if ((n & 7) == 0) adsReadBounded(ADS_CH_SUPPLY, vSup);   // supply 1 in 8
#else
      float vSens = ads.computeVolts(ads.getLastConversionResults());
#endif
      float vsig = vSens * SIG_DIV_RATIO;
      if (vsig < gWinMin) gWinMin = vsig;
      if (vsig > gWinMax) gWinMax = vsig;
      // wiring check
      bool bad = vsig < SENSOR_MIN_V || vsig > SENSOR_MAX_V;
      if (!bad) badSince = 0; else if (!badSince) badSince = millis();
      gSensorFault = badSince && millis() - badSince > 200;
      // median against single glitches
      if (hn < MED) hist[hn++] = vSens; else { for (int i = 1; i < MED; i++) hist[i - 1] = hist[i]; hist[MED - 1] = vSens; }
      float vMed = median(hist, hn);
      gVsensPin = vMed; gVsupPin = vSup;
      // The idle trim only while the engine runs. It's learned at idle against the ECU; with the engine
      // off the key-on zero already makes the sender read 0, and a trim there showed about +0.3 psi on a
      // stopped engine. It only switches when the ECU says the engine started or stopped, so an OBD
      // dropout while driving doesn't step the needle.
      static bool trimOn = false;
      if (obd::fresh(obd::F_RPM, 1500)) trimOn = obd::val[obd::F_RPM] > 400;
      psi = psiFromQty(zeroQty(vMed, vSup)) - ((S.obdTrim && trimOn) ? gPsiTrim : 0.0f);
      if (gSensorFault && obdMap) { psi = obdPsi; useObd = true; }   // sender unplugged / faulty: OBD stands in
#if SUPPLY_SENSE_ENABLED
      vTaskDelay(1);
#else
      vTaskDelay(pdMS_TO_TICKS(2));
#endif
    }
    // ---- G-meter: 100 Hz
    static uint32_t lastImu = 0;
    if (hasIMU && micros() - lastImu >= 10000) {
      uint32_t nowUs = micros();
      float dt = lastImu ? (nowUs - lastImu) / 1e6f : 0.01f;
      lastImu = nowUs;
      if (dt > 0.05f) dt = 0.05f;
      if (gReqGLevel)  { gReqGLevel = false; gimu.relevel(); }
      if (gReqGForget) { gReqGForget = false; gimu.forget(); }
      if (gReqGPeak)   { gReqGPeak = false; gimu.resetPeaks(); }
      imu::V3 a, w;
      if (imu::read(a, w)) {
        bool spd = obd::fresh(obd::F_KMH, 1500);
        // time in s from millis(): micros() wraps after 71 minutes
        gimu.step(a, w, dt, millis() / 1000.0f, spd ? obd::val[obd::F_KMH] : -1.0f, obd::valAt[obd::F_KMH] / 1000.0f);
        if (gimu.dirty || gimu.events) {               // the copy first, so the log line sees it
          portENTER_CRITICAL(&imuMux); imuCalOut = gimu.cal; portEXIT_CRITICAL(&imuMux);
        }
        if (gimu.dirty) { gimu.dirty = false; imuCalPending = true; }
        if (gimu.events) { gimu.events = 0; imuEvent = gimu.lastEvent; }
      } else imuErrors = imuErrors + 1;
    }

    // ---- performance timers: 100 Hz
    static uint32_t lastPerf = 0;
    if (!gDemo && micros() - lastPerf >= 10000) {
      lastPerf = micros();
      using namespace obd;
      float lonG = (hasIMU && gimu.state() == imu::READY) ? gimu.lonRaw : perf::NOV;
      float kmh = fresh(F_KMH, 1500) ? (float)val[F_KMH] : perf::NOV;
      float ped = fresh(F_PEDAL, ageFor(F_PEDAL)) ? (float)val[F_PEDAL] : perf::NOV;
      float bst = boostValid() ? (float)gPsiFilt : perf::NOV;
      portENTER_CRITICAL(&perfMux);
      gPerf.step(millis() / 1000.0f, lonG, kmh, valAt[F_KMH] / 1000.0f, ped, bst);
      portEXIT_CRITICAL(&perfMux);
    }

    if (useObd != gBoostObd) gBoostObd = useObd;
    // manifold pressure as often as possible the moment the sender can't be used, not only once a fresh
    // OBD reading has taken over (at cruise it comes every ~1.7 s, on boost ~3.3 s: the stand-in would wait)
    obd::mapFast = useObd || gSensorFault || !hasADS;
    gPsiRaw  = psi;
    // OBD readings come about 5 times a second: a slower needle (~170 ms) glides between them
    gPsiFilt = gPsiFilt + (useObd ? 0.012f : NEEDLE_ALPHA) * (psi - gPsiFilt);
    gPsiText = gPsiText + TEXT_ALPHA * (psi - gPsiText);
    obd::senderPsi = (useObd || gDemo) ? -99.0f : (float)gPsiFilt;   // the OBD task goes onto its boost schedule from this
    if (gPsiFilt > gPeakPsi) gPeakPsi = gPsiFilt;
    gSamples = gSamples + 1;
    n++;
  }
}

// ---------------------------------------------------------------------
//  Drawing
// ---------------------------------------------------------------------
// A good moment for a flash write (settings, learned values): each one pauses both cores briefly, so
// not on boost, not on the throttle and not during a timed run.
static inline bool flashCalm() {
  using namespace obd;
  return !(boostValid() && gPsiFilt > 1.0f) && !(fresh(F_PEDAL, ageFor(F_PEDAL)) && val[F_PEDAL] > 35) && gPerf.shownState() != 2;
}
// The ECU says the engine isn't turning (ignition on). Not known without OBD.
static inline bool engineStopped() { return obd::fresh(obd::F_RPM, 1500) && obd::val[obd::F_RPM] < 50; }
// The zero the ECU's barometer predicts: the last zero taken in the usual window, moved by how far the
// barometer has fallen since (the sender's slope). NOV = can't say (no fresh barometer, nothing learned
// yet, or a sender that reads the barometer itself): then only the usual window is accepted.
static float zeroExpect() {
#if SENSOR_TYPE == SENSOR_AWE
  using namespace obd;
  if (gZeroRefKpa < 50 || !fresh(F_BARO, ageFor(F_BARO))) return gr::NOV;
  return gZeroRefV + (val[F_BARO] - gZeroRefKpa) / 6.894757f * S.vPerPsi;
#else
  return gr::NOV;
#endif
}
// A re-zero: with the engine known to be stopped and the barometer known, it may accept a lower
// reading (altitude) that agrees with the barometer.
static void requestRezero() {
  float e = engineStopped() ? zeroExpect() : gr::NOV;
  gZeroExpect = e; gZeroWide = gr::has(e); gReqRezero = true;
}

static void toast(const char *msg, uint32_t ms = 1200) {
  strlcpy(toastMsg, msg, sizeof(toastMsg));
  toastAt = millis(); toastMs = ms;
}

// byUser: a swipe / key (remembered for next start). Otherwise the pod moved it (a warning or
// the pull summary) and it goes back afterwards.
static void setPage(int p, bool byUser = true) {
  page = (p + gr::P_COUNT) % gr::P_COUNT;
  pageAt = millis();
  autoPaged = !byUser;
  if (byUser) { prefs.putInt("page", page); alertFrom = -1; pullFrom = -1; pullUntil = 0; launchFrom = -1; }
  Serial.printf("[ui] page %d%s\n", page, byUser ? "" : " (auto)");
}

static volatile uint32_t peakGen = 0;           // bumped on a peak reset (the telltales start over too)
static void resetPeak() {
  gReqPeakReset = true;
  peakGen = peakGen + 1;
  maxIat = 0; maxRpm = 0;
  for (float &g : gearPk) g = 0;
  toast("PEAK RESET");
}

// Injector duty from the pulse width and the RPM when it was read: one injection per cylinder
// every 2 turns. (The pulse width comes every 2-4 s; with the RPM of now, a cruise pulse width and
// a 6000 rpm pull would have been mixed.)
static float idcNow() {
  using namespace obd;
  if (!fresh(F_INJ, ageFor(F_INJ)) || injRpm <= 0) return gr::NOV;
  return val[F_INJ] * injRpm / 1200.0f;
}

// ---- performance runs: clock, the runs list, and collecting results from the timer
static uint8_t fromBcd(uint8_t b) { return (b >> 4) * 10 + (b & 15); }
static uint8_t toBcd(int v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }
// PCF85063: registers 0x04-0x0A = seconds (bit 7: the clock stopped since it was set), minutes,
// hours, day, weekday, month, year. Kept in UTC.
static bool rtcRead(time_t &out) {
  Wire.beginTransmission(ADDR_RTC); Wire.write(0x04);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)ADDR_RTC, 7) != 7) return false;
  uint8_t r[7]; for (uint8_t &b : r) b = Wire.read();
  if (r[0] & 0x80) return false;                 // lost power since it was set
  struct tm tm = {};
  tm.tm_sec = fromBcd(r[0] & 0x7F); tm.tm_min = fromBcd(r[1] & 0x7F); tm.tm_hour = fromBcd(r[2] & 0x3F);
  tm.tm_mday = fromBcd(r[3] & 0x3F); tm.tm_mon = fromBcd(r[5] & 0x1F) - 1; tm.tm_year = fromBcd(r[6]) + 100;
  if (tm.tm_year < 126 || tm.tm_mon < 0 || tm.tm_mon > 11 || tm.tm_mday < 1) return false;
  out = mktime(&tm);                             // TZ is UTC on the ESP32 unless set
  return out > 0;
}
static void rtcWrite(time_t t) {
  struct tm tm; gmtime_r(&t, &tm);
  Wire.beginTransmission(ADDR_RTC); Wire.write(0x04);
  Wire.write(toBcd(tm.tm_sec)); Wire.write(toBcd(tm.tm_min)); Wire.write(toBcd(tm.tm_hour));
  Wire.write(toBcd(tm.tm_mday)); Wire.write(toBcd(tm.tm_wday)); Wire.write(toBcd(tm.tm_mon + 1)); Wire.write(toBcd(tm.tm_year - 100));
  Wire.endTransmission();
}
// The phone's time (each time the web page opens). Runs from this power-on that had no date
// yet are dated back from now.
volatile bool tzCheckReq = false;                // the clock was just set: compare with the car's clock
static void setClock(uint32_t epoch, uint8_t src = 2) {
  bool was = clockOk;
  if (src != 3) tzCheckReq = true;
  if (src != 3) clockSrc = src;
  if (was && labs((long)time(nullptr) - (long)epoch) < 3) return;   // already right
  clockSrc = src;
  struct timeval tv = {(time_t)epoch, 0};
  settimeofday(&tv, nullptr);
  clockOk = true;
  if (hasRTC) rtcWrite(epoch);
  int fixed = 0;
  uint32_t ms = millis();
  portENTER_CRITICAL(&runsMux);
  auto fix = [&](RunRec &x) { if (x.flags & RF_UNDATED) { x.when = epoch - (ms - x.ms) / 1000; x.flags &= ~RF_UNDATED; fixed++; } };
  for (RunRec &x : gRuns.r) fix(x);
  for (auto &row : gRuns.best) for (RunRec &x : row) fix(x);
  portEXIT_CRITICAL(&runsMux);
  if (fixed) { runsDirty = true; runsVer = runsVer + 1; }
  healthDateReq = true;
  if (!was) Serial.printf("[clock] set from the %s (%u)%s\n", src == 3 ? "car" : "phone", (unsigned)epoch, fixed ? ", earlier runs dated" : "");
}
static void runsClear() {
  portENTER_CRITICAL(&runsMux);
  memset(&gRuns, 0, sizeof(gRuns)); gRuns.magic = RUNS_MAGIC;
  portEXIT_CRITICAL(&runsMux);
  portENTER_CRITICAL(&perfMux); gPerf.resetBest(); portEXIT_CRITICAL(&perfMux);
  runsDirty = true; runsVer = runsVer + 1;
  Serial.println(F("[run] runs list cleared"));
}
static void runAdd(const perf::Result &r, uint32_t now) {
  RunRec x = {};
  x.kind = r.kind; x.t = r.t; x.mph = r.mph; x.psi = r.psi;
  x.ambF = obd::fresh(obd::F_AMB, obd::ageFor(obd::F_AMB)) ? (float)obd::val[obd::F_AMB] : gr::NOV;
  x.ms = now; x.when = clockOk ? (uint32_t)time(nullptr) : 0;
  x.flags = (r.obdOnly ? RF_OBD : 0) | (r.rollout ? RF_ROLLOUT : 0) | (clockOk ? 0 : RF_UNDATED);
  const char *ln = strstr(logx::curName, "log_");
  x.logNo = (logx::open && ln) ? (uint16_t)atoi(ln + 4) : 0;
  bool best;
  portENTER_CRITICAL(&runsMux);
  RunRec &b = gRuns.best[r.rollout ? 1 : 0][x.kind];
  best = b.t <= 0 || x.t < b.t;
  if (best) b = x;
  memmove(&gRuns.r[1], &gRuns.r[0], sizeof(RunRec) * 29);
  gRuns.r[0] = x;
  if (gRuns.n < 30) gRuns.n++;
  portEXIT_CRITICAL(&runsMux);
  runsDirty = true; runsVer = runsVer + 1;
  char m[32];
  if (x.kind == perf::KQM) snprintf(m, sizeof(m), "1/4 %.2f S @ %.0f%s", x.t, x.mph, best ? " BEST" : "");
  else snprintf(m, sizeof(m), "%s  %.2f S%s", perf::kindName(x.kind), x.t, best ? "  BEST" : "");
  toast(m, 4000);
  Serial.printf("[run] %s %.2f s%s%s, %.0f mph at the end, peak %.1f psi%s%s\n", perf::kindName(x.kind), x.t, best ? " (best)" : "",
                r.rollout ? " with rollout" : "", x.mph, gr::has(x.psi) ? x.psi : 0.0f, r.obdOnly ? ", OBD speed only" : "",
                x.logNo ? (String(", in log_") + String(x.logNo)).c_str() : "");
}
// the health file, written while nothing is happening (a flash write pauses the screen briefly)
static void healthUi(uint32_t now) {
  static uint32_t calmSince = 0;
  bool calm = !gWatch.inPull && gPerf.shownState() != 2 && !(boostValid() && gPsiFilt > 2);
  if (!calm) { calmSince = 0; return; }
  if (!calmSince) calmSince = now;
  if (now - calmSince < 3000 || !logx::flash.ok) return;
  uint8_t rq = healthReq;
  if (rq) {
    healthReq = 0;
    if (rq == 1) { health::setBaseline(HEALTH_PATH); Serial.println(F("[health] new baseline from the next pull")); }
    if (rq == 2) { remove(HEALTH_PATH); remove(HEALTH_TMP); healthPendN = 0; Serial.println(F("[health] history cleared")); }
    healthVer = healthVer + 1;
  }
  if (healthDateReq && clockOk) {
    healthDateReq = false;
    int n = health::dateBack(HEALTH_PATH, bootNo, (uint32_t)time(nullptr), now);
    if (n) { Serial.printf("[health] %d pull%s dated\n", n, n == 1 ? "" : "s"); healthVer = healthVer + 1; }
  }
  for (int i = 0; i < healthPendN; i++) {
    health::Rec &r = healthPend[i];
    if ((r.flags & health::F_UNDATED) && clockOk) { r.when = (uint32_t)time(nullptr) - (now - r.ms) / 1000; r.flags &= ~health::F_UNDATED; }
    long n = health::append(HEALTH_PATH, HEALTH_TMP, r);
    if (n < 0) Serial.println(F("[health] couldn't write the health file"));
    else Serial.printf("[health] pull %lu saved (%ld kept)\n", (unsigned long)r.seq, n);
  }
  if (healthPendN) { healthPendN = 0; healthVer = healthVer + 1; }
  // the catalyst trend: a new ECU result (dated: waits for the clock)
  if (catPend && clockOk) {
    catPend = false;
    catNew.when = (uint32_t)time(nullptr);
    if (health::catAppend(CATS_PATH, catNew)) {
      catSaved = catNew; catSavedOk = true; healthVer = healthVer + 1;
      Serial.printf("[cats] new catalyst result saved: bank 1 %04X of %04X..%04X, bank 2 %04X of %04X..%04X\n",
                    catNew.val[0], catNew.min[0], catNew.max[0], catNew.val[1], catNew.min[1], catNew.max[1]);
    }
  }
}
// called when the OBD task has a new catalyst reading: saved if the result changed
static void catCheck(uint32_t now) {
  static uint16_t seen = 0;
  if (obd::catN == seen) return;
  seen = obd::catN;
  health::CatRec r; memset(&r, 0, sizeof(r));
  portENTER_CRITICAL(&obd::selfMux);
  for (int b = 0; b < 2; b++) if (obd::catHave[b]) {
    r.flags |= b ? health::CF_B2 : health::CF_B1;
    r.val[b] = obd::catRes[b].val; r.min[b] = obd::catRes[b].min; r.max[b] = obd::catRes[b].max; r.tid[b] = obd::catRes[b].tid;
  }
  if (obd::monKnown) {
    r.flags |= health::CF_MONKNOWN;
    if (obd::monDone(obd::monSince, 0)) r.flags |= health::CF_DONE_SINCE;
    if (obd::monDone(obd::monDrive, 0)) r.flags |= health::CF_DONE_DRIVE;
  }
  portEXIT_CRITICAL(&obd::selfMux);
  if (!(r.flags & 3)) return;
  r.odoKm = (obd::odoAt && now - obd::odoAt < 600000) ? obd::odoKm : 0;
  if (catSavedOk && health::sameResult(r, catSaved)) return;    // the same result as last time
  catNew = r; catPend = true;
}

static void perfUi(uint32_t now) {
  perf::Result rr[6];
  int n;
  portENTER_CRITICAL(&perfMux);
  gPerf.rollout = S.perfRollout;
  gPerf.lag = constrain(S.obdLagMs, 0.0f, 500.0f) / 1000.0f;
  n = gPerf.take(rr, 6);
  portEXIT_CRITICAL(&perfMux);
  for (int i = 0; i < n; i++) runAdd(rr[i], now);
  // saved once things are calm (a flash write pauses the screen for a moment)
  static uint32_t calmSince = 0;
  bool calm = gPerf.shownState() != 2 && !(hasADS && gPsiFilt > 2);
  if (!calm) calmSince = 0; else if (!calmSince) calmSince = now;
  if (runsDirty && calmSince && now - calmSince > 3000) {
    runsDirty = false;
    RunStore copy;
    portENTER_CRITICAL(&runsMux); copy = gRuns; portEXIT_CRITICAL(&runsMux);
    prefs.putBytes("runs", &copy, sizeof(copy));
  }
}

// ---- the watcher, 20 times a second: latest readings in, then warnings and the pull summary
static void watchStep(uint32_t now) {
  using namespace obd;
  if (isDemo()) return;
  auto rd = [](int f) { return fresh(f, ageFor(f)) ? (float)val[f] : gr::NOV; };
  watch::In in;
  in.t = now / 1000.0f;
  in.psi = boostValid() ? (float)gPsiFilt : gr::NOV;
  in.rpm = rd(F_RPM); in.kmh = rd(F_KMH); in.pedal = rd(F_PEDAL); in.thr = rd(F_THR);
  in.gear = fresh(F_KMH, 1500) ? gearEst.gear : 0;
  in.iat = cacFromIat ? rd(F_IAT) : rd(F_CAC);
  in.timing = rd(F_TIM); in.knock = rd(F_KNOCK); in.knockCyl = knockCyl;
  in.hpfp = rd(F_HPFP); in.hpfpSet = rd(F_HPFPSET); in.lpfp = rd(F_LPFP); in.idc = idcNow();
  in.afr1 = rd(F_AFR1); in.afr2 = rd(F_AFR2); in.afrCmd = rd(F_AFRCMD);
  in.afrAt = valAt[F_AFR1] ? valAt[F_AFR1] / 1000.0f : gr::NOV; in.lpfpAt = valAt[F_LPFP] ? valAt[F_LPFP] / 1000.0f : gr::NOV;
  in.cat1 = rd(F_CAT1); in.cat2 = rd(F_CAT2);
  in.clt = rd(F_CLT); in.bat = rd(F_BAT); in.bypass = rd(F_BYPASS);
  in.oil = fresh(F_OIL, ageFor(F_OIL)) ? val[F_OIL] + S.oilTrimF : gr::NOV;
  in.lon = (hasIMU && gimu.state() == imu::READY) ? gimu.lon : gr::NOV;
  float baro = fresh(F_BARO, ageFor(F_BARO)) ? val[F_BARO] : 101.3f;
  in.obdPsi = fresh(F_MAP, ageFor(F_MAP)) ? (val[F_MAP] - baro) * 0.145038f : gr::NOV;
  in.tgtPsi = fresh(F_MAPCMD, ageFor(F_MAPCMD)) ? (val[F_MAPCMD] - baro) * 0.145038f : gr::NOV;
  in.redPsi = S.redPsi;
  bool running = fresh(F_RPM, 1500) && val[F_RPM] > 400;
  if (!running) engineSince = 0; else if (!engineSince) engineSince = now;
  in.runSec = engineSince ? (now - engineSince) / 1000.0f + 0.01f : 0;
  // warm-up cue: until the oil reaches WARM_OIL_F (back to cold only well below it, e.g. a cold start)
  {
    static bool warm = false, sawCold = false;
    if (running && gr::has(in.oil)) {
      if (!warm && in.oil >= WARM_OIL_F) {
        warm = true;
        if (sawCold) { toast("OIL WARM - READY", 3000); Serial.printf("[warm] oil %.0f F: warmed up\n", in.oil); }
      } else if (warm && in.oil < WARM_OIL_F - 25) warm = false;
      if (!warm) sawCold = true;
      gColdOilF = warm ? gr::NOV : in.oil;
    } else gColdOilF = gr::NOV;
  }
  int pullsBefore = gWatch.pullsDone;
  gWatch.step(in);
  if (gWatch.curveDirty && flashCalm()) {          // the usual-boost curve, saved after each pull (once off the throttle)
    gWatch.curveDirty = false;
    prefs.putBytes("bcurve", &gWatch.curve, sizeof(gWatch.curve));
  }
  if (gWatch.pullsDone != pullsBefore) {
    const gr::PullSum &p = gWatch.last;
    Serial.printf("[pull] %.1f s, gears %s, peak %.1f psi at %.0f rpm, vs usual %s%.1f, knock %.1f, leanest AFR %.1f, hottest cat %.0f F%s\n",
                  p.durS, p.gears, p.peakPsi, p.peakRpm, gr::has(p.vsNormal) ? "" : "(learning) ", gr::has(p.vsNormal) ? p.vsNormal : 0.0f,
                  gr::has(p.knock) ? p.knock : 0.0f, gr::has(p.afrMax) ? p.afrMax : 0.0f, gr::has(p.catMax) ? p.catMax : 0.0f, p.slip ? ", BOOST FELL (belt slip?)" : "");
    pullHist[pullHistN % 10] = p; pullHistN++;
    if (healthPendN < 8) {                         // for the health trends (written once it's calm)
      uint32_t odo = (obd::odoAt && now - obd::odoAt < 600000) ? obd::odoKm : 0;
      float amb = fresh(F_AMB, ageFor(F_AMB)) ? (float)val[F_AMB] : gr::NOV;
      bool fromObd = !(hasADS && !gSensorFault);
      healthPend[healthPendN++] = health::make(p, amb, in.oil, in.clt, odo, clockOk ? (uint32_t)time(nullptr) : 0, now, bootNo, fromObd);
    }
    if (S.autoPull && alertOn < 0 && launchFrom < 0) {   // the summary for a few seconds, then back (not after a launch: Perf stays)
      if (page != gr::P_PULL) { pullFrom = page; setPage(gr::P_PULL, false); }
      pullUntil = now + 8000;
    }
  }
  // back from the summary when its time is up, or straight away when the next pull starts
  if (pullUntil && (now > pullUntil || gWatch.inPull)) {
    pullUntil = 0;
    if (pullFrom >= 0 && page == gr::P_PULL && autoPaged && alertOn < 0) setPage(pullFrom, false);
    pullFrom = -1;
  }

  // Launch: held at a standstill with the revs up and steady (launch control or brake boosting)
  // for half a second -> the Perf page. Back to where you were 8 s after the run ends, or if
  // the launch is called off (revs back to idle at a stop for 3 s, or driving off gently).
  {
    static float rMin = 1e9f, rMax = 0; static uint32_t winAt = 0, ranAt = 0, offSince = 0;
    float rpm = in.rpm, kmh = in.kmh;
    bool still = gr::has(kmh) && kmh < 1 && gr::has(rpm);
    // not revving parked (parking brake on), and the brake pedal down when it's known (launch control
    // and brake boosting both need it: revs up at a stop with the foot off the brake is neutral or Park)
    bool held = still && rpm >= 2000 && (!gr::has(in.pedal) || in.pedal >= 60) && obd::parkBrake() != 1 && obd::brakeOn() != 0;
    if (!held) { rMin = 1e9f; rMax = 0; winAt = now; }
    else { rMin = min(rMin, rpm); rMax = max(rMax, rpm); }
    bool steady = held && now - winAt >= 500 && rMax - rMin < 600;      // a blip of the throttle isn't steady
    gStaged = steady;                                                    // the Perf page says STAGED
    if (S.autoLaunch && steady && launchFrom < 0 && alertOn < 0 && page != gr::P_PERF) {
      launchFrom = pullFrom >= 0 ? pullFrom : page;
      pullUntil = 0; pullFrom = -1;
      setPage(gr::P_PERF, false);
      ranAt = 0; offSince = 0;
      Serial.printf("[run] launch: %.0f rpm at a standstill -> Perf page\n", rpm);
    }
    if (launchFrom >= 0) {
      if (gPerf.shownState() == 2) ranAt = now;                         // the run is on
      bool calledOff = !ranAt && ((still && rpm < 1500) || (gr::has(kmh) && kmh > 15 && gPerf.shownState() != 2));
      if (calledOff) { if (!offSince) offSince = now; } else offSince = 0;
      bool done = (ranAt && gPerf.shownState() != 2 && now - ranAt > 8000) || (offSince && now - offSince > 3000);
      if (done) {
        if (page == gr::P_PERF && autoPaged && alertOn < 0) setPage(launchFrom, false);
        launchFrom = -1;
      }
    }
  }

  // warnings: the worst one that hasn't been tapped away (or has got worse since)
  int top = -1;
  for (int i = 0; i < watch::A_N; i++) {
    const watch::Alarm &a = gWatch.a[i];
    if (!a.on) { alertAck[i] = false; continue; }
    if (alertAck[i] && a.sev <= alertAckSev[i]) continue;
    if (top < 0 || a.sev > gWatch.a[top].sev || (a.sev == gWatch.a[top].sev && a.onAt > gWatch.a[top].onAt)) top = i;
  }
  if (top != alertOn) {
    if (top >= 0) {
      const watch::Alarm &a = gWatch.a[top];
      Serial.printf("[warn] %s (%s)\n", a.text, a.sev >= 2 ? "red" : "amber");
      if (S.autoAlert && page != a.page) {
        if (alertFrom < 0) alertFrom = launchFrom >= 0 ? launchFrom : (pullFrom >= 0 ? pullFrom : page);
        pullUntil = 0; pullFrom = -1; launchFrom = -1;
        setPage(a.page, false);
      }
    } else if (alertOn >= 0) {
      Serial.println(F("[warn] all clear"));
      if (alertFrom >= 0 && autoPaged) setPage(alertFrom, false);
      alertFrom = -1;
    }
    alertOn = top;
    if (top >= 0) alertPopAt = now;                // a new warning pops up big first
  }
  // the same warning getting worse (amber -> red) pops up again
  { static int popSev = 0;
    if (alertOn >= 0 && gWatch.a[alertOn].sev > popSev) alertPopAt = now;
    popSev = alertOn >= 0 ? gWatch.a[alertOn].sev : 0; }
}

// Assemble one frame: copy the page's static layer, then draw the moving parts
static void renderFrame(uint32_t now) {
  static char status[40];
  gr::Data d;
  bool demo = isDemo();
  d.boost = gPsiFilt;
  d.boostText = gPsiText;
  d.peak  = gPeakPsi;
  if (demo) {                                    // demo also fills the OBD pages
    gr::Data dd; gr::demo(now / 1000.0f, dd);
    d.obd = true; d.rpm = dd.rpm; d.thr = dd.thr; d.iatF = dd.iatF; d.cltF = dd.cltF;
    d.batV = dd.batV; d.timing = dd.timing; d.oilF = dd.oilF;
    // new pages (G-force, Perf) and the gear chip: demo values until the IMU / gear / timer code lands
    d.gear = dd.gear; d.gLat = dd.gLat; d.gLon = dd.gLon; d.gTrail = dd.gTrail; d.gTrailN = dd.gTrailN;
    d.gPkL = dd.gPkL; d.gPkR = dd.gPkR; d.gPkAcc = dd.gPkAcc; d.gPkBrk = dd.gPkBrk; d.gCal = dd.gCal;
    d.runState = dd.runState; d.runT060 = dd.runT060; d.runT6130 = dd.runT6130; d.runTQm = dd.runTQm; d.runMph = dd.runMph;
    d.t060 = dd.t060; d.t60130 = dd.t60130; d.tQm = dd.tQm; d.qmMph = dd.qmMph;
    d.b060 = dd.b060; d.b60130 = dd.b60130; d.bQm = dd.bQm; d.gearPeak = dd.gearPeak;
  }
  else if (obd::fresh(obd::F_RPM)) {             // live OBD from the Veepeak
    using namespace obd;
    d.obd = true;
    d.rpm = val[F_RPM];
    // the Engine page's bar: the pedal (the throttle plate says little on the 3.0T, and isn't read on its own)
    d.thr = fresh(F_PEDAL, ageFor(F_PEDAL)) ? val[F_PEDAL] : (fresh(F_THR, ageFor(F_THR)) ? val[F_THR] : 0);
    // Each reading only while it's fresh (NOV = "--"): these five used to be copied as they were, so
    // one that stopped arriving stayed on screen at its last value, looking live.
    auto rd = [](int f) { return fresh(f, ageFor(f)) ? (float)val[f] : gr::NOV; };
    d.iatF = rd(cacFromIat ? F_IAT : F_CAC);
    d.cltF = rd(F_CLT); d.batV = rd(F_BAT); d.timing = rd(F_TIM);
    d.oilF = fresh(F_OIL, ageFor(F_OIL)) ? val[F_OIL] + S.oilTrimF : gr::NOV;
    d.gear = fresh(F_KMH, 1500) ? gearEst.gear : 0;     // "-" until speed arrives
    d.trans = transMode();
    d.driveMode = 0;                               // drive select isn't read any more (see the poll list)
    d.gearPeak = gearPk;
    d.afr1 = rd(F_AFR1); d.afr2 = rd(F_AFR2); d.afrCmd = rd(F_AFRCMD);
    d.hpfp = rd(F_HPFP); d.hpfpSet = rd(F_HPFPSET); d.lpfp = rd(F_LPFP);
    d.idc = idcNow();
    d.knock = rd(F_KNOCK); d.knockCyl = knockCyl;
    d.cat1F = rd(F_CAT1); d.cat2F = rd(F_CAT2); d.ambF = rd(F_AMB);
    d.preF = rd(F_PRESC); d.icPump = rd(F_ICPUMP);
    d.clutchF = rd(F_GBT1); d.gbxF = rd(F_GBT2); d.diffF = rd(F_DIFT2);
  }
  // The ECU's requested boost (absolute kPa from 222029 / 0170, minus the barometer). Shown
  // only while it asks for boost (above 1 psi; hidden again under 0.5), gliding briefly to each
  // new reading. The target comes over OBD a couple of times a second, so after a lift-off the last
  // one can still say 17 psi for most of a second while the needle (the sender) is already in
  // vacuum: once the sender drops through 0.5 psi from boost, the marker hides until a target
  // reading from after that moment arrives.
  {
    static float shown = gr::NOV;
    static uint32_t lastAt = 0, liftAt = 0;
    static bool armed = false;
    float dt = lastAt ? min(0.2f, (now - lastAt) / 1000.0f) : 0; lastAt = now;
    if (hasADS && !demo && !gBoostObd) {
      if (gPsiFilt > 2.0f) armed = true;
      else if (armed && gPsiFilt < 0.5f) { armed = false; liftAt = now; }
    }
    bool stale = liftAt && (int32_t)(obd::valAt[obd::F_MAPCMD] - liftAt) < 100;   // asked before the lift
    float tgt = gr::NOV;
    if (!demo && !stale && obd::fresh(obd::F_MAPCMD, obd::ageFor(obd::F_MAPCMD))) {
      float baro = obd::fresh(obd::F_BARO, obd::ageFor(obd::F_BARO)) ? obd::val[obd::F_BARO] : 101.3f;
      tgt = (obd::val[obd::F_MAPCMD] - baro) * 0.145038f;
    } else if (demo) { gr::Data dd; gr::demo(now / 1000.0f, dd); tgt = dd.tgtPsi; }
    bool on = gr::has(tgt) && (gr::has(shown) ? tgt > 0.5f : tgt > 1.0f);
    if (!on) shown = gr::NOV;
    else if (!gr::has(shown)) shown = tgt;
    else shown += (tgt - shown) * (1 - expf(-dt / 0.12f));
    d.tgtPsi = shown;
  }
  d.onBoost = hasADS && !gDemo && gPsiFilt > 2;
  d.pull = &gWatch.last;
  if (alertOn >= 0) {
    const watch::Alarm &a = gWatch.a[alertOn];
    d.alertText = a.text; d.alertRed = a.sev >= 2;
    d.alertTitle = a.title; d.alertValue = a.value; d.alertUnit = a.unit; d.alertNote = a.note;
    uint32_t el = now - alertPopAt;
    if (el < ALERT_POP_MS) { d.alertBig = min(1.0f, (ALERT_POP_MS - el) / 400.0f); d.alertPulse = (el % 900) / 900.0f; }
  }
  if (!demo) d.coldOilF = gColdOilF;
  if (!demo) {                                      // performance timers
    portENTER_CRITICAL(&perfMux);
    d.runState = gPerf.shownState(); d.runT060 = gPerf.liveT060; d.runT6130 = gPerf.liveT6130; d.runTQm = gPerf.liveTQm;
    d.runMph = gPerf.mph; d.runMsg = gPerf.msg; d.runObd = gPerf.obdMode;
    d.t060 = gPerf.t060; d.t60130 = gPerf.t6130; d.tQm = gPerf.tQm; d.qmMph = gPerf.qmMph;
    d.b060 = gPerf.b060; d.b60130 = gPerf.b6130; d.bQm = gPerf.bQm;
    portEXIT_CRITICAL(&perfMux);
    d.epb = obd::parkBrake(); d.brake = obd::brakeOn(); d.staged = gStaged;   // launch readiness (Perf page)
  }
  if (!demo) {                                      // G-meter (the demo fills these itself)
    d.gCal = hasIMU ? gimu.state() : 0;
    d.gLat = gimu.lat; d.gLon = gimu.lon;
    d.gPkL = gimu.pkL; d.gPkR = gimu.pkR; d.gPkAcc = gimu.pkAcc; d.gPkBrk = gimu.pkBrk;
    static uint32_t lastTrail = 0;
    if (now - lastTrail >= 33) {                    // trail point every 33 ms (1.3 s of history)
      lastTrail = now;
      if (gTrailN == 40) { memmove(gTrail, gTrail + 2, sizeof(float) * 78); gTrailN--; }
      gTrail[2 * gTrailN] = d.gLat; gTrail[2 * gTrailN + 1] = d.gLon; gTrailN++;
    }
    d.gTrail = gTrail; d.gTrailN = d.gCal >= 2 ? gTrailN : 0;
  }
  if (d.obd) { if (gr::has(d.iatF)) maxIat = max(maxIat, d.iatF); maxRpm = max(maxRpm, d.rpm); }
  d.maxIat = maxIat; d.maxRpm = maxRpm;
  d.sysOrder = sysOrder;
  d.sysAlert = sysAlertNow = gr::sysAlert(sysAlertState, gcfg, d, (float)now);

  // Peak telltales on the dials: they follow the needle up, stay at the peak for S.peakHoldS
  // seconds after the needle drops well away from it (2 psi / 400 rpm: the end of the pull),
  // then sweep back down to the needle.
  // 0 = held until the peak is reset. The System page's PEAK BOOST / MAX RPM keep the whole session.
  {
    static float tB = -99, tR = 0;
    static uint32_t tBAt = 0, tRAt = 0, lastAt = 0, gen = 0;
    float dt = lastAt ? min(0.2f, (now - lastAt) / 1000.0f) : 0; lastAt = now;
    if (gen != peakGen) { gen = peakGen; tB = d.boost; tR = d.rpm; }
    uint32_t hold = (uint32_t)(S.peakHoldS * 1000);
    if (d.boost >= tB) tB = d.boost;
    if (d.boost >= tB - 2.0f) tBAt = now;                                        // still in the pull: keep holding
    else if (hold && now - tBAt > hold) tB = max(d.boost, tB - 30.0f * dt);       // about 1 s back from 30 psi
    float r = d.obd ? d.rpm : 0;
    if (r >= tR) tR = r;
    if (r >= tR - 400) tRAt = now;
    else if (hold && now - tRAt > hold) tR = max(r, tR - 7000.0f * dt);
    d.peakNeedle = tB; d.rpmNeedle = tR;
  }

  // key-on needle sweep
  if (now < readyAt) {
    float f = gr::sweep(1.4f - (readyAt - now) / 1000.0f);
    d.boost = d.boostText = GAUGE_MIN_PSI + f * (GAUGE_MAX_PSI - GAUGE_MIN_PSI);
    d.rpm = gcfg.tachMax * f;
    d.peak = 0; d.peakNeedle = gr::NOV; d.rpmNeedle = 0; d.tgtPsi = gr::NOV;
  }

  // status line
  status[0] = 0;
  if (demo) { strlcpy(status, hasADS ? "DEMO" : "DEMO \xc2\xb7 NO ADS1115", sizeof(status)); d.statusWarn = true; }
  else if (gBoostObd && (page == gr::P_BOOST || page == gr::P_OVERVIEW)) {
    strlcpy(status, hasADS ? "SENDER FAULT: BOOST FROM OBD" : "BOOST FROM OBD", sizeof(status)); d.statusWarn = hasADS;
  }
  else if (gSensorFault && (page == gr::P_BOOST || page == gr::P_OVERVIEW)) { strlcpy(status, "CHECK SENSOR WIRING", sizeof(status)); d.statusWarn = true; }
  else if (page == gr::P_GFORCE) { if (!hasIMU && !demo) { strlcpy(status, "NO MOTION SENSOR", sizeof(status)); d.statusWarn = true; } }
  else if (page != gr::P_BOOST && !d.obd) {      // say what the OBD link is doing
    snprintf(status, sizeof(status), "OBD %s", obd::stateName(obd::state));
    for (char *c = status; *c; c++) *c = (char)toupper((unsigned char)*c);
    d.statusWarn = true;
  }
  else if (page == gr::P_BOOST && now < readyAt + 5000) {
    char v[12];
    snprintf(status, sizeof(status), "ZERO %s V%s", gr::fmt(v, gZero, 2), gZeroFromBoot ? "" : " (SAVED)");
  }
  d.status = status;

  // toast (1.2 s unless longer, fades over the last 0.3 s) and page dots (2.5 s after a page change)
  uint32_t ta = now - toastAt, pa = now - pageAt;
  d.toast  = toastMsg;
  d.toastA = (toastAt && ta < toastMs) ? min(1.0f, (toastMs - ta) / 300.0f) : 0;
  d.dotsA  = (pageAt && pa < 2500) ? min(1.0f, (2500 - pa) / 400.0f) : 0;

  // Hold each displayed value until it moves by a visible amount, so sensor noise
  // doesn't redraw an otherwise still screen (needle step ~0.2 deg)
  static float sh[12];
  auto hold = [](float &shown, float v, float band) { if (fabsf(v - shown) > band) shown = v; return shown; };
  if (now >= readyAt) {
    d.boost = hold(sh[0], d.boost, 0.03f);   d.boostText = hold(sh[1], d.boostText, 0.04f);
    d.rpm   = hold(sh[2], d.rpm, 15);        d.thr  = hold(sh[3], d.thr, 0.5f);
    d.iatF  = hold(sh[4], d.iatF, 0.3f);     d.cltF = hold(sh[5], d.cltF, 0.3f);
    d.oilF  = hold(sh[6], d.oilF, 0.3f);     d.batV = hold(sh[7], d.batV, 0.02f);
    d.timing = hold(sh[8], d.timing, 0.1f);
  }

  // AMOLED burn-in guard: the whole image walks a 3x3 grid, 1 px per PIXEL_SHIFT_S
  static const int8_t OFF[9][2] = {{0,0},{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1}};
  static int lastT = -1, lastPage = -1, lastShift = -1;
  bool night = S.nightTheme && (lightsOn || forceNight);
  int t = night ? 1 : 0;
  if (t != lastT || page != lastPage || shiftIdx != lastShift) needFull = true;
  lastT = t; lastPage = page; lastShift = shiftIdx;

  // Fingerprint of everything that reaches the screen. Same as last frame = nothing to do.
  static uint32_t lastSig = 0;
  uint32_t sig = 2166136261u;
  auto mix = [&sig](const void *p, size_t n) { const uint8_t *b = (const uint8_t *)p; while (n--) { sig ^= *b++; sig *= 16777619u; } };
  float ov[] = {roundf(d.toastA * 32), roundf(d.dotsA * 32), roundf(d.alertBig * 32), d.alertPulse};   // overlays, on every page
  mix(ov, sizeof(ov));
  float fv[] = {d.boost, d.boostText, d.peak, d.rpm, d.thr, d.iatF, d.cltF, d.oilF, d.batV, d.timing, d.maxIat, d.maxRpm,
                d.peakNeedle, d.rpmNeedle, d.tgtPsi,
                d.gLat, d.gLon, d.gPkL, d.gPkR, d.gPkAcc, d.gPkBrk, d.runT060, d.runT6130, d.runTQm, d.runMph,
                d.t060, d.t60130, d.tQm, d.qmMph, d.b060, d.b60130, d.bQm,
                d.afr1, d.afr2, d.afrCmd, d.hpfp, d.hpfpSet, d.lpfp, d.idc, d.knock, d.cat1F, d.cat2F, d.ambF, d.preF, d.icPump,
                d.clutchF, d.gbxF, d.diffF};
  // The Last Pull page shows none of the live values (only the pull, which pullsDone covers), and its
  // halo touches every row: redrawing it for every boost or RPM change would cost a full frame each time.
  const bool live = page != gr::P_PULL;
  if (live) mix(fv, sizeof(fv));
  if (live && d.gearPeak) mix(d.gearPeak, 7 * sizeof(float));
  if (live && d.gTrailN) mix(d.gTrail + 2 * (d.gTrailN - 1), 2 * sizeof(float));
  if (d.alertText && *d.alertText) mix(d.alertText, strlen(d.alertText));
  int iv[] = {page, t, shiftIdx, d.obd, d.statusWarn, d.sysAlert, d.gear, d.trans, d.driveMode, d.gCal, d.runState, d.gTrailN,
              d.onBoost, d.alertRed, d.knockCyl, gWatch.pullsDone, d.runObd, d.epb, d.staged,
              (page == gr::P_ENGINE && d.rpm >= gcfg.shiftRpm) ? (int)(now / 90) & 1 : 0};   // shift-light flash
  mix(iv, sizeof(iv));
  mix(sysOrder, sizeof(sysOrder));
  mix(d.status, strlen(d.status));
  if (d.runMsg) mix(d.runMsg, strlen(d.runMsg));
  if (d.toastA > 0) mix(d.toast, strlen(d.toast));
  if (!needFull && sig == lastSig) { skipped++; return; }
  lastSig = sig;

  // Draw into the buffer that isn't being sent (its own last send finished before the
  // previous frame was handed over) and work out which rows differ from what the panel
  // shows, then wait for the push task to finish the previous frame and hand this one
  // over. The push task sends it while the next frame is drawn.
  static int cur = 0;
  const uint16_t *pf = pageFace(t, page);          // may draw a night face first (about 0.1 s, once)
  uint32_t t0 = micros();
  gr::setTheme(night);
  gr::setShift(OFF[shiftIdx][0], OFF[shiftIdx][1]);
  fp::draw(cur, bufs[cur], bufs[cur ^ 1], pf, needFull, [&] { gr::renderDynamic(page, gcfg, d, (float)now); }, jobs[cur]);
  uint32_t t1 = micros();
  if (xSemaphoreTake(pushIdle, pdMS_TO_TICKS(1000)) != pdTRUE) {
    Serial.println(F("[lcd] push task stuck for 1 s - frame dropped"));
    needFull = true;                               // resync the panel once it's back
    return;
  }
  uint32_t t2 = micros();
  xQueueSend(pushQ, &cur, 0);
  cur ^= 1;
  needFull = false;
  drawMs += 0.1f * ((t1 - t0) / 1000.0f - drawMs);
  waitMs += 0.1f * ((t2 - t1) / 1000.0f - waitMs);
}

// Serial 'P': times each part of a frame on the real hardware (PSRAM included) and prints a
// table. Draws into a scratch buffer, never the screen. Takes about a second.
static void benchProfile() {
  uint16_t *tmp = (uint16_t *)ps_malloc(LCD_W * LCD_H * 2);
  if (!tmp) { Serial.println(F("[prof] no PSRAM for the scratch buffer")); return; }
  xSemaphoreTake(pushIdle, pdMS_TO_TICKS(1000));   // no frame being sent while we measure
  auto T = [](const char *name, int reps, auto fn) {
    esp_task_wdt_reset();
    uint32_t a = micros();
    for (int i = 0; i < reps; i++) fn(i);
    uint32_t b = micros();
    Serial.printf("[prof] %-34s %7.2f ms\n", name, (b - a) / 1000.0f / reps);
  };
  using namespace gr;
  const Cfg &k = gcfg;
  setTheme(false); setShift(0, 0); target(tmp);
  Data d; demo(5.0f, d); d.boostText = d.boost; d.peak = 8; d.status = ""; d.sysOrder = sysOrder; d.maxRpm = 7000; d.maxIat = 130;
  Serial.printf("[prof] CPU %lu MHz, QSPI %lu MHz, rotation %d\n", (unsigned long)getCpuFrequencyMhz(),
                (unsigned long)(LCD_QSPI_HZ / 1000000), DISPLAY_ROTATION);
  const int NP = P_COUNT < DAY_SLOTS ? P_COUNT : DAY_SLOTS;   // as many pages as stay cached together
  const uint16_t *faces[P_COUNT];
  for (int p = 0; p < NP; p++) faces[p] = pageFace(0, p);
  setTheme(false); setShift(0, 0); target(tmp);
  T("restore whole face (434 KB copy)", 5, [&](int) { beginFrame(faces[P_BOOST], true); tracking = false; });
  static const char *names[gr::P_COUNT] = {"Boost", "Overview", "Temps", "Engine", "System", "G-force", "Perf", "Gears", "Fuel", "Heat", "Last pull"};
  for (int p = 0; p < NP; p++) {
    char n[40]; snprintf(n, sizeof(n), "%s frame (restore + draw)", names[p]);
    beginFrame(faces[p], true); tracking = false;
    T(n, 20, [&](int i) {
      d.boost = d.boostText = (i & 1) ? 4.0f : 12.0f; d.rpm = (i & 1) ? 3000 : 6000; d.thr = (i & 1) ? 20 : 90;
      d.iatF = (i & 1) ? 100 : 130; d.cltF = (i & 1) ? 190 : 210;
      beginFrame(faces[p], false); renderDynamic(p, k, d, 0); int16_t r[16][2]; endFrame(r, 16);
    });
  }
  beginFrame(faces[P_BOOST], true); tracking = false;
  T("main needle + shadow", 20, [&](int i) { needle(psiDeg(k, (i & 1) ? 4.0f : 12.0f), 206, 36, 5.2f, kWhite, false); });
  T("peak telltale", 20, [&](int) { needle(psiDeg(k, 8), 200, 0, 1.6f, kRed, true); });
  T("hub (cached)", 20, [&](int) { hub(26); });
  T("big digits '12.4'", 20, [&](int) { text(R40, "12.4", C + 92, C + 78, kWhite); });
  T("small labels x3", 20, [&](int) { label(C12, "PSI BOOST", C + 92, C + 104, kWhite); label(C13, "PEAK 8.0", C + 92, C + 122, kRed);
                                      text(C12, "OBD SEARCHING", C + 88, C + 142, kAmber, 0, 1.0f); });
  needleParts = 1;
  T("  shadow only", 20, [&](int i) { needle(psiDeg(k, (i & 1) ? 4.0f : 12.0f), 206, 36, 5.2f, kWhite, false); });
  needleParts = 2;
  T("  body only", 20, [&](int i) { needle(psiDeg(k, (i & 1) ? 4.0f : 12.0f), 206, 36, 5.2f, kWhite, false); });
  needleParts = 3;
  // where does a blend's time go: memory or maths?
  T("10k blends into PSRAM", 5, [&](int) { for (int y = 0; y < 100; y++) for (int x = 0; x < 100; x++) blendI(180 + x, 180 + y, 0xF800, 128); });
  T("10k plain stores into PSRAM", 5, [&](int) { for (int y = 0; y < 100; y++) for (int x = 0; x < 100; x++) blendI(180 + x, 180 + y, 0xF800, 256); });
  {
    uint16_t *ram = (uint16_t *)heap_caps_malloc(LCD_W * 22 * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (ram) {                                     // same blends into a 22-row strip of internal RAM
      int sq = rotQ; setRotation(0); target(ram);
      T("10k blends into internal RAM", 5, [&](int) { for (int y = 0; y < 22; y++) for (int x = 0; x < 455; x++) blendI(x, y, 0xF800, 128); });
      target(tmp); setRotation(sq);
      free(ram);
    } else Serial.println(F("[prof] (no internal RAM for the comparison)"));
  }
  T("needle maths only (nothing written)", 20, [&](int i) {
    Pt q[6]; float a = psiDeg(k, (i & 1) ? 4.0f : 12.0f) * D2R, ca = cos_(a), sa = sin_(a), px = -sa, py = ca;
    q[0] = {C + ca * 206 + px * 1.2f, C + sa * 206 + py * 1.2f}; q[1] = {C + ca * 206 - px * 1.2f, C + sa * 206 - py * 1.2f};
    q[2] = {C - px * 5.2f, C - py * 5.2f}; q[3] = {C - ca * 36 - px * 3.6f, C - sa * 36 - py * 3.6f};
    q[4] = {C - ca * 36 + px * 3.6f, C - sa * 36 + py * 3.6f}; q[5] = {C + px * 5.2f, C + py * 5.2f};
    poly(q, 6, kWhite, 0.0f, 3.0f); poly(q, 6, kWhite, 0.0f);   // alpha 0: every pixel computed, none written
  });
  setRotation(0);
  T("main needle + shadow, no rotation", 20, [&](int i) { needle(psiDeg(k, (i & 1) ? 4.0f : 12.0f), 206, 36, 5.2f, kWhite, false); });
  setRotation(DISPLAY_ROTATION);
  T("compare 100 rows (memcmp)", 10, [&](int) { volatile int c = 0; for (int y = 0; y < 100; y++) c = c + (memcmp(tmp + y * LCD_W, faces[1] + y * LCD_W, LCD_W * 2) != 0); });
  T("PSRAM copy 100 rows", 10, [&](int) { memcpy(tmp + 200 * LCD_W, faces[1], 100 * LCD_W * 2); });
  Serial.printf("[prof] live: draw %.1f ms, push %.1f ms (%.1f%% of the screen), waiting for push %.1f ms\n",
                drawMs, (float)pushMs, (float)rowsPct, waitMs);
  xSemaphoreGive(pushIdle);
  free(tmp);
  needFull = true;
}

// Push task (core 0): owns the panel once the splash is over. Sends each finished frame
// (only the rows that changed) and applies brightness changes between frames.
static void pushTask(void *) {
  for (;;) {
    int k;
    bool got = xQueueReceive(pushQ, &k, pdMS_TO_TICKS(50)) == pdTRUE;
    int16_t br = brightReq;
    if (br >= 0) { brightReq = -1; panel->setBrightness((uint8_t)br); }
    if (!got) continue;
    uint32_t t0 = micros();
    long px = fp::push(jobs[k], [](const uint16_t *b, int x, int y, int w, int h) {
      if (w == LCD_W) { panel->draw16bitRGBBitmap(0, y, (uint16_t *)b + y * LCD_W, LCD_W, h); return; }
      // Part of a row: set the panel window once, then stream the rows through a small
      // internal-RAM buffer (the rectangle isn't contiguous in the frame buffer)
      panel->startWrite();
      panel->writeAddrWindow(x, y, w, h);
      int n = 0;
      for (int r = 0; r < h; r++) {
        if (n + w > STAGE_PX) { bus->writePixels(stage, n); n = 0; }
        memcpy(stage + n, b + (y + r) * LCD_W + x, w * 2);
        n += w;
      }
      if (n) bus->writePixels(stage, n);
      panel->endWrite();
    });
    float ms = (micros() - t0) / 1000.0f;
    pushMs  = pushMs + 0.1f * (ms - pushMs);
    rowsPct = rowsPct + 0.1f * (100.0f * px / (LCD_W * LCD_H) - rowsPct);
    xSemaphoreGive(pushIdle);
  }
}

// Static layer of each page, day and night, from the current settings, drawn when first
// needed: the page on screen first (behind the boot badge), more while the screen is idle (see
// loop), the rest when shown. The charcoal dial every day page shares is computed once at boot
// (on the other core) and kept, so a day background takes about 40 ms.
static uint16_t *faceBaseBuf = nullptr;
static void cfgFromSettings() {
  gcfg.minPsi = GAUGE_MIN_PSI;  gcfg.maxPsi = GAUGE_MAX_PSI;  gcfg.redPsi = S.redPsi;
  gcfg.startDeg = GAUGE_START_DEG;  gcfg.zeroDeg = GAUGE_ZERO_DEG;  gcfg.endDeg = GAUGE_END_DEG;
  gcfg.shiftRpm = S.shiftRpm;  gcfg.redlineRpm = S.redlineRpm;
  gcfg.telltale = S.telltale;  gcfg.digital = S.digital;
}
// The page's background from its pool; if it isn't there (and build), the least recently used
// slot is redrawn for it. Without build: nullptr when it isn't cached.
static uint16_t *poolFace(bool night, int p, bool build = true) {
  uint16_t **buf = night ? nightBuf : dayBuf;
  int8_t *pg = night ? nightPage : dayPage;
  uint32_t *used = night ? nightUsed : dayUsed;
  int n = night ? NIGHT_SLOTS : DAY_SLOTS, pick = 0;
  for (int i = 0; i < n; i++) {
    if (pg[i] == p) { if (build) used[i] = millis(); return buf[i]; }
    if (pg[i] < 0 || (pg[pick] >= 0 && used[i] < used[pick])) pick = i;
  }
  if (!build) return nullptr;
  uint32_t t0 = millis();
  if (!night && !faceBaseBuf) {
    faceBaseBuf = (uint16_t *)ps_malloc(LCD_W * LCD_H * 2);
    gr::setFaceCache(faceBaseBuf);                  // may be null: then every face computes the dial itself
  }
  gr::setTheme(night); gr::setShift(0, 0);
  gr::target(buf[pick]);
  gr::renderStatic(p, gcfg);
  gr::setTheme(false);
  pg[pick] = (int8_t)p; used[pick] = millis();
  Serial.printf("[lcd] %s face for page %d drawn in %lu ms\n", night ? "night" : "day", p, (unsigned long)(millis() - t0));
  return buf[pick];
}
static const uint16_t *pageFace(int night, int p) { return poolFace(night != 0, p); }
static bool dayPoolHasRoom() { for (int8_t q : dayPage) if (q < 0) return true; return false; }
// Settings changed (red zone, shift point, redline...): every face is redrawn as it's needed
static void buildFaces() {
  cfgFromSettings();
  for (int i = 0; i < DAY_SLOTS; i++) dayPage[i] = -1;
  for (int i = 0; i < NIGHT_SLOTS; i++) nightPage[i] = -1;
}

#include "web_ui.h"       // Wi-Fi + web UI (reads the globals above)

// Boot splash: S badge fades in, stays while the faces are drawn, then fades out.
static uint32_t splashAt = 0;
static char     splashLine[64] = "";
static bool     splashWarn = false;
static void splashFrame(float fade) {
  gr::setShift(0, 0);
  gr::target(gfx->getFramebuffer());
  gr::renderSplash(fade, splashLine, splashWarn, lastReset, "V" FW_VERSION);
  gfx->flush();
}
static void splashFade(bool in, uint32_t ms) {
  uint32_t t0 = millis();
  for (;;) {
    float f = min(1.0f, (millis() - t0) / (float)ms);
    splashFrame(in ? f : 1 - f);
    if (f >= 1) break;
  }
}

// ---------------------------------------------------------------------
//  Setup / loop
// ---------------------------------------------------------------------
// Firmware updates over Wi-Fi (web_ui.h): a new build starts "on trial". The core would
// confirm it straight away at boot; instead it's confirmed only after 30 s of normal running
// (otaConfirm in the loop), so a build that crashes or hangs early is undone by the bootloader
// at the next reset, back to the build that installed it.
extern "C" bool verifyRollbackLater() { return true; }
static void otaConfirm() {
  const esp_partition_t *run = esp_ota_get_running_partition();
  esp_ota_img_states_t st;
  if (run && esp_ota_get_state_partition(run, &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY) {
    esp_ota_mark_app_valid_cancel_rollback();
    Serial.printf("[ota] new firmware (%s) has run 30 s: kept\n", run->label);
  }
}

// Start-up timing, printed with "first page ready"
static char     bootSteps[160] = "";
static uint32_t bootLast = 0;
static void bootMark(const char *what) {
  uint32_t now = millis();
  char b[32]; snprintf(b, sizeof(b), "%s%s %lu", bootSteps[0] ? ", " : "", what, (unsigned long)(now - bootLast));
  strlcat(bootSteps, b, sizeof(bootSteps));
  bootLast = now;
}
static volatile bool     faceDone = false;
static volatile uint32_t faceMs = 0;

static void halt(const char *why) {
  for (;;) { Serial.printf("[FATAL] %s\n", why); delay(2000); }
}

static void printHelp() {
  Serial.println(F("Commands: p=reset peak  z=re-zero (engine OFF)  d=toggle demo  s=I2C scan  1-9=page  n=next page  l=night face  f=fast log  w=Wi-Fi on/off  u=G-meter re-level  P=draw timing  h=help"));
}

void setup() {
  // USB serial must never hold up the gauge: with a USB host that isn't reading (a car's USB
  // port, a laptop with no monitor open) each print used to wait up to 2 s for room. Big
  // buffer, 1 ms timeout: if nobody reads, lines are simply lost.
  Serial.setTxBufferSize(4096);
  Serial.begin(115200);
  Serial.setTxTimeoutMs(1);
  // Wait for a serial monitor only if a computer is on the USB port (in the car nothing is,
  // and waiting cost 1.5 s on every start)
  uint32_t t0 = millis();
  while (!HWCDC::isPlugged() && millis() - t0 < 60) delay(5);
  if (HWCDC::isPlugged()) while (!Serial && millis() - t0 < 1500) delay(10);
  bootMark(HWCDC::isPlugged() ? "usb serial" : "start");
  Serial.println(F("\n=== S5 Gauge Pod firmware " FW_VERSION " (built " __DATE__ " " __TIME__ ") ==="));
  bootReport();
  Serial.printf("PSRAM: %u KB free, heap: %u KB free\n",
                (unsigned)(ESP.getFreePsram() / 1024), (unsigned)(ESP.getFreeHeap() / 1024));
  if (!psramFound()) halt("No PSRAM - set Tools > PSRAM > OPI PSRAM");

  prefs.begin("gaugepod", false);
  bootNo = (uint16_t)(prefs.getUShort("boot", 0) + 1); prefs.putUShort("boot", bootNo);
  {                                    // an update that didn't start properly was undone: say so once
    const esp_partition_t *bad = esp_ota_get_last_invalid_partition();
    uint32_t seen = prefs.getUInt("otabad", 0);
    if (bad && bad->address != seen) {
      prefs.putUInt("otabad", bad->address);
      Serial.println(F("[ota] the last update didn't start properly: went back to the previous firmware"));
      snprintf(lastReset, sizeof(lastReset), "UPDATE FAILED: PREVIOUS FIRMWARE KEPT");
    }
  }
  settingsLoad(prefs);
  gZero = prefs.getFloat(ZERO_KEY, ZERO_DEFAULT);
  gZeroRefV = prefs.getFloat("zrefV", 0); gZeroRefKpa = prefs.getFloat("zrefK", 0);
  if (gZero < ZERO_WIDE_MIN || gZero > ZERO_MAX) { // e.g. a zero saved mid-crank by older firmware
    Serial.printf("[cfg] saved zero %.3f is outside %.2f-%.2f, using the default %.3f\n",
                  (float)gZero, (float)ZERO_WIDE_MIN, (float)ZERO_MAX, (float)ZERO_DEFAULT);
    gZero = ZERO_DEFAULT;
    prefs.putFloat(ZERO_KEY, gZero);                // and stop it coming back
  }
  Serial.printf("[cfg] sensor: %s, saved zero %.3f %s\n", SENSOR_NAME, (float)gZero, ZERO_UNIT);
  gPsiTrim = prefs.getFloat("psitrim", 0);
  if (!(gPsiTrim > -1.5f && gPsiTrim < 1.5f)) gPsiTrim = 0;
  Serial.printf("[cal] sender trim against the ECU: %+.2f psi (%s)\n", (float)gPsiTrim, S.obdTrim ? "on" : "off");

  // --- Display first, so nothing on the I2C bus can leave the screen dark
  pinMode(PIN_LCD_EN, OUTPUT); digitalWrite(PIN_LCD_EN, HIGH);
#if LCD_PANEL == 0
  panelId = readPanelId();
  panelIsSH8601 = (panelId == 0x86);
  Serial.printf("[lcd] panel ID 0x%02X -> %s\n", panelId, panelIsSH8601 ? "SH8601" : "CO5300");
#else
  panelIsSH8601 = (LCD_PANEL == 2);
  Serial.printf("[lcd] %s panel (config.h LCD_PANEL)\n", panelIsSH8601 ? "SH8601" : "CO5300");
#endif

  bus = new Arduino_ESP32QSPI(PIN_LCD_CS, PIN_LCD_SCK, PIN_LCD_D0, PIN_LCD_D1, PIN_LCD_D2, PIN_LCD_D3);
  if (panelIsSH8601)
    panel = new Arduino_SH8601(bus, PIN_LCD_RST, LCD_ROTATION, LCD_W, LCD_H);
  else
    panel = new Arduino_CO5300(bus, PIN_LCD_RST, LCD_ROTATION, LCD_W, LCD_H,
                               CO5300_COL_OFFSET, 0, CO5300_COL_OFFSET, 0);
  gfx = new Arduino_Canvas(LCD_W, LCD_H, panel);
  if (!gfx->begin(LCD_QSPI_HZ)) halt("Display init failed (canvas alloc or panel begin)");
  gr::setRotation(DISPLAY_ROTATION);
  panel->setBrightness(brightness);
  bootMark("display");

  // --- The charcoal dial every day page sits on takes most of a second to compute, so the
  //     other core starts on it now, while this one sets up the sensors and shows the badge
  faceBaseBuf = (uint16_t *)ps_malloc(LCD_W * LCD_H * 2);
  if (faceBaseBuf) xTaskCreatePinnedToCore([](void *) {
      uint32_t t = millis();
      gr::precomputeFace(faceBaseBuf);
      faceMs = millis() - t;
      faceDone = true;
      vTaskDelete(nullptr);
    }, "face", 4096, nullptr, 1, nullptr, 0);
  else faceDone = true;

  // --- I2C bus + devices (every read here gives up on its own, so a stuck chip can't
  //     hold the boot: the screen is already on)
  bool busOk = i2cRecover();
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_HZ);
  Wire.setTimeOut(20);
  if (busOk) i2cScan(); else Serial.println(F("[i2c] bus stuck: scan skipped"));
  for (int i = 0; i < 3 && !hasTouch; i++) { hasTouch = i2cPing(ADDR_TOUCH); if (!hasTouch) delay(30); }
  hasRTC   = i2cPing(ADDR_RTC);
  uint8_t imuAddr = i2cPing(0x6B) ? 0x6B : (i2cPing(0x6A) ? 0x6A : 0);
  hasIMU   = IMU_ENABLED && imuAddr && imu::begin(imuAddr);
  {
    imu::Cal saved;
    if (prefs.getBytesLength("imucal") == sizeof(saved)) prefs.getBytes("imucal", &saved, sizeof(saved));
    gimu.begin(saved, imu::v3(IMU_FWD_X, IMU_FWD_Y, IMU_FWD_Z));
    Serial.printf("[imu] calibration: %s, forward %s\n", saved.upOk ? "level saved" : "level not learned yet",
                  saved.fwdSrc == 2 ? "learned" : "assumed until the first pulls");
  }
  {
    gearx::Cal gc;
    bool have = prefs.getBytesLength("gearcal") == sizeof(gc) && prefs.getBytes("gearcal", &gc, sizeof(gc)) == sizeof(gc);
    gearEst.begin(have ? &gc : nullptr);
    obd::onRpmSpeed = [](float rpm, float kmh) {
      portENTER_CRITICAL(&gearMux); gearEst.update(rpm, kmh, millis() / 1000.0f); portEXIT_CRITICAL(&gearMux);
    };
    Serial.printf("[gear] ratios %s:", have ? "learned" : "starting values");
    for (int g = 0; g < gearx::N; g++) Serial.printf(" %.1f", gearEst.cal.r[g]);
    Serial.println(F(" rpm per km/h"));
  }
  {
    watch::Curve c;
    bool have = prefs.getBytesLength("bcurve") == sizeof(c) && prefs.getBytes("bcurve", &c, sizeof(c)) == sizeof(c);
    gWatch.begin(have ? &c : nullptr);
    gWatch.pBoost = gr::P_BOOST; gWatch.pOverview = gr::P_OVERVIEW; gWatch.pTemps = gr::P_TEMPS; gWatch.pSystem = gr::P_SYSTEM;
    gWatch.pFuel = gr::P_FUEL; gWatch.pHeat = gr::P_HEAT; gWatch.pPull = gr::P_PULL;
    Serial.printf("[pull] usual boost learned from %u pull(s)\n", (unsigned)gWatch.curve.pulls);
  }
  {                                              // performance runs and the clock
    bool have = prefs.getBytesLength("runs") == sizeof(gRuns) && prefs.getBytes("runs", &gRuns, sizeof(gRuns)) == sizeof(gRuns) && gRuns.magic == RUNS_MAGIC;
    if (!have) { memset(&gRuns, 0, sizeof(gRuns)); gRuns.magic = RUNS_MAGIC; }
    for (RunRec &x : gRuns.r) x.flags &= ~RF_UNDATED;          // a previous power-on: can't be dated now
    for (auto &row : gRuns.best) for (RunRec &x : row) x.flags &= ~RF_UNDATED;
    time_t t;
    if (hasRTC && rtcRead(t)) { struct timeval tv = {t, 0}; settimeofday(&tv, nullptr); clockOk = true; clockSrc = 1; }
    tzOff = prefs.getInt("tzoff", INT32_MIN);
    Serial.printf("[run] %u saved run(s); clock %s\n", (unsigned)gRuns.n, clockOk ? "kept by the RTC" : "not set (the web page sets it from the phone)");
  }
  bootMark("i2c");
  hasADS   = i2cPing(ADDR_ADS1115) && ads.begin(ADDR_ADS1115, &Wire);
  if (hasTouch) touchInit();
  else Serial.println(F("[touch] FT3168 not answering at boot: will keep trying (touch the screen to wake it)"));
  if (hasADS) {
    ads.setGain(GAIN_ONE);                 // +/-4.096 V full scale, 125 uV/count
    ads.setDataRate(RATE_ADS1115_860SPS);
    captureZero(true);                     // first thing: before anyone can start the engine
    bootMark("zero");
  } else {
    gDemo = true;
    Serial.println(F("[ads] ADS1115 not found at 0x48 -> DEMO mode"));
  }

  xTaskCreatePinnedToCore(sensorTask, "sensor", 4096, nullptr, 2, nullptr, 0);

  // --- OBD, logging, Wi-Fi: started now, on the other core, so they come up while the badge
  //     and the first page are drawn (Wi-Fi and the log partition each take a moment)
  strlcpy(obd::nameFilter, S.obdName, sizeof(obd::nameFilter));
  // "obdaddr2": only written once a car has answered through the adapter. The old "obdaddr"
  // was saved on any connection (it could hold another car's adapter), so it's dropped.
  if (prefs.isKey("obdaddr")) prefs.remove("obdaddr");
  if (prefs.isKey("obdaddr2")) prefs.getString("obdaddr2", obd::savedAddr, sizeof(obd::savedAddr));
  obd::enabled = S.obdOn;
  xTaskCreatePinnedToCore([](void *) {
    obd::begin();
    // The internal flash is formatted only until it has mounted once. After that a failed mount is left
    // alone (it holds the health history; the Logs tab offers to format it).
    bool everOk = prefs.getBool("ffatok", false);
    if (!logx::begin(!everOk)) Serial.println(F("[log] no SD card and no usable internal storage: logging off (check Partition Scheme)"));
    if (logx::flash.ok && !everOk) prefs.putBool("ffatok", true);
    catSavedOk = logx::flash.ok && health::catLast(CATS_PATH, catSaved);   // the last catalyst result, to spot a new one
    web::begin();
    vTaskDelete(nullptr);
  }, "start", 8192, nullptr, 1, nullptr, 0);

  // --- Boot splash (missing parts are listed in amber under the badge)
  splashLine[0] = 0;
  if (!hasADS)   strlcat(splashLine, "ADS1115 MISSING \xc2\xb7 OBD BOOST  ", sizeof(splashLine));
  if (!hasTouch) strlcat(splashLine, "TOUCH NOT ANSWERING  ", sizeof(splashLine));
  splashWarn = splashLine[0] != 0;
  splashAt = millis();
  splashFade(true, SPLASH_FADE_MS);
  bootMark("badge");

  // --- Page backgrounds: buffers for all, but only the page about to be shown is drawn now
  for (int i = 0; i < DAY_SLOTS; i++) {
    dayBuf[i] = (uint16_t *)ps_malloc(LCD_W * LCD_H * 2);
    if (!dayBuf[i]) halt("Face buffer alloc failed");
  }
  for (int i = 0; i < NIGHT_SLOTS; i++) {
    nightBuf[i] = (uint16_t *)ps_malloc(LCD_W * LCD_H * 2);
    if (!nightBuf[i]) halt("Night face buffer alloc failed");
  }
  buildFaces();                                     // settings -> gcfg, nothing drawn yet
  // System page order: restore, and fall back to the default if it isn't a valid permutation
  memcpy(sysOrder, gr::SYS_DEFAULT_ORDER, sizeof(sysOrder));
  if (prefs.getBytes("sys", sysOrder, sizeof(sysOrder)) == sizeof(sysOrder)) {
    uint8_t seen = 0;
    for (uint8_t v : sysOrder) seen |= (v < gr::SI_COUNT) ? (1 << v) : 0x80;
    if (seen != (1 << gr::SI_COUNT) - 1) memcpy(sysOrder, gr::SYS_DEFAULT_ORDER, sizeof(sysOrder));
  } else memcpy(sysOrder, gr::SYS_DEFAULT_ORDER, sizeof(sysOrder));
  page = prefs.getInt("page", START_PAGE);
  if (page < 0 || page >= gr::P_COUNT) page = START_PAGE;
#if LIGHT_SENSE_ENABLED
  analogSetPinAttenuation(PIN_LIGHT_SENSE, ADC_11db);
  lightV = analogReadMilliVolts(PIN_LIGHT_SENSE) / 1000.0f * LIGHT_DIV_RATIO;   // start in the right theme
  lightsOn = lightV > LIGHT_ON_V;
#endif
  uint32_t tw = millis();
  while (!faceDone) delay(2);                       // the dial (other core) is usually done by now
  if (faceBaseBuf) gr::setFaceCacheReady(faceBaseBuf);
  if (millis() - tw > 2) bootMark("dial wait");
  pageFace(S.nightTheme && lightsOn ? 1 : 0, page);  // the first page's background, behind the badge
  bootMark("page");
  Serial.printf("[boot] first page ready %lu ms after power-on (%s; dial %lu ms on the other core)\n",
                (unsigned long)millis(), bootSteps, (unsigned long)faceMs);

  gPsiFilt = 0;
  gPeakPsi = 0;
  // a short flash of the badge, then straight to the gauge; longer if there's a warning to read under it
  uint32_t hold = (splashWarn || lastReset[0]) ? 3000 : SPLASH_MS;
  while (millis() - splashAt < hold) delay(10);
  readyAt = millis() + 1400;               // key-on needle sweep
  needFull = true;

  // --- Second frame buffer + push task (from here on only the push task talks to the panel)
  bufs[0] = gfx->getFramebuffer();
  bufs[1] = (uint16_t *)ps_malloc(LCD_W * LCD_H * 2);
  if (!bufs[1]) halt("Second frame buffer alloc failed");
  memcpy(bufs[1], bufs[0], LCD_W * LCD_H * 2);
  fp::wholeRows = !LCD_PARTIAL_COLUMNS;
  stage = (uint16_t *)heap_caps_malloc(STAGE_PX * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!stage) halt("Staging buffer alloc failed");
  pushQ = xQueueCreate(1, sizeof(int));
  pushIdle = xSemaphoreCreateBinary();
  xSemaphoreGive(pushIdle);
  xTaskCreatePinnedToCore(pushTask, "push", 4096, nullptr, 2, nullptr, 0);

  touchQ = xQueueCreate(8, sizeof(TouchEv));
  xTaskCreatePinnedToCore(touchTask, "touch", 4096, nullptr, 3, nullptr, 1);   // also if it didn't answer yet
  // Loop watchdog: if the UI loop ever hangs for 8 s the ESP restarts (and the splash says so)
  // instead of sitting on a frozen or dark screen.
  esp_task_wdt_config_t wdc = {};
  wdc.timeout_ms = 8000; wdc.idle_core_mask = 0; wdc.trigger_panic = true;
  if (esp_task_wdt_reconfigure(&wdc) != ESP_OK) esp_task_wdt_init(&wdc);
  esp_task_wdt_add(nullptr);
  printHelp();
}

void loop() {
  static uint32_t lastFrame = 0, lastLog = 0, lastSec = 0, lastLight = 0;
  static uint32_t lastSamples = 0, lastShift = 0;
  uint32_t now = millis();
  esp_task_wdt_reset();
  { static uint32_t prev = 0; if (prev && now - prev > loopGapMax) loopGapMax = now - prev; prev = now; }
  STAGE(1);

  // ---- serial commands
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'p') { resetPeak(); Serial.println(F("[cmd] peak reset")); }
    else if (c == 'z') { requestRezero(); Serial.println(F("[cmd] re-zero")); }
    else if (c == 'd') { if (hasADS) { gDemo = !gDemo; Serial.printf("[cmd] demo %s\n", gDemo ? "ON" : "OFF"); } }
    else if (c == 's') i2cScan();
    else if (c >= '1' && c <= '0' + gr::P_COUNT) setPage(c - '1');
    else if (c == 'n') setPage(page + 1);
    else if (c == 'l') { forceNight = !forceNight; Serial.printf("[cmd] night face %s\n", forceNight ? "forced ON" : "follows headlights"); }
    else if (c == 'f') { fastLog = !fastLog; Serial.printf("[cmd] fast log %s\n", fastLog ? "ON (50 Hz)" : "OFF"); }
    else if (c == 'w') { S.wifiOn = !S.wifiOn; settingsSave(prefs); Serial.printf("[cmd] Wi-Fi %s after a restart\n", S.wifiOn ? "ON" : "OFF"); }
    else if (c == 'P') benchProfile();
    else if (c == 'u') { gReqGLevel = true; Serial.println(F("[cmd] G-meter: re-level at the next 2 s of stillness")); }
    else if (c == 'h') printHelp();
  }

  STAGE(2);
  // ---- touch gestures from touchTask: hold 1 s = peak reset, tap = System item to top or
  //      dismiss a warning, hold 3 s = re-zero, swipe left/up = next page, right/down = previous
  TouchEv ev;
  while (xQueueReceive(touchQ, &ev, 0) == pdTRUE) {
    if (ev.type == 2 && page == gr::P_GFORCE) { gReqGLevel = true; toast("LEVELLING - HOLD STILL"); Serial.println(F("[touch] long press -> G-meter re-level")); }
    else if (ev.type == 2) { requestRezero(); toast("RE-ZERO"); Serial.println(F("[touch] long press -> re-zero")); }
    else if (ev.type == 3 || ev.type == 4) { setPage(page + (ev.type == 3 ? 1 : -1)); Serial.printf("[touch] swipe -> page %d\n", page); }
    else if ((ev.type == 1 || ev.type == 5) && alertOn >= 0) {   // a tap with a warning showing: dismiss it, go back
      alertAck[alertOn] = true; alertAckSev[alertOn] = gWatch.a[alertOn].sev;
      Serial.printf("[touch] warning \"%s\" dismissed\n", gWatch.a[alertOn].text);
      if (alertFrom >= 0 && autoPaged) setPage(alertFrom, false);
      alertFrom = -1; alertOn = -1;
    }
    else if (ev.type == 1) {                        // a quick tap only swaps the System items (nothing to lose)
      if (page == gr::P_SYSTEM && gr::systemTap(sysOrder, ev.x, ev.y, sysAlertNow)) {
        prefs.putBytes("sys", sysOrder, sizeof(sysOrder));     // tapped item is now the big one
        Serial.printf("[touch] system: item %u to the top\n", sysOrder[0]);
      }
    }
    else if (ev.type == 5) {                        // held 1 s: reset what this page keeps
      if (page == gr::P_GFORCE) {
        gReqGPeak = true; toast("G PEAKS RESET");
      } else if (page == gr::P_PERF) {
        portENTER_CRITICAL(&perfMux); gPerf.resetBest(); portEXIT_CRITICAL(&perfMux);
        toast("BESTS RESET");
      } else {
        resetPeak();
        Serial.printf("[touch] hold at %d,%d -> peak reset\n", ev.x, ev.y);
      }
    }
  }
  STAGE(3);
  // ---- requests from the web UI (it runs on another core)
  if (web::reqPeak)   { web::reqPeak = false; resetPeak(); }
  if (web::reqRezero) { web::reqRezero = false; requestRezero(); toast("RE-ZERO"); }
  // The key-on zero was refused (too low: altitude, or the engine was already turning) and the saved one
  // is in use. Once the ECU has said "engine stopped" for 3 s (manifold back at atmosphere), a steady
  // reading is the atmosphere whatever the altitude: take it then, once.
  {
    static bool tried = false; static uint32_t offSince = 0;
    if (!engineStopped()) offSince = 0; else if (!offSince) offSince = now;
    float e = zeroExpect();                      // and only with the barometer to check it against
    if (!tried && hasADS && !gDemo && !gZeroFromBoot && !gSensorFault && gr::has(e) && now > readyAt + 5000 && offSince && now - offSince > 3000) {
      tried = true; gZeroExpect = e; gZeroWide = true; gReqRezero = true;
      Serial.println(F("[zero] key-on zero wasn't taken and the ECU says the engine is stopped: zeroing now"));
    }
  }
  // Pair each zero taken in the usual window with the ECU's barometer (it arrives within a few seconds
  // of OBD coming up): that pair is what zeroExpect() works from on a trip to altitude. Saved only when
  // it has moved, so not every key-on writes to flash.
#if SENSOR_TYPE == SENSOR_AWE
  {
    static uint32_t pairedGen = 0;
    uint32_t gen = gZeroGen;
    if (gen != pairedGen) {
      if (gZero < ZERO_MIN || (int32_t)(millis() - gZeroAt) > 120000) pairedGen = gen;   // a wide zero, or no barometer in time
      else if (obd::fresh(obd::F_BARO, obd::ageFor(obd::F_BARO)) && flashCalm()) {
        pairedGen = gen;
        float k = obd::val[obd::F_BARO], v = gZero;
        if (k > 50 && k < 120 && (fabsf(v - gZeroRefV) > 0.004f || fabsf(k - gZeroRefKpa) > 0.6f)) {
          gZeroRefV = v; gZeroRefKpa = k;
          prefs.putFloat("zrefV", v); prefs.putFloat("zrefK", k);
          Serial.printf("[zero] %.3f V paired with the ECU's barometer %.0f kPa (for zeroing at altitude)\n", v, k);
        }
      }
    }
  }
#endif
  if (web::reqPage)   { setPage(page + web::reqPage); web::reqPage = 0; }
  if (web::reqClock)  { uint32_t e = web::reqClock; web::reqClock = 0; setClock(e); }
  {                                              // the car's clock (steering module): learn the time zone, or set the clock
    static uint16_t seenCar = 0;
    bool fresh = obd::carLocal && millis() - obd::carClockAt < 1200000;
    if (obd::carClockN != seenCar || (tzCheckReq && fresh)) {
      seenCar = obd::carClockN; tzCheckReq = false;
      uint32_t car = obd::carNow();
      if (clockOk && clockSrc != 3) {
        long d = (long)car - (long)time(nullptr);
        int32_t off = (int32_t)(lroundf(d / 900.0f) * 900);
        if (labs(d - off) < 300 && labs(off) <= 14 * 3600 && off != tzOff) {   // a clean zone offset (the car's clock within 5 min)
          tzOff = off; prefs.putInt("tzoff", tzOff);
          Serial.printf("[clock] the car's clock is UTC%+.2g h: saved\n", tzOff / 3600.0f);
        }
      } else if (!clockOk && tzOff != INT32_MIN) {
        setClock(car - tzOff, 3);
        Serial.printf("[clock] set from the car's clock (UTC%+.2g h)\n", tzOff / 3600.0f);
      }
    }
  }
  if (web::reqRunsClear) { web::reqRunsClear = false; runsClear(); }
  if (web::reqFaces)  { web::reqFaces = false; esp_task_wdt_reset(); buildFaces(); esp_task_wdt_reset(); needFull = true; }
  {                                              // warm-up ring: part of the backgrounds, so they're redrawn when it
    static uint32_t ringSince = 0;               // comes or goes (held 3 s first, so a dropped oil reading doesn't flap it)
    bool want = !isDemo() && gr::has(gColdOilF);
    if (want == gr::coldRing) ringSince = 0;
    else if (!ringSince) ringSince = now;
    else if (now - ringSince >= 3000) {
      gr::coldRing = want; ringSince = 0;
      buildFaces(); needFull = true;
      Serial.printf("[warm] ring %s: backgrounds redrawn as they're shown\n", want ? "on" : "off");
    }
  }
  // ---- firmware update in progress (web task): close the log first, show the progress
  {
    static int shownPct = -2;
    if (web::updating && shownPct == -2) { logx::stop(); Serial.println(F("[ota] update started: log closed")); }
    if (web::updating && web::updPct / 10 != shownPct / 10) {
      shownPct = web::updPct;
      char m[24]; snprintf(m, sizeof(m), "UPDATING %d%%", web::updPct < 0 ? 0 : web::updPct); toast(m);
    }
    if (!web::updating && shownPct != -2) shownPct = -2;
  }
  if (web::reqRestart) {
    logx::stop();
    for (int i = 0; i < 30 && logx::open; i++) { esp_task_wdt_reset(); delay(100); }   // let the log save
    delay(300); ESP.restart();
  }
  { static bool otaDone = false; if (!otaDone && now > 30000) { otaDone = true; otaConfirm(); } }
  // ---- sender trim against the ECU: at a steady idle, each new OBD manifold reading nudges the
  //      offset (about a 20 s time constant, whatever the reading rate), kept within +/-1.5 psi, saved every 5 minutes
  if (S.obdTrim && hasADS && !gDemo && !gSensorFault) {
    using namespace obd;
    static uint32_t lastMapAt = 0, lastSave = 0; static float prevMap = -1, savedTrim = 0;
    if (valAt[F_MAP] != lastMapAt) {
      float gap = lastMapAt ? min(3.0f, (valAt[F_MAP] - lastMapAt) / 1000.0f) : 0.4f;   // s since the last reading
      lastMapAt = valAt[F_MAP];
      float m = val[F_MAP];
      bool steady = prevMap > 0 && fabsf(m - prevMap) < 2.0f;
      prevMap = m;
      bool idle = fresh(F_RPM, 1000) && val[F_RPM] > 550 && val[F_RPM] < 1100 && fresh(F_KMH, 1500) && val[F_KMH] == 0 &&
                  fresh(F_BARO, ageFor(F_BARO));
      if (idle && steady) {
        float err = gPsiText - (m - val[F_BARO]) * 0.145038f;
        if (fabsf(err) < 2.5f) {
          gPsiTrim = fminf(1.5f, fmaxf(-1.5f, gPsiTrim + 0.05f * gap * err));   // 0.02 a reading at 2.5 a second
          gTrimN = gTrimN + 1;
          if (fabsf(gPsiTrim - savedTrim) > 0.03f && (now - lastSave > 300000 || gTrimN == 100)) {
            lastSave = now; savedTrim = gPsiTrim; prefs.putFloat("psitrim", gPsiTrim);
            Serial.printf("[cal] sender trim against the ECU: %+.2f psi (%lu idle samples)\n", (float)gPsiTrim, (unsigned long)gTrimN);
          }
        }
      }
    }
  }
  // ---- gear: peak boost per gear on the throttle, and the learned ratios saved now and then
  {
    int g = gearEst.gear;
    // "on the throttle" = pedal 70 % or more (the throttle plate is no guide on the 3.0T: it's
    // nearly wide open whenever the supercharger works); throttle only if there's no pedal reading
    bool wot = obd::fresh(obd::F_PEDAL, obd::ageFor(obd::F_PEDAL)) ? obd::val[obd::F_PEDAL] >= 70 : (obd::fresh(obd::F_THR, obd::ageFor(obd::F_THR)) && obd::val[obd::F_THR] >= 70);
    if (g >= 1 && g <= 7 && wot && !gDemo && hasADS && gPsiFilt > gearPk[g - 1])
      gearPk[g - 1] = gPsiFilt;
    if (gearEst.dirty && flashCalm()) {
      gearx::Cal c;
      portENTER_CRITICAL(&gearMux); c = gearEst.cal; gearEst.dirty = false; portEXIT_CRITICAL(&gearMux);
      prefs.putBytes("gearcal", &c, sizeof(c));
    }
  }
  if (imuCalPending || imuEvent) {
    imu::Cal c;
    portENTER_CRITICAL(&imuMux); c = imuCalOut; portEXIT_CRITICAL(&imuMux);
    if (imuEvent) {        // with the directions in the sensor's own axes (which way the chip sits)
      Serial.printf("[imu] %s: up %+.2f %+.2f %+.2f, forward %+.2f %+.2f %+.2f (%s), %d stops\n",
                    (const char *)imuEvent, c.up.x, c.up.y, c.up.z, c.fwd.x, c.fwd.y, c.fwd.z,
                    c.fwdSrc == 2 ? "learned" : c.fwdSrc == 1 ? "assumed" : "unknown: the assumed axis is vertical", c.upN);
      imuEvent = nullptr;
    }
    if (imuCalPending && flashCalm()) { imuCalPending = false; prefs.putBytes("imucal", &c, sizeof(c)); }   // forward is learned while accelerating: saved after
  }
  if (obd::addrLearned) { obd::addrLearned = false; prefs.putString("obdaddr2", obd::savedAddr); Serial.printf("[obd] remembered adapter %s (%s)\n", obd::devName, obd::devAddr); }

  { static uint32_t lastWatch = 0; if (now - lastWatch >= 50) { lastWatch = now; watchStep(now); perfUi(now); catCheck(now); healthUi(now); } }
  STAGE(4);
  // ---- logging at S.logHz
  static uint32_t lastLogSample = 0;
  if (S.logOn && !web::updating && now - lastLogSample >= 1000u / S.logHz) {
    lastLogSample = now;
    using namespace obd;
    bool rpmOk = fresh(F_RPM);
    // No OBD: idle vacuum on the sender means the engine runs. Not -13.5 psi or below: that's a
    // sender with no power or signal (0 V) in the first moments before the fault check catches it.
    bool engineOn = !S.logEngineOnly || (rpmOk ? val[F_RPM] > 300 :
                    (hasADS && !gDemo && !gSensorFault && now > 3000 && gPsiFilt < -3.0f && gPsiFilt > -13.5f));
    char line[480], f[27][12];
    auto fv = [&](int i, int fld, int dec) { if (fresh(fld, ageFor(fld))) dtostrf(val[fld], 0, dec, f[i]); else f[i][0] = 0; };
    // the same windows as the pages (see AGE_* in obd_ble.h): a held value, not a gap, between readings
    fv(0, F_RPM, 0); fv(1, F_THR, 0); fv(2, cacFromIat ? F_IAT : F_CAC, 0); fv(3, F_CLT, 0); fv(4, F_OIL, 0);
    fv(5, F_BAT, 2); fv(6, F_TIM, 1); fv(7, F_MAP, 1); fv(8, F_BARO, 0); fv(9, F_KMH, 0); fv(10, F_MAPCMD, 1);
    fv(11, F_KNOCK, 2); fv(12, F_PEDAL, 0); fv(13, F_BYPASS, 1);
    fv(14, F_HPFP, 0); fv(15, F_HPFPSET, 0); fv(16, F_LPFP, 1); fv(17, F_AFR1, 2); fv(18, F_AFR2, 2); fv(19, F_AFRCMD, 2);
    fv(20, F_CAT1, 0); fv(21, F_CAT2, 0); fv(22, F_AMB, 0); fv(23, F_PRESC, 0); fv(24, F_ICPUMP, 0);
    { float idc = idcNow(); if (gr::has(idc)) dtostrf(idc, 0, 1, f[25]); else f[25][0] = 0; }
    char gl[24] = ",";                             // lat_g, lon_g (two empty fields until the G-meter is calibrated)
    if (hasIMU && gimu.state() == imu::READY) snprintf(gl, sizeof(gl), "%.2f,%.2f", gimu.lat, gimu.lon);
    char kc[4] = ""; if (f[11][0]) snprintf(kc, sizeof(kc), "%d", (int)knockCyl);
    char gr_[4] = ""; if (fresh(F_KMH, 1500)) snprintf(gr_, sizeof(gr_), "%d", gearEst.gear);
    char tm[2] = {transMode(), 0};                 // gearbox program D / S / M (empty = not known)
    static const char *DMODE_NAME[] = {"", "comfort", "auto", "dynamic"};   // the column stays (empty) so logs line up
    char bk[4] = ""; { int b = brakeOn(); if (b >= 0) snprintf(bk, sizeof(bk), "%d", b); }   // brake pedal 1 / 0
    char o2[24], oa[10] = "", ob[10] = "";            // rear O2 bank 1, bank 2 (V)
    if (fresh(F_O2B1S2, ageFor(F_O2B1S2))) dtostrf(val[F_O2B1S2], 0, 3, oa);
    if (fresh(F_O2B2S2, ageFor(F_O2B2S2))) dtostrf(val[F_O2B2S2], 0, 3, ob);
    snprintf(o2, sizeof(o2), "%s,%s", oa, ob);
    char gp[12] = ",";                             // gearbox raw: sport flag 221035, shift program 221026 (decoding D / S / M)
    if (fresh(F_GSPORT, ageFor(F_GSPORT)) && fresh(F_GPROG, ageFor(F_GPROG))) snprintf(gp, sizeof(gp), "%d,%d", (int)val[F_GSPORT], (int)val[F_GPROG]);
    char rt[8] = "";                               // OBD answers per second (empty without the adapter)
    if (obd::state == obd::S_RUN) dtostrf(obd::rate, 0, 1, rt);
    char tt[40] = ",,,,";                          // gearbox 1259 / 125A / 125B, sport diff 1306 / 1307 (degF, empty = none)
    { char a[5][8]; const int fl[5] = {F_GBT1, F_GBT2, F_GBT3, F_DIFT1, F_DIFT2};
      for (int i = 0; i < 5; i++) { if (fresh(fl[i], ageFor(fl[i]))) snprintf(a[i], sizeof(a[i]), "%.0f", (float)val[fl[i]]); else a[i][0] = 0; }
      snprintf(tt, sizeof(tt), "%s,%s,%s,%s,%s", a[0], a[1], a[2], a[3], a[4]); }
    snprintf(line, sizeof(line), "%lu,%.2f,%.2f,%.3f,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%d,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n",
             (unsigned long)now, (float)gPsiFilt, (float)gPeakPsi, gVsensPin * SIG_DIV_RATIO,
             f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], f[8], f[9], page, gl, f[10], f[11], kc, gr_, f[12], f[13],
             f[14], f[15], f[16], f[17], f[18], f[19], f[20], f[21], f[22], f[23], f[24], f[25],
             isDemo() ? "demo" : gBoostObd ? "obd" : "sender", tm, DMODE_NAME[driveMode()], tt, o2, bk, rt, gp);
    // flash writes wait while it's in boost or on the throttle (they freeze the display briefly)
    bool boosting = (boostValid() && gPsiFilt > -2.0f) ||
                    (fresh(F_PEDAL, ageFor(F_PEDAL)) ? val[F_PEDAL] > 35 : (fresh(F_THR, ageFor(F_THR)) && val[F_THR] > 35));
    // standing still: the log is saved every few seconds (not while staged for a launch: no flash pause then)
    logx::parked = fresh(F_KMH, 1500) && val[F_KMH] < 1 && !gStaged;
    logx::sample(engineOn, boosting, line);
  } else if (!S.logOn) logx::off();
  if (web::reqLogStop) { web::reqLogStop = false; logx::stop(true); }

  if (gZeroDone) {
    gZeroDone = false;
    char v[12], m[32];
    snprintf(m, sizeof(m), "ZERO %s V%s", gr::fmt(v, gZero, 2), gZeroFromBoot ? "" : " (SAVED)");
    toast(m);
  }

  STAGE(5);
  // ---- headlight sense / dimming
#if LIGHT_SENSE_ENABLED
  if (now - lastLight >= 100) {
    lastLight = now;
    uint32_t mv = 0;
    for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(PIN_LIGHT_SENSE);   // 8 quick reads: evens out ripple
    float v = mv / 8000.0f * LIGHT_DIV_RATIO;
    // Headlights on / off: decided from the raw reading (two readings in a row, 0.2 s) and applied
    // at once: the theme and the brightness switch together, in one frame. Smoothing only for the
    // dimmer wheel while the lights stay on, so it doesn't step.
    static int flipN = 0;
    bool rawOn = lightsOn ? (v > LIGHT_OFF_V) : (v > LIGHT_ON_V);
    flipN = rawOn != lightsOn ? flipN + 1 : 0;
    bool on = lightsOn;
    if (flipN >= 2) { on = rawOn; flipN = 0; lightV = v; }
    else lightV = lightV + 0.3f * (v - lightV);
    uint8_t target = S.brightDay;
    if (on) {   // follow the dash dimmer: DIM_MIN_V..DIM_MAX_V -> S.brightDimMin..S.brightDimMax
      float t = constrain((lightV - DIM_MIN_V) / (DIM_MAX_V - DIM_MIN_V), 0.0f, 1.0f);
      target = S.brightDimMin + (uint8_t)(t * (S.brightDimMax - S.brightDimMin) + 0.5f);
    }
    if (on != lightsOn) Serial.printf("[light] %.1f V -> headlights %s\n", lightV, on ? "ON" : "OFF");
    bool flipped = on != lightsOn;
    lightsOn = on;
    if (target != brightness && (flipped || abs((int)target - (int)brightness) >= 3 || target == S.brightDay)) {
      brightness = target;
      brightReq = brightness;                     // the push task applies it between frames
    }
  }
#else
  (void)lastLight;
#endif

  STAGE(6);
  // ---- render
  if (now - lastFrame >= FRAME_MS) {
    lastFrame = now;
    if (PIXEL_SHIFT_S && now - lastShift >= PIXEL_SHIFT_S * 1000UL) { lastShift = now; shiftIdx = (shiftIdx + 1) % 9; }
    uint32_t sk = skipped;
    renderFrame(now);
    if (skipped == sk) frames++;                  // count only frames that were actually drawn
    else if (now > readyAt + 300 && dayPoolHasRoom()) {   // screen idle after the sweep: draw one more page's face
      for (int p = 0; p < gr::P_COUNT; p++) if (!poolFace(false, p, false)) { poolFace(false, p); break; }
    }
  }

  // ---- once-a-second rates
  if (now - lastSec >= 1000) {
    lastSec = now;
    fps = frames; frames = 0;
    loopGapShown = loopGapMax; loopGapMax = 0;
    if (now > 15000 && loopGapShown > rtcGapMax) rtcGapMax = loopGapShown;   // after start-up
    skippedShown = skipped; skipped = 0;
    uint32_t s = gSamples; sps = s - lastSamples; lastSamples = s;
    uint32_t h = ESP.getFreeHeap(); if (h < minHeap) minHeap = h;
    rtcUpMin = now / 60000; rtcMinHeapK = ESP.getMinFreeHeap() / 1024; rtcMinPsramK = ESP.getMinFreePsram() / 1024;
  }

  // ---- serial log: vsig = filtered signal, pp = min..max spread since the last line (noise),
  //      draw/push = ms per frame, rows = % of the screen sent per frame
  STAGE(7);
  if (now - lastLog >= (fastLog ? 20u : (uint32_t)LOG_MS) && Serial.availableForWrite() >= 400) {
    lastLog = now;
    float vsig = gVsensPin * SIG_DIV_RATIO, vsup = gVsupPin * SUP_DIV_RATIO;
    float pp = gWinMax > gWinMin ? gWinMax - gWinMin : 0;
    gWinReset = true;
    Serial.printf("t=%lu %s pg=%d vsig=%.3fV pp=%.3fV vsup=%.2fV zero=%.3f%s psi=%.2f raw=%.2f peak=%.2f%s "
                  "light=%.2fV %s br=%u fps=%lu gap=%lums skip=%lu draw=%.1fms push=%.1fms wait=%.1fms sent=%.1f%% sps=%lu heap=%luK min=%luK "
                  "g=%.2f,%.2f/%d obd=%.1f/s log=%luK/%lums\n",
                  (unsigned long)now, isDemo() ? "DEMO" : gBoostObd ? "OBD " : "SENS", page,
                  vsig, pp, vsup, (float)gZero, ZERO_UNIT, (float)gPsiFilt, (float)gPsiRaw, (float)gPeakPsi,
                  gSensorFault ? " FAULT" : "", lightV, lightsOn ? "ON" : "off", brightness,
                  (unsigned long)fps, (unsigned long)loopGapShown, (unsigned long)skippedShown, drawMs, (float)pushMs, waitMs, (float)rowsPct, (unsigned long)sps,
                  (unsigned long)(ESP.getFreeHeap() / 1024), (unsigned long)(ESP.getMinFreeHeap() / 1024),
                  gimu.lat, gimu.lon, hasIMU ? gimu.state() : -1, obd::rate,
                  (unsigned long)(logx::curBytes / 1024), (unsigned long)logx::worstWriteMs);
  }
}
