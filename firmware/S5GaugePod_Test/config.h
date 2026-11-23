// =====================================================================
//  S5 Gauge Pod - firmware configuration
//  Board: Waveshare ESP32-S3-Touch-AMOLED-1.43 (466x466, SH8601 or CO5300)
//  Every value you may need to change lives in this file.
// =====================================================================
#pragma once

// Firmware version: shown on the boot screen, in the serial log and on the web page's Settings tab
// (Firmware update). Raise it with every build that goes into the car; ENGINEERING.md (Version history) lists what changed.
#define FW_VERSION "1.0"

// The board in the Arduino IDE must be "ESP32S3 Dev Module" (Tools > Board > esp32), with
// PSRAM: OPI PSRAM, Flash Size: 16MB, Partition Scheme: 16M Flash (3MB APP/9.9MB FATFS),
// USB CDC On Boot: Enabled. With another chip selected the build fails with confusing errors
// (SD_MMC not declared), so say it plainly instead.
#include <sdkconfig.h>
#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Wrong board selected: pick Tools > Board > esp32 > ESP32S3 Dev Module (PSRAM: OPI PSRAM, Flash Size: 16MB, Partition Scheme: 16M Flash (3MB APP/9.9MB FATFS), USB CDC On Boot: Enabled)"
#endif

// ---------------- Display (fixed by the board) ------------------------
#define PIN_LCD_CS     9
#define PIN_LCD_SCK    10
#define PIN_LCD_D0     11
#define PIN_LCD_D1     12
#define PIN_LCD_D2     13
#define PIN_LCD_D3     14
#define PIN_LCD_RST    21
#define PIN_LCD_EN     42      // AMOLED_EN net; R9 is not fitted on most boards, harmless to drive
#define LCD_W          466
#define LCD_H          466
#define LCD_ROTATION   0       // panel driver rotation: leave at 0 (the drawing code rotates instead)
#define DISPLAY_ROTATION 3     // quarter turns counter-clockwise from the board's own orientation:
                               // 0 = as the board comes, 1 = 90 deg left, 2 = upside down, 3 = 90 deg right.
                               // 3 is upright on the bench (2026-10-14: 1 came out 180 deg off)
#define LCD_QSPI_HZ    80000000 // display bus clock. 80000000 roughly halves the push time if the panel
                               // takes it; go back to 40 MHz if the image shows noise or lines
#define LCD_PANEL      1        // 1 = CO5300 (this board, 2026-10), 2 = SH8601 (early boards), 0 = detect at boot
                                // (detecting costs about 0.4 s of every start-up: the panel is reset and read)
#define CO5300_COL_OFFSET 6    // Waveshare shifts CO5300 panels by 6 columns
#define LCD_PARTIAL_COLUMNS 1  // 1 = send only the changed part of each row (much less data; clean on the CO5300, 2026-10-14);
                               // 0 = whole rows, if the panel ever shows smears or offset strips

// ---------------- Shared I2C bus (touch, IMU, RTC, ADS1115) -----------
#define PIN_I2C_SDA    47
#define PIN_I2C_SCL    48
#define I2C_HZ         400000
#define ADDR_TOUCH     0x38    // FT3168
#define ADDR_RTC       0x51    // PCF85063
#define ADDR_IMU       0x6B    // QMI8658 (0x6A on some boards)
#define ADDR_ADS1115   0x48    // ADDR pin tied to GND

// ---------------- Pressure sensor selection ---------------------------
#define SENSOR_AWE     1       // reuse the AWE sender (12 V supply, 0-5 V out)
#define SENSOR_MPX4250 2       // NXP MPX4250AP (5 V supply, ratiometric)
#define SENSOR_TYPE    SENSOR_AWE

#define ADS_CH_SENSOR  0       // A0 = sensor signal
#define ADS_CH_SUPPLY  1       // A1 = sensor supply (12 V for AWE, 5 V for MPX)
// Divider B (supply -> A1) is optional for the AWE sender: battery voltage will come
// from OBD later. 0 = not fitted, A1 is never read and SUPPLY_NOMINAL is used.
// The MPX4250AP fallback needs it (ratiometric), so it is forced on there.
#define SUPPLY_SENSE_ENABLED 0

// Divider A (signal -> A0): 68k top / 100k bottom, 100nF at the ADS pin.
// 0-5 V signal -> 0-2.98 V at the pin. The 68k also limits fault current to
// ~0.12 mA if the signal wire ever shorts to 12 V.
#define SIG_DIV_RATIO  1.68f   // (68k + 100k) / 100k

#if SENSOR_TYPE == SENSOR_AWE
// ---------------- AWE sender ------------------------------------------
// Sender harness: RED = 12 V, BLACK = ground, WHITE = signal.
// Measured 2026-10-13: 12 V supply, 1.30 V at atmosphere (key on, engine off),
// 0.55 V at idle. Best fit so far: -1..+3 bar over 0-5 V = 1.25 V/bar.
// Zero is captured automatically at key-on; only the slope needs calibrating
// (hand vacuum pump at 10 and 20 inHg, or VCDS block 436 at idle).
#define SUP_DIV_RATIO   5.7f    // Divider B (12 V -> A1): 47k / 10k, 1uF at the pin (if fitted)
#define AWE_V_PER_PSI   0.0862f // 1.25 V/bar / 14.504 psi/bar  <-- CALIBRATE
#define AWE_ZERO_DEFAULT_V 1.30f
#define AWE_ZERO_MIN_V  1.15f   // key-on reading must fall in this window (measured 1.30 V) and be steady
#define AWE_ZERO_MAX_V  1.45f   // to be accepted as atmosphere.
#define AWE_ZERO_WIDE_MIN_V 0.95f // lower limit once the ECU says the engine is stopped (then a steady reading IS the
                                // atmosphere): about 8,000 ft. 1.15 V alone refused every key-on above about 3,500 ft.
                                // A saved zero outside 0.95-1.45 is ignored.
#define ZERO_BARO_TOL_V 0.06f   // a zero taken in that wider window must also agree with the ECU's barometer to
                                // within this (about 0.7 psi): see zeroExpect() in the sketch
#define ZERO_MAX_SPREAD_V 0.03f // max min-to-max spread over the capture (rejects cranking / engine running)
// If a bench check (12 V vs 14.4 V, port open) shows the signal follows the supply, set to 1:
// the signal is then scaled by AWE_SUPPLY_REF_V / measured supply. Needs Divider B
// (SUPPLY_SENSE_ENABLED 1) or, later, the OBD battery voltage.
#define AWE_RATIOMETRIC 0
#define AWE_SUPPLY_REF_V 12.4f  // supply voltage when the slope/zero were measured
#define SUPPLY_MIN_V    9.0f    // sanity window for the A1 reading
#define SUPPLY_MAX_V    16.5f
#define SUPPLY_NOMINAL  12.4f
#else
// ---------------- MPX4250AP --------------------------------------------
// Vout = Vs * (0.004 * P - 0.04), P in kPa abs. Divider B identical to A.
#undef  SUPPLY_SENSE_ENABLED
#define SUPPLY_SENSE_ENABLED 1  // MPX is ratiometric: Divider B required
#define SUP_DIV_RATIO   1.68f
#define MPX_K1          0.004f
#define MPX_K0          0.04f
#define CAL_GAIN        1.000f
#define CAL_OFFSET_KPA  0.0f
#define BARO_DEFAULT_KPA 101.3f
#define BARO_MIN_KPA    85.0f
#define BARO_MAX_KPA    106.0f
#define SUPPLY_MIN_V    4.5f
#define SUPPLY_MAX_V    5.5f
#define SUPPLY_NOMINAL  5.00f
#endif

#define ZERO_SAMPLES    64

// ---------------- Gauge scale and screens ------------------------------
// The browser preview (docs/screen-mockup.html) runs the same drawing code
// with these defaults. If you change one here, set the matching slider there.
#define GAUGE_MIN_PSI   -14.735f // = 30 inHg vacuum (1 psi = 2.036 inHg)
#define GAUGE_MAX_PSI    30.0f   // same range as the original AWE gauge
#define GAUGE_RED_PSI    20.0f   // start of the red blocks
// Laid out like the AWE face: 30 inHg at 6 o'clock, 0 at 9 o'clock, 30 psi at 3 o'clock.
// Vacuum and boost each get their own linear segment so 0 sits exactly on the horizontal.
#define GAUGE_START_DEG  90.0f   // angle of GAUGE_MIN_PSI (0 deg = 3 o'clock, clockwise)
#define GAUGE_ZERO_DEG   180.0f  // angle of 0 psi
#define GAUGE_END_DEG    360.0f  // angle of GAUGE_MAX_PSI
#define SHOW_TELLTALE    1       // red peak needle that stays at the highest boost
#define SHOW_DIGITAL     1       // number + PEAK readout under the hub
#define SHIFT_RPM        7200    // Engine page: shift ring fills over the 1500 rpm below, flashes here
#define REDLINE_RPM      7400    // red zone on the tach (tuned car: about 7400-7500). Tach reads to 8000.
#define NIGHT_THEME      1       // 1 = switch to the black AMOLED face while the headlights are on
#define NEEDLE_ALPHA     0.035f  // needle EMA per sample at ~500 Hz: about 60 ms (higher = snappier, noisier)
#define TEXT_ALPHA       0.008f  // digital readout EMA: about 250 ms, so the digits don't flicker
#define MEDIAN_SAMPLES   5       // median of the last N samples first: drops single bad readings (loose contacts)
#define SENSOR_MIN_V     0.10f   // signal outside this window for 0.2 s = wiring fault ("CHECK SENSOR WIRING")
#define SENSOR_MAX_V     4.95f
#define START_PAGE       0       // 0 Boost, 1 Overview, 2 Temps, 3 Engine, 4 System, 5 G-force, 6 Perf, 7 Gears, 8 Fuel, 9 Heat, 10 Pull (last page is remembered)
#define PERF_OBD_LAG_MS  120     // how late OBD road speed is, ms (performance timers; also in Settings)
#define PEAK_HOLD_S      8       // peak telltales on the dials fall back this many seconds after a pull (0 = until the peak is reset)
#define WARM_OIL_F       170     // "NOT WARM" until the oil reaches this (degF, as shown, after the oil offset)
#define ALERT_POP_MS     6000    // a new warning shows as the big round pop-up this long, then as the banner
#define SPLASH_MS        900     // shortest time the S badge shows at boot (incl. the fade-in); the first
                                 // page's background is drawn behind it, then it cuts straight to the gauge
#define SPLASH_FADE_MS   400     // badge fade-in
#define PIXEL_SHIFT_S    120     // AMOLED burn-in guard: move the image 1 px every N seconds (0 = off)

// ---------------- G-meter (QMI8658 on the board) -----------------------
#define IMU_ENABLED      1
// Which sensor axis points into the dash (forward) when the pod faces the driver. Only used
// until the pod learns forward from the first straight pulls (OBD speed); it then corrects
// itself even if this is backwards. +Z = out of the back of the board.
#define IMU_FWD_X        0
#define IMU_FWD_Y        0
#define IMU_FWD_Z        1

// ---------------- Dash-light / dimmer sense --------------------------
// AWE white wire (spliced to the vent's grey/blue lighting wire)
//   -> 47k -> GPIO1 -> 10k -> GND, 10uF across the 10k (averages the PWM)
// Measured 2026-10-13 (meter average): lights off 0 V, dimmer min 5 V, max 14 V.
#define LIGHT_SENSE_ENABLED 1    // Divider C wired. On the bench with nothing on T3 the 10k holds GPIO1 at 0 V
                                 // (= headlights off, full brightness). If the screen dims with T3 open, GPIO1
                                 // isn't seeing the divider's ground: check the log's light= value, or set 0.
#define PIN_LIGHT_SENSE  1
#define LIGHT_DIV_RATIO  5.7f    // (47k + 10k) / 10k
// Measured 2026-10-14 at the divider node (10 uF averaging): 0.4 V dimmer fully down, 2.4 V fully up
// = 2.3 V and 13.7 V on the wire. (The earlier 5 V meter reading was the meter misreading the PWM.)
// All values below are wire volts; the serial log prints the same as light=.
#define LIGHT_ON_V       1.2f    // above = headlights on (dimmer fully down is ~2.3 V)
#define LIGHT_OFF_V      0.7f    // below = headlights off (hysteresis)
#define DIM_MIN_V        2.3f    // dash dimmer fully down
#define DIM_MAX_V        13.7f   // dash dimmer fully up
#define BRIGHT_DAY       225     // headlights off (was 255 until 2026-11: a little lower for the panel's life)
#define BRIGHT_DIM_MIN   25      // headlights on, dimmer fully down
#define BRIGHT_DIM_MAX   175     // headlights on, dimmer fully up (was 200 until 2026-11)

// ---------------- Wi-Fi web UI, OBD, logging (defaults; the web UI can change them) ----
#define WIFI_ENABLED     1
#define WIFI_AP_SSID     "S5-GaugePod"   // the pod's own network; open http://192.168.4.1
#define WIFI_AP_PASS     "audis5pod"     // at least 8 characters
#define OBD_ENABLED      1
#define OBD_BLE_NAMES    "VEEPEAK,OBD,VLINK,ELM"   // adapter name contains any of these
#define OBD_MIN_RSSI     -80             // first-time pick only: ignore adapters weaker than this (dBm).
                                         // In the car it's about -50 to -70; a remembered adapter is
                                         // used at any strength
#define LOG_ENABLED      1
#define LOG_ENGINE_ONLY  1               // start a log only while the engine runs
#define LOG_HZ           10              // samples per second (1-25)

// ---------------- Timing ---------------------------------------------
#define FRAME_MS         16      // ~60 fps cap (drawing ~18 ms + sending is the real limit)
#define TOUCH_POLL_MS    12      // touch has its own task now, so swipes are tracked between frames
#define TOUCH_SWAP_XY    0       // touch-to-panel mapping, applied before DISPLAY_ROTATION.
#define TOUCH_FLIP_X     0       //   Check with the [touch] lines in the serial log: top-left of the
#define TOUCH_FLIP_Y     0       //   panel (as the board comes) should read near 0,0
#define TOUCH_DEBUG      1       // log every touch down/up with coordinates
#define TOUCH_RELEASE_MS 50      // finger counts as lifted only after this long with no touch reported
                                 // (the FT3168 drops single readings mid-swipe, which split one swipe in two)
#define SWIPE_PX         60      // horizontal travel that counts as a page swipe
#define LOG_MS           100     // serial log rate (10 Hz)
#define RESET_HOLD_MS    1000    // hold this long to reset the peak (a quick tap doesn't, so a short swipe can't)
#define LONGPRESS_MS     3000    // keep holding = re-zero (engine OFF!), or re-level on the G-force page
