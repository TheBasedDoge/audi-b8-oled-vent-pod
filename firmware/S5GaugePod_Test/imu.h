// =====================================================================
//  S5 Gauge Pod - QMI8658 motion sensor (on the Waveshare board, I2C 0x6B or 0x6A)
//
//  Accelerometer +/-4 g and gyro +/-512 deg/s, both at about 235 Hz with the chip's own
//  low-pass filter on; read 100 times a second from the sensor task (it shares the I2C bus
//  with the ADS1115, so the two take turns). The maths is in imu_cal.h.
//
//  Registers (QMI8658 datasheet): 0x00 WHO_AM_I (0x05), 0x02 CTRL1, 0x03 CTRL2 (accel range
//  and rate), 0x04 CTRL3 (gyro), 0x06 CTRL5 (low-pass filters), 0x08 CTRL7 (enable),
//  0x2E STATUS0 (bit 0 = new accel data), 0x35..0x40 accel X/Y/Z then gyro X/Y/Z, 16-bit
//  little-endian, 0x60 soft reset (0xB0).
// =====================================================================
#pragma once
#include <Wire.h>
#include "imu_cal.h"

namespace imu {

static uint8_t addr = 0;
static bool    ok = false, bigEndian = false;
static const float A_LSB = 1.0f / 8192.0f;        // +/-4 g
static const float G_LSB = 1.0f / 64.0f;          // +/-512 deg/s

static bool wr(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(addr); Wire.write(reg); Wire.write(v);
  return Wire.endTransmission() == 0;
}
static bool rd(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

// Raw sample: accel in g, gyro in deg/s (sensor axes). False if the bus read failed.
static bool read(V3 &a, V3 &w) {
  uint8_t b[12];
  if (!ok || !rd(0x35, b, 12)) return false;
  auto s16 = [&](int i) -> float {
    return (float)(int16_t)(bigEndian ? (b[i] << 8 | b[i + 1]) : (b[i + 1] << 8 | b[i]));
  };
  a = v3(s16(0), s16(2), s16(4)) * A_LSB;
  w = v3(s16(6), s16(8), s16(10)) * G_LSB;
  return true;
}

static bool begin(uint8_t address) {
  addr = address;
  uint8_t id = 0;
  if (!rd(0x00, &id, 1) || id != 0x05) { Serial.printf("[imu] no QMI8658 at 0x%02X (id 0x%02X)\n", addr, id); return false; }
  wr(0x60, 0xB0);                                  // soft reset
  delay(15);
  bool w = wr(0x02, 0x40)                          // CTRL1: register auto-increment, little-endian
        && wr(0x03, 0x15)                          // CTRL2: accel +/-4 g, ~235 Hz
        && wr(0x04, 0x55)                          // CTRL3: gyro +/-512 dps, ~235 Hz
        && wr(0x06, 0x77)                          // CTRL5: both low-pass filters on, widest setting
        && wr(0x08, 0x03);                         // CTRL7: accel + gyro on
  if (!w) { Serial.println(F("[imu] QMI8658 setup write failed")); return false; }
  ok = true;
  // wait for the first sample (STATUS0 bit 0 = new accelerometer data); the first read used to
  // come too early and see all zeros
  uint32_t t0 = millis();
  uint8_t st = 0;
  do { delay(5); } while (millis() - t0 < 200 && !(rd(0x2E, &st, 1) && (st & 1)));
  // sanity check: at rest the accelerometer reads about 1 g. If it only does with the bytes
  // swapped, the chip is in big-endian mode.
  V3 a, g; float n = 0;
  for (int i = 0; i < 10 && n < 0.05f; i++) { if (read(a, g)) n = norm(a); if (n < 0.05f) delay(5); }
  if (n < 0.6f || n > 1.5f) {
    bigEndian = true;
    float n2 = read(a, g) ? norm(a) : 0;
    if (n2 < 0.6f || n2 > 1.5f) bigEndian = false; else n = n2;
  }
  Serial.printf("[imu] QMI8658 at 0x%02X, |a| = %.2f g%s (first sample after %lu ms)\n", addr, n,
                bigEndian ? " (big-endian)" : "", (unsigned long)(millis() - t0));
  return true;
}

}  // namespace imu
