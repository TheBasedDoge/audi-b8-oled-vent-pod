// =====================================================================
//  S5 Gauge Pod - ELM327 response parsing (plain C++, unit-tested on the desktop:
//  tools/obd_test.cpp). No Arduino calls in here.
//
//  The adapter is set to ATE0 (no echo), ATS0 (no spaces), ATH0 (no headers),
//  so a Mode 01 reply looks like "410C1AF8" or, for a two-PID request,
//  "410C1AF81150". Lines end with '\r' and the reply ends with the '>' prompt.
//  Replies longer than 7 bytes (PID 70, boost) come as several CAN frames, which the
//  adapter prints as a byte count and numbered lines:
//      00C
//      0:41700F0C800C
//      1:B2...
//  Those are joined back into one line before parsing.
// =====================================================================
#pragma once
#include <stdint.h>
#include <string.h>

namespace obdp {

// Data bytes per Mode 01 PID for the PIDs this project asks for (0 = unknown)
static int pidLen(uint8_t pid) {
  switch (pid) {
    case 0x00: case 0x20: case 0x40: case 0x60: return 4;   // supported-PID bitmaps
    case 0x0C: return 2;                                     // RPM
    case 0x05: case 0x0B: case 0x0D: case 0x0E: case 0x0F:
    case 0x11: case 0x33: case 0x5C: return 1;
    case 0x42: return 2;                                     // module voltage
    case 0x77: return 5;                                     // charge air cooler temp
    case 0x70: return 10;                                    // boost pressure control (multi-frame)
    case 0x34: case 0x38: return 4;                          // wideband O2, bank 1 / bank 2 sensor 1: lambda + current
    case 0x44: return 2;                                     // commanded lambda
    case 0x46: return 1;                                     // ambient air temperature
    case 0x3C: case 0x3D: return 2;                          // catalyst temperature, bank 1 / bank 2
    case 0x15: case 0x19: return 2;                          // narrowband O2 after the cats (bank 1 / bank 2 sensor 2): volts, trim
    case 0x01: case 0x41: return 4;                          // monitor status (since codes cleared / this drive)
    default: return 0;
  }
}

static inline int hexv(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

enum Result { OK = 0, NODATA, ERROR, UNKNOWN };

// Classify a raw reply (everything up to the prompt)
static Result classify(const char *r) {
  if (strstr(r, "NO DATA")) return NODATA;
  if (strstr(r, "ERROR") || strstr(r, "UNABLE") || strstr(r, "STOPPED") || strstr(r, "?") ||
      strstr(r, "BUS INIT") || strstr(r, "BUFFER FULL") || strstr(r, "CAN ERROR")) return ERROR;
  return OK;
}

// One decoded PID value: raw data bytes A.. (up to 5)
struct PidVal { uint8_t pid; uint8_t n; uint8_t b[10]; };

// Join multi-frame replies ("00C" then "0:...", "1:...") into one line of hex; other lines are
// copied through. out gets '\r'-separated lines.
static void joinFrames(const char *reply, char *out, int outSize) {
  int o = 0;
  auto put = [&](char c) { if (o < outSize - 1) out[o++] = c; };
  int want = -1, got = 0;                          // hex digits expected / collected for a multi-frame message
  const char *p = reply;
  while (*p) {
    const char *ls = p;
    while (*p && *p != '\r' && *p != '\n') p++;
    const char *le = p;
    while (*p == '\r' || *p == '\n') p++;
    while (ls < le && *ls == ' ') ls++;
    while (le > ls && le[-1] == ' ') le--;
    int L = (int)(le - ls);
    // "00C": byte count of a multi-frame message
    if (L == 3 && hexv(ls[0]) >= 0 && hexv(ls[1]) >= 0 && hexv(ls[2]) >= 0) {
      if (want > 0) put('\r');
      want = (hexv(ls[0]) * 256 + hexv(ls[1]) * 16 + hexv(ls[2])) * 2; got = 0;
      continue;
    }
    // "0:...", "1:..." (frame number 0-F, then the data)
    if (want > 0 && L >= 2 && hexv(ls[0]) >= 0 && ls[1] == ':') {
      for (const char *c = ls + 2; c < le; c++) if (hexv(*c) >= 0 && got < want) { put(*c); got++; }
      if (got >= want) { put('\r'); want = -1; }
      continue;
    }
    if (want > 0) { put('\r'); want = -1; }           // message cut short: keep what came
    for (const char *c = ls; c < le; c++) put(*c);
    put('\r');
  }
  out[o] = 0;
}

// Parse every "41 pid data..." group in the reply. Handles several PIDs in one line,
// several ECU lines (the first valid answer for each PID wins), stray "SEARCHING..." text.
// Returns the number of values written to out.
static int parseMode01(const char *reply, PidVal *out, int maxOut) {
  int n = 0;
  char joined[320];
  joinFrames(reply, joined, sizeof(joined));
  const char *p = joined;
  while (*p) {
    // collect one line of hex digits
    char line[96]; int L = 0;
    while (*p && *p != '\r' && *p != '\n') {
      if (hexv(*p) >= 0 && L < (int)sizeof(line) - 1) line[L++] = *p;
      else if (*p != ' ') { L = 0; while (*p && *p != '\r' && *p != '\n') p++; break; }   // text line: skip it
      p++;
    }
    while (*p == '\r' || *p == '\n') p++;
    line[L] = 0;
    if (L < 4 || line[0] != '4' || line[1] != '1') continue;
    int i = 2;
    while (i + 2 <= L && n < maxOut) {
      uint8_t pid = (uint8_t)(hexv(line[i]) * 16 + hexv(line[i + 1]));
      int len = pidLen(pid);
      if (len == 0 || i + 2 + len * 2 > L) break;
      bool dup = false;
      for (int k = 0; k < n; k++) if (out[k].pid == pid) dup = true;
      if (!dup) {
        out[n].pid = pid; out[n].n = (uint8_t)len;
        for (int b = 0; b < len; b++) out[n].b[b] = (uint8_t)(hexv(line[i + 2 + b * 2]) * 16 + hexv(line[i + 3 + b * 2]));
        n++;
      }
      i += 2 + len * 2;
    }
  }
  return n;
}

// Engineering value for a PID (SAE J1979 formulas). Temperatures in degF, pressures kPa.
static float value(const PidVal &v) {
  const uint8_t *b = v.b;
  switch (v.pid) {
    case 0x0C: return (b[0] * 256 + b[1]) / 4.0f;            // rpm
    case 0x05: case 0x0F: case 0x5C:                          // coolant, intake, oil: A - 40 degC
      return (b[0] - 40) * 9.0f / 5.0f + 32.0f;
    case 0x77: return (b[1] - 40) * 9.0f / 5.0f + 32.0f;      // charge air cooler temp, sensor A (byte B)
    case 0x0E: return b[0] / 2.0f - 64.0f;                    // timing advance, deg
    case 0x11: return b[0] * 100.0f / 255.0f;                 // throttle %
    case 0x0B: case 0x33: return (float)b[0];                 // MAP / baro, kPa abs
    case 0x0D: return (float)b[0];                            // speed, km/h
    case 0x70: return (b[3] * 256 + b[4]) / 32.0f;            // boost: actual pressure A (D,E), kPa abs
    case 0x34: case 0x38: case 0x44:                          // lambda (A,B) * 2/65536, shown as gasoline AFR
      return (b[0] * 256 + b[1]) * (2.0f / 65536.0f) * 14.7f;
    case 0x46: return (b[0] - 40) * 9.0f / 5.0f + 32.0f;      // ambient, degF
    case 0x3C: case 0x3D: return ((b[0] * 256 + b[1]) / 10.0f - 40.0f) * 9.0f / 5.0f + 32.0f;   // catalyst, degF
    case 0x42: return (b[0] * 256 + b[1]) / 1000.0f;          // module voltage, V
    case 0x15: case 0x19: return b[0] * 0.005f;               // O2 after the cat, V
    default: return 0;
  }
}

// PID 70 (boost pressure control). Byte A says what's there: bit 0 commanded A, bit 1 actual A.
// kPa absolute (1/32 kPa per count). False if the car doesn't fill that part in.
static inline bool boostActual(const PidVal &v, float &kpa)  { if (v.pid != 0x70 || !(v.b[0] & 2)) return false; kpa = (v.b[3] * 256 + v.b[4]) / 32.0f; return true; }
static inline bool boostCommand(const PidVal &v, float &kpa) { if (v.pid != 0x70 || !(v.b[0] & 1)) return false; kpa = (v.b[1] * 256 + v.b[2]) / 32.0f; return true; }

// Mode 22 (manufacturer identifiers, e.g. "2220A1" oil temp on the Audi 3.0T ECU). The reply
// is "62" + the 4-digit identifier + data bytes ("6220A182"); a refusal is "7F22..".
// Copies the data bytes for that identifier into out; returns how many (0 = no answer).
static int parseMode22(const char *reply, uint16_t did, uint8_t *out, int maxOut) {
  char joined[320];
  joinFrames(reply, joined, sizeof(joined));
  const char *p = joined;
  while (*p) {
    char line[96]; int L = 0;
    while (*p && *p != '\r' && *p != '\n') { if (hexv(*p) >= 0 && L < (int)sizeof(line) - 1) line[L++] = *p; p++; }
    while (*p == '\r' || *p == '\n') p++;
    line[L] = 0;
    if (L < 8 || line[0] != '6' || line[1] != '2') continue;
    uint16_t d = (uint16_t)(hexv(line[2]) << 12 | hexv(line[3]) << 8 | hexv(line[4]) << 4 | hexv(line[5]));
    if (d != did) continue;
    int n = 0;
    for (int i = 6; i + 1 < L && n < maxOut; i += 2) out[n++] = (uint8_t)(hexv(line[i]) * 16 + hexv(line[i + 1]));
    return n;
  }
  return 0;
}

// Several identifiers in one Mode 22 request ("2213F213F313F4": as many as fit in one CAN frame,
// three). The reply is "62" then each identifier followed by its data, in the order asked:
// "6213F2008013F3007E13F40080" (2 data bytes each here). The reply is longer than one frame,
// so it arrives as a multi-frame message. Fills vals[i] = A*256+B for each identifier found;
// returns how many were found (0 = refused or garbled).
static int parseMode22List(const char *reply, const uint16_t *dids, const uint8_t *lens, int count, uint16_t *vals);
static inline int parseMode22Multi(const char *reply, const uint16_t *dids, int count, int dataLen, uint16_t *vals) {
  uint8_t lens[8]; for (int i = 0; i < count && i < 8; i++) lens[i] = (uint8_t)dataLen;
  return parseMode22List(reply, dids, lens, count, vals);
}
// Same, with each identifier's own data length (1 or 2 bytes): vals[i] = A, or A*256+B
static int parseMode22List(const char *reply, const uint16_t *dids, const uint8_t *lens, int count, uint16_t *vals) {
  char joined[320];
  joinFrames(reply, joined, sizeof(joined));
  const char *p = joined;
  while (*p) {
    char line[160]; int L = 0;
    while (*p && *p != '\r' && *p != '\n') { if (hexv(*p) >= 0 && L < (int)sizeof(line) - 1) line[L++] = *p; p++; }
    while (*p == '\r' || *p == '\n') p++;
    line[L] = 0;
    if (L < 6 || line[0] != '6' || line[1] != '2') continue;
    int i = 2, found = 0;
    auto byteAt = [&](int k) { return hexv(line[k]) * 16 + hexv(line[k + 1]); };
    for (int d = 0; d < count; d++) {
      int dataLen = lens[d];
      if (i + 4 + dataLen * 2 > L) break;
      uint16_t id = (uint16_t)(byteAt(i) << 8 | byteAt(i + 2));
      if (id != dids[d]) break;                     // not in the order asked: stop trusting the rest
      uint16_t v = 0;
      for (int b = 0; b < dataLen; b++) v = (uint16_t)(v << 8 | byteAt(i + 4 + b * 2));
      vals[d] = v; found++;
      i += 4 + dataLen * 2;
    }
    if (found) return found;
  }
  return 0;
}

// "ATRV" reply, e.g. "14.2V" -> 14.2 (0 if it doesn't parse)
static float parseVolts(const char *r) {
  float v = 0, div = 0;
  bool any = false;
  for (const char *p = r; *p; p++) {
    if (*p >= '0' && *p <= '9') { any = true; if (div > 0) { div *= 10; v += (*p - '0') / div; } else v = v * 10 + (*p - '0'); }
    else if (*p == '.' && any) div = 1;
    else if (any) break;
  }
  return (any && v > 5 && v < 20) ? v : 0;
}

// Supported-PID bitmap: bit for PID (base + 1 + i) is (mask >> (31 - i)) & 1
static inline bool supported(const uint32_t *masks, uint8_t pid) {
  if (pid == 0) return true;
  int range = (pid - 1) / 32, bit = (pid - 1) % 32;
  if (range > 3) return false;
  return (masks[range] >> (31 - bit)) & 1;
}

// ---------------------------------------------------------------- Mode 06 (the ECU's own self-tests)
// CAN format (ISO 15031-5): "46" then records of 9 bytes: OBD monitor ID, test ID, unit/scaling
// ID, test value, minimum, maximum (2 bytes each). "0621" on the S5: 46 21 84 05 0100 0000 E600 =
// catalyst bank 1, test 84, value 0x0100, allowed 0x0000..0xE600. A request for a range marker
// (0600, 0620 ...) answers 46 MID + 4 bytes: which monitor IDs in the next 32 are supported.
struct TestRec { uint8_t mid, tid, uas; uint16_t val, min, max; };
static int parseMode06(const char *reply, TestRec *out, int maxOut) {
  char joined[320];
  joinFrames(reply, joined, sizeof(joined));
  const char *p = joined;
  int n = 0;
  while (*p) {
    char line[200]; int L = 0;
    while (*p && *p != '\r' && *p != '\n') { if (hexv(*p) >= 0 && L < (int)sizeof(line) - 1) line[L++] = *p; p++; }
    while (*p == '\r' || *p == '\n') p++;
    line[L] = 0;
    if (L < 20 || line[0] != '4' || line[1] != '6') continue;
    auto byteAt = [&](int k) { return (uint8_t)(hexv(line[2 + k * 2]) * 16 + hexv(line[3 + k * 2])); };
    int nb = (L - 2) / 2;
    for (int k = 0; k + 9 <= nb && n < maxOut; k += 9) {
      TestRec &r = out[n++];
      r.mid = byteAt(k); r.tid = byteAt(k + 1); r.uas = byteAt(k + 2);
      r.val = (uint16_t)(byteAt(k + 3) << 8 | byteAt(k + 4));
      r.min = (uint16_t)(byteAt(k + 5) << 8 | byteAt(k + 6));
      r.max = (uint16_t)(byteAt(k + 7) << 8 | byteAt(k + 8));
    }
  }
  return n;
}
// "0600" -> which of monitor IDs base+1 .. base+32 are supported (bit 31 = base+1)
static bool parseMode06Mask(const char *reply, uint8_t base, uint32_t &mask) {
  char joined[160];
  joinFrames(reply, joined, sizeof(joined));
  for (const char *p = joined; *p; ) {
    char line[64]; int L = 0;
    while (*p && *p != '\r' && *p != '\n') { if (hexv(*p) >= 0 && L < (int)sizeof(line) - 1) line[L++] = *p; p++; }
    while (*p == '\r' || *p == '\n') p++;
    line[L] = 0;
    if (L < 12 || line[0] != '4' || line[1] != '6') continue;
    if ((uint8_t)(hexv(line[2]) * 16 + hexv(line[3])) != base) continue;
    mask = 0;
    for (int k = 0; k < 8; k++) mask = mask << 4 | (uint32_t)hexv(line[4 + k]);
    return true;
  }
  return false;
}

}  // namespace obdp
