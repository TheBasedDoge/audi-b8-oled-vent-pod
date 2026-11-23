// Desktop unit test for firmware/S5GaugePod_Test/obd_parse.h
// build: g++ -std=c++17 -O1 tools/obd_test.cpp -o obd_test && ./obd_test
#include <stdio.h>
#include <math.h>
#include "../firmware/S5GaugePod_Test/obd_parse.h"
using namespace obdp;

static int fails = 0;
static void expect(bool ok, const char *what) { if (!ok) { printf("FAIL: %s\n", what); fails++; } }
static bool near(float a, float b) { return fabsf(a - b) < 0.05f; }

int main() {
  PidVal v[8];
  // single PID
  int n = parseMode01("410C1AF8\r\r>", v, 8);
  expect(n == 1 && v[0].pid == 0x0C && near(value(v[0]), 1726.0f), "rpm 410C1AF8 = 1726");
  // two PIDs in one line (010C11)
  n = parseMode01("410C0BB81180\r", v, 8);
  expect(n == 2 && near(value(v[0]), 750) && v[1].pid == 0x11 && near(value(v[1]), 50.2f), "rpm + throttle in one line");
  // with spaces and SEARCHING... first
  n = parseMode01("SEARCHING...\r41 05 7B\r\r", v, 8);
  expect(n == 1 && v[0].pid == 0x05 && near(value(v[0]), 181.4f), "coolant 0x7B = 83 C = 181.4 F");
  // two ECUs answering: first answer wins
  n = parseMode01("410E8C\r410E90\r", v, 8);
  expect(n == 1 && near(value(v[0]), 6.0f), "timing 0x8C = 6 deg, duplicate ignored");
  // charge air cooler temp, 5 bytes
  n = parseMode01("4177015A00000000\r", v, 8);
  expect(n == 1 && v[0].pid == 0x77 && near(value(v[0]), 122.0f), "CACT byte B 0x5A = 50 C = 122 F");
  // supported bitmap
  n = parseMode01("4100BE3FA813\r", v, 8);
  uint32_t m[4] = {0, 0, 0, 0};
  if (n == 1) m[0] = (uint32_t)v[0].b[0] << 24 | v[0].b[1] << 16 | v[0].b[2] << 8 | v[0].b[3];
  expect(supported(m, 0x0C) && supported(m, 0x05) && !supported(m, 0x02) && supported(m, 0x20), "supported PIDs from 4100BE3FA813");
  // errors
  expect(classify("NO DATA\r\r") == NODATA, "NO DATA");
  expect(classify("CAN ERROR\r") == ERROR, "CAN ERROR");
  expect(classify("?\r") == ERROR, "?");
  expect(classify("410C1AF8") == OK, "ok");
  // garbage / truncated
  n = parseMode01("410C1A\r", v, 8);
  expect(n == 0, "truncated rpm ignored");
  n = parseMode01("STOPPED\r", v, 8);
  expect(n == 0, "text only");
  // volts
  expect(near(parseVolts("14.2V\r"), 14.2f), "ATRV 14.2V");
  expect(near(parseVolts("12.65V"), 12.65f), "ATRV 12.65V");
  expect(parseVolts("?") == 0, "ATRV junk");
  // MAP / baro
  n = parseMode01("410B26\r", v, 8);
  expect(n == 1 && near(value(v[0]), 38), "MAP 38 kPa");
  // PID 70 (boost) as the adapter prints a multi-frame reply: byte count, then numbered frames.
  // A=0x03 (commanded A + actual A), commanded A = 0x0BB8/32 = 93.75 kPa, actual A = 0x0C80/32 = 100 kPa
  float k = 0;
  n = parseMode01("00C\r0:4170030BB80C\r1:800000000000001F\r\r>", v, 8);
  expect(n == 1 && v[0].pid == 0x70 && boostActual(v[0], k) && near(k, 100.0f), "0170 actual 100 kPa (multi-frame)");
  expect(n == 1 && boostCommand(v[0], k) && near(k, 93.75f), "0170 commanded 93.75 kPa");
  // the same with spaces on, and a padding byte past the count
  n = parseMode01("00C \r0: 41 70 03 0B B8 0C \r1: 80 00 00 00 00 00 1F AA\r\r", v, 8);
  expect(n == 1 && boostActual(v[0], k) && near(k, 100.0f), "0170 with spaces");
  // Audizine formula (D*256+E)/32 on a boost reading: 0x1A00/32 = 208 kPa abs (~15.5 psi at sea level)
  n = parseMode01("00C\r0:4170030000\r", v, 8);
  expect(n == 0, "0170 cut short: ignored");
  n = parseMode01("00C\r0:417003190A1A\r1:000000000000001F\r", v, 8);
  expect(n == 1 && boostActual(v[0], k) && near(k, 208.0f), "0170 actual 208 kPa under boost");
  // actual not supported (A bit 1 clear)
  n = parseMode01("00C\r0:417001190A1A\r1:000000000000001F\r", v, 8);
  expect(n == 1 && !boostActual(v[0], k) && boostCommand(v[0], k), "0170 without actual pressure");
  // single-frame replies still fine after a multi-frame one in the same text
  n = parseMode01("00C\r0:4170030BB80C\r1:800000000000001F\r410C1AF8\r", v, 8);
  expect(n == 2 && v[1].pid == 0x0C && near(value(v[1]), 1726.0f), "multi-frame then a normal line");
  // Mode 22 (Audi identifiers)
  uint8_t b22[4];
  expect(parseMode22("6220A182\r\r>", 0x20A1, b22, 4) == 1 && b22[0] == 0x82, "2220A1 oil: 0x82 = 90 C");
  expect(parseMode22("62 20 29 2F 1A \r", 0x2029, b22, 4) == 2 && b22[0] == 0x2F && b22[1] == 0x1A, "222029 target with spaces");
  expect(near((0x2F * 256 + 0x1A) * 0.00829f, 100.0f), "  0x2F1A * 0.0829 hPa = 100 kPa");
  expect(parseMode22("7F2231\r", 0x20A1, b22, 4) == 0, "refusal 7F2231 = no answer");
  expect(parseMode22("6220A282\r", 0x20A1, b22, 4) == 0, "a different identifier is ignored");
  expect(parseMode22("NO DATA\r", 0x20A1, b22, 4) == 0, "NO DATA");
  // pedal and bypass (2 bytes each)
  expect(parseMode22("6210700200\r", 0x1070, b22, 4) == 2 && near((b22[0] * 256 + b22[1]) * 0.0977f, 50.0f), "221070 pedal 0x0200 = 50 %");
  expect(parseMode22("6229820DBB\r", 0x2982, b22, 4) == 2 && near((b22[0] * 256 + b22[1]) * 0.0256f, 90.0f), "222982 bypass 0x0DBB = 90 deg");
  // three knock identifiers in one request, multi-frame reply (13 bytes)
  uint16_t kd[3] = {0x13F2, 0x13F3, 0x13F4}, kv[3] = {0, 0, 0};
  int kn = parseMode22Multi("00D\r0:6213F2008013\r1:F3007A13F40080\r\r>", kd, 3, 2, kv);
  expect(kn == 3 && kv[0] == 0x80 && kv[1] == 0x7A && kv[2] == 0x80, "knock 1-3 in one reply");
  expect(near(kv[0] * 0.375f - 48, 0) && near(kv[1] * 0.375f - 48, -2.25f), "  0x80 = 0 deg, 0x7A = -2.25 deg");
  expect(parseMode22Multi("7F2213\r", kd, 3, 2, kv) == 0, "grouped request refused (7F2213)");
  expect(parseMode22Multi("6213F2008013F5\r", kd, 3, 2, kv) == 1, "only the first in order is trusted");
  // the real reply from the S5 (adapter log): one byte per cylinder, 0x80 = no retard
  { uint8_t kl[3] = {1, 1, 1}; uint16_t kr[3] = {0, 0, 0};
    int n = parseMode22List("00A\r0:6213F28013F3\r1:8013F480AAAAAA\r\r>", kd, kl, 3, kr);
    expect(n == 3 && kr[0] == 0x80 && kr[1] == 0x80 && kr[2] == 0x80, "S5 knock reply: 3 cylinders, one byte each, 0x80");
    expect(fabsf(kr[0] * 0.375f - 48.0f) < 0.01f, "0x80 = 0 deg of retard"); }
  { uint8_t b[12];                          // the steering's clock + odometer (712 222BC1), as the S5 sent it
    int n = parseMode22("00D\r0:622BC10001CF\r1:881A0A14371412\r\r", 0x2BC1, b, 12);
    expect(n == 10 && b[3] == 0x88 && b[4] == 0x1A && b[9] == 0x12, "car clock reply: odometer, then 26-10-20 55 s 20 min 18 h");
    uint8_t m[4]; expect(parseMode22("7F2278\r62170E02\r", 0x170E, m, 4) == 1 && m[0] == 2, "sport diff 'busy' (7F 22 78) then the answer: Auto"); }
  { TestRec t[4];                            // the S5's catalyst test results and monitor list (Mode 06)
    int n = parseMode06("00A\r0:462184050100\r1:0000E600AAAAAA\r\r", t, 4);
    expect(n == 1 && t[0].mid == 0x21 && t[0].tid == 0x84 && t[0].uas == 0x05 && t[0].val == 0x0100 && t[0].min == 0 && t[0].max == 0xE600, "catalyst bank 1: test 84, 0x0100 of 0x0000..0xE600");
    uint32_t m = 0; expect(parseMode06Mask("4600CC000001\r", 0x00, m) && m == 0xCC000001, "0600: monitors 01 02 05 06, and 20 for the next range");
    expect(parseMode06Mask("4620C0000C19\r", 0x20, m) && (m >> 31 & 1) && (m >> 30 & 1) && (m & 1), "0620: catalysts 21 22, and 40 next");
    PidVal v[3]; int k = parseMode01("411553FF\r", v, 3);
    expect(k == 1 && v[0].pid == 0x15 && fabsf(value(v[0]) - 0.415f) < 0.001f, "rear O2 bank 1: 0x53 = 0.415 V");
    k = parseMode01("411553FF1953FF\r", v, 3);
    expect(k == 2 && v[1].pid == 0x19, "both rear O2 sensors in one request (011519)"); }
  printf(fails ? "%d FAILED\n" : "all OBD parser tests passed\n", fails);
  return fails != 0;
}
