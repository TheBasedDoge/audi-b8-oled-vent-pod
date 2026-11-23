# OBD identifiers on this car

Every identifier the pod uses or the scans turned up on the 2016 S5 3.0T (B8.5, 0B5 S tronic, sport differential), with its scaling and how sure we are. "Confirmed" means it was checked on this car against something known (the dash, a mode change, a reading that makes physical sense over a drive). "Candidate" means it looks right but hasn't been checked yet. "Ruled out" values are listed so nobody chases them again.

Addresses: the pod asks on the first address and the module answers on the second. Mode 22 requests are `22` + the 4-digit identifier; answers are `62` + the identifier + the data. In the console, put the address first to ask another module: `7E1:221035`, `71D:22170E`.

**Status key:** ✅ confirmed and used by the pod · 🟡 candidate, shown on the OBD tab to check · ⚪ candidate, not used · ❌ ruled out

## Modules that answer

| Module | Asks on → answers on | Name it gives (`22F197`) | Scanned |
|---|---|---|---|
| Engine | 7E0 → 7E8 | 3.0 | 1000–2FFF |
| Gearbox (S tronic 0B5) | 7E1 → 7E9 | 0B5 | 1000–2FFF |
| Steering (electromechanical) | 712 → 77C | RCevo | 1000–2FFF |
| Airbag | 715 → 77F | AirbagVW-AU10 | not scanned |
| Steering angle sensor | 71C → 786 | SAS-GEN 2 | 0100–2FFF: no angle |
| Sport differential | 71D → 787 | 4 Wheel Drive | 1000–2FFF |
| Electronic parking brake | 752 → 7BC | EPB-4 | 1000–2FFF |

ABS, the gateway, the instrument cluster and the MMI don't answer, most likely because they use VW's older TP2.0 protocol, which an ELM327 can't speak. Found with the OBD tab's **Find modules** (a `3E00` tester-present to 700–795 and 7E0–7E7).

Modules other than 7E0–7E7 answer on their address + 0x6A. The pod sets `ATSH` (send to), `ATCRA` (listen on), and for long answers `ATFCSH` / `ATFCSD300000` / `ATFCSM1` (flow control), then goes back to the engine (`ATSH7E0`, `ATCRA`, `ATFCSM0`). Some modules answer `7F 22 78` ("busy, answer coming") first; the real answer follows in the same reply.

Every UDS module also gives `22F197` (system name), `22F187` (part number) and `22F190` (VIN).

## Engine (7E0)

### Standard OBD (Mode 01)

| PID | What | Scaling | Status |
|---|---|---|---|
| `010C` | RPM | (A·256+B) / 4 | ✅ every cycle, paired with speed (`010C0D`) |
| `010D` | Road speed | A km/h | ✅ |
| `0111` | Throttle plate | A × 100/255 % | ✅ (says little on the 3.0T: nearly open whenever the supercharger works; use the pedal) |
| `0105` | Coolant | A − 40 °C | ✅ |
| `0177` | Intake after the intercooler | standard charge-air-cooler PID | ✅ |
| `010F` | Intake air | A − 40 °C | ✅ fallback if `0177` is missing |
| `010E` | Ignition timing | A / 2 − 64 ° | ✅ |
| `0170` | Boost: actual and commanded manifold pressure | 1/32 kPa per count, absolute | ✅ (this car has no `010B`) |
| `0133` | Barometer | A kPa | ✅ |
| `0134` / `0138` | Wideband lambda, bank 1 / bank 2 | × 14.7 = AFR | ✅ |
| `0144` | Commanded lambda | × 14.7 = AFR | ✅ |
| `0146` | Outside air | A − 40 °C | ✅ |
| `013C` / `013D` | Catalyst temperature, bank 1 / bank 2 | (A·256+B) / 10 − 40 °C | ✅ (the ECU's estimate, not a sensor: no use for cat health) |
| `0113` | O2 sensors present | bit mask | ✅ 0x33: front and rear on each bank |
| `0115` / `0119` | Rear O2 (after the cats), bank 1 / bank 2 | A × 0.005 V (B = trim, FF = not used) | ✅ asked together (`011519`) once a second; logged as `o2_b1s2_V`, `o2_b2s2_V` |
| `0101` / `0141` | Monitor status, since codes cleared / this drive | byte C = tests supported, byte D = not yet complete (bit 0 catalyst, 2 evap, 5 O2 sensor, 6 O2 heater) | ✅ read with the self-tests |
| `ATRV` | Battery at the OBD port | adapter command | ✅ |
| `015C` | Oil temperature | | ❌ not on this car (use `2220A1`) |
| `010B` | Manifold pressure | | ❌ not on this car (use `0170`) |
| `01A4` | Gear | | ❌ not on this car (the pod calculates it) |

### Mode 06: the ECU's own self-tests

`0600` / `0620` list the monitors (this ECU: `01 02 05 06` O2 sensors, `21 22` catalysts, `35 36` valve timing, `3C 3D` evap). Each answers `46` + records of 9 bytes: monitor, test ID, unit ID, value, min, max. The OBD tab's **Read self-tests** reads them all and shows pass / fail and where each value sits against its limits.

| Request | What | Seen | Status |
|---|---|---|---|
| `0621` / `0622` | Catalyst, bank 1 / bank 2: test 84, unit 05 | 0x0100 of 0x0000–0xE600 on both banks (0.4 % of the limit) | ✅ read every 5 minutes; a new result is saved for the Health tab's catalyst trend. Both banks identical on the first read: probably not run since the codes were cleared (check `0101`) |

### Mode 22, confirmed

Source: Audizine "Torque Non-Standard PIDs" (from a 2015–16 SQ5/Q5 3.0T), each one checked on this car.

| Identifier | What | Scaling | Status |
|---|---|---|---|
| `221070` | Accelerator pedal | (A·256+B) × 0.0977 % | ✅ |
| `222982` | Supercharger bypass valve position | (A·256+B) × 0.0256 ° | ✅ |
| `222029` | ECU boost target (manifold, absolute) | (A·256+B) × 0.0829 hPa | ✅ about 18 psi at full throttle on this tune |
| `22130E` | High-pressure fuel pump, actual | (A·256+B) × 2.6534 hPa | ✅ |
| `22130F` | High-pressure fuel pump, target | (A·256+B) × 2.6534 hPa | ✅ |
| `221321` | Low-pressure fuel pump | (A·256+B) × 2.6534 hPa | ✅ |
| `2213A5` | Injector pulse width (one cylinder) | (A·256+B) × 0.004 ms | ✅ (turned into duty with RPM) |
| `2220A1` | Oil temperature | A − 40 °C | ✅ reads 6–10 °F above the dash; *Oil temp offset* in Settings |
| `221003` | Intake before the supercharger | A × 0.75 − 48 °C | ✅ |
| `2211D6` | Intercooler pump | (A·256+B) × 0.0015 % | ✅ (tops out at 87 %: probably full speed, scaling a little low) |
| `2213F2`–`2213F7` | Knock retard, cylinders 1–6 | one byte each: A × 0.375 − 48 °, so 0x80 = none | ✅ (held by the ECU and eased off slowly; cylinder 6 shows −5.6 ° in 1st-gear tip-ins, likely driveline noise) |
| `22121C` | Odometer | 4 bytes, km | ✅ 118,636 km = 73,717 mi, matched the dash |
| `2216CE` / `2216CF` | **Brake pedal**, the two switches (brake light and brake pedal) | 01 pressed, 00 released | ✅ checked pressed / released / pressed; read once a second, logged (`brake`), on the OBD tab; launch detection needs it pressed when known |
| `221602` | a status word with the brake in it | bit 0x8000 set while the brake is pressed (00068011 / 00060011) | ⚪ |

Asked up to three to a request where the ECU allows it (`2210702982`, `22130E130F1321` and so on); the boost target `222029` is asked on its own so it fits one frame and comes back quickly.

### Mode 22, candidates and ruled out

| Identifier | Seen | Best guess | Status |
|---|---|---|---|
| `2213A3`–`2213A8` | six values around 0x009E–0x00AD at idle | injector pulse width for all six cylinders (`13A5` is one of them) | ⚪ |
| `221456`–`221459` | 0x0400 / 0x0401 | lambda per bank, actual and target (÷ 1024 = 1.000) | ⚪ (AFR already comes from `0134` / `0138`) |
| `222981` | tracks `2982` closely | bypass valve target | ⚪ |
| `22138A`–`22138F` (and `22137A`) | six values 0x55–0x61 | something per cylinder | ⚪ |
| `2213F8`–`2213FD` | six values 0x1930–0x2490, moving | per-cylinder knock sensor signal or noise level | ⚪ |
| `221294`–`22129B` | eight values around 0x2160–0x2C30 | cam positions (four cams, actual and target?) | ⚪ |
| `221001`, `1004`, `1005`, `1007`, `1008` | 0x95 → 0x9C, rising while parked | temperatures warming up (same as each other) | ⚪ |
| `221006` | same as `1003` | duplicate of intake before the supercharger | ⚪ |
| `2215D4` | 0x000166FB → 0x000166FC (91,899) | a distance counter, but not the odometer | ⚪ |
| `221090` | 32-bit, counting | an operating-time counter | ⚪ |
| `2216DF` | ASCII "S85L2010 CBK" | software / calibration name | ⚪ |
| `2211F4`–`2211FD` | ASCII "no_error" | fault-memory text | ⚪ |
| `221188`, `2214EC` | flap by one count, even in the same mode | sensors | ❌ (not drive select) |
| `2216D6` / `16D7`, `221456`–`59`, `22150D` | changed once with the mode | changed again with no mode change | ❌ (not drive select) |

Nothing in the engine (1000–2FFF) follows drive select.

## Gearbox, S tronic 0B5 (7E1)

| Identifier | What | Scaling | Status |
|---|---|---|---|
| `221035` | Sport / manual flag | 00 = D, 01 = S or manual | ✅ |
| `221026` | Shift program | 06 = S, 07 = D or manual; another value in S with adaptive cruise on (raw value to confirm) | ✅ with `1035`: D / S / M (Dynamic shows as S). Only 07 counts as manual: anything else with the sport flag is S |
| `221259` / `22125A` / `22125B` | Three gearbox temperatures | A − 40 °C | ✅ temperatures. Over two drives (log_0017 / 0018): **`1259` warms fastest and swings with load** (97 → 194 °F in 21 min, rising and falling a few degrees around pulls): the **clutch oil**, confirmed by two standing launches in log_0038 (+38 °F and +30 °F; the other two 2–3 °F). `125A` and `125B` climb smoothly, `125B` about 9 °F behind `125A`: gearbox oil and the mechatronic unit (order not settled). Logged as `gbx_1259_F` … |
| `222104` | same as `125B` | A − 40 °C | ⚪ duplicate |
| `221162` | Odometer | 4 bytes, km (same as the engine's `121C`) | ⚪ |
| `221200` | 32-bit counter | gearbox running time in hundredths of a second (about 1,415 h) | ⚪ |
| `222102` | 0x0B1C = 2844 | supply voltage if × 0.005 (14.2 V) | ⚪ |
| `2210B4` | rose from 20 to 160 with spirited driving and manual | the gearbox's driving-style rating | ⚪ (not a mode) |
| `2210D4`, `2211A0` | 0x0B05 | "0B5", the gearbox type | ⚪ |
| `2211A2`, `2212FC` | ASCII "R28N1058", "03  SB8__30_" | software / part names | ⚪ |
| `222002` | 0001 / 0001 / 0003 (one reading per mode) | unknown | ⚪ |
| `2210D9` | 00 / 01 / 00 (one reading per mode) | unknown | ⚪ |
| `2211BF`, `2211C0`, `2211C1`, `2211C2` | went up between readings in the order they were taken | timers | ❌ (not drive select) |
| `22126A` | moved between readings in the same mode | a sensor | ❌ |

## Sport differential (71D → 787)

| Identifier | What | Scaling | Status |
|---|---|---|---|
| `22170E` | **Drive select** (as the sport differential has it) | 01 Comfort, 02 Auto, 03 Dynamic | ✅ checked in every mode; Individual reads its own sport differential setting (03 on this car). No longer polled (each read cost a switch to the diff and back for a pop-up); console only |
| `22282A` | same as `170E` | 01 / 02 / 03 | ⚪ duplicate |
| `221306` / `221307` | Sport differential temperatures | A − 40 °C | ✅ **`1307` is the diff oil**: 63 → 120 °F over two drives, steadily. `1306` hardly moves (64 → 81 °F): the control unit. Logged as `diff_1306_F`, `diff_1307_F` |
| `222B33`, `2B35`, `2B37`, `2B39` | values around 0x8000 that drift a little while parked | signed with a 0x8000 offset; left / right clutch torque or current? | ⚪ to check while driving |
| `222002`, `222B21` | wobble by a few counts | sensors | ⚪ |
| `222BBA` | ASCII "4OS56STMU779" | serial number | ⚪ |

## Steering, electromechanical (712 → 77C)

| Identifier | What | Scaling | Status |
|---|---|---|---|
| `222BC1` | **Odometer and the car's clock** | 4 bytes km, then year, month, day, second, minute, hour (`1A 0A 14 37 14 12` = 2026-10-20 18:20:55), the car's local time | ✅ the pod sets its clock from it (see ENGINEERING.md, *The car's clock*) |
| `222BC5` | words that read ±375 in Dynamic, ±461 in Comfort; also move with the wheel | a dynamic-steering setting, and steering torque? | ⚪ |
| `222BE2`, `222BC3` | change with the wheel, but not evenly either side of centre | steering torque or motor current | ⚪ (not the steering angle) |
| `222BBF` | drifted steadily over a few minutes, whatever the wheel did | a temperature or voltage settling | ⚪ |
| last bytes of `2BC3` and `2BC5` | went up by about 3,400–3,800 between readings | timers | ❌ |

## Electronic parking brake (752 → 7BC)

| Identifier | What | Scaling | Status |
|---|---|---|---|
| `222B78` | **Parking brake** | 0x33 released, 0x22 applied | ✅ checked twice each way; read every 3 s, on the OBD tab, and launch detection ignores revving at a standstill while it's applied |
| `222BB6` | 0000 released, 0044 applied | clamping force? | ⚪ |
| `222BB4` | 0025 on the first reading, then 0022 whatever the brake did | unknown | ❌ |

## Steering angle sensor (71C → 786)

| Identifier | Seen | Status |
|---|---|---|
| `221880` | 0x1105, the same at centre, left and right | ❌ fixed (a version number?) |
| `222A30` | 0x1C05F8, the same at centre, left and right | ❌ fixed (not the angle) |

| `220100`, `220102`, `220103`, `22092D` | 00 at centre, left and right | ❌ fixed |
| `220407` | 000100020002 at centre, left and right | ❌ fixed (status / configuration?) |
| `22040F` | 000100000000 at centre, left and right | ❌ fixed |

Scanned 1000–2FFF and 0100–0FFF with the wheel centred and a quarter turn each way: nothing follows the wheel. **The steering angle isn't offered as a diagnostic reading**; the sensor sends it to the ABS on the chassis network, which the OBD port doesn't reach.

## Still to find or check

- [x] Cold start: gearbox `221259` / `125A` / `125B` and sport differential `221306` / `1307` all read as temperatures (near the outside air cold, about 100 °C / 40 °C after a drive).
- [x] Which gearbox temperature is the clutch oil: **`1259`, confirmed** by two standing launches in log_0038: +38 °F (201 → 239) and +30 °F (216 → 246), while `125A` / `125B` moved 2–3 °F. Cruising about 200 °F, never above 226 °F in hard driving; back near 220 °F about 45 s after a launch.
- [x] Steering angle: not available (71C 0100–2FFF has nothing that moves with the wheel).
- [x] Catalyst health: Mode 06 catalyst results and the rear O2 sensors are there (see above). Next: `7E0:0101 0141` after a steady cruise, to see the catalyst test complete and real results come in.
- [x] Parking brake (752): `222B78`, 33 released / 22 applied.
- [ ] Sport differential clutches (`222B33`–`2B39`): read them while driving through a corner.
