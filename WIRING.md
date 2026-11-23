# S5 Gauge Pod — Connection List

Companion to `docs/wiring-diagram.png` and `docs/breadboard-layout.png` (rev 0.4: Divider B dropped; battery voltage will come from OBD). Use 22–26 AWG throughout; the pod draws about 100 mA at 12 V.

## 0. Before soldering

1. **Identify the board pads with a meter** (board on USB, nothing else connected):
   - **J1-4 USB_5V:** about 5 V to GND, and continuity to the USB-C VBUS pin.
   - **J1-6 GND:** 0 Ω to the USB-C shell.
   - **J8 3V3** reads about 3.3 V; **J8 GND** is 0 Ω to J1-6.
   - The other two J8 pins are SDA and SCL. The netlist says pin 1 = SCL, pin 2 = SDA. A swap is harmless: if the I2C scan doesn't find 0x48, swap them.
2. **Set the buck to 5.00 V** from a 12 V bench supply, before it's connected to anything.
3. **Tag the sender harness.** Both AWE harnesses are red/black/white.

## 1. Harness → interface board terminals

| Terminal | From | Wire |
|---|---|---|
| T1 | Gauge harness | **Red**, switched 12 V |
| T2 | Gauge harness | **Black**, ground |
| T3 | Gauge harness | **White**, dash-light feed (0 V off, 5–14 V with the dimmer) |
| — | Gauge harness | **Green**, constant 12 V: **not used, insulate** |
| T5 | Sender harness | **Red**, sender 12 V supply |
| T6 | Sender harness | **Black**, sender ground |
| T7 | Sender harness | **White**, sender signal (0–5 V) |

## 2. Power

| From | To | Notes |
|---|---|---|
| T1 | Buck IN+ | |
| T1 | T5 | Jumper: powers the sender, as the AWE gauge did |
| T2 | Buck IN− | |
| T2 | T6 | Jumper: sender ground |
| Buck OUT+ (5.00 V) | Board **J1-4 (USB_5V)** | Leave this link removable for bench work |
| Buck OUT− | Board **J1-6 (GND)** | |

## 3. ADS1115 (Adafruit #1085)

| ADS1115 | To | Notes |
|---|---|---|
| VDD | Board **J8 3V3** | Powered at 3.3 V, never 5 V |
| GND | Board **J8 GND** | |
| SDA | Board **J8 SDA (IO47)** | |
| SCL | Board **J8 SCL (IO48)** | |
| ADDR | Leave open | Adafruit board defaults to 0x48. On a generic board, tie to GND. |
| A0 | Divider A node | |
| A1, A2, A3, ALRT | Not connected | |

## 4. Dividers (1% resistors)

All divider and capacitor grounds go to the common ground. Return Divider A's to the ADS1115 GND pin and Divider C's to J1-6. (There is no Divider B any more; the names A and C are kept so the drawings and firmware still match.)

**Divider A, sender signal → A0**
- T7 → **68 kΩ** → node A
- node A → **100 kΩ** → GND
- node A → **100 nF** → GND
- node A → ADS1115 **A0**

**Divider C, dash lights → GPIO1**
- T3 → **47 kΩ** → node C
- node C → **10 kΩ** → GND
- node C → **10 µF** → GND
- node C → board **J1-13 (GPIO1)**
- *Optional:* 3.3 V zener from node C to GND, cathode (banded end) to node C

## 5. Board — everything that connects

| Pad | Signal |
|---|---|
| J1-4 | 5 V in from buck |
| J1-6 | GND |
| J1-13 | GPIO1, light sense (node C) |
| J8 3V3 / GND / SDA / SCL | ADS1115 |

Nothing else on the board is wired. USB-C stays unplugged in the car.

## 6. Checks before connecting the board

Interface board on a 12 V bench supply (T1 = +12 V, T2 = GND), board **not** connected:

| Check | Expect |
|---|---|
| T1 to T2 resistance (supply off) | Not a short |
| Buck OUT+ | 5.00 V |
| Node A with T7 open | 0 V (100k pulls it down) |
| Node C with T3 open, then T3 on +12 V | 0 V, then about 2.1 V |
| T5 | Same as T1; T6 continuous with T2 |

## 7. Bench bring-up with serial logging

USB and the buck must never feed J1-4 at the same time. For bench work with the serial log:

1. Remove the buck OUT+ → J1-4 link. Keep every ground connected.
2. Power the board from USB. The ADS1115 runs from the board's 3V3, and the dividers from the 12 V bench supply.
3. Boot: the splash should show **ADS1115 OK**. If it says MISSING, swap SDA/SCL at J8.
4. Feed 0–5 V into T7 from a pot across the buck's 5 V output. `vsig` in the log should match a meter on T7.
5. With the AWE sender connected to T5/T6/T7 and its port open to air, set the bench supply to 12.0 V and then 14.4 V. `vsig` should read about 1.30 V at both. (`vsup` in the log just shows the nominal 12.4 V now that Divider B is gone.)
6. Feed 0, 5 and 14 V into T3 and watch the brightness change (`LIGHT_SENSE_ENABLED` is 1 by default). In the car the divider reads about 0.4 V with the dimmer fully down and 2.4 V fully up; the log's `light=` shows the wire voltage (5.7× the divider).

For the car: put the buck → J1-4 link back, unplug USB, and land the harnesses per section 1.
