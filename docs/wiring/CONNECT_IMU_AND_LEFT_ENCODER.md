# Connect the IMU and the left encoder

Two devices left. Both need one spare Grove cable. Power the board **OFF** for
every step and unplug USB.

---

## Read this first: your cable colours

Your Grove cables are **not** in the standard Seeed order. Yours are:

| Wire | Grove connector pin | What it carries |
|---|---|---|
| **white** | pin 1 | signal A (first GPIO of the port) |
| **yellow** | pin 2 | signal B (second GPIO of the port) |
| **red** | pin 3 | 3V3 |
| **black** | pin 4 | GND |

Every tutorial online says yellow is signal 1. On this car it is white. That one
fact has already cost an afternoon; it is worth tape on the cable.

---

# Part A — Left encoder → Grove 1

The code was changed for this: left encoder phase A now expects **GP0**
(`S2_PIN_ENC_L_A 0u` in `subsystem2.h`). The right encoder already works on
Grove 2 / GP3 and does not move.

Grove 1 gives: **white = GP0**, yellow = GP1, red = 3V3, black = GND.

> Right encoder: by the same colour rule Grove 2's white wire is **GP2**, not
> GP3. Since 28 Oct this no longer matters: the firmware watches both pins of
> each encoder's port and counts the one with pulses. The test image prints
> `[CAL] left wheel counted on GPx, right on GPy`.

## A1. Identify the four encoder wires on the left motor

The two thick wires are the motor (already going to the M1 terminal). The other
four, reading left to right across the plug:

| Position | Signal | Use it? |
|---|---|---|
| 1st, 2nd | motor − / motor + | already in the M1 terminal |
| **3rd** | encoder **VCC** | yes |
| 4th | encoder **B** | no — tape the end |
| **5th** | encoder **A** | yes |
| **6th** (rightmost) | encoder **GND** | yes |

## A2. Connect three wires

| Grove wire | Goes to encoder wire |
|---|---|
| **red** | 3rd — VCC |
| **black** | 6th — GND |
| **white** | 5th — A |
| yellow | not used — tape it |

Plug the Grove connector into **Grove 1**.

> Reversed VCC/GND kills the hall sensor permanently. Before switching on, check
> with the meter in diode mode: black probe on GND, red probe on VCC should show
> a ~0.5 V drop one way and open the other. No drop either way → re-check the
> wire order, do not power it.

## A3. Prove it works — no code needed

1. Power on.
2. Turn the **left** wheel slowly by hand, a quarter turn over ~3 seconds.
3. The status LED next to **Grove 1** must blink as the teeth pass.

Blinks → left encoder is right, on GP0. Done.

---

# Part B — IMU (GY-511) → Grove 4

Grove 4 gives: **white = GP16 (SDA)**, **yellow = GP17 (SCL)**, red = 3V3,
black = GND. The firmware opens I2C0 on exactly those pins.

## B1. Connect four wires

| Grove wire | Goes to GY-511 pin |
|---|---|
| **red** | VCC (or VIN) |
| **black** | GND |
| **white** | **SDA** |
| **yellow** | **SCL** |

Plug the Grove connector into **Grove 4** — the port left empty on purpose.

> SDA and SCL swapped does no damage, it just goes silent. VCC and GND reversed
> does damage. Check those two twice.

## B2. Mount it before you power on

- Flat on the chassis, **rigid** tape (not foam — foam damps the tilt you measure).
- The **X arrow points at the front** of the car.
- As close to the centre as practical, and away from the motor wires.

Mounting it crooked will show up later as a permanent few degrees of pitch.

---

# Then: flash and read

1. Hold **BOOTSEL**, plug in USB, release → `RPI-RP2` drive.
2. Drag `firmware\IS25_TEST_does-not-drive.uf2` (from IS25_final.zip, 26 Oct evening) onto it.
3. Read the console with `python read_serial.py` (or any 115200 terminal).

## What to look for

| You see | Meaning |
|---|---|
| `[TEST] S3-… PASS` … ending in a subsystem 3 summary with 0 failed | IMU is alive and talking on GP16/GP17 |
| `[TEST] S2-… PASS` … subsystem 2 summary 0 failed | motors and encoders pass |
| a test line saying no sensor / no pulses | that device's wiring — see the table below |
| pitch around 0 while the car sits flat | mounting is right |
| pitch reads ~-10° when you tilt the nose up 10° | module is upside down — flip the mount |

The test image **does not drive the car**, so nothing runs away while you read.

## If something is wrong

| Symptom | First thing to check |
|---|---|
| No IMU, `no sensor` in the log | SDA/SCL swapped, Grove plug not fully seated, or IMU not getting 3V3 |
| IMU works but pitch drifts constantly | mounted loose, or taped to a vibrating part of the chassis |
| Left wheel gives no encoder pulses | A wire on the 4th position instead of the 5th, or VCC/GND not biting |
| Both encoders silent | both encoders' GROUND must reach the board — one shared rail works, a floating GND does not |
| Wheel spins but count goes down when driving forward | the two encoders are on each other's pins — swap the two signal wires, not the code |

---

# Tick list

- [ ] Grove cable found, one for each device
- [ ] Left encoder: red → VCC, black → GND, white → A, in **Grove 1**
- [ ] Unused B wire and the yellow wire taped separately
- [ ] Left wheel turn → LED by Grove 1 blinks
- [ ] IMU: red → VCC, black → GND, white → SDA, yellow → SCL, in **Grove 4**
- [ ] IMU taped flat, X arrow forward, cable labelled "white=SDA"
- [ ] Test image flashed, console read, summary lines saved into the repo
- [ ] Photo of the finished wiring taken from above

---

# Tell the group

- **Kai Xuan:** left encoder is on **Grove 1 / GP0**, right on **Grove 2 / GP3**.
  One-line change already made in `subsystem2.h`, and `IS25_README.md` updated to
  match. Do not build with `CRASH_REPORT=1` — it takes GP0/GP1 for the UART
  console and the left encoder silently stops counting.
- **Everyone:** the Grove cable colours are white=signal 1, yellow=signal 2, the
  reverse of the standard. Anyone following an online tutorial will wire it
  backwards.
