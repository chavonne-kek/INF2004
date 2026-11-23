> **Zi Siang's original README, kept for reference (26 Oct).** The code now
> lives in `app_program/subsystem3/` and the sensor is on **Grove 4 (GP16/GP17)**,
> not Grove 3. See `BUDDY4_INTEGRATION.md` for what changed.

# Subsystem 4 — Physics Watchdog (IMU / terrain / hump / collision)

**Owner:** Zi Siang (2500989) · **Code:** `app_program/subsystem4/` · **Tests:** `testing/subsystem4/`

This is my part of the IS25 robot car: the IMU-based terrain and motion
monitor. It answers three questions for the rest of the car:

| Question | Output | Consumer |
|---|---|---|
| Is the robot climbing / on a hump, and how tall is the hump? | `FLG_HUMP` + `hump_peak_mm` | `TSK_CTRL` (Soma), dashboard |
| Has it hit something? | `FLG_COLLISION` | `TSK_CTRL` (stop) |
| What is the robot doing right now? | motion class, pitch, roll (5 Hz telemetry) | `TSK_COMMS` (Chavonne) → MQTT |

It never commands the motors and never drives another subsystem's hardware.

---

## 1. Hardware

| Item | Value |
|---|---|
| Sensor | GY-511 breakout = **LSM303DLHC** (3-axis accelerometer + 3-axis magnetometer, **no gyroscope**) |
| Bus | I2C0, ~390 kHz (the kernel driver's fast-mode timing) |
| Port on the Robo Pico | **Grove 3 = GP4 (SDA) / GP5 (SCL)**, 3V3 and GND |
| Accelerometer address | **0x19** (factory default) or 0x18 — the driver probes both |
| Magnetometer address | 0x1E (phase 2 only) |
| Orientation | X = forward, Y = left, Z = up. Mount it flat, silkscreen X pointing at the nose. |

Wiring:

```
GY-511          Robo Pico Grove 3
 VIN  ───────── 3V3      (3.3 V — do not use 5 V)
 GND  ───────── GND
 SDA  ───────── GP4
 SCL  ───────── GP5
```

The kernel driver enables the RP2040's internal pull-ups; the GY-511 also has
its own, so no extra resistors are needed. Grove 3's 3V3 pin shares the
board's 300 mA budget — the LSM303DLHC draws well under 1 mA.

> **Grove 3 is GP4/GP5 on this board.** GP0/GP1 are the other I2C0 pair, but
> they are taken whenever the team builds with `CRASH_REPORT=1`, so Grove 3 is
> the right choice.

## 2. What is in this folder

| File | Role |
|---|---|
| `subsystem4.h` | the public interface: event bits, telemetry record, odometry input |
| `subsystem4.c` | creates the kernel objects, starts `TSK_IMU` |
| `imu.h` / `imu.c` | **pure** terrain maths — no kernel, no I/O, no floating point; host-testable |
| `drv_lsm303.h` / `drv_lsm303.c` | the sensor driver, on the kernel's I2C device driver |
| `task_imu.c` | `TSK_IMU`: 10 ms period, core 0, priority 16 |
| `subsystem4_test.c` | on-target tests (`make TEST=1`) |
| `host_test/` | the same maths compiled and run on a PC |
| `INTEGRATION.md` | what to paste into `TEAM.md`, and the two shared config edits |

## 3. Build, flash, run

Everything builds inside **WSL Ubuntu** (no Windows toolchain needed):

```sh
cd build_make
make -j8                 # normal image
make TEST=1 -j8          # image that runs every subsystem's tests at power-up
```

Flash `build_make/mtk3pico_smp1_usb_cdc_wifi*.uf2` by holding **BOOTSEL**,
plugging in USB, and copying the file onto the `RPI-RP2` drive. Console output
is the Pico's USB serial port at 115200 baud.

Two shared-config lines must be on for the sensor to appear (see
`INTEGRATION.md` §2). Without them the robot still runs: the task reports
`imu: no sensor …` and sets the fault bit instead of hanging.

### Before the GY-511 arrives: run the maths on the PC

```sh
cd app_program/subsystem4/host_test
./build_and_run.sh        # needs any C compiler; prints 35 PASS/FAIL lines
```

This compiles `imu.c` unchanged and drives it with synthetic traces whose
answers are known exactly (a 30 mm hump must read 30 mm ±15 %, a 60 % braking
ramp must not raise a collision, a 1.2 g vertical kick must not either, and so
on). It is the evidence recorded in `testing/subsystem4/TEST_LOG.md`.

### On the Pico W alone: run the same traces on-target

`make TEST=1` runs `S4-U01…S4-U10`, which drive `TSK_IMU` from the same
generator *inside the firmware*, so the task period, event bits, telemetry
buffers and the fault path are all exercised with no sensor connected.

## 4. How the maths works (and why)

**Pitch** comes from the accelerometer alone (there is no gyro on this part):

```
ax = -g·sin(pitch) - a_car      →      sin(pitch) = -(ax + a_car)/g
```

`a_car` is the longitudinal acceleration the motion subsystem measures from
the encoders, so the reading stays valid while the robot speeds up or brakes —
which is exactly what the Week 8 tribunal asked for (`A6`).

**Hump height** is the *peak* of the running integral of `sin(pitch)` over
distance, not its final value: over a complete hump the descent cancels the
climb, so the integral returns to zero and only its maximum is the height.

Geometry note for the report: that integral equals the moving average of the
terrain height over the wheelbase, so it is exact for humps longer than the
wheelbase and under-reads shorter ones by roughly `hump_length / wheelbase`.
The generator's ramp (2h/L) is the same model, which is why the ±15 % check is
meaningful. Record the measured wheelbase here when the chassis is built.

**Collision** needs two conditions, both true:

1. `|high-pass X| + |high-pass Y| ≥ 800 mg` for 20 ms — the high-pass term is
   `raw − low-pass`, so braking, hump ramps and normal acceleration (slow
   changes) cannot trigger it, and the vertical axis is excluded so a hump's
   kick does not count;
2. the encoder speed falls below 50 % of what it was 150 ms ago.

After a confirmed collision the terrain path is frozen for 0.5 s, because the
crash itself corrupts the accelerometer.

**A constant slope is not a hump.** If the running integral peaks above 150 mm
the robot is on a wedge or a ramp, so the angle is reported but nothing is
counted as a hump. A hump is also only started while the robot is moving
(≥ 30 mm/s).

**Numbers that were tuned, and where they live** (`imu.h`): 15 Hz tilt filter,
4 Hz collision reference, 3° hump entry / 2° exit, 0.4° pitch dead-band,
±2 g full scale (1 mg/LSB) with a `SATURATED` telemetry flag for hard hits.

## 5. Interfaces (see `INTEGRATION.md` for the `TEAM.md` rows)

| ID | Direction | Mechanism | Data |
|---|---|---|---|
| IF-02 | `TSK_MOTION` → here | two single-word stores, no mutex | `s4_odom_publish(dist_mm, speed_mmps, a_enc_mg)` |
| IF-03 | here → `TSK_CTRL` | `flg_events` bits | `FLG_HUMP`, `FLG_COLLISION`, `FLG_IMU_FAULT` |
| IF-08 | here → `TSK_COMMS` | message buffer `mbf_terrain` | 6-byte record, 5 Hz (see `imu.h` for the bit layout) |

## 6. Still to do (mine, in order)

1. **Bench-verify on hardware** once the GY-511 and Robo Pico arrive:
   `S4-U11` (flat → pitch ≈ 0 ± 1°), `S4-U12` (known-height wedge/hump → ±15 %),
   `S4-U13` (unplug the sensor while running → fault bit, task keeps its period).
2. **Wire IF-02 for real** with Kai Xuan: confirm the odometry word layout and
   that the distance is cumulative in mm.
3. **Hump height calibration** on the actual 20/30/40 mm humps: check the
   wheelbase geometry note above and, if it under-reads, add one scale factor
   together with its justification in the report.
4. **Magnetometer / heading** (phase 2) — only if the dashboard wants heading;
   the design already falls back to encoders for turn detection.
