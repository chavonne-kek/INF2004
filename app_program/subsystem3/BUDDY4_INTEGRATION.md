# Buddy 4 code: how it was integrated (26 Oct)

> **17 Nov update.** The module on the car is an **LSM303D**, not a GY-511
> (LSM303DLHC): `drv_lsm303.c` now finds it at 0x1D/0x1E (WHO_AM_I 0x49) and
> keeps the DLHC as a fallback. The lecturer removed IMU collision
> detection: the collision rule and FAULT_COLLISION below are history.
> `imu.c` now has `lift_step()` (lifted or overturned → FAULT_LIFTED, bit 5,
> motors stopped by TSK_IMU, run over; latched until the next run), and the
> hump ramp guard is 600 mm of travel instead of 4 s. See `imu.h` and
> `IS25_COMPLIANCE.md` (17 Nov).

Zi Siang's terrain maths (`imu.c` / `imu.h`: integer pitch, hump height
as the peak of the integral, the two-condition collision rule, motion
classes) is now subsystem 3's engine. It replaces the skeleton's
`imu_calc.c`. His algorithms are kept. The few changes are marked `IS25`
in the code, and `BUDDY4_CHANGES.diff` shows them line by line.

Tests: his own 35 PC checks still pass (`testing/subsystem3/buddy4_host`,
`make run` or `run_host_tests.bat`). The team's PC simulation
(`testing/subsystem3/host`, 15/15) now drives his code through the adapter
with a physically correct sensor model.

## Where his files went

| His file (subsystem4/) | Now | Why |
|---|---|---|
| `imu.c`, `imu.h` | `subsystem3/imu.c`, `imu.h` | In TEAM.md the IMU is **subsystem 3**. `subsystem4/` is Soma's controller and Buddy 5's obstacle engine; copying his folder over it would delete both. |
| `task_imu.c` | merged into `subsystem3/imu_task.c` | Same job. His re-probe and his sim test mode were added to it. |
| `drv_lsm303.c` | not used; the team driver (`subsystem3/drv_lsm303.c`) stays | See "Pins" and "Full scale" below. His re-probe idea was added to it. |
| `subsystem4.c/.h` | not used | They create a second `flg_events` with different bits, plus `mbf_summary` and `mbf_terrain`. The team already has these interfaces (below). |
| `subsystem4_test.c` | S3-U06..U11 in `subsystem3_test.c` | Same tests, new IDs. |
| `host_test/` | `testing/subsystem3/buddy4_host/` | Unchanged test file. |
| `tools/read_serial.py` | `tools/read_serial.py` | Now saves to any subsystem's `logs/` (2nd argument, default subsystem3). |

## Conflicts with the team design, and what was done

| # | His package | Team design (Week 6, Tribunal 1, TEAM.md) | Done |
|---|---|---|---|
| 1 | Folder `subsystem4/`, called "S4" | IMU = subsystem 3; S4 = controller + scan | Moved to `subsystem3/`. |
| 2 | I2C0 on **GP4/GP5** (Grove 3) | GP4/GP5 = HC-SR04 trigger/echo (Grove 3). IMU on **GP16/GP17** (Grove 4); `i2c_cnf_sysdep.h` already set | Team pins kept. **Plug the GY-511 into Grove 4.** |
| 3 | Own `flg_events` with FLG_HUMP 0x04, FLG_COLLISION 0x08, FLG_IMU_FAULT 0x10 | One team `flg_events` (EVT_HUMP bit 5) and `flg_faults` (FAULT_IMU, FAULT_COLLISION, owner S3) | Mapped in the adapter. |
| 4 | `s4_odom_publish()`: S2 pushes distance, speed and **a_enc** | Tribunal 1, Version B: S2 publishes one odometry word; **S3 derives the acceleration itself** | The adapter computes a_enc from the wheel speeds (50 ms window). S2 needs no change. |
| 5 | `imu_clear()` "called by Soma" | Owner sets and clears; TSK_CTRL only reads | S3 clears FAULT_COLLISION itself after both wheels have been stopped for 2 s, then calls `imu_clear()`. |
| 6 | `mbf_terrain` 6-byte record at 5 Hz | IF-09 status word (sent by S5 at 5 Hz) + 2-byte hump record (`subsystem5_post`) | Mapped into the team formats. His pack functions remain, unused. |
| 7 | ±2 g full scale | ±4 g (Week 6) | Team driver keeps ±4 g. His saturation limit changed 1950 → 3950 mg. |
| 8 | No TURNING class (his note: needs the wheel speeds) | FR10 lists TURNING | The adapter adds it (wheel speeds differ by > 60 mm/s). |
| 9 | 8 motion classes | 3-bit field, 6 values | Values 0–5 unchanged; **6 = DECELERATING, 7 = CRUISING** added. UNKNOWN is sent as STATIONARY. Chavonne: add the two names on the dashboard. |
| 10 | Hump states IDLE/RISING/FALLING | Statechart 6.3 FLAT/CLIMBING/CRESTING/DESCENDING/INVALID | Mapped: IDLE → FLAT, RISING → CLIMBING, FALLING → DESCENDING; INVALID while FAULT_IMU. CRESTING is not produced. |

## Bugs found and fixed

**1. Sign convention (important on the real car).** `imu.c` models
`ax = −g·sin(pitch) − a_car`. A real LSM303 with X forward reports the
opposite: `+g·sin(pitch) + a_car` (specific force; level reads +1 g on Z,
which his generator also uses). Fed raw, nose-up reads as nose-down, and
the encoder correction *doubles* the car's own acceleration instead of
cancelling it. His PC tests could not see this, because his generator uses
the same formula. The adapter (`imu_glue.c`) passes −forward and −left, so
`imu.c` is unchanged. Host test H-IM12 shows both results: through the
adapter +10.25°, raw −10.35°.

**2. Level reads +0.45°.** `asin_cdeg(0)` returned the first table bucket
(45 centi-degrees). Now 0 returns 0.

**3. A slow ramp was reported as a hump.** The 150 mm "slope" limit only
catches a 10° ramp if the robot covers it fast enough within the 4 s guard.
At 10–20 cm/s it was reported as a 69–137 mm hump. Now a guard expiry while
still RISING (it never crested) is dropped. A long hump at slow speed has
crested by then, so it is still reported (H-IM02, 55 mm at 12 cm/s).

**4. Zero-height "humps".** Sensor noise of 30 mg or more, and a hard
launch (the encoder acceleration lags the sensor by about 25 ms, giving a
short ±6° blip at 2.5 m/s²), produced 0 mm hump reports. Peaks under
`IMU_HUMP_MIN_MM` (5 mm) are no longer counted. The track humps are
20–40 mm.

**5. His on-target tests never used the generator.** `subsystem4_test.c`
called `imu_sim_start()`, which does not switch the task to the generator
(`s4_sim_on` stayed 0). S4-U05/U06 would have failed on the Pico.
`imu_task_sim_start()` now does both.

## Changes after the first car run (28 Oct)

The 27 Oct console showed `pitch -128/4 deg hump 0` with FAULT_IMU: the
GY-511 never answered, and the status word was still all zero, which
decodes as a real pitch of −32°.

- **No-data status.** Until the first good sample, and while FAULT_IMU is
  set, the status word says hump **INVALID**, pitch 0, heading unknown.
  The console prints `IMU n/a`. S3-U02 now also fails if there is no data.
- **I2C bus clear.** The GY-511 runs from the Robo Pico 3V3, so it stays
  powered when the Pico resets (RESET button, watchdog). A reset in the
  middle of a read can leave it holding SDA low, and then nothing works
  until its power is cycled. Every open (and the 1 s re-probe) first clocks
  SCL until SDA is free, then sends a STOP (NXP UM10204 3.1.16).
- **Wiring check on the console.** If the sensor does not answer at start,
  one line says whether a line is held low, whether it answers with SDA and
  SCL swapped (then swap the white and yellow wires at the GY-511), or not
  at all (power, ground, cable).
- **Encoder-acceleration window.** The 50 ms window assumes a fine encoder.
  In the firmware simulation with the 40-edge placeholder, one edge more or
  less in S2's speed window read as about 0.1 g: false ±6° pitch, so
  "humps" opened during normal driving and TSK_CTRL ignored obstacles
  further than 10 cm for up to a second. With 480 edges per turn, or with
  `GLUE_ACC_WINDOW` 20, there were none. The scale now follows the window
  automatically. If S2-M01 measures under 150 edges per turn, set
  `GLUE_ACC_WINDOW` to 20 in `imu_glue.h` (the test prints a reminder).

## Changes after the test image on the car (29 Oct)

The car test image failed S3-U01, S3-U02 and **S3-U06** (100 cycles per
second). In simulation, a GY-511 sitting on the bus without power (its
pull-ups then hold both lines low) gives the same three failures: every
I2C transfer waited for the kernel driver's 1 s timeout, so TSK_IMU ran
about once a second.

- **No more 1 s stalls.** `drv_lsm303_open()` first checks with a
  bit-banged address byte that the accelerometer answers, and every
  transfer first checks that SDA and SCL are high. A missing or
  miswired sensor now fails in microseconds.
- **Re-probe instead of reading a missing sensor.** A failed open at start
  raises FAULT_IMU at once. While FAULT_IMU is set, TSK_IMU does not read
  the sensor; once a second it re-probes, and after a successful re-probe
  five good reads clear the fault (`S3: IMU back`). Simulated unplug and
  replug: `IMU read failing` after 50 ms, `IMU back` 0.3 s after the plug
  goes back in.
- **S3-U06** counts its 100 cycles once TSK_IMU runs the synthetic sensor.

## Things for Zi Siang to check on the car

- **Axis signs:** tilt the nose up; pitch must go positive (S3-M01). If it
  does not, change `IMU_FWD_SIGN` / `IMU_FWD_FROM_X` in `imu_task.c`, not
  the sign in `imu.c`.
- **Collision threshold:** the high-pass is taken after the slow filter
  update, so a step decays to 78 % and then 61 %. "800 mg for 2 samples"
  really needs about **1.3 g held for 20 ms**. A 1.2 g bump does not
  trigger. Tune on the track (S3-M04).
- **Encoder-acceleration lag and noise:** check the pitch log during a
  hard start and while cruising with the motors running (S3-M02). If humps
  open while driving on flat ground, raise `GLUE_ACC_WINDOW` (see above);
  if they open at hard starts, lower the motor ramp.
- **Wheelbase:** his README note applies: humps shorter than the
  wheelbase under-read. Record it.
- **Wiring:** GY-511 on **Grove 4 (GP16 SDA / GP17 SCL)**, 3V3.
