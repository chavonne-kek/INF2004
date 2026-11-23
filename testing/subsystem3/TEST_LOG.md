# Subsystem 3 — test log

**Subsystem:** IMU (LSM303D), pitch, humps, lift / overturn stop  ·  **Owner:** Zi Siang  ·  **Student ID:** 2500989  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | TSK_IMU running, IMU answering on I2C0 (LSM303D since 17 Nov; LSM303DLHC before) | FR8 | auto | `subsystem3_test.c` | PASS |
| S3-U02 | Pitch near 0 on a level desk, from real sensor data (fails with hump INVALID = no data) | NFR9 | auto | `subsystem3_test.c` | PASS |
| S3-U03 | IMU status word round trip | CR1 | auto | `subsystem3_test.c` | PASS |
| S3-U04 | atan2 / heading accuracy on target (was: atan2 / asin / sqrt; pitch now uses Zi Siang's integer table) | NFR9 | auto | `subsystem3_test.c` | PASS |
| S3-U05 | Hump record 14 bits, saturating | CR1 | auto | `subsystem3_test.c` | PASS |
| S3-U06 | TSK_IMU runs 100 cycles per second (his S4-U03), counted once TSK_IMU runs the synthetic sensor (29 Oct) | NFR2 | auto | `subsystem3_test.c`, synthetic sensor | 95–105 |
| S3-U07 | Flat ground 3 s: no hump, no collision (his S4-U04); since 17 Nov: no lift | FR8, FR10 | auto | synthetic flat trace | PASS |
| S3-U08 | 30 mm × 400 mm hump: one report 25–35 mm, hump-active bit seen (his S4-U05) | FR8, NFR9 | auto | synthetic hump trace | PASS |
| S3-U09 | **Retired 17 Nov** (collision detection removed by the lecturer). Was: impact + 67 % speed collapse raises FAULT_COLLISION | – | – | – | – |
| S3-U10 | **Retired 17 Nov**. Was: 60 % braking ramp raises no collision | – | – | – | – |
| S3-U11 | Constant 10° slope for 4.5 s is not a hump (his S4-U09) | FR8 | auto | synthetic tilt trace | PASS |
| S3-U12 | Lifted by hand (0.2 g up for 0.3 s at t = 1.5 s, wheels turning): FAULT_LIFTED, cause LIFTED, once | Demo 10 | auto | synthetic lift trace | PASS |
| S3-U13 | 60 % braking ramp raises no FAULT_LIFTED (the start also releases the U12 latch: owner clears) | Demo 10 | auto | synthetic braking trace | PASS |
| S3-U14 | Rolled onto its roof in 1 s: FAULT_LIFTED, cause OVERTURNED | Demo 10 | auto | synthetic roll trace | PASS |
| S3-U15 | FAULT_LIFTED stays latched until `subsystem3_lift_rearm()`, then clears within 50 ms | Demo 10 | auto | after U14 | PASS |
| S3-U16 | Vibration (±150 mg, 50 Hz) and a 1.5 g vertical knock every 0.3 s for 4 s: no FAULT_LIFTED | Demo 10 | auto | synthetic bump trace | PASS |
| S3-M01 | Axis mapping | FR8 | manual | Lift the nose: pitch rises. Else change `IMU_FWD_FROM_X` / signs in `imu_task.c`. | Nose up = + pitch |
| S3-M02 | Stationary pitch noise with motors running | NFR9 | manual | Wheels on a stand, motors at 20 cm/s, log pitch 10 s. | σ < 1° |
| S3-M03 | Humps of known height, 10 runs | NFR9, A6 | manual | Measure the track humps; drive over at 15 and 20 cm/s. | ±15% |
| S3-M04 | No false collision (**retired 17 Nov**, see S3-M07) | – | – | – | – |
| S3-M05 | Magnetometer error with motors on | A6 | manual | Compare heading with a compass at 4 directions. | ≤ 10° (else encoders only) |
| S3-M06 | Unplug the IMU module mid-run, plug it back (his S4-U13) | FR10, safety | manual | Watch the console: `S3: IMU read failing`, then `S3: IMU back` within ~1 s of replugging. | FAULT_IMU set, then cleared |
| S3-M07 | No false lift on the course | Demo 10 | manual | 5 full runs over humps, turns, the obstacle bypass, hard braking. | 0 × `S3: LIFTED` / `OVERTURNED` |
| S3-M08 | Lift stops the car | Demo 10 | manual | While following, lift the car 10-20 cm, 10 times, slow and fast. | `S3: LIFTED`, wheels stop, state FINISHED, every time |
| S3-M09 | Overturn stops the car | Demo 10 | manual | Check image (CHECK=3): tip the car past 60° on each side and onto its roof. | `OVERTURNED` each time; 45° does nothing |
| H-IM01..11 | Maths vs libm; simulated humps 20–55 mm at 12–20 cm/s; braking; turning; impacts (H-IM06..08, impacts: **retired 17 Nov**) | NFR9, FR10 | host | `testing/subsystem3/host` | 11/11 PASS |
| H-IM12..17 | (26 Oct, Zi Siang's imu.c through imu_glue.c) sign convention, launch compensation, 10° wedge at 10–25 cm/s, motion classes, 60 s of 50 mg noise, hard launch | NFR9, FR8, FR10 | host | `testing/subsystem3/host` | PASS |
| H-IM18 | (2 Nov) 55° pitch for 250 ms latches the pickup: **retired 17 Nov** (pitch only: missed roll and upside down) | – | – | – | – |
| H-IM19 | 25° climb for 2 s is not a pickup; since 17 Nov also: a module mounted upside down reads upright as upright, and rolling it over is OVERTURNED | Demo 10 | host | `testing/subsystem3/host` | PASS |
| H-IM20 | LIFTED latched for 3 s, `imu_clear()` re-arms, a second lift is detected | Demo 10 | host | | PASS |
| H-IM21 | Humps 20–50 mm at 12° and 20°, at 2, 7 and 12 cm/s, 50 mg noise: every hump within ±15 %, never LIFTED (ramp guard is a distance since 17 Nov) | NFR9, Demo 3, 10 | host | | PASS |
| H-IM22 | Lifted by hand, 0.2 g for 0.3 s, wheels turning, 50 mg rms vibration: LIFTED within 0.3 s | Demo 10 | host | independent world-frame model | PASS |
| H-IM23 | Gentle lift, 0.1 g for 0.4 s, nose tipped 30°: LIFTED | Demo 10 | host | | PASS |
| H-IM24 | 20 ms knocks of ±1.5 g every 0.3 s: no LIFTED | Demo 10 | host | | PASS |
| H-IM25 | On its side for 0.3 s: OVERTURNED; a 70° wobble for 0.15 s: nothing | Demo 10 | host | | PASS |
| H-B4 | Zi Siang's own checks on imu.c (his S4-H01): 35 until 2 Nov; since 17 Nov 44 (T6–T8, T10 now lift checks; T14–T16 overturn, knocks, latch) | NFR9, FR8, Demo 10 | host | `testing/subsystem3/buddy4_host` | 44/44 PASS |

Since 26 Oct the host tests run Zi Siang's `imu.c`. **H-IM05** (hump while
turning not reported) and **H-IM11** (climb over 3 s discarded) are
retired: his design reports humps on curves and uses a 4 s guard (a ramp
that never crests is dropped, checked by H-IM14).

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**host** = PC test in `host/` (`make run`); **manual** = a person observes
the result (scope, ruler, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-24 | H-IM01..11 | (not committed yet) | skeleton author | PASS (11/11) | `logs/2026-10-24_host.txt` | PC run, gcc, simulated inputs only. |
| 2026-10-25 | S4-H01 (now H-B4) | (Zi Siang's local tree) | Zi Siang | PASS (35/35) | his `logs/2026-10-25_host_imu.txt` | Before integration. |
| 2026-10-26 | H-IM01..04, 06..10, 12..17 | (not committed yet) | integration | PASS (15/15) | `logs/2026-10-26_host.txt` | After integrating Zi Siang's imu.c. |
| 2026-10-26 | H-B4 | (not committed yet) | integration | PASS (35/35) | `logs/2026-10-26_buddy4_host.txt` | His test file unchanged, on the integrated imu.c. |
| YYYY-MM-DD | S3-U01..11 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | First on-target run: owner to fill in (`python tools/read_serial.py`). |
| 2026-10-29 | S3-U01..11 | 28 Oct image | team (car) | 8/11: U01, U02, U06 FAIL | console log (car) | GY-511 not answering. U06 failed because each I2C transfer waited 1 s; fixed 29 Oct (`IS25_COMPLIANCE.md` R10, R11). Simulated with the 29 Oct image: only U01/U02 fail without the sensor |
| 2026-11-17 | H-IM01..04, 09..10, 12..17, 19..25 | `IS25_final_17nov` | integration | PASS (19/19) | `make -C testing/subsystem3/host` | LSM303D driver, lift / overturn stop, distance ramp guard. |
| 2026-11-17 | H-B4 | `IS25_final_17nov` | integration | PASS (44/44) | `make -C testing/subsystem3/buddy4_host` | |
| 2026-11-17 | S3-U01..08, 11..16 | `IS25_final_17nov` test image | integration (simulator) | PASS (14/14) | `testing/integration/logs/sim_2026-11-17/` | rp2040js with an LSM303D model at 0x1D; the real kernel and TSK_IMU. On the car: owner to repeat. |
| 2026-11-23 | H-IM01..21 | v94n | PC host run | PASS (19/19) | `logs/2026-11-23_host_v94n.txt` | |
| 2026-11-23 | H-B4 | v94n | PC host run | PASS (44/44) | `logs/2026-11-23_buddy4_host_v94n.txt` | |
