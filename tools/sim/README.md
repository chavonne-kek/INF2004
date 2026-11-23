# Firmware simulation (firmware-in-the-loop)

`harness.mjs` runs the real IS25 firmware image on
[rp2040js](https://github.com/wokwi/rp2040js) (an RP2040 emulator) and
simulates the car around the chip:

| Part | Simulated as |
|---|---|
| Motors | PWM slices 4/5 (GP8–GP11) → wheel speed (dead band, 60 ms lag), battery switch |
| Encoders | Pulse train per wheel on a chosen pin (GP0 or GP1 left, GP2 or GP3 right), edges per turn |
| IR sensors | Robot pose → black 18 mm line, optional Code 39 barcode beside it and a left branch → ADC0–2, spot size |
| HC-SR04 | Trigger on GP4 → echo pulse on GP5: a fixed distance, or (17 Nov) a **box** on the track ray-cast from the sonar at the servo angle (GP15 pulse), so the scan, width and bypass run for real |
| IMU | **LSM303D** (17 Nov, the car's module) at 0x1D or 0x1E: WHO_AM_I, 16-bit accelerometer and magnetometer; or the old LSM303DLHC (GY-511) at 0x19/0x1E (`imuChip`). Gravity + the car's own acceleration + noise, heading, a hand **lift** (vertical push) and an **overturn** (roll onto the roof). GP16/GP17 also as open-drain wires with the module's address ACK, for the firmware's bit-banged bus clear, presence check and wiring check. Faults: absent, on the bus without power (both lines low, controller transfers hang), SDA/SCL crossed, unplugged for a while |
| Buttons | GP20 start, GP21 reset presses at set times |
| Console | USB CDC, printed with the simulated time |

**Limits.** The emulator has one core, so it runs the single-core build
(`SMP=0 WIFI=none`): the same application code, but every task shares one
core and there is no radio. Motor, optics and encoder numbers are guesses.
It checks logic, start-up, interrupts, drivers and messages; it does not
replace tests on the car.

## Run

```sh
cd tools/sim
npm install rp2040js@1.4.1
# RP2040 boot ROM (Raspberry Pi, BSD-3) as a JS module, from the rp2040js repo:
curl -L -o bootrom.mjs https://raw.githubusercontent.com/wokwi/rp2040js/main/demo/bootrom.ts

cd ../../build_make
make SMP=0 WIFI=none TEST=1 -j8      # mtk3pico_smp0_usb_cdc_test.uf2
make SMP=0 WIFI=none -j8             # mtk3pico_smp0_usb_cdc.uf2
cd ../tools/sim
node harness.mjs ../../build_make/mtk3pico_smp0_usb_cdc_test.uf2 40 \
  '{"leftPin":1,"rightPin":2,"edgesPerRev":46,"handTurnFromS":2.5,"handTurnToS":12.5}'
```

Arguments: image, simulated seconds, scenario (JSON). The console goes to
the terminal and to `last_run.log`. Scenario keys (defaults in the file):
`sweepFromS`/`sweepToS` (car slid sideways across the line by hand),
`spinFromS`/`spinToS`/`spinRevs` (car turned on the spot by hand),
`noseUpFromS`/`noseUpToS`/`noseUpDeg`, `leftOkFromS` (left encoder dead
until then), `sensorFwdMm` (IR sensors ahead of the axle), `mirror` (barcode sensor
on the car's left), `battery`, `leftPin`, `rightPin`,
`edgesPerRev`, `imu`
(`ok`/`absent`/`unpowered`/`swapped`), `imuOffFromS`/`imuOffToS`,
`obstacleCm`, `obstacleFromS`, `pressStartAt`, `pressResetAt`,
`handTurnFromS`/`handTurnToS`/`handTurnRevs` (turn the wheels by hand),
`pushFromS`/`pushToS`/`pushMmS` (push the car straight), `barcode` (e.g.
`"A"`), `branchLeftAtX`, `spotMm`, `accNoiseMg`, `profile` (CPU profile).

## Results on 28 Oct (logs in `testing/integration/logs/sim_2026-10-28/`)

| Run | Scenario | Result |
|---|---|---|
| `fin_a` | Test image, encoder wires on the *other* pins (GP1, GP2), wheels turned 10× by hand, IMU fitted | 42/42 PASS; wheels found on GP1/GP2; 46 edges per turn measured |
| `fin_b` | Test image, nothing connected, no IMU | Only S3-U01/U02 FAIL, with the I2C wiring check on the console |
| `fin_c` | Normal image, battery off, no IMU (the 27 Oct car) | Same FAULT as on the car (STALL + IMU), now with the reasons printed |
| `fin_e` | Normal image, 480-edge encoder, obstacle at 30 cm while following | OBSTACLE_AVOID 0.14 s after the obstacle appears |
| `fin_f` | Normal image, car pushed over `*A*` at 15 cm/s, 4 mm sensor spot | `S1: barcode 'A'`, `S4: barcode 'A' -> command 1` |

Findings that changed the code are listed in `IS25_COMPLIANCE.md`
section 0.

## Results on 29 Oct (logs in `testing/integration/logs/sim_2026-10-29/`)

The 29 Oct car test image printed S3-U01, S3-U02 **and S3-U06** FAIL and
no encoder edge at all. Run `g1` gives the same result with the 28 Oct
image and a GY-511 that sits on the bus without power (its pull-ups hold
both lines low, so every controller transfer waits for the driver's 1 s
timeout and TSK_IMU stalls).

| Run | Scenario | Result |
|---|---|---|
| `g1` | **28 Oct** test image, GY-511 without power, wheels not turned | S3-U01, U02, U06 FAIL, 39/42: the car's 29 Oct result. Boot open took 1 s (`-50` = timeout) |
| `g2` | 29 Oct test image, same | Only S3-U01/U02 FAIL; `I2C check: SDA (GP16) LOW, SCL (GP17) LOW … a line is held low`; the new `[CAL] NO encoder pulse …` lines |
| `a` | 29 Oct test image, encoder wires on the other pins, wheels turned 10× | 42/42 PASS |
| `b` | Nothing connected | Only S3-U01/U02 FAIL, `does not answer` |
| `h` | IMU SDA/SCL crossed | `answers with SDA/SCL SWAPPED` |
| `c`, `e`, `f` | As on 28 Oct | Same results (27 Oct FAULT explained; obstacle in 0.14 s; barcode `A`) |
| `i` | Normal image, GY-511 unplugged 8.0–10.2 s | `S3: IMU read failing` at 8.05 s, `S3: IMU back` at 10.47 s |

## Results on 30 Oct: the check images (logs in `testing/integration/logs/sim_2026-10-30/`)

| Run | Scenario | Result |
|---|---|---|
| `check1_ir_sweep_then_barcode` | CHECK=1, car slid across the line, then pushed over `*A*` | Calibration OK (spans 2740); `S1: barcode 'A'`; 500 samples/s |
| `check1_ir_line_under_right_sensor` | Line ends under the right sensor | `error -40 mm, steer RIGHT` |
| `check2_hand_turn_and_motor_test` | CHECK=2, wires on GP1/GP2, wheels turned 10×, START | 46.0 edges per turn; each motor `-> ok` (87 edges, other 0) |
| `check2_no_encoder_pulses` | No encoder pulses | `[CAL] NO encoder pulse …`; every motor phase `NO edges` |
| `check3_calibrate_and_nose_up` | CHECK=3, one circle by hand, nose up 10° | Calibration OK; pitch +9.25…+10.25 |
| `check3_imu_plugged_in_late` | GY-511 plugged in at 24 s | `does not answer` every 3 s, then `S3: IMU back` and data |
| `check3_imu_without_power` | GY-511 without power | `a line is held low` every 3 s |
| `check4_range_servo_buttons` | CHECK=4, object at 30 cm, START and RESET | Range 30 cm, sweep 90/0/180/90, presses counted |
| `check5_status_line` | CHECK=5 | 4/4 PASS, status line every 2 s |
| `test_image_all_pass_start_ignored` | Full test image, START after the tests | 42/42 PASS; `S4: START pressed (1)`, state stays IDLE |

## 17 Nov additions

Scenario keys: `imuChip` (`d` = LSM303D, default; `dlhc`), `imuAddr` (0x1d
or 0x1e), `liftedFromS` + `liftMg` + `liftPushS` (a hand lifts the car),
`overturnFromS` (rolled onto its roof in 1 s), `layout` (`arm`, default:
GP26 just right of the line, GP27 on its left edge, GP28 the barcode arm
`armMm` further left, labels 20 mm left of the line; `old`: 28 Oct),
`narrowMm` (Code 39 narrow element), `boxX`/`boxD`/`boxW`/`boxY` (a box
obstacle) with `sonarFwdMm`, `lineEndX`. `edgesPerRev` now defaults to the
car's 1270 (with 40 the turns overshoot and the start-up sweep stalls).

Results (logs in `testing/integration/logs/sim_2026-11-17/`):

| Log | Scenario | Result |
|---|---|---|
| `test_image` | Test image, LSM303D at 0x1D, wheels turned by hand | 46/46 PASS, including S3-U12..U16 (lift, braking, overturn, latch, knocks) in the real TSK_IMU |
| `lift` | Normal image, start, barcode `A`, lifted at 38 s | `S1: barcode 'A'`; `S3: LIFTED` 0.12 s after the push; STOP; FINISHED |
| `overturn_0x1e` | Normal image, LSM303D at 0x1E, rolled over at 28 s | found at 0x1E; `S3: OVERTURNED` 0.55 s later; FINISHED |
| `noisy_drive` | 70 s at 120 mg (uniform) vibration, 18° nose-up for 6 s | following throughout, no false LIFTED |
| `obstacle_10cm`, `obstacle_16cm` | Box 10 × 10 cm on the line; 16 × 6 cm off-centre | OBSTACLE_AVOID, scan, bypass, back to LINE_FOLLOW on the line |
| `barcode_turn` | `*A*` then a left branch | turns left at the junction and follows the branch |
| `old_chip` | `imuChip: dlhc` | `S3: IMU LSM303DLHC (GY-511)`: the fallback still works |

