# IS25 robot car: final code (26 Oct, updated 23 Nov v94n)

All five subsystems are complete and build together into one firmware
image for the Robo Pico (Pico W, micro T-Kernel 3.0 SMP). The design
follows Week 6 and the Tribunal 1 decisions (PostTri 1.2).
`FLASHING_GUIDE.md` explains how to build and flash it.

**Status, 23 Nov (v94n):** the code has been run and tuned on the car
(line following, barcode turns with the measured 130 mm sensor-to-axle
distance, MQTT dashboard, lift stop; the v94n obstacle rule and fixed bypass
are checked in the pre-demo run, SYS-07..09).
Test status, use cases and requirement coverage are in
`testing/TEST_SUMMARY.md`; known hardware limits (the HC-SR04 runs at 3.3 V
and does not report distance) are recorded there and in `CHANGES_v94n.md`.

## Who owns what

| Buddy | Folder | Task(s) | Read first |
|---|---|---|---|
| 1 Chavonne: WiFi, MQTT, telemetry | `subsystem5/` | comms_task 20, core 0 | `MQTT_PROTOCOL.md`; dashboard `tools/is25_dashboard.py` |
| 2 Kai Xuan: motors, encoders, PID | `subsystem2/` | motion_task 14, core 1 | `README.md` (API, statechart 6.5, calibration) |
| 3 Hao Ming: IR line + barcode | `subsystem1/` | line_task 12, core 1 | `README.md` |
| 4 Zi Siang: IMU (LSM303D), humps, lift / overturn stop | `subsystem3/` | imu_task 16, core 0 | `BUDDY4_INTEGRATION.md` (his own `imu.c`) |
| 5 Soma: ultrasonic, servo, controller | `subsystem4/` | ctrl_task 18, core 1; scan_task 19, core 0 | `BUDDY5_INTEGRATION.md` (his own `b5_*.c`) |

`TEAM.md` has every pin, task, kernel object and interface. Every packed
format is in its owner's `subsystemN.h`, with the bit layout above it.

## What was checked

| Check | Result |
|---|---|
| `make`, `make TEST=1`, `make NET=lwip MQTT=1` (+ `TEST=1`, with the IF-07 patch) | All 4 build. `CONSOLE=uart` / `CRASH_REPORT=1` stop with a clear error on purpose: UART0 would take GP0, the left encoder |
| All application code with `-Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Wdouble-promotion` | 0 warnings |
| PC tests (`make run` in each folder) | S1 24/24 · S2 13/13 · S3 15/15 + Zi Siang's 35/35 · S4 3/3 + Buddy 5's suite · S5 6/6 |
| Robot ↔ dashboard round trip (`testing/subsystem5/roundtrip`) | 2,275 checks, 0 failed |
| Final check against the brief, Week 6 report, lecturer feedback, Tribunal 1 and datasheets | `IS25_COMPLIANCE.md` |
| Dashboard through an MQTT broker with a simulated robot | Decodes, logs, sends commands |
| Whole firmware on a simulated car (28 Oct, `tools/sim/`: RP2040 emulator with motors, encoders, IR line, barcode, HC-SR04, GY-511) | Test image 42/43 PASS (encoder wires swapped on purpose); normal image calibrates, follows the line, reads `*A*`, stops for an obstacle in 0.14 s; the 27 Oct FAULT reproduced exactly with the battery off and no IMU. Single-core build (the emulator has one core), so not a substitute for the car |
| On the robot | 27 Oct: FAULT at start-up (STALL + IMU + WIFI). Explained in `IS25_COMPLIANCE.md` section 0 |
| Test image on the robot (29 Oct) | 39/42: S3-U01/U02 (GY-511 not answering), S3-U06 (each I2C transfer waited 1 s; fixed), and no encoder edge in S2-M01 (hardware). Reproduced in simulation with the GY-511 on the bus without power. `IS25_COMPLIANCE.md` section 0a |

## First steps on the car (in this order)

1. **Wiring** (as built on 26 Oct, `IS25_BUILD_LOG`; checked against the
   Robo Pico datasheet). **Cable colours in this kit: white = pin 1 = the
   port's first GPIO, yellow = pin 2 = second GPIO, red = 3V3, black = GND**
   (not Seeed's order).

   | Part | Port | Signal (wire) | Notes |
   |---|---|---|---|
   | Left motor / right motor | M1 / M2 screw terminals | GP8/9, GP10/11 | Swap a pair to reverse a wheel |
   | Left encoder (phase A) | **Grove 1** | GP0 or GP1 (either wire) | Own 3V3 + GND. Needs `CRASH_REPORT=0` (the default) |
   | Right encoder (phase A) | **Grove 2** | GP2 or GP3 (either wire) | Own 3V3 + GND |
   | IR left / centre / right (AO) | Grove 5 / 6 / 7 | GP26 / GP27 / GP28 (yellow) | White wires (GP6, GP26, GP7) taped off; 3.3 V; low, about 2–3 mm above the floor (see "IR sensor height" below) |
   | HC-SR04 | **Grove 3** | Trig GP4 (white), Echo GP5 (yellow) | Powered from 3V3, so no divider. The datasheet rates it 5 V: if ranges are short or missing, move VCC to 5 V **and** fit the 1k/2k divider |
   | LSM303D IMU (17 Nov: not a GY-511) | **Grove 4** | SDA GP16, SCL GP17 (the check image tells you if they are swapped) | Rigid, flat, X arrow forward |
   | Servo SG90 | Servo header | GP15 | V+ = battery; leave the GP14 servo port empty (same PWM slice) |
   | Battery | VIN terminal | 3.6–6 V | |

   **Encoder signal wire: either one works (28 Oct).** The build log was
   not sure whether the right encoder ended up on GP2 or GP3. Both pins of
   each encoder's port are now watched, and each wheel is counted on the
   pin that carries the pulses. The test image prints which pin that is
   (`[CAL] left wheel counted on GPx, right on GPy`).

   Keep the **Maker Port empty** (it shares GP2/GP3), and leave the
   **VBAT-sense jumper unsoldered** (it would put the battery on GP28, the
   barcode sensor).
2. **Test image.** `make TEST=1` (or `firmware/…_test.uf2`), flash, and save
   the console (`python tools/read_serial.py`). The car does not drive.
   Every `[TEST]` line should PASS; a FAIL usually means a sensor is not
   wired. At `[CAL] S2-M01`, turn each wheel 10 times by hand. **To find
   which wire is wrong, check one subsystem at a time** with the check
   images (`make CHECK=1` … `5`, or `firmware/checks/`): its self-tests,
   then its live readings. Steps: `docs/IS25_Subsystem_Checks.txt`.
3. **Kai Xuan:** encoder edges (set `MOTION_EDGES_PER_REV` before the
   first normal run: if the real count is higher than the placeholder 40,
   the start-up sweep stops far short and the IR calibration
   fails), motor direction, wheel size, PID,
   then 1 m and 90° accuracy (`subsystem2/README.md`).
4. **Zi Siang:** tilt the nose up; pitch must go positive (S3-M01).
5. **Soma:** servo angles, which side index 24 points to, car width.
6. **Hao Ming:** raw IR values on the real track (S1-U12), the barcodes
   both ways (S1-U14).
7. **Normal image.** Flash it, then put the car on the line, switch the
   battery ON and press the board's RUN/RESET button: the start-up spin
   needs motor power (on USB power alone it ends in STALL → FAULT, as in
   the 27 Oct log). It beeps, stays still for about 3 s and sweeps 60° left, 120° right,
   60° left to
   calibrate. Press GP20 to start; GP21 resets from FAULT or
   EMERGENCY_STOP. If it goes to FAULT, the console says why
   (`FLASHING_GUIDE.md`, "If the car goes to FAULT at start-up").
8. **Chavonne:** after instructor approval, apply the IF-07 patch, build
   `NET=lwip MQTT=1`, and run the dashboard (`MQTT_PROTOCOL.md`).

## Decisions still open (team)

- **IF-07 patch.** It changes two kernel files (`mqtt_utk.c`,
  `cyw43_utk.c`). Ask the instructor before applying it. Without it the
  robot runs normally, but with no MQTT.
- **End of course.** Week 6 says "end barcode or remote STOP". Set
  `S1_BARCODE_END_CHAR` once the lecturer names the end barcode. Until
  then, use the dashboard's `pause`.
- **Obstacle trigger** is 40 cm (Buddy 5's value; Week 6 said 20 cm).
- **Battery voltage** is not measured, as Week 6 NFR17 already says: on the
  Pico W, GP29 (VSYS/3) is the radio's, and GP28 is the barcode sensor.
- **Watchdog** (NFR19) is on: the robot resets if the control loop stalls
  for 500 ms. Set `CTRL_WATCHDOG_ENABLE` to 0 in `ctrl_task.c` only while
  debugging with breakpoints.
- **Lift / overturn stop** (17 Nov, replaces collision detection): see
  `IS25_COMPLIANCE.md` (17 Nov section) and `imu.h`. Check it on the car
  with the CHECK=3 image (S3-M07..M09) before the demo.
- **IR sensor height.** The 3 mm barcode bars need a sharp sensor: in
  simulation, a spot up to 4 mm read `*A*`, a 6 mm spot did not and gave a
  false junction. Mount the IR board low and check S1-U14.
- **Coarse encoder.** Under ~150 edges per wheel turn, set
  `GLUE_ACC_WINDOW` to 20 (S3), or false humps make TSK_CTRL ignore far
  obstacles (`BUDDY4_INTEGRATION.md`).

## AI-use note

This code was written with AI assistance from the team's Week 6 and
Tribunal 1 documents, and integrates Zi Siang's and Buddy 5's own code.
Declare it as the module requires, and review and own every line of your
subsystem before you submit it.
