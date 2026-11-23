# IS25 final check (26 Oct, reviewed 28 and 29 Oct; integration 17 Nov)

## 17 Nov: whole-system integration against the demo rubric

Merged: the team's `IS25_final_02nov_imu` + Buddy 3's 3 Nov barcode work.
Then every subsystem was checked against the demo rubric and the
lecturer's change (IMU collision detection removed; the IMU stops the
motors when the robot is lifted or overturned; lifting ends the run).

**Why the IMU never answered.** The module is an **LSM303D**, not an
LSM303DLHC (GY-511). The two chips share a name but not a register map:
the DLHC has its accelerometer at 0x19 and magnetometer at 0x1E; the
LSM303D has both at one address (0x1D, or 0x1E with SA0 low), WHO_AM_I
0x0F = 0x49, 16-bit data. The old driver could never find it, which is the
`FAULT IMU` in every log since 27 Oct. `drv_lsm303.c` now finds an LSM303D
at 0x1D or 0x1E (WHO_AM_I checked) and still falls back to the DLHC.

| # | Demo rubric item (1 point each) | Where in the code | Evidence (17 Nov) |
|---|---|---|---|
| 1 | Detect barcode | `subsystem1/line_task.c` (arm sensor GP28, 1 ms) | S1-U05..08, H-BC01..14; simulator reads `*A*` while following |
| 2 | Decode barcode | `subsystem1/barcode.c` (Code 39, either direction) | as 1; rejects reported with a reason, never a wrong letter |
| 3 | Measure hump height | `subsystem3/imu.c` (integral of sin(pitch) over distance) | H-IM02/03/21 (±15 % at 2–20 cm/s), S3-U08. **Fixed:** the 4 s ramp guard dropped real humps at the 3 cm/s demo speed; now 600 mm of travel |
| 4 | Follow line | `subsystem1/line_sense.c` + `subsystem2` PID | H-LS*, simulator: follows at 3 cm/s, error ±2 mm |
| 5 | Follow barcode instruction | `ctrl_task.c` (command stored, executed at the next junction; map from the dashboard) | simulator: `A` → left turn at the junction, then follows the branch |
| 6 | Detect obstacle | `scan_task.c` / `b5_task.c` (HC-SR04 on the servo, 40 cm) | simulator with a box: OBSTACLE_AVOID 0.1 s after 40 cm. **Fixed:** `CTRL_OBSTACLE_ENABLE` was still 0 (line tests) |
| 7 | Measure obstacle width | `b5_scan.c`, `b5_profile.c` → `robot/telemetry/obstacle` | Buddy 5 host tests; width on the web UI |
| 8 | Go around and rejoin the line | `b5_manoeuvre.c`, `ctrl_task.c` | simulator: 10 cm and 16 cm boxes passed, back on the line (y = 9 mm, heading 0°) |
| 9 | Telemetry via MQTT | `subsystem5/` + IF-07 patch | H-CO*, round trip 2532 vectors (C ↔ dashboard), web UI live test |
| 10 | Stop the motors when picked up | `imu.c` `lift_step()`, `imu_task.c`, `ctrl_task.c` | H-IM19..25, H-B4 T7/T14–T16, S3-U12..16; simulator: lift → STOP in 0.12 s, FINISHED |

**Lift / overturn rule** (`imu.h` has every number):

- *Lifted:* a robot held still in the air reads exactly 1 g, so the lift is
  caught while it happens. The acceleration along gravity, minus gravity,
  is integrated into an upward speed (leak 0.5 s). Horizontal pushes
  (braking, turning, the obstacle) are at right angles to g and add
  nothing; hump slopes only rotate the vector. The car cannot rise faster
  than about 40 mm/s on the course; a hand gives 200–600 mm/s. Threshold
  150 mm/s for 50 ms, or 5 × the measured noise if the chassis vibrates
  more. Simulated: no false stop in 21 five-minute runs at 10–110 mg rms
  vibration; lifts caught in 0.1–0.35 s up to 70 mg rms.
- *Overturned:* less than half of g on the up axis (tilt over 60°, any
  side, or upside down) for 200 ms. Humps are under 30°.
- TSK_IMU sends STOP itself in the same 10 ms cycle, then raises
  FAULT_LIFTED (bit 5, was COLLISION). TSK_CTRL ends the run (FINISHED).
  The fault stays until the next run starts (`subsystem3_lift_rearm()` in
  `start_run`), so a car held in the air cannot drive off.

**Other fixes found in the review:** S5 accepted gain 26–31 that TSK_CTRL
then silently clamped (now rejected, matches the dashboard); the C ↔
dashboard round-trip generator read past its table for GAIN; the S5 test
vector for "speed below the minimum" used the old minimum; the S5 status
line still said IMPACT; the check image scanned I2C from another task while
TSK_IMU was using the bus (now done inside TSK_IMU).


The code was checked against everything in the project pack: the Project
Briefing and Write-up, the Barcode & Line specification and samples, the
Week 6 design review and the lecturer's feedback, the Tribunal 1 report
(PostTri), and the datasheets (Robo Pico, Pico W, TCRT5000 / IR module,
LM393, HC-SR04, LSM303DLHC / GY-511, encoder motor). Status: **✓** in the
code and tested on a PC, **car** = in the code but only the car can
prove it, **open** = a team decision.

## 0. Review after the first car run (28 Oct)

The 27 Oct console showed `state=8` (FAULT), `faults=0x15`, `link=2`,
`pitch -128/4`, `t=3356011s` and `mqtt: TX … "hb 22"`. Every line of the
application code was read again, and the whole firmware was run on a
simulated car (`tools/sim/`, logs in
`testing/integration/logs/sim_2026-10-28/`).

**What the 27 Oct log means.** `0x15` = STALL + IMU + WIFI.

- **STALL:** in the start-up spin both motors were driven but neither
  encoder gave a single pulse for 0.5 s (odometry 0 in every line). So the
  wheels did not turn (battery switch, motor power), or they turned and the
  encoders were not powered or not on Grove 1/2. The calibration then
  failed, hence FAULT. The simulation with the battery off and no IMU
  gives exactly this log.
- **IMU:** the GY-511 never answered on I2C0 (GP16/GP17). The driver,
  pins and addresses were checked against the RP2040 function table and
  the LSM303DLHC datasheet; in simulation the same code reads the sensor.
  So: power, ground, or SDA/SCL swapped (the old `WIRING_RECONNECT.md`
  step 1 said yellow→SDA, which is the wrong way round for this kit).
- **WIFI / `hb 22`:** the image was built with `MQTT=1` but **without** the
  IF-07 patch, so the kernel's demo ran instead of the robot's topics and
  TSK_COMMS stayed OFFLINE.
- **`pitch -128/4`, `t=3356011s`:** display bugs, fixed below.

| # | Found | Fix |
|---|---|---|
| R1 | **Wrong clock.** `tk_get_tim()` is the calendar clock (TRON epoch), which starts at about 3.36 million seconds, not 0. The console `t=` and the heartbeat **uptime** sent to the dashboard were wrong | `tk_get_otm()` (time since start-up) in `ctrl_task.c` and `comms_task.c`; buzzer timeout made wrap-safe |
| R2 | **IMU status before any reading** was all zero, which decodes as pitch −32° | "No data" status (hump INVALID, pitch 0, heading unknown) at start and while FAULT_IMU; console prints `IMU n/a`; S3-U02 also needs real data |
| R3 | **MQTT build without IF-07** built silently and ran the wrong program | The makefile stops `MQTT=1` builds until the patch is applied, with the command to apply it |
| R4 | **S1-U05 would fail on the Pico**: at the slowest speed the test fed 60 white samples after the barcode, the decoder needs 100 to see the end | Test feeds the Code 39 quiet zone (10 narrow widths). Found by running the test image in simulation |
| R5 | **S2-M01 "15 s" lasted about 30 s**: `tk_dly_tsk(1)` can take 2 ticks | Each second is now paced by the 1 MHz timer |
| R6 | **Encoder wire uncertainty** (right encoder GP2 or GP3, build log) | Both pins of each encoder's Grove port are watched; each wheel is counted on the pin with pulses (`drv_encoder.c`). Simulated with both encoders on the *other* wire: 42/42 PASS |
| R7 | **I2C bus can lock after a Pico reset** (the GY-511 keeps 3V3 power; a reset mid-read leaves SDA low until its power is cycled; the watchdog makes this more likely) | Bus clear (9 clocks + STOP) before every open and re-probe; one-line wiring check on the console when the sensor does not answer (`drv_lsm303.c`) |
| R8 | **Messages did not say why** (state and faults as numbers, `S2: stall`, calibration "FAILED"); S1-U12 pointed to a raw-IR console line that did not exist | Names for states, faults and link; STALL / ENCODER messages with duty and edges per pin; calibration result with each IR sensor's span; the 2 s status line shows the raw IR readings (`IR l c r`) |
| R9 | `GLUE_MG_PER_MM_S2_NUM` was fixed at 20000 (only right for a 5-sample window) | Derived from `GLUE_ACC_WINDOW` |

**Tuning findings from the simulation (team decisions, documented, not
changed):**

- With the 40-edge placeholder encoder, S2's speed steps (≈ 50 mm/s) made
  S3's encoder acceleration jump by ≈ 0.1 g: false 6° pitch, false humps,
  and TSK_CTRL ignored obstacles over 10 cm for up to 1.2 s. With a
  480-edge encoder, or `GLUE_ACC_WINDOW` 20, none (obstacle stop in
  0.14 s). The test image now warns below 150 edges per turn.
- 3 mm barcode bars were read with an IR spot up to 4 mm, not with 6 mm
  (also a false right-branch junction). Mount the IR board low.
- `MOTION_EDGES_PER_REV` must be measured before the first normal run.
- The motor and steering gains are placeholders: in simulation the car
  follows a straight line but weaves; tune on the track.

## 0a. Test image on the car (29 Oct)

The car ran the 28 Oct test image: 39 of 42 PASS. S3-U01, S3-U02 and
S3-U06 failed, and `[CAL] S2-M01` saw no encoder edge on GP0–GP3 at all.

- **Encoders: hardware.** Not one level change on any of the four pins,
  counted by interrupt *and* by polling every 1 ms. The pins use the same
  input set-up as the HC-SR04 echo on GP5, which passed (S4-U02). So the
  signal never reached the Pico: wheels not turned during the 15 s, or
  the encoders not powered / not plugged in. On 26 Oct the Grove 2 status
  LED blinked when the right wheel was turned (build log).
- **S3-U01/U02: hardware**, the GY-511 still does not answer.
- **S3-U06: a consequence of the IMU wiring, made worse by the code.** In
  simulation, a GY-511 that sits on the bus *without power* (its own
  pull-ups then hold SDA and SCL low) gives exactly the car's result with
  the 28 Oct image (`tools/sim` run `g1`): every I2C transfer waited for
  the kernel driver's 1 s timeout, so TSK_IMU managed about one cycle per
  second and S3-U06, which starts while TSK_IMU is inside such a wait,
  counted too few cycles. A crossed SDA/SCL pair can also stall the bus.

| # | Found | Fix |
|---|---|---|
| R10 | A missing or miswired GY-511 stalled TSK_IMU for 1 s per transfer (kernel driver timeout) | `drv_lsm303_open()` first checks with a bit-banged address byte that the accelerometer answers (cannot hang); every transfer first checks that both lines are high; while FAULT_IMU is set the sensor is not read, only re-probed once a second (`imu_task.c`), and a failed boot open raises FAULT_IMU at once. Simulated: S3-U06 PASS with the module unpowered, unplug/replug gives `IMU read failing` / `IMU back` |
| R11 | S3-U06 timed its 1 s window from the call that starts the synthetic sensor, not from when TSK_IMU uses it | Waits until TSK_IMU has run two generator cycles (up to 3 s), then counts |
| R12 | `S4: faults …` printed on every fault event, also when nothing changed (S4-U03 toggles NO_ECHO 100 times: 23 lines on the car, 20 of them the same) | Printed only when the mask differs from the last line |
| R13 | No hint when S2-M01 sees no pulse at all | Three `[CAL]` lines: were the wheels turned; watch the Grove 1/2 status LEDs; power and plug |

## 0b. One subsystem at a time (30 Oct)

To find wiring faults one subsystem at a time, `make CHECK=n` (n = 1..5)
builds a check image: subsystem n's self-tests only, then a live readout
of its hardware every 0.5 s (`app_program/subsystemN/subsystemN_check.c`,
steps in `docs/IS25_Subsystem_Checks.txt`). The other subsystems run as
usual underneath. All five were run on the simulated car with good and
faulty wiring (`testing/integration/logs/sim_2026-10-30/`).

| Check | Live readout | Hooks added (test builds only) |
|---|---|---|
| 1 IR + barcode | Hand calibration sweep with spans; raw L/C/R, error as a steering direction, samples/s, barcode counters | — |
| 2 Motors + encoders | Level and edges of GP0–GP3; on START, each motor alone forward/backward 2 s at 50 % with the edges each encoder counted | `motion_task_test_drive()` (open loop while BRAKED, no PID, no stall check) |
| 3 IMU | Compass calibration by hand; raw acc/mag, pitch, heading, read counters; wiring check every 3 s while there is no data | `imu_task_raw_get()`, `imu_task_request_check()` (TSK_IMU runs the check between transfers) |
| 4 Ultrasonic, servo, buttons | Servo sweep front/right/left/front; range, NO_ECHO, presses | `ctrl_task_presses()` |
| 5 Comms | The 2 s status line (normally off in test builds); with MQTT, link and dashboard commands | — |

**Bug found on the car (30 Oct, after a GP21 reset):** the log showed
IDLE, then LINE_FOLLOW with the ENCODER fault still set and both wheels at
0 mm/s until LINE_RECOVERY timed out. Two faults in `motion_task.c`:
(1) TSK_CTRL's reset sends RESET and then, entering IDLE, STOP in the same
10 ms cycle, and the command queue kept only the newest, so STOP replaced
RESET; (2) even alone, RESET called `brake_now()`, which by design stays
in STALLED, so TSK_MOTION never left it and ignored every later command.
Fix: RESET (like SPEED) is applied at once and never replaced, and it
enters BRAKED itself. New test S2-U09; reproduced with the old image and
fixed with the new one in simulation.

Two follow-ups the same day: with the measured 360 edges per turn
(`MOTION_EDGES_PER_REV`, set in `motion_calc.h`) one edge is 0.57 mm, and
the straight-line trim worked in whole millimetres, so it ignored a
one-edge difference (host test H-MO11 failed): it now works in
micrometres. And after GP21 from a failed start-up calibration the IR
levels are defaults, so the car circled in simulation: START is now
refused with `S4: START refused - IR not calibrated …` until a RUN/RESET
calibrates.

**Sensors as built (30 Oct): barcode sensor on either side.** The car has
two line sensors side by side and the barcode sensor further out on one
side. `S1_TRACK_RIGHT_EDGE` (`subsystem1.h`) selects which line edge the
inner sensor rides on; 1 flips only the sign of the line error. Simulated
both ways (`logs/sim_2026-10-30/autostart_*`): with the matching setting the
car follows and reads `*B*`; with the wrong one it circles, which the S1
check image shows at once ("steer" toward the wrong side).

**Line-test mode (30 Oct 16:00, team request).** `CTRL_AUTO_START` 1:
the car starts following as soon as the start-up calibration succeeds (no
GP20; GP20 still restarts it from IDLE). `CTRL_OBSTACLE_ENABLE` 0: TSK_CTRL
refuses every obstacle hand-over, so the ultrasonic is ignored. Set it
back to 1 for the full demo. Two bugs found while simulating this: (1)
going to IDLE sends STOP, and a STOP queued in the same cycle as FOLLOW
wins, so the auto-start goes straight to following; (2) while following,
a wheel's target could go negative (hard correction), but with phase A
only the measured speed always has the commanded (+) sign, so the PI loop
drove that wheel backwards at full power and the car span. Following
targets are now clamped at 0 (`MOTION_FOLLOW_MIN_MM_S`): a hard correction
pivots on the slow wheel.

**Start-up beep off (30 Oct, team request):** `CTRL_STARTUP_BEEP` 0 in
`ctrl_task.c`; the 2 s wait before the sweep stays. The other beeps
(barcode, START, faults) are unchanged.

**Slower first runs (30 Oct, team request).** Following 12 cm/s (was
20, `CTRL_SPEED_DEFAULT_CMS`), line search and after a bypass 8 cm/s (was
10), turns and the start-up sweep 8 cm/s (was 12,
`MOTION_TURN_SPEED_MM_S`). Raise them again once the steering is tuned;
the dashboard's `speed` command still sets 5–25 cm/s.

**Start-up sweep instead of a 360° spin (30 Oct, team request).** The
car now turns 60° left, 120° right and 60° left (`CTRL_CAL_SWEEP_DEG`),
so it ends facing the line as it started and calibrates in about 6.4 s
instead of 7.3 s. One direction alone would not do: turning left only
moves the left sensor away from the line. Simulated with the sensors 60,
30 and 20 mm ahead of the axle: all three spans OK each time
(`logs/sim_2026-10-30/sweep_*`). The compass now sees 120° rather than a
full circle, so the heading is approximate; it is only reported, never
used for steering.

Also changed: in every test image (full or check) TSK_CTRL only counts
START (button or dashboard) and beeps; it no longer starts line
following, so the "does not drive" images really never drive. RESET
still clears FAULT. Every dashboard command is printed
(`S4: dashboard command …`), which S5-M03 needs.

## Fixed in the 26 Oct check

| # | Found | Source | Fix |
|---|---|---|---|
| 1 | Watchdog was off, and its start came *before* the 2 s start-up beep, so switching it on would have reset the robot at every boot | Week 6 NFR19 | Started after the start-up wait, then fed every ≤ 50 ms while TSK_MOTION runs; now on (`ctrl_task.c`) |
| 2 | HC-SR04 pings were echo time + 15 ms apart (≈ 17 ms at 30 cm) | HC-SR04 datasheet ("over 60 ms measurement cycle"), Week 6 NFR7, PostTri | ≥ 60 ms from trigger to trigger (`b5_sonar.c`, `B5_SONAR_MIN_CYCLE_MS`) |
| 3 | A branch to the **right only** was never a junction (left-edge tracking keeps the left sensor white), so a "B = turn right" barcode could never execute there | Week 6 IF-01: "…or when the right sensor stays black longer than any barcode bar" | Right sensor black ≥ 14 mm of travel (odometry) with the centre on its edge = JUNCTION (`line_sense.c`); host tests H-LS10..12 |
| 4 | The phase-B encoder fallback only set pull-ups; nothing read phase B | Feedback (phase A miscounts braking/reversing; GP6/GP7), Week 6 A2, PostTri R3 | Quadrature direction in the interrupt, signed counts, measured direction for the speed sign (`drv_encoder.c/.h`, `motion_task.c`); compiles clean both ways |
| 5 | Barcode reject counter not in telemetry | Week 6 statechart 6.2 ("an error counter is sent in telemetry") | Third byte of `robot/telemetry/comms`; dashboard and round trip updated |
| 6 | Turn rate not reported | Project Briefing, Buddy 4 ("report turn rate") | Dashboard shows turn rate from the wheel speeds; heading and TURNING class are in the IMU word |

## As-built wiring (build log, 26 Oct)

The car was wired on 26 Oct and the code follows it. Differences from the
Week 6 pin table:

| Item | Week 6 | Car | Code |
|---|---|---|---|
| Left encoder | GP2 (both encoders on Grove 2) | **GP0, Grove 1** (one Grove port has only one 3V3/GND pair) | `S2_PIN_ENC_L_A` = 0; builds with `CRASH_REPORT=1` / `CONSOLE=uart` now stop with an error, because UART0 uses GP0/GP1 |
| Phase-B fallback pins | GP6/GP7 | GP1 (Grove 1) / GP2 (Grove 2), taped | `S2_PIN_ENC_L_B` = 1, `S2_PIN_ENC_R_B` = 2 |
| HC-SR04 supply | 5 V + 1k/2k divider | 3V3, no divider (echo ≤ 3.3 V, NFR20 still met) | No change. The datasheet rates 5 V: if ranges drop out, use 5 V **with** the divider |
| Right encoder | GP3 | Logged as GP3, but on the **white** wire, which the kit's colour rule makes GP2 | Resolved 28 Oct: either pin works (R6) |
| Grove colour order | — | white = signal 1, yellow = signal 2, red = 3V3, black = GND | Recorded in `IS25_README.md` and `TEAM.md` |

## Functional requirements (Week 6)

| FR | Requirement | Where | Status |
|---|---|---|---|
| FR1 | Follow the 18 mm line, left + centre IR | `line_sense.c` (left-edge tracking), `motion_task.c` FOLLOW | ✓ host, car |
| FR2 | Calibrate IR, accelerometer zero, 360° magnetometer spin | `ctrl_task.c` INIT_CALIBRATE, `line_task.c`, `imu_task.c` | ✓ |
| FR3 | Line lost / found / junction events | `line_sense.c`, H-LS03..06, H-LS10..12 | ✓ |
| FR4 | Code 39 with `*`, right sensor, either direction | `barcode.c`, H-BC01..11 | ✓ |
| FR5 | A/B/C/D at the next junction; others reported only | `barcode_char_to_cmd`, `ctrl_task.c` | ✓ (end barcode: open) |
| FR6 | PID, move/turn/stop API, stall detection | `motion_calc.c`, `subsystem2_move_forward()` … `_stop()`, `check_stall()` | ✓ host H-MO06..13 |
| FR7 | Speed and distance from encoders | `motion_task.c` read_encoders, odometry word | ✓ (edges/rev: car) |
| FR8 | Humps from pitch | `imu.c` (Zi Siang) via `imu_glue.c` | ✓ host H-IM02 |
| FR9 | Hump height and run maximum | hump record, `subsystem5_post` | ✓ host H-IM02/03 |
| FR10 | Motion classes; collision **replaced 17 Nov** by the lift / overturn stop (lecturer) | `imu.c` + glue (TURNING from wheel speeds), `lift_step()` | ✓ host + simulator; S3-M07/M08 on the car |
| FR11 | Detect obstacles, stop | `b5_task.c` watch, TSK_CTRL stop | ✓ (trigger 40 cm, see open) |
| FR12 | Coarse 30–150°, fine scan, profile | `b5_scan.c`, `b5_profile.c` | ✓ Buddy 5 tests |
| FR13 | Stop / continue / turn / reverse; bypass | `b5_plan.c`, `b5_manoeuvre.c` (5 actions, 3 bits) | ✓ Buddy 5 tests |
| FR14 | Reacquire the line | B5 recover + LINE_RECOVERY (5 s → FAULT) | ✓ / car |
| FR15 | Telemetry topics incl. link status | `comms_task.c`, `MQTT_PROTOCOL.md`, `robot/status` | ✓ round trip |
| FR16 | start / stop / e-stop / reset / speed; malformed ignored + logged | `comms_codec.c`, `handle_command()` (counted + printed) | ✓ host, D-01 |
| FR17 | Online / Last Will, 1 s heartbeat, reconnect | IF-07 patch, heartbeat, rejoin 5 s / MQTT retry 3 s | ✓ build; car + broker |

## Non-functional requirements (Week 6)

| NFR | Target | How it is met | Status |
|---|---|---|---|
| NFR1 | TSK_LINE every 2 ms, ≤ 0.5 ms | 1 ms tick, cyclic handler; S1-U16 measures it | car (test prints it) |
| NFR2 | PID every 10 ms, 20 kHz PWM, ±5 % | cyclic 10 ms; wrap 6249 at 125 MHz; S2-U08 timing | ✓ / car |
| NFR3 | 90° ±5°, 1 m ±2 cm | braking profile + straight-line trim (sim: −2.8 mm, 0.0°) | car |
| NFR4 | Stop ≤ 50 ms (local), ≤ 150 ms (MQTT) | STOP brakes in the same 10 ms cycle; TSK_CTRL wakes at once | car (scope) |
| NFR5 | Line found ≥ 99 % of cycles | left-edge error, hysteresis | car |
| NFR6 | Barcodes ≥ 95 % up to 25 cm/s | ratio decoding; host ≥ 98 % at 3–30 cm/s | ✓ host, car |
| NFR7 | ±2 cm, 2–100 cm, ≥ 60 ms cycle, sweep ≤ 1 s | µs/58, 10 µs trigger, **60 ms cycle (fixed)** | ✓ / car |
| NFR8 | Width ≤ 20 %, bypass < 5 s | Buddy 5 profile / planner | car |
| NFR9 | 100 Hz, ±4 g, σ < 1°, ±15 % | ODR 100 Hz, CTRL_REG4_A ±4 g HR | ✓ host, car |
| NFR10 | Telemetry ≤ 300 ms, 5 Hz | 200 ms period | ✓ design |
| NFR11–13 | Connect ≤ 10 s, reconnect ≤ 12 s, 16 offline events | CONNECTING 10 s, rejoin 5 s + MQTT 3 s, 16-entry ring | ✓ host ring |
| NFR14 | ≥ 98 % delivered | QoS 0 periodic, 1 events; dashboard `rates` | car + broker |
| NFR15 | IR calibration valid 30 min | calibration at start | car |
| NFR16 | Static objects, RAM ≤ 75 %, CPU ≤ 70 % | no malloc; static RAM ≈ 25 KB (83 KB with MQTT) of 264 KB | ✓ RAM; CPU: car |
| NFR17 | Supply voltage not measured | GP29 radio, GP28 barcode | ✓ as designed |
| NFR18 | Last Will ≤ 1.5 × keep-alive | keep-alive 5 s (PostTri change from 2 s) | ✓ |
| NFR19 | Watchdog 500 ms, fed only if TSK_MOTION runs | **now on (fixed)**; RP2040-E1 handled | ✓ |
| NFR20 | All pins 0–3.3 V | IR, encoders, IMU and HC-SR04 all on 3V3 (as built) | ✓ wiring |
| NFR21 | C, Barr-C, zero warnings | 0 warnings with -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow | ✓ |
| NFR22 | Dashboard shows data ≤ 500 ms | prints on arrival | ✓ |

## Lecturer feedback (Week 6)

| Point | Answer in the code |
|---|---|
| Pin map; keep the Maker Port empty | Pins match the Robo Pico Grove table; Maker Port and VBAT jumper notes in `IS25_README.md` |
| 1 ms tick and IF-07 need agreement; measure core 0 idle | 1 ms tick in `config.h`; IF-07 is a separate patch (open); task times measured by S1-U16 / S2-U08 |
| Two sensors on one side give a one-sided error | Left-edge tracking, signed error, last sign held (Tribunal 1) |
| Phase A miscounts; GP6/GP7 could carry phase B | Both edges counted; **working phase-B option (fixed)** |
| No gyro: pitch only valid at steady speed | Encoder acceleration subtracted (Zi Siang's `imu.c`, sign corrected) |
| Define the COLLISION threshold; avoid false positives | (17 Nov: collision removed.) Lift threshold: upward speed 150 mm/s or 5 × noise; braking, humps, knocks and vibration tested |
| Obstacles ignored while HUMP is set | Always stop below 10 cm; hump guard ends after 4 s |
| Where NFR4's 50 ms starts | From the confirming echo; STOP brakes in the same 10 ms cycle |
| FINE_SCAN edge not found | Buddy 5: a side with no edge counts as blocked |
| Undefined end of course | `S1_BARCODE_END_CHAR` (open until the lecturer names it); dashboard `pause` |
| No TURN_EXECUTE timeout | 8 s motion timeout → FAULT; LINE_RECOVERY 5 s → FAULT; START while the line is lost (e.g. after the start-up sweep) now goes to LINE_RECOVERY with that 5 s limit, not to LINE_FOLLOW with no limit (30 Oct) |
| MQTT STOP went straight to FINISHED | `pause` → PAUSED; `resume`; `reset` |
| No motion statechart | Statechart 6.5 in `motion_task.c` and `subsystem2/README.md` |
| 5 Hz vs 10 Hz; payloads, command syntax, speed range, "malformed" | 5 Hz; `MQTT_PROTOCOL.md`; 1-byte commands, 5–25 cm/s, exact validation |
| 2 s keep-alive gives false Offline | 5 s (Tribunal 1) |

## Tribunal 1 (minimum-bit decisions)

All agreed layouts are in the code with masks and shifts, one writer per
word, written in one store: line word 11 bits in 16 · odometry 29 bits in
32 · motion command 11 bits · heartbeat 29 bits · command 1 byte · pitch
8 + heading 9 + hump 3 + motion 3 · hump record 14 bits · obstacle
profile 35 bits in 5 bytes · state 4 bits · scan index 5 bits · faults in
`flg_faults`, cleared only by their owner. The robot ↔ dashboard round
trip checks every layout (2,275 checks).

## Datasheet values checked

| Part | Checked |
|---|---|
| Robo Pico | Grove 1–7 pins (Grove 1 = GP0/GP1, free when `CRASH_REPORT=0`), motor truth table (low/low = brake, high/high = coast), 20 kHz PWM, buttons GP20/21, buzzer GP22, servo ports GP12–15, Maker Port = GP2/3, VBAT on GP28 |
| Pico W | GP23/24/25/29 belong to the radio; ADC3 never used |
| LSM303D (the car's module, 17 Nov) | 0x1D (SA0 high) or 0x1E; WHO_AM_I 0x0F = 0x49; CTRL1 0x6F (100 Hz, BDU, XYZ); CTRL2 0xC8 (50 Hz anti-alias, ±4 g → 0.122 mg/LSB); CTRL5 0x70 (mag high resolution, 50 Hz); CTRL6 0x20 (±4 G); CTRL7 0 (continuous); accelerometer 0x28, magnetometer 0x08, both 16-bit little-endian X, Y, Z; bit 7 of the register = auto-increment |
| LSM303DLHC (fallback) | 0x19 / 0x1E; CTRL_REG1_A 0x57 (100 Hz, XYZ); CTRL_REG4_A 0x98 (BDU, ±4 g, HR → 2 mg/LSB); CRA_REG_M 0x14 (30 Hz); CRB_REG_M 0x20 (±1.3 G); MR_REG_M 0 (continuous); IRA = 0x48; magnetometer order X, Z, Y |
| HC-SR04 | 10 µs trigger, µs/58 = cm, **60 ms cycle**; datasheet supply 5 V (car runs it on 3V3: re-check range) |
| TCRT5000 / IR module | AO used (DO's LM393 has no hysteresis), 3.3 V supply, 0.2–15 mm range |
| Encoder motor | PH2.0-6P: 1 GND, 2 phase A, 3 phase B, 4 power, 5/6 motor |
| Servo | 50 Hz (1 MHz tick, wrap 19 999) on slice 7B; 500–2500 µs to verify on the real servo |

## Still open

- **End barcode**: which character ends the course (`S1_BARCODE_END_CHAR`).
- **IF-07 patch**: instructor approval (MQTT).
- **Obstacle trigger**: 40 cm (Buddy 5) vs 20 cm (Week 6).
- **Car measurements**: encoder edges (first), wheel/track size, PID and
  steering gains, IR levels and sensor height, axis signs, servo range,
  the lift test on the car (S3-M07/M08), `GLUE_ACC_WINDOW` if the encoder
  is coarse. Each owner's `TEST_LOG.md` lists them.
- **Why the IMU did not answer since 27 Oct**: answered 17 Nov: it is an
  LSM303D, which the DLHC driver could not find. Confirm on the car that
  the console prints `S3: IMU LSM303D at 0x1D` (or 0x1E).
