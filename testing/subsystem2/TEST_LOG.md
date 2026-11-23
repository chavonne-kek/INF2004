# Subsystem 2 — test log

**Subsystem:** Motors, encoders, PID, odometry  ·  **Owner:** Kai Xuan  ·  **Student ID:** 2501473  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | TSK_MOTION runs every 10 ms | NFR2 | auto | `subsystem2_test.c` | `[TEST] S2-U01 PASS` |
| S2-U02 | STOP leaves the car BRAKED | IF-04, NFR4 | auto | `subsystem2_test.c` | PASS |
| S2-U03 | Motion command 11 bits round trip | CR1, IF-04 | auto | `subsystem2_test.c` | PASS |
| S2-U04 | Odometry word packs, speeds saturate | CR1, IF-02 | auto | `subsystem2_test.c` | PASS |
| S2-U05 | Distance delta across the 15-bit wrap | IF-02, Version C | auto | `subsystem2_test.c` | PASS |
| S2-U06 | PID output sign and limit | FR6 | auto | `subsystem2_test.c` | PASS |
| S2-U07 | Braking profile and straight-line trim on the target CPU | FR6, NFR3 | auto | `subsystem2_test.c` | PASS |
| S2-U09 | RESET leaves STALLED and clears STALL/ENCODER, also when a STOP follows in the same cycle (GP21 path; failed on the car 30 Oct) | Statechart 6.5 | auto | `subsystem2_test.c` | `[TEST] S2-U09 PASS` |
| S2-U08 | Worst TSK_MOTION cycle time, 1 MHz timer | NFR2, Tribunal 1 timing gap | auto | `subsystem2_test.c`; prints `TSK_MOTION worst cycle N us` | ≤ 1000 µs |
| S2-M01 | Edges per wheel revolution (A2) | FR7, A2 | manual | Test image: when `[CAL] S2-M01` appears, turn each wheel exactly 10 times by hand (wheels coast for 15 s). Set `MOTION_EDGES_PER_REV` to the printed value before the first normal run. `[CAL] left wheel counted on GPx, right on GPy` shows which wire carries each encoder (either works). Simulated 28 Oct (46 edges/turn, wires on GP1/GP2): `left 46.0 / right 46.0`, counted on GP1/GP2. With no pulse at all it prints `[CAL] NO encoder pulse on GP0..GP3 at all` and what to check (29 Oct) | Value recorded; each wheel counted on a pin of its own port |
| S2-M02 | Motor direction | FR6 | manual | FOLLOW at 10 cm/s with the car on a stand: both wheels forward. Else set `MOTOR_INVERT_x`. | Both forward |
| S2-M03 | Speed hold | NFR2 | manual | FOLLOW 20 cm/s on the floor, log speeds 10 s; tune `MOTION_K*`. | Within ±5% |
| S2-M04 | 1 m straight, 10 x each way | NFR3 | manual | `MOVE_FWD 100` / `MOVE_BACK 100`, measure with a tape. | ±2 cm |
| S2-M05 | 10 x 90 degree turns | NFR3 | manual | `TURN_LEFT 90` x10; measure heading with a protractor. Tune `MOTION_TRACK_MM`. | ±5° each |
| S2-M06 | Stall detection | FR6 | manual | Hold one wheel while driving. | Brakes, FAULT_STALL in ≤ 0.6 s |
| H-MO01..09 | Packing, wrap, conversions, PI on a model wheel incl. a 40-edge encoder | CR1, FR6 | host | `testing/subsystem2/host` | 9/9 PASS |
| H-MO10..13 | isqrt and braking profile; straight trim; simulated 1 m MOVE with one motor 10 % weak (distance ±2 cm, heading ±5°); PID derivative | FR6, NFR3 | host | `testing/subsystem2/host` | 4/4 PASS |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**host** = PC test in `host/` (`make run`); **manual** = a person observes
the result (scope, ruler, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-24 | H-MO01..09 | (not committed yet) | skeleton author | PASS (9/9) | `logs/2026-10-24_host.txt` | PC run, gcc, simulated inputs only. |
| 2026-10-26 | H-MO01..13 | (not committed yet) | final code | PASS (13/13) | `logs/2026-10-26_host.txt` | Simulated 1 m MOVE: −2.8 mm, 0.0° (4.8° without the straight-line trim). |
| YYYY-MM-DD | S2-U01..08 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | First on-target run: owner to fill in. |
| 2026-10-30 | GP21 reset after ENCODER fault | 30 Oct image | team (car) | FAIL | console log (car) | After GP21: IDLE, then LINE_FOLLOW with ENCODER still set and L 0 R 0 mm/s: the reset was lost. Fixed (S2-U09); reproduced and fixed in simulation (`logs/sim_2026-10-30/reset_after_encoder_fault_*`) |
| 2026-10-29 | S2-U01..08, S2-M01 | 28 Oct image | team (car) | 8/8 PASS; S2-M01 0 edges on GP0–GP3 | console log (car) | No level change on any encoder pin (interrupts and 1 ms polling). Hardware: wheels not turned in the 15 s, or encoders unpowered/unplugged. Repeat S2-M01 |
| 2026-11-23 | H-MO01..13 | v94n | PC host run | PASS (13/13) | `logs/2026-11-23_host_v94n.txt` | |

### PID tuning report and motion accuracy (fill in on the car)

| Step | KP / KI / KD (milli) | Result (speed error, overshoot, settling) | Kept? |
|---|---|---|---|
| Start | 15000 / 40000 / 0 | | |
| | | | |

| Test | Run 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | Mean | Worst | NFR3 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 m forward, error mm | | | | | | | | | | | | | ±20 |
| 1 m backward, error mm | | | | | | | | | | | | | ±20 |
| 90° left, error ° | | | | | | | | | | | | | ±5 |
| 90° right, error ° | | | | | | | | | | | | | ±5 |
