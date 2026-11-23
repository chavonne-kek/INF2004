# Subsystem 4 — test log

**Subsystem:** Ultrasonic + servo scan, vehicle controller  ·  **Owner:** Soma  ·  **Student ID:** 2402033  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | flg_events / flg_faults created | IF-03 | auto | `subsystem4_test.c` | PASS |
| S4-U02 | HC-SR04 echo seen, range measured | FR11 | auto | `subsystem4_test.c` (point the sensor at something < 1 m) | PASS |
| S4-U03 | Fault bit set/clear 100 times | CR1, IF-03c | auto | `subsystem4_test.c` | PASS |
| S4-U04 | Telemetry profile 35 bits round trip | CR1 | auto | `subsystem4_test.c` | PASS |
| S4-U05 | Echo width -> cm, out of range = 127 | NFR7 | auto | `subsystem4_test.c` | PASS |
| S4-U06 | Buddy 5 engine idle and watching (robot idle: control refused) | IF-05 | auto | `subsystem4_test.c` | PASS |
| S4-U07 | Buddy 5 geometry with integer maths | FR12 | auto | `subsystem4_test.c` | PASS |
| S4-M01 | Range accuracy 2–100 cm | NFR7 | manual | Board at 5, 20, 50, 100 cm, 10 readings each. | ±2 cm |
| S4-M02 | Servo angle calibration | FR12 | manual | Check 0/90/180°; set `B5_SERVO_PULSE_MIN/MAX_US`; confirm 0° = right. | ±3° |
| S4-M03 | Coarse sweep time | NFR7 | manual | Time the 5-angle sweep. | ≤ 1 s |
| S4-M04 | Width estimate, 3 shapes x 3 distances x 5 | NFR8, A7 | manual | Compare the published profile with a ruler. | ≤ 20% |
| S4-M05 | Stop chain with WiFi on (scope GPIO) | NFR4 | manual | Obstacle confirmed -> motor pins braked. | < 50 ms after the confirming echo |
| S4-M06 | Full avoidance: box on the line, both sides, 5 runs each | FR13, FR14, NFR8 | manual | Drive into a box; robot stops, scans, bypasses, finds the line. | Bypass + reacquire < 5 s, no contact |
| S4-M07 | Start/reset buttons, buzzer, watchdog | NFR19 | manual | Press buttons; set `CTRL_WATCHDOG_ENABLE` 1 and stop TSK_MOTION. | Reset in ≤ 0.5 s |
| H-SC01..03 | Echo conversion, index, telemetry packing | CR1 | host | `testing/subsystem4/host` | 3/3 PASS |
| B5 host | Buddy 5's profiler, planner, packing, status word (9 scenes) | FR12, FR13, CR1 | host | `testing/subsystem4/buddy5_host` | ALL PASSED |
| S4-M08 | HC-SR04 characterisation on this car (3.3 V supply) | NFR7, NFR20, FR11 | manual | Car in IDLE, `IS25_CHECK_4` image or dashboard `range`: nothing in front, box at 50, 30, 20 cm, box touching. | Record what the module reports (a 5 V HC-SR04 would read the true distance ±2 cm) |
| S4-M09 | 1 cm rule (v94n): leaky count, `B5_ONE_CM_COUNT` 3 | FR11 | manual | Box ahead at 30, 20, 10 cm (stationary watch) and moving toward the box; empty track 2 min. | Fires only when the box is close; never on the empty track |
*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**host** = PC test in `host/` (`make run`); **manual** = a person observes
the result (scope, ruler, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-24 | H-SC01..08 | (not committed yet) | skeleton author | PASS (8/8) | `logs/2026-10-24_host.txt` | PC run, gcc, simulated inputs only (old scan code). |
| 2026-10-26 | H-SC01..03, B5 host | (not committed yet) | integration | PASS | `logs/2026-10-26_host.txt` | After merging Buddy 5's engine. |
| YYYY-MM-DD | S4-U01..05 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | First on-target run: owner to fill in. |
| 2026-11-23 | H-SC01..03 | v94n | PC host run | PASS (3/3) | `logs/2026-11-23_host_v94n.txt` | |
| 2026-11-23 | B5 host | v94n | PC host run | ALL PASSED | `logs/2026-11-23_buddy5_host_v94n.txt` | |
| 2026-11-23 | S4-M08 | v94 | team | Characterised: module does not measure distance | team observation, no log | Nothing in front: steady 24 cm (floor). Box at 20 cm: 3 cm (then 24). Box touching: 1 cm. Object seen from ~50 cm. Matches the datasheet: HC-SR04 is a 5 V part, run here at 3.3 V. Width (S4-M04) cannot be measured with it |
| 2026-11-23 | S4-M09 | v94n | | PASS / FAIL | | Fill in before the demo |
