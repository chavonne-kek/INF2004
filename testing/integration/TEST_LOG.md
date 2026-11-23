# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | Line following on a straight and a curve | IF-01 (S1 + S2) | manual | FOLLOW 15 cm/s on the mock track, 10 laps. | ≤ 1 false LINE_LOST per lap (NFR5) |
| INT-02 | Barcode -> turn at the next junction | IF-03b, IF-04 (S1 + S4 + S2) | manual | Barcodes A, B, C, D before a junction. | Correct turn each time |
| INT-03 | Obstacle stop with WiFi on | IF-03, IF-04 (S4 + S2) | manual | Scope: echo edge to motor brake, 10 runs (Version B test 1). | < 50 ms |
| INT-04 | Fault bits from both cores, 10,000 times | IF-03c (S2 + S3 + S4) | manual | Temporary test code sets different bits from tasks on both cores (Version B test 2). | TSK_CTRL sees every bit |
| INT-05 | Odometry read while written; humps of known height | IF-02, IF-09 (S2 + S3) | manual | Version B test 3. | No impossible values; ±15% |
| INT-06 | Known values over MQTT; 20 bad commands | IF-06, IF-07, IF-08 (S5 + all) | manual | Version B test 4. | Dashboard = serial log; bad commands change nothing |
| INT-07 | Buddy 5 hand-over: take / move / release, and abort | IF-05, IF-04 (S4 + S2) | manual | Box on the line: avoidance completes. Then repeat and press pause mid-bypass. | Completes; on pause the car brakes, B5 aborts, no further motion |
*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-28 | SIM (all auto tests) | 28 Oct code | firmware simulation | 42/42 PASS | `logs/sim_2026-10-28/fin_a_test_image_wires_swapped.log` | Simulated car (`tools/sim`), single-core build, encoder wires on GP1/GP2 on purpose. Not a car test |
| 2026-10-28 | SIM (27 Oct fault) | 28 Oct code | firmware simulation | Reproduced | `logs/sim_2026-10-28/fin_c_normal_battery_off_no_imu.log` | Battery off + no IMU gives the 27 Oct FAULT (STALL + IMU), now with the reasons printed |
| 2026-10-28 | SIM (INT-03 logic) | 28 Oct code | firmware simulation | 0.14 s to OBSTACLE_AVOID | `logs/sim_2026-10-28/fin_e_normal_obstacle_480_edges.log` | 480-edge encoder; with the 40-edge placeholder false humps delayed it to 1.2 s (`IS25_COMPLIANCE.md` 0) |
| 2026-10-29 | Test image on the car | 28 Oct image | team (log from Hao Ming) | 39/42 | console log (car) | S3-U01, U02, U06 FAIL; S2-M01 no encoder edge on GP0–GP3. Explained in `IS25_COMPLIANCE.md` 0a |
| 2026-10-29 | SIM (car result) | 28 Oct image | firmware simulation | Reproduced | `logs/sim_2026-10-29/g1_car_29oct_reproduced_28oct_image.log` | GY-511 on the bus without power + wheels not turned gives the car's 39/42 exactly |
| 2026-10-29 | SIM (car case, fixed) | 29 Oct code | firmware simulation | 40/42 | `logs/sim_2026-10-29/g2_car_29oct_with_29oct_image.log` | Only S3-U01/U02 FAIL (no sensor); I2C check names the held-low lines; encoder hint printed |
| 2026-10-29 | SIM (all auto tests) | 29 Oct code | firmware simulation | 42/42 PASS | `logs/sim_2026-10-29/a_test_image_wires_swapped.log` | Encoder wires on GP1/GP2 on purpose |
| 2026-10-29 | SIM (S3-M06 logic) | 29 Oct code | firmware simulation | PASS | `logs/sim_2026-10-29/i_normal_imu_unplugged_and_back.log` | Unplug: `IMU read failing` after 50 ms; plug back: `IMU back` within 0.3 s |
| 2026-10-30 | SIM (check images 1–5) | 30 Oct code | firmware simulation | All behave as designed | `logs/sim_2026-10-30/` (10 logs) | Good and faulty wiring for each subsystem; see `tools/sim/README.md` |
| 2026-10-30 | SIM (test image) | 30 Oct code | firmware simulation | 42/42 PASS | `logs/sim_2026-10-30/test_image_all_pass_start_ignored.log` | START after the tests is counted; the car does not drive |
| 2026-10-30 | SIM (GP21 reset fix) | 30 Oct afternoon code | firmware simulation | Fixed | `logs/sim_2026-10-30/reset_after_encoder_fault_before_fix.log`, `…_after_fix.log` | Old image: after GP21 the ENCODER fault stays and the wheels never move (the car's 30 Oct log); new image drives |
| 2026-10-30 | SIM (360 edges per turn) | 30 Oct afternoon code | firmware simulation | PASS | `logs/sim_2026-10-30/normal_360_edges_follow_and_obstacle.log` | Calibrates, follows, OBSTACLE_AVOID; 43/43 test image; START refused without IR calibration |
| 2026-10-30 | SIM (start-up sweep 60/120/60) | 30 Oct code | firmware simulation | PASS | `logs/sim_2026-10-30/sweep_*` | Replaces the 360° spin; IR calibration OK with the sensors 60, 30 and 20 mm ahead of the axle; ends facing the line; then follows and stops for an obstacle |
| 2026-10-30 | SIM (sensor side) | 30 Oct code | firmware simulation | PASS | `logs/sim_2026-10-30/autostart_*` | Re-run 16:00 from clean builds (the 15:35 runs used a stale simulator build and were removed): each layout follows with its setting, 0 line losses in 13 s; the wrong setting circles |
| 2026-10-30 | SIM (slower speeds, auto-start, ultrasonic off) | 30 Oct 16:00 code | firmware simulation | PASS | `logs/sim_2026-10-30/autostart_*` | Calibrates, starts following by itself at 12 cm/s, does not stop for an object at 20 cm |
| 2026-11-17 | SIM (whole system) | `IS25_final_17nov` | firmware simulation | Test image 46/46; lift, overturn, obstacle bypass ×2, barcode turn, noisy drive, old chip: all as expected | `logs/sim_2026-11-17/` | LSM303D model, lift/overturn stop, ray-cast box obstacle. On the car: S3-M07..M09 and a full rubric run. |
