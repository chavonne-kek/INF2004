# System — test log

Tests of the complete car against its requirements and the Week 13 demo rubric, on the
final hardware (Robo Pico + Pico W, track mat, 13 × 13 × 13 cm box, phone hotspot
"The network", laptop broker). Run the full set before the demo and record every run.

See `testing/README.md` for how to run tests and record results, `testing/USE_CASES.md`
for the scenarios these tests check, and `testing/TRACEABILITY.md` for requirement coverage.

## Test cases

| ID | What is tested | Requirement / rubric | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | Power-on, zeroing and IDLE | FR2, NFR11 | manual | Power on on the track, hands off. Watch console + dashboard. | 2 s wait, 1 s still, state INIT_CALIBRATE → IDLE; `robot/status` = online within 10 s |
| SYS-02 | Start and follow the line | FR1, FR19, NFR2, NFR5 · rubric 4 | manual | Press START (GP20) or dashboard Start. Let it follow the full straight and curve. | LINE_FOLLOW; stays on the line; lateral deviation ≤ 2 cm on straights; no FAULT |
| SYS-03 | Detect a barcode | FR4 · rubric 1 | manual | Follow over the course barcode. | `robot/telemetry/barcode` event appears once per pass |
| SYS-04 | Decode the barcode, both directions | FR4, NFR6 · rubric 2 | manual | Pass the barcode forwards, then turn the car round and pass it backwards. | Same character both ways (e.g. X), never a wrong character |
| SYS-05 | Follow the barcode instruction | FR5 · rubric 5 | manual | Map the course letter (e.g. Z → left) over MQTT; drive to the + junction. | Creeps until the wheels are on the crossing (~139 mm), pivots ~90° the mapped way, follows the branch |
| SYS-06 | Measure hump height | FR8, FR9, NFR9 · rubric 3 | manual | Drive over the course hump; measure its real height with a ruler. | `robot/telemetry/hump` event within ±15 % of the ruler height; no hump on flat track |
| SYS-07 | Detect the obstacle | FR11 (v94n) · rubric 6 | manual | Box on the line ahead. | Car stops in front of the box without touching it; console `S4: watch fired`; state OBSTACLE_AVOID |
| SYS-08 | Obstacle width | FR12 · rubric 7 | manual | Read the `robot/telemetry/obstacle` event. | Width reported ≤ 20 % error — **known limitation:** on this HC-SR04 (3.3 V) the stopgap path cannot measure width (see S4-M08) |
| SYS-09 | Go round the box and rejoin | FR13, FR14, NFR8 · rubric 8 | manual | After SYS-07. | Fixed bypass on the LEFT (45° out 20 cm, parallel 20 cm, back 20 cm), sweep finds the line, LINE_FOLLOW again, box not touched |
| SYS-10 | Telemetry over MQTT | FR15, FR17, NFR10, NFR14 · rubric 9 | manual | Run `dashboard/start_dashboard.py`; watch during SYS-02..09. | ir/range/line/motion at 5 Hz, heartbeat 1 Hz, events (barcode, hump, obstacle, state) appear; heartbeat faults = 0 |
| SYS-11 | Stop when picked up (end of demo) | FR18, NFR4, NFR9 · rubric 10 | manual | While following, lift the car 10–20 cm. | Motors stop within 0.5 s; state FINISHED; car stays stopped until a new START |
| SYS-12 | Remote commands and e-stop | FR16, NFR4 | manual | Dashboard: pause, resume, e-stop, speed 2–25, one invalid command. | Pause/e-stop stop the car within 150 ms; invalid command counted, no state change |
| SYS-13 | WiFi loss does not stop the run | NFR11, NFR12, NFR13 | manual | Mid-run, turn the hotspot off for 20 s, then on. | Car keeps following; reconnects within 12 s; held events published after reconnect |
| SYS-14 | Stall / wheel fault | FR6, FR19 | manual | Hold one wheel while following. | Brakes; FAULT_STALL or FAULT_ENCODER within 500 ms; FAULT state; needs RESET |
| SYS-15 | Firmware builds clean | NFR21 | auto | `make -j8`, `make TEST=1 -j8`, `make NET=lwip MQTT=1 -j8` | 0 warnings, 0 errors |
| SYS-16 | Full demo run, rubric 1–10 in one run | Demo rubric | manual | Whole course once, as in the demo (max 3 runs, last run counts). | All of SYS-02..SYS-11 in a single run |

*Method:* **auto** = checked by a tool or `TEST_CHECK`, result printed; **manual** = a person
observes the result (dashboard, console, ruler, stopwatch).

## Results

Add a new row for every run. Never edit or delete old rows. "Team" rows are what the team
saw on the car and reported; where no log file was kept, the Log column says so.

| Date | Test ID | Build | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-23 | SYS-15 | v94n source | build check (PC, arm-none-eabi-gcc 13.3.1) | PASS | `logs/2026-11-23_build_v94n.txt` | Normal and test images: 0 warnings, 0 errors. MQTT image built the same way for v94b–v94h (needs `config/wifi_secrets.h`, not in the repo) |
| 2026-11-23 | SYS-05 | v93 | team (Hao Ming) | FAIL | team observation, no log | Barcode Z (left) at the + junction: pivot ran to the 150° cap → FAULT. Cause: creep stopped with the axle ~121 mm short of the crossing (sensors measured 130 mm ahead of the axle, code assumed 60). Fixed in v94 (`CTRL_SENSOR_AXLE_MM` 130) |
| 2026-11-23 | SYS-05 | v94e | team | PASS | team observation, no log | Team: "everything is working perfectly fine except for the obstacle detection" |
| 2026-11-23 | SYS-10 | v94b MQTT | team + dashboard | PASS | dashboard CSV reviewed 15:51–15:53 (file not kept) | Online; ir/line/motion/range ~5 Hz, heartbeat 1 Hz, signal −50 to −61 dBm, faults 0 |
| 2026-11-23 | SYS-07 | v94e | team | FAIL | team observation, no log | Stopped ~50 cm before the box, and sometimes with nothing ahead (sonar reads 1–3 cm for anything within ~50 cm). Led to v94h/v94n 1 cm rule |
| YYYY-MM-DD | SYS-01..16 | v94n | | PASS / FAIL | `logs/…` | Pre-demo full run: fill in |
