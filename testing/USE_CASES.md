# IS25 use cases

How the car is used, step by step, with the tests that check each step.
Actors: **Operator** (a team member at the track and dashboard), **Car** (Pico W firmware),
**Dashboard** (laptop: Mosquitto broker + `dashboard/is25_web.py`), **Track** (line, barcodes,
humps, junctions, box).

| ID | Use case | Main tests |
|---|---|---|
| UC-01 | Start a run | SYS-01, SYS-02, S2-U01, S3-U01 |
| UC-02 | Follow the line | SYS-02, INT-01, S1-U09..U11, H-LS01..LS21 |
| UC-03 | Read a barcode | SYS-03, SYS-04, S1-U05..U08, S1-U14, H-BC01..BC12 |
| UC-04 | Turn at a junction as the barcode says | SYS-05, INT-02, INT-08 |
| UC-05 | Measure a hump | SYS-06, S3-U08, S3-M03, H-IM01..21 |
| UC-06 | Handle an obstacle | SYS-07..09, INT-07, INT-09, INT-10, S4-M06, S4-M08 |
| UC-07 | Watch the car on the dashboard | SYS-10, INT-06, S5-U03, S5-M01, S5-M02, roundtrip selftest |
| UC-08 | Control the car remotely | SYS-12, S5-U01, S5-U02, S5-M03 |
| UC-09 | End the run by picking the car up | SYS-11, S3-U12..U15, S3-M08, S3-M09 |
| UC-10 | Keep going through faults (WiFi loss, stall) | SYS-13, SYS-14, INT-11, S2-M06, S5-M04, S5-M05 |

---

## UC-01 Start a run
- **Actor:** Operator. **Pre:** car on the line, battery on, hotspot and dashboard running.
- **Main flow:**
  1. Operator powers the car.
  2. Car waits 2 s, keeps still 1 s to zero the IMU, enters IDLE, publishes `robot/status` online.
  3. Operator presses START (GP20) or dashboard Start.
  4. Car enters LINE_FOLLOW at the set speed (default 3 cm/s).
- **Exceptions:** car moved during zeroing → pitch zero is wrong and false humps can follow
  (keep it still); WiFi not found → car still runs (NFR11), heartbeat fault WIFI.
- **Post:** car following; dashboard shows state LINE_FOLLOW.

## UC-02 Follow the line
- **Pre:** LINE_FOLLOW.
- **Main flow:**
  1. TSK_LINE samples the two track sensors every 1 ms.
  2. Both white = line between them, drive straight; one black = steer toward it.
  3. TSK_MOTION steers every 10 ms from the IF-01 error.
- **Alternate:** in a bend the line leaves both sensors → the last correction is held and
  faded over 40 mm (FR1).
- **Post:** car on the line, deviation ≤ 2 cm on straights.

## UC-03 Read a barcode
- **Pre:** following; Code 39 label across the line.
- **Main flow:**
  1. The arm sensor (GP28) sees the bars.
  2. Bar widths are measured by encoder travel.
  3. `*` + character + `*` is decoded in either direction.
  4. The character is published as a barcode event.
- **Exceptions:** damaged or partial label → reject with a reason code (1–5), never a
  character.
- **Post:** pending command set from the map (A left, B right, C straight, D U-turn, A–Z
  re-mappable over MQTT).

## UC-04 Turn at a junction as the barcode says
- **Pre:** pending command set; car reaches a + junction (both track sensors black).
- **Main flow:**
  1. Car creeps until the bar has passed behind the sensors.
  2. It creeps a further 121 mm so the axle is on the crossing (v94; sensors 130 mm ahead
     of the axle).
  3. It pivots until the leading sensor meets the branch (45–150°; U-turn 135–240°).
  4. It brakes, then follows the branch.
- **Exceptions:** no line found by the pivot cap → FAULT, motors stopped.
- **Post:** following the new branch.

## UC-05 Measure a hump
- **Main flow:**
  1. Pitch rises above 3° for 20 ms while moving.
  2. Height is integrated over encoder distance until pitch falls below 2°.
  3. A hump event is sent (last and run-maximum height, 7 bits each, mm).
- **Exceptions:** still climbing after 600 mm → a ramp, not a hump (dropped).

## UC-06 Handle an obstacle
- **Pre:** following; box on the line.
- **Main flow:**
  1. TSK_SCAN pings ahead.
  2. On this car the HC-SR04 (3.3 V) reports 1 cm only when an object is close, so a leaky
     count of 1 cm readings reaching `B5_ONE_CM_COUNT` asks TSK_CTRL to stop.
  3. The car stops and confirms with 3 of 8 short readings while stationary.
  4. It does the fixed bypass on the LEFT: 45° out 20 cm, parallel 20 cm, back 20 cm,
     square up, then sweep up to 90° until a sensor sees the line.
  5. TSK_CTRL rejoins and resumes LINE_FOLLOW.
- **Exceptions:** confirm fails → false alarm, re-armed when the reading clears; the bypass
  finds no line → FAULT.
- **Limitation:** width cannot be measured with this module (S4-M08).

## UC-07 Watch the car on the dashboard
- **Main flow:**
  1. Operator runs `python dashboard/start_dashboard.py`.
  2. The broker binds the laptop's hotspot address.
  3. The car publishes packed binary telemetry: 5 Hz ir/range/line/motion, 1 Hz
     heartbeat/imu/comms, plus events.
  4. The dashboard decodes and logs to CSV.
- **Exceptions:** car silent > 3 s → dashboard flags it; broker publishes Last Will
  "offline".

## UC-08 Control the car remotely
- **Main flow:**
  1. Operator sends start / pause / resume / e-stop / reset / speed / gain / map from the
     dashboard.
  2. TSK_COMMS validates the command and passes it to TSK_CTRL.
- **Exceptions:** wrong length, opcode or argument → ignored and counted; state unchanged.

## UC-09 End the run by picking the car up
- **Main flow:**
  1. Operator lifts the car.
  2. TSK_IMU detects the lift (upward speed ≥ 150 mm/s for 50 ms) and commands STOP in the
     same 10 ms cycle.
  3. TSK_CTRL enters FINISHED.
- **Post:** stays stopped until a new START re-arms the detector.

## UC-10 Keep going through faults
- **WiFi lost:** periodic data dropped, last 16 events kept, car keeps following,
  reconnects within 12 s.
- **Wheel held / encoder lost:** brake, FAULT_STALL or FAULT_ENCODER, state FAULT;
  operator RESET then START.
- **IMU unplugged:** FAULT_IMU after 5 bad reads, run continues; IMU re-probed every 1 s.
