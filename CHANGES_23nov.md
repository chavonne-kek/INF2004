# Changes 23 Nov — v93 (what is different from the 22 Nov tree)

Everything below is in the tree you are holding. Each item says what was
wrong, what changed, and which constant to touch if it misbehaves.

## 1. Lift detector fired on the hump landing (v89)

**Symptom:** car crosses the hump, then FAULT.

**Why:** dropping off the far slope is a short upward impact. The lift rule
integrates upward acceleration and latches `LIFTED` at 150 mm/s held for
50 ms; a 20 mm drop produces that easily, and `LIFTED` is a latch, so the
run ends.

**Fix:** `imu.c` `lift_step()` holds the integrator at zero while the hump
state machine says the car is on terrain, and for `IMU_LIFT_HUMP_GUARD`
(100 samples = 1 s) after it returns level.

**Cost:** a genuine lift while crossing a hump is detected up to 1 s late.

## 2. Standing tilt treated as terrain (v90)

**Symptom:** the module reads 2 deg with the car level, and the hump rule
threshold is 3 deg absolute — so 8% of the frames in one run were flagged
`CLIMBING` on flat track.

**Fix:** `imu.c` learns the standing pitch over 1 s of standing still at the
start of each run (100 samples, only while speed < `IMU_HUMP_MIN_SPEED`,
abandoned after 3 s) and subtracts it from both the hump threshold and the
height integral. Telemetry still shows absolute pitch.

**Condition:** leave the car still for about a second after START/RESET.

## 3. Obstacle watch armed on the empty floor (v91)

**Symptom:** every START went to `OBSTACLE_AVOID` with nothing in front.

**Why:** the sonar returned a steady 24 cm on open floor, and the trigger
band was `<= 25 cm`.

**Fix:** `b5_config.h` `B5_TRIGGER_CM` 25 -> **20**. The floor phantom at
24 no longer arms the watch; a real box reads 17-18 and still does. The
confirm step still demands 2 of 8 reads in band **and** at least one at or
below `B5_CONFIRM_CLOSE_CM` (18).

**Caveat:** this works only while the box reads clearly under the phantom.
If the module reports 24 for the box too, that is the 3.3 V HC-SR04
problem, not a threshold.

## 4. Pivot jolt read as a hump (v92)

**Symptom:** pitch jumps to 8.5 deg at a flat junction.

**Why:** no gyroscope — pitch is inferred from the accelerometer, so the
jolt from creep to pivot (~0.8 m/s^2) reads as slope. The encoder-based
compensation is noisy at creep speed.

**Fix:** `imu.c` only calls `hump_step()` while `|a_enc_mg|` is at or below
`IMU_HUMP_JOLT_MG` (80). Above that the hump state machine holds its state
and integral.

## 5. Junction approach is a sensor event, not 65 mm (v93)

**Symptom:** barcode `Z` mapped to left, the car pivoted past 90 deg and
stopped with FAULT, "no line seen".

**Why (first half):** the approach crept a fixed `CTRL_JUNCTION_CREEP_MM`
(65 mm) before pivoting. That number is a guess at the sensor-to-axle gap;
if it is wrong the car pivots off the crossing center and the sweep misses
the branch.

**Fix:** the creep now ends when both track sensors are white again (the
crossing bar has just passed behind them), with `CTRL_JUNCTION_LEAVE_MM`
(6 mm) as a floor and the old distance/time as caps.

## Known open issue at v93

The pivot still sweeps to `CTRL_PIVOT_MAX_DEG` (150) on a cross junction
and faults. The stop test waits for a track sensor to go **black**, but at
the correct heading the branch line lies **between** the two sensors, so
both are white. The fused line word already reports `valid, error ~0` —
the follow-up build (v94) stops the pivot on that signature instead
(`CTRL_PIVOT_CENTRE_MIN_DEG`, `CTRL_PIVOT_CENTRE_MM`). If you take this v93
tree and hit that fault, that is the change to port.

## Build notes

- `make MQTT=1` refuses until `lib_patches/IF-07_mqtt_utk.patch` is applied
  (instructor approval). Guard is in `build_make/pico_rp2040.mk`.
- `config/wifi_secrets.h` is **not** in this archive: copy
  `config/wifi_secrets.example.h` and fill in your own SSID/password.
- Host tests: `testing/subsystem3/host` and `testing/subsystem4/host`,
  plain `make && ./test_host`.

## 6. v94b (23 Nov, after the car tests) - supersedes "Known open issue at v93"

Measured on the car: track sensors 130 mm ahead of the axle (not 60), 25 mm apart;
the sonar reads the 13 cm box as 1-3 cm and the empty track as 24 cm.

- **Junction turn** (`ctrl_task.c`): v93's creep ended when the bar passed behind the
  sensors, leaving the axle ~121 mm short of the crossing, so the branch was crossed
  inside the 45 deg ignore window and the pivot ran to the cap (FAULT). v94 creeps
  `CTRL_SENSOR_AXLE_MM - 9` = 121 mm more after the bar has passed. The "stop when both
  sensors are white" idea above was NOT used: both white also happens off the line.
- **Obstacle stopgap** (`b5_task.c`, `b5_config.h`): 3 readings in a row <= 5 cm = box;
  2 of 8 short readings while stopped confirm it; then the fixed bypass
  (`B5_FIXED_BYPASS` 1). Width is not measured on this path.
- Rejoin creep after a bypass: 150 mm (was 90).
- `ctrl_task.c` includes `b5_task.h` (the implicit-declaration warning is gone).

Details, limits and the test list: `FINAL_v94b_README.md`.
