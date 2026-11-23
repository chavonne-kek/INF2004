# Whole-system simulation, 17 Nov evening (final firmware)

Same harness as `sim_2026-11-17/` (rp2040js, single-core `SMP=0 WIFI=none`
builds of commit "S3: reset I2C0 whenever the bit-banged checks hand
GP16/GP17 back"), after the IMU driver moved to polled SDK-style I2C.

| Log | Scenario JSON | Result |
|---|---|---|
| `test_image.log` | (none) | S1 12, S2 9, S3 14, S4 7, S5 4: 46/46 PASS |
| `check3.log` | (none) | S3 14/14, `Chip found: LSM303D at 0x1D` |
| `lift.log` | `{"pressStartAt":9,"barcode":"A","liftedFromS":38}` | barcode `A` at 22.0 s, `LIFTED` at 38.127 s, FINISHED |
| `lift_0x1e.log` | `{"imuAddr":30,"pressStartAt":9,"liftedFromS":28}` | found at 0x1E, `LIFTED` at 28.139 s, FINISHED |
| `barcode_turn.log` | `{"pressStartAt":9,"barcode":"A","branchLeftAtX":800}` | TURN_EXECUTE at 33.9 s, follows the branch (ends y = 326 mm, 90°) |
| `obstacle_10cm.log` | `{"pressStartAt":9,"boxX":700}` | OBSTACLE_AVOID 16.7 s, back to LINE_FOLLOW 82.7 s |
| `noisy_drive.log` | `{"pressStartAt":9,"accNoiseMg":120,"noseUpFromS":20,"noseUpToS":26,"noseUpDeg":18}` | LINE_FOLLOW for 61 s, no false lift |
| `old_chip.log` | `{"imuChip":"dlhc"}` | GY-511 fallback found |
| `unplug.log` | `{"imuOffFromS":6,"imuOffToS":12}` | read failing at 6.05 s, `IMU back` at 12.48 s |
| `absent.log` | `{"imu":"absent"}` | `IMU not found (-57)`, the car carries on |

Run with: `node harness.mjs <image.uf2> <seconds> '<scenario JSON>'`.
