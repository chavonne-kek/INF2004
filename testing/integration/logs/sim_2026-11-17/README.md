# Whole-system simulation, 17 Nov (integration)

Firmware-in-the-loop: the real firmware (`make SMP=0 WIFI=none`, and
`TEST=1` for the self-tests) on rp2040js with `tools/sim/harness.mjs`.
IMU modelled as the car's **LSM303D** (0x1D unless stated), encoder 1270
edges per wheel turn, current sensor layout (`layout: arm`). Scenario JSON
for each run is in `tools/sim/README.md` (17 Nov additions).

| Log | What | Result |
|---|---|---|
| `test_image.log` | Test image | 46/46 PASS (S1 12, S2 9, S3 14 incl. lift/overturn U12..U16, S4 7, S5 4) |
| `lift.log` | Follow, read `*A*`, lifted at 38.0 s | `S3: LIFTED` at 38.116 s, STOP, state FINISHED |
| `overturn_0x1e.log` | LSM303D at 0x1E, rolled over at 28 s | found at 0x1E; `S3: OVERTURNED` at 28.546 s, FINISHED |
| `noisy_drive.log` | 70 s, 120 mg uniform vibration, 18° nose-up 20-26 s | LINE_FOLLOW throughout, no false lift |
| `obstacle_10cm.log` | 10 × 10 cm box on the line at x = 700 mm | OBSTACLE_AVOID at 16.7 s, back to LINE_FOLLOW at 82.7 s, ends on the line |
| `obstacle_16cm.log` | 16 × 6 cm box, 3 cm off-centre | OBSTACLE_AVOID at 16.7 s, LINE_FOLLOW at 86.6 s, ends on the line |
| `barcode_turn.log` | `*A*`, then a left branch at x = 800 mm | `A` -> command 1 (left); TURN_EXECUTE at the junction; follows the branch (ends at y = 325 mm, heading 90°) |
| `old_chip.log` | LSM303DLHC instead | fallback found it at 0x19/0x1E |

Limits: one core (single-core build), no radio, guessed motor and optics
numbers. It proves the logic, the drivers and the task interplay; the car
tests in each `TEST_LOG.md` still have to be done.
