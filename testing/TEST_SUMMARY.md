# IS25 test summary (v94n, 23 Nov 2026) — start here

| Level | Where | Count | Latest result |
|---|---|---|---|
| Use cases | `USE_CASES.md` | 10 (UC-01..UC-10) | each linked to its tests |
| Traceability | `TRACEABILITY.md` | FR1–FR19, NFR1–NFR23 | every requirement has at least one test |
| On-target unit tests (auto, `make TEST=1`) | `app_program/subsystemN/subsystemN_test.c` | 47 `TEST_CHECK`s | last car run logged per subsystem |
| PC host tests (`make run`) | `testing/subsystemN/host`, `buddy4_host`, `buddy5_host` | S1 28, S2 13, S3 19 + 44, S4 3 + 9 scenes, S5 7 | **all PASS, 23 Nov** (`subsystemN/logs/2026-11-23_*_v94n.txt`) |
| Firmware → dashboard round trip | `testing/subsystem5/roundtrip` | 2532 checks | **PASS, 23 Nov** |
| Manual subsystem tests | `subsystemN/TEST_LOG.md` (`S1-U12..`, `S2-M..`, `S3-M..`, `S4-M..`, `S5-M..`) | 33 | see each log |
| Integration | `integration/TEST_LOG.md` | INT-01..INT-11 | INT-08 PASS (v94e); simulator runs 42/42–46/46 |
| System | `system/TEST_LOG.md` | SYS-01..SYS-16 (demo rubric 1–10 + faults) | SYS-15 build PASS (0 warnings); SYS-05/10 PASS; full run before the demo |
| Simulator (`tools/sim`) | `integration/logs/sim_*` | whole-system runs | 46/46 test image, lift, bypass, barcode turn |

Run everything on a PC:

```sh
for d in testing/subsystem*/host testing/subsystem3/buddy4_host testing/subsystem4/buddy5_host testing/subsystem5/roundtrip; do make -C $d run; done
cd build_make && make -j8 && make TEST=1 -j8
```

Known limitation, tested and documented: the HC-SR04 runs at 3.3 V (datasheet: 5 V) and does
not report distance (S4-M08). Obstacle detection uses the v94n 1 cm rule and a fixed LEFT
bypass, so obstacle width (rubric 7, S4-M04) cannot be measured with this module.
