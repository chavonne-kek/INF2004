# Subsystem 5 — test log

**Subsystem:** WiFi / MQTT telemetry and commands  ·  **Owner:** Chavonne  ·  **Student ID:** 2502481  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | Command validation (FR16, 1 byte) | FR16, CR1 | auto | `subsystem5_test.c` | PASS |
| S5-U02 | Malformed command dropped and counted | FR16 | auto | `subsystem5_test.c` | PASS |
| S5-U03 | Heartbeat packs into 29 bits | CR1, FR17 | auto | `subsystem5_test.c` | PASS |
| S5-U04 | Telemetry post: length checked, non-blocking | IF-06 | auto | `subsystem5_test.c` | PASS |
| S5-M01 | Broker round trip (needs IF-07 approval) | FR15, IF-07 | manual | Apply `lib_patches/IF-07_mqtt_utk.patch`, `make NET=lwip MQTT=1`, subscribe to `robot/#`. | Heartbeat every 1 s |
| S5-M02 | Dashboard decoder matches serial log | R2, CR1 | manual | Run `tools/is25_dashboard.py --log run.csv`; compare with the robot's `S5: t=…` console lines (every 2 s in the normal image). | All fields equal |
| S5-M03 | 20 bad commands | FR16 | manual | In the dashboard type `bad`. | No state change, `bad_commands` = 20 |
| S5-M04 | Last Will within 7.5 s | NFR18 | manual | Power off the robot; time `robot/status` = offline. | ≤ 7.5 s |
| S5-M05 | Offline events kept and flushed | NFR13 | manual | Turn the AP off, read 2 barcodes, AP on. | Both barcodes published ≤ 5 s after reconnect |
| H-CO01..06 | All 256 command bytes, heartbeat, offline ring, topics, endianness | FR16, CR1 | host | `testing/subsystem5/host` | 6/6 PASS |
| RT-01 | Round trip: every payload built by the firmware's own pack functions (S1–S5) decodes to the same values in the dashboard; all 256 command bytes judged the same | R2, CR1 | host | `testing/subsystem5/roundtrip` (`make run`, gcc + python3) | 0 failed |
| D-01 | Dashboard live: subscribe, decode, log, send commands through a broker | FR15, FR16 | host | local broker + simulated robot | Commands arrive as the right bytes; out-of-range speed refused |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**host** = PC test in `host/` (`make run`); **manual** = a person observes
the result (scope, ruler, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-24 | H-CO01..06 | (not committed yet) | skeleton author | PASS (6/6) | `logs/2026-10-24_host.txt` | PC run, gcc, simulated inputs only. |
| 2026-10-26 | H-CO01..06 | (not committed yet) | final code | PASS (6/6) | `logs/2026-10-26_host.txt` | |
| 2026-10-26 | RT-01 | (not committed yet) | final code | PASS (2,273 checks) | `logs/2026-10-26_roundtrip.txt` | 200 cases per topic + all 256 command bytes. |
| 2026-10-26 | D-01 | (not committed yet) | final code | PASS | `logs/2026-10-26_dashboard_live.txt` | amqtt broker on 127.0.0.1, simulated robot; not yet the real robot. |
| YYYY-MM-DD | S5-U01..04 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | First on-target run: owner to fill in. |
