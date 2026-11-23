# Requirement → test traceability (v94n)

Requirements are the PreTri2 list (FR1–FR19, NFR1–NFR23). Each row names the tests that
check it. Unit IDs (`S1-U..`, `S2-U..`) are in `subsystemN/TEST_LOG.md`, host IDs (`H-..`) run
on a PC with `make run`, `INT-..` are in `integration/TEST_LOG.md`, `SYS-..` in
`system/TEST_LOG.md`. "Gap" means no test checks it yet.

| Req | Short description | Unit / host (auto) | Manual / integration / system |
|---|---|---|---|
| FR1 | Follow the line, straddle sensors, hold in bends | S1-U09, S1-U11, H-LS01..LS21 | INT-01, SYS-02 |
| FR2 | Power-on wait, zero, IDLE | S3-U02 | SYS-01 |
| FR3 | Junction event, arm state in IF-01 | S1-U10, H-LS04, H-LS05 | INT-02, SYS-05 |
| FR4 | Code 39 both directions, rejects | S1-U05..U08, H-BC01..BC11 | S1-U14, SYS-03, SYS-04 |
| FR5 | Map + execute turn at the junction | H-BC12 | INT-02, INT-08, SYS-05 |
| FR6 | PI speed control, STOP/MOVE/TURN, stall | S2-U02, S2-U06, S2-U07, S2-U09, H-MO01..13 | S2-M02, S2-M06, SYS-14 |
| FR7 | Encoder speed/distance, IF-02 | S2-U04, S2-U05 | S2-M01, INT-05 |
| FR8 | Hump detection from pitch/roll | S3-U07, S3-U08, S3-U11, H-IM01..21 | S3-M03, SYS-06 |
| FR9 | Hump height, 2-byte event | S3-U05, S3-U08, H-IM21 | S3-M03, SYS-06 |
| FR10 | Motion class, lift, overturn | S3-U12..U16, H-IM19, H-IM20 | S3-M07..M09 |
| FR11 | Obstacle detect (v94n: 1 cm rule) | S4-U02, S4-U05, S4-U06 | S4-M08, S4-M09, INT-09, SYS-07 |
| FR12 | Scan, width, clearance | S4-U07, H-SC01..03, B5 host | S4-M03, S4-M04, SYS-08 (limited, see S4-M08) |
| FR13 | Bypass (v94n fixed path, LEFT) | B5 host | S4-M06, INT-10, SYS-09 |
| FR14 | Rejoin the line | – | INT-10, SYS-09 |
| FR15 | Binary MQTT telemetry | S5-U03, S5-U04, H-CO01..07, roundtrip selftest (2532 checks) | S5-M01, S5-M02, INT-06, SYS-10 |
| FR16 | Commands validated | S5-U01, S5-U02, H-CO01 | S5-M03, SYS-12 |
| FR17 | Online / Last Will, reconnect | H-CO01..07 | S5-M04, SYS-13 |
| FR18 | Lift → STOP in the same cycle, FINISHED | S3-U12, S3-U15 | S3-M08, SYS-11 |
| FR19 | Single motion commander, statechart, faults | S4-U01, S4-U03, S4-U06 | INT-04, INT-07, SYS-14 |
| NFR1 | 1 ms sampling, WCET ≤ 0.5 ms | S1-U02..U04, S1-U16 | S1-U15 |
| NFR2 | 10 ms loop, ±5 % speed | S2-U01, S2-U08 | S2-M03 |
| NFR3 | ±2 cm/m, ±5°/90° | S2-U07, H-MO12 | S2-M04, S2-M05 |
| NFR4 | Stop within 50 ms / 150 ms | S2-U02 | S4-M05, INT-03, SYS-11, SYS-12 |
| NFR5 | Line position ≥ 99 % | H-LS01..LS21 | INT-01, SYS-02 |
| NFR6 | Barcode ≥ 95 % at 2–25 cm/s | H-BC03, H-BC04 | S1-U14, SYS-04 |
| NFR7 | Range ±2 cm, ping timing | S4-U05 | S4-M01, S4-M08 (fails on 3.3 V: documented) |
| NFR8 | Width ≤ 20 %, bypass margins | B5 host | S4-M04, S4-M06, SYS-09 |
| NFR9 | IMU noise, hump ±15 %, lift ≤ 0.5 s | S3-U06, H-IM21 | S3-M02, S3-M03, S3-M08 |
| NFR10 | Telemetry latency/rates | S5-U04 | SYS-10 |
| NFR11 | Join ≤ 10 s, WiFi failure never stops | – | SYS-01, SYS-13, INT-11 |
| NFR12 | Reconnect ≤ 12 s | – | SYS-13, INT-11 |
| NFR13 | Offline event queue | H-CO01..07 (offline ring) | S5-M05, SYS-13 |
| NFR14 | ≥ 98 % delivery | – | SYS-10 |
| NFR15 | No IR calibration, thresholds | H-LS08, H-LS09 | S1-U12 |
| NFR16 | Static kernel objects, RAM ≤ 75 % | S4-U01 | SYS-15 (size report: RAM 32 %) |
| NFR17 | Course on one charge | – | SYS-16 |
| NFR18 | Last Will within 45 s | – | S5-M04 |
| NFR19 | Watchdog 500 ms | – | S4-M07 |
| NFR20 | Pin levels 0–3.3 V | – | S4-M08 (HC-SR04 is a 5 V part; see note) |
| NFR21 | Builds with zero warnings | – | SYS-15 |
| NFR22 | Dashboard shows data ≤ 500 ms | roundtrip selftest | S5-M02, SYS-10 |
| NFR23 | Minimum bits, masks/shifts | S1-U10, S2-U03, S2-U04, S3-U03, S3-U05, S4-U04, S5-U03, H-SC03 | – |

Gaps: none uncovered. FR14, NFR11, NFR12, NFR14 and NFR17–NFR20 have manual tests only.
