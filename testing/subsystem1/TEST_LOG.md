# Subsystem 1 — test log

**Subsystem:** IR line following + Code 39 barcode decoding (Buddy 3)  ·  **Student ID:** 2500331  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

### On target (Pico W)

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | ADC opens and all three readings are 12-bit values | IR sensor integration | auto | `subsystem1_test.c` | `[TEST] S1-U01 PASS` |
| S1-U02 | 2 ms cyclic handler running, `line_task` exists | 2 ms sampling | auto | `subsystem1_test.c` | `[TEST] S1-U02 PASS` |
| S1-U03 | 500 ± 25 samples in 1 s | 2 ms sampling (needs `CNF_TIMER_PERIOD` = 1) | auto | `subsystem1_test.c` | `[TEST] S1-U03 PASS` |
| S1-U04 | No late (missed) samples in 1 s | CR1 timing, no overruns | auto | `subsystem1_test.c` | `[TEST] S1-U04 PASS` |
| S1-U05 | Synthetic \*A\* decodes as Turn Left at 3 speeds, with the Code 39 quiet zone after it (28 Oct: was too short for the slowest speed) | Barcode decoding, varying speed | auto | `subsystem1_test.c` | `[TEST] S1-U05 PASS` |
| S1-U06 | \*A\* read backwards decodes as A, reversed bit set | Bidirectional decoding | auto | `subsystem1_test.c` | `[TEST] S1-U06 PASS` |
| S1-U07 | Damaged barcode is rejected | No false commands | auto | `subsystem1_test.c` | `[TEST] S1-U07 PASS` |
| S1-U08 | Decodes while speed changes during the read | Robust at varying speeds | auto | `subsystem1_test.c` | `[TEST] S1-U08 PASS` |
| S1-U09 | Line error sign, straddle layout (20 Nov): both white 0; left sensor black +15 (steer left); arm alone 0; right sensor black −15; both black 0 + junction | IF-01 line state | auto | `subsystem1_test.c` | `[TEST] S1-U09 PASS` |
| S1-U10 | 16-bit line word packs/unpacks −40..+40 and flags | CR1 minimum bits, IF-01 | auto | `subsystem1_test.c` | `[TEST] S1-U10 PASS` |
| S1-U11 | Straddle layout (20 Nov): both track sensors white for 320 ms stays valid, no LINE_LOST (was: LINE_LOST after 300 ms — the straddle layout cannot see a lost line) | No false recovery sweep | auto | `subsystem1_test.c` | `[TEST] S1-U11 PASS` |
| S1-U12 | Raw readings on white floor and black line | Sensor integration | manual | 1. Flash the normal image and let the start-up finish (state IDLE or FAULT: the car then stays still). 2. Hold the car level with all three sensors at their mounted height (2–3 mm) over the white floor, then over the black line, 4 s each. 3. Read `IR <left> <centre> <right>` (raw ADC 0–4095) in the `S5: up …` line, printed every 2 s. | Black and white differ by > 400 counts on every sensor |
| S1-U13 | Calibration sweep | Sensor calibration | manual | Normal image, car on the line, battery on, press RUN/RESET: the start-up spin calls `subsystem1_calibrate_begin()`/`_end()`. | Console `S4: calibration IR ok (black-white span L … C … R …, 400 needed)`, every span > 400 |
| S1-U14 | Real barcodes A, B, C, D on the track, both directions | Barcode decoding | manual | Push the car over each printed barcode by hand (slow) and at driving speed, both directions. Console prints `S1: barcode 'A'` etc. | 10/10 correct per barcode, no wrong character |
| S1-U15 | Sampling period on a scope | 2 ms sampling | manual | Set `LINE_TIMING_PIN_ENABLE` to 1 (GP19), probe GP19. | Pulse every 2.0 ms; pulse width = processing time (record it) |
| S1-U16 | Worst TSK_LINE sample time, 1 MHz timer (Tribunal 1 gap: packing cost measured) | NFR1 | auto | `subsystem1_test.c`; prints `TSK_LINE worst sample N us` | ≤ 500 µs |

### On a PC (no hardware)

`testing/subsystem1/host/`: `make run` builds `barcode.c` and `line_sense.c` with gcc and checks them against a physical model: 3 mm narrow bars (141 mm barcode, per *Barcode & Line Specification*), 2 mm sensor spot, ADC noise, 2 ms sampling and the real hysteresis.

| ID | What is tested | Expected result |
|---|---|---|
| H-BC01 | Bar widths measured from the slide barcodes A F Z W X Y, both directions | All decode correctly |
| H-BC02 | Every Code 39 character, both directions | All decode correctly |
| H-BC03 | A/B/C/D at 3, 5, 10, 15, 20, 25, 30 cm/s with noise, 200 runs each | ≥ 98 % read at every speed |
| H-BC04 | Same runs | Never a wrong character |
| H-BC05 | Speed doubling or halving during the read | Decodes |
| H-BC06 | Printed wide:narrow ratio 2.2:1 to 3.5:1 | Decodes |
| H-BC07 | Bars only 75 % as dark as the line | Decodes |
| H-BC08 | Any one wide bar printed narrow | Rejected |
| H-BC09 | Plain black lines 6–60 mm under the sensor | Never decode |
| H-BC10 | 20 000 random bar sequences | Never decode |
| H-BC11 | Below minimum speed (1.5 cm/s, widths saturate) | No wrong character |
| H-BC12 | A/B/C/D → left/right/straight/U-turn | Correct mapping |
| H-LS01 | Error rises as the centre sensor moves onto the line | Never decreases |
| H-LS02 | Left sensor on line / only right sensor on line | +40 / −15 (17 Nov: was −40) |
| H-LS03 | LINE_LOST timing | 300 ms |
| H-LS04 | Line lost: keep steering to the side it was last seen | ±40 with the last sign |
| H-LS05 | JUNCTION timing | 60 ms |
| H-LS06 | 20 ms dark blip | Not a junction |
| H-LS07 | Line word, every error × every flag | Round-trips, bits 10–15 zero |
| H-LS08 | Calibration applies; a flat sweep is rejected | As stated |
| H-LS09 | Sensor that reads lower on black | Normalises correctly |
| H-LS10 | Right branch (18 mm line) at 5–25 cm/s, odometry in whole mm | JUNCTION |
| H-LS11 | Widest barcode bar (9 mm + blur) at 5–25 cm/s | Never a junction |
| H-LS12 | Sharp right curve (centre off the line); standing still on black | No junction |
| H-LS21 | Right guard alone / centre fully dark, with travel 0, 10, 20, 30 mm | −15, −27, −40, −40 / +15 … +40; back to 15 when it ends (17 Nov) |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-23 | H-BC01 … H-LS09 | (not committed yet) | Hao Ming | PASS (21/21) | `logs/2026-10-23_host_decoder_linesense.txt` | PC run, gcc. One miss in 200 at 5 cm/s, no wrong reads. |
| 2026-10-23 | build | (not committed yet) | Hao Ming | PASS | — | `make CONSOLE=uart WIFI=none` and `TEST=1` build with no errors; S1 files also clean with `-Wall -Wextra -Wconversion`. |
| 2026-10-26 | H-BC01 … H-LS09 | (not committed yet) | Hao Ming | PASS (21/21) | `logs/2026-10-26_host.txt` | Final code (end-barcode option, timing measurement added). |
| 2026-10-26 | H-BC01 … H-LS12 | (not committed yet) | Hao Ming | PASS (24/24) | `logs/2026-10-26_final_host.txt` | Right-branch junction added (Week 6 IF-01). |
| YYYY-MM-DD | S1-U01..11, U16 | `abc1234` | Hao Ming | PASS / FAIL | `logs/YYYY-MM-DD_unit.txt` | First on-target run (`python tools/read_serial.py auto subsystem1`). |
