# Subsystem 1: IR line following and barcode decoding (Buddy 3, Hao Ming)

TSK_LINE, priority 12, core 1, woken every **1 ms** by a cyclic handler
(`S1_SAMPLE_MS`). Reads the three TCRT5000 analogue outputs (ADC0–2 =
GP26–28), publishes the line word for subsystem 2, and decodes Code 39
barcodes with the arm sensor (GP28).

> Updated 23 Nov (v94n) to match the final code. The 1 Nov "centre sensor on
> the edge" layout and the start-up IR calibration sweep described in older
> notes are gone.

| File | What it does |
|---|---|
| `line_task.c` | The 1 ms task: ADC, events, barcode messages, timing |
| `line_sense.c` | Black/white decisions, steering error, hold in bends, JUNCTION |
| `barcode.c` | Code 39 decoder (statechart 6.2), both directions, character map |
| `subsystem1.h` | Public interface: line word, barcode byte, events |

## How it works

**Sensor layout as built (20 Nov, measured 23 Nov) and the track.**
The track: 18 mm black line; Code 39 barcodes about 141 mm long (3 mm
narrow, 9 mm wide elements, e.g. `*X*`), beside the line.

| Sensor | Pin | Where it sits | Job |
|---|---|---|---|
| Arm (`LINE_LEFT`, `LINE_BARCODE`) | Grove 7, GP28 | Left of the track sensors, over the barcodes | Reads barcodes; **never steers** |
| Left track (`LINE_CENTRE`) | Grove 6, GP27 | Just outside the line's LEFT edge | Steering |
| Right track (`LINE_RIGHT`) | Grove 5, GP26 | Just outside the line's RIGHT edge | Steering |

The two track sensors are 25 mm apart and **130 mm ahead of the wheel
axle** (measured on the car, 23 Nov). The line runs between them.

**Black or white (NFR15, no calibration).** Each track sensor is decided on
its raw 12-bit reading with hysteresis: black above `LINE_RAW_BLACK_ON`
(900), white again below `LINE_RAW_BLACK_OFF` (650). The arm uses a
self-adjusting envelope and needs at least `LINE_BC_MIN_SPAN` (240) counts
of contrast. The start-up IR sweep was removed on 20 Nov: with the line
between the sensors they never saw black during it, so it failed every
boot. The console still prints `S4: calibration IR FAILED (…)` at start-up;
ignore it (START is not refused).

**Steering (straddle layout, 20–22 Nov).**

| Track sensors | Meaning | Error (positive = steer left) |
|---|---|---|
| Both white | Line between them | 0, or the last correction held and faded to 0 over 40 mm of travel (`LINE_HOLD_MM`) so the car keeps turning through a bend |
| Left black | Car has drifted right | +15 mm, held for 10 mm, then rising to +40 over 20 mm (`LINE_RAMP_*`) |
| Right black | Car has drifted left | the same, negative |
| Both black | Crossing | 0 (straight) |

The arm is the barcode reader and never steers. Both white is the healthy
state, so the line is always reported valid and LINE_LOST is not raised
in this layout (FR3).

**Junctions.** Both track sensors black for `LINE_BRANCH_MM` (10 mm) of
travel (subsystem 2 odometry) = JUNCTION. A barcode bar (at most 9 mm,
under the arm) cannot cause it.

**Line word (IF-01), 16 bits, one writer, one store.** Error + 40 in bits
0–6, valid / lost / junction in bits 7–9, barcode-arm state in bit 10.
TSK_MOTION reads it every 10 ms without a lock.

**Barcodes (FR4).** Bar widths are measured by encoder travel (by time if
the car moved less than 20 mm), and each bar is classed narrow or wide by
ratio, so the speed does not matter. A read needs exactly 29 elements, a
valid start/stop `*` and a known character, in either direction.
Otherwise it is rejected with a reason code (1–5), never guessed. The
result is one byte (index + reversed bit), sent to TSK_CTRL.

**What the car does with it (FR5).** The barcode is read while the car
follows the line; the command waits for the next junction:

| Barcode (default map; any letter A–Z can be mapped from the dashboard) | At the next junction |
|---|---|
| A | Turn left, follow the branch |
| B | Turn right |
| C | Cross 40 mm straight (`CTRL_CROSS_MM`), follow |
| D | U-turn, follow back |

Turning (ctrl_task.c, v94): after the crossing bar has passed behind the
sensors, the car creeps `CTRL_SENSOR_AXLE_MM` − 9 = **121 mm** more so the
**axle** is on the crossing, then pivots until the leading track sensor
meets the branch (45–150°; U-turn 135–240°), brakes and follows. No line
by the pivot limit → FAULT, motors stopped. An unmapped letter is only
reported.

**Sensor height matters for barcodes (28 Oct, firmware simulation).**
The narrow bars and spaces are 3 mm. With a sensor spot up to 4 mm wide
the firmware reads them; a 6 mm spot blurs them. Mount the IR board low
(the TCRT5000 is sharpest at about 2.5 mm) and check S1-U14 on the real
barcodes.

## Settings to check on the car

| Constant | File | Value | Check with |
|---|---|---|---|
| `LINE_RAW_BLACK_ON` / `LINE_RAW_BLACK_OFF` | `line_sense.c` | 900 / 650 (of 4095) | S1-U12 raw values on black and white |
| `LINE_BC_MIN_SPAN` | `line_sense.c` | 240 | Arm contrast on the barcodes (S1-U14) |
| `LINE_BRANCH_MM` | `line_sense.c` | 10 mm | Junctions found, barcode bars ignored |
| `LINE_HOLD_MM` | `line_sense.c` | 40 mm | Car stays on the line through bends |
| `CTRL_SENSOR_AXLE_MM` | `ctrl_task.c` | 130 mm (measured) | Junction turn ends on the branch (INT-08) |
| `S1_BARCODE_END_CHAR` | `subsystem1.h` | off | Only if a course "end" character is used |
| `LINE_TIMING_PIN_ENABLE` | `line_task.c` | 0 | 1 = GP19 high while sampling (scope, S1-U15) |

## Tests

PC: `testing/subsystem1/host`, `make run` (28 checks against a physical
model of the sensors and the 141 mm barcodes, including H-LS20: barcode
bars under the arm never steer the car, and H-LS21: the hold in bends).
Last run 23 Nov: 28/28 PASS (`testing/subsystem1/logs/2026-11-23_host_v94n.txt`).

S1-U05 fed only 60 white samples after the barcode, but at the slowest
speed (20 samples per narrow bar) the decoder needs 5 x 20 = 100 to see
the end, so it would have failed on the Pico.  The test now feeds the
Code 39 quiet zone (10 narrow widths), like the printed barcodes.

Two deliberate differences from statechart 6.2: the quiet zone before a
read is 60 ms (not 100 ms), so a 30 mm quiet zone still works at
30 cm/s; and a black bar is rejected after 510 ms (not 150 ms), so a
read still works down to about 2 cm/s. Pico: `make TEST=1`,
S1-U01…U11 and S1-U16 (worst sample time ≤ 0.5 ms, NFR1). Manual tests
S1-U12…U15 are in `testing/subsystem1/TEST_LOG.md`. For S1-U12 the normal
image's 2 s console line shows the raw readings, e.g.
`line -3 mm, IR arm 812 C 3012 R 790` (arm, left track, right track, 0–4095).
