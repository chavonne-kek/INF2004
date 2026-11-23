# Subsystem 1: IR line following and barcode decoding (Buddy 3, Hao Ming)

TSK_LINE, priority 12, core 1, woken every 2 ms by a cyclic handler.
Reads the three TCRT5000 analogue outputs (ADC0–2 = GP26–28), publishes
the line word for subsystem 2, and decodes Code 39 barcodes with the arm
sensor (GP28).

| File | What it does |
|---|---|
| `line_task.c` | The 2 ms task: ADC, calibration, events, barcode messages, timing |
| `line_sense.c` | Normalisation, hysteresis, signed left-edge error, LOST/FOUND/JUNCTION |
| `barcode.c` | Code 39 decoder (statechart 6.2), both directions, A–D commands |
| `subsystem1.h` | Public interface: line word, barcode byte, events |

## How it works

**Sensor layout as built (1 Nov) and the track (project specification).**
The track: 18 mm black line; Code 39 barcodes about 141 mm long (3 mm
narrow, 9 mm wide elements, `*X*`), 20 mm beside the line.

| Sensor | Pin | Where it sits | Job |
|---|---|---|---|
| Arm (`LINE_LEFT`, `LINE_BARCODE`) | Grove 7, GP28 | 35–45 mm LEFT of the centre sensor, over the barcodes | Reads barcodes; far-left guard |
| Centre (`LINE_CENTRE`) | Grove 6, GP27 | Half on the line's LEFT edge | Does the steering |
| Right (`LINE_RIGHT`) | Grove 5, GP26 | Just past the line's right edge, on white | Near-right guard |

`S1_TRACK_RIGHT_EDGE` stays 0 with this layout (the pin names already
describe it). If the barcodes are on the other side of the line on the
real track, mount the arm on the RIGHT, the centre on the line's RIGHT
edge and GP26 just past the left edge, and set it to 1.

**Steering (Tribunal 1, 1 Nov order).** The first rule that matches wins:

1. Centre fully black and a side sensor black: crossing, error 0 (straight).
2. Centre on the edge: proportional, -9..+9 mm (more black = steer left).
3. Centre fully on the line: +15 (drifted right a little).
4. Right sensor black: -40 (drifted left off the edge, or a sharp right).
5. Arm black for **12 mm or more** of travel: +40 (drifted right past the
   whole line, or a sharp left).
6. Nothing: steer to the last remembered side (only errors of 12 mm or
   more are remembered); LINE_LOST after 300 ms.

Until 1 Nov the arm was rule 1 and acted on any black, so every bar of a
barcode forced a hard left and the car could swerve off the line beside a
barcode. Now the centre steers while a barcode passes, and a bar (at most
9 mm) never counts as the line.

**Junctions (Week 6 IF-01).** A branch line is 18 mm wide, a barcode bar at
most 9 mm, so both sides use distance (subsystem 2 odometry), not time:
left = arm and centre black for 14 mm of travel; right = right sensor black
for 14 mm while the centre is still on the line. (Until 1 Nov the left side
used 60 ms, which a 9 mm bar beats at 12 cm/s.)

**Line word (IF-01), 16 bits, one writer, one store.** Error + 40 in bits
0–6, valid / lost / junction in bits 7–9. TSK_MOTION reads it every
10 ms without a lock.

**Barcodes.** Bar widths are counted in 2 ms samples, and each bar is
classed narrow or wide by ratio, so the speed does not matter (tested from
3 to 30 cm/s, and with the speed doubling mid-read). A read needs exactly
29 elements, a valid start/stop `*`, and a known character; it is decoded
the moment the last bar ends (1 Nov; before, about 15 mm later). Otherwise
it is rejected, never guessed. The result is one byte (index 6 bits +
reversed bit), sent to TSK_CTRL with EVT_BARCODE.

**What the car does with it (project specification).** The barcode is read
while the car follows the line; the command waits for the next junction:

| Barcode (default map, changeable from the dashboard) | At the next junction |
|---|---|
| A | Drive the wheels onto the junction (`CTRL_JUNCTION_APPROACH_CM`), turn 90° left, follow |
| B | Same, 90° right |
| C | Drive 5 cm straight across, follow |
| D | Drive onto the junction, turn 180°, follow back |

If the line ends (dead end) with a command waiting, it is done there.
`CTRL_JUNCTION_APPROACH_CM` (5) = the distance from the IR sensors to the
wheel axle minus about 5 mm: measure it on the car.

**Calibration (FR2).** In INIT_CALIBRATE, TSK_CTRL sweeps the car 60° left, 120° right and 60° left (until
30 Oct a full 360° spin; one direction alone would keep one side sensor
off the line)
between `subsystem1_calibrate_begin()` and `_end()`. Each sensor's lowest
and highest readings become its white and black levels. If any sensor's
black and white differ by less than 400 counts, the whole calibration is
rejected and the previous levels are kept (`subsystem1_calibrate_end()`
returns false).

**Sensor height matters for barcodes (28 Oct, firmware simulation).**
The narrow bars and spaces are 3 mm. Run on a simulated car, the
firmware read `*A*` with a sensor spot up to 4 mm wide on the floor. With
a 6 mm spot, the 3 mm bars blur together: the read is rejected, and the
merged bars can even look like a right branch (false JUNCTION). So mount
the IR board low (the TCRT5000 is sharpest at about 2.5 mm) and check
S1-U14 on the real barcodes. A failed calibration is now explained on the
console, e.g. `S4: calibration IR FAILED (black-white span L 40 C 40 R 40,
400 needed)`.

## Settings to check on the car

| Constant | File | Default | Check with |
|---|---|---|---|
| `LINE_ON_LEVEL` / `LINE_OFF_LEVEL` | `line_sense.c` | 600 / 400 (of 1000) | S1-U12 raw values |
| `LINE_LOST_MS` | `line_sense.c` | 300 ms | No false LINE_LOST on curves (A4) |
| `LINE_BRANCH_MM` | `line_sense.c` | 14 mm | Branches (both sides) vs barcode bars on the track |
| `LINE_ARM_MM` | `line_sense.c` | 12 mm | Arm on the line vs a barcode bar |
| `LINE_MEMORY_MM` | `line_sense.h` | 12 mm | Smallest error remembered for a lost line |
| `CTRL_JUNCTION_APPROACH_CM` | `ctrl_task.c` | 5 cm | Sensor-to-axle distance minus 5 mm |
| `S1_BARCODE_END_CHAR` | `subsystem1.h` | off | Ask the lecturer |
| `LINE_TIMING_PIN_ENABLE` | `line_task.c` | 0 | 1 = GP19 high while sampling (scope) |

## Tests

PC: `testing/subsystem1/host`, `make run` (23 checks against a physical
model of the sensor and the 141 mm barcodes, including branches on both
sides and H-LS20: barcode bars under the arm never steer the car).

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
`line -3 mm, IR 812 3012 790` (left, centre, right, 0–4095).
