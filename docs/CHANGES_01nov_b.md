# IS25 - fixes on top of the 1 Nov source (IS25_source_01nov.zip)

Everything here was checked on the PC (host tests), in the self-test image
(43/43 pass in the simulator) and in the firmware simulator on a track
built from the project specification (`testing/integration/logs/sim_2026-11-01/`).

## 1. Every motion command lost its low 4 bits (biggest fix)

`subsystem2.h`: when the opcode grew to 4 bits for `S2_OP_SET_GAIN`, the
argument mask became `0x0FF0`, but it is applied AFTER the shift, so it
cleared the low 4 bits of every argument:

| Sent | Arrived | Effect on the car |
|---|---|---|
| FOLLOW 12 cm/s | 0, so the 5 cm/s minimum | Followed at 5 cm/s, not 12 |
| Sweep 60° left | 48° | Start-up sweep 48 L + 112 R + 48 L: ends **16° to the RIGHT** (the "points right after calibration" seen on the car) |
| Sweep 120° right | 112° | |
| Turn 90° | 80° | Junction turns short |
| U-turn 180° | 176° | |
| Straight across 5 cm | 0 | |
| Gain 1200 (arg 120) | 112 = 1120 | |

Now `S2_CMD_ARG_MASK 0x00FF`. The host test H-MO01 and the on-car test
S2-U03 were already failing because of this; both updated for 12 bits and
pass. Simulator: heading after the sweep -14° before, +1° after.

Note: the 30/31 Oct tuning (steering gain 2000 -> 1200, integrator limit)
was done while the car really crept at 5 cm/s. At the intended 12 cm/s
re-check the steering on the track; `gain` on the dashboard changes it live.

## 2. Barcode bars steered the car (the "acts weirdly" bug)

`line_sense.c compute_error()`: the arm (GP28) is both the barcode reader
and the far-left guard, and was checked FIRST, on any black. Every bar of
a barcode gave +40 (hard left) while the centre was still on its edge, and
was remembered as "line on the left".

Now (first match wins): crossing -> centre on edge -> centre dark -> right
guard -> arm -> memory. The arm only counts as the line after **12 mm** of
unbroken black (`LINE_ARM_MM`; a bar is at most 9 mm), so bars never steer
and never reset the LINE_LOST timer. The right guard is checked before the
arm: drifted left beside a barcode, the arm is over bars and the line is
to the right. New host test H-LS20 (H-LS13 is taken by the channel-map test in the team tree).

## 3. Barcode bars could look like a left junction

Left junction was "arm + centre black for 60 ms"; a 9 mm bar lasts 75 ms at
12 cm/s. Now both sides use distance: 14 mm of travel (a branch line is
18 mm). Host tests H-LS05/H-LS06 cover both sides at 5-25 cm/s.

## 4. Barcode decoded as soon as its last bar ends

Before, the decoder also waited for 5 narrow widths of white (about 15 mm),
so a junction right after a barcode could come before the command. Now the
read is validated the moment the 29th element ends. All decoder tests pass
(3-30 cm/s, speed changing, reversed, damaged, random bars).

## 5. Barcode command at the junction (project specification)

The barcode is read while following; the command waits for the next
junction. Then (`ctrl_task.c`):

- A / B / D: drive `CTRL_JUNCTION_APPROACH_CM` (5 cm) so the WHEELS are on
  the junction (the turn spins about the axle, the sensors are ahead of it),
  then 90° left / 90° right / 180°, then follow.
- C: 5 cm straight across, then follow.
- If the line ends with a command waiting (dead end), it is done there.

**Measure on the car:** distance from the IR sensors to the wheel axle,
minus about 5 mm, in cm -> `CTRL_JUNCTION_APPROACH_CM`.

## 6. Smaller fixes

- `gain 2600`-`3100` wrapped to almost no steering (8-bit overflow): the
  robot clamps at 2500, the dashboard accepts 100-2500.
- The test and check images did not compile (`subsystem5_test.c` used
  `SUBSYSTEM1_CMD_LEFT` without including `subsystem1.h`).
- Console labels follow the pins: `IR arm C R`, calibration `span arm C R`,
  check image text updated for the 1 Nov layout.
- S1 host tests used array positions for left/centre/right; now the
  `LINE_*` names. 4 S1 and 2 S2 host tests had been failing since the
  30/31 Oct changes; all 60 host checks pass now.

## 7. 2 Nov (after team review)

- `motion_straight_trim()` rounds instead of truncating, so a one-edge
  difference (0.96 mm/s at 1270 edges per turn) trims again; H-MO11 back
  to its one-edge check.
- The barcode-arm host test is H-LS20 (H-LS13 is the channel-map test in
  the team tree).

## Still open

- `IS25_COMPLIANCE.md` / `MQTT_PROTOCOL.md`: describe the 2-byte map frame
  (from CHANGES_01nov.md) and the 4-bit motion opcode.
- The MQTT image must be rebuilt from this source with your
  `config/wifi_secrets.h` (`make NET=lwip MQTT=1` after the IF-07 patch).
- IMU still not answering on the car.
