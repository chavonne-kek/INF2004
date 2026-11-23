# IS25 v94b - final version (23 Nov)

v93 + two fixes measured against the real car. Built with zero warnings.

**Superseded by v94n for the obstacle part:** the final obstacle rule and go-round path
are described in `CHANGES_v94n.md`. Fix 1 below is still in the final code.

## Fix 1 - barcode turn at a junction (`ctrl_task.c`)

Measured on the car: track sensors **130 mm** ahead of the wheel axle, **25 mm** apart.
The code assumed 60 mm.

- v93 pivoted as soon as the crossing bar passed behind the sensors, with the axle still
  ~121 mm short of the crossing. The left sensor crossed the branch at ~16 deg of pivot,
  inside the 45 deg "ignore" window, so the pivot ran to the 150 deg cap -> FAULT
  (barcode Z, 23 Nov).
- v94: after the bar passes behind the sensors, creep **121 mm** more (130 - 9) so the
  **axle** is on the crossing, then pivot. The leading sensor now meets the branch at
  ~80-88 deg.
- The approach (~139 mm, ~7 s at 2 cm/s) has its own 12 s cap and is excluded from the
  8 s step timeout.
- Rejoin after an obstacle: creep from the first touch is now 150 mm (130 + 20), was 90.
- The v93 note suggested stopping the pivot when both sensors read white. **Not done:**
  with sensors either side of the line, both white also happens anywhere off the line.

Tune: `CTRL_SENSOR_AXLE_MM` (ctrl_task.c, 130). Turn ends with the line beside the car on
the side it came from -> raise. Ends past the branch -> lower.

## Fix 2 - obstacle stopgap (`b5_task.c`, `b5_config.h`) - replaced in v94n

Static test on the car (23 Nov): empty track **24 cm**; the 13 cm box at 20 cm **3 cm**;
right in front **1 cm**. The HC-SR04 (a 5 V part on 3.3 V) never reports the box's real
distance, and moving it to 5 V is not possible on this board. v93 ignored every reading of
10 cm or less as "self-echo", so the box was never seen.

v94b's first stopgap (3 short readings in a row, then a fixed go-round) was replaced in
v94n by the 1 cm rule and the small-track go-round: see `CHANGES_v94n.md`.

Limits - say these honestly if asked:
- **Width is not measured** with this module (rubric item 7), and no obstacle event with
  a width is published.
- A sonar stuck at 1 cm while driving looks like a box.

Turn the stopgap off when the sonar is fixed (5 V + 1k/2k Echo divider, or a 3.3 V module
such as HC-SR04P / RCWL-1601): `B5_STOPGAP_SHORT` 0 and `B5_FIXED_BYPASS` 0 in
`b5_config.h`.

## Build

```
cd build_make
make NET=lwip MQTT=1 -j8      # needs config/wifi_secrets.h + lib_patches/IF-07 applied
make -j8                      # no MQTT
make CHECK=4 -j8              # sonar check image
```

## Test checklist (car)

1. Barcode Z (left) at the cross: wheels stop on the crossing, ~90 deg pivot, follows the
   branch. Then B (right), D (U-turn). Console: `S4: junction bar passed ...`,
   `S4: pivot starts ... edges after the junction`.
2. Box on the line: `S4: watch fired ...`, `S4: obstacle (stopgap): ...`, the go-round,
   back on the line (v94n path, see `CHANGES_v94n.md`).
3. Dashboard shows heartbeat, ir, range, barcode and state.
4. Lift the car at the end: motors stop (FINISHED).
