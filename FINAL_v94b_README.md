# IS25 v94b - final version (23 Nov)

v93 + two fixes measured against the real car. Built with zero warnings.
**Not yet run on the car** - test before the demo (checklist at the bottom).

## Flash

| File | Use |
|---|---|
| `firmware/mqtt_contains_hotspot_password/IS25_v94b_MQTT_bypassLEFT.uf2` | **Demo.** Dashboard on, goes round the box on the LEFT |
| `firmware/mqtt_contains_hotspot_password/IS25_v94b_MQTT_bypassRIGHT.uf2` | Same, goes round on the RIGHT |
| `firmware/IS25_v94b_NO_MQTT_bypassLEFT.uf2` | No WiFi/dashboard (if the hotspot fails); LEFT |
| `firmware/IS25_CHECK_4_ultrasonic.uf2` | Test image: prints the raw sonar range every 0.5 s; car does not drive |

Pick LEFT or RIGHT by which side of the box has room on the track.

The MQTT images join Wi-Fi **"The network"** and the broker at **10.222.14.184:1883**
(the laptop's address on that hotspot on 23 Nov). If the laptop gets a different IP,
the dashboard will not connect: rebuild with the new IP in `config/wifi_secrets.h`.

**The `mqtt_contains_hotspot_password` images contain the hotspot password. Keep them
in the team; never push them to GitHub.** `config/wifi_secrets.h` is not in this zip.

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

## Fix 2 - obstacle stopgap (`b5_task.c`, `b5_config.h`)

Static test on the car (23 Nov): empty track **24 cm**; the 13 cm box at 20 cm **3 cm**;
right in front **1 cm**. The HC-SR04 (a 5 V part on 3.3 V) never reports the box's real
distance, and moving it to 5 V is not possible on this board. v93 ignored every reading of
10 cm or less as "self-echo", so the box was never seen.

- Watch: **3 readings in a row of 5 cm or less** = box ahead -> stop. Any other reading
  restarts the count (the 1 cm / 24 cm alternation of 21-22 Nov cannot fire it).
- Confirm while stopped: at least **2 of 8** readings of 5 cm or less.
- Then the **fixed bypass** (`B5_FIXED_BYPASS` = 1, tuned 22 Nov): turn out 45 deg, 26 cm,
  back to parallel, 28 cm past the box, turn back in 4 deg steps until a track sensor finds
  the line; TSK_CTRL then rejoins (Fix 1 geometry).
- If the sonar ever reads the box properly (11-20 cm), the full scan pipeline (width etc.)
  still runs, then uses the fixed path.

Limits - say these honestly if asked:
- **Width is not measured** on the stopgap path (rubric item 7), and no obstacle event
  with a width is published.
- A sonar stuck at 1-2 cm while driving looks like a box: the car would go round nothing.
- The guarded legs still stop on any reading of 8 cm or less, so a stray short reading
  during the go-round ends it in FAULT.

Turn off when the sonar is fixed (5 V + 1k/2k Echo divider, or a 3.3 V module such as
HC-SR04P / RCWL-1601): `B5_STOPGAP_SHORT` 0 and `B5_FIXED_BYPASS` 0 in `b5_config.h`.

## Build

```
cd build_make
make NET=lwip MQTT=1 -j8      # needs config/wifi_secrets.h + lib_patches/IF-07 applied
make -j8                      # no MQTT
make CHECK=4 -j8              # sonar check image
```
Go round on the right: set `B5_STOPGAP_LEFT` to 0 in `b5_config.h`.

## Test checklist (car)

1. Barcode Z (left) at the cross: wheels stop on the crossing, ~90 deg pivot, follows the
   branch. Then B (right), D (U-turn). Console: `S4: junction bar passed ...`,
   `S4: pivot starts ... edges after the junction`.
2. Box on the line: `S4: watch fired (stopgap: short readings)`,
   `S4: obstacle (stopgap): ...`, the go-round, back on the line.
3. Dashboard shows heartbeat, ir, range, barcode and state.
4. Lift the car at the end: motors stop (FINISHED).
