# IS25 fixes on top of IS25_group_2000 (v4, 21 Nov)

## Flash
| File | Use |
|---|---|
| firmware/mqtt_contains_hotspot_password/IS25_21nov_v4_MQTT_obstacle_ON.uf2 | full demo |
| firmware/mqtt_contains_hotspot_password/IS25_21nov_v4_MQTT_obstacle_OFF.uf2 | barcode / junction testing |
| firmware/IS25_21nov_v4_NO_MQTT_obstacle_OFF.uf2 | no WiFi |

**Use the `dashboard/` folder in this zip, not the old one.** The old dashboard sends 'A' as 10;
this firmware expects 'A' = 0 (so every letter A-Z can be mapped - the old way could not send W-Z).

## Barcode
- Map: all letters A-Z can be mapped from the web page now (before: Q-Z could not).
- A barcode only sets the next turn while the car is following the line. Read during a turn or an
  obstacle bypass (the arm sweeps across track lines then) it is logged "barcode ignored" and
  does nothing.
- A rejected read printed up to 29 lines from inside the 1 kHz sensor loop (stalling line sensing
  just before the junction); now one line. The full widths still go to the dashboard.
- Reset race fixed (v1): the reset is done by the S1 task itself.

## Junction (after a barcode; both track sensors black = junction)
1. Creep CTRL_JUNCTION_CREEP_MM (65 mm) held straight, so the axle is on the crossing.
2. Pivot on the spot; sensors ignored for the first 45 deg (U-turn 135).
3. Brake the moment the leading sensor touches the new line.
4. Wait until the encoders show both wheels stopped, then follow the new line.
5. If it turns 150 deg (U-turn 240) without seeing a line: STOP (FAULT) - it used to follow
   from wherever it was and drive off the track.
- C (straight): held straight 40 mm over the crossing, then follows.
- Fixed earlier: TURN state never entered; pivot duty never written to the motors; 24 deg "lean"
  turn with an 8 s FAULT; full-speed reverse spin after the pivot (encoder cannot see direction).

## Obstacle
Detect (25 cm ahead, 3 readings) -> stop -> scan -> width -> plan a side -> go round -> turn back
toward the line -> TSK_CTRL rejoins: drive until a track sensor touches the line, creep
CTRL_JUNCTION_CREEP_MM + 25 mm, pivot back until the leading sensor touches the line, brake, follow.

## Simulator results (straddle layout)
| Case | Result |
|---|---|
| Cross junction, barcode A / B / C / D | left / right / straight / U-turn, ends on the line |
| Same, motor like the car (~320 mm/s full) + left motor 10 % weak | A, B, D all on the line |
| T junction A / B | left / right |
| Barcode narrow bar 2 mm and 4 mm | decoded, turned |
| Obstacle, go round left / right / car-like motor | rejoined, followed to the end |
| Obstacle, then barcode A, then cross junction | bypass + rejoin, then left turn |
| 90 deg corner, radius 6 / 8 / 10 cm, both ways | followed |
| 90 deg corner, radius 4 cm | LEAVES the line (see below) |

Tests: host S1 28, S2 13, S3 19, buddy4 44, S4 3, buddy5, S5 7 - all pass. TEST image: 46/46.

## For tomorrow - tune on the car
1. **Junction creep** `CTRL_JUNCTION_CREEP_MM` (ctrl_task.c, 65). Car pivots before reaching the
   branch -> raise. Pivots after passing it -> lower. Distance from the IR sensors to the wheel axle
   is the right starting value. Obstacle rejoin uses the same value + 25 mm.
2. **Pivot speed** `MOTION_PIVOT_DUTY` (motion_calc.h, DUTY_MAX/2). Swings past the new line ->
   lower (e.g. MOTION_DUTY_MAX * 2 / 5). Too low = it will not turn (STALL fault).
3. **Sharp corners**: at the default speed the follower holds a 90 deg corner down to ~6 cm radius.
   Tighter corners: lower the speed or raise the steering gain from the web page.
4. **Obstacle trigger** `B5_TRIGGER_CM` (b5_config.h, 25 cm) if it stops too early / late.

## Known limits (layout, not bugs)
- Sensors either side of the line cannot see a lost line (both white = "on the line").
- A junction with no barcode command: the car goes straight on. At a T junction it then leaves
  the line.
- The car must meet a junction within ~33 deg of square, or the two sensors are never black
  together and the junction is missed.
