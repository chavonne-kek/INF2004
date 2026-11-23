# v94e - obstacle watch off (23 Nov)

Base: your v94b_final.  Everything else is untouched.

## What changed

1. `app_program/subsystem4/ctrl_task.c`
   - `CTRL_OBSTACLE_ENABLE` 1 -> 0.  That is the only switch that matters: it
     sets `b_running = false` before the single `enter(SYS_OBSTACLE_AVOID)`
     at line ~1320, so the state can never change to obstacle avoided.
     Everything in subsystem4 stays in the tree and still compiles.

2. `app_program/subsystem4/b5_config.h`
   - Added `B5_NEAR_CM` (6) and the near-field rule that the watch uses:
     this car's module reports 1-3 cm for an object and 24 cm for empty
     space, so a reading at or under 6 is treated as an object.

3. `app_program/subsystem4/b5_task.c`
   - Watch counts a hit on a reading at or under `B5_NEAR_CM`, before the
     old self-echo branch (which used to throw those away as noise).

Both 2 and 3 are inert while `CTRL_OBSTACLE_ENABLE` is 0 - they are there so
the code is correct when the sensor is replaced.

## Why

The HC-SR04 does not report distance on this car.  Measured today with a box
at 20 cm and with nothing in front: 1-3 cm either way, plus 24 cm when the
echo fails.  The watch therefore either never fires or fires on empty track -
the mid-barcode stop.  Obstacle avoidance is dropped for the demo.

## To turn it back on (5 V supply + Echo divider, or RCWL-1601)

  CTRL_OBSTACLE_ENABLE  1        ctrl_task.c
  B5_TRIGGER_CM         20       b5_config.h
  B5_CONFIRM_CLOSE_CM   18
  B5_NEAR_CM            6
  B5_STOPGAP_SHORT      1
  B5_STOPGAP_SHORT_CM   6
  B5_STOPGAP_CONFIRM_READS 3
