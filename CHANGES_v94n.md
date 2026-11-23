# v94n - obstacle detection that fires, and a small-track bypass (23 Nov)

Base: your v94b_final + your v94h files (b5_task.c, b5_config.h).

## 1. Obstacle rule (your v94h idea, two fixes)

* `B5_ONE_CM_COUNT` 20 -> 15 -> **3**, and the counter is now LEAKY:
    - a reading of 1 cm  : +1
    - any other reading  : -1 (never below 0)
  A strict "N in a row" reset to zero on every stray reading.  This module
  emits 2, 3 and 24 between the 1s, so the run never completed and the car
  never fired.  Leaky tolerates the noise.
* `CTRL_OBSTACLE_ENABLE` back to **1** in ctrl_task.c.  Your v94h zip only
  replaced b5_task.c and b5_config.h; on a v94e base that leaves the watch
  switched off, so the new rule could never run.

## 2. Bypass no longer aborts on the first step

`GuardedForward()` checked the sonar before every 5 cm step and aborted below
`B5_STOP_DIST_CM` (8 cm).  This module reads 1 cm wherever it points, so the
car turned 45 deg, moved a little, and stopped.  New switch
`B5_BYPASS_SONAR_GUARD (0)` in b5_config.h; the guard code is still there.

## 3. Bypass shape for the small track (B5_FIXED_OUT_DEG 45)

    1. 45 deg out (left)            20 cm   B5_FIXED_OUT_CM
    2. 45 deg back, parallel        20 cm   B5_FIXED_PASS_CM
    3. 45 deg back toward line      20 cm   B5_FIXED_RETURN_CM
    4. 45 deg left, square up
    5. rotate on the spot, 10 deg steps, up to B5_FIXED_SWEEP_DEG (90),
       stop the moment a sensor sees black
    6. hand back to TSK_CTRL (B5_REJOIN_BY_CTRL 1)

Steps 4-6 replace the old "2 deg + 2 cm" hunt loop, which had no room to work
on a short track.

## Note on B5_LOG

`B5_PORT.H`'s `B5_LOG(fmt, ...)` needs at least one argument; a format-only
call fails to compile ("expected expression before ')' token").  Pass %u.

## Still open

Sonar still reports 1 cm for an object about 20 cm away and 24 cm for empty
lane - it is a presence code, not a distance, because the HC-SR04 is on 3.3 V.
Width measurement and the five-stage scan are therefore unused on this car.
