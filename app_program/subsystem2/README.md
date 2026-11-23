# Subsystem 2: motion control (Buddy 2, Kai Xuan)

TSK_MOTION, priority 14, core 1, woken every 10 ms. Owns the two motors
(PWM on GP8–GP11, 20 kHz) and the phase-A wheel encoders (left encoder on
Grove 1 = GP0/GP1, right encoder on Grove 2 = GP2/GP3, since 26 Oct)
through the shared GPIO interrupt. It is the only code that drives the
motors, and only TSK_CTRL sends it commands.

**Either signal wire works (28 Oct).** Both pins of each encoder's Grove
port are watched, and each wheel is counted on the pin that carries the
pulses (the first one to lead the other by 8 edges). So it no longer
matters whether the encoder output went to the white or the yellow wire.
The test image prints which pin each wheel ended up on.

| File | What it does |
|---|---|
| `motion_task.c` | Motion statechart 6.5, command handling, odometry, stall detection |
| `motion_calc.c` | PID, ramp, braking profile, straight-line trim, conversions, packing |
| `drv_motor.c` | PWM, brake (both inputs low) and coast (both high), per the Robo Pico truth table |
| `drv_encoder.c` | Edge counting in the shared GPIO interrupt, on whichever port pin is active (also dispatches S4's echo pin) |
| `subsystem2.h` | Public interface: commands, motion API, odometry word |

## Motion API (Project Write-up)

| Write-up name | Function | Notes |
|---|---|---|
| `moveForward(distance)` | `subsystem2_move_forward(cm)` | Ends with EVT_MOTION_DONE |
| `moveBackward(distance)` | `subsystem2_move_backward(cm)` | |
| `turnLeft(angle)` | `subsystem2_turn_left(deg)` | On the spot, encoder based |
| `turnRight(angle)` | `subsystem2_turn_right(deg)` | |
| `stop()` | `subsystem2_stop()` | Brakes in the same 10 ms cycle (NFR4) |
| line following | `subsystem2_command(S2_OP_FOLLOW, cm_s)` | Steers from the line word |
| speed limit | `subsystem2_command(S2_OP_SPEED, cm_s)` | 5–25 cm/s |

Each call queues a 2-byte command (11 bits) and returns at once.

## Motion statechart 6.5

```
BRAKED --FOLLOW--> FOLLOW --STOP--> BRAKED
BRAKED --MOVE/TURN--> MOVE/TURN --mark reached: brake--> STOPPING --wheels < 10 mm/s--> BRAKED + MOTION_DONE
MOVE/TURN --2 x expected time--> STALLED (+ FAULT_STALL)
any moving state --duty >= 30 % and no edge for 500 ms--> STALLED (+ FAULT_STALL, or FAULT_ENCODER if only one wheel)
STALLED --RESET--> BRAKED            (only RESET or STOP are accepted while STALLED)
```

Commands that arrive mid-move: STOP always wins and brakes at once. A new
MOVE/TURN replaces the old target from the current position, and the old
one never reports MOTION_DONE. SPEED only changes the limit.

When the stall rule fires, the console says what it saw, for example:

```
S2: STALL - motors driven (duty L -100% R 100%) but no encoder pulse from either wheel for 0.5 s
S2:   wheels did not turn -> battery switch, motor wires; wheels turned -> encoder 3V3/GND/signal wires
S2:   encoder edges since start: GP0 0 GP1 0 (left), GP2 0 GP3 0 (right); reset with GP21
```

The start-up spin is the first time the motors run, so a battery that is
off, or encoders that are not powered, show up there (the 27 Oct console
log: `faults=0x15`, STALL during INIT_CALIBRATE).

## What makes it accurate

- **PID per wheel**, with speed feed-forward. The D term works on the
  measurement, so a set-point step gives no kick (`MOTION_KD_MILLI`,
  default 0 until tuned).
- **Sliding-window speed** (100 ms), because a coarse encoder gives less
  than one edge per 10 ms at low speed.
- **Braking profile**: during MOVE/TURN the speed is limited to
  sqrt(2 × 500 mm/s² × remaining), so the robot arrives at 5 cm/s and
  brakes on the mark instead of coasting about 4 cm past it.
- **Straight-line correction**: during MOVE the wheel that has counted more
  edges is slowed and the other sped up (6 mm/s per mm of difference,
  ±40 mm/s max). During TURN the same keeps both arcs equal.
- **Odometry word (IF-02)**: distance mm (15 bits, wraps) + both speeds
  (7 bits each), one 32-bit store, read without a lock.

Simulation (`testing/subsystem2/host`, H-MO12): 1 m with the left motor
10 % weaker ends −2.8 mm off and 0.0° off course (4.8° without the trim).

## Calibration on the car (in this order)

1. **Edges per revolution (S2-M01).** Flash the test image. When
   `[CAL] S2-M01` appears, turn each wheel exactly 10 times by hand (the
   wheels coast for 15 s). Put the printed value in `MOTION_EDGES_PER_REV`
   **before the first normal run**: with the placeholder (40) and a real
   encoder of several hundred edges per turn, the start-up spin stops after
   a few degrees and the IR calibration fails. Under 150 edges per turn the
   test prints a note for Zi Siang (`GLUE_ACC_WINDOW`, S3). If no pin sees
   a single edge (car test, 29 Oct), it says so and what to check: turn a
   wheel slowly and watch the status LEDs next to Grove 1 (GP0/GP1) and
   Grove 2 (GP2/GP3); no blink means that encoder has no power or is not
   plugged in.
2. **Direction (S2-M02).** If a wheel runs backwards in FOLLOW, set
   `MOTOR_INVERT_LEFT/RIGHT` in `drv_motor.c`.
3. **Wheel diameter and track.** Measure them into `MOTION_WHEEL_DIAMETER_UM`
   and `MOTION_TRACK_MM`. Then fine-tune: if 1 m reads short, lower the
   diameter; if 90° turns overshoot, lower the track.
4. **PID (S2-M03).** KD = 0. Raise KP until the speed oscillates, then
   halve it. Raise KI until the steady error is gone. Add a little KD if it
   overshoots. Record each step in the PID table in `TEST_LOG.md`.
5. **Accuracy (S2-M04, S2-M05).** 10 × 1 m each way, 10 × 90° each way;
   fill in the accuracy table. Tune `MOTION_STRAIGHT_MILLI` if 1 m drifts
   sideways.
6. If phase A alone is not accurate enough (A2), connect each encoder's
   phase B to the other signal wire of its own Grove port, with phase A on
   `S2_PIN_ENC_x_A` (GP0 left, GP3 right) and phase B on `S2_PIN_ENC_x_B`
   (GP1 left, GP2 right). Then set `ENC_USE_PHASE_B` to 1 in
   `drv_encoder.h` (in this mode the pins are fixed). The counters then count
   up forward and down backward, and the speed takes its sign from them
   (braking, coasting and reversing count correctly). If a wheel shows a
   negative speed while driving forward, flip `ENC_INVERT_LEFT/RIGHT`.

The encoder interrupt counts **both edges** of phase A, which doubles the
resolution of the Week 6 rising-edge plan (the feedback's "half
resolution" point). `MOTION_EDGES_PER_REV` is therefore edges, not
pulses: use the S2-M01 helper, which counts the same way.
