/**
 * @file b5_config.h
 * @brief Buddy 5 tunables: pins, geometry, timing, thresholds.
 *
 * Every value marked VERIFY must be checked against your board pin map and
 * your physical car before first run.
 *
 * IS25 integration (26 Oct): pins moved to the team pin map (TEAM.md),
 * fine step 5 degrees (statechart 6.1 / Tribunal 1), the echo always goes
 * through subsystem 2's shared GPIO interrupt.
 */
#ifndef B5_CONFIG_H
#define B5_CONFIG_H

/* ---- Platform ---------------------------------------------------------- */
/* 1 = use micro T-Kernel services (tk_dly_tsk, DI/EI). 0 = bare Pico SDK. */
#ifndef B5_USE_TKERNEL
#define B5_USE_TKERNEL              (1)
#endif

/* IS25: the echo edge always comes from subsystem 2's shared GPIO bank-0
 * interrupt (subsystem2_gpio_attach), so there is no own-callback option. */

/* ---- Pins (VERIFY against Robo Pico pin map / team pin allocation) ------ */
#define B5_PIN_SERVO                (15u)
#define B5_PIN_SONAR_TRIG           (4u)   /* IS25: GP4, Grove 3 (GP3 = encoder) */
#define B5_PIN_SONAR_ECHO           (5u)   /* IS25: GP5, Grove 3; module on 3V3 (5 V needs a divider) */

/* ---- Servo ------------------------------------------------------------- */
/* Angles are logical: 0 deg = car's right, 90 = straight ahead, 180 = left,
 * 11.1 us of pulse per degree ((2500 - 500) / 180).  IS25 car, 17 Nov
 * (servo trim tool on the car): the sensor faces straight ahead at 2200 us,
 * and 1850 us (right) / 2500 us (left) are as far as it can turn before it
 * touches the car.  So only 60..115 deg can be reached: every scan and look
 * below stays inside that fan, and b5_servo.c clamps the pulse to the
 * limits as a last guard. */
#define B5_SERVO_FRONT_US           (2200u)  /* pulse that points straight ahead */
#define B5_SERVO_LIMIT_RIGHT_US     (1850u)  /* furthest safe turn to the right */
#define B5_SERVO_LIMIT_LEFT_US      (2500u)  /* furthest safe turn to the left  */
#define B5_SERVO_US_PER_DEG_X10     (111u)   /* 11.1 us per degree */
#define B5_SERVO_PERIOD_US          (20000u)
#define B5_SERVO_SETTLE_BASE_MS     (20u)
#define B5_SERVO_SETTLE_PER_DEG_MS  (3u)
#define B5_SERVO_RIGHT_DEG          (60u)    /* furthest right look (1867 us) */
#define B5_SERVO_FRONT_DEG          (90u)
#define B5_SERVO_LEFT_DEG           (115u)   /* furthest left look (2478 us)  */

/* ---- Ultrasonic -------------------------------------------------------- */
#define B5_SONAR_TIMEOUT_MS         (40u)    /* IS25: was 30; HC-SR04 out-of-range pulse is ~38 ms */
#define B5_SONAR_GUARD_MS           (15u)    /* quiet time between pings */
#define B5_SONAR_MIN_CYCLE_MS       (60u)    /* IS25: HC-SR04 datasheet: >= 60 ms
                                                from trigger to next trigger */
#define B5_SONAR_NOECHO_FAULT_COUNT (20u)    /* consecutive misses -> fault */

/* ---- Scanning ---------------------------------------------------------- */
#define B5_COARSE_COUNT             (5u)     /* 66,78,90,102,114 deg (was 30..150) */
#define B5_COARSE_STEP_DEG          (12u)    /* fits the 60..115 deg fan (was 30) */
#define B5_FINE_STEP_DEG            (5u)     /* IS25: 5 deg grid (was 4) */
#define B5_FINE_MARGIN_DEG          (30u)    /* IS25: multiple of 5 (was 28) */
#define B5_FINE_MIN_DEG             (B5_SERVO_RIGHT_DEG)  /* fine scan stays in the fan */
#define B5_FINE_MAX_DEG             (B5_SERVO_LEFT_DEG)
#define B5_UNSEEN_EDGE_PAD_CM       (10u)    /* planning: object runs past the
                                                fan edge, so add this much width
                                                per side whose edge was not seen */
#define B5_FINE_MAX_SAMPLES         (48u)
#define B5_FINE_REPS                (3u)     /* median-of-3 per fine angle (1 or 3) */

/* ---- Detection / profiling -------------------------------------------- */
/* 21 Nov car: the watch reads the table itself - 24 cm and 7 cm straight
 * ahead with nothing in front - so 25 cm triggered on empty track
 * (the car stopped mid-barcode and "found" an obstacle that was the
 * floor).  20 cm keeps a real box well inside the band. */
/* 22 Nov (car): the sonar reads 24 cm on an EMPTY track, so a 25 cm
 * trigger counted "nothing there" as an obstacle and the car stopped on a
 * curve.  22 keeps a real box (<= 20 cm) inside the band and the empty
 * reading outside it. */
/* 22 Nov (car): back to 25.  At 22 the box stopped triggering at all - the
 * only workable reading this sonar gives for it is 24 cm, which is also
 * what an empty track gives.  Sensitivity is therefore back, and the
 * empty track is rejected by B5_CONFIRM_CLOSE_CM below instead: a real box
 * produces at least one genuinely short reading, an empty floor never does. */
#define B5_TRIGGER_CM               (25u)

/* 22 Nov: the sonar alternates a real reading with a 1 cm self-echo, so the
 * confirmation has to tolerate misses.  6 reads, 3 in band = a real box. */
#define B5_CONFIRM_READS            (8u)
/* 22 Nov (car): the alternation is 7,1,24,7,1,24 - only two reads in
 * band, so 3 of 6 called a real box a false alarm and the car drove into
 * it.  2 of 8 accepts the pattern; a lone 24 cm glance is already outside
 * the 22 cm band, so it cannot confirm on its own. */
#define B5_CONFIRM_HITS             (2u)

/* 22 Nov (car): what separates a real obstacle from the empty floor.  The
 * floor reads a flat 24 cm every time; a box within range always produces
 * at least one reading well under that.  No short reading = no obstacle,
 * whatever the count says. */
#define B5_CONFIRM_CLOSE_CM         (18u)
#define B5_TRIGGER_CONFIRM          (3u)     /* consecutive hits to trigger (17 Nov night: 2 -> 3) */
#define B5_SELF_ECHO_CM             (10u)   /* 22 Nov: the module ghosts at 1-7 cm
                                               * (its own burst plus the floor); the
                                               * box now reads 17, so 10 removes the
                                               * ghost without hiding the box */     /* 17 Nov night (car run1.csv): a reading
                                                this close is the car's own bracket or
                                                body (2 cm at 110 deg made the car
                                                stop and scan with nothing in front),
                                                never an obstacle.  21 Nov car: 5 cm
                                                was not enough - the table reads 7 cm,
                                                so raise it to 10 cm. */
#define B5_CRUISE_PERIOD_MS         (50u)
#define B5_DETECT_RANGE_CM          (45u)    /* forward reach for profiling (was 60: walls behind the box made it 57 cm wide) */
#define B5_CORRIDOR_HALF_WIDTH_CM   (25u)    /* lateral band that matters */
#define B5_EDGE_JUMP_CM             (10u)    /* range jump that ends an object */
#define B5_LOOKAHEAD_CM             (60u)    /* clearance search depth */

/* ---- Vehicle geometry (VERIFY) ----------------------------------------- */
#define B5_CAR_WIDTH_CM             (15u)
#define B5_CAR_LENGTH_CM            (20u)
#define B5_SAFETY_MARGIN_CM         (3u)

/* ---- Planning ---------------------------------------------------------- */
#define B5_STOP_DIST_CM             (8u)     /* too close: back off first */
#define B5_MAX_SHIFT_CM             (40u)    /* never leave track by more */
/* 22 Nov car: with an unreliable sonar the scan has reported a 30-40 cm box
 * as 1-4 cm, and the planner then asked for almost no sideways shift - the
 * car barely left the line and came back onto it.  Never trust a width
 * below B5_MIN_WIDTH_CM (use the default instead), and always move at least
 * B5_MIN_SHIFT_CM sideways, whatever the scan says. */
#define B5_MIN_WIDTH_CM             (10u)
#define B5_DEFAULT_WIDTH_CM         (30u)
#define B5_MIN_SHIFT_CM             (12u)    /* 12, not 18: on a 45 cm track an
                                               18 cm offset puts the wheel over
                                               the edge; 12 still clears the
                                               10.5 cm the plan needs */
#define B5_MAX_RETRIES              (3u)     /* fits 2-bit counter */
#define B5_REVERSE_CM               (10u)

/* ---- Bypass / recovery ------------------------------------------------- */
#define B5_BYPASS_TURN_DEG          (45u)

/* ---- Fixed-geometry bypass (22 Nov) -----------------------------------
 * The demo obstacle is always the same box, so the path is tuned to it once
 * instead of being re-derived from a sonar that reads it badly.  Set
 * B5_FIXED_BYPASS to 0 to go back to the scan-driven bypass.
 */
/* 22 Nov night: back to the rubric pipeline.  The fixed shape below was a
 * stopgap while the sonar was returning nonsense (width 1-4 cm for a 30-40 cm
 * box); it replaces stages 2-4, which are the ones being marked.  The sonar
 * now measures properly (17 cm steady at the box), so coarse -> fine ->
 * profile -> plan -> recover runs on real numbers again. */
#define B5_FIXED_BYPASS             (0u)
#define B5_FIXED_OUT_DEG            (45u)   /* 22 Nov car: 30 deg needed a
                                               * long diagonal (30 cm forward
                                               * for 17 cm sideways), so the
                                               * car was past the box before
                                               * it straightened           */
#define B5_FIXED_OUT_CM             (26u)   /* 26 cm at 45 deg = 18 cm out
                                               * and 18 cm forward */
#define B5_FIXED_PASS_CM            (28u)   /* 22 Nov car: 40 overshot the box by
                                               * 15-20 cm                              */
#define B5_FIXED_RETURN_STEP_DEG    (4u)    /* 22 Nov car: 5 deg steps left the
                                               * car badly angled when a sensor
                                               * caught the line               */
#define B5_FIXED_RETURN_STEP_CM     (3u)    /* ... each followed by a creep */
#define B5_FIXED_RETURN_MAX_DEG     (120u)   /* 22 Nov car: past 90 the car is
                                               * across the track, not onto it -
                                               * stop and fault instead of
                                               * facing the wrong way          */
#define B5_STEP_CM                  (5u)     /* guarded-move granularity */
#define B5_SIDE_SEEN_CM             (30u)    /* side range that means "obstacle" */
#define B5_PASS_MAX_CM              (80u)
#define B5_RETURN_OVERSHOOT_CM      (15u)
#define B5_RECOVER_CREEP_CM         (15u)
#define B5_RECOVER_SWEEP_DEG        (60u)
/* IS25 20 Nov: straddle sensor layout (track sensors just OUTSIDE the line
 * edges, both white = on the line).  Stopping at the first touch of the line
 * and squaring up left the car parallel to the line but beside it, and both
 * white then looked like "on the line" (simulator: drove off).  With 1, the
 * engine stops after turning back toward the line and TSK_CTRL rejoins it:
 * drive until a track sensor touches the line, creep the axle onto it,
 * pivot until the line is between the sensors (ctrl_task.c STEP_REJOIN_*). */
/* 22 Nov: the fixed bypass finds the line itself (its last leg stops on the
 * sensors), so TSK_CTRL must NOT run its own seek/creep/pivot afterwards -
 * that was what turned a successful rejoin into a sideways swing. */
#define B5_REJOIN_BY_CTRL           (1)
#define B5_RECOVER_SWEEP_STEP_DEG   (10u)
#define B5_FAULT_HOLD_MS            (1000u)

#endif /* B5_CONFIG_H */
