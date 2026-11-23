/** @file motion_calc.h
 *
 * @brief Motion maths for subsystem 2: PID speed control with
 *        feed-forward, acceleration ramp, braking profile, straight-line
 *        correction, encoder-to-distance conversion and the packed
 *        command / odometry formats.  Pure logic: no kernel calls, so it
 *        is unit-tested on a PC (testing/subsystem2/host).
 */

#ifndef MOTION_CALC_H
#define MOTION_CALC_H

#include <stdint.h>
#include <stdbool.h>

#include "subsystem2.h"

/* PWM: 125 MHz / (6249 + 1) = 20 kHz, the Robo Pico driver maximum. */
#define MOTION_PWM_WRAP        6249
#define MOTION_DUTY_MAX        MOTION_PWM_WRAP

/* ---------------------------------------------------------------------
 * Car geometry.  TODO(Kai Xuan): MEASURE all three (Week 6 A2).
 *   Edges per wheel revolution: count both edges of phase A while turning
 *   the wheel 10 times by hand, divide by 10.
 * ------------------------------------------------------------------- */
#define MOTION_WHEEL_DIAMETER_UM  65000     /* 65 mm wheel (placeholder).   */
#define MOTION_EDGES_PER_REV      1270      /* Measured on the car 30 Oct:
                                            * 12699 edges / 10 turns.       */
#define MOTION_TRACK_MM           120       /* Wheel centre to centre.      */

/* Speed limits (FR16 speed command range 5..25 cm/s). */
#define MOTION_SPEED_MIN_MM_S     20   /* 2 Nov: 50 -> 20 so 4 cm/s is reachable;
                                        * at 50 every slower speed clamped back up. */
#define MOTION_SPEED_MAX_MM_S     250
#define MOTION_SPEED_DEFAULT_MM_S 200
/* Open-loop duty for a pivot.  Closed-loop reversing was unstable
 * (see run_wheels), so the turn is driven at a fixed duty. */
#define MOTION_PIVOT_DUTY      ((int32_t)(MOTION_DUTY_MAX / 2))

#define MOTION_TURN_SPEED_MM_S    60   /* 2 Nov: 80 -> 60, less overshoot.  */
#define MOTION_RAMP_MM_S_PER_TICK 5         /* 50 cm/s^2 at 10 ms.           */

/* Speed is measured over a sliding window, because a coarse encoder gives
 * less than one edge per 10 ms at low speed.  With 40 edges per wheel
 * turn, 10 ticks (100 ms) resolve about 50 mm/s.  TODO(Kai Xuan): shorten
 * if the measured edges per turn are much higher. */
#define MOTION_SPEED_WINDOW       10u

/* PID gains in thousandths.  TODO(Kai Xuan): tune on the floor (PID
 * tuning report): start with KD = 0, raise KP until the wheel speed
 * oscillates, halve it, then add KI until the steady error is gone. */
#define MOTION_KFF_MILLI          20000     /* duty per mm/s (feed-forward). */
#define MOTION_KP_MILLI           15000
#define MOTION_KI_MILLI           40000     /* Per second.                  */
#define MOTION_KD_MILLI           0         /* Seconds x 1000; on the
                                               measurement, so a set-point
                                               step gives no kick.          */
#define MOTION_STEER_MILLI        500       /* 17 Nov evening: 700 -> 500 (car
                                             * still weaved on straights, which
                                             * also made the barcode arm miss
                                             * bars).  At e = 40 mm the wheels
                                             * still differ by 40 mm/s for bends.
                                             * 2 Nov: 1200 -> 700.  At 1200 the
                                             * car snaked left/right on a
                                             * straight: 15 mm of error gave an
                                             * 18 mm/s wheel difference, which
                                             * overshot before the next sample.
                                             * 700 gives 10 mm/s for the same
                                             * error - it settles instead. */

/* 22 Nov (car): a single gain cannot serve both a straight and a bend.
 * 500 is calm on a straight (1200 snaked), but on a bend it asks for only
 * 15 mm/s of wheel difference at a 30 mm error, so the car crawled round
 * and the line left the sensor array - "turns too little, then loses the
 * track".  Below the knee the old gain is kept untouched; beyond it each
 * extra mm of error adds a much larger correction, so the bend is taken
 * without touching the straight-line behaviour.
 *   error 12 mm -> 6 mm/s    (as before)
 *   error 30 mm -> 15 + 15x1.2 = 33 mm/s
 *   error 45 mm -> 22 + 30x1.2 = 58 mm/s
 */
#define MOTION_STEER_KNEE_MM      15
#define MOTION_STEER_HARD_MILLI   1200

/* Braking profile for MOVE and TURN: the speed is limited to
 * sqrt(2 x decel x remaining distance), so the robot arrives at
 * MOTION_FINAL_MM_S and stops on the mark instead of coasting past it.
 * The deceleration equals the ramp (5 mm/s per 10 ms = 500 mm/s^2). */
#define MOTION_DECEL_MM_S2        500
#define MOTION_FINAL_MM_S         50

/* Straight-line correction (MOVE) and equal arcs (TURN): mm/s taken from
 * the wheel that is ahead and given to the other, per mm of difference.
 * TODO(Kai Xuan): tune with the 1 m test (NFR3). */
#define MOTION_STRAIGHT_MILLI     6000
#define MOTION_STRAIGHT_MAX_MM_S  40

typedef struct
{
    int32_t kff_milli;
    int32_t kp_milli;
    int32_t ki_milli;
    int32_t kd_milli;
    int32_t integral;       /* Sum of error x ticks (mm/s x 10 ms). */
    int32_t integral_limit;
    int32_t last_measured;  /* For the derivative term.            */
    bool    b_have_last;
} motion_pid_t;

void     motion_pid_init(motion_pid_t * p_pid);
void     motion_pid_reset(motion_pid_t * p_pid);
int32_t  motion_pid_step(motion_pid_t * p_pid, int32_t target_mm_s,
                         int32_t measured_mm_s);

int32_t  motion_ramp(int32_t current, int32_t target, int32_t step);
int32_t  motion_clamp(int32_t value, int32_t low, int32_t high);

uint32_t motion_edges_to_um(uint32_t edges);
int32_t  motion_window_speed(uint32_t edges, uint32_t ticks);
uint32_t motion_mm_to_edges(uint32_t mm);
uint32_t motion_turn_arc_mm(uint32_t degrees);
int32_t  motion_steer_mm_s(int8_t line_error_mm);
void     motion_set_steer_milli(int32_t milli);
int32_t  motion_steer_milli(void);
int32_t  motion_profile_speed(uint32_t remaining_mm, int32_t cruise_mm_s);
int32_t  motion_straight_trim(uint32_t done_left, uint32_t done_right);
uint32_t motion_isqrt(uint32_t value);

uint16_t motion_cmd_pack(subsystem2_op_t op, uint8_t arg);
bool     motion_cmd_unpack(uint16_t raw, subsystem2_op_t * p_op,
                           uint8_t * p_arg);
uint32_t motion_odom_pack(subsystem2_odom_t const * p_odom);
void     motion_odom_unpack(uint32_t word, subsystem2_odom_t * p_odom);
int16_t  motion_distance_delta(uint16_t now_mm, uint16_t before_mm);

#endif /* MOTION_CALC_H */

/*** end of file ***/
