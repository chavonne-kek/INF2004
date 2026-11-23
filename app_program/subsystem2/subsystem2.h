/** @file subsystem2.h
 *
 * @brief Subsystem 2 (Buddy 2): motors, encoders, PID speed control and
 *        odometry (TSK_MOTION).  PUBLIC interface.
 *
 * Owner: Kai Xuan (student ID 2501473).
 *
 * Interfaces provided (TEAM.md section 5):
 *   IF-02  S2 -> S3, S4, S5  odometry: one 32-bit word, single writer,
 *                            read with subsystem2_odometry_read().
 *   IF-04  S4 -> S2          motion commands in mbf_motion_cmd (created
 *                            here, the receiver); only TSK_CTRL sends,
 *                            through subsystem2_command().
 *   GPIO   S2 owns the one GPIO bank-0 interrupt (encoders) and lets S4
 *                            attach the HC-SR04 echo pin to it.
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include <stdint.h>
#include <stdbool.h>

#include "../app.h"

/* Task (TEAM.md section 4). */
#define S2_PRI_MOTION          14      /* Core 1, every 10 ms.  */
#define S2_PERIOD_MS           10u

/* Pins (TEAM.md section 2). */
#define S2_PIN_MOT_L_A         8u      /* M1A, PWM slice 4A. */
#define S2_PIN_MOT_L_B         9u      /* M1B, PWM slice 4B. */
#define S2_PIN_MOT_R_A         10u     /* M2A, PWM slice 5A. */
#define S2_PIN_MOT_R_B         11u     /* M2B, PWM slice 5B. */
/* Encoders (build log 26 Oct): one Grove port each, so each encoder has its
 * own 3V3 and GND.  Grove 1 (GP0/GP1) needs CRASH_REPORT=0 and the USB
 * console (UART0 also uses GP0/GP1; drv_encoder.c stops such builds).
 * Both signal pins of each port are watched and the wheel is counted on
 * the one that carries the pulses (drv_encoder.h), so the white/yellow
 * order does not matter.  A/B only matter with ENC_USE_PHASE_B = 1. */
#define S2_PIN_ENC_L_A         0u      /* Left encoder, Grove 1 white.   */
#define S2_PIN_ENC_L_B         1u      /* Left encoder, Grove 1 yellow.  */
#define S2_PIN_ENC_R_A         3u      /* Right encoder, Grove 2 yellow. */
#define S2_PIN_ENC_R_B         2u      /* Right encoder, Grove 2 white.  */

/* ---------------------------------------------------------------------
 * Motion command (IF-04), 12 bits in a 16-bit message, little-endian:
 *   bits 0-3   opcode (S2_OP_*; 4 bits since S2_OP_SET_GAIN, 31 Oct)
 *   bits 4-11  argument: cm (MOVE), degrees (TURN), cm/s (FOLLOW, SPEED)
 *   bits 12-15 zero
 * ------------------------------------------------------------------- */
typedef enum
{
    S2_OP_STOP = 0,     /* Brake now (same 10 ms cycle).            */
    S2_OP_FOLLOW,       /* Follow the line at arg cm/s.             */
    S2_OP_MOVE_FWD,     /* Drive arg cm forward, then MOTION_DONE.  */
    S2_OP_MOVE_BACK,    /* Drive arg cm backward, then MOTION_DONE. */
    S2_OP_TURN_LEFT,    /* Spin arg degrees left, then MOTION_DONE. */
    S2_OP_TURN_RIGHT,   /* Spin arg degrees right, then MOTION_DONE.*/
    S2_OP_SPEED,        /* Set the speed limit to arg cm/s.         */
    S2_OP_RESET,        /* Leave STALLED, back to BRAKED.           */
    S2_OP_SET_GAIN      /* Steering gain in 10 mm/s per mm (arg).   */
} subsystem2_op_t;

#define S2_CMD_OP_MASK         0x000Fu
#define S2_CMD_ARG_SHIFT       4u
#define S2_CMD_ARG_MASK        0x00FFu /* Applied AFTER the shift.  1 Nov:
                                        * was 0x0FF0, which cleared the low
                                        * 4 bits of every argument: speed
                                        * 12 cm/s -> 0 (so the 5 cm/s
                                        * minimum), 90 deg -> 80, 60 -> 48,
                                        * 120 -> 112, 5 cm -> 0.            */
#define S2_CMD_BYTES           2u

/* ---------------------------------------------------------------------
 * Odometry word (IF-02), 29 bits in one 32-bit word:
 *   bits  0-14  distance travelled, mm, WRAPS at 32768: readers take
 *               differences with subsystem2_distance_delta()
 *   bits 15-21  left wheel speed:  (raw - 64) x 5 mm/s  (-320..+315)
 *   bits 22-28  right wheel speed: (raw - 64) x 5 mm/s
 *   bits 29-31  zero
 * ------------------------------------------------------------------- */
#define S2_DIST_MASK           0x7FFFu
#define S2_SPEED_SHIFT_L       15u
#define S2_SPEED_SHIFT_R       22u
#define S2_SPEED_MASK          0x7Fu
#define S2_SPEED_OFFSET        64
#define S2_SPEED_STEP_MM_S     5

typedef struct
{
    uint16_t distance_mm;   /* 0..32767, wraps.               */
    int16_t  left_mm_s;     /* Signed wheel speed, 5 mm/s steps. */
    int16_t  right_mm_s;
} subsystem2_odom_t;

/* Motion statechart 6.5 states, 3 bits. */
typedef enum
{
    S2_STATE_BRAKED = 0,
    S2_STATE_FOLLOW,
    S2_STATE_MOVE,
    S2_STATE_TURN,
    S2_STATE_STOPPING,
    S2_STATE_STALLED
} subsystem2_state_t;

/* Callback for an attached GPIO pin.  Runs in the GPIO interrupt: keep it
 * to a few microseconds and call only tk_sig_sem / tk_set_flg /
 * tk_wup_tsk.  b_high = pin level after the edge; time_us = TIMERAWL.
 */
typedef void (*subsystem2_gpio_cb_t)(bool b_high, uint32_t time_us);

/* --- Commands (TSK_CTRL only) ---------------------------------------- */
ER       subsystem2_command(subsystem2_op_t op, uint8_t arg);

/* The motion API named in the Project Write-up (Buddy 2).  Each queues one
 * command and returns at once; MOVE/TURN end with EVT_MOTION_DONE.
 * Distances in cm (1..255), angles in degrees (1..255). */
ER       subsystem2_move_forward(uint8_t distance_cm);
ER       subsystem2_move_backward(uint8_t distance_cm);
ER       subsystem2_turn_left(uint8_t angle_deg);
ER       subsystem2_turn_right(uint8_t angle_deg);
ER       subsystem2_stop(void);

/* --- Readers (any task, any core) ------------------------------------ */
void     subsystem2_odometry_read(subsystem2_odom_t * p_odom);
uint32_t subsystem2_odometry_word(void);
int16_t  subsystem2_distance_delta(uint16_t now_mm, uint16_t before_mm);
void     subsystem2_encoder_counts(uint16_t * p_left, uint16_t * p_right);
subsystem2_state_t subsystem2_state(void);
uint32_t subsystem2_alive_count(void);

/* --- Shared GPIO interrupt (call in subsystemN_init only) ------------ */
ER       subsystem2_gpio_attach(uint8_t pin, subsystem2_gpio_cb_t cb);

/* --- Packing helpers (also used by subsystem 5 and the tests) -------- */
uint16_t subsystem2_cmd_pack(subsystem2_op_t op, uint8_t arg);
uint32_t subsystem2_odom_pack(subsystem2_odom_t const * p_odom);
void     subsystem2_odom_unpack(uint32_t word, subsystem2_odom_t * p_odom);

#endif /* SUBSYSTEM2_H */

/*** end of file ***/
