/** @file subsystem4.h
 *
 * @brief Subsystem 4 (Buddy 5): ultrasonic obstacle sensing, servo scan
 *        and the vehicle controller (TSK_SCAN + TSK_CTRL).  PUBLIC
 *        interface: only what other subsystems may use is declared here.
 *
 * Owner: Soma (student ID 2402033).
 *
 * Subsystem 4 owns the two team-wide event flags:
 *   flg_events  IF-03  "something happened" bits; TSK_CTRL is the only
 *                      waiter.  Any task or handler may set a bit.
 *   flg_faults         fault bitmask (Change Requirement 1).  A fault bit
 *                      is set and cleared ONLY by the subsystem that
 *                      detects that fault (Tribunal 1, Version C rule).
 *                      TSK_CTRL only reads it.
 *
 * Scanning, profiling, planning and the bypass are Buddy 5's engine
 * (b5_*.c), running in TSK_SCAN.  It never drives the motors itself: it
 * asks TSK_CTRL, the only motion commander (IF-05, private to S4).
 *
 * Other interfaces provided:
 *   IF-05  TSK_SCAN <-> TSK_CTRL  avoidance hand-over (private to S4).
 *   S4 owns mbf_remote_cmd: subsystem 5 forwards validated MQTT commands
 *          to it (receiver creates the buffer).
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include <stdint.h>
#include <stdbool.h>

#include "../app.h"

/* Tasks (TEAM.md section 4). */
#define S4_PRI_CTRL            18      /* Core 1, event driven.      */
#define S4_PRI_SCAN            19      /* Core 0, 60 ms ping loop.   */

/* Pins (TEAM.md section 2). */
#define S4_PIN_US_TRIG         4u      /* HC-SR04 trigger (Grove 3).    */
#define S4_PIN_US_ECHO         5u      /* HC-SR04 echo (module on 3V3).  */
#define S4_PIN_SERVO           15u     /* Scan servo, PWM slice 7B.     */
#define S4_PIN_BTN_START       20u     /* On-board button, active low.  */
#define S4_PIN_BTN_RESET       21u     /* On-board button, active low.  */
#define S4_PIN_BUZZER          22u     /* On-board piezo, PWM slice 3A. */

/* ---------------------------------------------------------------------
 * flg_events bit allocation (IF-03).  Bits 0-3 MUST equal the S1_EVT_*
 * values in subsystem1.h (checked at compile time in subsystem4.c).
 * ------------------------------------------------------------------- */
#define EVT_BARCODE            (1u << 0)   /* S1: barcode message queued. */
#define EVT_LINE_LOST          (1u << 1)   /* S1: no line for 300 ms.     */
#define EVT_LINE_FOUND         (1u << 2)   /* S1: line seen again.        */
#define EVT_JUNCTION           (1u << 3)   /* S1: junction for 60 ms.     */
#define EVT_MOTION_DONE        (1u << 4)   /* S2: MOVE or TURN finished.  */
#define EVT_HUMP               (1u << 5)   /* S3: hump started or ended.  */
#define EVT_OBSTACLE           (1u << 6)   /* S4: obstacle confirmed: scan
                                              engine asks for control.    */
#define EVT_AVOID_REQ          (1u << 7)   /* S4: scan engine queued a
                                              motion / hand-back request. */
#define EVT_REMOTE_CMD         (1u << 8)   /* S5: remote command queued.  */
#define EVT_FAULT              (1u << 9)   /* Any: a fault bit was set.   */
#define EVT_ALL                0x03FFu     /* 10 of 32 bits used.         */

/* ---------------------------------------------------------------------
 * flg_faults bit allocation (6 bits; heartbeat carries them as-is).
 * Owner = the only subsystem allowed to set AND clear the bit.
 * ------------------------------------------------------------------- */
#define FAULT_STALL            (1u << 0)   /* Owner S2: motor stalled.    */
#define FAULT_ENCODER          (1u << 1)   /* Owner S2: encoder silent.   */
#define FAULT_IMU              (1u << 2)   /* Owner S3: I2C/IMU failure.  */
#define FAULT_NO_ECHO          (1u << 3)   /* Owner S4: HC-SR04 silent.   */
#define FAULT_WIFI             (1u << 4)   /* Owner S5: link/broker lost. */
#define FAULT_LIFTED           (1u << 5)   /* Owner S3: lifted/overturned
                                            * (Nov 2026; was COLLISION). */
#define FAULT_ALL              0x003Fu
#define FAULT_BITS             6u

/* ---------------------------------------------------------------------
 * System state (statechart section 5), 4 bits.
 * ------------------------------------------------------------------- */
typedef enum
{
    SYS_INIT_CALIBRATE = 0,
    SYS_IDLE,
    SYS_LINE_FOLLOW,
    SYS_TURN_EXECUTE,
    SYS_OBSTACLE_AVOID,
    SYS_LINE_RECOVERY,
    SYS_PAUSED,
    SYS_EMERGENCY_STOP,
    SYS_FAULT,
    SYS_FINISHED,
    SYS_STATE_COUNT             /* 10 states fit in 4 bits. */
} subsystem4_state_t;

#define S4_STATE_MASK          0x0Fu

/* ---------------------------------------------------------------------
 * Remote command byte (IF-07 -> S4), built by subsystem 5:
 *   bits 0-2  opcode (S4_RCMD_*)
 *   bits 3-7  speed in cm/s (5..25) for S4_RCMD_SPEED, else 0
 * ------------------------------------------------------------------- */
#define S4_RCMD_START          1u
#define S4_RCMD_PAUSE          2u
#define S4_RCMD_RESUME         3u
#define S4_RCMD_ESTOP          4u
#define S4_RCMD_RESET          5u
#define S4_RCMD_SPEED          6u
#define S4_RCMD_GAIN           7u      /* arg x 100 = mm/s per mm      */
#define S4_RCMD_GAIN_MAX       25u     /* 2500: S2 takes 0..255 x 10.  */

/* Map a barcode character to a movement (Project Write-up: the four demo
 * characters are announced by the instructor and sent from the PC, so the
 * mapping is not fixed in the code).  This is the one 2-byte command:
 *   byte 0 = S4_RCMD_MAP
 *   byte 1 = (movement << S4_RCMD_MAP_SHIFT) | letter index (0 = 'A')
 * Movement uses subsystem1_cmd_t: 1 left, 2 right, 3 straight, 4 U-turn. */
#define S4_RCMD_MAP            8u
#define S4_RCMD_MAP_SHIFT      5u
#define S4_RCMD_MAP_INDEX_MASK 0x1Fu
#define S4_RCMD_MAP_LETTERS    26u     /* 'A'..'Z'                     */

/* Records forwarded to TSK_CTRL: {opcode, argument}. */
#define S4_RCMD_REC_LEN        2u

#define S4_RCMD_OP_MASK        0x07u
#define S4_RCMD_ARG_SHIFT      3u
#define S4_RCMD_SPEED_MIN      2u   /* 2 Nov: 5 -> 2 to match the motion floor
                                    * MOTION_SPEED_MIN_MM_S = 20 mm/s.  With 5
                                    * here, 'speed 3' was rejected on arrival
                                    * even though the motors can drive it. */
#define S4_RCMD_SPEED_MAX      25u

/* ---------------------------------------------------------------------
 * Obstacle profile for telemetry (statechart 6.1), 35 bits packed into 5
 * bytes, little-endian (Change Requirement 1; Version B Constraint
 * Injection).  Converted from Buddy 5's in-RAM B5_Profile_t:
 *   bits  0-6   width, cm (0..126, 127 = unknown)
 *   bits  7-13  closest distance, cm (0..126, 127 = none)
 *   bits 14-18  angle of closest point, scan index (0..24 = 30..150 deg,
 *               clamped)
 *   bits 19-25  left clearance: free lateral gap beside the obstacle, cm
 *               (127 = open)
 *   bits 26-32  right clearance, same coding
 *   bit  33     obstacle detected
 *   bit  34     spare (0)
 * ------------------------------------------------------------------- */
#define S4_PROFILE_BYTES       5u

typedef struct
{
    uint8_t width_cm;
    uint8_t closest_cm;
    uint8_t closest_index;
    uint8_t left_clear_cm;
    uint8_t right_clear_cm;
    bool    b_left_edge;        /* Telemetry: obstacle detected.  */
    bool    b_right_edge;       /* Spare (always false).          */
} subsystem4_profile_t;

/* Range values: 7 bits, 127 = nothing in range / no echo. */
#define S4_RANGE_NONE          127u

/* Scan index: 5 bits, 0..24 = 30..150 degrees in 5 degree steps. */
#define S4_SCAN_INDEX_MAX      24u

/* --- Event and fault API (any subsystem) ---------------------------- */
void     subsystem4_event_set(uint32_t bits);
void     subsystem4_fault_raise(uint32_t fault);
void     subsystem4_fault_clear(uint32_t fault);
uint32_t subsystem4_faults(void);
ID       subsystem4_flg_events(void);

/* --- State and data readers (any subsystem) -------------------------- */
subsystem4_state_t subsystem4_state(void);

/* Console names (debug prints): "LINE_FOLLOW", ...; and the set fault
 * bits as words, e.g. "STALL IMU" ("none" if 0).  The text buffer needs
 * S4_FAULT_TEXT_SIZE bytes. */
#define S4_FAULT_TEXT_SIZE     48u
char const * subsystem4_state_name(subsystem4_state_t state);
char const * subsystem4_fault_text(uint32_t faults,
                                   char text[S4_FAULT_TEXT_SIZE]);
uint8_t  subsystem4_range_cm(void);
#if APP_TEST
/* Test and check images: START presses so far (GP20 or dashboard).  In
 * test images a START is only counted and beeps; the car does not drive. */
uint32_t subsystem4_start_presses(void);
#endif
ID       subsystem4_remote_cmd_mbf(void);

/* --- Packing helpers (shared with subsystem 5 / the dashboard) ------- */
void     subsystem4_profile_pack(subsystem4_profile_t const * p_profile,
                                 uint8_t bytes[S4_PROFILE_BYTES]);
void     subsystem4_profile_unpack(uint8_t const bytes[S4_PROFILE_BYTES],
                                   subsystem4_profile_t * p_profile);

#endif /* SUBSYSTEM4_H */

/*** end of file ***/
