/** @file subsystem3.h
 *
 * @brief Subsystem 3 (Buddy 4): IMU (LSM303D; LSM303DLHC also works),
 *        pitch, heading, hump detection and lift / overturn detection
 *        (TSK_IMU).  PUBLIC interface.
 *
 * Owner: Zi Siang (student ID 2500989).
 *
 * Since 26 Oct the terrain maths is Zi Siang's own imu.c (integer pitch,
 * hump integral), driven through imu_glue.c.  See BUDDY4_INTEGRATION.md.
 *
 * Nov 2026 (lecturer's change): IMU collision detection is REMOVED.  The
 * IMU instead detects the robot being LIFTED or OVERTURNED, stops the
 * motors at once (TSK_IMU sends STOP itself, then raises FAULT_LIFTED),
 * and a lift ends the demo run (TSK_CTRL enters FINISHED).  The detector
 * latches; subsystem3_lift_rearm() (TSK_CTRL, at the start of each run)
 * arms it again.  Obstacles are the ultrasonic's job (subsystem 4).
 *
 * Interfaces provided:
 *   IF-03  EVT_HUMP in flg_events when a hump starts and when it ends;
 *          FAULT_LIFTED and FAULT_IMU in flg_faults (S3 owns both bits).
 *   IMU status word: one 32-bit word, single writer, read with
 *          subsystem3_status_read() (used by S4 and S5).
 *   Hump record: 14 bits in 2 bytes, posted to telemetry by S3.
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include <stdint.h>
#include <stdbool.h>

#include "../app.h"

/* Task (TEAM.md section 4). */
#define S3_PRI_IMU             16      /* Core 0, every 10 ms (100 Hz). */
#define S3_PERIOD_MS           10u

/* Pins: I2C0 on GP16 (SDA) / GP17 (SCL), kernel I2C driver "iica". */
#define S3_PIN_SDA             16u
#define S3_PIN_SCL             17u

/* Hump detector states (statechart 6.3), 3 bits.  From imu.c:
 * IDLE -> FLAT, RISING -> CLIMBING, FALLING -> DESCENDING; INVALID while
 * the sensor is faulty.  CRESTING is kept for the wire format but is not
 * produced (his detector goes straight from rising to falling). */
typedef enum
{
    S3_HUMP_FLAT = 0,
    S3_HUMP_CLIMBING,
    S3_HUMP_CRESTING,
    S3_HUMP_DESCENDING,
    S3_HUMP_INVALID
} subsystem3_hump_state_t;

/* Motion event classes (FR10), 3 bits.  Values 0-5 are unchanged; 6 and 7
 * were added on 26 Oct for imu.c's extra classes (S5 dashboard: add the
 * two names). */
typedef enum
{
    S3_MOTION_STATIONARY = 0,
    S3_MOTION_ACCELERATING,
    S3_MOTION_TURNING,
    S3_MOTION_CLIMBING,
    S3_MOTION_DESCENDING,
    S3_MOTION_LIFTED,           /* Lifted or overturned (was IMPACT). */
    S3_MOTION_DECELERATING,
    S3_MOTION_CRUISING
} subsystem3_motion_t;

/* ---------------------------------------------------------------------
 * IMU status word, 24 bits in one 32-bit word:
 *   bits  0-7   pitch: (raw - 128) x 0.25 deg  (-32..+31.75), + = nose up
 *   bits  8-16  heading 0..359 deg (511 = not calibrated)
 *   bits 17-19  hump state (subsystem3_hump_state_t)
 *   bits 20-22  motion class (subsystem3_motion_t)
 *   bit  23     hump in progress (HUMP active: S4 ignores far obstacles)
 *   bits 24-31  zero
 * ------------------------------------------------------------------- */
#define S3_PITCH_MASK          0xFFu
#define S3_PITCH_OFFSET        128
#define S3_HEADING_SHIFT       8u
#define S3_HEADING_MASK        0x1FFu
#define S3_HEADING_NONE        511u
#define S3_HUMP_SHIFT          17u
#define S3_HUMP_MASK           0x7u
#define S3_MOTION_SHIFT        20u
#define S3_MOTION_MASK         0x7u
#define S3_HUMP_ACTIVE_BIT     (1u << 23)

typedef struct
{
    int16_t  pitch_q;       /* Pitch in 0.25 degree steps.   */
    uint16_t heading_deg;   /* 0..359, or S3_HEADING_NONE.   */
    subsystem3_hump_state_t hump;
    subsystem3_motion_t     motion;
    bool     b_hump_active;
} subsystem3_status_t;

/* ---------------------------------------------------------------------
 * Hump record (telemetry), 14 bits in 2 bytes, little-endian:
 *   bits 0-6   height of the last hump, mm (0..127, saturates)
 *   bits 7-13  highest hump of the run, mm (0..127, saturates)
 * ------------------------------------------------------------------- */
#define S3_HUMP_MM_MASK        0x7Fu
#define S3_HUMP_MAX_SHIFT      7u
#define S3_HUMP_RECORD_BYTES   2u

/* --- Readers (any task, any core) ------------------------------------ */
void     subsystem3_status_read(subsystem3_status_t * p_status);
uint32_t subsystem3_status_word(void);
bool     subsystem3_hump_active(void);
uint16_t subsystem3_hump_record(void);

/* --- Lift / overturn (FAULT_LIFTED) --------------------------------- */
#define S3_LIFT_NONE           0u
#define S3_LIFT_LIFTED         1u      /* Picked up (upward speed).      */
#define S3_LIFT_OVERTURNED     2u      /* Tilted over 60 deg / upside down. */
uint8_t  subsystem3_lift_cause(void);  /* S3_LIFT_*, latched.            */
void     subsystem3_lift_rearm(void);  /* TSK_CTRL: a new run starts.    */

/* --- Calibration (called by TSK_CTRL in INIT_CALIBRATE) -------------- */
void     subsystem3_calibrate_begin(void);
bool     subsystem3_calibrate_end(void);

/* --- Packing helpers -------------------------------------------------- */
uint32_t subsystem3_status_pack(subsystem3_status_t const * p_status);
void     subsystem3_status_unpack(uint32_t word,
                                  subsystem3_status_t * p_status);

#endif /* SUBSYSTEM3_H */

/*** end of file ***/
