/** @file imu_glue.h
 *
 * @brief IS25 adapter between Buddy 4's terrain maths (imu.c, unchanged
 *        algorithms) and the team interfaces of subsystem 3:
 *          - robot-frame accelerometer + subsystem 2 odometry -> IMU_SAMPLE
 *            (sign convention, encoder acceleration, distance in um);
 *          - IMU_STATUS -> the team's IMU status word (IF-09), EVT_HUMP
 *            edges, the hump record and the lift / overturn edge.
 *        Pure logic: no kernel calls, tested on a PC
 *        (testing/subsystem3/host).
 */

#ifndef IMU_GLUE_H
#define IMU_GLUE_H

#include <stdint.h>
#include <stdbool.h>

#include "subsystem3.h"
#include "imu.h"

/* Encoder acceleration (Tribunal 1, Version B: S3 derives it from the
 * odometry speeds itself).  5 samples = 50 ms difference window.
 * TODO(Zi Siang): with a coarse encoder (under ~150 edges per wheel turn,
 * see the S2-M01 test output) one edge more or less in S2's speed window
 * reads as about 0.1 g here, i.e. a false 6 degree pitch; then raise the
 * window to 20 (200 ms).  The scale below follows the window. */
#define GLUE_ACC_WINDOW        5u
#define GLUE_MG_PER_MM_S2_NUM  (100000 / (int32_t)GLUE_ACC_WINDOW)
                                       /* mg = dv(mm/s) x 1000 /
                                          (window x 10 ms x 9806.65 mm/s^2) */
#define GLUE_MG_PER_MM_S2_DEN  9807
#define GLUE_TURNING_MM_S      60      /* Wheel speed difference.       */
#define GLUE_STILL_MM_S        30      /* Both wheels below: stopped.   */

/* Events returned by imu_glue_events(). */
#define GLUE_EVT_HUMP_START    (1u << 0)
#define GLUE_EVT_HUMP_END      (1u << 1)   /* With or without a report. */
#define GLUE_EVT_HUMP_REPORT   (1u << 2)   /* last_mm is valid.         */
#define GLUE_EVT_LIFTED        (1u << 3)   /* Lifted / overturned edge. */

/* One 10 ms input in ROBOT axes, as the LSM303 really reports it
 * (specific force): level and still = (0, 0, +1000) mg; nose up by theta
 * -> fwd_mg = +1000 sin(theta); speeding up -> fwd_mg rises. */
typedef struct
{
    int16_t fwd_mg;
    int16_t left_mg;
    int16_t up_mg;
    int16_t left_mm_s;      /* Subsystem 2 odometry word. */
    int16_t right_mm_s;
    int16_t delta_mm;       /* subsystem2_distance_delta() since last.   */
    bool    b_odom_valid;   /* S2 has published and FAULT_ENCODER clear. */
} imu_glue_in_t;

typedef struct
{
    int16_t  speed_hist[GLUE_ACC_WINDOW];
    uint8_t  hist_index;
    uint8_t  hist_fill;
    uint8_t  hump_prev;     /* IMU_HUMP_* of the previous sample.  */
    uint16_t count_prev;    /* imu_hump_count() of the previous.   */
    bool     b_lift_prev;
    uint8_t  last_mm;       /* Last reported hump, saturated 127.  */
    uint8_t  max_mm;        /* Highest hump of the run.            */
} imu_glue_t;

void     imu_glue_init(imu_glue_t * p_glue);
void     imu_glue_sample(imu_glue_t * p_glue, imu_glue_in_t const * p_in,
                         IMU_SAMPLE * p_out);
uint8_t  imu_glue_events(imu_glue_t * p_glue, IMU_STATUS const * p_imu,
                         uint16_t hump_count);
bool     imu_glue_turning(int16_t left_mm_s, int16_t right_mm_s);
void     imu_glue_status(IMU_STATUS const * p_imu, bool b_turning,
                         bool b_fault, uint16_t heading_deg,
                         subsystem3_status_t * p_status);
int16_t  imu_glue_pitch_q(int16_t pitch_cdeg);

/* Team formats (moved here from the skeleton's imu_calc.c). */
uint32_t imu_status_pack(subsystem3_status_t const * p_status);
void     imu_status_unpack(uint32_t word, subsystem3_status_t * p_status);
uint16_t imu_hump_record_pack(uint8_t last_mm, uint8_t max_mm);
uint16_t imu_heading_deg(float mx, float my);

#endif /* IMU_GLUE_H */

/*** end of file ***/
