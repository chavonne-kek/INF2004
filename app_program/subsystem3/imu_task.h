/** @file imu_task.h
 *
 * @brief TSK_IMU internals.  Private to subsystem 3.
 */

#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <stdint.h>
#include <stdbool.h>

#include "subsystem3.h"
#include "drv_lsm303.h"

ER       imu_task_init(void);
ER       imu_task_start(void);
uint32_t imu_task_status_word(void);
uint16_t imu_task_hump_record(void);
ID       imu_task_cyclic_id(void);
uint32_t imu_task_loops(void);
uint32_t imu_task_hump_events(void);
uint32_t imu_task_lift_events(void);
uint8_t  imu_task_lift_cause(void);
void     imu_task_lift_rearm(void);
void     imu_task_cal_begin(void);
bool     imu_task_cal_end(void);

#if APP_TEST
void     imu_task_sim_start(uint8_t shape, uint16_t height_mm,
                            uint16_t length_mm, uint16_t speed_mm_s,
                            int16_t tilt_cdeg);
void     imu_task_sim_stop(void);

/* S3 check image: the last raw readings and how many reads worked. */
typedef struct
{
    lsm303_vec_t acc;           /* Accelerometer, mg (sensor axes).   */
    lsm303_vec_t mag;           /* Magnetometer, raw counts.          */
    uint32_t     reads_ok;
    uint32_t     reads_failed;
    int16_t      mag_span_x;    /* Max - min seen in the last          */
    int16_t      mag_span_y;    /* calibration (needs > 100 each).    */
    int16_t      lift_speed_mm_s;   /* Lift detector: upward speed.    */
    int16_t      up_pct;            /* % of g on the up axis.          */
    int16_t      gravity_mg;        /* Learned |g|.                    */
} imu_task_raw_t;

void     imu_task_request_check(void);
void     imu_task_request_scan(bool b_sweep);     /* Bus scan in TSK_IMU. */
void     imu_task_raw_get(imu_task_raw_t * p_raw);
#endif

#endif /* IMU_TASK_H */

/*** end of file ***/
