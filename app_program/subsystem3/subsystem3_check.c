/** @file subsystem3_check.c
 *
 * @brief S3 check image (make CHECK=3), run after the S3 self-tests:
 *          1. if the IMU (LSM303D) answers: keep still 2 s, then turn the
 *             car one full circle by hand (magnetometer calibration);
 *          2. then, forever, every 0.5 s: raw accelerometer (mg) and
 *             magnetometer, pitch, heading, the lift detector (upward
 *             speed, % of g on the up axis, LIFTED / OVERTURNED) and the
 *             read counters; while the sensor gives no data, the I2C
 *             wiring check and a bus scan every 3 s.
 *          Pick-up test: lift the car - "LIFTED" must appear within a
 *          fraction of a second; turn it on its side - "OVERTURNED".  The
 *          detector re-arms itself 3 s after each detection (in the car
 *          image only a new run re-arms it).
 *        The car does not drive.  Steps and expected values:
 *        IS25_Subsystem_Checks.txt, subsystem 3.
 */

#include "subsystem3.h"
#include "imu_task.h"
#include "../subsystem4/subsystem4.h"

#if APP_TEST

#define CHECK_PRINT_MS         500
#define CHECK_MS_PER_S         1000
#define CHECK_SETTLE_MS        500     /* Synthetic sensor off -> real.   */
#define CHECK_CAL_S            14      /* 2 s still + 12 s one circle.    */
#define CHECK_STILL_S          2
#define CHECK_WIRING_EVERY     6u      /* x 0.5 s = 3 s.                  */
#define CHECK_PITCH_Q_PER_DEG  4
#define CHECK_REARM_PRINTS     6u      /* x 0.5 s = 3 s after a detection. */
#define CHECK_SCAN_WAIT_MS     300     /* TSK_IMU prints the scan.        */

static bool has_data(void);

/*!
 * @brief S3 check: calibration if possible, then the live readout (never
 *        returns).
 */
void
subsystem3_check (void)
{
    imu_task_raw_t      raw;
    subsystem3_status_t status;
    uint32_t            prints = 0u;
    uint32_t            rearm_at = 0u;
    INT                 second = 0;

    (void)tk_dly_tsk((TMO)CHECK_SETTLE_MS);
    {
        /* The I2C time limits count the 1 MHz timer: check its rate
         * against the kernel clock (expect 100000 in 100 ms). */
        uint32_t t0 = in_w(TIMER_TIMERAWL);

        (void)tk_dly_tsk(100);
        tm_printf((UB *)"[CHECK] S3: 1 MHz timer counted %u in 100 ms "
                  "(expect about 100000), watchdog tick 0x%X\n",
                  (UINT)(in_w(TIMER_TIMERAWL) - t0),
                  (UINT)in_w(0x4005802Cu));
    }

    tm_printf((UB *)"[CHECK] S3 IMU (LSM303D) on Grove 4: SDA = GP16, "
              "SCL = GP17, VCC = 3V3, GND = GND. Chip found: %s\n",
              drv_lsm303_chip_name());

    /* What is actually on the bus?  Scan first, then talk about wiring.
     * TSK_IMU runs the scan between its own transfers. */
    imu_task_request_scan(false);
    (void)tk_dly_tsk((TMO)CHECK_SCAN_WAIT_MS);

    if (has_data())
    {
        tm_printf((UB *)"[CHECK] S3 step 1 (%d s): keep the car STILL for %d "
                  "s, then turn it slowly through ONE FULL CIRCLE on the "
                  "floor\n", CHECK_CAL_S, CHECK_STILL_S);
        subsystem3_calibrate_begin();

        for (second = 1; second <= CHECK_CAL_S; second++)
        {
            (void)tk_dly_tsk((TMO)CHECK_MS_PER_S);
            tm_printf((UB *)"[CHECK]   %2d s  %s\n", second,
                      (second < CHECK_STILL_S) ? "keep still" :
                      (second == CHECK_STILL_S) ? "now turn" : "turning");
        }

        imu_task_raw_get(&raw);     /* Spans are read before the end. */
        tm_printf((UB *)"[CHECK] S3 calibration %s: compass span X %d  Y %d "
                  "(each needs more than 100)\n",
                  subsystem3_calibrate_end() ? "OK" : "FAILED",
                  (INT)raw.mag_span_x, (INT)raw.mag_span_y);
    }
    else
    {
        tm_printf((UB *)"[CHECK] S3: NO DATA from the IMU, so no "
                  "calibration. The wiring check below repeats every 3 s "
                  "while you fix it; when 'S3: IMU back' appears, press "
                  "RUN/RESET to calibrate\n");
    }

    tm_printf((UB *)"[CHECK] S3 step 2: readout every 0.5 s. Level car: Z "
              "about +1000 mg, X and Y near 0. Lift the NOSE: pitch must go "
              "POSITIVE. Turn the car: heading must change\n");
    tm_printf((UB *)"[CHECK] S3 step 3 (pick-up test, rubric 10): put the "
              "car down for 1 s, then LIFT it: 'LIFTED' must appear. Put it "
              "down, wait for 'armed', tip it on its side: 'OVERTURNED'\n");

    for (;;)
    {
        (void)tk_dly_tsk((TMO)CHECK_PRINT_MS);
        prints++;
        imu_task_raw_get(&raw);

        if (has_data())
        {
            UB      heading[24];
            int32_t q     = 0;
            int32_t mag_q = 0;

            subsystem3_status_read(&status);
            q     = status.pitch_q;
            mag_q = (q < 0) ? -q : q;

            if (S3_HEADING_NONE == status.heading_deg)
            {
                (void)tm_sprintf(heading, (UB const *)"n/a (no calibration)");
            }
            else
            {
                (void)tm_sprintf(heading, (UB const *)"%u deg",
                                 (UINT)status.heading_deg);
            }

            tm_printf((UB *)"[CHECK] S3: acc X %+5d  Y %+5d  Z %+5d mg | "
                      "pitch %s%d.%02d deg | heading %s | compass X %d  Y %d"
                      "  Z %d | lift: up %+4d mm/s, %3d %% of g on Z, g %d "
                      "mg -> %s | reads ok %u, failed %u\n",
                      (INT)raw.acc.x, (INT)raw.acc.y, (INT)raw.acc.z,
                      (q < 0) ? "-" : "+",
                      (INT)(mag_q / CHECK_PITCH_Q_PER_DEG),
                      (INT)((mag_q % CHECK_PITCH_Q_PER_DEG) * 25),
                      heading, (INT)raw.mag.x, (INT)raw.mag.y,
                      (INT)raw.mag.z, (INT)raw.lift_speed_mm_s,
                      (INT)raw.up_pct, (INT)raw.gravity_mg,
                      (S3_LIFT_LIFTED == subsystem3_lift_cause()) ?
                      "LIFTED" :
                      (S3_LIFT_OVERTURNED == subsystem3_lift_cause()) ?
                      "OVERTURNED" : "armed",
                      (UINT)raw.reads_ok, (UINT)raw.reads_failed);

            /* Bench only: re-arm 3 s after a detection so the pick-up can
             * be repeated without a reset. */
            if (S3_LIFT_NONE != subsystem3_lift_cause())
            {
                if (0u == rearm_at)
                {
                    rearm_at = prints + CHECK_REARM_PRINTS;
                }
                else if (prints >= rearm_at)
                {
                    rearm_at = 0u;
                    subsystem3_lift_rearm();
                    tm_printf((UB *)"[CHECK] S3: lift detector re-armed (put "
                              "the car down: it learns g for 1 s)\n");
                }
                else
                {
                    /* Waiting before re-arming. */
                }
            }
        }
        else if (0u == (prints % CHECK_WIRING_EVERY))
        {
            tm_printf((UB *)"[CHECK] S3: no data from the IMU (re-tried "
                      "every second); wiring check:\n");
            imu_task_request_check();   /* TSK_IMU prints the line. */

            /* Scan the bus here too, so the answer is not lost in the first
             * second after a reset (the USB port re-enumerates and whatever
             * was printed at boot is gone).  Every third pass, widen it to
             * every Grove pair in case the port labels are not the pins.
             * TSK_IMU does both, between its own transfers. */
            {
                static uint8_t sweep_tick = 0u;

                imu_task_request_scan(0u == (sweep_tick % 3u));
                sweep_tick++;
            }
        }
        else
        {
            /* No data: print only with the wiring check, every 3 s. */
        }
    }
}

/*!
 * @brief True while TSK_IMU has real sensor data (no FAULT_IMU).
 */
static bool
has_data (void)
{
    return (0u == (subsystem4_faults() & FAULT_IMU));
}

#endif /* APP_TEST */

/*** end of file ***/
