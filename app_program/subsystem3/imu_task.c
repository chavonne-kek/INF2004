/** @file imu_task.c
 *
 * @brief TSK_IMU (priority 16, core 0), woken every 10 ms by a cyclic
 *        handler.  Reads the accelerometer every cycle and the
 *        magnetometer every third cycle (30 Hz), runs Zi Siang's terrain
 *        maths (imu.c) through the adapter (imu_glue.c), and publishes the
 *        IMU status word in one store.
 *
 * Owns FAULT_IMU and FAULT_LIFTED (sets and clears them).
 *
 * Lift / overturn (Nov 2026, replaces the collision rule): the moment
 * imu.c latches IMU_FLAG_LIFTED this task sends STOP to TSK_MOTION itself
 * (same 10 ms cycle, no wait for TSK_CTRL), then raises FAULT_LIFTED so
 * TSK_CTRL ends the run.  The fault stays until TSK_CTRL starts a new run
 * (subsystem3_lift_rearm), so a car held in the air cannot drive off.
 *
 * Test builds (make TEST=1) can replace the sensor with Zi Siang's
 * synthetic generator (imu_task_sim_start), so the hump and lift paths
 * are proven on the Pico with or without the IMU fitted.
 */

#include "subsystem3.h"
#include "imu_task.h"
#include "imu.h"
#include "imu_glue.h"
#include "drv_lsm303.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"

/* Mounting: which sensor axis points forward / left.  TODO(Zi Siang):
 * check on the chassis (tilt the nose up: pitch must go positive). */
#define IMU_FWD_FROM_X         1       /* 1: forward = +X, 0: forward = +Y. */
#define IMU_FWD_SIGN           1       /* -1 if the sensor faces backwards. */
#define IMU_LEFT_SIGN          1
#define IMU_UP_SIGN            1       /* -1 if mounted upside down.        */

#define IMU_MAG_EVERY          3u      /* 100 Hz / 3 = 33 Hz >= 30 Hz ODR.  */
#define IMU_FAULT_AFTER        5u      /* Bad reads in a row -> FAULT_IMU.  */
#define IMU_CLEAR_AFTER        5u      /* Good reads in a row -> clear.     */
#define IMU_REPROBE_CYCLES     100u    /* While faulty: re-probe every 1 s. */
#define IMU_CAL_IDLE           0u
#define IMU_CAL_RUNNING        1u
#define IMU_CAL_DONE_OK        2u
#define IMU_CAL_DONE_FAIL      3u
#define IMU_CAL_MIN_SAMPLES    50u
#define IMU_CAL_MIN_SPAN       100     /* Magnetometer counts.              */
#define IMU_CAL_WAIT_TRIES     10u
#define IMU_PRINT_EVENTS       1
#define CHECK_REQ_DIAGNOSE     0x01u
#define CHECK_REQ_SCAN         0x02u
#define CHECK_REQ_SWEEP        0x04u

static ID gh_imu_task = 0;
static ID gh_imu_cyc  = 0;

/* Published: one writer (this task). */
static volatile uint32_t g_status_word      = 0u;
static volatile uint16_t g_hump_record      = 0u;
static volatile uint8_t  g_cal_command      = IMU_CAL_IDLE;
static volatile uint8_t  g_cal_status       = IMU_CAL_IDLE;
static volatile uint32_t g_loops            = 0u;
static volatile uint32_t g_hump_events      = 0u;
static volatile uint32_t g_lift_events      = 0u;
static volatile uint8_t  g_lift_cause       = S3_LIFT_NONE;
static volatile uint8_t  g_rearm_req        = 0u;

#if APP_TEST
static volatile uint8_t  g_sim_on    = 0u;
static volatile uint8_t  g_reset_req = 0u;
static volatile uint8_t  g_check_req = 0u;     /* CHECK_REQ_* bits.    */
static imu_task_raw_t    g_raw;                /* Check image readout.  */
#endif

/* Private to the task. */
static imu_glue_t g_glue;
static bool       gb_lifted         = false;
static int32_t    g_acc_offset_fwd  = 0;     /* Zero-g offsets, mg. */
static int32_t    g_acc_offset_left = 0;
static int16_t    g_mag_offset_x    = 0;     /* Hard-iron offsets.  */
static int16_t    g_mag_offset_y    = 0;
static bool       gb_mag_calibrated = false;
static uint16_t   g_heading         = S3_HEADING_NONE;

/* Calibration accumulators. */
static int32_t      g_cal_sum_fwd  = 0;
static int32_t      g_cal_sum_left = 0;
static uint32_t     g_cal_count    = 0u;
static lsm303_vec_t g_mag_min;
static lsm303_vec_t g_mag_max;

static void tick_isr(void * p_exinf);
static void imu_thread(INT stacd, void * p_exinf);
static bool sensor_sample(uint32_t cycle, bool * p_b_fault,
                          IMU_SAMPLE * p_sample, bool * p_b_turning);
static void publish(IMU_SAMPLE const * p_sample, bool b_turning,
                    bool b_fault);
static void publish_no_data(void);
static void handle_calibration(int32_t fwd_mg, int32_t left_mg,
                               lsm303_vec_t const * p_mag, bool b_mag);
static void track_faults(bool b_ok, bool * p_b_fault);
static void handle_rearm(void);
#if APP_TEST
static void reset_run(void);
#endif


/*!
 * @brief Create the 10 ms cyclic handler (stopped).
 */
ER
imu_task_init (void)
{
    T_CCYC ccyc = {0};

    imu_init();
    imu_hump_diag(1u);    /* test build: log every hump peak, even a
                             * dropped one, so the height is visible */
    imu_glue_init(&g_glue);
    publish_no_data();      /* Until the first good sample. */

    ccyc.cycatr = TA_HLNG;
    ccyc.cychdr = (FP)tick_isr;     /* Kernel handler type for C code. */
    ccyc.cyctim = S3_PERIOD_MS;
    ccyc.cycphs = S3_PERIOD_MS;
    gh_imu_cyc  = tk_cre_cyc(&ccyc);

    return ((gh_imu_cyc < E_OK) ? (ER)gh_imu_cyc : E_OK);
}

/*!
 * @brief Start TSK_IMU on core 0, then its 10 ms tick.
 */
ER
imu_task_start (void)
{
    ER result = E_OK;

    gh_imu_task = app_start_task((FP)imu_thread, S3_PRI_IMU, TP_PRC1,
                                 "imu_task");   /* FP: kernel entry type. */

    if (gh_imu_task < E_OK)
    {
        result = (ER)gh_imu_task;
    }
    else
    {
        result = tk_sta_cyc(gh_imu_cyc);
    }

    return (result);
}

uint32_t
imu_task_status_word (void)
{
    return (g_status_word);
}

uint16_t
imu_task_hump_record (void)
{
    return (g_hump_record);
}

ID
imu_task_cyclic_id (void)
{
    return (gh_imu_cyc);
}

uint32_t
imu_task_loops (void)
{
    return (g_loops);
}

uint32_t
imu_task_hump_events (void)
{
    return (g_hump_events);
}

uint32_t
imu_task_lift_events (void)
{
    return (g_lift_events);
}

uint8_t
imu_task_lift_cause (void)
{
    return (g_lift_cause);
}

/*!
 * @brief Ask TSK_IMU to clear the lift latch and FAULT_LIFTED and arm the
 *        detector again (done at its next cycle: single writer).
 */
void
imu_task_lift_rearm (void)
{
    g_rearm_req = 1u;
}

/*!
 * @brief Start calibration: keep the robot still on flat ground for at
 *        least 0.5 s (accelerometer zero), and spin it 360 degrees
 *        (magnetometer hard-iron) before imu_task_cal_end().
 */
void
imu_task_cal_begin (void)
{
    g_cal_command = IMU_CAL_RUNNING;
}

/*!
 * @brief Finish calibration.
 *
 * @return True if both offsets were applied.
 */
bool
imu_task_cal_end (void)
{
    uint8_t tries = 0u;

    g_cal_command = IMU_CAL_DONE_OK;    /* Request "finish". */

    for (tries = 0u; tries < IMU_CAL_WAIT_TRIES; tries++)
    {
        if (IMU_CAL_RUNNING != g_cal_status)
        {
            break;
        }

        (void)tk_dly_tsk((TMO)S3_PERIOD_MS);
    }

    return (IMU_CAL_DONE_OK == g_cal_status);
}

#if APP_TEST
/*!
 * @brief Test only: drive TSK_IMU from Zi Siang's synthetic generator
 *        (shapes IMU_SIM_*), starting from a clean state.
 */
void
imu_task_sim_start (uint8_t shape, uint16_t height_mm, uint16_t length_mm,
                    uint16_t speed_mm_s, int16_t tilt_cdeg)
{
    g_sim_on = 0u;
    imu_sim_start(shape, height_mm, length_mm, speed_mm_s, tilt_cdeg);
    g_reset_req = 1u;           /* Reset at the next cycle boundary.  */
    g_sim_on    = 1u;
}

/*!
 * @brief Test only: back to the real sensor, clean state.
 */
void
imu_task_sim_stop (void)
{
    g_sim_on    = 0u;
    g_reset_req = 1u;
}

/*!
 * @brief Test only: ask TSK_IMU to run the I2C wiring check (it prints
 *        one "S3: I2C check" line).  TSK_IMU does it between transfers,
 *        so it never disturbs the bus.
 */
void
imu_task_request_check (void)
{
    g_check_req = (uint8_t)(g_check_req | CHECK_REQ_DIAGNOSE);
}

/*!
 * @brief Test only: ask TSK_IMU to scan I2C0 for every address (and, with
 *        b_sweep, every Grove pin pair).  Done between its own transfers:
 *        a scan from another task would take the pins in the middle of a
 *        TSK_IMU read.
 */
void
imu_task_request_scan (bool b_sweep)
{
    g_check_req = (uint8_t)(g_check_req | CHECK_REQ_SCAN |
                            (b_sweep ? CHECK_REQ_SWEEP : 0u));
}

/*!
 * @brief Test only: last raw readings and read counters (S3 check image).
 */
void
imu_task_raw_get (imu_task_raw_t * p_raw)
{
    *p_raw = g_raw;     /* A display copy: a torn value only lasts 0.5 s. */
    p_raw->mag_span_x = (int16_t)(g_mag_max.x - g_mag_min.x);
    p_raw->mag_span_y = (int16_t)(g_mag_max.y - g_mag_min.y);
    p_raw->lift_speed_mm_s = imu_lift_speed_mm_s();
    p_raw->up_pct          = imu_up_pct();
    p_raw->gravity_mg      = imu_gravity_mg();
}
#endif

/*!
 * @brief Cyclic handler: only wakes the task.
 */
static void
tick_isr (void * p_exinf)
{
    (void)p_exinf;
    (void)tk_wup_tsk(gh_imu_task);
}

/*!
 * @brief TSK_IMU body.
 */
static void
imu_thread (INT stacd, void * p_exinf)
{
    uint32_t cycle       = 0u;
    bool     b_imu_fault = false;
    ER       result      = E_OK;

    (void)stacd;
    (void)p_exinf;

    result = drv_lsm303_open();

    if (E_OK == result)
    {
        tm_printf((UB *)"S3: IMU %s, 100 Hz, +-4 g\n",
                  drv_lsm303_chip_name());
    }
    else
    {
        tm_printf((UB *)"S3: IMU not found on I2C0 GP16/GP17 (%d): no "
                  "LSM303D at 0x1D/0x1E, no LSM303DLHC at 0x19\n", result);
        drv_lsm303_diagnose();

        /* FAULT_IMU now; no reads until a re-probe answers (every 1 s). */
        b_imu_fault = true;
        subsystem4_fault_raise(FAULT_IMU);
        imu_set_fault(1u);
    }

    for (;;)
    {
        IMU_SAMPLE sample;
        bool       b_turning = false;
        bool       b_ok      = true;

        (void)tk_slp_tsk(TMO_FEVR);
        (void)tk_can_wup(TSK_SELF);
        cycle++;
        g_loops++;

        if (0u != g_rearm_req)
        {
            g_rearm_req = 0u;
            handle_rearm();
        }

#if APP_TEST
        if (0u != g_reset_req)
        {
            g_reset_req = 0u;
            reset_run();
        }

        if (0u != g_check_req)
        {
            uint8_t req = g_check_req;

            g_check_req = 0u;

            if (0u != (req & CHECK_REQ_SCAN))
            {
                drv_lsm303_scan();
            }

            if (0u != (req & CHECK_REQ_SWEEP))
            {
                drv_lsm303_sweep();
            }

            if (0u != (req & CHECK_REQ_DIAGNOSE))
            {
                drv_lsm303_diagnose();
            }
        }

        if (0u != g_sim_on)
        {
            imu_sim_next(&sample);  /* The generator supplies odometry too. */
        }
        else
#endif
        {
            b_ok = sensor_sample(cycle, &b_imu_fault, &sample, &b_turning);
        }

        if (b_ok)
        {
#if APP_TEST
            publish(&sample, b_turning, b_imu_fault && (0u == g_sim_on));
#else
            publish(&sample, b_turning, b_imu_fault);
#endif
        }
        else if (b_imu_fault)
        {
            publish_no_data();  /* Readers see INVALID, not old values. */
        }
        else
        {
            /* One bad read: keep the last status (FAULT_IMU after 5). */
        }
    }
}

/*!
 * @brief Status word with no sensor data: pitch 0, heading unknown, hump
 *        INVALID (statechart 6.3), so no reader takes it for a real level
 *        reading (an all-zero word would read as pitch -32 degrees).
 */
static void
publish_no_data (void)
{
    subsystem3_status_t status;

    status.pitch_q       = 0;
    status.heading_deg   = S3_HEADING_NONE;
    status.hump          = S3_HUMP_INVALID;
    status.motion        = S3_MOTION_STATIONARY;
    status.b_hump_active = false;
    g_status_word        = imu_status_pack(&status);      /* One store. */
}

/*!
 * @brief Read the sensor and the odometry and build one IMU_SAMPLE.
 *
 * While FAULT_IMU is set the sensor is not read every cycle: once a
 * second drv_lsm303_open() checks whether it answers again (a loose wire
 * may come back); only then are reads tried, and five good ones in a row
 * clear the fault.  So a missing sensor costs no bus time.
 *
 * @return False if there is no new sample (read failed or skipped).
 */
static bool
sensor_sample (uint32_t cycle, bool * p_b_fault, IMU_SAMPLE * p_sample,
               bool * p_b_turning)
{
    lsm303_vec_t      acc       = {0, 0, 0};
    lsm303_vec_t      mag       = {0, 0, 0};
    subsystem2_odom_t odom;
    imu_glue_in_t     in;
    static uint16_t   last_dist = 0u;
    static bool       b_probed  = false;   /* Re-probe answered: read. */
    bool              b_mag     = false;
    ER                result    = E_OK;
    int32_t           fwd       = 0;
    int32_t           left      = 0;

    if (*p_b_fault && (!b_probed))
    {
        if (0u == (cycle % IMU_REPROBE_CYCLES))
        {
            b_probed = (E_OK == drv_lsm303_open());
        }

        if (!b_probed)
        {
            return (false);     /* Still missing: no read this cycle. */
        }
    }

    result = drv_lsm303_read_accel(&acc);

    if ((E_OK == result) && (0u == (cycle % IMU_MAG_EVERY)))
    {
        b_mag = (E_OK == drv_lsm303_read_mag(&mag));
#if APP_TEST
        if (b_mag)
        {
            g_raw.mag = mag;
        }
#endif

        if (b_mag && gb_mag_calibrated)
        {
            g_heading = imu_heading_deg((float)(mag.x - g_mag_offset_x),
                                        (float)(mag.y - g_mag_offset_y));
        }
    }

    track_faults(E_OK == result, p_b_fault);
    imu_set_fault(*p_b_fault ? 1u : 0u);
#if APP_TEST
    if (E_OK == result)
    {
        g_raw.acc = acc;
        g_raw.reads_ok++;
    }
    else
    {
        g_raw.reads_failed++;
    }
#endif

    if ((E_OK != result) || (!*p_b_fault))
    {
        b_probed = false;   /* Failed again, or the fault has cleared. */
    }

    if (E_OK != result)
    {
        return (false);
    }

    /* Sensor axes -> robot axes (milli-g, as the sensor reports them). */
#if IMU_FWD_FROM_X
    fwd  = IMU_FWD_SIGN * (int32_t)acc.x;
    left = IMU_LEFT_SIGN * (int32_t)acc.y;
#else
    fwd  = IMU_FWD_SIGN * (int32_t)acc.y;
    left = IMU_LEFT_SIGN * (int32_t)acc.x;
#endif
    handle_calibration(fwd, left, &mag, b_mag);

    subsystem2_odometry_read(&odom);
    in.fwd_mg     = (int16_t)(fwd - g_acc_offset_fwd);
    in.left_mg    = (int16_t)(left - g_acc_offset_left);
    in.up_mg      = (int16_t)(IMU_UP_SIGN * (int32_t)acc.z);
    in.left_mm_s  = odom.left_mm_s;
    in.right_mm_s = odom.right_mm_s;
    in.delta_mm   = subsystem2_distance_delta(odom.distance_mm, last_dist);
    in.b_odom_valid = (0u != subsystem2_odometry_word()) &&
                      (0u == (subsystem4_faults() & FAULT_ENCODER));
    last_dist     = odom.distance_mm;

    imu_glue_sample(&g_glue, &in, p_sample);
    *p_b_turning = imu_glue_turning(odom.left_mm_s, odom.right_mm_s);

    return (true);
}

/*!
 * @brief Run imu.c on one sample and publish everything the team reads.
 */
static void
publish (IMU_SAMPLE const * p_sample, bool b_turning, bool b_fault)
{
    IMU_STATUS          imu;
    subsystem3_status_t status;
    uint8_t             events = 0u;

    imu_update(p_sample);
    imu_snapshot(&imu);
    events = imu_glue_events(&g_glue, &imu, imu_hump_count());

    imu_glue_status(&imu, b_turning, b_fault, g_heading, &status);
    g_status_word = imu_status_pack(&status);       /* One store. */

    if (0u != (events & (GLUE_EVT_HUMP_START | GLUE_EVT_HUMP_END)))
    {
        subsystem4_event_set(EVT_HUMP);
    }

    if (0u != (events & GLUE_EVT_HUMP_REPORT))
    {
        uint16_t record = imu_hump_record_pack(g_glue.last_mm, g_glue.max_mm);
        uint8_t  bytes[S3_HUMP_RECORD_BYTES];

        g_hump_record = record;
        g_hump_events++;
        bytes[0] = (uint8_t)(record & 0xFFu);
        bytes[1] = (uint8_t)(record >> 8u);
        (void)subsystem5_post(S5_TLM_HUMP, bytes, S3_HUMP_RECORD_BYTES);

#if IMU_PRINT_EVENTS
        tm_printf((UB *)"S3: hump %u mm (max %u mm)\n",
                  (UINT)g_glue.last_mm, (UINT)g_glue.max_mm);
#endif
    }

    /* LIFTED / OVERTURNED (rubric item 10): stop the motors in THIS
     * cycle - TSK_IMU sends STOP itself rather than waiting for TSK_CTRL -
     * then raise FAULT_LIFTED, which ends the run (TSK_CTRL -> FINISHED).
     * No auto-clear: only the next run's subsystem3_lift_rearm(). */
    if (0u != (events & GLUE_EVT_LIFTED))
    {
        (void)subsystem2_stop();
        gb_lifted    = true;
        g_lift_cause = imu_lift_cause();
        g_lift_events++;
        subsystem4_fault_raise(FAULT_LIFTED);

#if IMU_PRINT_EVENTS
        if (S3_LIFT_OVERTURNED == g_lift_cause)
        {
            tm_printf((UB *)"S3: OVERTURNED (up axis %d %% of g) - motors "
                      "stopped, run over\n", (int)imu_up_pct());
        }
        else
        {
            tm_printf((UB *)"S3: LIFTED (moving up at %d mm/s) - motors "
                      "stopped, run over\n", (int)imu_lift_speed_mm_s());
        }
#endif
    }
}

/*!
 * @brief A new run: release the latch (owner rule), clear FAULT_LIFTED and
 *        arm the detector again (it learns gravity for 1 s first).
 */
static void
handle_rearm (void)
{
    imu_clear(IMU_FLAG_LIFTED);
    g_glue.b_lift_prev = false;
    g_lift_cause       = S3_LIFT_NONE;

    if (gb_lifted)
    {
        gb_lifted = false;
        subsystem4_fault_clear(FAULT_LIFTED);
    }
}

#if APP_TEST
/*!
 * @brief Test reset: clean maths, clean adapter, release our lift fault.
 */
static void
reset_run (void)
{
    imu_init();
    imu_glue_init(&g_glue);

    if (gb_lifted)
    {
        subsystem4_fault_clear(FAULT_LIFTED);
    }

    gb_lifted     = false;
    g_lift_cause  = S3_LIFT_NONE;
    g_hump_events = 0u;
    g_lift_events = 0u;
}
#endif

/*!
 * @brief Accumulate calibration data while a calibration is running.
 */
static void
handle_calibration (int32_t fwd_mg, int32_t left_mg,
                    lsm303_vec_t const * p_mag, bool b_mag)
{
    uint8_t command = g_cal_command;

    if ((IMU_CAL_RUNNING == command) && (IMU_CAL_RUNNING != g_cal_status))
    {
        g_cal_sum_fwd  = 0;
        g_cal_sum_left = 0;
        g_cal_count    = 0u;
        g_mag_min.x = INT16_MAX;
        g_mag_min.y = INT16_MAX;
        g_mag_max.x = INT16_MIN;
        g_mag_max.y = INT16_MIN;
        g_cal_status = IMU_CAL_RUNNING;
    }

    if (IMU_CAL_RUNNING == g_cal_status)
    {
        /* Accelerometer: average only the first second (robot still). */
        if (g_cal_count < (IMU_CAL_MIN_SAMPLES * 2u))
        {
            g_cal_sum_fwd  += fwd_mg;
            g_cal_sum_left += left_mg;
            g_cal_count++;
        }

        if (b_mag)
        {
            g_mag_min.x = (p_mag->x < g_mag_min.x) ? p_mag->x : g_mag_min.x;
            g_mag_min.y = (p_mag->y < g_mag_min.y) ? p_mag->y : g_mag_min.y;
            g_mag_max.x = (p_mag->x > g_mag_max.x) ? p_mag->x : g_mag_max.x;
            g_mag_max.y = (p_mag->y > g_mag_max.y) ? p_mag->y : g_mag_max.y;
        }

        if (IMU_CAL_DONE_OK == command)
        {
            bool b_acc_ok = (g_cal_count >= IMU_CAL_MIN_SAMPLES);
            bool b_mag_ok = ((g_mag_max.x - g_mag_min.x) > IMU_CAL_MIN_SPAN) &&
                            ((g_mag_max.y - g_mag_min.y) > IMU_CAL_MIN_SPAN);

            if (b_acc_ok)
            {
                g_acc_offset_fwd  = g_cal_sum_fwd / (int32_t)g_cal_count;
                g_acc_offset_left = g_cal_sum_left / (int32_t)g_cal_count;
            }

            if (b_mag_ok)
            {
                g_mag_offset_x = (int16_t)((g_mag_max.x + g_mag_min.x) / 2);
                g_mag_offset_y = (int16_t)((g_mag_max.y + g_mag_min.y) / 2);
                gb_mag_calibrated = true;
            }

            g_cal_command = IMU_CAL_IDLE;
            g_cal_status  = (b_acc_ok && b_mag_ok) ? IMU_CAL_DONE_OK
                                                   : IMU_CAL_DONE_FAIL;
        }
    }
}

/*!
 * @brief FAULT_IMU after 5 failed reads in a row; clear after 5 good.
 */
static void
track_faults (bool b_ok, bool * p_b_fault)
{
    static uint8_t bad_reads  = 0u;
    static uint8_t good_reads = 0u;

    if (b_ok)
    {
        bad_reads = 0u;

        if (good_reads < IMU_CLEAR_AFTER)
        {
            good_reads++;
        }

        if (*p_b_fault && (good_reads >= IMU_CLEAR_AFTER))
        {
            *p_b_fault = false;
            subsystem4_fault_clear(FAULT_IMU);

#if IMU_PRINT_EVENTS
            tm_printf((UB *)"S3: IMU back (%s)\n", drv_lsm303_chip_name());
#endif
        }
    }
    else
    {
        good_reads = 0u;

        if (bad_reads < IMU_FAULT_AFTER)
        {
            bad_reads++;
        }

        if ((!*p_b_fault) && (bad_reads >= IMU_FAULT_AFTER))
        {
            *p_b_fault = true;
            subsystem4_fault_raise(FAULT_IMU);

#if IMU_PRINT_EVENTS
            tm_printf((UB *)"S3: IMU read failing\n");
#endif
        }
    }
}

/*** end of file ***/
