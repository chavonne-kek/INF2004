/** @file subsystem3_test.c
 *
 * @brief On-target tests for subsystem 3.  Built only with: make TEST=1
 *        IDs S3-U01 ... must match testing/subsystem3/TEST_LOG.md.
 *
 * S3-U06..U11 are Zi Siang's on-target tests (his S4-U03..U07 and S4-U09),
 * moved here with the code on 26 Oct.  They drive TSK_IMU from his
 * synthetic generator, so they pass with or without the IMU fitted.
 * Nov 2026 (lecturer's change): the collision tests S3-U09 and S3-U10 are
 * retired (IDs never reused); S3-U12..U16 test the lift / overturn stop.
 */

#include "subsystem3.h"
#include "imu_task.h"
#include "imu.h"
#include "imu_glue.h"
#include "imu_math.h"
#include "../subsystem4/subsystem4.h"
#include "../test_report.h"

#if APP_TEST

#define TEST_WAIT_MS           200
#define TEST_SIM_WAIT_STEPS    15u     /* x 200 ms: up to 3 s.            */
#define TEST_SPEED_MM_S        250u

/* Wait up to wait_ms for the hump-active bit; true if it was seen. */
static bool
saw_hump_active (TMO wait_ms)
{
    TMO waited = 0;

    while (waited < wait_ms)
    {
        if (subsystem3_hump_active())
        {
            return (true);
        }

        (void)tk_dly_tsk(S3_PERIOD_MS);
        waited += (TMO)S3_PERIOD_MS;
    }

    return (false);
}

void
subsystem3_test (void)
{
    T_RCYC              rcyc;
    subsystem3_status_t in;
    subsystem3_status_t out;
    uint32_t            loops   = 0u;
    uint32_t            wait    = 0u;
    uint16_t            record  = 0u;
    bool                b_active = false;

    TEST_INIT();

    /* S3-U01: cyclic running and the IMU answers (no FAULT_IMU). */
    (void)tk_dly_tsk(TEST_WAIT_MS);
    TEST_CHECK("S3-U01",
               (E_OK == tk_ref_cyc(imu_task_cyclic_id(), &rcyc)) &&
               (0u != (rcyc.cycstat & TCYC_STA)) &&
               (0u == (subsystem4_faults() & FAULT_IMU)),
               "TSK_IMU running, IMU (LSM303D) answering on I2C0");

    /* S3-U02: robot level on the desk -> pitch within +-5 degrees, from
     * real sensor data (INVALID = no data yet). */
    subsystem3_status_read(&out);
    TEST_CHECK("S3-U02", (S3_HUMP_INVALID != out.hump) &&
               (out.pitch_q > -20) && (out.pitch_q < 20),
               "pitch near 0 on a level desk");

    /* S3-U03: status word round trip, 24 bits used. */
    in.pitch_q       = -128;
    in.heading_deg   = 359u;
    in.hump          = S3_HUMP_DESCENDING;
    in.motion        = S3_MOTION_CRUISING;
    in.b_hump_active = true;
    imu_status_unpack(imu_status_pack(&in), &out);
    TEST_CHECK("S3-U03",
               (-128 == out.pitch_q) && (359u == out.heading_deg) &&
               (S3_HUMP_DESCENDING == out.hump) &&
               (S3_MOTION_CRUISING == out.motion) && out.b_hump_active &&
               (0u == (imu_status_pack(&in) & 0xFF000000u)),
               "IMU status word round trip");

    /* S3-U04: heading maths close to the real values. */
    TEST_CHECK("S3-U04",
               (imu_absf((imu_atan2f(1.0f, 1.0f) * IMU_RAD_TO_DEG_F) - 45.0f)
                < 0.2f) &&
               (imu_heading_deg(1.0f, 0.0f) == 0u),
               "atan2 / heading accuracy");

    /* S3-U05: hump record saturates at 127 mm. */
    TEST_CHECK("S3-U05",
               (imu_hump_record_pack(200u, 30u) == (127u | (30u << 7))),
               "hump record 14 bits, saturating");

    /* S3-U06 (his S4-U03): 100 cycles in 1 s, timed once the generator
     * runs (TSK_IMU may first finish a sensor access it had started). */
    imu_task_sim_start(IMU_SIM_FLAT, 0u, 0u, TEST_SPEED_MM_S, 0);
    loops = imu_task_loops();

    for (wait = 0u; (wait < TEST_SIM_WAIT_STEPS) &&
                    ((imu_task_loops() - loops) < 2u); wait++)
    {
        (void)tk_dly_tsk(TEST_WAIT_MS);
    }

    loops = imu_task_loops();
    (void)tk_dly_tsk(1000);
    loops = imu_task_loops() - loops;
    TEST_CHECK("S3-U06", (loops >= 95u) && (loops <= 105u),
               "TSK_IMU runs 100 cycles per second");

    /* S3-U07 (his S4-U04): flat ground, 3 s, no hump, no lift. */
    imu_task_sim_start(IMU_SIM_FLAT, 0u, 0u, TEST_SPEED_MM_S, 0);
    (void)tk_dly_tsk(3000);
    TEST_CHECK("S3-U07",
               (0u == imu_task_hump_events()) &&
               (0u == imu_task_lift_events()) &&
               (0u == (subsystem4_faults() & FAULT_LIFTED)),
               "flat ground: no hump, no lift in 3 s");

    /* S3-U08 (his S4-U05): 30 mm x 400 mm hump: one report, 25..35 mm,
     * hump-active bit seen while on it (TSK_CTRL's obstacle rule). */
    imu_task_sim_start(IMU_SIM_HUMP, 30u, 400u, TEST_SPEED_MM_S, 0);
    b_active = saw_hump_active(3000);
    (void)tk_dly_tsk(3000);
    record = subsystem3_hump_record();
    TEST_CHECK("S3-U08",
               b_active && (1u == imu_task_hump_events()) &&
               ((record & S3_HUMP_MM_MASK) >= 25u) &&
               ((record & S3_HUMP_MM_MASK) <= 35u) &&
               (!subsystem3_hump_active()),
               "30 mm hump: one report within +-15 % (NFR9)");

    /* S3-U12: lifted by hand (200 mg up for 0.3 s at t = 1.5 s, wheels
     * still turning): FAULT_LIFTED, cause LIFTED, once. */
    imu_task_sim_start(IMU_SIM_LIFT, 0u, 0u, TEST_SPEED_MM_S, 0);
    (void)tk_dly_tsk(2500);
    TEST_CHECK("S3-U12",
               (1u == imu_task_lift_events()) &&
               (S3_LIFT_LIFTED == subsystem3_lift_cause()) &&
               (0u != (subsystem4_faults() & FAULT_LIFTED)),
               "lifted -> FAULT_LIFTED within 0.3 s (rubric: stop when "
               "picked up)");

    /* S3-U13 (was U10, his S4-U07): hard braking is not a lift.  The start also
     * releases the lift above (owner rule: S3 clears its own bit). */
    imu_task_sim_start(IMU_SIM_BRAKE, 0u, 0u, TEST_SPEED_MM_S, 0);
    (void)tk_dly_tsk(2000);
    TEST_CHECK("S3-U13",
               (0u == imu_task_lift_events()) &&
               (0u == (subsystem4_faults() & FAULT_LIFTED)),
               "a 60 % braking ramp raises no FAULT_LIFTED");

    /* S3-U11 (his S4-U09): a constant 10 degree wedge is not a hump. */
    imu_task_sim_start(IMU_SIM_TILT, 0u, 0u, TEST_SPEED_MM_S, 1000);
    (void)tk_dly_tsk(4500);     /* Past his 4 s guard: decided, dropped. */
    TEST_CHECK("S3-U11", 0u == imu_task_hump_events(),
               "constant 10 deg slope is rejected, not counted");

    /* S3-U14: rolled over onto its roof (from t = 1.5 s, 1 s roll). */
    imu_task_sim_start(IMU_SIM_FLIP, 0u, 0u, TEST_SPEED_MM_S, 0);
    (void)tk_dly_tsk(3000);
    TEST_CHECK("S3-U14",
               (1u == imu_task_lift_events()) &&
               (S3_LIFT_OVERTURNED == subsystem3_lift_cause()) &&
               (0u != (subsystem4_faults() & FAULT_LIFTED)),
               "overturned -> FAULT_LIFTED, cause OVERTURNED");

    /* S3-U15: the latch holds until a new run re-arms it (TSK_CTRL calls
     * subsystem3_lift_rearm() in start_run); then the bit clears within
     * one 10 ms cycle. */
    (void)tk_dly_tsk(1000);
    b_active = (0u != (subsystem4_faults() & FAULT_LIFTED));
    subsystem3_lift_rearm();
    (void)tk_dly_tsk(50);
    TEST_CHECK("S3-U15",
               b_active && (0u == (subsystem4_faults() & FAULT_LIFTED)) &&
               (S3_LIFT_NONE == subsystem3_lift_cause()),
               "FAULT_LIFTED latched until re-armed, then cleared");

    /* S3-U16: motor vibration and a hard vertical knock every 0.3 s (a
     * hump edge, a seam in the floor): no lift. */
    imu_task_sim_start(IMU_SIM_BUMP, 0u, 0u, TEST_SPEED_MM_S, 0);
    (void)tk_dly_tsk(4000);
    TEST_CHECK("S3-U16",
               (0u == imu_task_lift_events()) &&
               (0u == (subsystem4_faults() & FAULT_LIFTED)),
               "vibration and knocks raise no FAULT_LIFTED");

    /* Back to the real sensor for the rest of the run. */
    imu_task_sim_stop();
    (void)tk_dly_tsk(TEST_WAIT_MS);

    TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */

/*** end of file ***/
