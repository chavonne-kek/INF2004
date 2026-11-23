/** @file subsystem4_test.c
 *
 * @brief On-target tests for subsystem 4.  Built only with: make TEST=1
 *        IDs S4-U01 ... must match testing/subsystem4/TEST_LOG.md.
 */

#include "subsystem4.h"
#include "scan_calc.h"
#include "scan_task.h"
#include "b5_types.h"
#include "b5_profile.h"
#include "../test_report.h"

#if APP_TEST

#define TEST_FAULT_ROUNDS      100u

void
subsystem4_test (void)
{
    subsystem4_profile_t in;
    subsystem4_profile_t out;
    uint8_t              bytes[S4_PROFILE_BYTES];
    uint32_t             round = 0u;
    bool                 b_ok  = true;

    TEST_INIT();

    /* S4-U01: both flags exist; event bits match subsystem 1. */
    TEST_CHECK("S4-U01", subsystem4_flg_events() > 0,
               "flg_events and flg_faults created");

    /* S4-U02: HC-SR04 answers: 3 s of pings (B5 raises the fault after 20
     * misses, about 2 s) and a range was measured. */
    (void)tk_dly_tsk(3000);
    TEST_CHECK("S4-U02", (0u == (subsystem4_faults() & FAULT_NO_ECHO)) &&
               (S4_RANGE_NONE != subsystem4_range_cm()),
               "HC-SR04 echo seen (point it at something < 1 m)");

    /* S4-U03: S4's own fault bit (NO_ECHO) set and cleared 100 times is
     * never lost or stuck.  The cross-core stress test with S2 and S3 is
     * an integration test (TEST_LOG.md). */
    for (round = 0u; round < TEST_FAULT_ROUNDS; round++)
    {
        subsystem4_fault_raise(FAULT_NO_ECHO);
        b_ok = b_ok && (0u != (subsystem4_faults() & FAULT_NO_ECHO));
        subsystem4_fault_clear(FAULT_NO_ECHO);
        b_ok = b_ok && (0u == (subsystem4_faults() & FAULT_NO_ECHO));
    }
    TEST_CHECK("S4-U03", b_ok, "fault bit set/clear 100 times");

    /* S4-U04: profile packs into 35 bits / 5 bytes and back. */
    in.width_cm       = 126u;
    in.closest_cm     = 18u;
    in.closest_index  = 24u;
    in.left_clear_cm  = 127u;
    in.right_clear_cm = 0u;
    in.b_left_edge    = true;
    in.b_right_edge   = false;
    scan_profile_pack(&in, bytes);
    scan_profile_unpack(bytes, &out);
    TEST_CHECK("S4-U04",
               (126u == out.width_cm) && (18u == out.closest_cm) &&
               (24u == out.closest_index) && (127u == out.left_clear_cm) &&
               (0u == out.right_clear_cm) && out.b_left_edge &&
               (!out.b_right_edge) && (0u == (bytes[4] & 0xF8u)),
               "obstacle profile 35 bits round trip");

    /* S4-U05: echo width to range (58 us per cm, 127 = none). */
    TEST_CHECK("S4-U05",
               (20u == scan_echo_to_cm(1160u)) &&
               (127u == scan_echo_to_cm(38000u)),
               "echo width -> cm, out of range = 127");

    /* S4-U06: Buddy 5's engine is watching (robot idle: control refused,
     * so it must be in B5_STATE_IDLE with no B5 faults except NO_ECHO). */
    TEST_CHECK("S4-U06",
               (B5_STATE_IDLE == B5_ST_GET(scan_task_b5_status(), STATE)) &&
               (0u == (B5_ST_GET(scan_task_b5_status(), FAULTS) &
                       (B5_FAULT_BLOCKED | B5_FAULT_BYPASS_ABORT))),
               "Buddy 5 engine idle and watching");

    /* S4-U07: Buddy 5 geometry on target (integer maths, no libm). */
    TEST_CHECK("S4-U07",
               (B5_Geom_LateralCm(0u, 100u) == 100) &&
               (B5_Geom_LateralCm(180u, 100u) == -100) &&
               (B5_Geom_ForwardCm(90u, 100u) == 100) &&
               (B5_Geom_LateralCm(60u, 100u) == 50),
               "B5 lateral/forward geometry");

    TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */

/*** end of file ***/
