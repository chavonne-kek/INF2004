/** @file subsystem2_test.c
 *
 * @brief On-target tests for subsystem 2.  Built only with: make TEST=1
 *        IDs S2-U01 ... must match testing/subsystem2/TEST_LOG.md.
 *
 * These tests never drive the motors, so the car can sit on the desk.
 * Motor and encoder behaviour is tested by hand (TEST_LOG.md, manual).
 *
 * At the end, the calibration helper (S2-M01) prints the encoder counts
 * for 15 s: turn each wheel exactly 10 times by hand while it runs, then
 * set MOTION_EDGES_PER_REV.  Set S2_CAL_HELPER to 0 to skip it.
 */

#include "subsystem2.h"
#include "motion_task.h"
#include "motion_calc.h"
#include "drv_encoder.h"
#include "../test_report.h"

#if APP_TEST

#define TEST_WAIT_MS           100
#define S2_CAL_HELPER          1
#define S2_CAL_SECONDS         15
#define S2_CAL_TURNS           10u
#define S2_CAL_PINS            4u      /* Wiring check: GP0..GP3. */
#define S2_CAL_US_PER_S        1000000u
#define S2_CAL_COARSE_EDGES    150u    /* Below: S3 needs a longer window. */
#define S2_EXEC_LIMIT_US       1000u   /* 10% of the 10 ms period. */

void
subsystem2_test (void)
{
    T_RCYC            rcyc;
    subsystem2_odom_t in;
    subsystem2_odom_t out;
    subsystem2_op_t   op    = S2_OP_STOP;
    uint8_t           arg   = 0u;
    uint32_t          alive = 0u;
    bool              b_stalled = false;

    TEST_INIT();

    /* S2-U01: 10 ms cyclic handler running, TSK_MOTION alive. */
    alive = subsystem2_alive_count();
    (void)tk_dly_tsk(TEST_WAIT_MS);
    TEST_CHECK("S2-U01",
               (E_OK == tk_ref_cyc(motion_task_cyclic_id(), &rcyc)) &&
               (0u != (rcyc.cycstat & TCYC_STA)) &&
               ((subsystem2_alive_count() - alive) >= 8u),
               "TSK_MOTION runs every 10 ms");

    /* S2-U02: STOP is accepted and leaves the car BRAKED. */
    TEST_CHECK("S2-U02",
               (E_OK == subsystem2_command(S2_OP_STOP, 0u)) &&
               (E_OK == tk_dly_tsk(30)) &&
               (S2_STATE_BRAKED == subsystem2_state()),
               "STOP command -> BRAKED");

    /* S2-U09: a stall is cleared by RESET even when a STOP follows in the
     * same cycle (TSK_CTRL's GP21 path; failed on the car 30 Oct). */
    motion_task_test_stall();
    (void)tk_dly_tsk(30);
    b_stalled = (S2_STATE_STALLED == subsystem2_state());
    (void)subsystem2_command(S2_OP_RESET, 0u);
    (void)subsystem2_command(S2_OP_STOP, 0u);
    (void)tk_dly_tsk(30);
    TEST_CHECK("S2-U09", b_stalled && (S2_STATE_BRAKED == subsystem2_state()),
               "RESET leaves STALLED, also with a STOP right after it");

    /* S2-U03: motion command packs into 12 bits and back (4-bit opcode
     * since S2_OP_SET_GAIN).  1 Nov: also 12 and 90, whose low 4 bits the
     * old argument mask cleared (speed 12 cm/s arrived as 0). */
    TEST_CHECK("S2-U03",
               motion_cmd_unpack(motion_cmd_pack(S2_OP_TURN_RIGHT, 180u),
                                 &op, &arg) &&
               (S2_OP_TURN_RIGHT == op) && (180u == arg) &&
               motion_cmd_unpack(motion_cmd_pack(S2_OP_FOLLOW, 12u),
                                 &op, &arg) && (12u == arg) &&
               motion_cmd_unpack(motion_cmd_pack(S2_OP_SET_GAIN, 90u),
                                 &op, &arg) && (S2_OP_SET_GAIN == op) &&
               (90u == arg) &&
               (0u == (motion_cmd_pack(S2_OP_SET_GAIN, 255u) & 0xF000u)),
               "motion command 12 bits round trip");

    /* S2-U04: odometry word round trip, speeds saturate, 29 bits used. */
    in.distance_mm = 32767u;
    in.left_mm_s   = -320;
    in.right_mm_s  = 1000;      /* Out of range: must saturate at +315. */
    motion_odom_unpack(motion_odom_pack(&in), &out);
    TEST_CHECK("S2-U04",
               (32767u == out.distance_mm) && (-320 == out.left_mm_s) &&
               (315 == out.right_mm_s) &&
               (0u == (motion_odom_pack(&in) & 0xE0000000u)),
               "odometry word packs, speeds saturate");

    /* S2-U05: distance difference is correct across the 15-bit wrap. */
    TEST_CHECK("S2-U05",
               (12 == motion_distance_delta(5u, 32761u)) &&
               (-12 == motion_distance_delta(32761u, 5u)),
               "distance delta across the wrap");

    /* S2-U06: PID controller pushes toward the target. */
    {
        motion_pid_t pid;

        motion_pid_init(&pid);
        TEST_CHECK("S2-U06",
                   (motion_pid_step(&pid, 200, 0) > 0) &&
                   (motion_pid_step(&pid, -200, 0) < 0) &&
                   (MOTION_DUTY_MAX >= motion_pid_step(&pid, 10000, 0)),
                   "PID output sign and limit");
    }

    /* S2-U07: braking profile and straight-line trim on the target CPU. */
    TEST_CHECK("S2-U07",
               (motion_isqrt(40000u) == 200u) &&
               (motion_profile_speed(1000u, 200) == 200) &&
               (motion_profile_speed(10u, 200) == 100) &&
               (motion_profile_speed(0u, 200) == MOTION_FINAL_MM_S) &&
               (motion_straight_trim(110u, 100u) > 0) &&
               (motion_straight_trim(100u, 110u) < 0),
               "braking profile and straight-line trim");

    /* S2-U08: worst TSK_MOTION cycle time so far (timed on the 1 MHz
     * timer; 0 means the timer was not running). */
    tm_printf((UB *)"        TSK_MOTION worst cycle %u us\n",
              (UINT)motion_task_exec_max_us());
    TEST_CHECK("S2-U08",
               (motion_task_exec_max_us() > 0u) &&
               (motion_task_exec_max_us() <= S2_EXEC_LIMIT_US),
               "TSK_MOTION cycle under 1 ms (NFR2 budget)");

#if S2_CAL_HELPER
    /* S2-M01 helper: edges per wheel revolution (Week 6 A2). */
    {
        uint16_t left0  = 0u;
        uint16_t right0 = 0u;
        uint16_t left   = 0u;
        uint16_t right  = 0u;
        INT      second = 0;
        uint32_t prev   = in_w(GPIO_IN);
        uint32_t toggles[S2_CAL_PINS] = {0u, 0u, 0u, 0u};
        uint32_t edges0[S2_CAL_PINS];
        uint32_t any_edges = 0u;
        uint16_t per_rev[2];
        uint8_t  pin    = 0u;

        motion_task_coast(true);        /* Free wheels while turning. */
        subsystem2_encoder_counts(&left0, &right0);

        for (pin = 0u; pin < S2_CAL_PINS; pin++)
        {
            edges0[pin] = drv_encoder_pin_edges(pin);
        }

        tm_printf((UB *)"[CAL] S2-M01: turn EACH wheel forward exactly %u "
                  "times by hand now (%d s)\n", S2_CAL_TURNS,
                  S2_CAL_SECONDS);

        for (second = 0; second < S2_CAL_SECONDS; second++)
        {
            /* Wiring check (build log 26 Oct): which of GP0..GP3 really
             * toggle, sampled about every 1 ms (undercounts, but shows
             * activity).  Paced by the 1 MHz timer: tk_dly_tsk(1) can last
             * up to 2 ticks, so counting 1000 delays would take ~2 s. */
            uint32_t start_us = in_w(TIMER_TIMERAWL);

            while ((in_w(TIMER_TIMERAWL) - start_us) < S2_CAL_US_PER_S)
            {
                uint32_t now     = in_w(GPIO_IN);
                uint32_t changed = now ^ prev;

                for (pin = 0u; pin < S2_CAL_PINS; pin++)
                {
                    toggles[pin] += (changed >> pin) & 1u;
                }

                prev = now;
                (void)tk_dly_tsk(1);
            }

            subsystem2_encoder_counts(&left, &right);
            tm_printf((UB *)"[CAL]   %2d s  left %u  right %u edges\n",
                      second + 1, (UINT)(uint16_t)(left - left0),
                      (UINT)(uint16_t)(right - right0));
        }

        tm_printf((UB *)"[CAL] MOTION_EDGES_PER_REV = left %u.%u / right "
                  "%u.%u (edges / %u; use the whole number); now %d in "
                  "motion_calc.h\n",
                  (UINT)((uint16_t)(left - left0) / S2_CAL_TURNS),
                  (UINT)((uint16_t)(left - left0) % S2_CAL_TURNS),
                  (UINT)((uint16_t)(right - right0) / S2_CAL_TURNS),
                  (UINT)((uint16_t)(right - right0) % S2_CAL_TURNS),
                  S2_CAL_TURNS, MOTION_EDGES_PER_REV);
        per_rev[ENC_LEFT]  = (uint16_t)((uint16_t)(left - left0) /
                                        S2_CAL_TURNS);
        per_rev[ENC_RIGHT] = (uint16_t)((uint16_t)(right - right0) /
                                        S2_CAL_TURNS);

        if (((0u != per_rev[ENC_LEFT]) &&
             (per_rev[ENC_LEFT] < S2_CAL_COARSE_EDGES)) ||
            ((0u != per_rev[ENC_RIGHT]) &&
             (per_rev[ENC_RIGHT] < S2_CAL_COARSE_EDGES)))
        {
            tm_printf((UB *)"[CAL] under %u edges per turn: coarse speed "
                      "steps; S3 may see false humps (Zi Siang: "
                      "GLUE_ACC_WINDOW 20 in imu_glue.h)\n",
                      S2_CAL_COARSE_EDGES);
        }

        tm_printf((UB *)"[CAL] pin activity GP0 %u  GP1 %u  GP2 %u  GP3 %u "
                  "(interrupt edges; sampled every 1 ms: %u %u %u %u)\n",
                  (UINT)(drv_encoder_pin_edges(0u) - edges0[0]),
                  (UINT)(drv_encoder_pin_edges(1u) - edges0[1]),
                  (UINT)(drv_encoder_pin_edges(2u) - edges0[2]),
                  (UINT)(drv_encoder_pin_edges(3u) - edges0[3]),
                  (UINT)toggles[0], (UINT)toggles[1], (UINT)toggles[2],
                  (UINT)toggles[3]);
        tm_printf((UB *)"[CAL] left wheel counted on GP%u, right on GP%u "
                  "(GP0/GP1 = left Grove 1, GP2/GP3 = right Grove 2; a wheel "
                  "with no edges on either pin: check its encoder wires)\n",
                  (UINT)drv_encoder_active_gpio(ENC_LEFT),
                  (UINT)drv_encoder_active_gpio(ENC_RIGHT));

        for (pin = 0u; pin < S2_CAL_PINS; pin++)
        {
            any_edges += (drv_encoder_pin_edges(pin) - edges0[pin]) +
                         toggles[pin];
        }

        if (0u == any_edges)
        {
            /* Car test 29 Oct: all zero.  The pins never changed level, so
             * the signal never reached the Pico (the same input set-up
             * reads the HC-SR04 echo on GP5). */
            tm_printf((UB *)"[CAL] NO encoder pulse on GP0..GP3 at all. "
                      "Were both wheels turned during the 15 s?\n");
            tm_printf((UB *)"[CAL]   Turn a wheel slowly and watch the "
                      "small LEDs next to Grove 1 (GP0/GP1) and Grove 2 "
                      "(GP2/GP3): one must blink as the slots pass.\n");
            tm_printf((UB *)"[CAL]   No blink: that encoder has no power "
                      "or is not plugged in (red 3V3, black GND, signal "
                      "wire, Grove plug fully in).\n");
        }

        motion_task_coast(false);
    }
#endif

    TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */

/*** end of file ***/
