/** @file ctrl_task.c
 *
 * @brief TSK_CTRL (priority 18, core 1): the vehicle controller and the
 *        system statechart (Week 6 section 5).  The ONLY task that commands
 *        motion (subsystem2_command).
 *
 * Obstacle avoidance (statechart 6.1) is Buddy 5's engine in TSK_SCAN.
 * It asks for control (AVOID_MSG_TAKE): TSK_CTRL brakes at once (NFR4)
 * and enters OBSTACLE_AVOID, or refuses (on a hump with the obstacle 10 cm
 * or more away, or when the robot is not following the line).  While in
 * OBSTACLE_AVOID, TSK_CTRL runs each motion request from the engine and
 * answers when MOTION_DONE arrives; a fault, pause or e-stop answers "no",
 * so the engine aborts.  DONE_OK resumes line following; DONE_FAIL is the
 * statechart's FAILED -> system FAULT.
 *
 * Loop: tk_wai_flg(flg_events, all bits, TWF_ORW | TWF_BITCLR, 50 ms).
 * It wakes the moment a bit is set; the 50 ms timeout only paces the
 * buttons, timeouts, buzzer and watchdog.
 */

#include "subsystem4.h"
#include "ctrl_task.h"
#include "scan_task.h"
#include "drv_board.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem1/barcode.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem2/motion_calc.h"   /* mm / degrees -> encoder edges */
#include "../subsystem3/subsystem3.h"
#include "../subsystem5/subsystem5.h"

/* Options.  The watchdog (Week 6 NFR19: reset within 500 ms if the
 * control loop stalls) is on; set CTRL_WATCHDOG_ENABLE to 0 only while
 * debugging with breakpoints. */
#if APP_TEST
#define CTRL_CALIBRATE_AT_BOOT 0   /* Test build: the car must not move. */
#else
#define CTRL_CALIBRATE_AT_BOOT 1
#endif
#define CTRL_CAL_WARN_MS       2000u   /* Wait before the start-up sweep.  */
#define CTRL_STARTUP_BEEP      0       /* 30 Oct: start-up beep off (1 = on). */
/* Line-test options (30 Oct, team request):
 * CTRL_AUTO_START 1: start following as soon as the start-up calibration
 *   succeeds, without pressing START (GP20).  GP20 still starts it again
 *   from IDLE after a GP21 reset.
 * CTRL_OBSTACLE_ENABLE 0: ignore the ultrasonic (TSK_CTRL refuses every
 *   obstacle hand-over, so the car never stops or scans for obstacles).
 *   Nov 2026: back to 1 for the demo (detect, measure and bypass the
 *   obstacle are rubric items 6-8).  0 only for line-only testing. */
#define CTRL_AUTO_START        0
#define CTRL_OBSTACLE_ENABLE   1       /* Demo: on (rubric 6-8).     */
#define CTRL_WATCHDOG_ENABLE   1
#define CTRL_WATCHDOG_MS       500u
#define CTRL_PRINT_EVENTS      1

#define CTRL_WAIT_MS           50
#define CTRL_SPEED_DEFAULT_CMS 3u      /* FR16 default speed (2 Nov: 12 -> 7 -> 4 -> 3). */
                                       /* Slower is what makes the barcode      */
                                       /* readable: 3 mm bars get ~25 samples.  */
/* 22 Nov: how long after the last barcode bar the obstacle watch stays
 * deaf, so a box beyond the sheet cannot stop the car mid-read. */
#define CTRL_BARCODE_QUIET_MS  1500u

#define CTRL_SPEED_SLOW_CMS    2u      /* Line recovery / after bypass.    */
#define CTRL_CAL_STILL_MS      1000u   /* Accelerometer zero.              */
/* Start-up sweep (30 Oct, was a full 360 degree spin): left, right twice
 * as far, left again, so the car ends facing the line as it started.  A
 * sweep in ONE direction is not enough: turning left only moves the left
 * sensor further from the line (it would never see black), turning right
 * only the right one.  With the sensors 60 mm ahead of the axle, the side
 * sensors cross the 18 mm line within about 35 degrees each way; 60 leaves
 * margin for a sensor board mounted closer to the axle (about 55 degrees
 * at 30 mm).  The compass then sees 120 degrees of rotation, not a full
 * circle, so the heading is approximate (it is only reported, never used
 * for steering). */
#define CTRL_CAL_SWEEP_DEG     60u
/* 20 Nov (night): junction turn = creep, then PIVOT on the spot until the
 * LEADING track sensor has crossed the new line (crossroads; pivot allowed
 * by the owner).  The lean it replaces only ever turned ~24 deg (1.2 s cap)
 * and gave up as soon as either sensor saw black - at a crossroads that is
 * the RIGHT sensor swinging back over the straight line, so the car went
 * straight on (simulator, both cases). */
#define CTRL_JUNCTION_CREEP_MM 65u     /* After both sensors go black, drive
                                        * this far so the AXLE is on the
                                        * crossing: sensors 60 mm ahead of the
                                        * axle + half the 18 mm bar - 4 mm.
                                        * TUNE ON THE CAR: if the pivot ends
                                        * with the line beside the car on the
                                        * side it came from, raise it; past
                                        * the branch, lower it. */
#define CTRL_JUNCTION_WAIT_MS  6000u   /* Cap on the creep. */
#define CTRL_PIVOT_MIN_DEG     45u     /* Ignore the sensors until then: the
                                        * trailing sensor crosses the straight
                                        * line at ~20-40 deg. */
#define CTRL_PIVOT_MAX_DEG     150u    /* Stop anyway (the follower takes
                                        * over).  The pivot ends on the
                                        * leading sensor long before. */
#define CTRL_UTURN_MIN_DEG     135u    /* Past the side branch (90 deg). */
#define CTRL_UTURN_MAX_DEG     240u
#define CTRL_CROSS_MM          40u     /* "Go straight": past the crossing
                                        * bar, held straight, then follow. */
/* 20 Nov (car): brake after a pivot and wait this long before following.
 * Switching from the pivot straight into following left the reversing
 * wheel spinning backwards; the single-phase encoder reported it as fast
 * FORWARD, so the speed loop drove it further backwards and the car sped
 * up and kept spinning.  Braked, the wheels stop and the speed window
 * (MOTION_SPEED_WINDOW) empties before the loop runs again. */
#define CTRL_TURN_STOP_MS      300u    /* At least this long, and ...    */
#define CTRL_TURN_QUIET_CYCLES 3u      /* ... no encoder edge for 3 x 50 ms
                                        * (wheels really stopped: a car
                                        * that coasts kept a wheel turning
                                        * backwards after 300 ms - simulator
                                        * with a heavy motor). */
#define CTRL_TURN_STOP_MAX_MS  2000u
/* 20 Nov: rejoining the line after an obstacle bypass (straddle layout).
 * The car comes back at ~45 deg; from the first touch of a track sensor it
 * creeps until the axle is on the line - longer than at a junction because
 * it crosses at 45 deg, not 90, and a little long on purpose (stopping
 * short leaves the car beside the line).  Tune with CTRL_JUNCTION_CREEP_MM. */
#define CTRL_REJOIN_CREEP_MM   (CTRL_JUNCTION_CREEP_MM + 25u)
#define CTRL_REJOIN_SEEK_MM    450u    /* No line by then: FAULT (stop).   */
#define CTRL_REJOIN_SEEK_MS    30000u  /* At the slow speed, 2 cm/s.      */
#define CTRL_REJOIN_MIN_DEG    10u
#define CTRL_REJOIN_MAX_DEG    100u
#define CTRL_MOTION_TIMEOUT_MS 8000u   /* Any MOVE/TURN must finish.       */
#define CTRL_RECOVERY_MS       15000u  /* SEARCH_LINE / LINE_RECOVERY
                                        * (18 Nov: 5 -> 15 s: 5 s one way,
                                        * then 10 s the other way, which
                                        * first turns back to the start
                                        * heading and then as far again). */
#define CTRL_RECOVERY_FLIP_MS  5000u   /* 18 Nov: then search the other way. */
#define CTRL_HUMP_NEAR_CM      10u     /* Always stop below this on a hump. */
#define CTRL_AVOID_MOVE_MS     9000u   /* < TSK_SCAN's 10 s wait.          */
#define CTRL_AVOID_SILENT_MS   30000u  /* No request from TSK_SCAN.        */
/* 1 Nov: before a turn at a junction, drive this far so the WHEELS are on
 * the junction, not the IR sensors (they are ahead of the axle, and a turn
 * spins about the axle).  = distance from the IR sensors to the wheel
 * axle, minus about 5 mm (the junction is reported 14 mm into an 18 mm
 * branch line).  MEASURE on the car: sensors 60 mm ahead -> 5 cm. */
#define CTRL_JUNCTION_APPROACH_CM 2u   /* Drive this far before pivoting so
                                        * the AXLE, not the sensors, is on the
                                        * junction.  The sensors sit ahead of
                                        * the axle, so the branch is seen early:
                                        * at 5 cm the car still pivoted short of
                                        * the junction and ended beside the new
                                        * line (20 Nov car test).  Raise it if
                                        * it still pivots early, lower it if it
                                        * overruns the branch. */
#define CTRL_GAIN_ARG_MAX      S4_RCMD_GAIN_MAX  /* S5 rejects above. */
#define CTRL_DEBOUNCE_POLLS    2u
#define CTRL_BEEP_MS           120u
#define CTRL_TONE_OK_HZ        2500u
#define CTRL_TONE_BAD_HZ       2000u

/* Boot jingle: four rising notes on the piezo, 80 ms each with a 40 ms gap.
 * C6-E6-G6-C7 reads as "ready" and is short enough not to delay the
 * calibration sweep.  A piezo can only play tones, not recorded audio. */
#define CTRL_BOOT_NOTES        4u
#define CTRL_BOOT_ON_MS        80u
#define CTRL_BOOT_GAP_MS       40u

/* Sub-steps inside the multi-step states. */
typedef enum
{
    STEP_NONE = 0,
    STEP_CAL_STILL,
    STEP_CAL_SPIN_1,
    STEP_CAL_SPIN_2,
    STEP_CAL_SPIN_3,
    STEP_AVOID_WAIT,        /* Engine scanning / planning.      */
    STEP_AVOID_MOVING,      /* Engine's motion request running. */
    STEP_TURN_APPROACH,     /* Wheels onto the junction.        */
    STEP_TURN_ROTATE,       /* The barcode's turn (or straight). */
    STEP_TURN_CROSS,        /* 20 Nov (night): straight across.  */
    STEP_TURN_STOP,         /* 20 Nov (car): braked after a pivot. */
    STEP_REJOIN_SEEK,       /* 20 Nov: after a bypass, find the line. */
    STEP_RECOVERY_OTHER_WAY /* 18 Nov: searching the other side.  */
} ctrl_step_t;

static ID gh_ctrl_task = 0;

static volatile uint8_t g_state = (uint8_t)SYS_INIT_CALIBRATE;  /* 1 writer */

/* 20 Nov (night): encoder counts when the current turn step began, and
 * the pivot's progress (0: leaving the old line, 1: leading sensor on the
 * new line).  Distances use the wheel that counted MORE edges, as the
 * pivot itself does (motion_task.c): the left encoder under-counts. */
static uint16_t g_turn_left0  = 0u;
static uint16_t g_turn_right0 = 0u;
static uint8_t  g_pivot_phase = 0u;
static char const * g_turn_why = "";
static bool    g_b_rejoin    = false;  /* Turn is a bypass rejoin. */

static uint16_t g_stop_left   = 0u;
static uint16_t g_stop_right  = 0u;
static uint8_t  g_stop_quiet  = 0u;

static void
stop_pivot (char const * p_why)
{
    (void)subsystem2_stop();
    g_turn_why = p_why;
    subsystem2_encoder_counts(&g_stop_left, &g_stop_right);
    g_stop_quiet = 0u;
}

/* True once both wheels have stopped turning (no encoder edge, either
 * direction, for CTRL_TURN_QUIET_CYCLES control cycles). */
static bool
wheels_quiet (void)
{
    uint16_t left  = 0u;
    uint16_t right = 0u;

    subsystem2_encoder_counts(&left, &right);

    if ((left == g_stop_left) && (right == g_stop_right))
    {
        if (g_stop_quiet < 255u)
        {
            g_stop_quiet++;
        }
    }
    else
    {
        g_stop_quiet = 0u;
        g_stop_left  = left;
        g_stop_right = right;
    }

    return (g_stop_quiet >= CTRL_TURN_QUIET_CYCLES);
}

static void
turn_mark (void)
{
    subsystem2_encoder_counts(&g_turn_left0, &g_turn_right0);
}

static uint32_t
turn_edges (void)
{
    uint16_t left  = 0u;
    uint16_t right = 0u;
    uint16_t d_left;
    uint16_t d_right;

    subsystem2_encoder_counts(&left, &right);
    d_left  = (uint16_t)(left - g_turn_left0);      /* Wrap-safe. */
    d_right = (uint16_t)(right - g_turn_right0);

    return ((d_left > d_right) ? (uint32_t)d_left : (uint32_t)d_right);
}

/* Private to the task. */
typedef struct
{
    subsystem4_state_t state;
    subsystem4_state_t paused_from;
    ctrl_step_t        step;
    uint32_t           step_start_ms;
    subsystem1_cmd_t   pending_cmd;     /* From the last barcode.       */
    subsystem1_cmd_t   turn_cmd;        /* Being executed (TURN_EXECUTE). */
    uint8_t            speed_cms;
    bool               b_avoid_pending;  /* Engine waits for an answer. */
    uint32_t           beep_off_ms;
    bool               b_beeping;
    uint8_t            btn_count[2];
    bool               b_btn_down[2];
    uint32_t           last_alive;
    uint32_t           faults_seen;     /* Mask at the last handle_faults. */
} ctrl_t;

static ctrl_t g_c;

static void     ctrl_thread(INT stacd, void * p_exinf);
static uint32_t now_ms(void);
static void     enter(subsystem4_state_t state);
static void     set_step(ctrl_step_t step);
static uint32_t step_age(void);
static void     beep(uint16_t hz);
static void     boot_chime(void);
static void     poll_buttons(bool * p_b_start, bool * p_b_reset);
static void     handle_remote(bool * p_b_start, bool * p_b_reset);
static void     handle_faults(void);
static void     handle_barcodes(void);
static void     run_state(uint32_t events, bool b_start, bool b_reset);
static void     start_following(uint8_t speed_cms);
static void     start_turn_execute(bool b_approach);
static void     start_turn_rotate(void);
static void     run_turn(uint32_t events);
static void     finish_turn(char const * p_why);
static void     start_rejoin(bool b_line_on_left);
static void     handle_avoid_requests(void);
static void     run_avoid(uint32_t events);
static void     answer_avoid(bool b_yes);
static void     feed_watchdog(void);
static void     report_calibration(bool b_ir, bool b_imu);
static bool     ir_calibrated(void);
static void     start_run(void);
#if APP_TEST
static void     test_presses(bool * p_b_start, bool b_reset);

/* Start / reset presses seen (buttons or dashboard), test images only. */
static volatile uint32_t g_presses[2] = {0u, 0u};
#endif


/*!
 * @brief Board I/O (buttons, buzzer).
 */
ER
ctrl_task_init (void)
{
    drv_board_init();
    g_c.state       = SYS_INIT_CALIBRATE;
    g_c.speed_cms   = CTRL_SPEED_DEFAULT_CMS;
    g_c.pending_cmd = SUBSYSTEM1_CMD_NONE;
    g_c.turn_cmd    = SUBSYSTEM1_CMD_NONE;

    return (E_OK);
}

/*!
 * @brief Start TSK_CTRL on core 1.
 */
ER
ctrl_task_start (void)
{
    gh_ctrl_task = app_start_task((FP)ctrl_thread, S4_PRI_CTRL, TP_PRC2,
                                  "ctrl_task");     /* FP: kernel entry. */

    return ((gh_ctrl_task < E_OK) ? (ER)gh_ctrl_task : E_OK);
}

/*!
 * @brief System state (one 8-bit load).
 */
uint8_t
ctrl_task_state (void)
{
    return (g_state);
}

/*!
 * @brief TSK_CTRL body.
 */
static void
ctrl_thread (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

#if CTRL_CALIBRATE_AT_BOOT
    /* INIT_CALIBRATE: warn, then keep still (accelerometer zero), then
     * sweep left 60, right 120, left 60 degrees (IR black/white,
     * magnetometer hard-iron; see CTRL_CAL_SWEEP_DEG). */
    enter(SYS_INIT_CALIBRATE);
#if CTRL_STARTUP_BEEP
    boot_chime();
#endif
    (void)tk_dly_tsk(CTRL_CAL_WARN_MS);
    drv_board_buzzer(0u);
    subsystem1_calibrate_begin();
    subsystem3_calibrate_begin();
    set_step(STEP_CAL_STILL);
#else
    enter(SYS_IDLE);
#endif

#if CTRL_WATCHDOG_ENABLE
    /* Started here, after the 2 s start-up warning: from now on this
     * loop runs at least every 50 ms and feeds it while TSK_MOTION runs. */
    drv_board_watchdog_start(CTRL_WATCHDOG_MS);
#endif

    for (;;)
    {
        UINT pattern  = 0u;
        bool b_start  = false;
        bool b_reset  = false;
        ER   result   = tk_wai_flg(subsystem4_flg_events(), EVT_ALL,
                                   TWF_ORW | TWF_BITCLR, &pattern,
                                   CTRL_WAIT_MS);

        if (E_OK != result)
        {
            pattern = 0u;       /* Timeout: housekeeping only. */
        }

        poll_buttons(&b_start, &b_reset);

        if (0u != (pattern & EVT_REMOTE_CMD))
        {
            handle_remote(&b_start, &b_reset);
        }

#if APP_TEST
        test_presses(&b_start, b_reset);    /* The car never drives. */
#endif

        if (0u != (pattern & EVT_BARCODE))
        {
            handle_barcodes();
        }

        if (0u != (pattern & EVT_FAULT))
        {
            handle_faults();
        }

        handle_avoid_requests();    /* Cheap: drains TSK_SCAN's requests. */

        run_state((uint32_t)pattern, b_start, b_reset);

        if (g_c.b_beeping && ((int32_t)(now_ms() - g_c.beep_off_ms) >= 0))
        {
            drv_board_buzzer(0u);
            g_c.b_beeping = false;
        }

        feed_watchdog();
    }
}

/*!
 * @brief One step of the system statechart.
 */
static void
run_state (uint32_t events, bool b_start, bool b_reset)
{
    subsystem1_line_t line;

    /* 20 Nov: read the line state every cycle.  `line` used to be an
     * uninitialised local that only the recovery state filled in, so the
     * turn step was testing garbage - usually "true" for the arm, which made
     * the car pivot the instant the junction was seen instead of waiting for
     * the arm to reach the branch. */
    (void)subsystem1_line_read(&line);

    /* Reset (button or MQTT) from any stopped or finished state. */
    if (b_reset && ((SYS_FAULT == g_c.state) ||
                    (SYS_EMERGENCY_STOP == g_c.state) ||
                    (SYS_PAUSED == g_c.state) ||
                    (SYS_FINISHED == g_c.state)))
    {
        (void)subsystem2_command(S2_OP_RESET, 0u);
        g_c.pending_cmd = SUBSYSTEM1_CMD_NONE;
        enter(SYS_IDLE);
        return;
    }

    switch (g_c.state)
    {
        case SYS_INIT_CALIBRATE:
            /* 20 Nov: the IR sweep is gone.  It only exists to teach each
             * sensor its black and white levels, but with the sensors now
             * sitting OUTSIDE the line they never see black during the sway,
             * so the span check failed every boot (span 68-113, 400 needed)
             * and the car came up in FAULT with START refused.  Black/white
             * is decided on the raw ADC reading instead, so only the IMU
             * needs its still period. */
            if (step_age() >= CTRL_CAL_STILL_MS)
            {
                bool b_imu = subsystem3_calibrate_end();

                (void)subsystem1_calibrate_end();
                report_calibration(false, b_imu);
                enter(SYS_IDLE);
            }
            else
            {
                /* Let the IMU settle. */
            }
        break;

        case SYS_IDLE:
            /* 20 Nov: the IR calibration can no longer succeed on this car -
             * the line sits BETWEEN the two track sensors at power-on, so the
             * boot sweep never sees black under either of them and the span
             * check fails.  Black/white is decided on the RAW reading now, so
             * a missing calibration is harmless and must not block START
             * (car log 19:39:24: INIT_CALIBRATE -> FAULT -> IDLE, START
             * refused all evening). */
            if (false && b_start && (!ir_calibrated()))
            {
                /* After GP21 from a failed start-up calibration the IR
                 * levels are defaults: the car would circle (30 Oct). */
                beep(CTRL_TONE_BAD_HZ);
#if CTRL_PRINT_EVENTS
                tm_printf((UB *)"S4: START refused - IR not calibrated. Car "
                          "on the line, battery ON, press RUN/RESET\n");
#endif
            }
            else if (b_start)
            {
                start_run();
            }
            else
            {
                /* Waiting for START. */
            }
        break;

        case SYS_LINE_FOLLOW:
            /* Obstacles arrive as TSK_SCAN requests (handle_avoid_requests). */
            if ((0u != (events & EVT_JUNCTION)) &&
                     (SUBSYSTEM1_CMD_NONE != g_c.pending_cmd))
            {
                /* The barcode read while following decides the way. */
                start_turn_execute(true);
            }
            else if ((0u != (events & EVT_LINE_LOST)) &&
                     (SUBSYSTEM1_CMD_NONE != g_c.pending_cmd))
            {
                /* The line ended (dead end, or a corner the junction test
                 * missed) with a command waiting: do it here.  The line
                 * was lost 300 ms ago, so the wheels are already at the
                 * end: no approach. */
                start_turn_execute(false);
            }
            else if (0u != (events & EVT_LINE_LOST))
            {
                start_following(CTRL_SPEED_SLOW_CMS);
                enter(SYS_LINE_RECOVERY);
            }
            else
            {
                /* Keep following; TSK_MOTION steers from the line word. */
            }
        break;

        case SYS_TURN_EXECUTE:
            run_turn(events);
        break;

        case SYS_LINE_RECOVERY:
            (void)subsystem1_line_read(&line);

            if (((0u != (events & EVT_LINE_FOUND)) || line.b_valid) &&
                (!line.b_lost))
            {
                start_following(g_c.speed_cms);
                enter(SYS_LINE_FOLLOW);
            }
            else if (step_age() >= CTRL_RECOVERY_MS)
            {
                enter(SYS_FAULT);
            }
            else if ((STEP_RECOVERY_OTHER_WAY != g_c.step) &&
                     (step_age() >= CTRL_RECOVERY_FLIP_MS))
            {
                /* 18 Nov: 5 s toward the remembered side found nothing.
                 * That side can be stale (car log 17 Nov 22:15: lost from
                 * a well-centred stretch, turned right for 2 s, line not
                 * there).  Before, this was FAULT; now search the other
                 * way for 10 s first (5 s back to where it started, 5 s
                 * past it: simulator, line end, -97 deg then +90 deg).
                 * The first 5 s are unchanged.
                 * g_c.step is set directly so the timer keeps running. */
                subsystem1_line_hint((line.error_mm > 0)
                                     ? (int8_t)(-S1_LINE_ERR_MAX_MM)
                                     : (int8_t)S1_LINE_ERR_MAX_MM);
                g_c.step = STEP_RECOVERY_OTHER_WAY;
#if CTRL_PRINT_EVENTS
                tm_printf((UB *)"S4: line not found that way in %u s: "
                          "searching the other way\n",
                          (UINT)(CTRL_RECOVERY_FLIP_MS / 1000u));
#endif
            }
            else
            {
                /* Searching toward the last error sign (S1 holds it). */
            }
        break;

        case SYS_OBSTACLE_AVOID:
            run_avoid(events);
        break;

        case SYS_PAUSED:
            if (b_start)        /* "resume" arrives as a start. */
            {
                start_following(CTRL_SPEED_SLOW_CMS);
                enter(SYS_LINE_RECOVERY);
            }
        break;

        case SYS_EMERGENCY_STOP:
        case SYS_FAULT:
        case SYS_FINISHED:
        default:
            /* Motors stay braked until reset. */
        break;
    }
}

/*!
 * @brief Barcode: store the navigation command for the next junction
 *        (Week 6 section 5); unknown characters are reported only (FR5).
 */
static void
handle_barcodes (void)
{
    uint8_t msg = 0u;

    while (tk_rcv_mbf(subsystem1_barcode_mbf(), &msg, TMO_POL) > 0)
    {
        subsystem1_cmd_t cmd;

        if (0u != (msg & S1_BC_REJECT_BIT))
        {
            /* A rejected read, not a character: report it and carry on, so
             * the reason reaches the dashboard instead of being dropped. */
            (void)subsystem5_post(S5_TLM_BARCODE, &msg, 1u);
            continue;
        }

        cmd = subsystem1_barcode_cmd(msg);

        (void)subsystem5_post(S5_TLM_BARCODE, &msg, 1u);

        if (SUBSYSTEM1_CMD_END == cmd)
        {
            /* End of the course (Week 6 section 5): stop and report. */
            if ((SYS_LINE_FOLLOW == g_c.state) ||
                (SYS_LINE_RECOVERY == g_c.state))
            {
                g_c.pending_cmd = SUBSYSTEM1_CMD_NONE;
                enter(SYS_FINISHED);
                beep(CTRL_TONE_OK_HZ);
            }
        }
        else if ((SUBSYSTEM1_CMD_NONE != cmd) &&
                 ((SYS_LINE_FOLLOW == g_c.state) ||
                  (SYS_LINE_RECOVERY == g_c.state)))
        {
            g_c.pending_cmd = cmd;
            beep(CTRL_TONE_OK_HZ);
        }
        else if (SUBSYSTEM1_CMD_NONE != cmd)
        {
            /* 21 Nov: read while turning, going round an obstacle, or
             * stopped.  The arm sweeps across the track lines then, so a
             * "letter" there is not a barcode on the course - it must not
             * decide the next junction.  Reported only. */
#if CTRL_PRINT_EVENTS
            tm_printf((UB *)"S4: barcode ignored (not following the line)\n");
#endif
        }
        else
        {
            /* Not a command: reported only. */
        }

#if CTRL_PRINT_EVENTS
        tm_printf((UB *)"S4: barcode '%c' -> command %d\n",
                  subsystem1_barcode_char(msg), (int)cmd);
#endif
    }
}

/*!
 * @brief React to the fault mask (read only: TSK_CTRL never clears it).
 */
static void
handle_faults (void)
{
    uint32_t faults = subsystem4_faults();
    uint32_t rise   = faults & ~g_c.faults_seen;   /* Raised since last. */

#if CTRL_PRINT_EVENTS
    {
        /* Only when the mask differs from the last line printed: a bit
         * raised and cleared again in between (test S4-U03 does it 100
         * times) wakes TSK_CTRL but changes nothing worth a line. */
        static uint32_t printed = 0xFFFFFFFFu;
        char            text[S4_FAULT_TEXT_SIZE];

        if (faults != printed)
        {
            printed = faults;
            tm_printf((UB *)"S4: faults 0x%02x (%s)\n", (UINT)faults,
                      subsystem4_fault_text(faults, text));
        }
    }
#endif

    g_c.faults_seen = faults;

    if ((SYS_INIT_CALIBRATE == g_c.state) &&
        (0u != (faults & (FAULT_STALL | FAULT_ENCODER | FAULT_LIFTED))))
    {
        /* The spin stopped: close the calibrations (the IR one is still
         * applied if every sensor saw enough black and white). */
        bool b_ir  = subsystem1_calibrate_end();
        bool b_imu = subsystem3_calibrate_end();

        report_calibration(b_ir, b_imu);
    }

    if (0u != (rise & FAULT_LIFTED))
    {
        /* Lifted or overturned (lecturer, Nov 2026).  TSK_IMU has already
         * sent STOP in the same 10 ms cycle; this makes it final.  Lifting
         * is how a demo run ENDS, so a run in progress goes to FINISHED
         * (motors braked until GP21 / "reset"), not to a fault.  Edge
         * triggered: the bit stays set until the next run re-arms S3. */
        (void)subsystem2_stop();
        g_c.pending_cmd = SUBSYSTEM1_CMD_NONE;

        if ((SYS_LINE_FOLLOW == g_c.state) ||
            (SYS_LINE_RECOVERY == g_c.state) ||
            (SYS_TURN_EXECUTE == g_c.state) ||
            (SYS_OBSTACLE_AVOID == g_c.state) ||
            (SYS_PAUSED == g_c.state))
        {
#if CTRL_PRINT_EVENTS
            tm_printf((UB *)"S4: robot %s - run over, motors stopped "
                      "(GP21 or 'reset' for the next run)\n",
                      (S3_LIFT_OVERTURNED == subsystem3_lift_cause()) ?
                      "OVERTURNED" : "LIFTED");
#endif
            enter(SYS_FINISHED);
            beep(CTRL_TONE_OK_HZ);
        }
        else if (SYS_INIT_CALIBRATE == g_c.state)
        {
            enter(SYS_FAULT);   /* Sweep interrupted: reset, then start. */
            beep(CTRL_TONE_BAD_HZ);
        }
        else
        {
            /* IDLE (being carried to the start), or already stopped. */
        }
    }

    if (0u != (faults & FAULT_LIFTED))
    {
        /* Nothing else to do while lifted: the motors stay stopped. */
    }
    else if (0u != (faults & (FAULT_STALL | FAULT_ENCODER)))
    {
        /* TSK_MOTION has already braked in the same cycle. */
        enter(SYS_FAULT);
        beep(CTRL_TONE_BAD_HZ);
    }
    else
    {
        /* FAULT_IMU / NO_ECHO / WIFI: degraded but keep driving. */
    }
}

/*!
 * @brief Remote commands from subsystem 5 (already validated there).
 */
static void
handle_remote (bool * p_b_start, bool * p_b_reset)
{
    uint8_t cmd[S4_RCMD_REC_LEN];

    while (tk_rcv_mbf(subsystem4_remote_cmd_mbf(), cmd, TMO_POL) > 0)
    {
        uint8_t op  = cmd[0];
        uint8_t arg = cmd[1];

#if CTRL_PRINT_EVENTS
        {
            static char const * const names[9] =
            {
                "?", "START", "PAUSE", "RESUME", "ESTOP", "RESET", "SPEED",
                "GAIN", "MAP"
            };

            tm_printf((UB *)"S4: dashboard command %s %u\n", names[op],
                      (UINT)arg);
        }
#endif

        switch (op)
        {
            case S4_RCMD_START:
            case S4_RCMD_RESUME:
                *p_b_start = true;
            break;

            case S4_RCMD_PAUSE:
                if ((SYS_IDLE != g_c.state) && (SYS_PAUSED != g_c.state) &&
                    (SYS_FAULT != g_c.state) &&
                    (SYS_EMERGENCY_STOP != g_c.state))
                {
                    (void)subsystem2_stop();
                    g_c.paused_from = g_c.state;
                    enter(SYS_PAUSED);
                }
            break;

            case S4_RCMD_ESTOP:
                (void)subsystem2_stop();
                enter(SYS_EMERGENCY_STOP);
            break;

            case S4_RCMD_RESET:
                *p_b_reset = true;
            break;

            case S4_RCMD_SPEED:
                g_c.speed_cms = arg;
                (void)subsystem2_command(S2_OP_SPEED, arg);
            break;

            case S4_RCMD_GAIN:
                /* arg is the steering gain in 100 mm/s per mm.  S2 takes
                 * it in 10 mm/s per mm in one byte, so above 25 (2500) the
                 * product wrapped (2600 became 40): clamped (1 Nov). */
                if (arg > CTRL_GAIN_ARG_MAX)
                {
                    arg = CTRL_GAIN_ARG_MAX;
                }

                (void)subsystem2_command(S2_OP_SET_GAIN,
                                         (uint8_t)(arg * 10u));
#if CTRL_PRINT_EVENTS
                tm_printf((UB *)"S4: steering gain now %u mm/s per mm\n",
                          (unsigned)(arg * 100u));
#endif
            break;

            case S4_RCMD_MAP:
                /* arg = (movement << 5) | letter index.  Stored in RAM so
                 * the instructor's four characters apply to the next
                 * barcode without a rebuild. */
                {
                    uint8_t nav = (uint8_t)((arg >> S4_RCMD_MAP_SHIFT) & 0x07u);
                    uint8_t idx = (uint8_t)(arg & S4_RCMD_MAP_INDEX_MASK);

                    barcode_map_set(idx, (subsystem1_cmd_t)nav);
#if CTRL_PRINT_EVENTS
                    tm_printf((UB *)"S4: map '%c' -> movement %u\n",
                              (char)('A' + idx), (UINT)nav);
#endif
                }
            break;

            default:
                /* Validated by S5: cannot happen. */
            break;
        }
    }
}

/*!
 * @brief Debounced start/reset buttons (active low): a press counts once,
 *        after two consecutive polls.
 */
static void
poll_buttons (bool * p_b_start, bool * p_b_reset)
{
    uint8_t button = 0u;

    for (button = 0u; button < 2u; button++)
    {
        if (drv_board_button_pressed(button))
        {
            if (g_c.btn_count[button] < CTRL_DEBOUNCE_POLLS)
            {
                g_c.btn_count[button]++;
            }

            if ((CTRL_DEBOUNCE_POLLS == g_c.btn_count[button]) &&
                (!g_c.b_btn_down[button]))
            {
                g_c.b_btn_down[button] = true;

                if (BOARD_BTN_START == button)
                {
                    *p_b_start = true;
                }
                else
                {
                    *p_b_reset = true;
                }
            }
        }
        else
        {
            g_c.btn_count[button]  = 0u;
            g_c.b_btn_down[button] = false;
        }
    }
}

/*!
 * @brief FOLLOW at a speed (set the limit first so it is never exceeded).
 */
static void
start_following (uint8_t speed_cms)
{
    (void)subsystem2_command(S2_OP_SPEED, g_c.speed_cms);
    (void)subsystem2_command(S2_OP_FOLLOW, speed_cms);
}

/*!
 * @brief Execute the stored barcode command at a junction (Project
 *        specification: the barcode read while following decides which
 *        way the car goes).  Left, right and U-turn first drive the wheels
 *        onto the junction, then spin; straight drives across it.
 *
 * @param[in] b_approach True at a junction (drive onto it first), false
 *                       when the line has already ended.
 */
static void
start_turn_execute (bool b_approach)
{
    g_c.turn_cmd    = g_c.pending_cmd;
    g_c.pending_cmd = SUBSYSTEM1_CMD_NONE;
    g_b_rejoin      = false;
    enter(SYS_TURN_EXECUTE);
    turn_mark();

#if CTRL_PRINT_EVENTS
    tm_printf((UB *)"S4: junction - executing barcode command %d\n",
              (int)g_c.turn_cmd);
#endif

    if (SUBSYSTEM1_CMD_STRAIGHT == g_c.turn_cmd)
    {
        /* Straight across: held straight over the crossing bar (both
         * sensors black there), then ordinary following. */
        subsystem1_line_hold(true);
        start_following(CTRL_SPEED_SLOW_CMS);
        set_step(STEP_TURN_CROSS);
    }
    else if (b_approach)
    {
        /* Creep straight on until the AXLE is on the crossing (the sensors
         * are ahead of it), by FOLLOWING at the slow speed with the steering
         * held straight: a MOVE could finish at once on a stale DONE
         * (car log 19:57:40). */
        subsystem1_line_hold(true);
        start_following(CTRL_SPEED_SLOW_CMS);
        set_step(STEP_TURN_APPROACH);
    }
    else
    {
        start_turn_rotate();
    }
}

/*!
 * @brief The turn itself, once the axle is on the crossing: pivot on the
 *        spot (20 Nov night).  The pivot runs for at most
 *        CTRL_PIVOT_MAX_DEG; run_turn() stops it earlier on the leading
 *        track sensor.
 */
static void
start_turn_rotate (void)
{
    subsystem1_line_hold(false);
    subsystem1_line_bias(0);
    turn_mark();
    g_pivot_phase = 0u;

    switch (g_c.turn_cmd)
    {
        case SUBSYSTEM1_CMD_LEFT:
            (void)subsystem2_turn_left(g_b_rejoin ? CTRL_REJOIN_MAX_DEG
                                                  : CTRL_PIVOT_MAX_DEG);
        break;

        case SUBSYSTEM1_CMD_RIGHT:
            (void)subsystem2_turn_right(g_b_rejoin ? CTRL_REJOIN_MAX_DEG
                                                   : CTRL_PIVOT_MAX_DEG);
        break;

        case SUBSYSTEM1_CMD_UTURN:
            (void)subsystem2_turn_left(CTRL_UTURN_MAX_DEG);
        break;

        case SUBSYSTEM1_CMD_STRAIGHT:
        default:
            finish_turn("straight");
            return;
    }

    set_step(STEP_TURN_ROTATE);
}

/*!
 * @brief TURN_EXECUTE, every cycle (20 Nov night).
 *
 * APPROACH: creep until the axle is on the crossing.
 * ROTATE:   pivot.  For the first CTRL_PIVOT_MIN_DEG (UTURN_MIN for a
 *           U-turn) the sensors are ignored - the trailing sensor swings
 *           over the straight line there.  Then wait for the LEADING sensor
 *           (left for a left turn) to go black on the new line, and stop
 *           when it goes white again (the line is now between the two
 *           sensors) or the trailing one goes black (turned too far): the
 *           follower centres the rest.  If S2 reaches the cap first
 *           (MOTION_DONE), hand over anyway.
 * CROSS:    straight across, then follow.
 */
static void
run_turn (uint32_t events)
{
    bool     b_right = (SUBSYSTEM1_CMD_RIGHT == g_c.turn_cmd);
    uint32_t min_deg = g_b_rejoin ? CTRL_REJOIN_MIN_DEG
                     : ((SUBSYSTEM1_CMD_UTURN == g_c.turn_cmd)
                        ? CTRL_UTURN_MIN_DEG : CTRL_PIVOT_MIN_DEG);
    uint32_t creep   = g_b_rejoin ? CTRL_REJOIN_CREEP_MM
                                  : CTRL_JUNCTION_CREEP_MM;
    uint32_t moved   = turn_edges();

    if (STEP_REJOIN_SEEK == g_c.step)
    {
        if (subsystem1_left_black() || subsystem1_right_black())
        {
            turn_mark();                /* Creep from the first touch. */
            set_step(STEP_TURN_APPROACH);
        }
        else if ((moved >= motion_mm_to_edges(CTRL_REJOIN_SEEK_MM)) ||
                 (step_age() >= CTRL_REJOIN_SEEK_MS))
        {
#if CTRL_PRINT_EVENTS
            tm_printf((UB *)"S4: after the obstacle: line not found\n");
#endif
            enter(SYS_FAULT);
        }
        else
        {
            /* Heading back toward the line, held straight. */
        }
        return;
    }

    if (STEP_TURN_APPROACH == g_c.step)
    {
        if ((moved >= motion_mm_to_edges(creep)) ||
            (step_age() >= CTRL_JUNCTION_WAIT_MS))
        {
            start_turn_rotate();
        }
    }
    else if (STEP_TURN_ROTATE == g_c.step)
    {
        bool b_lead  = b_right ? subsystem1_right_black()
                               : subsystem1_left_black();
        bool b_trail = b_right ? subsystem1_left_black()
                               : subsystem1_right_black();
        bool b_past  = (moved >= motion_mm_to_edges(
                                     motion_turn_arc_mm(min_deg)));

        /* 20 Nov (car): brake the moment the LEADING sensor touches the
         * new line.  Waiting for it to cross the line meant braking ~20 deg
         * later, and a car that coasts then carried on past the branch (a
         * heavy motor in the simulator went round to 180 deg).  Braked
         * here, the coast carries the line in between the sensors; if the
         * car stops short, the leading sensor is on the line and the
         * follower steers the rest of the way that same direction. */
        if (!b_past)
        {
            /* Still leaving the old line: sensors ignored. */
        }
        else if (b_lead || b_trail)
        {
            g_pivot_phase = 1u;
            stop_pivot(b_lead ? "on the line" : "on the line (trailing sensor)");
            set_step(STEP_TURN_STOP);
        }
        else if (0u != (events & EVT_MOTION_DONE))
        {
            /* 21 Nov: turned the whole cap and neither sensor saw a line.
             * Following from here would drive off the track: with the
             * sensors outside the line, both white looks like "on the
             * line", so nothing would ever stop it.  Stop instead. */
#if CTRL_PRINT_EVENTS
            tm_printf((UB *)"S4: turned %u deg, no line seen - stopped. "
                      "Check CTRL_JUNCTION_CREEP_MM / MOTION_PIVOT_DUTY\n",
                      (UINT)(g_b_rejoin ? CTRL_REJOIN_MAX_DEG
                             : ((SUBSYSTEM1_CMD_UTURN == g_c.turn_cmd)
                                ? CTRL_UTURN_MAX_DEG : CTRL_PIVOT_MAX_DEG)));
#endif
            enter(SYS_FAULT);
        }
        else
        {
            /* Sweeping toward the new line. */
        }
    }
    else if (STEP_TURN_CROSS == g_c.step)
    {
        if (moved >= motion_mm_to_edges(CTRL_CROSS_MM))
        {
            finish_turn("across");
        }
    }
    else if (STEP_TURN_STOP == g_c.step)
    {
        bool b_quiet = wheels_quiet();

        if ((b_quiet && (step_age() >= CTRL_TURN_STOP_MS)) ||
            (step_age() >= CTRL_TURN_STOP_MAX_MS))
        {
            finish_turn(g_turn_why);
        }
    }
    else
    {
        /* No other step in this state. */
    }

    if ((SYS_TURN_EXECUTE == g_c.state) &&
        (step_age() >= CTRL_MOTION_TIMEOUT_MS))
    {
        enter(SYS_FAULT);
    }
}

/*!
 * @brief After a bypass (20 Nov, straddle layout): head on toward the line
 *        held straight, creep the axle onto it, then pivot back toward
 *        the old heading until the line is between the track sensors - the
 *        junction turn's own steps (run_turn()).
 *
 * @param b_line_on_left  The line is on the car's left (bypassed on the
 *                        right), so the final pivot is to the RIGHT.
 */
static void
start_rejoin (bool b_line_on_left)
{
    g_c.turn_cmd = b_line_on_left ? SUBSYSTEM1_CMD_RIGHT
                                  : SUBSYSTEM1_CMD_LEFT;
    g_b_rejoin   = true;
    enter(SYS_TURN_EXECUTE);
    turn_mark();
    subsystem1_line_hold(true);
    subsystem1_line_bias(0);
    start_following(CTRL_SPEED_SLOW_CMS);
    set_step(STEP_REJOIN_SEEK);

#if CTRL_PRINT_EVENTS
    tm_printf((UB *)"S4: after the obstacle: heading back to the line "
              "(on the %s)\n", b_line_on_left ? "left" : "right");
#endif
}

/*!
 * @brief Turn done: back to ordinary line following (20 Nov night).  Was
 *        a SETTLE step that only a MOTION_DONE could end - following never
 *        sends one, so every barcode turn ended in FAULT 8 s later.
 */
static void
finish_turn (char const * p_why)
{
    g_b_rejoin = false;
    subsystem1_line_hold(false);
    subsystem1_line_bias(0);
    start_following(g_c.speed_cms);
    enter(SYS_LINE_FOLLOW);

#if CTRL_PRINT_EVENTS
    tm_printf((UB *)"S4: turn done (%s) after %u encoder edges\n", p_why,
              (UINT)turn_edges());
#else
    (void)p_why;
#endif
}

/*!
 * @brief Requests from Buddy 5's engine in TSK_SCAN (IF-05).
 */
static void
handle_avoid_requests (void)
{
    uint8_t  type = 0u;
    uint16_t cmd  = 0u;

    while (scan_task_next_request(&type, &cmd))
    {
        bool b_avoiding = (SYS_OBSTACLE_AVOID == g_c.state);

        if (AVOID_MSG_TAKE == type)
        {
            bool b_running = (SYS_LINE_FOLLOW == g_c.state) ||
                             (SYS_LINE_RECOVERY == g_c.state);

            /* 22 Nov (car): halfway along the barcode the sonar already sees
             * the box on the far side and stopped the car mid-read.  While
             * the arm sensor is over the sheet (bars alternate with gaps,
             * so a short memory covers them) the request is refused: the
             * watch simply fires again once the read is done. */
            {
                static uint32_t   s_last_bar_ms = 0u;
                subsystem1_line_t line;
                const uint32_t    t = now_ms();

                if (subsystem1_line_read(&line) && line.b_arm)
                {
                    s_last_bar_ms = t;
                }
                if ((s_last_bar_ms != 0u) &&
                    ((t - s_last_bar_ms) < CTRL_BARCODE_QUIET_MS))
                {
                    b_running = false;
                }
            }
            /* On a hump, ignore readings of 10 cm or more (Week 6). */
            /* 22 Nov car: the IMU reports "hump CLIMBING" on flat track
             * (pitch 5.75-7.5 deg, not 0), so this suppression refused every
             * obstacle request and the car drove into the box. */
            bool b_ignore  = false;

#if !CTRL_OBSTACLE_ENABLE
            b_running = false;      /* Ultrasonic off for line tests. */
#endif

            if (b_running && (!b_ignore))
            {
                (void)subsystem2_stop();  /* NFR4 path. */
                enter(SYS_OBSTACLE_AVOID);
                set_step(STEP_AVOID_WAIT);
                answer_avoid(true);
            }
            else if (b_avoiding && (!g_c.b_avoid_pending))
            {
                /* 17 Nov night: the engine asks again while we already
                 * wait for it - it missed our "yes" (answer late) and went
                 * back to watching.  Before, this answered "no", so TSK_CTRL
                 * waited for an engine that thought it had no control and
                 * went to FAULT 30 s later (car logs: OBSTACLE_AVOID ->
                 * FAULT after exactly 30 s).  Grant it again. */
                set_step(STEP_AVOID_WAIT);
                answer_avoid(true);
            }
            else
            {
                answer_avoid(false);
            }
        }
        else if ((AVOID_MSG_MOTION == type) && b_avoiding)
        {
            subsystem2_op_t op  = (subsystem2_op_t)(cmd & S2_CMD_OP_MASK);
            uint8_t         arg = (uint8_t)((cmd >> S2_CMD_ARG_SHIFT) &
                                            S2_CMD_ARG_MASK);

            if (S2_OP_STOP == op)
            {
                (void)subsystem2_stop();
                answer_avoid(true);
            }
            else if ((S2_OP_MOVE_FWD == op) || (S2_OP_MOVE_BACK == op) ||
                     (S2_OP_TURN_LEFT == op) || (S2_OP_TURN_RIGHT == op))
            {
                (void)subsystem2_command(op, arg);
                g_c.b_avoid_pending = true;     /* Answer on MOTION_DONE. */
                set_step(STEP_AVOID_MOVING);
            }
            else
            {
                answer_avoid(false);            /* Not allowed from B5. */
            }
        }
        else if ((AVOID_MSG_DONE_OK == type) && b_avoiding)
        {
            bool b_line_left = false;

            answer_avoid(true);

            if (scan_task_take_rejoin(&b_line_left))
            {
                start_rejoin(b_line_left);  /* After a bypass (20 Nov). */
            }
            else
            {
                start_following(g_c.speed_cms);   /* False alarm etc. */
                enter(SYS_LINE_FOLLOW);
            }
        }
        else if ((AVOID_MSG_DONE_FAIL == type) && b_avoiding)
        {
            answer_avoid(true);
            enter(SYS_FAULT);       /* Statechart 6.1 FAILED -> FAULT. */
            beep(CTRL_TONE_BAD_HZ);
        }
        else
        {
            answer_avoid(false);    /* Not avoiding (paused, e-stop...). */
        }
    }
}

/*!
 * @brief OBSTACLE_AVOID: answer the engine when its motion finishes.
 */
static void
run_avoid (uint32_t events)
{
    if (!g_c.b_avoid_pending)
    {
        /* Engine scanning or planning.  If it goes silent (task stuck),
         * do not wait for ever. */
        if (step_age() >= CTRL_AVOID_SILENT_MS)
        {
            enter(SYS_FAULT);
        }

        return;
    }

    if (0u != (events & EVT_MOTION_DONE))
    {
        answer_avoid(true);
        set_step(STEP_AVOID_WAIT);
    }
    else if (step_age() >= CTRL_AVOID_MOVE_MS)
    {
        (void)subsystem2_stop();
        answer_avoid(false);
        set_step(STEP_AVOID_WAIT);
    }
    else
    {
        /* Still moving. */
    }
}

/*!
 * @brief Answer the engine (and forget a pending motion).
 */
static void
answer_avoid (bool b_yes)
{
    g_c.b_avoid_pending = false;
    scan_task_answer(b_yes);
}

/*!
 * @brief Change state: brake where needed, publish, report.
 */
static void
enter (subsystem4_state_t state)
{
    uint8_t byte = (uint8_t)state;

    if ((SYS_FAULT == state) || (SYS_EMERGENCY_STOP == state) ||
        (SYS_IDLE == state) || (SYS_PAUSED == state) ||
        (SYS_FINISHED == state))
    {
        (void)subsystem2_stop();
    }

    if ((SYS_OBSTACLE_AVOID != state) && g_c.b_avoid_pending)
    {
        answer_avoid(false);    /* Pause, e-stop or fault mid-manoeuvre. */
    }

    /* Starting a drive drops any barcode read left open by the previous
     * run: an unfinished capture swallowed an entire pass on 19 Nov (30
     * black/white transitions, zero barcode events). */
    if (SYS_LINE_FOLLOW == state)
    {
        subsystem1_barcode_reset();
    }

    g_c.state = state;
    g_state   = byte;           /* One writer, one store. */
    set_step(STEP_NONE);
    (void)subsystem5_post(S5_TLM_STATE, &byte, 1u);

#if CTRL_PRINT_EVENTS
    tm_printf((UB *)"S4: state %s\n", subsystem4_state_name(state));
#endif
}

/*!
 * @brief Start a sub-step and its timer.
 */
static void
set_step (ctrl_step_t step)
{
    g_c.step          = step;
    g_c.step_start_ms = now_ms();
}

/*!
 * @brief Milliseconds since the current sub-step (or state) began.
 */
static uint32_t
step_age (void)
{
    return (now_ms() - g_c.step_start_ms);
}

/*!
 * @brief Milliseconds since start-up (low 32 bits; wraps after 49 days).
 *        tk_get_otm, not tk_get_tim: the system clock counts from the
 *        TRON epoch (1985) and is not zero at start-up.
 */
static uint32_t
now_ms (void)
{
    SYSTIM tim;

    (void)tk_get_otm(&tim);

    return ((uint32_t)tim.lo);
}

/*!
 * @brief Short beep, switched off by the main loop.
 */
static void
boot_chime (void)
{
    static uint16_t const hz[CTRL_BOOT_NOTES] = {1047u, 1319u, 1568u, 2093u};
    uint8_t i = 0u;

    for (i = 0u; i < CTRL_BOOT_NOTES; i++)
    {
        drv_board_buzzer(hz[i]);
        (void)tk_dly_tsk(CTRL_BOOT_ON_MS);
        drv_board_buzzer(0u);
        (void)tk_dly_tsk(CTRL_BOOT_GAP_MS);
    }
}

/*!
 * @brief Start a short beep that switches itself off (see the loop).
 */
static void
beep (uint16_t hz)
{
    drv_board_buzzer(hz);
    g_c.beep_off_ms = now_ms() + CTRL_BEEP_MS;
    g_c.b_beeping   = true;
}

#if APP_TEST
/*!
 * @brief Test and check images: count and beep for start / reset (button
 *        or dashboard), and drop the start so the car never drives by
 *        itself ("does not drive" images).  Reset still clears FAULT.
 */
static void
test_presses (bool * p_b_start, bool b_reset)
{
    if (*p_b_start)
    {
        g_presses[BOARD_BTN_START]++;
        beep(CTRL_TONE_OK_HZ);
        tm_printf((UB *)"S4: START pressed (%u) - GP20 or dashboard; a test "
                  "image does not drive\n", (UINT)g_presses[BOARD_BTN_START]);
        *p_b_start = false;
    }

    if (b_reset)
    {
        g_presses[BOARD_BTN_RESET]++;
        beep(CTRL_TONE_BAD_HZ);
        tm_printf((UB *)"S4: RESET pressed (%u) - GP21 or dashboard\n",
                  (UINT)g_presses[BOARD_BTN_RESET]);
    }
}

/*!
 * @brief Test only: start (BOARD_BTN_START) or reset presses so far.
 */
uint32_t
ctrl_task_presses (uint8_t button)
{
    return (g_presses[(BOARD_BTN_RESET == button) ? BOARD_BTN_RESET
                                                  : BOARD_BTN_START]);
}
#endif

/*!
 * @brief Leave IDLE and drive: follow the line, or, if it is not under
 *        the sensors (e.g. after the start-up sweep), search slowly toward
 *        its last side with the 5 s LINE_RECOVERY limit instead of
 *        following a held error with no timeout (lecturer feedback).
 */
static void
start_run (void)
{
    /* 22 Nov: a new run also resets the obstacle engine's watch state. */
    B5_Task_RequestWatchReset();
    subsystem1_line_hold(false);
    subsystem1_line_t line;

    /* A new run: arm the lift / overturn detector again (it latched at
     * the end of the previous run, or while the car was carried), and
     * forget that FAULT_LIFTED was seen, so the next lift is a new edge
     * even if TSK_IMU clears the bit without a fault event in between. */
    subsystem3_lift_rearm();
    g_c.faults_seen &= ~FAULT_LIFTED;

    (void)subsystem1_line_read(&line);

    if (line.b_lost)
    {
        start_following(CTRL_SPEED_SLOW_CMS);
        enter(SYS_LINE_RECOVERY);
    }
    else
    {
        start_following(g_c.speed_cms);
        enter(SYS_LINE_FOLLOW);
    }
}

/*!
 * @brief True once subsystem 1 has applied an IR calibration.
 */
static bool
ir_calibrated (void)
{
    subsystem1_diag_t diag;

    subsystem1_diag_get(&diag);

    return (diag.b_calibrated);
}

/*!
 * @brief Print the INIT_CALIBRATE result and what to do if it failed.
 */
static void
report_calibration (bool b_ir, bool b_imu)
{
#if CTRL_PRINT_EVENTS
    subsystem1_diag_t diag;

    subsystem1_diag_get(&diag);
    tm_printf((UB *)"S4: calibration IR %s (black-white span arm %u C %u R %u, "
              "400 needed), IMU %s\n", b_ir ? "ok" : "FAILED",
              (UINT)diag.cal_span[2], (UINT)diag.cal_span[1],
              (UINT)diag.cal_span[0],
              b_imu ? "ok" : "not calibrated (heading off)");

    if (!b_ir)
    {
        tm_printf((UB *)"S4:   IR calibration failed: the start-up sweep (left "
                  "60, right 120, left 60 degrees) must carry every sensor "
                  "over the black line and the white floor: start with the "
                  "centre sensor on the line. Fix the cause, then press the "
                  "board's RUN/RESET button\n");
    }
#else
    (void)b_ir;
    (void)b_imu;
#endif
}

/*!
 * @brief Feed the watchdog only if TSK_MOTION has run since the last feed
 *        (NFR19: a stalled control loop must reset the robot).
 */
static void
feed_watchdog (void)
{
    uint32_t alive = subsystem2_alive_count();

    if (alive != g_c.last_alive)
    {
        g_c.last_alive = alive;
        drv_board_watchdog_feed();
    }
}

/*** end of file ***/
