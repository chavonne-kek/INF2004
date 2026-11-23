/** @file motion_task.c
 *
 * @brief TSK_MOTION (priority 14, core 1), woken every 10 ms by a cyclic
 *        handler.  Implements motion statechart 6.5:
 *
 *   BRAKED --FOLLOW(v)--> FOLLOW --STOP--> BRAKED (brake in the same cycle)
 *   BRAKED --MOVE(d)/TURN(a)--> MOVE/TURN --target reached--> STOPPING
 *   STOPPING (motors braked) --both wheels < 10 mm/s--> BRAKED + MOTION_DONE
 *   any moving state --stall--> STALLED (+ FAULT_STALL) --RESET--> BRAKED
 *
 * Each cycle: drain mbf_motion_cmd (STOP wins, otherwise the newest
 * command; SPEED limits are applied as they are read), read the
 * encoders, update odometry, run the state, run the two wheel PID loops,
 * and publish the odometry word in ONE store.
 *
 * MOVE and TURN slow down along a braking profile (sqrt(2 a d)) so they
 * stop on the mark, and keep both wheels' edge counts equal (straight-
 * line correction for MOVE, equal arcs for TURN).  The cycle's execution
 * time is measured on the 1 MHz timer (worst case kept for the tests).
 */

#include "subsystem2.h"
#include "motion_task.h"
#include "motion_calc.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem4/subsystem4.h"

#define MOTION_MBF_MESSAGES        8u
#define MOTION_MBF_MSG_BYTES       8u      /* 2 data bytes + header.        */
#define MOTION_DONE_SPEED_MM_S     10      /* Both wheels below: stopped.   */
#define MOTION_STALL_DUTY          (MOTION_DUTY_MAX * 3 / 10)  /* 30%.      */
#define MOTION_STALL_TICKS         50u     /* 500 ms without an edge.       */
#define MOTION_TIMEOUT_FACTOR      2u      /* MOVE/TURN: 2 x expected time. */
#define MOTION_TIMEOUT_MIN_TICKS   100u    /* At least 1 s.                 */
#define MOTION_TICKS_PER_S         100u
#define MOTION_UM_PER_MM           1000u
#define MOTION_MM_PER_CM           10u
#define MOTION_DIST_WRAP           0x8000u
#define MOTION_PRINT_EVENTS        1
#define MOTION_FOLLOW_MIN_MM_S     0       /* No reverse while following. */
#define MOTION_US_MAX              0xFFFFu

static ID gh_motion_task = 0;
static ID gh_motion_cyc  = 0;
static ID gh_motion_mbf  = 0;

/* Published to other tasks: one writer (this task), one store each. */
static volatile uint32_t g_odom_word  = 0u;
static volatile uint8_t  g_state      = (uint8_t)S2_STATE_BRAKED;
static volatile uint32_t g_alive      = 0u;
static volatile uint16_t g_exec_max_us = 0u;    /* Worst cycle time.    */
#if APP_TEST
static volatile bool     gb_coast      = false;  /* Calibration helper.  */
static volatile int32_t  g_test_duty[2] = {0, 0}; /* Check image: open loop. */
static volatile bool     gb_test_stall  = false;   /* S2-U09.             */
#endif

/* Private to the task. */
typedef struct
{
    motion_pid_t pid[2];
    int32_t  target_mm_s[2];    /* Ramped set-point per wheel.        */
    int32_t  speed_mm_s[2];     /* Filtered signed measured speed.    */
    int32_t  duty[2];
    int8_t   direction[2];      /* Commanded: +1 fwd, -1 back, 0 stop. */
    int8_t   meas_dir[2];       /* Measured (phase B only).           */
    uint16_t last_count[2];
    uint16_t idle_ticks[2];     /* Ticks with duty but no edge.       */
    uint8_t  window[2][MOTION_SPEED_WINDOW];  /* Edges per recent tick.  */
    uint16_t window_sum[2];
    uint8_t  window_pos;
    uint32_t fraction_um;       /* Distance below 1 mm, carried over. */
    uint16_t distance_mm;       /* 15-bit wrapping odometer.          */
    int32_t  cruise_mm_s;       /* FOLLOW/MOVE speed.                 */
    int32_t  move_mm_s;         /* This MOVE/TURN's top speed.        */
    int32_t  speed_limit_mm_s;  /* SPEED command.                     */
    uint32_t goal_edges;        /* MOVE/TURN: edges to travel.        */
    uint32_t done_edges[2];     /* MOVE/TURN: edges so far.           */
    uint32_t ticks_in_move;
    uint32_t timeout_ticks;
    bool     b_report_done;     /* Set MOTION_DONE when stopped.      */
    subsystem2_state_t state;
} motion_t;

static motion_t g_m;

static void   tick_isr(void * p_exinf);
static void   motion_thread(INT stacd, void * p_exinf);
static bool   take_command(subsystem2_op_t * p_op, uint8_t * p_arg);
static void   apply_command(subsystem2_op_t op, uint8_t arg);

/* Ramp state per wheel.  File scope (it used to be a static local) so that a
 * new move can clear it: 20 Nov, the ramp carried +30 mm/s of forward speed
 * into a pivot whose direction had ALREADY been reversed, so the PI chased a
 * positive target on a wheel that reported a negative speed and drove it
 * forward at full duty - the car accelerated LEFT instead of pivoting right
 * (car telemetry: no -60 at all, a burst of +duty then a spin). */
static int32_t g_ramped[2] = {0, 0};
static void   start_move(int8_t dir_left, int8_t dir_right, uint32_t mm,
                         int32_t speed_mm_s);
static void   read_encoders(uint32_t edges[2]);
static void   run_state(uint32_t const edges[2]);
static void   run_wheels(void);
static void   check_stall(uint32_t const edges[2]);
static void   report_stall(bool b_one_moving, bool const b_stuck[2],
                           int32_t duty_left, int32_t duty_right);
static int32_t duty_percent(int32_t duty);
static void   brake_now(void);
static void   brake_hold(void);
static uint32_t now_us(void);
static void   enter(subsystem2_state_t state);
static bool   is_slow(int32_t speed_mm_s);
static void   publish(void);


/*!
 * @brief Create the 10 ms cyclic handler and mbf_motion_cmd; set up the
 *        motor and encoder hardware (motors braked).
 */
ER
motion_task_init (void)
{
    T_CCYC ccyc   = {0};
    T_CMBF cmbf   = {0};
    ER     result = E_OK;
    uint8_t wheel = 0u;

    for (wheel = 0u; wheel < 2u; wheel++)
    {
        motion_pid_init(&g_m.pid[wheel]);
    }

    g_m.cruise_mm_s      = MOTION_SPEED_DEFAULT_MM_S;
    g_m.speed_limit_mm_s = MOTION_SPEED_DEFAULT_MM_S;
    g_m.state            = S2_STATE_BRAKED;

    drv_motor_init();
    result = drv_encoder_init();

    ccyc.cycatr = TA_HLNG;
    ccyc.cychdr = (FP)tick_isr;     /* Kernel handler type for C code. */
    ccyc.cyctim = S2_PERIOD_MS;
    ccyc.cycphs = S2_PERIOD_MS;
    gh_motion_cyc = tk_cre_cyc(&ccyc);

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = (SZ)(MOTION_MBF_MESSAGES * MOTION_MBF_MSG_BYTES);
    cmbf.maxmsz = (INT)S2_CMD_BYTES;
    gh_motion_mbf = tk_cre_mbf(&cmbf);

    if (E_OK != result)
    {
        /* Keep the encoder error. */
    }
    else if (gh_motion_cyc < E_OK)
    {
        result = (ER)gh_motion_cyc;
    }
    else if (gh_motion_mbf < E_OK)
    {
        result = (ER)gh_motion_mbf;
    }
    else
    {
        result = E_OK;
    }

    return (result);
}

/*!
 * @brief Start TSK_MOTION on core 1, then its 10 ms tick.
 */
ER
motion_task_start (void)
{
    ER result = E_OK;

    gh_motion_task = app_start_task((FP)motion_thread, S2_PRI_MOTION,
                                    TP_PRC2, "motion_task");  /* FP: entry. */

    if (gh_motion_task < E_OK)
    {
        result = (ER)gh_motion_task;
    }
    else
    {
        result = tk_sta_cyc(gh_motion_cyc);
    }

    return (result);
}

/*!
 * @brief Queue a motion command without waiting (IF-04).
 */
ER
motion_task_command (subsystem2_op_t op, uint8_t arg)
{
    uint16_t raw = motion_cmd_pack(op, arg);
    uint8_t  bytes[S2_CMD_BYTES];

    bytes[0] = (uint8_t)(raw & 0xFFu);      /* Little-endian. */
    bytes[1] = (uint8_t)(raw >> 8u);

    return (tk_snd_mbf(gh_motion_mbf, bytes, (INT)S2_CMD_BYTES, TMO_POL));
}

uint32_t
motion_task_odometry_word (void)
{
    return (g_odom_word);
}

uint8_t
motion_task_state (void)
{
    return (g_state);
}

uint32_t
motion_task_alive (void)
{
    return (g_alive);
}

ID
motion_task_id (void)
{
    return (gh_motion_task);
}

ID
motion_task_cyclic_id (void)
{
    return (gh_motion_cyc);
}

/*!
 * @brief Worst TSK_MOTION cycle time so far, microseconds (NFR2 budget:
 *        well under 10 ms).  0 if the 1 MHz timer is not running.
 */
uint16_t
motion_task_exec_max_us (void)
{
    return (g_exec_max_us);
}

#if APP_TEST
/*!
 * @brief Test only: while BRAKED, let the wheels coast so they can be
 *        turned by hand (S2-M01 calibration helper).
 */
void
motion_task_coast (bool b_coast)
{
    gb_coast = b_coast;
}

/*!
 * @brief Test only (S2-U09): put TSK_MOTION into STALLED, as a stall
 *        would, but without raising a fault bit (TSK_CTRL would react).
 */
void
motion_task_test_stall (void)
{
    gb_test_stall = true;
}

/*!
 * @brief Test only (S2 check image, wheels lifted): while BRAKED, drive
 *        the motors open loop at these duties in percent (+ = forward,
 *        after MOTOR_INVERT_x).  No PID, no stall check.  0, 0 = brake.
 */
void
motion_task_test_drive (int8_t left_percent, int8_t right_percent)
{
    g_test_duty[ENC_LEFT]  = ((int32_t)left_percent * MOTION_DUTY_MAX) / 100;
    g_test_duty[ENC_RIGHT] = ((int32_t)right_percent * MOTION_DUTY_MAX) /
                             100;
}
#endif


/*!
 * @brief Cyclic handler: only wakes the task.
 */
static void
tick_isr (void * p_exinf)
{
    (void)p_exinf;
    (void)tk_wup_tsk(gh_motion_task);
}

/*!
 * @brief TSK_MOTION body.
 */
static void
motion_thread (INT stacd, void * p_exinf)
{
    ER result = E_OK;

    (void)stacd;
    (void)p_exinf;

    /* Enable the shared GPIO interrupt from here, so it runs on core 1. */
    result = drv_encoder_enable_irq();

    if (E_OK != result)
    {
        tm_printf((UB *)"S2: GPIO interrupt not enabled (%d)\n", result);
        subsystem4_fault_raise(FAULT_ENCODER);
    }

    g_m.last_count[ENC_LEFT]  = drv_encoder_count(ENC_LEFT);
    g_m.last_count[ENC_RIGHT] = drv_encoder_count(ENC_RIGHT);

    for (;;)
    {
        subsystem2_op_t op    = S2_OP_STOP;
        uint8_t         arg   = 0u;
        uint32_t        edges[2];
        uint32_t        start = 0u;
        uint32_t        spent = 0u;

        (void)tk_slp_tsk(TMO_FEVR);
        (void)tk_can_wup(TSK_SELF);     /* Drop late wake-ups. */
        start = now_us();

#if APP_TEST
        if (gb_test_stall)
        {
            gb_test_stall = false;
            brake_now();
            enter(S2_STATE_STALLED);    /* Done in this task: one writer. */
        }
#endif

        if (take_command(&op, &arg))
        {
            apply_command(op, arg);
        }

        read_encoders(edges);
        run_state(edges);
        run_wheels();
        check_stall(edges);
        publish();
        g_alive++;

        spent = now_us() - start;       /* Unsigned: correct across wrap. */

        if (spent > g_exec_max_us)
        {
            g_exec_max_us = (uint16_t)((spent > MOTION_US_MAX) ? MOTION_US_MAX
                                                               : spent);
        }
    }
}

/*!
 * @brief Drain mbf_motion_cmd.  STOP always wins; otherwise the newest.
 *        SPEED and RESET are applied at once, in order, and never
 *        replaced: TSK_CTRL's GP21 reset sends RESET and then (entering
 *        IDLE) STOP in the same cycle, and before 30 Oct the STOP
 *        replaced the RESET, so STALLED and FAULT_ENCODER/STALL stayed
 *        and every later FOLLOW was ignored (car test 30 Oct).
 *
 * @return True if a command was taken.
 */
static bool
take_command (subsystem2_op_t * p_op, uint8_t * p_arg)
{
    uint8_t bytes[S2_CMD_BYTES];
    bool    b_got  = false;
    bool    b_stop = false;

    while (tk_rcv_mbf(gh_motion_mbf, bytes, TMO_POL) == (INT)S2_CMD_BYTES)
    {
        subsystem2_op_t op  = S2_OP_STOP;
        uint8_t         arg = 0u;
        uint16_t        raw = (uint16_t)(bytes[0] | ((uint16_t)bytes[1] << 8u));

        if (!motion_cmd_unpack(raw, &op, &arg))
        {
            /* Corrupt message: ignore. */
        }
        else if ((S2_OP_SPEED == op) || (S2_OP_RESET == op))
        {
            apply_command(op, arg);     /* Never "replaced" by a newer one. */
        }
        else if (!b_stop)
        {
            *p_op  = op;
            *p_arg = arg;
            b_got  = true;
            b_stop = (S2_OP_STOP == op);
        }
        else
        {
            /* A STOP is already waiting: it wins. */
        }
    }

    return (b_got);
}

/*!
 * @brief Apply one command (statechart 6.5 "commands mid-move").
 */
static void
apply_command (subsystem2_op_t op, uint8_t arg)
{
    int32_t  speed = (int32_t)arg * (int32_t)MOTION_MM_PER_CM;
    uint32_t mm    = (uint32_t)arg * MOTION_MM_PER_CM;

    if ((S2_STATE_STALLED == g_m.state) && (S2_OP_RESET != op) &&
        (S2_OP_STOP != op))
    {
        return;     /* Only RESET leaves STALLED. */
    }

    switch (op)
    {
        case S2_OP_STOP:
            brake_now();    /* Same cycle, no ramp (NFR4 budget). */
        break;

        case S2_OP_FOLLOW:
            g_m.cruise_mm_s = motion_clamp(speed, MOTION_SPEED_MIN_MM_S,
                                           g_m.speed_limit_mm_s);
            g_m.direction[ENC_LEFT]  = 1;
            g_m.direction[ENC_RIGHT] = 1;
            g_m.b_report_done        = false;
            enter(S2_STATE_FOLLOW);
        break;

        case S2_OP_MOVE_FWD:
            start_move(1, 1, mm, g_m.cruise_mm_s);
            enter(S2_STATE_MOVE);
        break;

        case S2_OP_MOVE_BACK:
            start_move(-1, -1, mm, g_m.cruise_mm_s);
            enter(S2_STATE_MOVE);
        break;

        case S2_OP_TURN_LEFT:
            /* (left wheel back, right wheel forward) - anticlockwise. */
            start_move(-1, 1, motion_turn_arc_mm(arg), MOTION_TURN_SPEED_MM_S);
            /* 20 Nov (night): without this the pivot never ran - the state
             * stayed FOLLOW/BRAKED, so a TURN command did nothing (the open
             * loop pivot in run_wheels() is only reached in TURN). */
            enter(S2_STATE_TURN);
        break;

        case S2_OP_TURN_RIGHT:
            /* (left wheel forward, right wheel back) - clockwise.  The
             * "turns left" reports were this pairing OVER-ROTATING to ~170
             * deg because the turn ended on the average of the two encoders
             * and the left one undercounts; see done_edges below. */
            start_move(1, -1, motion_turn_arc_mm(arg), MOTION_TURN_SPEED_MM_S);
            enter(S2_STATE_TURN);   /* 20 Nov (night): see TURN_LEFT. */
        break;

        case S2_OP_SPEED:
            g_m.speed_limit_mm_s = motion_clamp(speed, MOTION_SPEED_MIN_MM_S,
                                                MOTION_SPEED_MAX_MM_S);
            /* Adopt the new speed while FOLLOW/MOVE is running.  The old
             * code only clamped cruise DOWN, so raising the speed on the
             * dashboard did nothing at all (18 Nov car test: every run
             * held 30 mm/s whatever number was set). */
            g_m.cruise_mm_s = g_m.speed_limit_mm_s;
        break;

        case S2_OP_RESET:
            brake_now();                /* Keeps STALLED by design...  */
            enter(S2_STATE_BRAKED);     /* ...so RESET leaves it here. */
            subsystem4_fault_clear(FAULT_STALL | FAULT_ENCODER);
        break;

        case S2_OP_SET_GAIN:
            /* arg is the gain in 10 mm/s per mm (0..255 = 0..2550). */
            motion_set_steer_milli((int32_t)arg * 10);
        break;

        default:
            /* 3-bit opcode: all values handled above. */
        break;
    }
}

/*!
 * @brief Set up a MOVE or TURN.  A new one replaces the old target from
 *        the current position; the old one never reports MOTION_DONE.
 */
static void
start_move (int8_t dir_left, int8_t dir_right, uint32_t mm,
            int32_t speed_mm_s)
{
    uint32_t expected_ticks = 0u;

    g_m.direction[ENC_LEFT]  = dir_left;
    g_m.direction[ENC_RIGHT] = dir_right;
    g_ramped[ENC_LEFT]       = 0;
    g_ramped[ENC_RIGHT]      = 0;
    /* A pivot reverses a wheel: the integral wound up while it drove forward
     * would fight the new direction. */
    motion_pid_reset(&g_m.pid[ENC_LEFT]);
    motion_pid_reset(&g_m.pid[ENC_RIGHT]);
    g_m.goal_edges           = motion_mm_to_edges(mm);
    g_m.done_edges[ENC_LEFT]  = 0u;
    g_m.done_edges[ENC_RIGHT] = 0u;
    g_m.move_mm_s            = motion_clamp(speed_mm_s, MOTION_SPEED_MIN_MM_S,
                                            g_m.speed_limit_mm_s);
    g_m.ticks_in_move        = 0u;
    g_m.b_report_done        = true;

    expected_ticks = (mm * MOTION_TICKS_PER_S) / (uint32_t)g_m.move_mm_s;
    g_m.timeout_ticks = (expected_ticks * MOTION_TIMEOUT_FACTOR) +
                        MOTION_TIMEOUT_MIN_TICKS;
}

/*!
 * @brief Edges since the last cycle per wheel; update speed and the
 *        15-bit odometer.
 */
static void
read_encoders (uint32_t edges[2])
{
    uint8_t  wheel  = 0u;
    uint32_t avg_um = 0u;

    for (wheel = 0u; wheel < 2u; wheel++)
    {
        uint16_t now = drv_encoder_count(wheel);
        int32_t  raw_mm_s = 0;

        /* 16-bit difference: correct across the wrap. */
#if ENC_USE_PHASE_B
        {
            int16_t delta = (int16_t)(uint16_t)(now - g_m.last_count[wheel]);

            /* Quadrature: the measured direction replaces the commanded
             * one, so braking, coasting and reversing count right. */
            if (0 != delta)
            {
                g_m.meas_dir[wheel] = (delta > 0) ? 1 : -1;
            }

            edges[wheel] = (uint32_t)((delta < 0) ? -(int32_t)delta : delta);
        }
#else
        edges[wheel] = (uint16_t)(now - g_m.last_count[wheel]);
#endif
        g_m.last_count[wheel] = now;

        /* Sliding-window speed (sign from the commanded direction:
         * phase A alone cannot tell the direction). */
        g_m.window_sum[wheel] = (uint16_t)(g_m.window_sum[wheel] -
                                           g_m.window[wheel][g_m.window_pos]);
        g_m.window[wheel][g_m.window_pos] =
            (uint8_t)((edges[wheel] > 255u) ? 255u : edges[wheel]);
        g_m.window_sum[wheel] = (uint16_t)(g_m.window_sum[wheel] +
                                           g_m.window[wheel][g_m.window_pos]);

        raw_mm_s = motion_window_speed(g_m.window_sum[wheel],
                                       MOTION_SPEED_WINDOW);
#if ENC_USE_PHASE_B
        g_m.speed_mm_s[wheel] = (g_m.meas_dir[wheel] < 0) ? -raw_mm_s
                                                          : raw_mm_s;
#else
        g_m.speed_mm_s[wheel] = (g_m.direction[wheel] < 0) ? -raw_mm_s
                                                           : raw_mm_s;
#endif

        avg_um += motion_edges_to_um(edges[wheel]);
    }

    g_m.window_pos = (uint8_t)((g_m.window_pos + 1u) % MOTION_SPEED_WINDOW);

    /* Odometer: average path of both wheels, carried in micrometres. */
    g_m.fraction_um += avg_um / 2u;
    g_m.distance_mm  = (uint16_t)((g_m.distance_mm +
                                   (g_m.fraction_um / MOTION_UM_PER_MM)) &
                                  (MOTION_DIST_WRAP - 1u));
    g_m.fraction_um %= MOTION_UM_PER_MM;
}

/*!
 * @brief One step of statechart 6.5: set each wheel's target speed.
 */
static void
run_state (uint32_t const edges[2])
{
    subsystem1_line_t line;
    int32_t           steer = 0;

    switch (g_m.state)
    {
        case S2_STATE_FOLLOW:
            (void)subsystem1_line_read(&line);
            steer = motion_steer_mm_s(line.error_mm);
            /* Never below 0 while following: with phase A only, the
             * measured speed has the commanded sign (+), so a negative
             * target could never be reached and the PI loop drove that
             * wheel backwards at full power (car span, simulation 30 Oct).
             * A hard correction now pivots on the slow wheel instead. */
            g_m.target_mm_s[ENC_LEFT]  = motion_clamp(g_m.cruise_mm_s - steer,
                                                      MOTION_FOLLOW_MIN_MM_S,
                                                      MOTION_SPEED_MAX_MM_S);
            g_m.target_mm_s[ENC_RIGHT] = motion_clamp(g_m.cruise_mm_s + steer,
                                                      MOTION_FOLLOW_MIN_MM_S,
                                                      MOTION_SPEED_MAX_MM_S);
        break;

        case S2_STATE_MOVE:
        case S2_STATE_TURN:
        {
            uint32_t done      = 0u;
            uint32_t remaining = 0u;
            int32_t  speed     = 0;
            int32_t  trim      = 0;

            g_m.ticks_in_move++;
            g_m.done_edges[ENC_LEFT]  += edges[ENC_LEFT];
            g_m.done_edges[ENC_RIGHT] += edges[ENC_RIGHT];

            /* 20 Nov: the average was the bug that spun the car round.  A
             * 90 deg pivot needs 318 edges per wheel; with a weak left
             * encoder reporting ~6% of the real travel the average reached
             * 318 only after the car had physically turned ~170 deg, which
             * is why a mapped right turn looked like a left turn.  Ending on
             * the leading wheel stops the turn on the wheel that is honest. */
            done = (g_m.done_edges[ENC_LEFT] > g_m.done_edges[ENC_RIGHT])
                 ? g_m.done_edges[ENC_LEFT]
                 : g_m.done_edges[ENC_RIGHT];
            remaining = (done < g_m.goal_edges) ? (g_m.goal_edges - done) : 0u;

            /* Braking profile: arrive slowly, stop on the mark (NFR3). */
            speed = motion_profile_speed(motion_edges_to_um(remaining) /
                                         MOTION_UM_PER_MM, g_m.move_mm_s);

            /* Keep both wheels' paths equal: straight line for MOVE,
             * equal and opposite arcs for TURN.  A pivot gets NO trim:
             * trim is a straight-line correction of +-40 mm/s, and feeding
             * it into a 60 mm/s turn pinned one wheel at 0 and the other at
             * 100, so "turn left" became a full-speed lurch (19 Nov car). */
            if (S2_STATE_TURN == g_m.state)
            {
                trim = 0;
            }
            else
            {
                trim = motion_straight_trim(g_m.done_edges[ENC_LEFT],
                                            g_m.done_edges[ENC_RIGHT]);
            }
            g_m.target_mm_s[ENC_LEFT]  = g_m.direction[ENC_LEFT] *
                                         motion_clamp(speed - trim, 0,
                                                      MOTION_SPEED_MAX_MM_S);
            g_m.target_mm_s[ENC_RIGHT] = g_m.direction[ENC_RIGHT] *
                                         motion_clamp(speed + trim, 0,
                                                      MOTION_SPEED_MAX_MM_S);

            /* 20 Nov: the first 40 ms of a pivot are open loop.  Reversing a
             * wheel out of a forward roll left the speed window full of
             * forward edges, so the loop read a huge bogus speed and drove
             * the wheel at 320 mm/s instead of 60 (car telemetry L +30 /
             * R -320).  Holding both targets at 0 for four cycles lets the
             * window flush without stopping the car first. */
            if ((S2_STATE_TURN == g_m.state) && (g_m.ticks_in_move < 4u))
            {
                g_m.target_mm_s[ENC_LEFT]  = 0;
                g_m.target_mm_s[ENC_RIGHT] = 0;
            }

            if (0u == remaining)
            {
                brake_hold();               /* Stop on the mark. */
                enter(S2_STATE_STOPPING);
            }
            else if (g_m.ticks_in_move > g_m.timeout_ticks)
            {
#if MOTION_PRINT_EVENTS
                tm_printf((UB *)"S2: STALL - MOVE/TURN not finished after "
                          "%u ms (%u of %u encoder edges)\n",
                          (UINT)(g_m.ticks_in_move * S2_PERIOD_MS),
                          (UINT)done, (UINT)g_m.goal_edges);
#endif
                brake_now();
                enter(S2_STATE_STALLED);
                subsystem4_fault_raise(FAULT_STALL);
            }
            else
            {
                /* Still moving. */
            }
        }
        break;

        case S2_STATE_STOPPING:
            g_m.target_mm_s[ENC_LEFT]  = 0;
            g_m.target_mm_s[ENC_RIGHT] = 0;

            if (is_slow(g_m.speed_mm_s[ENC_LEFT]) &&
                is_slow(g_m.speed_mm_s[ENC_RIGHT]))
            {
                bool b_report = g_m.b_report_done;

                brake_now();

                if (b_report)
                {
                    subsystem4_event_set(EVT_MOTION_DONE);
                }
            }
        break;

        case S2_STATE_BRAKED:
        case S2_STATE_STALLED:
        default:
            g_m.target_mm_s[ENC_LEFT]  = 0;
            g_m.target_mm_s[ENC_RIGHT] = 0;
        break;
    }
}

/*!
 * @brief Ramp each wheel toward its target and run its PI loop.
 */
static void
run_wheels (void)
{
    uint8_t wheel = 0u;

    if ((S2_STATE_BRAKED == g_m.state) || (S2_STATE_STALLED == g_m.state) ||
        (S2_STATE_STOPPING == g_m.state))
    {
        g_ramped[ENC_LEFT]  = 0;
        g_ramped[ENC_RIGHT] = 0;
#if APP_TEST
        if ((S2_STATE_BRAKED == g_m.state) &&
            ((0 != g_test_duty[ENC_LEFT]) || (0 != g_test_duty[ENC_RIGHT])))
        {
            drv_motor_set(g_test_duty[ENC_LEFT], g_test_duty[ENC_RIGHT]);
            return;     /* g_m.duty stays 0: no stall check. */
        }

        if (gb_coast && (S2_STATE_BRAKED == g_m.state))
        {
            drv_motor_coast();
            return;
        }
#endif
        drv_motor_brake();
        return;
    }

    if (S2_STATE_TURN == g_m.state)
    {
        /* 20 Nov: drive the pivot OPEN LOOP.  Closing the speed loop through
         * a direction reversal does not settle on this car - the measured
         * speed lags the reversal, the integral winds up and the wheel pins
         * at full duty (car telemetry: target -60, actual -320, every run),
         * so the car whipped round instead of turning.  A fixed duty has no
         * loop to wind up; the encoder still counts the turn and stops it on
         * the mark. */
        g_m.duty[ENC_LEFT]  = (int32_t)g_m.direction[ENC_LEFT] *
                              (int32_t)MOTION_PIVOT_DUTY;
        g_m.duty[ENC_RIGHT] = (int32_t)g_m.direction[ENC_RIGHT] *
                              (int32_t)MOTION_PIVOT_DUTY;
        /* 20 Nov (night): this branch returned without writing the duty to
         * the motors, so a pivot kept whatever the wheels were doing before
         * (simulator: both wheels went on forward at the follow speed, 0 deg
         * of turn). */
        drv_motor_set(g_m.duty[ENC_LEFT], g_m.duty[ENC_RIGHT]);

        return;
    }

    for (wheel = 0u; wheel < 2u; wheel++)
    {
        /* 20 Nov: no ramp.  The owner asked for it removed, and the pivot
         * no longer uses the loop at all, so its only job was to smear the
         * straight-line targets. */
        g_ramped[wheel] = g_m.target_mm_s[wheel];
        g_m.duty[wheel] = motion_pid_step(&g_m.pid[wheel],
                                          g_ramped[wheel],
                                          g_m.speed_mm_s[wheel]);
    }

    drv_motor_set(g_m.duty[ENC_LEFT], g_m.duty[ENC_RIGHT]);
}

/*!
 * @brief Stall / encoder fault: duty >= 30% and no edge for 500 ms.
 *        If the other wheel moves, the silent one is an encoder fault.
 */
static void
check_stall (uint32_t const edges[2])
{
    uint8_t wheel   = 0u;
    bool    b_stuck[2] = {false, false};

    for (wheel = 0u; wheel < 2u; wheel++)
    {
        int32_t duty = g_m.duty[wheel];

        if (((duty >= MOTION_STALL_DUTY) || (duty <= -MOTION_STALL_DUTY)) &&
            (0u == edges[wheel]))
        {
            g_m.idle_ticks[wheel]++;
        }
        else
        {
            g_m.idle_ticks[wheel] = 0u;
        }

        b_stuck[wheel] = (g_m.idle_ticks[wheel] >= MOTION_STALL_TICKS);
    }

    if (b_stuck[ENC_LEFT] || b_stuck[ENC_RIGHT])
    {
        bool    b_one_moving = (b_stuck[ENC_LEFT] != b_stuck[ENC_RIGHT]);
        int32_t duty_left    = duty_percent(g_m.duty[ENC_LEFT]);
        int32_t duty_right   = duty_percent(g_m.duty[ENC_RIGHT]);

        brake_now();
        enter(S2_STATE_STALLED);
        subsystem4_fault_raise(b_one_moving ? FAULT_ENCODER : FAULT_STALL);
        report_stall(b_one_moving, b_stuck, duty_left, duty_right);
    }
}

/*!
 * @brief Say on the console what the stall rule saw and what to check.
 */
static void
report_stall (bool b_one_moving, bool const b_stuck[2], int32_t duty_left,
              int32_t duty_right)
{
#if MOTION_PRINT_EVENTS
    if (b_one_moving)
    {
        tm_printf((UB *)"S2: ENCODER fault - %s wheel driven (duty %d%%) but "
                  "no encoder pulse for 0.5 s while the other wheel counts\n",
                  b_stuck[ENC_LEFT] ? "left" : "right",
                  (int)(b_stuck[ENC_LEFT] ? duty_left : duty_right));
    }
    else
    {
        tm_printf((UB *)"S2: STALL - motors driven (duty L %d%% R %d%%) but no "
                  "encoder pulse from either wheel for 0.5 s\n",
                  (int)duty_left, (int)duty_right);
        tm_printf((UB *)"S2:   wheels did not turn -> battery switch, motor "
                  "wires; wheels turned -> encoder 3V3/GND/signal wires\n");
    }

    tm_printf((UB *)"S2:   encoder edges since start: GP0 %u GP1 %u (left), "
              "GP2 %u GP3 %u (right); reset with GP21\n",
              (UINT)drv_encoder_pin_edges(0u), (UINT)drv_encoder_pin_edges(1u),
              (UINT)drv_encoder_pin_edges(2u), (UINT)drv_encoder_pin_edges(3u));
#else
    (void)b_one_moving;
    (void)b_stuck;
    (void)duty_left;
    (void)duty_right;
#endif
}

/*!
 * @brief Signed duty as a percentage of full scale.
 */
static int32_t
duty_percent (int32_t duty)
{
    return ((duty * 100) / MOTION_DUTY_MAX);
}

/*!
 * @brief Brake both motors now and go to BRAKED (unless STALLED).
 */
static void
brake_now (void)
{
    uint8_t wheel = 0u;

    drv_motor_brake();

    for (wheel = 0u; wheel < 2u; wheel++)
    {
        motion_pid_reset(&g_m.pid[wheel]);
        g_m.target_mm_s[wheel] = 0;
        g_m.duty[wheel]        = 0;
        g_m.direction[wheel]   = 0;
        g_m.idle_ticks[wheel]  = 0u;
    }

    g_m.b_report_done = false;

    if (S2_STATE_STALLED != g_m.state)
    {
        enter(S2_STATE_BRAKED);
    }
}

/*!
 * @brief End of a MOVE/TURN: brake both motors now but keep the state
 *        and the MOTION_DONE report (STOPPING waits for the wheels).
 */
static void
brake_hold (void)
{
    uint8_t wheel = 0u;

    drv_motor_brake();

    for (wheel = 0u; wheel < 2u; wheel++)
    {
        motion_pid_reset(&g_m.pid[wheel]);
        g_m.target_mm_s[wheel] = 0;
        g_m.duty[wheel]        = 0;
        g_m.idle_ticks[wheel]  = 0u;
    }
}

/*!
 * @brief RP2040 1 MHz timer, low 32 bits (cycle-time measurement only).
 */
static uint32_t
now_us (void)
{
    return ((uint32_t)in_w(TIMER_TIMERAWL));
}

/*!
 * @brief Change state; the new state is published at the end of the cycle.
 */
static void
enter (subsystem2_state_t state)
{
    g_m.state = state;
}

/*!
 * @brief True if a wheel is (almost) stopped.
 */
static bool
is_slow (int32_t speed_mm_s)
{
    return ((speed_mm_s < MOTION_DONE_SPEED_MM_S) &&
            (speed_mm_s > -MOTION_DONE_SPEED_MM_S));
}

/*!
 * @brief Publish the odometry word and state, one store each (IF-02).
 */
static void
publish (void)
{
    subsystem2_odom_t odom;
    uint32_t          word = 0u;

    odom.distance_mm = g_m.distance_mm;
    odom.left_mm_s   = (int16_t)motion_clamp(g_m.speed_mm_s[ENC_LEFT],
                                             -32768, 32767);
    odom.right_mm_s  = (int16_t)motion_clamp(g_m.speed_mm_s[ENC_RIGHT],
                                             -32768, 32767);

    word        = motion_odom_pack(&odom);
    g_odom_word = word;                         /* Single 32-bit store. */
    g_state     = (uint8_t)g_m.state;           /* 3-bit state.         */
}

/*** end of file ***/
