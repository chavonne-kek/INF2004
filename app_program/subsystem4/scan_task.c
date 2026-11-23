/** @file scan_task.c
 *
 * @brief TSK_SCAN (priority 19, core 0): runs Buddy 5's engine
 *        (B5_Task_Run: watch -> coarse -> fine -> plan -> bypass ->
 *        recover) and implements the B5_Interfaces_t it needs.
 *
 * Team rules kept (Week 6, Tribunal 1):
 *   - TSK_SCAN owns the HC-SR04 and the servo (one owner per device).
 *   - TSK_CTRL is the only task that commands motion.  Buddy 5's blocking
 *     motion calls become requests to TSK_CTRL (mbf_avoid_req + an event
 *     bit); TSK_CTRL forwards them to subsystem 2 and answers on
 *     mbf_avoid_ack when MOTION_DONE, a fault, a pause or an e-stop
 *     arrives.  Any refusal or failure makes the B5 call return false,
 *     which B5 already handles (abort, FAULT state).
 *   - Faults: B5_FAULT_NO_ECHO drives FAULT_NO_ECHO in flg_faults (owner
 *     S4).  The other B5 fault bits stay in the B5 status word.
 */

#include "subsystem4.h"
#include "scan_task.h"
#include "scan_calc.h"
#include "b5_task.h"
#include "b5_sonar.h"
#include "b5_types.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem5/subsystem5.h"

#define AVOID_REQ_BYTES        3u      /* type + 16-bit motion command.   */
#define AVOID_REQ_BUF          24
#define AVOID_ACK_BUF          8
#define AVOID_WAIT_SHORT_MS    1000    /* Take / stop / release answers
                                         * (17 Nov night: 300 -> 1000; TSK_CTRL
                                         * answers within its 50 ms loop, but
                                         * a slow console write can delay it). */
#define AVOID_WAIT_MOVE_MS     10000   /* A MOVE or TURN must finish.      */
#define B5_ARG_MAX             255u    /* 8-bit motion command argument.   */
#define B5_TLM_RANGE_MAX       126u
#define B5_ANGLE_FIRST         30u
#define B5_ANGLE_STEP          5u

static ID gh_scan_task = 0;
static ID gh_req_mbf   = 0;     /* mbf_avoid_req: TSK_SCAN -> TSK_CTRL. */
static ID gh_ack_mbf   = 0;     /* mbf_avoid_ack: TSK_CTRL -> TSK_SCAN. */

static bool io_move_forward(uint16_t distance_cm);
static bool io_move_backward(uint16_t distance_cm);
static bool io_turn_left(uint16_t angle_deg);
static bool io_turn_right(uint16_t angle_deg);
static void io_stop(void);
static bool io_line_detected(void);
static bool io_take_control(void);
static void io_release_control(bool b_success);
static void io_publish_profile(uint8_t const * p_payload, uint8_t length);
static void io_line_hint(bool b_line_on_left);

static B5_Interfaces_t const g_io =
{
    io_move_forward,
    io_move_backward,
    io_turn_left,
    io_turn_right,
    io_stop,
    io_line_detected,
    io_take_control,
    io_release_control,
    io_publish_profile,
    io_line_hint
};

/* 20 Nov: set by the engine's line hint just before it hands back after a
 * bypass; TSK_CTRL takes it on DONE_OK and rejoins the line itself. */
static volatile bool g_b_rejoin       = false;
static volatile bool g_b_rejoin_left  = false;

/* 18 Nov: subsystem1_line_seen_count() when the last move or turn began. */
static uint32_t g_seen_at_move = 0u;

static void scan_thread(INT stacd, void * p_exinf);
static bool ask(uint8_t type, uint16_t cmd, TMO wait_ms);
static uint8_t to_range7(uint32_t cm);


/*!
 * @brief Phase 1: hand-over buffers, then Buddy 5's servo, sonar (echo pin
 *        attached to the shared GPIO interrupt) and status word.
 */
ER
scan_task_init (void)
{
    T_CMBF cmbf   = {0};
    ER     result = E_OK;

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = AVOID_REQ_BUF;
    cmbf.maxmsz = (INT)AVOID_REQ_BYTES;
    gh_req_mbf  = tk_cre_mbf(&cmbf);

    cmbf.bufsz  = AVOID_ACK_BUF;
    cmbf.maxmsz = 1;
    gh_ack_mbf  = tk_cre_mbf(&cmbf);

    if (gh_req_mbf < E_OK)
    {
        result = (ER)gh_req_mbf;
    }
    else if (gh_ack_mbf < E_OK)
    {
        result = (ER)gh_ack_mbf;
    }
    else
    {
        B5_Task_Init(&g_io);
    }

    return (result);
}

/*!
 * @brief Phase 2: start TSK_SCAN on core 0.
 */
ER
scan_task_start (void)
{
    gh_scan_task = app_start_task((FP)scan_thread, S4_PRI_SCAN, TP_PRC1,
                                  "scan_task");     /* FP: kernel entry. */

    return ((gh_scan_task < E_OK) ? (ER)gh_scan_task : E_OK);
}

/*!
 * @brief Latest forward range, 7 bits (127 = nothing in range).
 */
uint8_t
scan_task_range_cm (void)
{
    return (to_range7(B5_Sonar_LastCm()));
}

/*!
 * @brief Buddy 5's 13-bit status word (state, action, retries, faults).
 */
uint16_t
scan_task_b5_status (void)
{
    return (B5_Task_GetStatus());
}

/*!
 * @brief TSK_CTRL: take the next hand-over request, without waiting.
 */
bool
scan_task_next_request (uint8_t * p_type, uint16_t * p_cmd)
{
    uint8_t msg[AVOID_REQ_BYTES];
    bool    b_got = false;

    if (tk_rcv_mbf(gh_req_mbf, msg, TMO_POL) == (INT)AVOID_REQ_BYTES)
    {
        *p_type = msg[0];
        *p_cmd  = (uint16_t)(msg[1] | ((uint16_t)msg[2] << 8u));
        b_got   = true;
    }

    return (b_got);
}

/*!
 * @brief TSK_CTRL: after a bypass, which side of the car the line is on
 *        (20 Nov).  True once per bypass.
 */
bool
scan_task_take_rejoin (bool * p_b_line_on_left)
{
    bool b_got = g_b_rejoin;

    if (b_got)
    {
        *p_b_line_on_left = g_b_rejoin_left;
        g_b_rejoin        = false;
    }

    return (b_got);
}

/*!
 * @brief TSK_CTRL: answer the request TSK_SCAN is waiting on.
 */
void
scan_task_answer (bool b_yes)
{
    uint8_t answer = b_yes ? 1u : 0u;

    (void)tk_snd_mbf(gh_ack_mbf, &answer, 1, TMO_POL);
}

/*!
 * @brief TSK_SCAN body: Buddy 5's state machine (never returns).
 */
static void
scan_thread (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    B5_Task_Run();
}

/*!
 * @brief Send one request to TSK_CTRL and wait for its answer.
 *
 * @return True if TSK_CTRL answered yes (granted / motion done).
 */
static bool
ask (uint8_t type, uint16_t cmd, TMO wait_ms)
{
    uint8_t msg[AVOID_REQ_BYTES];
    uint8_t answer = 0u;
    bool    b_yes  = false;

    while (tk_rcv_mbf(gh_ack_mbf, &answer, TMO_POL) > 0)
    {
        /* Drop a late answer to an earlier request. */
    }

    msg[0] = type;
    msg[1] = (uint8_t)(cmd & 0xFFu);
    msg[2] = (uint8_t)(cmd >> 8u);

    if (E_OK == tk_snd_mbf(gh_req_mbf, msg, (INT)AVOID_REQ_BYTES, TMO_POL))
    {
        subsystem4_event_set((AVOID_MSG_TAKE == type) ? EVT_OBSTACLE
                                                      : EVT_AVOID_REQ);

        if (tk_rcv_mbf(gh_ack_mbf, &answer, wait_ms) > 0)
        {
            b_yes = (0u != answer);
        }
    }

    return (b_yes);
}

static bool
motion (subsystem2_op_t op, uint16_t arg)
{
    uint8_t limited = (uint8_t)((arg > B5_ARG_MAX) ? B5_ARG_MAX : arg);

    g_seen_at_move = subsystem1_line_seen_count();  /* io_line_detected() */

    return (ask(AVOID_MSG_MOTION, subsystem2_cmd_pack(op, limited),
                AVOID_WAIT_MOVE_MS));
}

static bool
io_move_forward (uint16_t distance_cm)
{
    return (motion(S2_OP_MOVE_FWD, distance_cm));
}

static bool
io_move_backward (uint16_t distance_cm)
{
    return (motion(S2_OP_MOVE_BACK, distance_cm));
}

static bool
io_turn_left (uint16_t angle_deg)
{
    return (motion(S2_OP_TURN_LEFT, angle_deg));
}

static bool
io_turn_right (uint16_t angle_deg)
{
    return (motion(S2_OP_TURN_RIGHT, angle_deg));
}

static void
io_stop (void)
{
    (void)ask(AVOID_MSG_MOTION, subsystem2_cmd_pack(S2_OP_STOP, 0u),
              AVOID_WAIT_SHORT_MS);
}

/*!
 * @brief Buddy 3: the line is under a sensor now, or a sensor saw it at
 *        any time during the last move or turn.
 *
 * 18 Nov: was "not LINE_LOST", read after each 5 cm step.  Crossing the
 * 18 mm line in the middle of a step and leaving it more than 300 ms
 * before the step ended was missed, so the car drove on past the line
 * (simulator: some off-centre bypasses never rejoined), and a noisy
 * reading kept "not lost" true off the line.  Both now use S1's
 * debounced counts (20 ms in a row).
 */
static bool
io_line_detected (void)
{
    return (subsystem1_line_on() ||
            (subsystem1_line_seen_count() != g_seen_at_move));
}

/*!
 * @brief Line side for the follower after a bypass (18 Nov).
 */
static void
io_line_hint (bool b_line_on_left)
{
    g_b_rejoin_left = b_line_on_left;
    g_b_rejoin      = true;     /* Read by TSK_CTRL on DONE_OK. */
    subsystem1_line_hint(b_line_on_left ? (int8_t)S1_LINE_ERR_MAX_MM
                                        : (int8_t)(-S1_LINE_ERR_MAX_MM));
}

static bool
io_take_control (void)
{
    g_b_rejoin = false;     /* 21 Nov: a new obstacle; no stale rejoin. */

    return (ask(AVOID_MSG_TAKE, 0u, AVOID_WAIT_SHORT_MS));
}

static void
io_release_control (bool b_success)
{
    (void)ask(b_success ? AVOID_MSG_DONE_OK : AVOID_MSG_DONE_FAIL, 0u,
              AVOID_WAIT_SHORT_MS);
}

/*!
 * @brief Buddy 1: convert B5's 7-byte profile to the team's 35-bit,
 *        5-byte telemetry record (subsystem4.h) and post it (IF-06).
 */
static void
io_publish_profile (uint8_t const * p_payload, uint8_t length)
{
    B5_Profile_t         b5;
    subsystem4_profile_t team;
    uint8_t              bytes[S4_PROFILE_BYTES];
    uint32_t             angle = 0u;

    if (B5_PROFILE_WIRE_BYTES != length)
    {
        return;
    }

    B5_Profile_Unpack(p_payload, &b5);
    angle = (b5.closestAngle < B5_ANGLE_FIRST) ? B5_ANGLE_FIRST
                                               : b5.closestAngle;

    team.width_cm       = to_range7(b5.width);
    team.closest_cm     = to_range7(b5.closestDist);
    angle = (angle - B5_ANGLE_FIRST) / B5_ANGLE_STEP;    /* 5-bit index. */
    team.closest_index  = (uint8_t)((angle > S4_SCAN_INDEX_MAX) ?
                                    S4_SCAN_INDEX_MAX : angle);
    team.left_clear_cm  = to_range7(b5.leftClear);
    team.right_clear_cm = to_range7(b5.rightClear);
    team.b_left_edge    = (0u != b5.detected);
    team.b_right_edge   = false;

    scan_profile_pack(&team, bytes);
    (void)subsystem5_post(S5_TLM_OBSTACLE, bytes, S4_PROFILE_BYTES);
}

/*!
 * @brief 8-bit B5 range (255 = none) to the team's 7-bit field.
 */
static uint8_t
to_range7 (uint32_t cm)
{
    return ((uint8_t)((cm > B5_TLM_RANGE_MAX) ? S4_RANGE_NONE : cm));
}

/*** end of file ***/
