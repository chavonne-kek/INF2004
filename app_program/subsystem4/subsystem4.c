/** @file subsystem4.c
 *
 * @brief Subsystem 4 (Buddy 5): creates flg_events, flg_faults and
 *        mbf_remote_cmd, starts TSK_SCAN and TSK_CTRL, and provides the
 *        public interface in subsystem4.h.
 *
 * Owner: Soma (student ID 2402033).
 */

#include "subsystem4.h"
#include "ctrl_task.h"
#include "scan_task.h"
#include "scan_calc.h"
#include "drv_board.h"
#include "../subsystem1/subsystem1.h"

#define S4_REMOTE_MBF_BYTES    16

/* The S1 event bits must match the team allocation (subsystem4.h). */
typedef char s4_check_bits_t[((EVT_BARCODE == S1_EVT_BARCODE) &&
                               (EVT_LINE_LOST == S1_EVT_LINE_LOST) &&
                               (EVT_LINE_FOUND == S1_EVT_LINE_FOUND) &&
                               (EVT_JUNCTION == S1_EVT_JUNCTION)) ? 1 : -1];

static ID gh_flg_events  = 0;
static ID gh_flg_faults  = 0;
static ID gh_remote_mbf  = 0;


/*!
 * @brief Phase 1: event flags, remote-command buffer, scan objects and
 *        hardware.  Also tells subsystem 1 where to set its event bits.
 */
ER
subsystem4_init (void)
{
    T_CFLG cflg   = {0};
    T_CMBF cmbf   = {0};
    ER     result = E_OK;

    cflg.flgatr  = TA_TFIFO | TA_WSGL;     /* Only TSK_CTRL waits. */
    cflg.iflgptn = 0u;
    gh_flg_events = tk_cre_flg(&cflg);

    cflg.flgatr  = TA_TFIFO | TA_WMUL;     /* Nobody waits: read only. */
    gh_flg_faults = tk_cre_flg(&cflg);

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = S4_REMOTE_MBF_BYTES;
    /* {opcode, argument}: 2 bytes since the map command (2 Nov).  A
     * maxmsz of 1 made tk_snd_mbf() fail with E_PAR, so every dashboard
     * command was dropped before TSK_CTRL saw it. */
    cmbf.maxmsz = (INT)S4_RCMD_REC_LEN;
    gh_remote_mbf = tk_cre_mbf(&cmbf);

    if (gh_flg_events < E_OK)
    {
        result = (ER)gh_flg_events;
    }
    else if (gh_flg_faults < E_OK)
    {
        result = (ER)gh_flg_faults;
    }
    else if (gh_remote_mbf < E_OK)
    {
        result = (ER)gh_remote_mbf;
    }
    else
    {
        subsystem1_bind_events(gh_flg_events);
        result = scan_task_init();

        if (E_OK == result)
        {
            result = ctrl_task_init();
        }
    }

    return (result);
}

/*!
 * @brief Phase 2: start TSK_SCAN (core 0) and TSK_CTRL (core 1).
 */
ER
subsystem4_start (void)
{
    ER result = scan_task_start();

    if (E_OK == result)
    {
        result = ctrl_task_start();
    }

    return (result);
}

/*!
 * @brief Set event bits for TSK_CTRL (tasks and handlers).
 */
void
subsystem4_event_set (uint32_t bits)
{
    if (gh_flg_events > 0)
    {
        (void)tk_set_flg(gh_flg_events, (UINT)(bits & EVT_ALL));
    }
}

/*!
 * @brief Set a fault bit and wake TSK_CTRL.  Call only for a fault your
 *        subsystem owns (subsystem4.h); the kernel updates the bits
 *        atomically, so this is safe from both cores.
 */
void
subsystem4_fault_raise (uint32_t fault)
{
    if (gh_flg_faults > 0)
    {
        (void)tk_set_flg(gh_flg_faults, (UINT)(fault & FAULT_ALL));
        subsystem4_event_set(EVT_FAULT);
    }
}

/*!
 * @brief Clear a fault bit, once its condition has gone.  Only the owner
 *        of the fault may call this (Tribunal 1, Version C).
 */
void
subsystem4_fault_clear (uint32_t fault)
{
    if (gh_flg_faults > 0)
    {
        (void)tk_clr_flg(gh_flg_faults, (UINT)~(fault & FAULT_ALL));
    }
}

/*!
 * @brief Current fault mask (6 bits).
 */
uint32_t
subsystem4_faults (void)
{
    T_RFLG rflg;
    uint32_t mask = 0u;

    if ((gh_flg_faults > 0) && (E_OK == tk_ref_flg(gh_flg_faults, &rflg)))
    {
        mask = (uint32_t)rflg.flgptn & FAULT_ALL;
    }

    return (mask);
}

ID
subsystem4_flg_events (void)
{
    return (gh_flg_events);
}

subsystem4_state_t
subsystem4_state (void)
{
    return ((subsystem4_state_t)(ctrl_task_state() & S4_STATE_MASK));
}

/*!
 * @brief Name of a system state, for console messages.
 */
char const *
subsystem4_state_name (subsystem4_state_t state)
{
    static char const * const names[SYS_STATE_COUNT] =
    {
        "INIT_CALIBRATE", "IDLE", "LINE_FOLLOW", "TURN_EXECUTE",
        "OBSTACLE_AVOID", "LINE_RECOVERY", "PAUSED", "EMERGENCY_STOP",
        "FAULT", "FINISHED"
    };

    return (((uint32_t)state < (uint32_t)SYS_STATE_COUNT) ? names[state]
                                                          : "?");
}

/*!
 * @brief The set fault bits as words, e.g. "STALL IMU", for console
 *        messages.  "none" if no bit is set.
 *
 * @return text (always NUL-terminated).
 */
char const *
subsystem4_fault_text (uint32_t faults, char text[S4_FAULT_TEXT_SIZE])
{
    static char const * const names[FAULT_BITS] =
    {
        "STALL", "ENCODER", "IMU", "NO_ECHO", "WIFI", "LIFTED"
    };
    uint32_t used = 0u;
    uint32_t bit  = 0u;

    text[0] = '\0';

    for (bit = 0u; bit < FAULT_BITS; bit++)
    {
        char const * p_name = names[bit];

        if (0u != (faults & (1u << bit)))
        {
            if ((0u != used) && (used < (S4_FAULT_TEXT_SIZE - 1u)))
            {
                text[used] = ' ';
                used++;
            }

            while (('\0' != *p_name) && (used < (S4_FAULT_TEXT_SIZE - 1u)))
            {
                text[used] = *p_name;
                used++;
                p_name++;
            }
        }
    }

    if (0u == used)
    {
        text[0] = 'n';
        text[1] = 'o';
        text[2] = 'n';
        text[3] = 'e';
        used    = 4u;
    }

    text[used] = '\0';

    return (text);
}

#if APP_TEST
/*!
 * @brief Test only: START presses so far (GP20 or dashboard).
 */
uint32_t
subsystem4_start_presses (void)
{
    return (ctrl_task_presses(BOARD_BTN_START));
}
#endif

uint8_t
subsystem4_range_cm (void)
{
    return (scan_task_range_cm());
}

ID
subsystem4_remote_cmd_mbf (void)
{
    return (gh_remote_mbf);
}

void
subsystem4_profile_pack (subsystem4_profile_t const * p_profile,
                         uint8_t bytes[S4_PROFILE_BYTES])
{
    scan_profile_pack(p_profile, bytes);
}

void
subsystem4_profile_unpack (uint8_t const bytes[S4_PROFILE_BYTES],
                           subsystem4_profile_t * p_profile)
{
    scan_profile_unpack(bytes, p_profile);
}

/*** end of file ***/
