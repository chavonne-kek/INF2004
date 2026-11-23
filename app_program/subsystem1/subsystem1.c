/** @file subsystem1.c
 *
 * @brief Subsystem 1 (Buddy 3): creates its kernel objects, starts
 *        TSK_LINE and provides the public interface in subsystem1.h.
 *
 * Owner: Hao Ming (student ID 2500331).
 */

#include "subsystem1.h"
#include "line_task.h"
#include "line_sense.h"
#include "barcode.h"

/* Kernel tick needed for 2 ms sampling (Week 6 assumption A5). */
#define S1_REQUIRED_TICK_MS    1


/*!
 * @brief Phase 1 of start-up: create the cyclic handler and the barcode
 *        message buffer.  Called by usermain() before any task starts.
 *
 * @return E_OK, or a kernel error code.
 */
ER
subsystem1_init (void)
{
#if defined(CNF_TIMER_PERIOD) && (CNF_TIMER_PERIOD > S1_REQUIRED_TICK_MS)
    tm_printf((UB *)"S1 WARNING: kernel tick is %d ms; 2 ms IR sampling "
              "needs CNF_TIMER_PERIOD = 1 (config/config.h)\n",
              CNF_TIMER_PERIOD);
#endif

    return (line_task_init());
}

/*!
 * @brief Phase 2 of start-up: start TSK_LINE on core 1 and its 2 ms tick.
 *
 * @return E_OK, or a kernel error code.
 */
ER
subsystem1_start (void)
{
    return (line_task_start());
}

/*!
 * @brief Tell subsystem 1 which event flag to set its event bits in
 *        (S1_EVT_BARCODE, S1_EVT_LINE_LOST, S1_EVT_LINE_FOUND,
 *        S1_EVT_JUNCTION).  Called once by the flag's owner (subsystem 4)
 *        after it has created the flag.  Until then no bits are set; the
 *        line state and barcode messages are still available.
 *
 * @param[in] flg_id ID of the controller's event flag.
 */
void
subsystem1_bind_events (ID flg_id)
{
    line_task_bind_events(flg_id);
}

/*!
 * @brief Read the latest line state (IF-01).  Safe from any task on any
 *        core: the state is one 16-bit word read in a single load.
 *
 * @param[out] p_line Unpacked line state.
 *
 * @return True if at least one sensor sees the line.
 */
bool
subsystem1_line_read (subsystem1_line_t * p_line)
{
    line_sense_unpack(line_task_word(), p_line);

    return (p_line->b_valid);
}

/*!
 * @brief ID of the message buffer that receives one byte per decoded
 *        barcode (IF-03).  Receive with
 *        tk_rcv_mbf(subsystem1_barcode_mbf(), &msg, timeout).
 */
ID
subsystem1_barcode_mbf (void)
{
    return (line_task_barcode_mbf());
}

/*!
 * @brief Decoded character of a barcode message, e.g. 'A'.
 */
char
subsystem1_barcode_char (uint8_t msg)
{
    return (barcode_index_to_char((uint8_t)(msg & S1_BC_INDEX_MASK)));
}

/*!
 * @brief Navigation command of a barcode message (A, B, C or D).
 */
subsystem1_cmd_t
subsystem1_barcode_cmd (uint8_t msg)
{
    return (barcode_char_to_cmd(subsystem1_barcode_char(msg)));
}

/*!
 * @brief True if the barcode was read in the reverse direction.
 */
bool
subsystem1_barcode_is_reversed (uint8_t msg)
{
    return (0u != (msg & S1_BC_REVERSED_BIT));
}

/*!
 * @brief Start IR calibration.  Sweep all three sensors across both the
 *        white floor and the black line (e.g. INIT_CALIBRATE spin), then
 *        call subsystem1_calibrate_end().
 */
void
subsystem1_calibrate_begin (void)
{
    line_task_cal_begin();
}

/*!
 * @brief Finish IR calibration.
 *
 * @return True if applied; false if a sensor did not see both white and
 *         black (the previous calibration is kept).
 */
bool
subsystem1_calibrate_end (void)
{
    return (line_task_cal_end());
}

/*!
 * @brief Copy subsystem 1's diagnostic counters.
 */
void
subsystem1_diag_get (subsystem1_diag_t * p_diag)
{
    line_task_diag(p_diag);
}

/*** end of file ***/
