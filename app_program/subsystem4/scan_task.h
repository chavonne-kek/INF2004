/** @file scan_task.h
 *
 * @brief TSK_SCAN: runs Buddy 5's scan / profile / plan / bypass engine
 *        (b5_task.c) and implements its interfaces (b5_if.h) on top of the
 *        team design.  Private to subsystem 4.
 */

#ifndef SCAN_TASK_H
#define SCAN_TASK_H

#include <stdint.h>
#include <stdbool.h>

#include "subsystem4.h"

/* Avoidance hand-over messages, TSK_SCAN -> TSK_CTRL (IF-05). */
#define AVOID_MSG_TAKE         1u      /* Obstacle: may I take over?      */
#define AVOID_MSG_MOTION       2u      /* Please run this motion command. */
#define AVOID_MSG_DONE_OK      3u      /* Finished: resume line following. */
#define AVOID_MSG_DONE_FAIL    4u      /* Gave up: no safe way round.     */

ER       scan_task_init(void);
ER       scan_task_start(void);
uint8_t  scan_task_range_cm(void);
uint16_t scan_task_b5_status(void);

/* TSK_CTRL side of the hand-over. */
bool     scan_task_next_request(uint8_t * p_type, uint16_t * p_cmd);
void     scan_task_answer(bool b_yes);
bool     scan_task_take_rejoin(bool * p_b_line_on_left);

#endif /* SCAN_TASK_H */

/*** end of file ***/
