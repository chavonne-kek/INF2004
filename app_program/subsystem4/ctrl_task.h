/** @file ctrl_task.h
 *
 * @brief TSK_CTRL internals.  Private to subsystem 4.
 */

#ifndef CTRL_TASK_H
#define CTRL_TASK_H

#include <stdint.h>

#include "subsystem4.h"

ER      ctrl_task_init(void);
ER      ctrl_task_start(void);
uint8_t ctrl_task_state(void);

#if APP_TEST
/* Start (BOARD_BTN_START) or reset presses so far, button or dashboard.
 * In test images a start is only counted: the car does not drive. */
uint32_t ctrl_task_presses(uint8_t button);
#endif

#endif /* CTRL_TASK_H */

/*** end of file ***/
