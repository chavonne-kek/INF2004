/** @file motion_task.h
 *
 * @brief TSK_MOTION internals.  Private to subsystem 2; other subsystems
 *        use subsystem2.h.
 */

#ifndef MOTION_TASK_H
#define MOTION_TASK_H

#include <stdint.h>

#include "subsystem2.h"

ER       motion_task_init(void);
ER       motion_task_start(void);
ER       motion_task_command(subsystem2_op_t op, uint8_t arg);
uint32_t motion_task_odometry_word(void);
uint8_t  motion_task_state(void);
uint32_t motion_task_alive(void);
ID       motion_task_id(void);
ID       motion_task_cyclic_id(void);
uint16_t motion_task_exec_max_us(void);

#if APP_TEST
void     motion_task_coast(bool b_coast);
void     motion_task_test_drive(int8_t left_percent, int8_t right_percent);
void     motion_task_test_stall(void);
#endif

#endif /* MOTION_TASK_H */

/*** end of file ***/
