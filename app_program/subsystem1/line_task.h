/** @file line_task.h
 *
 * @brief TSK_LINE: samples the three IR sensors every 2 ms on core 1,
 *        publishes the line state and decodes barcodes.  Private to
 *        subsystem 1; other subsystems use subsystem1.h.
 */

#ifndef LINE_TASK_H
#define LINE_TASK_H

#include <stdint.h>
#include <stdbool.h>

#include "subsystem1.h"

ER       line_task_init(void);
ER       line_task_start(void);
void     line_task_bind_events(ID flg_id);
uint16_t line_task_word(void);
ID       line_task_barcode_mbf(void);
ID       line_task_id(void);
ID       line_task_cyclic_id(void);
void     line_task_cal_begin(void);
bool     line_task_cal_end(void);
bool     line_task_cal_end_barcode(void);
void     line_task_quiet_line(bool b_quiet);
uint8_t  line_task_barcode_pin(void);
void     line_task_diag(subsystem1_diag_t * p_diag);

#endif /* LINE_TASK_H */

/*** end of file ***/
