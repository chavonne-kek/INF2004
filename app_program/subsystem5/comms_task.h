/** @file comms_task.h
 *
 * @brief TSK_COMMS internals.  Private to subsystem 5.
 */

#ifndef COMMS_TASK_H
#define COMMS_TASK_H

#include <stdint.h>
#include <stdbool.h>

#include "subsystem5.h"

ER       comms_task_init(void);
ER       comms_task_start(void);
ER       comms_task_post(subsystem5_tlm_t type, uint8_t const * p_data,
                         uint8_t len);
ID       comms_task_tx_mbf(void);
ID       comms_task_rx_mbf(void);
uint8_t  comms_task_link(void);
uint16_t comms_task_drops(void);
uint16_t comms_task_bad(void);
void     comms_task_set_connected(bool b_up);
void     comms_task_set_signal(uint8_t signal);

#endif /* COMMS_TASK_H */

/*** end of file ***/
