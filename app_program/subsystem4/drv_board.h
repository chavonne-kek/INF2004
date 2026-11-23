/** @file drv_board.h
 *
 * @brief Robo Pico on-board start/reset buttons (GP20/GP21), piezo buzzer
 *        (GP22) and the RP2040 hardware watchdog.  Called only by
 *        TSK_CTRL.
 */

#ifndef DRV_BOARD_H
#define DRV_BOARD_H

#include <stdint.h>
#include <stdbool.h>

#define BOARD_BTN_START        0u
#define BOARD_BTN_RESET        1u

void drv_board_init(void);
bool drv_board_button_pressed(uint8_t button);
void drv_board_buzzer(uint16_t frequency_hz);
void drv_board_watchdog_start(uint32_t timeout_ms);
void drv_board_watchdog_feed(void);

#endif /* DRV_BOARD_H */

/*** end of file ***/
