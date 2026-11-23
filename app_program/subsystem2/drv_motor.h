/** @file drv_motor.h
 *
 * @brief Robo Pico on-board motor driver (M1 = left, M2 = right), 20 kHz
 *        PWM on GP8-GP11.  Called only by TSK_MOTION.
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include <stdint.h>

void drv_motor_init(void);
void drv_motor_set(int32_t left_duty, int32_t right_duty);
void drv_motor_brake(void);
void drv_motor_coast(void);

#endif /* DRV_MOTOR_H */

/*** end of file ***/
