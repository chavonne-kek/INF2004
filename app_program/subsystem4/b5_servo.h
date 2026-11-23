/**
 * @file b5_servo.h
 * @brief Scanning servo driver (50 Hz hardware PWM).
 */
#ifndef B5_SERVO_H
#define B5_SERVO_H

#include <stdint.h>

void    B5_Servo_Init(void);

/** Move to angle (0..180, clamped) and block until it has settled. */
void    B5_Servo_MoveTo(uint8_t angleDeg);

uint8_t B5_Servo_GetAngle(void);

#endif /* B5_SERVO_H */
