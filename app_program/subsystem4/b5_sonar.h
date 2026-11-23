/**
 * @file b5_sonar.h
 * @brief HC-SR04 style ultrasonic ranging, echo timed in a GPIO ISR.
 */
#ifndef B5_SONAR_H
#define B5_SONAR_H

#include <stdint.h>

void    B5_Sonar_Init(void);

/** Single ping. Returns 1..250 cm, or B5_DIST_NO_ECHO. Blocks the task only. */
uint8_t B5_Sonar_MeasureCm(void);

/** reps = 1 -> single ping, reps >= 3 -> median of 3 (rejects spikes). */
uint8_t B5_Sonar_MeasureMedianCm(uint8_t reps);

/** Call from the shared GPIO IRQ dispatcher for the echo pin edges. */
void    B5_Sonar_OnGpioEdge(uint32_t gpioPin, uint32_t events);

/** IS25: last measured range (1..250 cm, 255 = none); any task may read. */
uint8_t B5_Sonar_LastCm(void);

#endif /* B5_SONAR_H */
