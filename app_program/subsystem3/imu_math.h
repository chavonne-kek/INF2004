/** @file imu_math.h
 *
 * @brief Small float maths for the IMU: the firmware does not link libm,
 *        so atan2, asin, sin and sqrt are approximated here.  Accuracy is
 *        better than 0.1 degree, finer than the 0.25 degree pitch field.
 *        Pure logic, tested against libm on a PC.
 */

#ifndef IMU_MATH_H
#define IMU_MATH_H

#include <stdint.h>

#define IMU_PI_F               3.14159265f
#define IMU_RAD_TO_DEG_F       57.2957795f
#define IMU_DEG_TO_RAD_F       0.0174532925f

float imu_absf(float x);
float imu_sqrtf(float x);
float imu_atan2f(float y, float x);
float imu_asinf(float s);
float imu_sinf(float rad);

#endif /* IMU_MATH_H */

/*** end of file ***/
