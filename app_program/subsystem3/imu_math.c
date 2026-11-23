/** @file imu_math.c
 *
 * @brief Float maths without libm (see imu_math.h).
 */

#include "imu_math.h"

#define IMU_HALF_PI_F          1.57079633f
#define IMU_QUARTER_PI_F       0.78539816f
#define IMU_ATAN_A_F           0.2447f     /* atan(z) for |z| <= 1:        */
#define IMU_ATAN_B_F           0.0663f     /* max error 0.0015 rad.        */
#define IMU_SQRT_ITERATIONS    20u
#define IMU_SIN_C3_F           (1.0f / 6.0f)
#define IMU_SIN_C5_F           (1.0f / 120.0f)
#define IMU_SIN_C7_F           (1.0f / 5040.0f)


/*!
 * @brief |x|.
 */
float
imu_absf (float x)
{
    return ((x < 0.0f) ? -x : x);
}

/*!
 * @brief Square root by Newton's method (x >= 0; returns 0 otherwise).
 */
float
imu_sqrtf (float x)
{
    float   guess = 0.0f;
    uint8_t loop  = 0u;

    if (x > 0.0f)
    {
        guess = (x > 1.0f) ? x : 1.0f;

        for (loop = 0u; loop < IMU_SQRT_ITERATIONS; loop++)
        {
            guess = 0.5f * (guess + (x / guess));
        }
    }

    return (guess);
}

/*!
 * @brief atan(z) for |z| <= 1 (polynomial approximation).
 */
static float
atan_unit (float z)
{
    float abs_z = imu_absf(z);

    return ((IMU_QUARTER_PI_F * z) -
            (z * (abs_z - 1.0f) * (IMU_ATAN_A_F + (IMU_ATAN_B_F * abs_z))));
}

/*!
 * @brief Four-quadrant arctangent, radians (-pi..pi).
 */
float
imu_atan2f (float y, float x)
{
    float result = 0.0f;

    if ((0.0f == x) && (0.0f == y))
    {
        result = 0.0f;
    }
    else if (imu_absf(x) >= imu_absf(y))
    {
        result = atan_unit(y / x);

        if (x < 0.0f)
        {
            result += (y >= 0.0f) ? IMU_PI_F : -IMU_PI_F;
        }
    }
    else
    {
        result = ((y > 0.0f) ? IMU_HALF_PI_F : -IMU_HALF_PI_F) -
                 atan_unit(x / y);
    }

    return (result);
}

/*!
 * @brief Arcsine, radians.  The input is clamped to -1..1.
 */
float
imu_asinf (float s)
{
    float clamped = s;

    if (clamped > 1.0f)
    {
        clamped = 1.0f;
    }
    else if (clamped < -1.0f)
    {
        clamped = -1.0f;
    }
    else
    {
        /* In range. */
    }

    return (imu_atan2f(clamped, imu_sqrtf(1.0f - (clamped * clamped))));
}

/*!
 * @brief Sine for small angles (|rad| <= pi/2), Taylor series to x^7.
 */
float
imu_sinf (float rad)
{
    float x2 = rad * rad;

    return (rad * (1.0f - (x2 * (IMU_SIN_C3_F -
                                 (x2 * (IMU_SIN_C5_F -
                                        (x2 * IMU_SIN_C7_F)))))));
}

/*** end of file ***/
