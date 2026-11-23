/**
 * @file b5_scan.c
 * @brief Coarse and fine servo/ultrasonic scans.
 */
#include "b5_scan.h"
#include "b5_config.h"
#include "b5_profile.h"
#include "b5_servo.h"
#include "b5_sonar.h"

/* IS25 17 Nov night: a reading this close is the car's own bracket or
 * body seen at the edge of the servo's range, not an obstacle. */
static uint8_t Valid(uint8_t cm)
{
    return (cm <= B5_SELF_ECHO_CM) ? (uint8_t)B5_DIST_NO_ECHO : cm;
}

static const uint8_t k_coarseAngles[B5_COARSE_COUNT] = { 66u, 78u, 90u, 102u, 114u };
/* IS25 17 Nov: the servo can only reach 60..115 deg on the car (b5_config.h),
 * so the coarse fan is 12 deg apart inside it (was 30, 60, 90, 120, 150). */

uint8_t B5_Scan_Coarse(B5_Sample_t *out)
{
    uint8_t i;

    for (i = 0u; i < B5_COARSE_COUNT; i++)
    {
        B5_Servo_MoveTo(k_coarseAngles[i]);
        out[i].angle = k_coarseAngles[i];
        out[i].dist = Valid(B5_Sonar_MeasureMedianCm(1u));
    }
    return B5_COARSE_COUNT;
}

uint8_t B5_Scan_Fine(const B5_Sample_t *coarse, uint8_t coarseCount, B5_Sample_t *out)
{
    uint8_t lo = B5_ANGLE_MAX_DEG;
    uint8_t hi = 0u;
    uint8_t start;
    uint8_t end;
    uint8_t angle;
    uint8_t count = 0u;
    uint8_t i;

    for (i = 0u; i < coarseCount; i++)
    {
        if (B5_Profile_IsObstacle((uint8_t)coarse[i].angle, (uint8_t)coarse[i].dist))
        {
            if (coarse[i].angle < lo)
            {
                lo = (uint8_t)coarse[i].angle;
            }
            if (coarse[i].angle > hi)
            {
                hi = (uint8_t)coarse[i].angle;
            }
        }
    }
    if (lo > hi)
    {
        return 0u;   /* no hits */
    }

    start = (lo > B5_FINE_MARGIN_DEG) ? (uint8_t)(lo - B5_FINE_MARGIN_DEG) : 0u;
    end = ((uint16_t)hi + B5_FINE_MARGIN_DEG > B5_ANGLE_MAX_DEG)
              ? (uint8_t)B5_ANGLE_MAX_DEG : (uint8_t)(hi + B5_FINE_MARGIN_DEG);
    /* IS25: stay inside the angles the servo can reach on the car. */
    if (start < B5_FINE_MIN_DEG)
    {
        start = (uint8_t)B5_FINE_MIN_DEG;
    }
    if (end > B5_FINE_MAX_DEG)
    {
        end = (uint8_t)B5_FINE_MAX_DEG;
    }

    for (angle = start; (angle <= end) && (count < B5_FINE_MAX_SAMPLES);
         angle = (uint8_t)(angle + B5_FINE_STEP_DEG))
    {
        B5_Servo_MoveTo(angle);
        out[count].angle = angle;
        out[count].dist = Valid(B5_Sonar_MeasureMedianCm(B5_FINE_REPS));
        count++;
    }
    return count;
}
