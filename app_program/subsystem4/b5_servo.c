/**
 * @file b5_servo.c
 * @brief Scanning servo driver.
 *
 * Uses a PWM slice with a 1 MHz tick so the compare level equals the pulse
 * width in microseconds. Make sure the slice is not shared with Buddy 2's
 * motor PWM (RP2040: slice = GPIO / 2 mod 8; channels of one slice share
 * frequency).
 *
 * IS25 integration (26 Oct): Pico SDK PWM calls replaced by libbsp plus a
 * direct write of the slice DIV register (libbsp has no divider call):
 * 125 MHz / 125 = 1 MHz.  GP15 = slice 7B, used by nothing else.
 */
#include "b5_servo.h"
#include "b5_config.h"
#include "b5_port.h"
#include "b5_types.h"
#ifndef B5_HOST_TEST
#include "subsystem4.h"   /* IS25: libbsp pwm_* and register names */
#endif

#define SERVO_SLICE         (7u)                /* GP15 -> slice 7 */
#define SERVO_DIV_1MHZ      (125u << 4)         /* DIV is 8.4 fixed point */
#define PWM_SLICE_STRIDE    (0x14u)

static uint8_t s_angleDeg = B5_SERVO_FRONT_DEG;

void B5_Servo_Init(void)
{
    clr_w(RESETS_RESET, RESETS_RESET_PWM);
    while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0u)
    {
        /* wait (no effect if the PWM block is already running) */
    }

    (void)pwm_set_pin(B5_PIN_SERVO);
    out_w(PWM_BASE + (SERVO_SLICE * PWM_SLICE_STRIDE) + PWM_CHx_DIV, SERVO_DIV_1MHZ);
    (void)pwm_set_wrap(B5_PIN_SERVO, (UW)(B5_SERVO_PERIOD_US - 1u));
    (void)pwm_set_enabled(B5_PIN_SERVO, TRUE);

    s_angleDeg = B5_ANGLE_MAX_DEG;          /* unknown: force a full settle */
    B5_Servo_MoveTo(B5_SERVO_FRONT_DEG);
}

void B5_Servo_MoveTo(uint8_t angleDeg)
{
    uint32_t pulseUs;
    uint32_t delta;
    uint8_t target = angleDeg;

    if (target > B5_ANGLE_MAX_DEG)
    {
        target = B5_ANGLE_MAX_DEG;
    }

    /* IS25 17 Nov: pulse from the measured front pulse (2200 us on the car,
     * not 1500), clamped to the measured safe limits so the sensor never
     * presses against the car whatever angle is asked for. */
    {
        int32_t us = (int32_t)B5_SERVO_FRONT_US +
                     ((((int32_t)target - (int32_t)B5_SERVO_FRONT_DEG) *
                       (int32_t)B5_SERVO_US_PER_DEG_X10) / 10);

        if (us < (int32_t)B5_SERVO_LIMIT_RIGHT_US)
        {
            us = (int32_t)B5_SERVO_LIMIT_RIGHT_US;
        }
        if (us > (int32_t)B5_SERVO_LIMIT_LEFT_US)
        {
            us = (int32_t)B5_SERVO_LIMIT_LEFT_US;
        }
        pulseUs = (uint32_t)us;
    }
    (void)pwm_set_cc(B5_PIN_SERVO, (UW)pulseUs);

    delta = (target > s_angleDeg) ? (uint32_t)(target - s_angleDeg)
                                  : (uint32_t)(s_angleDeg - target);
    s_angleDeg = target;

    /* Wait for the horn to arrive before the caller pings. */
    B5_Port_DelayMs(B5_SERVO_SETTLE_BASE_MS + (delta * B5_SERVO_SETTLE_PER_DEG_MS));
}

uint8_t B5_Servo_GetAngle(void)
{
    return s_angleDeg;
}
