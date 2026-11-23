/** @file drv_motor.c
 *
 * @brief Robo Pico motor driver (datasheet table 3):
 *          A = PWM, B = low  -> forward      A = low, B = PWM -> backward
 *          A = low, B = low  -> brake        A = high, B = high -> coast
 *        A signed duty selects the direction.  If a wheel turns the wrong
 *        way, swap its motor wires or set MOTOR_INVERT_x to 1.
 */

#include "subsystem2.h"
#include "drv_motor.h"
#include "motion_calc.h"

#define MOTOR_INVERT_LEFT      0   /* TODO(Kai Xuan): set after wiring check. */
#define MOTOR_INVERT_RIGHT     0

static void set_pair(uint8_t pin_a, uint8_t pin_b, int32_t duty);
static void enable_pwm_block(void);


/*!
 * @brief Configure the four motor pins as 20 kHz PWM, braked.
 */
void
drv_motor_init (void)
{
    static uint8_t const pins[4] =
    {
        S2_PIN_MOT_L_A, S2_PIN_MOT_L_B, S2_PIN_MOT_R_A, S2_PIN_MOT_R_B
    };
    uint8_t index = 0u;

    enable_pwm_block();

    for (index = 0u; index < 4u; index++)
    {
        (void)pwm_set_pin(pins[index]);
        (void)pwm_set_wrap(pins[index], (UW)MOTION_PWM_WRAP);
        (void)pwm_set_cc(pins[index], 0u);
        (void)pwm_set_enabled(pins[index], TRUE);
    }
}

/*!
 * @brief Drive both motors.  Positive duty = forward.
 *
 * @param[in] left_duty  -MOTION_DUTY_MAX..+MOTION_DUTY_MAX.
 * @param[in] right_duty -MOTION_DUTY_MAX..+MOTION_DUTY_MAX.
 */
void
drv_motor_set (int32_t left_duty, int32_t right_duty)
{
#if MOTOR_INVERT_LEFT
    left_duty = -left_duty;
#endif
#if MOTOR_INVERT_RIGHT
    right_duty = -right_duty;
#endif

    set_pair(S2_PIN_MOT_L_A, S2_PIN_MOT_L_B, left_duty);
    set_pair(S2_PIN_MOT_R_A, S2_PIN_MOT_R_B, right_duty);
}

/*!
 * @brief Brake both motors (all four inputs low).
 */
void
drv_motor_brake (void)
{
    set_pair(S2_PIN_MOT_L_A, S2_PIN_MOT_L_B, 0);
    set_pair(S2_PIN_MOT_R_A, S2_PIN_MOT_R_B, 0);
}

/*!
 * @brief Let both motors spin freely (both inputs high = Hi-Z, Robo Pico
 *        datasheet table 3).  Only for turning the wheels by hand during
 *        calibration; normal stops use drv_motor_brake().
 */
void
drv_motor_coast (void)
{
    (void)pwm_set_cc(S2_PIN_MOT_L_A, (UW)MOTION_PWM_WRAP + 1u);
    (void)pwm_set_cc(S2_PIN_MOT_L_B, (UW)MOTION_PWM_WRAP + 1u);
    (void)pwm_set_cc(S2_PIN_MOT_R_A, (UW)MOTION_PWM_WRAP + 1u);
    (void)pwm_set_cc(S2_PIN_MOT_R_B, (UW)MOTION_PWM_WRAP + 1u);
}

/*!
 * @brief Set one motor's two inputs from a signed duty.
 */
static void
set_pair (uint8_t pin_a, uint8_t pin_b, int32_t duty)
{
    int32_t limited = motion_clamp(duty, -MOTION_DUTY_MAX, MOTION_DUTY_MAX);

    if (limited >= 0)
    {
        (void)pwm_set_cc(pin_b, 0u);
        (void)pwm_set_cc(pin_a, (UW)limited);
    }
    else
    {
        (void)pwm_set_cc(pin_a, 0u);
        (void)pwm_set_cc(pin_b, (UW)(-limited));
    }
}

/*!
 * @brief Take the PWM block out of reset.  The kernel releases it only
 *        when the physical timer is configured, so every PWM user does
 *        this; releasing a block that is already running has no effect.
 */
static void
enable_pwm_block (void)
{
    clr_w(RESETS_RESET, RESETS_RESET_PWM);

    while (0u == (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM))
    {
        /* Wait for the reset to finish (a few clock cycles). */
    }
}

/*** end of file ***/
