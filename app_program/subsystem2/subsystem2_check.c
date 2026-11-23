/** @file subsystem2_check.c
 *
 * @brief S2 check image (make CHECK=2), run after the S2 self-tests and
 *        the S2-M01 wheel-turn count:
 *          - forever, every 0.5 s: the level of each encoder pin GP0..GP3
 *            and the edges counted on it (turn a wheel slowly by hand);
 *          - on each START press (GP20), with the wheels lifted: each
 *            motor alone, forward then backward, 2 s at 50 %, and the
 *            edges each encoder counted meanwhile.
 *        Steps and expected values: IS25_Subsystem_Checks.txt, subsystem 2.
 */

#include "subsystem2.h"
#include "motion_task.h"
#include "drv_encoder.h"
#include "../subsystem4/subsystem4.h"

#if APP_TEST

#define CHECK_PRINT_MS         500
#define CHECK_PINS             4u
#define CHECK_RUN_MS           2000
#define CHECK_PAUSE_MS         1000
#define CHECK_DUTY_PERCENT     50
#define CHECK_PHASES           4u

typedef struct
{
    char const * p_text;
    int8_t       left;          /* Duty in percent, + = forward. */
    int8_t       right;
    uint8_t      wheel;         /* The wheel that should turn.   */
} check_phase_t;

static void     motor_test(void);
static uint32_t port_edges(uint8_t wheel);

/*!
 * @brief S2 check: encoder pin readout, motor test on START (never
 *        returns).
 */
void
subsystem2_check (void)
{
    uint32_t presses = subsystem4_start_presses();
    uint32_t base[CHECK_PINS];
    uint8_t  pin = 0u;

    for (pin = 0u; pin < CHECK_PINS; pin++)
    {
        base[pin] = drv_encoder_pin_edges(pin);
    }

    tm_printf((UB *)"[CHECK] S2 motors + encoders. Left encoder = Grove 1 "
              "(GP0/GP1), right encoder = Grove 2 (GP2/GP3); left motor "
              "M1, right motor M2\n");
    tm_printf((UB *)"[CHECK] S2 step 1: turn each wheel SLOWLY by hand. The "
              "level of its pin must change between 1 and 0 and its edges "
              "must count up (either pin of the port is fine)\n");
    tm_printf((UB *)"[CHECK] S2 step 2: LIFT THE CAR, battery ON, press "
              "START (GP20): each motor turns alone for 2 s, forward then "
              "backward\n");

    for (;;)
    {
        uint32_t levels = 0u;

        (void)tk_dly_tsk((TMO)CHECK_PRINT_MS);
        levels = in_w(GPIO_IN);

        tm_printf((UB *)"[CHECK] S2: LEFT  GP0 %u (%u edges)  GP1 %u (%u) | "
                  "RIGHT GP2 %u (%u)  GP3 %u (%u) | counted on GP%u / GP%u\n",
                  (UINT)(levels & 1u),
                  (UINT)(drv_encoder_pin_edges(0u) - base[0]),
                  (UINT)((levels >> 1u) & 1u),
                  (UINT)(drv_encoder_pin_edges(1u) - base[1]),
                  (UINT)((levels >> 2u) & 1u),
                  (UINT)(drv_encoder_pin_edges(2u) - base[2]),
                  (UINT)((levels >> 3u) & 1u),
                  (UINT)(drv_encoder_pin_edges(3u) - base[3]),
                  (UINT)drv_encoder_active_gpio(ENC_LEFT),
                  (UINT)drv_encoder_active_gpio(ENC_RIGHT));

        if (subsystem4_start_presses() != presses)
        {
            motor_test();
            presses = subsystem4_start_presses();
        }
    }
}

/*!
 * @brief Each motor alone, forward then backward, with the edges both
 *        encoders counted.  Wheels must be lifted.
 */
static void
motor_test (void)
{
    static check_phase_t const phases[CHECK_PHASES] =
    {
        {"LEFT wheel FORWARD",   CHECK_DUTY_PERCENT, 0, ENC_LEFT},
        {"RIGHT wheel FORWARD",  0, CHECK_DUTY_PERCENT, ENC_RIGHT},
        {"LEFT wheel BACKWARD", -CHECK_DUTY_PERCENT, 0, ENC_LEFT},
        {"RIGHT wheel BACKWARD", 0, -CHECK_DUTY_PERCENT, ENC_RIGHT}
    };
    uint8_t index = 0u;

    if (S2_STATE_BRAKED != subsystem2_state())
    {
        tm_printf((UB *)"[CHECK] S2 motor test skipped: motion state %u, not "
                  "BRAKED (press RESET, GP21)\n",
                  (UINT)subsystem2_state());
        return;
    }

    for (index = 0u; index < CHECK_PHASES; index++)
    {
        check_phase_t const * p_phase = &phases[index];
        uint32_t              left0   = port_edges(ENC_LEFT);
        uint32_t              right0  = port_edges(ENC_RIGHT);
        uint32_t              moved   = 0u;
        uint32_t              other   = 0u;

        tm_printf((UB *)"[CHECK] S2 motor %u/%u: %s, %d %% for 2 s - LOOK "
                  "which wheel turns and which way\n", (UINT)(index + 1u),
                  CHECK_PHASES, p_phase->p_text, CHECK_DUTY_PERCENT);
        motion_task_test_drive(p_phase->left, p_phase->right);
        (void)tk_dly_tsk((TMO)CHECK_RUN_MS);
        motion_task_test_drive(0, 0);
        (void)tk_dly_tsk((TMO)CHECK_PAUSE_MS);  /* Wheel stops turning. */

        moved = (ENC_LEFT == p_phase->wheel) ? (port_edges(ENC_LEFT) - left0)
                                             : (port_edges(ENC_RIGHT) -
                                                right0);
        other = (ENC_LEFT == p_phase->wheel) ? (port_edges(ENC_RIGHT) -
                                                right0)
                                             : (port_edges(ENC_LEFT) - left0);

        tm_printf((UB *)"[CHECK]   edges: %s encoder %u, other encoder %u "
                  "-> %s\n", (ENC_LEFT == p_phase->wheel) ? "left" : "right",
                  (UINT)moved, (UINT)other,
                  ((0u != moved) && (other < (moved / 4u))) ? "ok" :
                  ((0u == moved) && (0u != other)) ?
                  "the OTHER encoder counted: encoders or motors swapped "
                  "(Grove 1 + M1 = left, Grove 2 + M2 = right)" :
                  (0u == moved) ?
                  "NO edges: wheel did not turn (battery, motor wires) or "
                  "its encoder gives no pulses (step 1)" :
                  "both encoders counted: check the encoder wiring");
    }

    tm_printf((UB *)"[CHECK] S2 motor test done. A wheel that turned the "
              "WRONG way: swap that motor's two wires at M1/M2, or set "
              "MOTOR_INVERT_x 1 in drv_motor.c\n");
}

/*!
 * @brief Edges on both pins of one wheel's Grove port.
 */
static uint32_t
port_edges (uint8_t wheel)
{
    uint8_t first = (ENC_LEFT == wheel) ? 0u : 2u;

    return (drv_encoder_pin_edges(first) +
            drv_encoder_pin_edges((uint8_t)(first + 1u)));
}

#endif /* APP_TEST */

/*** end of file ***/
