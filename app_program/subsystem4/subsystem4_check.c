/** @file subsystem4_check.c
 *
 * @brief S4 check image (make CHECK=4), run after the S4 self-tests:
 *          - a servo sweep front -> right -> left -> front (again on every
 *            START press);
 *          - forever, every 0.5 s: the HC-SR04 range, the echo fault and
 *            the button presses (TSK_CTRL beeps and prints each press).
 *        The car does not drive.  Steps and expected values:
 *        IS25_Subsystem_Checks.txt, subsystem 4.
 */

#include "subsystem4.h"
#include "ctrl_task.h"
#include "drv_board.h"
#include "b5_servo.h"
#include "b5_config.h"

#if APP_TEST

#define CHECK_PRINT_MS         500
#define CHECK_HOLD_MS          1500
#define CHECK_SWEEP_STEPS      4u

static void servo_sweep(void);

/*!
 * @brief S4 check: servo sweep, then the live readout (never returns).
 */
void
subsystem4_check (void)
{
    uint32_t presses = ctrl_task_presses(BOARD_BTN_START);

    tm_printf((UB *)"[CHECK] S4 ultrasonic + servo + buttons. HC-SR04 on "
              "Grove 3: Trig = WHITE = GP4, Echo = YELLOW = GP5; servo "
              "signal GP15; buttons GP20 (START), GP21 (RESET)\n");
    servo_sweep();
    tm_printf((UB *)"[CHECK] S4: readout every 0.5 s. Hold a flat board in "
              "front of the sensor at 5, 20, 50 and 100 cm and compare. "
              "START (GP20) beeps and repeats the servo sweep; RESET (GP21) "
              "beeps\n");

    for (;;)
    {
        uint8_t  range  = 0u;
        uint32_t faults = 0u;

        (void)tk_dly_tsk((TMO)CHECK_PRINT_MS);
        range  = subsystem4_range_cm();
        faults = subsystem4_faults();

        if (S4_RANGE_NONE == range)
        {
            tm_printf((UB *)"[CHECK] S4: range nothing within 1.26 m | echo "
                      "%s | servo %u deg | presses START %u, RESET %u\n",
                      (0u != (faults & FAULT_NO_ECHO)) ?
                      "MISSING (NO_ECHO fault)" : "ok",
                      (UINT)B5_Servo_GetAngle(),
                      (UINT)ctrl_task_presses(BOARD_BTN_START),
                      (UINT)ctrl_task_presses(BOARD_BTN_RESET));
        }
        else
        {
            tm_printf((UB *)"[CHECK] S4: range %u cm | echo %s | servo %u deg "
                      "| presses START %u, RESET %u\n", (UINT)range,
                      (0u != (faults & FAULT_NO_ECHO)) ?
                      "MISSING (NO_ECHO fault)" : "ok",
                      (UINT)B5_Servo_GetAngle(),
                      (UINT)ctrl_task_presses(BOARD_BTN_START),
                      (UINT)ctrl_task_presses(BOARD_BTN_RESET));
        }

        if (ctrl_task_presses(BOARD_BTN_START) != presses)
        {
            servo_sweep();
            presses = ctrl_task_presses(BOARD_BTN_START);
        }
    }
}

/*!
 * @brief Front, right, left, front, 1.5 s each.  TSK_SCAN only moves the
 *        servo while it avoids an obstacle, which TSK_CTRL never allows in
 *        a test image, so the check may move it here.
 */
static void
servo_sweep (void)
{
    static uint8_t const angles[CHECK_SWEEP_STEPS] =
    {
        B5_SERVO_FRONT_DEG, B5_SERVO_RIGHT_DEG, B5_SERVO_LEFT_DEG,
        B5_SERVO_FRONT_DEG
    };
    static char const * const where[CHECK_SWEEP_STEPS] =
    {
        "straight AHEAD", "as far RIGHT as it may go (60 deg, ~1867 us)",
        "as far LEFT as it may go (115 deg, ~2478 us)", "straight AHEAD"
    };
    uint8_t step = 0u;

    for (step = 0u; step < CHECK_SWEEP_STEPS; step++)
    {
        tm_printf((UB *)"[CHECK] S4 servo -> %u deg: the sensor must point %s"
                  "\n", (UINT)angles[step], where[step]);
        B5_Servo_MoveTo(angles[step]);
        (void)tk_dly_tsk((TMO)CHECK_HOLD_MS);
    }
}

#endif /* APP_TEST */

/*** end of file ***/
