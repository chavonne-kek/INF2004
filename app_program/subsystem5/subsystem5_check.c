/** @file subsystem5_check.c
 *
 * @brief S5 check image (make CHECK=5), run after the S5 self-tests.
 *        TSK_COMMS prints its 2 s status line in this image (link,
 *        signal, dropped messages, bad commands, and every value the
 *        telemetry carries).  With MQTT (make CHECK=5 NET=lwip MQTT=1,
 *        after the IF-07 patch) the link must come ONLINE and every
 *        dashboard command is printed by TSK_CTRL; the car does not drive.
 *        Steps and expected values: IS25_Subsystem_Checks.txt,
 *        subsystem 5.
 */

#include "subsystem5.h"

#if APP_TEST

#define CHECK_REMIND_MS        30000

/*!
 * @brief S5 check: instructions, then sleep (TSK_COMMS does the printing).
 */
void
subsystem5_check (void)
{
#if defined(TM_MQTT)
    tm_printf((UB *)"[CHECK] S5 WiFi + MQTT. Laptop on the same WiFi, "
              "broker running, then: python tools/is25_dashboard.py --broker "
              "<laptop IP>\n");
    tm_printf((UB *)"[CHECK] S5: within about 10 s 'S5: link ONLINE' and a "
              "signal in dBm; the dashboard shows a heartbeat every second. "
              "Type start / pause / reset / speed 15: each prints 'S4: "
              "dashboard command ...' here. Type bad: 'bad commands' goes up "
              "by 20\n");
#else
    tm_printf((UB *)"[CHECK] S5: this image has NO MQTT (that needs the "
              "IF-07 patch, after instructor approval: make CHECK=5 "
              "NET=lwip MQTT=1). The status line below is what the robot "
              "would send, decoded\n");
#endif

    for (;;)
    {
        (void)tk_dly_tsk((TMO)CHECK_REMIND_MS);
        tm_printf((UB *)"[CHECK] S5 check image running (status line every "
                  "2 s)\n");
    }
}

#endif /* APP_TEST */

/*** end of file ***/
