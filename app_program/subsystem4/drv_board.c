/** @file drv_board.c
 *
 * @brief Buttons, buzzer and watchdog (see drv_board.h).
 *
 * Watchdog (NFR19): RP2040 erratum RP2040-E1 makes the counter count down
 * twice per microsecond tick, so the load value is 2 x timeout in us.  PSM
 * WDSEL selects what the watchdog resets (everything but the oscillators).
 */

#include "subsystem4.h"
#include "drv_board.h"

#define BUZZER_SYS_HZ          125000000u
#define BUZZER_SLICE_WRAP_MAX  65535u
#define WDG_CTRL               0x40058000u
#define WDG_LOAD               0x40058004u
#define WDG_CTRL_ENABLE        (1u << 30)
#define WDG_CTRL_PAUSE_DBG     (7u << 24)  /* Pause while debugging.      */
#define WDG_LOAD_MAX           0x00FFFFFFu
#define WDG_TICKS_PER_MS       2000u       /* Erratum RP2040-E1.          */
#define PSM_WDSEL              0x40010008u
#define PSM_WDSEL_ALL_BUT_OSC  0x0001FFFCu

static uint32_t g_wdg_load = 0u;

static void input_pull_up(uint8_t pin);


/*!
 * @brief Buttons as inputs with pull-ups; buzzer PWM off.
 */
void
drv_board_init (void)
{
    input_pull_up(S4_PIN_BTN_START);
    input_pull_up(S4_PIN_BTN_RESET);

    clr_w(RESETS_RESET, RESETS_RESET_PWM);

    while (0u == (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM))
    {
        /* Wait for the reset release (no effect if already running). */
    }

    (void)pwm_set_pin(S4_PIN_BUZZER);
    (void)pwm_set_cc(S4_PIN_BUZZER, 0u);
    (void)pwm_set_enabled(S4_PIN_BUZZER, TRUE);
}

/*!
 * @brief Raw button state (active low).  TSK_CTRL debounces.
 */
bool
drv_board_button_pressed (uint8_t button)
{
    uint8_t pin = (BOARD_BTN_RESET == button) ? S4_PIN_BTN_RESET
                                              : S4_PIN_BTN_START;

    return (0u == gpio_get_val(pin));
}

/*!
 * @brief Square wave on the buzzer; 0 Hz = silent.
 */
void
drv_board_buzzer (uint16_t frequency_hz)
{
    uint32_t wrap = 0u;

    if (0u == frequency_hz)
    {
        (void)pwm_set_cc(S4_PIN_BUZZER, 0u);
    }
    else
    {
        wrap = (BUZZER_SYS_HZ / frequency_hz) - 1u;

        if (wrap > BUZZER_SLICE_WRAP_MAX)
        {
            wrap = BUZZER_SLICE_WRAP_MAX;   /* Lowest tone ~1.9 kHz. */
        }

        (void)pwm_set_wrap(S4_PIN_BUZZER, wrap);
        (void)pwm_set_cc(S4_PIN_BUZZER, wrap / 2u);
    }
}

/*!
 * @brief Start the hardware watchdog.  After this, drv_board_watchdog_feed
 *        must be called more often than timeout_ms or the chip resets.
 */
void
drv_board_watchdog_start (uint32_t timeout_ms)
{
    g_wdg_load = timeout_ms * WDG_TICKS_PER_MS;

    if (g_wdg_load > WDG_LOAD_MAX)
    {
        g_wdg_load = WDG_LOAD_MAX;
    }

    out_w(PSM_WDSEL, PSM_WDSEL_ALL_BUT_OSC);
    out_w(WDG_LOAD, g_wdg_load);
    set_w(WDG_CTRL, WDG_CTRL_ENABLE | WDG_CTRL_PAUSE_DBG);
}

/*!
 * @brief Reload the watchdog counter.
 */
void
drv_board_watchdog_feed (void)
{
    if (0u != g_wdg_load)
    {
        out_w(WDG_LOAD, g_wdg_load);
    }
}

/*!
 * @brief Make a pin a digital input with the pull-up on.
 */
static void
input_pull_up (uint8_t pin)
{
    (void)gpio_set_pin(pin, GPIO_MODE_IN);
    clr_w(GPIO((UW)pin), GPIO_PDE);    /* UW: register address maths. */
    set_w(GPIO((UW)pin), GPIO_PUE);
}

/*** end of file ***/
