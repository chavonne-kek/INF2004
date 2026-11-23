/* IS25 servo trim finder (plain Pico SDK, no micro T-Kernel).
 * 50 Hz PWM on GP15, pulse in microseconds.  Starts at 1500 us (the pulse
 * the firmware uses for "front" today).  Keys typed in PuTTY:
 *   a / d : pulse -50 / +50 us        z / c : pulse -10 / +10 us
 *   1 : remember this as FRONT        2 : remember as furthest RIGHT
 *   3 : remember as furthest LEFT     p : print the remembered numbers
 * Nothing else on the car is driven (motors stay off). */
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#define SERVO_PIN 15
#define PULSE_MIN 500
#define PULSE_MAX 2500

static int pulse = 1500, front = -1, right = -1, left = -1;

static void show_marks(void)
{
    printf("\n==== REMEMBERED: FRONT %d us | furthest RIGHT %d us | furthest LEFT %d us"
           " (-1 = not set yet) ====\n\n", front, right, left);
}

static void help(void)
{
    printf("\nIS25 SERVO TRIM on GP15.  Watch the ultrasonic sensor and type:\n"
           "  a / d = move -50 / +50 us     z / c = move -10 / +10 us\n"
           "  1 = this is FRONT   2 = this is as far RIGHT as it can go safely\n"
           "  3 = this is as far LEFT as it can go safely   p = print numbers\n"
           "Stop before it touches anything behind it.\n\n");
}

int main(void)
{
    uint slice;
    pwm_config cfg;
    absolute_time_t next_print;

    stdio_init_all();
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    slice = pwm_gpio_to_slice_num(SERVO_PIN);
    cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys) / 1000000.0f);
    pwm_config_set_wrap(&cfg, 20000 - 1);
    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(SERVO_PIN, (uint16_t)pulse);

    next_print = make_timeout_time_ms(3000);
    for (;;) {
        int ch = getchar_timeout_us(100000);
        int moved = 0;

        switch (ch) {
        case 'a': case 'A': pulse -= 50; moved = 1; break;
        case 'd': case 'D': pulse += 50; moved = 1; break;
        case 'z': case 'Z': pulse -= 10; moved = 1; break;
        case 'c': case 'C': pulse += 10; moved = 1; break;
        case '1': front = pulse; printf(">> FRONT = %d us\n", pulse); show_marks(); break;
        case '2': right = pulse; printf(">> furthest RIGHT = %d us\n", pulse); show_marks(); break;
        case '3': left = pulse; printf(">> furthest LEFT = %d us\n", pulse); show_marks(); break;
        case 'p': case 'P': show_marks(); break;
        case 'h': case 'H': case '?': help(); break;
        default: break;
        }
        if (moved) {
            if (pulse < PULSE_MIN) { pulse = PULSE_MIN; }
            if (pulse > PULSE_MAX) { pulse = PULSE_MAX; }
            pwm_set_gpio_level(SERVO_PIN, (uint16_t)pulse);
            printf("servo pulse now %d us\n", pulse);
        }
        if (time_reached(next_print)) {
            help();
            printf("servo pulse now %d us\n", pulse);
            show_marks();
            next_print = make_timeout_time_ms(15000);
        }
    }
}
