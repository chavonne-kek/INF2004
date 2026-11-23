/** @file subsystem1_check.c
 *
 * @brief S1 check image (make CHECK=1), run after the S1 self-tests:
 *          1. a 10 s calibration sweep by hand (slide the car across the
 *             line), with the raw IR readings every second;
 *          2. then, forever, every 0.5 s: the raw readings, the line
 *             position, the sample rate and the barcode counters.
 *        The car does not drive.  Steps and expected values:
 *        IS25_Subsystem_Checks.txt, subsystem 1.
 */

#include "subsystem1.h"
#include "line_task.h"

#if APP_TEST

#define CHECK_CAL_S            10
#define CHECK_PRINT_MS         500
#define CHECK_MS_PER_S         1000u
#define CHECK_SPAN_NEEDED      400u
#define CHECK_ERR_BIG_MM       10      /* Beyond the 18 mm line's half. */

static char const * line_text(subsystem1_line_t const * p_line);

/*!
 * @brief S1 check: calibration sweep, then the live readout (never
 *        returns).
 */
void
subsystem1_check (void)
{
    subsystem1_diag_t diag;
    subsystem1_line_t line;
    uint32_t          last_samples = 0u;
    INT               second       = 0;
    bool              b_cal        = false;

    tm_printf((UB *)"[CHECK] S1 IR line + barcode. Sensors (1 Nov layout): "
              "R = Grove 5 (GP26) just right of the line, C = Grove 6 "
              "(GP27) on the line's LEFT edge, ARM = Grove 7 (GP28) 35-45 mm "
              "left, over the barcodes\n");
    tm_printf((UB *)"[CHECK] S1 step 1 (%d s): car on the track, battery "
              "may stay off. Slide the car slowly LEFT and RIGHT so every "
              "sensor crosses the black line several times\n", CHECK_CAL_S);

    subsystem1_calibrate_begin();

    for (second = 1; second <= CHECK_CAL_S; second++)
    {
        (void)tk_dly_tsk((TMO)CHECK_MS_PER_S);
        subsystem1_diag_get(&diag);
        tm_printf((UB *)"[CHECK]   %2d s  IR ARM %4u  C %4u  R %4u\n", second,
                  (UINT)diag.raw[2], (UINT)diag.raw[1], (UINT)diag.raw[0]);
    }

    b_cal = subsystem1_calibrate_end();
    subsystem1_diag_get(&diag);
    tm_printf((UB *)"[CHECK] S1 calibration %s: black-white span ARM %u  C %u  "
              "R %u (each needs %u or more)\n", b_cal ? "OK" : "FAILED",
              (UINT)diag.cal_span[2], (UINT)diag.cal_span[1],
              (UINT)diag.cal_span[0], CHECK_SPAN_NEEDED);

    if (!b_cal)
    {
        tm_printf((UB *)"[CHECK]   A small span = that sensor never saw both "
                  "black and white: not moved over the line, too high (needs "
                  "2-3 mm), or not connected (then its number hardly "
                  "changes). Press RUN/RESET to try again\n");
    }

    tm_printf((UB *)"[CHECK] S1 step 2: readout every 0.5 s. Hold each "
              "sensor over the white floor, then over the black line "
              "(black and white must differ by 400 or more). Centre more "
              "black: steer left; more white: steer right; only R on the "
              "line: steer RIGHT; ARM on the line (12 mm of black, centre "
              "off): steer LEFT. Barcode bars under the ARM never steer. "
              "Then push the car over a barcode: 'S1: barcode ...'\n");

    subsystem1_diag_get(&diag);
    last_samples = diag.samples;

    for (;;)
    {
        uint32_t rate = 0u;

        (void)tk_dly_tsk((TMO)CHECK_PRINT_MS);
        subsystem1_diag_get(&diag);
        (void)subsystem1_line_read(&line);
        rate = ((diag.samples - last_samples) * CHECK_MS_PER_S) /
               (uint32_t)CHECK_PRINT_MS;
        last_samples = diag.samples;

        tm_printf((UB *)"[CHECK] S1: IR ARM %4u  C %4u  R %4u | error %+d mm, "
                  "%s | %u samples/s, ADC errors %u | barcodes %u "
                  "(rejected %u)\n",
                  (UINT)diag.raw[2], (UINT)diag.raw[1], (UINT)diag.raw[0],
                  (INT)line.error_mm, line_text(&line), (UINT)rate,
                  (UINT)diag.adc_errors, (UINT)diag.barcodes,
                  (UINT)diag.barcode_errors);
    }
}

/*!
 * @brief The line state in words.  Left-edge tracking: the centre sensor
 *        should sit on the LEFT edge of the line (half black, error 0);
 *        positive error = robot drifted right = steer left.
 */
static char const *
line_text (subsystem1_line_t const * p_line)
{
    char const * p_text = "centre on the left edge (ideal)";

    if (p_line->b_lost)
    {
        p_text = "LOST (no sensor on the line)";
    }
    else if (p_line->b_junction)
    {
        p_text = "JUNCTION";
    }
    else if (p_line->error_mm >= CHECK_ERR_BIG_MM)
    {
        p_text = "steer LEFT";
    }
    else if (p_line->error_mm > 0)
    {
        p_text = "steer left a little";
    }
    else if (p_line->error_mm <= -CHECK_ERR_BIG_MM)
    {
        p_text = "steer RIGHT";
    }
    else if (p_line->error_mm < 0)
    {
        p_text = "steer right a little";
    }
    else
    {
        /* On the edge. */
    }

    return (p_text);
}

/* ---------------------------------------------------------------------
 * Barcode bench (make CHECK=6): only the barcode sensor on GP28 and a bare
 * Pico; labels are swiped by hand.  Same decoder as the car.
 * ------------------------------------------------------------------- */
#define BENCH_CAL_S            8
#define BENCH_PRINT_S          3u

void
subsystem1_barcode_bench (void)
{
    subsystem1_diag_t diag;
    INT               second    = 0;
    bool              b_cal     = false;
    uint16_t          last_good = 0u;
    uint16_t          last_bad  = 0u;
    uint16_t          lo        = 0xFFFFu;
    uint16_t          hi        = 0u;

    line_task_quiet_line(true);     /* GP26/GP27 are not connected. */

    tm_printf((UB *)"\n[BENCH] === Barcode bench (Code 39, any size, any "
              "speed, either direction) ===\n");
    tm_printf((UB *)"[BENCH] Wiring: barcode sensor A0 -> GP26, GP27 or GP28 "
              "(Maker Pi Pico: Grove 6 = GP26/GP27; Robo Pico: Grove 5/6/7), "
              "VCC -> 3V3 (not 5 V), GND -> GND. Hold it 2-3 mm above the "
              "paper, pointing straight down.\n");
    tm_printf((UB *)"[BENCH] Step 1 (%d s): slide the sensor back and forth "
              "across the barcode so it sees the black bars AND the white "
              "paper.\n", BENCH_CAL_S);

    subsystem1_calibrate_begin();

    for (second = 1; second <= BENCH_CAL_S; second++)
    {
        (void)tk_dly_tsk((TMO)CHECK_MS_PER_S);
        subsystem1_diag_get(&diag);
        tm_printf((UB *)"[BENCH]   %d s  GP26 %4u  GP27 %4u  GP28 %4u\n",
                  second, (UINT)diag.raw[0], (UINT)diag.raw[1],
                  (UINT)diag.raw[2]);
    }

    b_cal = line_task_cal_end_barcode();
    subsystem1_diag_get(&diag);

    if (b_cal)
    {
        tm_printf((UB *)"[BENCH] Calibration OK on GP%u: black-white span %u "
                  "(400 needed). Black now means as dark as these bars.\n",
                  (UINT)line_task_barcode_pin(),
                  (UINT)diag.cal_span[line_task_barcode_pin() - 26u]);
    }
    else
    {
        tm_printf((UB *)"[BENCH] Calibration FAILED: biggest span %u (GP%u), "
                  "400 needed. Sensor too high, not over the bars, or A0 not "
                  "on GP26/27/28. Using default levels on GP%u; press RUN to "
                  "retry.\n",
                  (UINT)diag.cal_span[line_task_barcode_pin() - 26u],
                  (UINT)line_task_barcode_pin(), (UINT)line_task_barcode_pin());
    }

    tm_printf((UB *)"[BENCH] Step 2: swipe across the WHOLE barcode, "
              "starting and ending on white paper (at least 1 cm beyond "
              "the first and last bar). Any steady-ish speed.\n");
    tm_printf((UB *)"[BENCH]   Good read:  S1: barcode 'A'   (then S4: ... "
              "command n)\n");
    tm_printf((UB *)"[BENCH]   Bad read:   S1: BC reject r=.. n=..: r=1 extra "
              "bars (noise), r=2 wide/narrow unclear (blur: lower the "
              "sensor), r=3 not Code 39, r=5 stopped before 29 bars (swipe "
              "the whole label)\n");

    subsystem1_diag_get(&diag);
    last_good = diag.barcodes;
    last_bad  = diag.barcode_errors;

    for (;;)
    {
        uint8_t tick = 0u;
        uint8_t pin  = (uint8_t)(line_task_barcode_pin() - 26u);

        /* Track the darkest/whitest reading between status lines. */
        for (tick = 0u; tick < (uint8_t)(BENCH_PRINT_S * 20u); tick++)
        {
            (void)tk_dly_tsk((TMO)50);
            subsystem1_diag_get(&diag);
            lo = (diag.raw[pin] < lo) ? diag.raw[pin] : lo;
            hi = (diag.raw[pin] > hi) ? diag.raw[pin] : hi;
        }

        tm_printf((UB *)"[BENCH] GP%u now %4u (range %u..%u) | read %u "
                  "(+%u) | rejected %u (+%u)\n", (UINT)(26u + pin),
                  (UINT)diag.raw[pin],
                  (UINT)lo, (UINT)hi, (UINT)diag.barcodes,
                  (UINT)(uint16_t)(diag.barcodes - last_good),
                  (UINT)diag.barcode_errors,
                  (UINT)(uint16_t)(diag.barcode_errors - last_bad));
        last_good = diag.barcodes;
        last_bad  = diag.barcode_errors;
        lo        = 0xFFFFu;
        hi        = 0u;
    }
}

#endif /* APP_TEST */

/*** end of file ***/
