/** @file test_host.c
 *
 * @brief PC (host) tests for the pure parts of subsystem 1: the Code 39
 *        decoder (barcode.c) and line sensing (line_sense.c).
 *
 * Build and run from this folder:  make        (or: make run)
 *
 * The barcode is simulated physically: 3 mm narrow bars (141 mm barcode,
 * as in "Barcode & Line Specification"), a 2 mm sensor spot, ADC noise,
 * 2 ms sampling and the real hysteresis in line_sense.c.  Results are
 * printed as [HOST] lines; the program returns 1 if any check fails.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Include the modules' source so their private tables can be checked. */
#include "../../../app_program/subsystem1/barcode.c"
#include "../../../app_program/subsystem1/line_sense.c"

/* Same as the wrappers in subsystem1.c (which needs the kernel). */
#define subsystem1_barcode_char(m) \
    barcode_index_to_char((uint8_t)((m) & S1_BC_INDEX_MASK))
#define subsystem1_barcode_is_reversed(m) (0u != ((m) & S1_BC_REVERSED_BIT))

#define SAMPLE_S        (S1_SAMPLE_MS / 1000.0)   /* Sampling period, s. */
#define NARROW_MM       3.0     /* 141 mm / 47 narrow units.            */
#define SPOT_MM         2.0     /* Sensor spot width (box blur).        */
#define QUIET_MM        40.0    /* White before and after the barcode.  */
#define RAW_WHITE       300.0
#define RAW_BLACK       3000.0
#define MAX_EDGES       64

static int g_pass = 0;
static int g_fail = 0;

/* Geometry used by the physical model (H-BC13 changes them). */
static double g_narrow_mm = NARROW_MM;
static double g_spot_mm   = SPOT_MM;
static double g_wobble    = 0.0;    /* Hand: speed varies +-this fraction. */
static double g_weave     = 0.0;    /* 18 Nov: car weaving: arm speed varies
                                       +-this fraction every g_weave_mm.  */
static double g_weave_mm  = 40.0;
static int    g_use_travel = 0;     /* 18 Nov: feed encoder travel too.   */

static void
check (char const * p_id, int b_ok, char const * p_desc)
{
    if (b_ok)
    {
        g_pass++;
    }
    else
    {
        g_fail++;
    }

    printf("[HOST] %-8s %s  %s\n", p_id, b_ok ? "PASS" : "FAIL", p_desc);
}

/* ---------------------------------------------------------------------
 * Physical model
 * ------------------------------------------------------------------- */

/* Bar edges in mm: black between edge[2k] and edge[2k+1]. */
typedef struct
{
    double edge[MAX_EDGES];
    int    count;
    double length;
} track_t;

/* Lay out "*c*" with the given wide:narrow ratio.  One bar can be damaged:
 * damage_elem >= 0 forces that element to narrow.
 */
static void
make_track (uint16_t data_pattern, double ratio, int damage_elem,
            track_t * p_track)
{
    uint16_t chars[3] = {BARCODE_STAR_PATTERN, data_pattern,
                         BARCODE_STAR_PATTERN};
    double   x        = QUIET_MM;
    int      elem_no  = 0;

    p_track->count = 0;

    for (int c = 0; c < 3; c++)
    {
        for (int e = 0; e < 9; e++)
        {
            int    b_wide = (chars[c] >> (8 - e)) & 1;
            double width;

            if (elem_no == damage_elem)
            {
                b_wide = 0;
            }

            width = g_narrow_mm * (b_wide ? ratio : 1.0);

            if (0 == (e % 2))
            {
                p_track->edge[p_track->count++] = x;
                p_track->edge[p_track->count++] = x + width;
            }

            x += width;
            elem_no++;
        }

        if (c < 2)
        {
            x += g_narrow_mm;   /* Inter-character gap. */
            elem_no++;
        }
    }

    p_track->length = x + QUIET_MM;
}

/* Fraction of the sensor spot centred at x that is black. */
static double
darkness (track_t const * p_track, double x)
{
    double lo    = x - (g_spot_mm / 2.0);
    double hi    = x + (g_spot_mm / 2.0);
    double black = 0.0;

    for (int k = 0; k < p_track->count; k += 2)
    {
        double a = fmax(lo, p_track->edge[k]);
        double b = fmin(hi, p_track->edge[k + 1]);

        if (b > a)
        {
            black += b - a;
        }
    }

    return (black / g_spot_mm);
}

static double
gauss (void)
{
    double u1 = (rand() + 1.0) / ((double)RAND_MAX + 2.0);
    double u2 = (rand() + 1.0) / ((double)RAND_MAX + 2.0);

    return (sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2));
}

/* Drive over a track.  Speed changes linearly from v0 to v1 (cm/s).
 * b_reverse drives from the far end.  contrast < 1 models a faded print
 * or a lower black reading.  Returns the number of DECODED results; the
 * last message goes to *p_msg and REJECTED count to *p_rejects.
 */
static int
drive (track_t const * p_track, double v0, double v1, int b_reverse,
       double noise, double contrast, uint8_t * p_msg, int * p_rejects)
{
    barcode_t    bc;
    line_sense_t sense;
    line_cal_t   cal;
    double       x       = 0.0;
    int          decodes = 0;
    uint8_t      msg     = 0u;

    barcode_init(&bc);
    line_sense_init(&sense);
    line_sense_cal_default(&cal);
    *p_rejects = 0;

    while (x < p_track->length)
    {
        double   pos  = b_reverse ? (p_track->length - x) : x;
        double   v    = (v0 + ((v1 - v0) * x / p_track->length)) *
                        (1.0 + (g_wobble *
                                sin(2.0 * M_PI * x / p_track->length))) *
                        (1.0 + (g_weave * sin(2.0 * M_PI * x / g_weave_mm)));
        double   raw  = RAW_WHITE + ((RAW_BLACK - RAW_WHITE) * contrast *
                                     darkness(p_track, pos)) +
                        (noise * gauss());
        uint16_t r[LINE_SENSORS];
        uint16_t word;

        if (raw < 0.0)
        {
            raw = 0.0;
        }

        if (raw > 4095.0)
        {
            raw = 4095.0;
        }

        r[LINE_RIGHT]   = 300u;    /* Right guard on white.        */
        r[LINE_CENTRE]  = 1650u;   /* Centre on the line's edge.   */
        r[LINE_BARCODE] = (uint16_t)raw;
        (void)line_sense_update(&sense, &cal, r, &word);

        double   step_mm = v * 10.0 * SAMPLE_S;
        static double carry = 0.0;      /* 1/16 mm not yet passed on. */
        uint16_t travel = 0u;

        if (g_use_travel)
        {
            double t16 = (step_mm * 16.0) + carry;

            travel = (uint16_t)t16;
            carry  = t16 - (double)travel;
        }

        barcode_result_t res = barcode_step_travel(&bc, sense.b_bc_black,
                                                   travel, &msg);

        if (BARCODE_RESULT_DECODED == res)
        {
            decodes++;
            *p_msg = msg;
        }
        else if (BARCODE_RESULT_REJECTED == res)
        {
            (*p_rejects)++;
        }

        x += v * 10.0 * SAMPLE_S;   /* cm/s -> mm per sample. */
    }

    return (decodes);
}

/* Feed a list of element widths (in samples) between quiet zones. */
static int
feed_widths (int const * p_widths, int n, int b_reverse, uint8_t * p_msg)
{
    barcode_t bc;
    int       decodes = 0;
    uint8_t   msg     = 0u;

    barcode_init(&bc);

    for (int s = 0; s < 60; s++)
    {
        (void)barcode_step(&bc, false, &msg);
    }

    for (int i = 0; i < n; i++)
    {
        int w = b_reverse ? p_widths[n - 1 - i] : p_widths[i];

        for (int s = 0; s < w; s++)
        {
            if (BARCODE_RESULT_DECODED ==
                barcode_step(&bc, 0 == (i % 2), &msg))
            {
                decodes++;
                *p_msg = msg;
            }
        }
    }

    for (int s = 0; s < 300; s++)
    {
        if (BARCODE_RESULT_DECODED == barcode_step(&bc, false, &msg))
        {
            decodes++;
            *p_msg = msg;
        }
    }

    return (decodes);
}

/* ---------------------------------------------------------------------
 * Tests
 * ------------------------------------------------------------------- */

/* Slide barcodes (BARCODE.pptx), measured bar widths in pixels. */
static int const g_slide[6][29] =
{
    {92,278,93,92,278,93,278,92,93,93,278,92,93,93,92,278,93,92,278,93,
     93,278,92,93,278,92,278,93,93},                                /* A */
    {92,278,93,92,278,93,278,92,93,93,92,93,278,93,277,278,93,93,92,93,
     93,278,92,93,278,92,278,93,93},                                /* F */
    {92,278,93,92,278,93,278,92,93,93,92,278,278,93,278,92,93,93,92,93,
     93,278,92,93,278,92,278,93,93},                                /* Z */
    {91,274,91,91,274,91,274,91,91,92,273,274,274,91,91,91,92,91,91,91,
     92,273,92,91,274,91,273,92,91},                                /* W */
    {91,272,90,91,272,91,272,90,91,91,90,272,91,91,272,90,91,91,272,90,
     91,272,91,90,272,91,272,91,90},                                /* X */
    {9,27,9,9,27,9,27,9,9,9,27,27,9,9,27,9,9,9,9,9,9,27,9,9,27,9,27,9,9}
};
static char const g_slide_char[6] = {'A', 'F', 'Z', 'W', 'X', 'Y'};

static void
test_slides (void)
{
    int ok = 1;

    for (int k = 0; k < 6; k++)
    {
        for (int b_rev = 0; b_rev < 2; b_rev++)
        {
            /* Scale pixels so the narrow bar is 8 samples (~19 cm/s). */
            int     w[29];
            uint8_t msg = 0u;
            double  scale = 8.0 / (double)g_slide[k][0];

            for (int i = 0; i < 29; i++)
            {
                w[i] = (int)lround(g_slide[k][i] * scale);
            }

            if ((1 != feed_widths(w, 29, b_rev, &msg)) ||
                (g_slide_char[k] != subsystem1_barcode_char(msg)) ||
                (b_rev != (int)subsystem1_barcode_is_reversed(msg)))
            {
                printf("        slide %c rev=%d failed\n", g_slide_char[k],
                       b_rev);
                ok = 0;
            }
        }
    }

    check("H-BC01", ok, "slide barcodes A F Z W X Y, both directions");
}

static void
test_all_chars (void)
{
    int ok = 1;

    for (uint8_t idx = 0u; idx < BARCODE_SET_SIZE; idx++)
    {
        track_t t;

        if (BARCODE_STAR_PATTERN == g_patterns[idx])
        {
            continue;   /* '*' is start/stop only. */
        }

        make_track(g_patterns[idx], 3.0, -1, &t);

        for (int b_rev = 0; b_rev < 2; b_rev++)
        {
            uint8_t msg = 0u;
            int     rej = 0;

            if ((1 != drive(&t, 15.0, 15.0, b_rev, 0.0, 1.0, &msg, &rej)) ||
                (barcode_index_to_char(idx) !=
                 subsystem1_barcode_char(msg)) ||
                (b_rev != (int)subsystem1_barcode_is_reversed(msg)))
            {
                printf("        char %c rev=%d failed\n",
                       barcode_index_to_char(idx), b_rev);
                ok = 0;
            }
        }
    }

    check("H-BC02", ok, "every Code 39 character, both directions");
}

/* Speed sweep with noise: commands A B C D, 200 runs per speed. */
static void
test_speeds (void)
{
    static char const   cmds[4]   = {'A', 'B', 'C', 'D'};
    static double const speeds[7] = {3.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0};
    int                 b_no_wrong = 1;
    int                 b_rate_ok  = 1;

    for (int s = 0; s < 7; s++)
    {
        int good = 0;
        int wrong = 0;
        int none = 0;

        for (int run = 0; run < 200; run++)
        {
            char    c = cmds[run % 4];
            uint8_t idx = (uint8_t)(strchr(g_symbols, c) - g_symbols);
            track_t t;
            uint8_t msg = 0u;
            int     rej = 0;
            int     n;

            make_track(g_patterns[idx], 3.0, -1, &t);
            n = drive(&t, speeds[s], speeds[s], run & 1, 120.0, 1.0, &msg,
                      &rej);

            if ((1 == n) && (c == subsystem1_barcode_char(msg)))
            {
                good++;
            }
            else if (0 == n)
            {
                none++;
            }
            else
            {
                wrong++;
            }
        }

        printf("        %4.1f cm/s: %3d good, %d missed, %d wrong\n",
               speeds[s], good, none, wrong);

        if (0 != wrong)
        {
            b_no_wrong = 0;
        }

        if (good < 196)
        {
            b_rate_ok = 0;
        }
    }

    check("H-BC03", b_rate_ok, ">= 98% read at 3-30 cm/s with noise");
    check("H-BC04", b_no_wrong, "never a wrong character at 3-30 cm/s");
}

/* H-BC13 (2 Nov): the label on the track (3.5 mm narrow, 10.5 mm wide),
 * a blurry spot (5 mm: the arm sits higher than the line sensors) and
 * motor noise on the reading (sigma 350 counts, about a quarter of the
 * arm's black-white span, so the decision flickers at every edge), at the
 * slow speeds the car now uses.  Before the debounce and 16-bit widths
 * this read 0-60 %; 2 cm/s could not read at all (wide bar > 0.51 s). */
static void
test_slow_noisy (void)
{
    static double const speeds[5] = {2.0, 3.0, 4.0, 8.0, 12.0};
    static char const   cmds[4]   = {'A', 'B', 'C', 'D'};
    int                 b_ok      = 1;

    g_narrow_mm = 3.5;
    g_spot_mm   = 5.0;

    for (int s = 0; s < 5; s++)
    {
        int good = 0;
        int wrong = 0;

        for (int run = 0; run < 100; run++)
        {
            char    c = cmds[run % 4];
            uint8_t idx = (uint8_t)(strchr(g_symbols, c) - g_symbols);
            track_t t;
            uint8_t msg = 0u;
            int     rej = 0;
            int     n;

            make_track(g_patterns[idx], 3.0, -1, &t);
            n = drive(&t, speeds[s], speeds[s], run & 1, 350.0, 1.0, &msg,
                      &rej);

            if ((1 == n) && (c == subsystem1_barcode_char(msg)))
            {
                good++;
            }
            else if (0 != n)
            {
                wrong++;
            }
        }

        printf("        noisy, 3.5 mm bars, %4.1f cm/s: %3d/100 read, "
               "%d wrong\n", speeds[s], good, wrong);
        b_ok = b_ok && (good >= 95) && (0 == wrong);
    }

    g_narrow_mm = NARROW_MM;
    g_spot_mm   = SPOT_MM;

    check("H-BC13", b_ok,
          "track label, blurry spot, motor noise: >= 95% at 2-12 cm/s");
}

/* H-BC16 (18 Nov): the car weaves while the label passes.  The barcode
 * sensor sits beside the left wheel, so steering speeds it up and slows it
 * down although the car's speed is steady; here its speed swings +-60 %
 * every 40 mm (run1.csv 17 Nov: wheel speeds 5..55 mm/s while weaving).
 * Widths measured in time are then wrong; measured in travel (encoders)
 * they are right.  Prints both; the check is on travel. */
static void
test_weave (void)
{
    static char const cmds[4] = {'A', 'B', 'C', 'D'};
    int               good[2] = {0, 0};
    int               wrong   = 0;

    g_weave     = 0.6;
    g_weave_mm  = 40.0;
    g_narrow_mm = 3.5;

    for (int mode = 0; mode < 2; mode++)
    {
        g_use_travel = mode;

        for (int run = 0; run < 100; run++)
        {
            char    c   = cmds[run % 4];
            uint8_t idx = (uint8_t)(strchr(g_symbols, c) - g_symbols);
            track_t t;
            uint8_t msg = 0u;
            int     rej = 0;
            int     n;

            g_weave_mm = 30.0 + (double)(run % 5) * 5.0;   /* 30..50 mm */
            make_track(g_patterns[idx], 3.0, -1, &t);
            n = drive(&t, 3.0, 3.0, run & 1, 60.0, 1.0, &msg, &rej);

            if ((1 == n) && (c == subsystem1_barcode_char(msg)))
            {
                good[mode]++;
            }
            else if ((0 != n) && (1 == mode))
            {
                wrong++;
            }
        }
    }

    printf("        weaving car (arm speed +-60 %%): time widths %d/100, "
           "travel widths %d/100 read, %d wrong\n", good[0], good[1], wrong);

    g_weave      = 0.0;
    g_use_travel = 0;
    g_narrow_mm  = NARROW_MM;

    check("H-BC16", (good[1] >= 95) && (0 == wrong),
          "weaving car: travel widths read >= 95 %, none wrong");
}

/* H-BC17 (18 Nov): travel fed but the car parked (a label swiped by hand
 * past it): the decoder falls back to time widths. */
static void
test_parked_travel (void)
{
    track_t t;
    uint8_t msg = 0u;
    int     rej = 0;
    int     n   = 0;
    uint8_t idx = (uint8_t)(strchr(g_symbols, 'B') - g_symbols);

    g_use_travel = 0;   /* drive() feeds 0 travel: the parked car. */
    make_track(g_patterns[idx], 3.0, -1, &t);
    n = drive(&t, 3.0, 3.0, 0, 30.0, 1.0, &msg, &rej);
    check("H-BC17", (1 == n) && ('B' == subsystem1_barcode_char(msg)),
          "no travel (hand swipe, car parked): time widths still read");
}

/* H-BC14 (3 Nov): swiped by hand over the two demo label sizes from
 * Barcode_Sample.pdf (3.3 mm narrow / 165 mm, and 6.9 mm narrow / 328 mm),
 * from very slow to fast, the hand speeding up 2x or slowing down 2x
 * across the label AND wobbling +-40 %, both directions, 3 mm spot. */
static void
test_hand (void)
{
    static double const sizes[2]  = {3.3, 6.9};
    static double const speeds[5] = {0.7, 2.0, 5.0, 10.0, 20.0};
    static char const   cmds[4]   = {'A', 'Z', 'C', 'X'};
    int                 b_ok      = 1;

    g_spot_mm = 3.0;
    g_wobble  = 0.4;

    for (int z = 0; z < 2; z++)
    {
        g_narrow_mm = sizes[z];

        for (int s = 0; s < 5; s++)
        {
            int good = 0;
            int wrong = 0;

            for (int run = 0; run < 40; run++)
            {
                char    c   = cmds[run % 4];
                uint8_t idx = (uint8_t)(strchr(g_symbols, c) - g_symbols);
                double  v0  = speeds[s] * (((run / 2) % 2) ? 0.7 : 1.4);
                double  v1  = speeds[s] * (((run / 2) % 2) ? 1.4 : 0.7);
                track_t t;
                uint8_t msg = 0u;
                int     rej = 0;
                int     n;

                make_track(g_patterns[idx], 3.0, -1, &t);
                n = drive(&t, v0, v1, run & 1, 60.0, 1.0, &msg, &rej);

                if ((1 == n) && (c == subsystem1_barcode_char(msg)))
                {
                    good++;
                }
                else if (0 != n)
                {
                    wrong++;
                }
            }

            printf("        hand, %.1f mm narrow, %4.1f cm/s: %2d/40 read, "
                   "%d wrong\n", sizes[z], speeds[s], good, wrong);
            b_ok = b_ok && (good >= 39) && (0 == wrong);
        }
    }

    g_narrow_mm = NARROW_MM;
    g_spot_mm   = SPOT_MM;
    g_wobble    = 0.0;

    check("H-BC14", b_ok,
          "hand swipe, both demo label sizes, 0.7-20 cm/s, uneven speed");
}

static void
test_speed_change (void)
{
    static double const ramps[4][2] = {{8.0, 16.0}, {16.0, 8.0},
                                       {10.0, 25.0}, {25.0, 10.0}};
    int ok = 1;

    for (int r = 0; r < 4; r++)
    {
        track_t t;
        uint8_t msg = 0u;
        int     rej = 0;

        make_track(0x109u, 3.0, -1, &t);   /* 'A' */

        if ((1 != drive(&t, ramps[r][0], ramps[r][1], 0, 80.0, 1.0, &msg,
                        &rej)) ||
            ('A' != subsystem1_barcode_char(msg)))
        {
            printf("        ramp %.0f->%.0f failed\n", ramps[r][0],
                   ramps[r][1]);
            ok = 0;
        }
    }

    check("H-BC05", ok, "speed changing x2 while reading (both ways)");
}

static void
test_print_quality (void)
{
    int     ok_ratio    = 1;
    int     ok_contrast = 1;
    track_t t;
    uint8_t msg = 0u;
    int     rej = 0;

    /* Printed wide:narrow ratio from 2.2:1 to 3.5:1. */
    for (double ratio = 2.2; ratio <= 3.51; ratio += 0.1)
    {
        make_track(0x109u, ratio, -1, &t);

        if ((1 != drive(&t, 15.0, 15.0, 0, 60.0, 1.0, &msg, &rej)) ||
            ('A' != subsystem1_barcode_char(msg)))
        {
            printf("        ratio %.1f failed\n", ratio);
            ok_ratio = 0;
        }
    }

    /* Bars only 75% as dark as the calibrated line. */
    make_track(0x109u, 3.0, -1, &t);
    ok_contrast = (1 == drive(&t, 15.0, 15.0, 0, 60.0, 0.75, &msg, &rej)) &&
                  ('A' == subsystem1_barcode_char(msg));

    check("H-BC06", ok_ratio, "print ratio 2.2:1 to 3.5:1 still reads");
    check("H-BC07", ok_contrast, "bars 75% as dark as the line still read");
}

static void
test_damaged (void)
{
    int ok = 1;

    /* Make every wide element narrow, one at a time: never decode. */
    for (int e = 0; e < 29; e++)
    {
        track_t t;
        uint8_t msg = 0u;
        int     rej = 0;
        int     c = e / 10;
        int     k = e % 10;
        uint16_t pat = (1 == c) ? 0x109u : BARCODE_STAR_PATTERN;

        if ((9 == k) || (0 == ((pat >> (8 - k)) & 1)))
        {
            continue;   /* Gap or already narrow. */
        }

        make_track(0x109u, 3.0, e, &t);

        if (0 != drive(&t, 15.0, 15.0, 0, 0.0, 1.0, &msg, &rej))
        {
            printf("        damaged element %d decoded as %c\n", e,
                   subsystem1_barcode_char(msg));
            ok = 0;
        }
    }

    check("H-BC08", ok, "any one wide bar printed narrow is rejected");
}

/* An 18 mm black line under the barcode sensor (crossing or turn). */
static void
test_black_line (void)
{
    track_t t;
    uint8_t msg = 0u;
    int     rej = 0;
    int     ok  = 1;

    for (double w = 6.0; w <= 60.0; w += 6.0)
    {
        t.count   = 2;
        t.edge[0] = QUIET_MM;
        t.edge[1] = QUIET_MM + w;
        t.length  = QUIET_MM * 2.0 + w;

        for (double v = 3.0; v <= 30.0; v += 9.0)
        {
            if (0 != drive(&t, v, v, 0, 120.0, 1.0, &msg, &rej))
            {
                ok = 0;
            }
        }
    }

    check("H-BC09", ok, "plain black lines 6-60 mm never decode");
}

/* Random bar sequences: count false decodes. */
static void
test_random (void)
{
    int false_decodes = 0;
    int runs = 20000;

    for (int run = 0; run < runs; run++)
    {
        int     n = 5 + (rand() % 40);
        int     w[64];
        uint8_t msg = 0u;

        for (int i = 0; i < n; i++)
        {
            w[i] = 4 + (rand() % 30);
        }

        false_decodes += feed_widths(w, n, 0, &msg);
    }

    printf("        %d false decodes in %d random bar sequences\n",
           false_decodes, runs);
    check("H-BC10", 0 == false_decodes, "random bar sequences never decode");
}

static void
test_too_slow (void)
{
    track_t t;
    uint8_t msg = 0u;
    int     rej = 0;
    int     n;

    /* At 1.5 cm/s a 9 mm bar is 300 samples (> 255): must not mis-read. */
    make_track(0x109u, 3.0, -1, &t);
    n = drive(&t, 1.5, 1.5, 0, 0.0, 1.0, &msg, &rej);
    check("H-BC11", (0 == n) || ('A' == subsystem1_barcode_char(msg)),
          "below minimum speed: no wrong character");
}

static void
test_commands (void)
{
    int ok = (SUBSYSTEM1_CMD_LEFT     == barcode_char_to_cmd('A')) &&
             (SUBSYSTEM1_CMD_RIGHT    == barcode_char_to_cmd('B')) &&
             (SUBSYSTEM1_CMD_STRAIGHT == barcode_char_to_cmd('C')) &&
             (SUBSYSTEM1_CMD_UTURN    == barcode_char_to_cmd('D')) &&
             (SUBSYSTEM1_CMD_NONE     == barcode_char_to_cmd('Z'));

    check("H-BC12", ok, "A/B/C/D map to left/right/straight/U-turn");
}

/* ---------------- line sensing ---------------- */

static uint8_t
sense (line_sense_t * p_s, line_cal_t const * p_cal, uint16_t l,
       uint16_t c, uint16_t r, subsystem1_line_t * p_line)
{
    uint16_t raw[LINE_SENSORS];
    uint16_t word   = 0u;

    raw[LINE_LEFT]   = l;     /* The arm (barcode sensor). */
    raw[LINE_CENTRE] = c;
    raw[LINE_RIGHT]  = r;
    uint8_t  ev     = line_sense_update(p_s, p_cal, raw, &word);

    line_sense_unpack(word, p_line);

    return (ev);
}

static void
test_line (void)
{
    line_sense_t      s;
    line_cal_t        cal;
    subsystem1_line_t ln;
    int               ok;
    int               prev;

    line_sense_cal_default(&cal);

    /* 20 Nov: the centre is binary - on black it holds straight, off black
     * it steers.  A black centre must never produce a correction (that
     * proportional ramp is what made the car zigzag on a straight line). */
    ok = 1;
    for (uint16_t c = 2000u; c <= 3000u; c += 100u)
    {
        line_sense_init(&s);
        (void)sense(&s, &cal, 300u, c, 300u, &ln);

        if (ln.error_mm <= 0)     /* Line under the left sensor: steer left. */
        {
            ok = 0;
        }
    }
    check("H-LS01", ok, "left sensor black alone = steer left");

    /* The arm steers only after LINE_ARM_MM (12 mm) of unbroken black. */
    line_sense_init(&s);
    line_sense_travel(&s, 13u);
    (void)sense(&s, &cal, 3000u, 300u, 300u, &ln);
    ok = (0 == ln.error_mm);             /* The arm never steers. */
    line_sense_init(&s);
    (void)sense(&s, &cal, 300u, 300u, 3000u, &ln);
    ok = ok && (-15 == ln.error_mm);     /* Right guard alone: ramp start. */
    check("H-LS02", ok, "arm never steers; right guard alone = -15");

    /* 20 Nov, straddle layout: the line runs BETWEEN the two sensors, so
     * both white is the centred state and must be dead straight. */
    {
        line_sense_init(&s);
        (void)sense(&s, &cal, 300u, 300u, 3000u, &ln);
        {
            int8_t right_only = ln.error_mm;

            line_sense_init(&s);
            (void)sense(&s, &cal, 300u, 300u, 300u, &ln);
            check("H-LS21", (-15 == right_only) && (0 == ln.error_mm),
                  "right guard alone = right, both white = centred/straight");
        }
    }

    /* Centred between the sensors is the healthy state: never LOST. */
    line_sense_init(&s);
    ok = 1;
    for (int i = 0; i < 2000; i++)
    {
        uint8_t ev = sense(&s, &cal, 300u, 300u, 300u, &ln);
        ok = ok && (!(ev & LINE_EVT_LOST)) && ln.b_valid && (0 == ln.error_mm);
    }
    check("H-LS03", ok, "both white = centred, straight, never LOST");

    /* Four binary rules. */
    line_sense_init(&s);
    (void)sense(&s, &cal, 300u, 3000u, 300u, &ln);
    {
        int8_t left_only = ln.error_mm;

        line_sense_init(&s);
        (void)sense(&s, &cal, 300u, 3000u, 3000u, &ln);
        check("H-LS04", (left_only > 0) && (0 == ln.error_mm),
              "left sensor black = steer left, both black = junction/straight");
    }

    /* JUNCTION on either side (Week 6 IF-01): the side sensor black for
     * >= 14 mm of travel (left: with the centre black; right: with the
     * centre still on the line).  Distance comes from the odometry in
     * whole mm, as on the robot (updated every 10 ms).  1 Nov: the left
     * side used a 60 ms timer, which a 9 mm bar beats at 12 cm/s. */
    {
        static double const speeds[4] = {50.0, 100.0, 200.0, 250.0};
        int ok_branch = 1;
        int ok_bar    = 1;

        for (int side = 0; side < 2; side++)
        {
            for (int k = 0; k < 4; k++)
            {
                for (int pass = 0; pass < 2; pass++)
                {
                    /* pass 0: 18 mm branch line + 2 mm blur; pass 1: 9 mm
                     * wide bar + 2 mm blur. */
                    double black_mm = (0 == pass) ? 60.0 : 11.0;
                    double x        = 0.0;
                    int    odom     = 0;
                    int    seen     = 0;

                    line_sense_init(&s);
                    for (int i = 0; i < 2000; i++)
                    {
                        int      tick_odom = ((i % 5) == 0) ? (int)x : odom;
                        bool     b_on      = (x > 50.0) &&
                                             (x < 50.0 + black_mm);
                        uint16_t on        = b_on ? 3000u : 300u;
                        uint8_t  ev        = 0u;

                        line_sense_travel(&s, (uint8_t)(tick_odom - odom));
                        odom = tick_odom;
                        /* Both track sensors black together = junction. */
                        /* pass 0: both track sensors black = junction.
                         * pass 1: a bar under the arm only. */
                        ev   = (0 == pass)
                               ? sense(&s, &cal, on, on, on, &ln)
                               : sense(&s, &cal, on, 900u, 300u, &ln);
                        seen |= (ev & LINE_EVT_JUNCTION) ? 1 : 0;
                        x += speeds[k] * SAMPLE_S;
                    }

                    if (0 == pass) { ok_branch = ok_branch && seen; }
                    else           { ok_bar    = ok_bar && !seen; }
                }
            }
        }

        check("H-LS05", ok_branch,
              "left and right branch -> JUNCTION at 5..25 cm/s");
        check("H-LS06", ok_bar,
              "widest barcode bar is never a junction (either side)");
    }

    /* Barcode beside the line under the arm, centre on its edge: the
     * steering stays proportional (never the +40 of a drift), the line is
     * never LOST, and nothing is remembered as "line on the left".  Then
     * the same bars while drifted left (right guard on the line): always
     * a right correction (-15 growing to -40, 17 Nov). */
    {
        static int const  pattern[9] = {1, 0, 0, 0, 0, 1, 0, 0, 1}; /* A */
        double            x     = 0.0;
        int               odom  = 0;
        int               worst = 0;
        int               b_left_steer = 0;

        line_sense_init(&s);
        for (int i = 0; i < 1500; i++)
        {
            int      tick_odom = ((i % 5) == 0) ? (int)x : odom;
            int      unit      = (int)(x / 3.0);       /* 3 mm units. */
            int      elem      = 0;
            int      u         = 0;
            bool     b_bar     = false;

            /* Walk the 3 mm units through 9 elements (wide = 3 units). */
            for (elem = 0; elem < 9; elem++)
            {
                int w = pattern[elem] ? 3 : 1;

                if ((unit >= u) && (unit < u + w))
                {
                    b_bar = (0 == (elem % 2));
                    break;
                }
                u += w;
            }

            line_sense_travel(&s, (uint8_t)(tick_odom - odom));
            odom = tick_odom;
            (void)sense(&s, &cal, b_bar ? 3000u : 300u, 300u, 300u, &ln);
            worst = (abs(ln.error_mm) > worst) ? abs(ln.error_mm) : worst;
            b_left_steer |= ln.b_lost ? 1 : 0;
            x += 120.0 * SAMPLE_S;     /* 12 cm/s. */
        }

        ok = (worst == 0) && (!b_left_steer) && (s.last_error_mm < 12);

        line_sense_init(&s);
        x    = 0.0;
        odom = 0;
        for (int i = 0; i < 1500; i++)
        {
            int  tick_odom = ((i % 5) == 0) ? (int)x : odom;
            bool b_bar     = (((int)(x / 9.0)) % 2) == 0;    /* 9 mm bars. */

            line_sense_travel(&s, (uint8_t)(tick_odom - odom));
            odom = tick_odom;
            (void)sense(&s, &cal, b_bar ? 3000u : 300u, 300u, 3000u, &ln);
            ok = ok && (ln.error_mm <= -15);   /* Right, never left. */
            x += 120.0 * SAMPLE_S;
        }

        printf("        bars under the arm: worst error %d mm\n", worst);
        check("H-LS20", ok,
              "barcode bars under the arm never steer the car (1 Nov)");
    }

    /* A brush the car corrects away in a few mm is not a branch, and a
     * stationary car never accumulates branch travel. */
    ok = 1;
    line_sense_init(&s);
    for (int i = 0; i < 500; i++)
    {
        line_sense_travel(&s, (uint8_t)(((i % 5) == 0) ? ((i < 25) ? 2u : 0u)
                                                       : 0u));
        ok = ok && !(sense(&s, &cal, 300u, 300u, 3000u, &ln) &
                     LINE_EVT_JUNCTION);
    }
    line_sense_init(&s);
    for (int i = 0; i < 500; i++)
    {
        line_sense_travel(&s, 0u);
        ok = ok && !(sense(&s, &cal, 300u, 400u, 3000u, &ln) &
                     LINE_EVT_JUNCTION);
    }
    check("H-LS12", ok, "10 mm brush / standing still: no junction");

    /* Pack/unpack every error value with every flag combination. */
    ok = 1;
    for (int e = -40; e <= 40; e++)
    {
        for (int f = 0; f < 8; f++)
        {
            subsystem1_line_t in = {(int8_t)e, f & 1, (f >> 1) & 1,
                                    (f >> 2) & 1};
            subsystem1_line_t out;
            uint16_t          w = line_sense_pack(&in);

            line_sense_unpack(w, &out);

            if ((in.error_mm != out.error_mm) ||
                (in.b_valid != out.b_valid) ||
                (in.b_lost != out.b_lost) ||
                (in.b_junction != out.b_junction) ||
                (0u != (w & 0xFC00u)))
            {
                ok = 0;
            }
        }
    }
    check("H-LS07", ok, "line word: -40..+40 x all flags, 10 bits used");

    /* Calibration works with either sensor polarity. */
    {
        line_cal_run_t run;
        uint16_t       hi[3] = {3500u, 3400u, 3300u};
        uint16_t       lo[3] = {200u, 250u, 300u};
        uint16_t       flat[3] = {1000u, 1000u, 1000u};

        line_sense_cal_start(&run);
        line_sense_cal_track(&run, hi);
        line_sense_cal_track(&run, lo);
        ok = line_sense_cal_apply(&run, &cal) &&
             (line_sense_normalise(3500u, cal.white[0], cal.black[0]) >= 990u)
             && (line_sense_normalise(200u, cal.white[0], cal.black[0]) <= 10u);

        line_sense_cal_start(&run);
        line_sense_cal_track(&run, flat);
        ok = ok && (!line_sense_cal_apply(&run, &cal));
        check("H-LS08", ok, "calibration applies, rejects a flat sweep");
    }

    {
        /* Inverted polarity: line reads LOW. */
        uint16_t n_black = line_sense_normalise(200u, 3500u, 200u);
        uint16_t n_white = line_sense_normalise(3500u, 3500u, 200u);

        check("H-LS09", (n_black >= 990u) && (n_white <= 10u),
              "normalise works if black reads low");
    }
}

int
main (void)
{
    srand(2004u);

    test_slides();
    test_all_chars();
    test_speeds();
    test_speed_change();
    test_slow_noisy();
    test_weave();
    test_parked_travel();
    test_hand();
    test_print_quality();
    test_damaged();
    test_black_line();
    test_random();
    test_too_slow();
    test_commands();
    test_line();

    printf("[HOST] SUMMARY %d passed, %d failed\n", g_pass, g_fail);

    return ((0 == g_fail) ? 0 : 1);
}

/*** end of file ***/
