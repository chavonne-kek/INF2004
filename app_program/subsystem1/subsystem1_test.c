/** @file subsystem1_test.c
 *
 * @brief On-target tests for subsystem 1.  Built only with: make TEST=1
 *
 * Each TEST_CHECK prints one PASS/FAIL line on the console.  Save the
 * console output to testing/subsystem1/logs/ and record the result in
 * testing/subsystem1/TEST_LOG.md.  Test IDs: S1-U01 ... S1-U11.
 *
 * The decoder and line-sensing checks feed synthetic samples into fresh
 * decoder/sensing structures, so they do not disturb the running task.
 */

#include "subsystem1.h"
#include "line_task.h"
#include "line_sense.h"
#include "barcode.h"
#include "../test_report.h"

#if APP_TEST

#define TEST_RATE_WINDOW_MS    1000u   /* Window for the sample-rate test. */
#define TEST_RATE_EXPECTED     (1000u / S1_SAMPLE_MS)  /* Samples per s. */
#define TEST_RATE_TOLERANCE    (TEST_RATE_EXPECTED / 20u)  /* +-5 %. */
#define TEST_QUIET_SAMPLES     (120u / S1_SAMPLE_MS)  /* White before. */
#define TEST_QUIET_NARROWS     10u     /* Code 39 quiet zone after it: 10 x
                                          narrow (the decoder needs 5 x).   */
#define TEST_WIDE_UNITS        3u      /* Wide = 3 x narrow.              */
#define TEST_CHARS             3u
#define TEST_CHAR_ELEMENTS     9u
#define TEST_PATTERN_MSB       8u
#define TEST_PATTERN_STAR      0x094u  /* '*' */
#define TEST_PATTERN_A         0x109u  /* 'A' */
#define TEST_LOST_SAMPLES      (320u / S1_SAMPLE_MS)  /* 320 ms white. */
#define TEST_RAW_WHITE         300u
#define TEST_RAW_BLACK         3000u
#define TEST_RAW_EDGE          1650u   /* Half way: centre on the edge. */
#define TEST_RAW_DARKER        2200u   /* Centre more on the line. */

static uint8_t  feed_barcode(uint16_t data_pattern, uint8_t narrow,
                             uint8_t narrow_end, bool b_reverse,
                             bool b_corrupt, uint8_t * p_msg);
static void     build_units(uint16_t data_pattern, uint8_t units[]);
static uint8_t  sense_once(line_sense_t * p_sense, line_cal_t const * p_cal,
                           uint16_t left, uint16_t centre, uint16_t right,
                           subsystem1_line_t * p_line);


/*!
 * @brief Run every subsystem 1 test and print the results.
 */
void
subsystem1_test (void)
{
    subsystem1_diag_t before;
    subsystem1_diag_t after;
    T_RCYC            rcyc;
    T_RTSK            rtsk;
    ER                result = E_OK;

    TEST_INIT();

    /* S1-U01: the ADC opened and every reading is a 12-bit value. */
    (void)tk_dly_tsk(50);
    subsystem1_diag_get(&before);
    TEST_CHECK("S1-U01", before.b_adc_open &&
               (before.raw[0] <= LINE_ADC_MAX) &&
               (before.raw[1] <= LINE_ADC_MAX) &&
               (before.raw[2] <= LINE_ADC_MAX),
               "ADC open, readings 0..4095");

    /* S1-U02: the 2 ms cyclic handler is running and TSK_LINE exists. */
    result = tk_ref_cyc(line_task_cyclic_id(), &rcyc);
    TEST_CHECK("S1-U02", (E_OK == result) &&
               (0u != (rcyc.cycstat & TCYC_STA)) &&
               (E_OK == tk_ref_tsk(line_task_id(), &rtsk)),
               "cyclic handler running, task exists");

    /* S1-U03 and S1-U04: 1000 / S1_SAMPLE_MS samples per second, none late.
     * S1-U03 fails if the kernel tick is still 10 ms.
     */
    subsystem1_diag_get(&before);
    (void)tk_dly_tsk((TMO)TEST_RATE_WINDOW_MS);
    subsystem1_diag_get(&after);
    {
        uint32_t taken = after.samples - before.samples;

        TEST_CHECK("S1-U03",
                   (taken >= (TEST_RATE_EXPECTED - TEST_RATE_TOLERANCE)) &&
                   (taken <= (TEST_RATE_EXPECTED + TEST_RATE_TOLERANCE)),
                   "sampling every 1 ms (1000 per second)");
        TEST_CHECK("S1-U04", after.overruns == before.overruns,
                   "no late samples in 1 s");
    }

    /* S1-U05: *A* forward at three speeds (6, 10 and 20 samples per narrow
     * bar: about 25, 15 and 7.5 cm/s for a 3 mm bar).
     */
    {
        uint8_t msg      = 0u;
        bool    b_all_ok = true;
        uint8_t narrow   = 0u;
        static uint8_t const speeds[3] = {6u, 10u, 20u};
        uint8_t speed_no = 0u;

        for (speed_no = 0u; speed_no < 3u; speed_no++)
        {
            narrow = speeds[speed_no];

            if (((uint8_t)BARCODE_RESULT_DECODED !=
                 feed_barcode(TEST_PATTERN_A, narrow, narrow, false, false,
                              &msg)) ||
                (SUBSYSTEM1_CMD_LEFT != subsystem1_barcode_cmd(msg)) ||
                subsystem1_barcode_is_reversed(msg))
            {
                b_all_ok = false;
            }
        }

        TEST_CHECK("S1-U05", b_all_ok, "decodes *A* forward = turn left");
    }

    /* S1-U06: *A* read in reverse is still 'A', flagged reversed. */
    {
        uint8_t msg = 0u;

        TEST_CHECK("S1-U06",
                   ((uint8_t)BARCODE_RESULT_DECODED ==
                    feed_barcode(TEST_PATTERN_A, 8u, 8u, true, false, &msg)) &&
                   ('A' == subsystem1_barcode_char(msg)) &&
                   subsystem1_barcode_is_reversed(msg),
                   "decodes *A* in reverse");
    }

    /* S1-U07: a damaged barcode (one wide bar printed narrow) is rejected. */
    {
        uint8_t msg = 0u;

        TEST_CHECK("S1-U07",
                   (uint8_t)BARCODE_RESULT_DECODED !=
                   feed_barcode(TEST_PATTERN_A, 8u, 8u, false, true, &msg),
                   "rejects a damaged barcode");
    }

    /* S1-U08: speed changes by 60% while the barcode passes. */
    {
        uint8_t msg = 0u;

        TEST_CHECK("S1-U08",
                   ((uint8_t)BARCODE_RESULT_DECODED ==
                    feed_barcode(TEST_PATTERN_A, 5u, 8u, false, false,
                                 &msg)) &&
                   ('A' == subsystem1_barcode_char(msg)),
                   "decodes while the speed changes");
    }

    /* S1-U09: error sign convention (positive = steer left).
     * 20 Nov straddle layout: the two track sensors (LINE_CENTRE = left,
     * LINE_RIGHT = right) sit just outside the line edges, so both white
     * is the healthy straight-ahead state.  The arm (LINE_LEFT) is the
     * barcode reader and never steers. */
    {
        line_sense_t      sense;
        line_cal_t        cal;
        subsystem1_line_t line;
        bool              b_ok = true;

        line_sense_init(&sense);
        line_sense_cal_default(&cal);

        /* Line between the sensors: valid, straight. */
        (void)sense_once(&sense, &cal, TEST_RAW_WHITE, TEST_RAW_WHITE,
                         TEST_RAW_WHITE, &line);
        b_ok = b_ok && line.b_valid && (0 == line.error_mm);

        /* Left sensor on the line: the car drifted right, steer left.
         * 17 Nov ramp: starts at 15 mm and grows with travel. */
        (void)sense_once(&sense, &cal, TEST_RAW_WHITE, TEST_RAW_BLACK,
                         TEST_RAW_WHITE, &line);
        b_ok = b_ok && (15 == line.error_mm);

        /* Arm alone on black (a barcode bar): no steering. */
        (void)sense_once(&sense, &cal, TEST_RAW_BLACK, TEST_RAW_WHITE,
                         TEST_RAW_WHITE, &line);
        b_ok = b_ok && line.b_valid && (0 == line.error_mm);

        /* Right sensor on the line: the car drifted left, steer right. */
        (void)sense_once(&sense, &cal, TEST_RAW_WHITE, TEST_RAW_WHITE,
                         TEST_RAW_BLACK, &line);
        b_ok = b_ok && (-15 == line.error_mm);

        /* Both on black: a junction, held straight. */
        (void)sense_once(&sense, &cal, TEST_RAW_WHITE, TEST_RAW_BLACK,
                         TEST_RAW_BLACK, &line);
        b_ok = b_ok && (0 == line.error_mm) && line.b_junction;

        TEST_CHECK("S1-U09", b_ok, "line error sign: + = steer left");
    }

    /* S1-U10: the packed IF-01 word round-trips at the limits. */
    {
        subsystem1_line_t in  = {S1_LINE_ERR_MAX_MM, true, false, true};
        subsystem1_line_t out = {0, false, false, false};
        bool              b_ok = true;

        line_sense_unpack(line_sense_pack(&in), &out);
        b_ok = (in.error_mm == out.error_mm) && out.b_valid &&
               (!out.b_lost) && out.b_junction;

        in.error_mm = -S1_LINE_ERR_MAX_MM;
        in.b_lost   = true;
        line_sense_unpack(line_sense_pack(&in), &out);
        b_ok = b_ok && (in.error_mm == out.error_mm) && out.b_lost;

        TEST_CHECK("S1-U10", b_ok, "line word packs -40..+40 and flags");
    }

    /* S1-U11: 20 Nov straddle layout - both track sensors white is the
     * healthy state (line between them), so 320 ms of it must NOT raise
     * LINE_LOST and start a recovery sweep. */
    {
        line_sense_t      sense;
        line_cal_t        cal;
        subsystem1_line_t line;
        uint8_t           events = 0u;
        uint16_t          count  = 0u;

        line_sense_init(&sense);
        line_sense_cal_default(&cal);

        for (count = 0u; count < TEST_LOST_SAMPLES; count++)
        {
            events |= sense_once(&sense, &cal, TEST_RAW_WHITE,
                                 TEST_RAW_WHITE, TEST_RAW_WHITE, &line);
        }

        TEST_CHECK("S1-U11",
                   (0u == (events & (uint8_t)LINE_EVT_LOST)) &&
                   line.b_valid && (!line.b_lost),
                   "both sensors white 320 ms: valid, no LINE_LOST");
    }

    /* S1-U16: NFR1, worst TSK_LINE sample time <= 0.5 ms (1 MHz timer). */
    {
        subsystem1_diag_t diag;

        subsystem1_diag_get(&diag);
        tm_printf((UB *)"        TSK_LINE worst sample %u us\n",
                  (UINT)diag.exec_max_us);
        TEST_CHECK("S1-U16",
                   (diag.exec_max_us > 0u) && (diag.exec_max_us <= 500u),
                   "TSK_LINE worst-case time <= 0.5 ms (NFR1)");
    }

    TEST_SUMMARY("subsystem1");
}

/*!
 * @brief Expand '*' data '*' into 29 element widths in narrow units.
 */
static void
build_units (uint16_t data_pattern, uint8_t units[])
{
    uint16_t const chars[TEST_CHARS] =
    {
        TEST_PATTERN_STAR, data_pattern, TEST_PATTERN_STAR
    };
    uint8_t char_no = 0u;
    uint8_t elem    = 0u;
    uint8_t index   = 0u;

    for (char_no = 0u; char_no < TEST_CHARS; char_no++)
    {
        for (elem = 0u; elem < TEST_CHAR_ELEMENTS; elem++)
        {
            bool b_wide = (0u != (chars[char_no] &
                                  (1u << (TEST_PATTERN_MSB - elem))));

            units[index] = b_wide ? TEST_WIDE_UNITS : 1u;
            index++;
        }

        if ((char_no + 1u) < TEST_CHARS)
        {
            units[index] = 1u;      /* Narrow inter-character gap. */
            index++;
        }
    }
}

/*!
 * @brief Feed quiet zone, a barcode and quiet zone into a fresh decoder.
 *
 * @param[in]  data_pattern 9-bit Code 39 pattern of the data character.
 * @param[in]  narrow       Samples per narrow unit at the first element.
 * @param[in]  narrow_end   Samples per narrow unit at the last element
 *                          (different from narrow = changing speed).
 * @param[in]  b_reverse    Feed the elements in reverse order.
 * @param[in]  b_corrupt    Print the first wide element as narrow.
 * @param[out] p_msg        Barcode message if decoded.
 *
 * @return The last non-NONE barcode_result_t seen, as a uint8_t.
 */
static uint8_t
feed_barcode (uint16_t data_pattern, uint8_t narrow, uint8_t narrow_end,
              bool b_reverse, bool b_corrupt, uint8_t * p_msg)
{
    barcode_t        bc;
    uint8_t          units[BARCODE_ELEMENTS];
    uint8_t          outcome = (uint8_t)BARCODE_RESULT_NONE;
    uint8_t          elem    = 0u;
    uint16_t         sample  = 0u;
    barcode_result_t result  = BARCODE_RESULT_NONE;
    uint16_t         trail   = (uint16_t)(TEST_QUIET_NARROWS *
                                          (uint16_t)((narrow > narrow_end) ?
                                                     narrow : narrow_end));

    barcode_init(&bc);
    build_units(data_pattern, units);

    if (trail < TEST_QUIET_SAMPLES)
    {
        trail = TEST_QUIET_SAMPLES;
    }

    if (b_corrupt)
    {
        units[1] = 1u;  /* First wide element of '*' becomes narrow. */
    }

    for (sample = 0u; sample < TEST_QUIET_SAMPLES; sample++)
    {
        (void)barcode_step(&bc, false, p_msg);
    }

    for (elem = 0u; elem < BARCODE_ELEMENTS; elem++)
    {
        uint8_t source  = b_reverse ? (uint8_t)(BARCODE_ELEMENTS - 1u - elem)
                                    : elem;
        /* Narrow width moves linearly from narrow to narrow_end. */
        int32_t  scale  = (int32_t)narrow +
                          ((((int32_t)narrow_end - (int32_t)narrow) *
                            (int32_t)elem) / (int32_t)(BARCODE_ELEMENTS - 1u));
        /* Cast: scale is between narrow and narrow_end (1..255). */
        uint16_t length = (uint16_t)((uint32_t)units[source] *
                                     (uint32_t)scale);
        bool     b_bar  = (0u == (elem % 2u));

        for (sample = 0u; sample < length; sample++)
        {
            result = barcode_step(&bc, b_bar, p_msg);

            if (BARCODE_RESULT_NONE != result)
            {
                outcome = (uint8_t)result;
            }
        }
    }

    for (sample = 0u; sample < trail; sample++)
    {
        result = barcode_step(&bc, false, p_msg);

        if (BARCODE_RESULT_NONE != result)
        {
            outcome = (uint8_t)result;
        }
    }

    return (outcome);
}

/*!
 * @brief Run one line-sensing sample with the given raw readings.
 */
static uint8_t
sense_once (line_sense_t * p_sense, line_cal_t const * p_cal,
            uint16_t left, uint16_t centre, uint16_t right,
            subsystem1_line_t * p_line)
{
    uint16_t raw[LINE_SENSORS];
    uint16_t word   = 0u;
    uint8_t  events = 0u;

    raw[LINE_LEFT]   = left;
    raw[LINE_CENTRE] = centre;
    raw[LINE_RIGHT]  = right;

    events = line_sense_update(p_sense, p_cal, raw, &word);
    line_sense_unpack(word, p_line);

    return (events);
}

#endif /* APP_TEST */

/*** end of file ***/
