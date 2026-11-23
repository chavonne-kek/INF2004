/** @file line_task.c
 *
 * @brief TSK_LINE (priority 12, core 1): every 1 ms a cyclic handler wakes
 *        the task, which reads the three IR sensors through the kernel ADC
 *        driver, publishes the packed line state, runs the barcode decoder
 *        on the right-hand sensor and reports events.
 *
 * Shared data and why it is safe without a mutex (Change Requirement 1):
 *   g_line_word   One aligned 16-bit word.  Only this task writes it, with
 *                 a single store; a reader on any core gets either the old
 *                 or the new word, never a mix.  This replaces mtx_line.
 *   g_cal_command Written only by the caller of the calibration functions.
 *   g_cal_status  Written only by this task.
 *   g_diag        Counters for tests and telemetry, written only by this
 *                 task; readers accept that a counter may be one sample old.
 *
 * NOTE: the 1 ms period needs the kernel tick CNF_TIMER_PERIOD = 1 ms
 *       (config/config.h, Week 6 assumption A5).  With the template's 10 ms
 *       tick the task only runs every 10 ms, which is too slow to read the
 *       3 mm barcode bars; subsystem1_init() prints a warning.
 */

#include "line_task.h"
#include "line_sense.h"
#include "barcode.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem5/subsystem5.h"   /* 18 Nov: barcode trace (IF-06) */

/* Kernel object settings. */
#define LINE_ADC_DEVICE        "adca"  /* Kernel ADC driver, unit a.      */
#define LINE_MBF_MESSAGES      8u      /* Barcode messages buffered.      */
#define LINE_MBF_MSG_BYTES     8u      /* 1 data byte + header, aligned. */
#define LINE_ADC_READ_UNITS    1       /* One conversion per read.        */
#define LINE_ADC_AVERAGE       2u      /* Conversions averaged per sample
                                        * (2 Nov): halves the random part of
                                        * motor noise on the IR readings.  */

/* Calibration handshake values. */
#define LINE_CAL_IDLE          0u
#define LINE_CAL_BEGIN         1u
#define LINE_CAL_END           2u
#define LINE_CAL_END_BARCODE   3u      /* Barcode sensor only (bench).   */
#define LINE_CAL_RUNNING       1u
#define LINE_CAL_DONE_OK       2u
#define LINE_CAL_DONE_FAIL     3u
#define LINE_CAL_WAIT_TRIES    10u     /* Wait up to 10 samples for the task. */

/* Print decoded barcodes and line events on the console (1 = on). */
#define LINE_PRINT_EVENTS      1

/* Optional GPIO marker for timing on a scope: high while a sample is being
 * processed.  Claim the pin in TEAM.md before setting this to 1.
 */
#define LINE_TIMING_PIN_ENABLE 0
#define LINE_TIMING_PIN        19u

#define LINE_COUNT_MAX         0xFFFFu

static ID gh_line_task   = 0;
static ID gh_line_cyc    = 0;
static ID gh_barcode_mbf = 0;
static ID gh_adc_dev     = 0;
static volatile ID gh_event_flag = 0;

static volatile uint16_t g_line_word   = 0u;

/* 18 Nov (obstacle rejoin): a debounced "the line was seen" counter.  It
 * goes up by one each time some sensor has seen the line for
 * LINE_SEEN_RUN_MS in a row, so a reader can tell whether the line was
 * crossed while it was busy (a 5 cm move) - the line word alone only says
 * what the sensors see now.  One noisy sample does not count. */
#define LINE_SEEN_RUN_MS        20u
static volatile uint32_t g_line_seen_count = 0u;
static uint16_t          g_line_valid_run  = 0u;

/* 18 Nov: which side the line is on, from the obstacle engine after a
 * bypass (0 = none).  Taken by this task at its next sample. */
static volatile int8_t   g_line_hint_mm    = 0;
static volatile bool     g_arm_black       = false;
static volatile bool     g_track_black     = false;
static volatile bool     g_both_black      = false;
/* 20 Nov (night): each track sensor on its own, for the junction pivot
 * (TSK_CTRL stops the pivot on the LEADING sensor). */
static volatile bool     g_left_black      = false;   /* GP27, left of line  */
static volatile bool     g_right_black     = false;   /* GP26, right of line */
/* 20 Nov (night): TSK_CTRL asks for the barcode reader to be cleared; this
 * task does it at its next sample, so two tasks never write g_barcode. */
static volatile bool     g_bc_reset_req    = false;
static volatile bool     g_b_quiet_line = false;
/* Last raw IR sample (arm, centre, right), read by comms for telemetry. */
static volatile uint16_t g_ir_raw[LINE_SENSORS] = {0u, 0u, 0u};
static volatile uint8_t  g_cal_command = LINE_CAL_IDLE;
static volatile uint8_t  g_cal_status  = LINE_CAL_IDLE;
static volatile subsystem1_diag_t g_diag;

/* Used only by the line task itself. */
static line_sense_t   g_sense;
static line_cal_t     g_cal;
static line_cal_run_t g_cal_run;
static barcode_t      g_barcode;

static void     tick_isr(void * p_exinf);
static void     sample_thread(INT stacd, void * p_exinf);
static bool     read_sensors(uint16_t raw[LINE_SENSORS]);
static void     handle_calibration(uint16_t const raw[LINE_SENSORS]);
static uint8_t  run_barcode(void);
#if S1_TRACK_RIGHT_EDGE
static uint16_t mirror_word(uint16_t word);
#endif
static void     report(uint8_t events);
static uint16_t count_up(uint16_t count, uint16_t amount);
static uint32_t now_us(void);

static uint32_t g_rate_t0 = 0u;   /* Scan-rate window start, us.        */
static uint16_t g_rate_n  = 0u;   /* Samples since the window opened.   */
static uint8_t  travel_mm(void);
static uint16_t arm_travel_16th(void);
static void     post_trace(uint8_t result_msg, uint8_t count);

/* 18 Nov: darkest / lightest filtered arm reading (0..1000) during the
 * barcode read in progress, sent with its trace. */
static uint16_t g_bc_hi  = 0u;
static uint16_t g_bc_lo  = 0u;
static uint8_t  g_bc_seq = 0u;


/*!
 * @brief Create the cyclic handler and the barcode message buffer.
 *
 * @return E_OK, or the first kernel error.
 */
ER
line_task_init (void)
{
    T_CCYC ccyc   = {0};
    T_CMBF cmbf   = {0};
    ER     result = E_OK;

    line_sense_init(&g_sense);
    line_sense_cal_default(&g_cal);
    barcode_init(&g_barcode);

    /* The cyclic handler is created stopped; subsystem1_start() starts it
     * once the task it wakes exists.
     */
    ccyc.cycatr = TA_HLNG;
    ccyc.cychdr = (FP)tick_isr;     /* Kernel handler type for C code. */
    ccyc.cyctim = S1_SAMPLE_MS;
    ccyc.cycphs = S1_SAMPLE_MS;
    gh_line_cyc = tk_cre_cyc(&ccyc);

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = (SZ)(LINE_MBF_MESSAGES * LINE_MBF_MSG_BYTES);
    cmbf.maxmsz = (INT)S1_BC_MSG_SIZE;
    gh_barcode_mbf = tk_cre_mbf(&cmbf);

    if (gh_line_cyc < E_OK)
    {
        result = (ER)gh_line_cyc;
    }
    else if (gh_barcode_mbf < E_OK)
    {
        result = (ER)gh_barcode_mbf;
    }
    else
    {
        result = E_OK;
    }

    return (result);
}

/*!
 * @brief Create and start TSK_LINE on core 1, then start the 1 ms tick.
 */
ER
line_task_start (void)
{
    ER result = E_OK;

    gh_line_task = app_start_task((FP)sample_thread, S1_PRI_LINE, TP_PRC2,
                                  "line_task");  /* FP: kernel entry type. */

    if (gh_line_task < E_OK)
    {
        result = (ER)gh_line_task;
    }
    else
    {
        result = tk_sta_cyc(gh_line_cyc);
    }

    return (result);
}

/*!
 * @brief Remember the controller's event flag, so events are reported.
 */
void
line_task_bind_events (ID flg_id)
{
    gh_event_flag = flg_id;
}

/*!
 * @brief Latest packed line-state word (one 16-bit load).
 */
uint16_t
line_task_word (void)
{
    return (g_line_word);
}

/*!
 * @brief Debounced count of "the line was seen" (18 Nov): +1 each time a
 *        sensor has seen the line for LINE_SEEN_RUN_MS in a row.
 */
uint32_t
subsystem1_line_seen_count (void)
{
    return (g_line_seen_count);
}

/*!
 * @brief True while a sensor has seen the line for LINE_SEEN_RUN_MS in a
 *        row (debounced "on the line now", 18 Nov).
 */
bool
subsystem1_line_on (void)
{
    return (g_line_valid_run >= LINE_SEEN_RUN_MS);
}

/*!
 * @brief Tell the line follower which side the line is on if it loses it
 *        before it has seen it properly (18 Nov, after an obstacle
 *        bypass): positive = left, negative = right (as error_mm).
 */
void
subsystem1_line_bias (int8_t bias_mm)
{
    line_sense_bias(bias_mm);
}

bool
subsystem1_track_black (void)
{
    return (g_track_black);
}

bool
subsystem1_track_both_black (void)
{
    return (g_both_black);
}

bool
subsystem1_arm_black (void)
{
    return (g_arm_black);
}

void
subsystem1_line_hold (bool b_on)
{
    line_sense_hold(b_on);
}

void
subsystem1_line_hint (int8_t error_mm)
{
    g_line_hint_mm = error_mm;
}

/*!
 * @brief Copy the last raw IR sample (arm, centre, right) for telemetry.
 *
 * @param[out] out Three raw ADC values; 0 while the task is not sampling.
 */
void
subsystem1_line_raw (uint16_t out[3])
{
    /* g_ir_raw is in GPIO order (GP26 right, GP27 centre, GP28 arm); the
     * telemetry order is arm, centre, right (2 Nov: was sent in GPIO
     * order, so "arm" showed the right sensor). */
    out[0] = g_ir_raw[LINE_LEFT];
    out[1] = g_ir_raw[LINE_CENTRE];
    out[2] = g_ir_raw[LINE_RIGHT];
}

/*!
 * @brief Message buffer that receives one byte per decoded barcode.
 */
ID
line_task_barcode_mbf (void)
{
    return (gh_barcode_mbf);
}

/*!
 * @brief Task ID of TSK_LINE (for tests).
 */
ID
line_task_id (void)
{
    return (gh_line_task);
}

/*!
 * @brief ID of the 1 ms cyclic handler (for tests).
 */
ID
line_task_cyclic_id (void)
{
    return (gh_line_cyc);
}

/*!
 * @brief Start a calibration sweep: move all three sensors across both
 *        the white floor and the black line until line_task_cal_end().
 */
void
line_task_cal_begin (void)
{
    g_cal_command = LINE_CAL_BEGIN;
}

/*!
 * @brief End the calibration sweep and apply it if every sensor saw both
 *        white and black.  Waits up to 10 samples for TSK_LINE.
 *
 * @return True if the new calibration was applied.
 */
bool
line_task_cal_end (void)
{
    uint8_t tries = 0u;

    g_cal_command = LINE_CAL_END;

    for (tries = 0u; tries < LINE_CAL_WAIT_TRIES; tries++)
    {
        if (LINE_CAL_RUNNING != g_cal_status)
        {
            break;
        }

        (void)tk_dly_tsk((TMO)S1_SAMPLE_MS);
    }

    return (LINE_CAL_DONE_OK == g_cal_status);
}

/*!
 * @brief End the calibration sweep for the barcode sensor only (barcode
 *        bench image, CHECK=6).
 *
 * @return True if the barcode sensor saw both white and black.
 */
bool
line_task_cal_end_barcode (void)
{
    uint8_t tries = 0u;

    g_cal_command = LINE_CAL_END_BARCODE;

    for (tries = 0u; tries < LINE_CAL_WAIT_TRIES; tries++)
    {
        if (LINE_CAL_RUNNING != g_cal_status)
        {
            break;
        }

        (void)tk_dly_tsk((TMO)S1_SAMPLE_MS);
    }

    return (LINE_CAL_DONE_OK == g_cal_status);
}

/*!
 * @brief Mute LINE_LOST / FOUND / JUNCTION on the console (barcode bench:
 *        the two line inputs are not connected and float).
 */
uint8_t
line_task_barcode_pin (void)
{
    return ((uint8_t)(26u + g_sense.bc_sensor));   /* Index 0..2 = GP26..28. */
}

/*!
 * @brief Mute LINE_LOST / FOUND / JUNCTION (see below).
 */
void
line_task_quiet_line (bool b_quiet)
{
    g_b_quiet_line = b_quiet;
}

/*!
 * @brief Copy the diagnostic counters.
 */
void
line_task_diag (subsystem1_diag_t * p_diag)
{
    uint8_t sensor = 0u;

    for (sensor = 0u; sensor < LINE_SENSORS; sensor++)
    {
        p_diag->raw[sensor]      = g_diag.raw[sensor];
        p_diag->cal_span[sensor] = g_diag.cal_span[sensor];
    }

    p_diag->samples        = g_diag.samples;
    p_diag->overruns       = g_diag.overruns;
    p_diag->adc_errors     = g_diag.adc_errors;
    p_diag->barcodes       = g_diag.barcodes;
    p_diag->barcode_errors = g_diag.barcode_errors;
    p_diag->mbf_drops      = g_diag.mbf_drops;
    p_diag->exec_max_us    = g_diag.exec_max_us;
    p_diag->b_adc_open     = g_diag.b_adc_open;
    p_diag->b_calibrated   = g_diag.b_calibrated;
}

/*!
 * @brief Cyclic handler, every 1 ms.  Runs in interrupt context: it only
 *        wakes the task and must never wait.
 */
static void
tick_isr (void * p_exinf)
{
    (void)p_exinf;
    (void)tk_wup_tsk(gh_line_task);
}

/*!
 * @brief TSK_LINE body.
 */
static void
sample_thread (INT stacd, void * p_exinf)
{
    uint16_t raw[LINE_SENSORS] = {0u, 0u, 0u};

    (void)stacd;
    (void)p_exinf;

    gh_adc_dev = tk_opn_dev((UB const *)LINE_ADC_DEVICE, TD_READ);
    g_diag.b_adc_open = (gh_adc_dev >= E_OK);

    if (!g_diag.b_adc_open)
    {
        tm_printf((UB *)"S1: cannot open ADC device (%d)\n", gh_adc_dev);
    }

#if LINE_TIMING_PIN_ENABLE
    gpio_set_pin(LINE_TIMING_PIN, GPIO_MODE_OUT);
#endif

    for (;;)
    {
        uint8_t  events = 0u;
        INT      missed = 0;
        uint16_t word   = 0u;
        uint32_t start  = 0u;
        uint32_t spent  = 0u;

        (void)tk_slp_tsk(TMO_FEVR);
        start = now_us();

        /* Wake-ups still queued mean samples were late: count, then drop
         * them, so the task does not run several samples back to back.
         */
        missed = tk_can_wup(TSK_SELF);

        if (missed > 0)
        {
            g_diag.overruns = count_up(g_diag.overruns, (uint16_t)missed);
        }

#if LINE_TIMING_PIN_ENABLE
        gpio_set_val(LINE_TIMING_PIN, 1);
#endif

        if (read_sensors(raw))
        {
            line_sense_travel(&g_sense, travel_mm());
            handle_calibration(raw);
            g_ir_raw[0] = raw[0];
            g_ir_raw[1] = raw[1];
            g_ir_raw[2] = raw[2];
            if (0 != g_line_hint_mm)
            {
                /* The side to search if the line is lost (18 Nov). */
#if S1_TRACK_RIGHT_EDGE
                g_sense.last_error_mm = (int8_t)(-g_line_hint_mm);
#else
                g_sense.last_error_mm = g_line_hint_mm;
#endif
                g_line_hint_mm = 0;
            }

            events      = line_sense_update(&g_sense, &g_cal, raw, &word);
            /* The packed word has no room for the arm, so publish it apart:
             * the ctrl waits for the arm to reach the branch before turning. */
            g_arm_black   = g_sense.b_black[LINE_LEFT];
            g_track_black = (g_sense.b_black[LINE_CENTRE] ||
                             g_sense.b_black[LINE_RIGHT]);
            g_both_black  = (g_sense.b_black[LINE_CENTRE] &&
                             g_sense.b_black[LINE_RIGHT]);
            g_left_black  = g_sense.b_black[LINE_CENTRE];
            g_right_black = g_sense.b_black[LINE_RIGHT];
#if S1_TRACK_RIGHT_EDGE
            word        = mirror_word(word);
#endif
            g_line_word = word;     /* Single 16-bit store (IF-01). */

            /* 20 Nov (night): with the sensors outside the line, "valid" is
             * always true (both white = on the line), so it no longer says
             * a sensor is OVER the line.  Count a track sensor on black
             * instead: that is what the obstacle bypass needs to see when it
             * crosses the line on the way back. */
            if (g_track_black)
            {
                if (g_line_valid_run < LINE_SEEN_RUN_MS)
                {
                    g_line_valid_run++;

                    if (LINE_SEEN_RUN_MS == g_line_valid_run)
                    {
                        g_line_seen_count++;    /* Only this task writes. */
                    }
                }
            }
            else
            {
                g_line_valid_run = 0u;
            }
            if (g_bc_reset_req)
            {
                barcode_init(&g_barcode);
                g_bc_reset_req = false;
            }
            events     |= run_barcode();
            report(events);
        }
        else
        {
            g_diag.adc_errors = count_up(g_diag.adc_errors, 1u);
        }

        g_diag.samples++;

        /* NFR1 evidence: worst execution time of one 1 ms sample. */
        spent = now_us() - start;   /* Unsigned: correct across the wrap. */

        /* 19 Nov: report the ACTUAL scan rate. S1_SAMPLE_MS says 1 ms,
         * but if a sample runs long the kernel coalesces the wakes and the
         * real rate is lower - which is the difference between "the sensor
         * misses thin bars" and "the loop is too slow to see them". */
        g_rate_n++;
        if (g_rate_n >= 1000u)
        {
            uint32_t dt = now_us() - g_rate_t0;

            if (dt > 0u)
            {
                tm_printf((UB *)"S1: scan %u Hz, overruns %u, exec max %u us\n",
                          (UINT)(1000000000u / dt), (UINT)g_diag.overruns,
                          (UINT)g_diag.exec_max_us);
            }

            g_rate_t0       = now_us();
            g_rate_n        = 0u;
            g_diag.overruns = 0u;
            g_diag.exec_max_us = 0u;
        }

        if (spent > g_diag.exec_max_us)
        {
            g_diag.exec_max_us = (uint16_t)((spent > 0xFFFFu) ? 0xFFFFu
                                                              : spent);
        }

#if LINE_TIMING_PIN_ENABLE
        gpio_set_val(LINE_TIMING_PIN, 0);
#endif
    }
}

/*!
 * @brief Read ADC0..2 (left, centre, right).
 *
 * @return True if all three conversions succeeded.
 */
static bool
read_sensors (uint16_t raw[LINE_SENSORS])
{
    static W const channel[LINE_SENSORS] =
    {
        (W)S1_ADC_CH_LEFT, (W)S1_ADC_CH_CENTRE, (W)S1_ADC_CH_RIGHT
    };
    bool    b_ok   = g_diag.b_adc_open;
    uint8_t sensor = 0u;

    for (sensor = 0u; (sensor < LINE_SENSORS) && b_ok; sensor++)
    {
        UW      sum  = 0u;
        uint8_t read = 0u;

        for (read = 0u; (read < LINE_ADC_AVERAGE) && b_ok; read++)
        {
            UW value = 0u;
            SZ asize = 0;
            ER result = tk_srea_dev(gh_adc_dev, channel[sensor], &value,
                                    (SZ)LINE_ADC_READ_UNITS, &asize);

            if ((E_OK != result) || ((SZ)LINE_ADC_READ_UNITS != asize) ||
                (value > LINE_ADC_MAX))
            {
                b_ok = false;
            }
            else
            {
                sum += value;
            }
        }

        if (b_ok)
        {
            /* The average of 12-bit values is a 12-bit value. */
            raw[sensor]        = (uint16_t)(sum / LINE_ADC_AVERAGE);
            g_diag.raw[sensor] = raw[sensor];
        }
    }

    return (b_ok);
}

/*!
 * @brief Follow the calibration commands from line_task_cal_begin/end().
 */
static void
handle_calibration (uint16_t const raw[LINE_SENSORS])
{
    uint8_t command = g_cal_command;

    if ((LINE_CAL_BEGIN == command) && (LINE_CAL_RUNNING != g_cal_status))
    {
        line_sense_cal_start(&g_cal_run);
        g_cal_status = LINE_CAL_RUNNING;
    }
    else if (((LINE_CAL_END == command) || (LINE_CAL_END_BARCODE == command)) &&
             (LINE_CAL_RUNNING == g_cal_status))
    {
        uint8_t sensor = 0u;

        for (sensor = 0u; sensor < LINE_SENSORS; sensor++)
        {
            g_diag.cal_span[sensor] =
                (g_cal_run.highest[sensor] >= g_cal_run.lowest[sensor]) ?
                (uint16_t)(g_cal_run.highest[sensor] -
                           g_cal_run.lowest[sensor]) : 0u;
        }

        if (LINE_CAL_END_BARCODE == command)
        {
            /* Bench: use whichever input moved most (GP26, GP27 or GP28),
             * so the sensor works on any analogue pin or Grove port. */
            uint8_t best = LINE_BARCODE;

            for (sensor = 0u; sensor < LINE_SENSORS; sensor++)
            {
                if (g_diag.cal_span[sensor] > g_diag.cal_span[best])
                {
                    best = sensor;
                }
            }

            g_sense.bc_sensor = best;
        }

        if ((LINE_CAL_END_BARCODE == command)
            ? line_sense_cal_apply_one(&g_cal_run, &g_cal, g_sense.bc_sensor)
            : line_sense_cal_apply(&g_cal_run, &g_cal))
        {
            g_cal_status        = LINE_CAL_DONE_OK;
            g_diag.b_calibrated = true;
        }
        else
        {
            g_cal_status = LINE_CAL_DONE_FAIL;
        }
    }
    else
    {
        /* No change of calibration state this sample. */
    }

    if (LINE_CAL_RUNNING == g_cal_status)
    {
        line_sense_cal_track(&g_cal_run, raw);
    }
}

#if S1_TRACK_RIGHT_EDGE
/*!
 * @brief Right-edge tracking: the same logic as left-edge tracking seen
 *        in a mirror, so only the sign of the error changes (positive is
 *        still "steer left" for TSK_MOTION).
 */
static uint16_t
mirror_word (uint16_t word)
{
    subsystem1_line_t line;

    line_sense_unpack(word, &line);
    line.error_mm = (int8_t)(-line.error_mm);

    return (line_sense_pack(&line));
}
#endif

/*!
 * @brief Feed the right-hand sensor into the barcode decoder and send any
 *        decoded barcode to the message buffer without waiting.
 *
 * @return S1_EVT_BARCODE if a barcode was decoded, else 0.
 */
static uint8_t
run_barcode (void)
{
    uint8_t          events = 0u;
    uint8_t          msg    = 0u;
    barcode_result_t result = barcode_step_travel(&g_barcode,
                                                  g_sense.b_bc_black,
                                                  arm_travel_16th(),
                                                  &msg);

    /* Arm level range of the read in progress (for its trace). */
    if (BARCODE_CAPTURE == (barcode_state_t)g_barcode.state)
    {
        if (g_sense.bc_level > g_bc_hi)
        {
            g_bc_hi = g_sense.bc_level;
        }
        if (g_sense.bc_level < g_bc_lo)
        {
            g_bc_lo = g_sense.bc_level;
        }
    }

    if (BARCODE_RESULT_DECODED == result)
    {
        g_diag.barcodes = count_up(g_diag.barcodes, 1u);
        events = (uint8_t)S1_EVT_BARCODE;   /* Bit value fits. */

        if (E_OK != tk_snd_mbf(gh_barcode_mbf, &msg, (INT)S1_BC_MSG_SIZE,
                               TMO_POL))
        {
            g_diag.mbf_drops = count_up(g_diag.mbf_drops, 1u);
        }

#if LINE_PRINT_EVENTS
        tm_printf((UB *)"S1: barcode '%c'%s (%s widths, arm %u..%u of 1000)\n",
                  barcode_index_to_char(msg & S1_BC_INDEX_MASK),
                  (0u != (msg & S1_BC_REVERSED_BIT)) ? " (reversed)" : "",
                  g_barcode.b_used_travel ? "travel" : "time",
                  (UINT)g_bc_lo, (UINT)g_bc_hi);
#endif
        post_trace(msg, 29u);
    }
    else if (BARCODE_RESULT_REJECTED == result)
    {
        g_diag.barcode_errors = count_up(g_diag.barcode_errors, 1u);

#if LINE_PRINT_EVENTS
        /* r: 1 too many elements, 2 no wide/narrow split, 3 not Code 39,
         * 4 black too long (the line, not a bar), 5 ended before 29. */
        /* 21 Nov: ONE call.  This runs in the 1 kHz sampling task: the
         * loop of up to 29 tm_printf calls stalled line sensing right after
         * a barcode - just before the junction it announces.  The full
         * widths still reach the dashboard (post_trace, bctrace). */
        tm_printf((UB *)"S1: BC reject r=%u n=%u min=%u w=%u %u %u %u %u "
                  "%u %u %u %u %u\n",
                  g_barcode.reason, g_barcode.report_count,
                  g_barcode.report_min, g_barcode.report[0],
                  g_barcode.report[1], g_barcode.report[2],
                  g_barcode.report[3], g_barcode.report[4],
                  g_barcode.report[5], g_barcode.report[6],
                  g_barcode.report[7], g_barcode.report[8],
                  g_barcode.report[9]);
#endif

        /* Bit 7 marks a diagnostic, not a character: the same path reaches
         * the console AND robot/telemetry/barcode, so a reject can be read
         * off MQTT without the USB cable (2 Nov). */
        /* At most one diagnostic every 250 ms: while the arm sits over a
         * dark area the decoder can reject every sample, and the
         * 8-deep telemetry mailbox then overflows (drop 184 in 20 s on
         * 2 Nov) which starves the periodic telemetry down to 1 Hz. */
        static uint16_t s_reject_gap = 0u;

        if (0u != s_reject_gap)
        {
            s_reject_gap--;
            return (events);
        }

        s_reject_gap = (uint16_t)(250u / S1_SAMPLE_MS);    /* 250 ms */

        /* bit 7 reject, bits 6..3 element count (capped at 15), bits 2..0
         * reason.  The count is also how a label that is not Code 39 shows
         * up: a real one captures 29 elements. */
        {
            uint8_t n = g_barcode.report_count;

            if (n > 15u)
            {
                n = 15u;
            }

            msg = (uint8_t)(S1_BC_REJECT_BIT |
                            (uint8_t)(n << 3) |
                            (g_barcode.reason & 0x07u));
        }

        if (E_OK != tk_snd_mbf(gh_barcode_mbf, &msg, (INT)S1_BC_MSG_SIZE,
                               TMO_POL))
        {
            g_diag.mbf_drops = count_up(g_diag.mbf_drops, 1u);
        }

        /* 18 Nov: what the sensor saw, for the dashboard (not for a reject
         * of a few elements: that is the arm over the line or a stray mark). */
        if (g_barcode.report_count >= 4u)
        {
            post_trace(msg, g_barcode.report_count);
        }

        events = (uint8_t)S1_EVT_BARCODE;
    }
    else
    {
        /* Nothing finished this sample. */
    }

    if ((BARCODE_RESULT_NONE != result) ||
        (BARCODE_CAPTURE != (barcode_state_t)g_barcode.state))
    {
        g_bc_hi = g_sense.bc_level;     /* Next read starts from here. */
        g_bc_lo = g_sense.bc_level;
    }

    return (events);
}

/*!
 * @brief How far the barcode (arm) sensor moved this sample, 1/16 mm.
 *
 * 18 Nov.  The arm sensor is 40 mm left of the centre sensor and the wheels
 * are 120 mm apart, so (centre on the car's middle) it moves at 5/6 of the
 * left wheel's speed plus 1/6 of the right one's: when the car steers, the
 * arm speeds up and slows down although the car's own speed does not.  One
 * encoder edge is pi x 65 mm / 1270 = 0.161 mm; (5 dL + dR) / 6 edges x
 * 0.161 mm x 16 = (5 dL + dR) x 0.429.  Only ratios of widths matter, so
 * the exact scale does not.
 */
static uint16_t
arm_travel_16th (void)
{
    static uint16_t last_left  = 0u;
    static uint16_t last_right = 0u;
    static bool     b_have     = false;
    static uint32_t acc        = 0u;    /* Thousandths carried over.  */
    uint16_t        left       = 0u;
    uint16_t        right      = 0u;
    uint32_t        d_left     = 0u;
    uint32_t        d_right    = 0u;
    uint32_t        units      = 0u;

    subsystem2_encoder_counts(&left, &right);

    if (b_have)
    {
        d_left  = (uint16_t)(left - last_left);     /* Wrap-safe.      */
        d_right = (uint16_t)(right - last_right);

        if ((d_left > 50u) || (d_right > 50u))
        {
            d_left  = 0u;   /* Not possible in 1 ms: a counter reset. */
            d_right = 0u;
        }

        acc  += ((5u * d_left) + d_right) * 429u;
        units = acc / 1000u;
        acc  -= units * 1000u;
    }

    last_left  = left;
    last_right = right;
    b_have     = true;

    return ((uint16_t)units);   /* <= 300 x 429 / 1000: fits. */
}

/*!
 * @brief Send the last read's element widths to the dashboard
 *        (robot/telemetry/bctrace, 18 Nov): up to 5 parts of 8 bytes.
 *
 * Part 0: [seq<<4 | 0, count, result, flags, unit, hi/4, lo/4, w0]
 * Part k: [seq<<4 | k, w(7k-6) .. w(7k)]
 * result: the robot/telemetry/barcode byte; flags bit 0: widths are
 * travel (unit x 1/16 mm), else time (unit x 1 ms); w = width / unit,
 * rounded, the first element is black and they alternate.
 */
static void
post_trace (uint8_t result_msg, uint8_t count)
{
    uint16_t const * p_w     = g_barcode.widths;
    uint32_t         total   = 0u;
    uint16_t         widest  = 0u;
    uint16_t         unit    = 1u;
    uint8_t          n       = (count > BARCODE_ELEMENTS) ?
                               (uint8_t)BARCODE_ELEMENTS : count;
    uint8_t          i       = 0u;
    uint8_t          part    = 0u;
    uint8_t          rec[8];
    bool             b_trav  = false;

    for (i = 0u; i < n; i++)
    {
        total += g_barcode.travel[i];
    }

    b_trav = (total >= (20u * 16u));    /* The car moved: show it in mm. */
    p_w    = b_trav ? g_barcode.travel : g_barcode.widths;

    for (i = 0u; i < n; i++)
    {
        widest = (p_w[i] > widest) ? p_w[i] : widest;
    }

    unit = (uint16_t)((widest + 254u) / 255u);
    unit = (unit < 1u) ? 1u : ((unit > 255u) ? 255u : unit);
    g_bc_seq = (uint8_t)((g_bc_seq + 1u) & 0x0Fu);

    for (part = 0u; (part < 5u) && ((0u == part) || ((7u * part) - 6u < n));
         part++)
    {
        uint8_t first = (0u == part) ? 0u : (uint8_t)((7u * part) - 6u);
        uint8_t slot  = (0u == part) ? 7u : 1u;
        uint8_t k     = 0u;

        for (k = 0u; k < 8u; k++)
        {
            rec[k] = 0u;
        }

        rec[0] = (uint8_t)((g_bc_seq << 4) | part);

        if (0u == part)
        {
            rec[1] = n;
            rec[2] = result_msg;
            rec[3] = b_trav ? 1u : 0u;
            rec[4] = (uint8_t)unit;
            rec[5] = (uint8_t)(g_bc_hi / 4u);
            rec[6] = (uint8_t)(g_bc_lo / 4u);
        }

        for (k = 0u; (slot < 8u) && ((first + k) < n); k++, slot++)
        {
            uint32_t w = ((uint32_t)p_w[first + k] + (unit / 2u)) / unit;

            rec[slot] = (uint8_t)((w > 255u) ? 255u : w);
        }

        (void)subsystem5_post(S5_TLM_BCTRACE, rec, 8u);
    }
}

/*!
 * @brief Set the event bits in the controller's event flag, if bound.
 */
static void
report (uint8_t events)
{
    ID flag = gh_event_flag;

    if ((0u != events) && (flag > 0))
    {
        (void)tk_set_flg(flag, (UINT)events);
    }

#if LINE_PRINT_EVENTS
    if (g_b_quiet_line)
    {
        return;
    }

    if (0u != (events & (uint8_t)LINE_EVT_LOST))
    {
        tm_printf((UB *)"S1: LINE_LOST\n");
    }

    if (0u != (events & (uint8_t)LINE_EVT_FOUND))
    {
        tm_printf((UB *)"S1: LINE_FOUND\n");
    }

    if (0u != (events & (uint8_t)LINE_EVT_JUNCTION))
    {
        tm_printf((UB *)"S1: JUNCTION\n");
    }
#endif
}

/*!
 * @brief Add to a 16-bit counter without wrapping.
 */
static uint16_t
count_up (uint16_t count, uint16_t amount)
{
    uint16_t result = LINE_COUNT_MAX;

    if (count <= (uint16_t)(LINE_COUNT_MAX - amount))
    {
        result = (uint16_t)(count + amount);
    }

    return (result);
}

/*!
 * @brief RP2040 1 MHz timer, low 32 bits (execution-time measurement;
 *        reads 0 if the timer is not running).
 */
static uint32_t
now_us (void)
{
    return ((uint32_t)in_w(TIMER_TIMERAWL));
}

/*!
 * @brief Distance since the previous sample from subsystem 2's odometry
 *        word (IF-02, one 32-bit read), for the right-branch junction.
 */
static uint8_t
travel_mm (void)
{
    static uint16_t   last_mm = 0u;
    static bool       b_have  = false;
    subsystem2_odom_t odom;
    int16_t           delta   = 0;
    uint8_t           result  = 0u;

    subsystem2_odometry_read(&odom);

    if (b_have)
    {
        delta  = subsystem2_distance_delta(odom.distance_mm, last_mm);
        delta  = (delta < 0) ? (int16_t)(-delta) : delta;
        result = (uint8_t)((delta > 255) ? 255 : delta);
    }

    last_mm = odom.distance_mm;
    b_have  = true;

    return (result);
}

/*** end of file ***/

/*!
 * @brief Drop any half-finished barcode read.
 *
 *        A read only clears itself when it decodes, is rejected, or sees a
 *        gap long enough to time out.  If the car stops mid-label (PAUSED,
 *        FINISHED, or an aborted run) the capture can stay open forever and
 *        every later pass feeds bars into a read that never closes - the car
 *        log showed 30 black/white transitions during a run with no barcode
 *        event at all (19 Nov).  TSK_CTRL calls this whenever a drive
 *        starts, so each pass begins from WAIT_QUIET.
 */
void
subsystem1_barcode_reset (void)
{
    /* 20 Nov (night): was barcode_init(&g_barcode) straight from TSK_CTRL,
     * while this task could be in the middle of barcode_step() on it. */
    g_bc_reset_req = true;
}

/*!
 * @brief Left track sensor (GP27, just left of the line) on black.
 */
bool
subsystem1_left_black (void)
{
    return (g_left_black);
}

/*!
 * @brief Right track sensor (GP26, just right of the line) on black.
 */
bool
subsystem1_right_black (void)
{
    return (g_right_black);
}
