/** @file line_sense.c
 *
 * @brief Line sensing for left-edge tracking (decision logged at
 *        Tribunal 1).
 *
 * Sensor layout as built (1 Nov), looking down, robot moving up the page,
 * and the track from the project specification (18 mm line, barcode
 * 20 mm beside it, about 141 mm long, 3 mm narrow / 9 mm wide elements):
 *
 *     arm (GP28)          centre (GP27)            right (GP26)
 *     35-45 mm left  <--  on the line's  -->  just past the line's
 *     over barcodes       LEFT edge           right edge (on white)
 *
 *   - The centre sensor sits on the left edge of the 18 mm line, so its
 *     analogue reading is about half black.  More black means the robot
 *     has drifted right (steer left, positive error); more white means it
 *     has drifted left (steer right, negative error).  The error therefore
 *     always has a sign, and the centre alone steers on straights, curves
 *     and while a barcode passes under the arm.
 *   - The arm reads the barcodes (LINE_BARCODE), and is also the far-left
 *     guard: it only steers once the centre has lost the line (the robot
 *     drifted right past the whole line), and only after 12 mm of
 *     unbroken black, longer than any bar (9 mm).  1 Nov: it used to be
 *     checked BEFORE the centre, on any black, so every bar of a barcode
 *     forced a hard left turn and the robot could swerve off the line.
 *   - The right sensor is the near-right guard: if only it sees black,
 *     the robot has drifted left off the edge (or a sharp right curve).
 *   - With no sensor on the line, the last clear error sign is held so the
 *     robot keeps turning toward where the line was; LINE_LOST is raised
 *     only after 300 ms of this.
 *   - JUNCTION (Week 6 IF-01): the arm and the centre both black for at
 *     least 14 mm of travel (a branch or crossing to the left), OR the
 *     right sensor black for at least 14 mm while the centre sensor is
 *     still on the line (a branch to the right).  14 mm is longer than any
 *     barcode bar (wide bar 9 mm) and shorter than an 18 mm branch line;
 *     it is measured in distance (subsystem 2 odometry,
 *     line_sense_travel()), because a time threshold cannot tell a bar
 *     from a branch at different speeds.  1 Nov: the left side used a
 *     60 ms timer, and at 12 cm/s a 9 mm barcode bar lasts 75 ms, so a bar
 *     under the arm could be taken for a junction.

 * Assumption: the TCRT5000 analogue output differs by at least 400 ADC
 * counts between the white floor and the black line (checked by
 * calibration).  Either polarity works, because normalisation uses the
 * calibrated white and black readings.
 */

#include "line_sense.h"

/* Hysteresis on the normalised reading (0 = white, 1000 = black). */
#define LINE_ON_LEVEL          600     /* White -> black above this.  */
#define LINE_OFF_LEVEL         400     /* Black -> white below this.  */
/* 20 Nov: the black/white decision uses the RAW reading, not the calibrated
 * one.  On this car the centre sensor reads ~1700 on black where the default
 * calibration expects 3000, and the boot sweep cannot calibrate it because
 * the line sits BETWEEN the sensors at power-on, not under either of them.
 * Every sensor on the car reads < 400 on white and > 1200 on black, so a raw
 * threshold works for all three without a good sweep. */
#define LINE_RAW_BLACK_ON      900
#define LINE_RAW_BLACK_OFF     650

/* Barcode arm: RAW ADC counts, not normalised.  The arm senses printed
 * black on paper, which is far lighter than the track's black, and it
 * never crosses the line, so its boot calibration span is tiny (66-84
 * counts measured 19 Nov) and the normalised reading is unusable there -
 * some bars classified black, some white.  Classify raw instead. */
#define LINE_BC_MIN_SPAN       240u    /* raw counts: below this, no ink */
#define LINE_BC_CAL_OK         400     /* calibration span that is usable */

/* Centre-sensor band in which the reading is proportional to the error. */
#define LINE_BLACK_NORM        500     /* >500 of 1000 counts as
                                         * black: no grey band. */
#define LINE_EDGE_LOW          100
#define LINE_EDGE_HIGH         900
#define LINE_EDGE_MID          500
#define LINE_EDGE_HALF_SPAN    400     /* LINE_EDGE_HIGH - LINE_EDGE_MID. */

/* Error values, mm.  Positive = steer left. */
#define LINE_EDGE_HALF_MM      9       /* Half of the 18 mm line width. */
#define LINE_CENTRE_DARK_MM    15      /* Centre fully on the line.     */
/* 17 Nov (car video): the right guard used to give -40 at once and a dark
 * centre only +15, so a small drift to the left made the car swing hard
 * right (the wobble), and a left bend got a gentle +15 that could not
 * follow it, while the jolt to the right on the way into the bend left
 * the car pointing past it.  Both sides now start at 15 mm and grow to
 * 40 mm over LINE_RAMP_MM of travel while the condition lasts: a drift
 * is corrected gently, a bend that keeps going gets the full turn. */
/* 17 Nov evening (car): the ramp made the straight-line weave worse - a
 * small drift reached the larger values before the car came back.  The
 * correction now stays at 15 mm for the first LINE_RAMP_HOLD_MM of travel
 * (a drift on a straight is over by then) and only then grows, so only a
 * bend that keeps going gets the full turn. */
#define LINE_RAMP_START_MM     15
#define LINE_RAMP_HOLD_MM      10u     /* Travel held at 15 mm first. */
#define LINE_RAMP_MM           20u     /* Then travel from 15 to 40 mm. */
#define LINE_ERROR_MAX_MM      S1_LINE_ERR_MAX_MM

/* Event timing, ms. */
#define LINE_LOST_MS           300u    /* No sensor on line -> LINE_LOST. */
#define LINE_BRANCH_MM         10u     /* BOTH track sensors black this far:
                                        * a crossing - wider than the gap. */
#define LINE_BRANCH_SINGLE_MM  30u     /* ONE sensor black this far: a one-
                                        * sided branch (not used: the junction
                                        * is both track sensors black). */
#define LINE_ARM_MM            12u     /* Arm black this far: the line,
                                        * not a barcode bar (max 9 mm).   */
#define LINE_MM_MAX            0xFFu
#define LINE_TIME_MAX          0xFFFFu

/* Default calibration until line_sense_cal_apply() succeeds.  These are
 * placeholders: measure the real values on the track (TEST_LOG S1-H01).
 */
#define LINE_DEFAULT_WHITE     300u
#define LINE_DEFAULT_BLACK     3000u
#define LINE_CAL_MIN_SPAN      400u    /* Minimum black-white difference. */

static uint16_t add_time(uint16_t time_ms);
static uint8_t  add_travel(uint8_t total, bool b_cond, uint8_t step);
static int32_t  ramp_mm(uint8_t travel_mm);
static void     update_barcode_level(line_sense_t * p_sense, uint16_t level,
                                     bool b_raw);
static void     update_hysteresis(line_sense_t * p_sense,
                                  uint16_t const norm[LINE_SENSORS]);
static int8_t   compute_error(line_sense_t const * p_sense,
                              uint16_t centre_norm, bool * p_b_valid);


/*!
 * @brief Distance travelled since the previous sample (subsystem 2
 *        odometry), used by the next line_sense_update() for the
 *        right-branch junction.  0 if unknown: then only the left-side
 *        junction rule can fire.
 */
void
line_sense_travel (line_sense_t * p_sense, uint8_t travel_mm)
{
    p_sense->step_mm = travel_mm;
}

/*!
 * @brief Start with every sensor on white and no events active.
 */
void
line_sense_init (line_sense_t * p_sense)
{
    uint8_t sensor = 0u;

    for (sensor = 0u; sensor < LINE_SENSORS; sensor++)
    {
        p_sense->b_black[sensor] = false;
    }

    p_sense->last_error_mm = 0;
    p_sense->lost_ms       = 0u;
    p_sense->left_mm       = 0u;
    p_sense->arm_mm        = 0u;
    p_sense->bc_sum        = 0u;
    p_sense->bc_next       = 0u;
    p_sense->b_bc_black    = false;
    p_sense->bc_level      = 0u;
    p_sense->bc_min        = 4095u;
    p_sense->bc_max        = 0u;
    p_sense->bc_sensor     = LINE_BARCODE;

    for (sensor = 0u; sensor < LINE_BC_AVG_SAMPLES; sensor++)
    {
        p_sense->bc_hist[sensor] = 0u;
    }

    p_sense->step_mm       = 0u;
    p_sense->branch_mm     = 0u;
    p_sense->dark_mm       = 0u;
    p_sense->guard_mm      = 0u;
    p_sense->b_lost        = false;
    p_sense->b_junction    = false;
}

/*!
 * @brief Load the placeholder calibration.
 */
void
line_sense_cal_default (line_cal_t * p_cal)
{
    uint8_t sensor = 0u;

    for (sensor = 0u; sensor < LINE_SENSORS; sensor++)
    {
        p_cal->white[sensor] = LINE_DEFAULT_WHITE;
        p_cal->black[sensor] = LINE_DEFAULT_BLACK;
    }
}

/*!
 * @brief Begin collecting the lowest and highest reading of each sensor.
 */
void
line_sense_cal_start (line_cal_run_t * p_run)
{
    uint8_t sensor = 0u;

    for (sensor = 0u; sensor < LINE_SENSORS; sensor++)
    {
        p_run->lowest[sensor]  = LINE_ADC_MAX;
        p_run->highest[sensor] = 0u;
    }
}

/*!
 * @brief Include one sample in the calibration.
 */
void
line_sense_cal_track (line_cal_run_t * p_run,
                      uint16_t const raw[LINE_SENSORS])
{
    uint8_t sensor = 0u;

    for (sensor = 0u; sensor < LINE_SENSORS; sensor++)
    {
        if (raw[sensor] < p_run->lowest[sensor])
        {
            p_run->lowest[sensor] = raw[sensor];
        }

        if (raw[sensor] > p_run->highest[sensor])
        {
            p_run->highest[sensor] = raw[sensor];
        }
    }
}

/*!
 * @brief Turn the collected range into a calibration.
 *
 * Every sensor must have seen both the floor and the line during the
 * calibration sweep.  The line is assumed to give the HIGHER reading
 * (TCRT5000 module: less reflected light, higher AO voltage).
 *
 * @return True if applied; false (calibration unchanged) if any sensor's
 *         range is smaller than LINE_CAL_MIN_SPAN.
 */
bool
line_sense_cal_apply (line_cal_run_t const * p_run, line_cal_t * p_cal)
{
    bool    b_ok   = true;
    uint8_t sensor = 0u;

    for (sensor = 0u; sensor < LINE_SENSORS; sensor++)
    {
        if ((p_run->highest[sensor] < p_run->lowest[sensor]) ||
            ((uint16_t)(p_run->highest[sensor] - p_run->lowest[sensor]) <
             LINE_CAL_MIN_SPAN))    /* Cast: difference is >= 0 here. */
        {
            b_ok = false;
        }
    }

    if (b_ok)
    {
        for (sensor = 0u; sensor < LINE_SENSORS; sensor++)
        {
            p_cal->white[sensor] = p_run->lowest[sensor];
            p_cal->black[sensor] = p_run->highest[sensor];
        }
    }

    return (b_ok);
}

/*!
 * @brief Calibrate one sensor only (barcode bench, CHECK=6: only the
 *        barcode sensor is connected, so the other two never pass).
 *
 * @return True if that sensor saw both white and black.
 */
bool
line_sense_cal_apply_one (line_cal_run_t const * p_run, line_cal_t * p_cal,
                          uint8_t sensor)
{
    bool b_ok = (sensor < LINE_SENSORS) &&
                (p_run->highest[sensor] >= p_run->lowest[sensor]) &&
                ((uint16_t)(p_run->highest[sensor] - p_run->lowest[sensor]) >=
                 LINE_CAL_MIN_SPAN);

    if (b_ok)
    {
        p_cal->white[sensor] = p_run->lowest[sensor];
        p_cal->black[sensor] = p_run->highest[sensor];
    }

    return (b_ok);
}

/*!
 * @brief Scale a raw reading to 0 (white) .. 1000 (black).
 *
 * @return The normalised reading, clamped to 0..1000.  0 if the
 *         calibration has no range.
 */
uint16_t
line_sense_normalise (uint16_t raw, uint16_t white, uint16_t black)
{
    int32_t span   = (int32_t)black - (int32_t)white;
    int32_t scaled = 0;

    if (0 != span)
    {
        scaled = (((int32_t)raw - (int32_t)white) *
                  (int32_t)LINE_FULL_SCALE) / span;
    }

    if (scaled < 0)
    {
        scaled = 0;
    }
    else if (scaled > (int32_t)LINE_FULL_SCALE)
    {
        scaled = (int32_t)LINE_FULL_SCALE;
    }
    else
    {
        /* Already in range. */
    }

    return ((uint16_t)scaled);  /* 0..1000 fits in 16 bits. */
}

/*!
 * @brief Process one sample of the three sensors.
 *
 * @param[in,out] p_sense State kept between samples.
 * @param[in]     p_cal   Current calibration.
 * @param[in]     raw     Raw ADC readings: left, centre, right.
 * @param[out]    p_word  Packed line-state word to publish (IF-01).
 *
 * @return New events this sample (LINE_EVT_ bits), 0 if none.
 */
uint8_t
line_sense_update (line_sense_t * p_sense, line_cal_t const * p_cal,
                   uint16_t const raw[LINE_SENSORS], uint16_t * p_word)
{
    uint8_t           events = 0u;
    uint16_t          norm[LINE_SENSORS];
    subsystem1_line_t line;
    uint8_t           sensor = 0u;

    for (sensor = 0u; sensor < LINE_SENSORS; sensor++)
    {
        norm[sensor] = line_sense_normalise(raw[sensor],
                                            p_cal->white[sensor],
                                            p_cal->black[sensor]);
    }

    update_hysteresis(p_sense, raw);
    {
        uint16_t arm_white = p_cal->white[p_sense->bc_sensor];
        uint16_t arm_black = p_cal->black[p_sense->bc_sensor];

        if ((uint32_t)(arm_black - arm_white) >= (uint32_t)LINE_BC_CAL_OK)
        {
            update_barcode_level(p_sense, norm[p_sense->bc_sensor], false);
        }
        else
        {
            /* The arm never crosses the line, so its boot span is tiny
             * (66-84 counts measured): normalising by it is meaningless.
             * Classify the raw reading against its own envelope instead. */
            update_barcode_level(p_sense, raw[p_sense->bc_sensor], true);
        }
    }

    /* How far the arm has been black without a break: a barcode bar is at
     * most 9 mm, so only a longer run is the line itself. */
    p_sense->arm_mm = add_travel(p_sense->arm_mm,
                                 p_sense->b_black[LINE_LEFT],
                                 p_sense->step_mm);

    /* How far the car has gone with the centre fully on the line, or with
     * only the right guard on it: the steering ramps up with these (the
     * same tests as in compute_error()). */
    {
        bool b_dark  = p_sense->b_black[LINE_CENTRE] &&
                       (!p_sense->b_black[LINE_RIGHT]);
        bool b_guard = p_sense->b_black[LINE_RIGHT] &&
                       (!p_sense->b_black[LINE_CENTRE]);

        p_sense->dark_mm  = add_travel(p_sense->dark_mm, b_dark,
                                       p_sense->step_mm);
        p_sense->guard_mm = add_travel(p_sense->guard_mm, b_guard,
                                       p_sense->step_mm);
    }

    line.error_mm = compute_error(p_sense, norm[LINE_CENTRE], &line.b_valid);

    if (line.b_valid)
    {
        if ((line.error_mm >= LINE_MEMORY_MM) ||
            (line.error_mm <= -LINE_MEMORY_MM))
        {
            /* Remember only a meaningful drift, not noise.  Storing every
             * reading meant a -6 mm brush got remembered as "the line is on
             * the right", so the very next lost sample committed to a hard
             * right turn and never recovered (30 Oct). */
            p_sense->last_error_mm = line.error_mm;
        }

        if (p_sense->b_lost)
        {
            p_sense->b_lost = false;
            events |= (uint8_t)LINE_EVT_FOUND;  /* Bit value fits. */
        }

        p_sense->lost_ms = 0u;
    }
    else
    {
        p_sense->lost_ms = add_time(p_sense->lost_ms);

        if ((!p_sense->b_lost) && (p_sense->lost_ms >= LINE_LOST_MS))
        {
            p_sense->b_lost = true;
            events |= (uint8_t)LINE_EVT_LOST;   /* Bit value fits. */
        }
    }

    /* JUNCTION: a branch line is 18 mm wide, a barcode bar at most 9 mm,
     * so each side needs 14 mm of travel with its sensor black.
     */
    {
        /* 20 Nov: the arm now sits BEHIND the centre sensor, so arm and
         * centre can never be black together - the old test could not fire
         * and the car drove straight past every left branch.  A left branch
         * is now: arm on black, right sensor clear (a branch is one-sided).
         */
        /* 20 Nov: a junction is the track line turning under a sensor and
         * STAYING there for 17 mm.  A drift brush is corrected away in a
         * few mm, so it never gets that far. */
        /* 20 Nov, owner's rule: BOTH track sensors black is a junction, full
         * stop - no distance, no timer.  A single sensor black is just a
         * drift or a curve, and the steering handles that with no rules at
         * all.  The turn itself waits for the ARM to reach the junction,
         * which is what centres the car on the branch.
         */
        bool b_left   = p_sense->b_black[LINE_CENTRE];
        bool b_right  = p_sense->b_black[LINE_RIGHT];
        bool b_active = b_left && b_right;

        p_sense->left_mm   = 0u;
        p_sense->branch_mm = 0u;
        p_sense->step_mm   = 0u;

        if (b_active && (!p_sense->b_junction))
        {
            events |= (uint8_t)LINE_EVT_JUNCTION;  /* Bit value fits. */
        }

        p_sense->b_junction = b_active;
        p_sense->step_mm    = 0u;   /* Used once. */
    }

    line.b_lost     = p_sense->b_lost;
    line.b_junction = p_sense->b_junction;
    line.b_arm      = p_sense->b_black[LINE_LEFT];
    *p_word         = line_sense_pack(&line);

    return (events);
}

/*!
 * @brief Pack a line state into the 16-bit IF-01 word.
 *
 * The error is clamped to -40..+40 mm and stored with an offset of 40,
 * because Barr-C does not allow signed bit-fields.
 */
uint16_t
line_sense_pack (subsystem1_line_t const * p_line)
{
    int16_t  error = p_line->error_mm;
    uint16_t word  = 0u;

    if (error > LINE_ERROR_MAX_MM)
    {
        error = LINE_ERROR_MAX_MM;
    }
    else if (error < -LINE_ERROR_MAX_MM)
    {
        error = -LINE_ERROR_MAX_MM;
    }
    else
    {
        /* Already in range. */
    }

    /* error + 40 is 0..80, so the cast to unsigned is safe. */
    word = (uint16_t)((uint16_t)(error + (int16_t)S1_LINE_ERR_OFFSET) &
                      S1_LINE_ERR_MASK);

    if (p_line->b_valid)
    {
        word |= S1_LINE_VALID_BIT;
    }

    if (p_line->b_lost)
    {
        word |= S1_LINE_LOST_BIT;
    }

    if (p_line->b_junction)
    {
        word |= S1_LINE_JUNCTION_BIT;
    }

    if (p_line->b_arm)
    {
        word |= S1_LINE_ARM_BIT;       /* 22 Nov: see S1_LINE_ARM_BIT */
    }

    return (word);
}

/*!
 * @brief Unpack the 16-bit IF-01 word.
 */
void
line_sense_unpack (uint16_t word, subsystem1_line_t * p_line)
{
    int16_t offset_error = (int16_t)(word & S1_LINE_ERR_MASK);  /* 0..127 */

    p_line->error_mm   = (int8_t)(offset_error -
                                  (int16_t)S1_LINE_ERR_OFFSET);  /* -40..87 */
    p_line->b_valid    = (0u != (word & S1_LINE_VALID_BIT));
    p_line->b_lost     = (0u != (word & S1_LINE_LOST_BIT));
    p_line->b_junction = (0u != (word & S1_LINE_JUNCTION_BIT));
    p_line->b_arm      = (0u != (word & S1_LINE_ARM_BIT));
}

/*!
 * @brief Add one sample period to a timer, saturating at 65535 ms.
 */
/*!
 * @brief The barcode decoder's own view of the arm: a moving average of the
 *        last LINE_BC_AVG_SAMPLES readings, then the same hysteresis as the
 *        line sensors.  Averaging removes the motor noise that made the
 *        black/white decision flicker at every bar edge while driving
 *        (2 Nov); both edges of a bar are delayed alike, so widths stay.
 */
static void
update_barcode_level (line_sense_t * p_sense, uint16_t level, bool b_raw)
{
    uint32_t average = 0u;

    p_sense->bc_sum -= p_sense->bc_hist[p_sense->bc_next];
    p_sense->bc_hist[p_sense->bc_next] = level;
    p_sense->bc_sum += level;
    p_sense->bc_next = (uint8_t)((p_sense->bc_next + 1u) %
                                 LINE_BC_AVG_SAMPLES);
    average = p_sense->bc_sum / LINE_BC_AVG_SAMPLES;
    p_sense->bc_level = (uint16_t)average;

    if (!b_raw)
    {
        /* The arm has a usable white/black calibration: the spec path. */
        if (p_sense->b_bc_black && (average < (uint32_t)LINE_OFF_LEVEL))
        {
            p_sense->b_bc_black = false;
        }
        else if ((!p_sense->b_bc_black) && (average > (uint32_t)LINE_ON_LEVEL))
        {
            p_sense->b_bc_black = true;
        }
        else
        {
            /* Inside the hysteresis band: keep the previous decision. */
        }
        return;
    }

    /* Adaptive threshold: the arm reads printed ink, which can be any
     * darkness, and its boot calibration is useless (it never crosses the
     * line).  Track the envelope of the raw reading - fast attack, slow
     * release - and slice at its midpoint.  Scale-independent, so the
     * 12-bit ADC and the 0..1000 simulated levels in the tests both work. */
    if (average < (uint32_t)p_sense->bc_min)
    {
        p_sense->bc_min = (uint16_t)(p_sense->bc_min +
                                     (((int32_t)average -
                                       (int32_t)p_sense->bc_min) / 4));
    }
    else
    {
        p_sense->bc_min = (uint16_t)(p_sense->bc_min +
                                     (((int32_t)average -
                                       (int32_t)p_sense->bc_min) / 256));
    }

    if (average > (uint32_t)p_sense->bc_max)
    {
        p_sense->bc_max = (uint16_t)(p_sense->bc_max +
                                     (((int32_t)average -
                                       (int32_t)p_sense->bc_max) / 4));
    }
    else
    {
        p_sense->bc_max = (uint16_t)(p_sense->bc_max +
                                     (((int32_t)average -
                                       (int32_t)p_sense->bc_max) / 256));
    }

    {
        uint32_t span = (uint32_t)(p_sense->bc_max - p_sense->bc_min);
        uint32_t mid;
        uint32_t band;

        if (span < (uint32_t)LINE_BC_MIN_SPAN)
        {
            /* Too little contrast (blur, shadow, plain paper): hold the
             * last decision rather than declaring white for ever. */
        }
        else
        {
            mid  = ((uint32_t)p_sense->bc_min + (uint32_t)p_sense->bc_max) / 2u;
            band = span / 8u;

            if (p_sense->b_bc_black && (average < (mid - band)))
            {
                p_sense->b_bc_black = false;
            }
            else if ((!p_sense->b_bc_black) && (average > (mid + band)))
            {
                p_sense->b_bc_black = true;
            }
        }
    }
}

/*!
 * @brief Travel accumulated while a junction condition holds, 0 when not.
 *
 * @param[in] total  Travel so far, mm (saturates at 255).
 * @param[in] b_cond The condition this sample.
 * @param[in] step   Travel since the last sample, mm.
 */
static uint8_t
add_travel (uint8_t total, bool b_cond, uint8_t step)
{
    uint32_t sum = (uint32_t)total + step;

    if (!b_cond)
    {
        sum = 0u;
    }
    else if (sum > LINE_MM_MAX)
    {
        sum = LINE_MM_MAX;
    }
    else
    {
        /* In range. */
    }

    return ((uint8_t)sum);
}

static uint16_t
add_time (uint16_t time_ms)
{
    uint16_t result = LINE_TIME_MAX;

    if (time_ms <= (LINE_TIME_MAX - S1_SAMPLE_MS))
    {
        result = (uint16_t)(time_ms + S1_SAMPLE_MS);
    }

    return (result);
}

/*!
 * @brief Black/white decision for each sensor, with hysteresis so that a
 *        reading near the threshold does not chatter.
 */
static void
update_hysteresis (line_sense_t * p_sense, uint16_t const raw[LINE_SENSORS])
{
    uint8_t sensor = 0u;

    for (sensor = 0u; sensor < LINE_SENSORS; sensor++)
    {
        int32_t level = (int32_t)raw[sensor];

        if (p_sense->b_black[sensor] && (level < LINE_RAW_BLACK_OFF))
        {
            p_sense->b_black[sensor] = false;
        }
        else if ((!p_sense->b_black[sensor]) && (level > LINE_RAW_BLACK_ON))
        {
            p_sense->b_black[sensor] = true;
        }
        else
        {
            /* Inside the hysteresis band: keep the previous decision. */
        }
    }
}

/*!
 * @brief 15 mm for LINE_RAMP_HOLD_MM of travel, then rising linearly to
 *        LINE_ERROR_MAX_MM over the next LINE_RAMP_MM.
 */
static int32_t
ramp_mm (uint8_t travel_mm)
{
    uint32_t t = (travel_mm > LINE_RAMP_HOLD_MM) ?
                 ((uint32_t)travel_mm - LINE_RAMP_HOLD_MM) : 0u;

    if (t > LINE_RAMP_MM)
    {
        t = LINE_RAMP_MM;
    }

    return ((int32_t)LINE_RAMP_START_MM +
            (((int32_t)(LINE_ERROR_MAX_MM - LINE_RAMP_START_MM) * (int32_t)t) /
             (int32_t)LINE_RAMP_MM));
}

/*!
 * @brief Signed left-edge error in mm.  Positive = steer left.
 *
 * @param[in]  p_sense     Sensor decisions after hysteresis.
 * @param[in]  centre_norm Normalised centre reading, 0..1000.
 * @param[out] p_b_valid   True if any sensor sees the line.
 */
/* 20 Nov: while the car creeps over a junction the steering must be dead
 * straight.  One sensor alone going black there already means "branch" or
 * "junction", not "I have drifted", so correcting on it swung the car at
 * full gain on the junction itself (car test 20 Nov: "it just speeds up
 * turn left"). */
static bool g_hold_straight = false;

/* 20 Nov, owner's design: at a junction do not pivot - just lean the
 * follower towards the barcode's direction and let it follow the branch the
 * same way it follows any curve.  This is the extra lean. */
static int8_t g_turn_bias = 0;

/* 22 Nov (car): on a bend the line slips out from between the two sensors
 * long before the car has turned enough, and "both white" was read as
 * "straight" - so the car straightened up mid-bend and drove off the track.
 * Keep the last real correction and fade it out over LINE_HOLD_MM of
 * travel instead of dropping to zero on the first white sample.  A straight
 * only sees this after an actual correction, and then it fades to nothing
 * within 40 mm. */
#define LINE_HOLD_MM   40
static int8_t  s_hold_mm      = 0;
static int16_t s_hold_left_mm = 0;

void
line_sense_hold (bool b_on)
{
    g_hold_straight = b_on;
}

void
line_sense_bias (int8_t bias_mm)
{
    g_turn_bias = bias_mm;
}

static int8_t
compute_error (line_sense_t const * p_sense, uint16_t centre_norm,
               bool * p_b_valid)
{
    /* 20 Nov, owner's layout: the two track sensors sit just OUTSIDE the
     * left and right edges of the line, so the line runs between them in
     * the normal case.  Everything is binary, no grey band:
     *
     *   both white           line between them   -> straight
     *   left (centre) black   line under left     -> car is right, steer LEFT
     *   right black           line under right    -> car is left,  steer RIGHT
     *   both black            black wider than the gap -> junction, straight
     *
     * "Both white" is the healthy state, so the line counts as valid there:
     * the car must not call that "lost" and start a recovery sweep.
     *
     * The arm (LINE_LEFT, Grove 7) is the BARCODE reader and never steers.
     */
    bool    b_centre_black = p_sense->b_black[LINE_CENTRE];
    bool    b_right_black  = p_sense->b_black[LINE_RIGHT];
    int32_t error          = 0;

    (void)centre_norm;
    *p_b_valid = true;

    if (g_hold_straight)
    {
        /* Creeping over a junction: see line_sense_hold(). */
        return (0);
    }

    if (b_centre_black && b_right_black)
    {
        error = 0;
    }
    else if (b_centre_black)
    {
        error = ramp_mm(p_sense->dark_mm);
    }
    else if (b_right_black)
    {
        error = -ramp_mm(p_sense->guard_mm);
    }
    else
    {
        /* 22 Nov: both white.  Inside a bend this is "the line has slipped
         * out of view", not "we are centred" - hold the last correction and
         * fade it, otherwise the car straightens up and leaves the track. */
        if (s_hold_left_mm > 0)
        {
            s_hold_left_mm -= (int16_t)p_sense->step_mm;
            if (s_hold_left_mm <= 0)
            {
                s_hold_left_mm = 0;
                s_hold_mm      = 0;
                error          = 0;
            }
            else
            {
                error = ((int32_t)s_hold_mm * s_hold_left_mm) / LINE_HOLD_MM;
            }
        }
        else
        {
            error = 0;
        }
    }

    /* Arm the hold ONLY from a real sensor reading (exactly one sensor
     * black).  Arming on any non-zero error would re-arm from the held
     * value itself and the car would curve forever. */
    if ((!g_hold_straight) && (b_centre_black != b_right_black))
    {
        int32_t held = error + ((int32_t)g_turn_bias);
        if (held > 127)  { held = 127; }
        if (held < -128) { held = -128; }
        s_hold_mm      = (int8_t)held;
        s_hold_left_mm = LINE_HOLD_MM;
    }
    else if (b_centre_black && b_right_black)
    {
        s_hold_left_mm = 0;         /* junction: stop curving */
        s_hold_mm      = 0;
    }

    error += (int32_t)g_turn_bias;

    if (error > 127)
    {
        error = 127;
    }
    if (error < -128)
    {
        error = -128;
    }

    return ((int8_t)error);
}

/*** end of file ***/
