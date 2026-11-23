/** @file barcode.c
 *
 * @brief Code 39 barcode decoder (Week 6 statechart 6.2).
 *
 * The decoder is fed one black/white decision per IR sample (every 2 ms)
 * from the right-hand (barcode) sensor, after software hysteresis.
 *
 *   WAIT_QUIET      Wait for at least 60 ms of white, so a read never
 *                   starts in the middle of a barcode.
 *   CAPTURE         Count the width (in samples) of every black and white
 *                   element.  As soon as the 29th element (the last bar of
 *                   the stop '*') ends, the read is validated (1 Nov:
 *                   before, the decoder also waited for 5 narrow widths of
 *                   white, about 15 mm of travel, so a junction right after
 *                   the barcode could arrive before the command).  Capture
 *                   also ends when a white element is longer than 5 x the
 *                   narrowest element seen so far: inside a barcode no
 *                   space is wider than 3 narrow units, while Code 39
 *                   requires a quiet zone of at least 10.  Neither end
 *                   test depends on speed.
 *   VALIDATE        Exactly 29 elements must have been captured.  In each
 *                   9-element character the 3 widest elements are wide;
 *                   the read is rejected unless every wide element is at
 *                   least 1.5 x every narrow one.  Choosing the 3 widest
 *                   per character keeps the result correct if the speed
 *                   changes while the barcode passes the sensor.
 *   MATCH_FORWARD   '*' at both ends, data character in the middle.
 *   MATCH_REVERSED  Same, with the element order reversed (the robot drove
 *                   over the barcode the other way, e.g. after a U-turn).
 *   DECODED/ERROR   Report the result, then go back to WAIT_QUIET.
 *
 * Code 39 reference: each character is 9 elements (bar, space, bar, ...)
 * of which exactly 3 are wide.  Patterns below list element 1 as the most
 * significant of 9 bits, 1 = wide.
 */

#include "barcode.h"

/* Barcode geometry. */
#define BARCODE_CHARS            3u      /* '*', data character, '*'.  */
#define BARCODE_CHAR_ELEMENTS    9u      /* Elements per character.    */
#define BARCODE_CHAR_PITCH       10u     /* 9 elements + 1 gap.        */
#define BARCODE_WIDE_PER_CHAR    3u      /* Wide elements per character. */
#define BARCODE_DATA_CHAR        1u      /* Position of the data character. */
#define BARCODE_LAST_CHAR        2u      /* Position of the stop '*'.  */

/* Timing thresholds: written in ms, used in samples of S1_SAMPLE_MS, so
 * they keep their meaning if the sampling period changes (2 Nov: 2 -> 1 ms). */
#define BARCODE_MS(ms)           ((uint16_t)((ms) / S1_SAMPLE_MS))
#define BARCODE_QUIET_SAMPLES    BARCODE_MS(60u)  /* White before a read. */
#define BARCODE_END_RATIO        20u     /* End: white > 20 x narrowest
                                         * (3 Nov: 5 -> 20).  A good read
                                         * is decoded at its last bar, so
                                         * this only resets a partial one;
                                         * a hand that speeds up or slows
                                         * down 3x across the label must
                                         * not end the read early.       */
/* A narrow element shorter than this (2 ms samples) is noise, not a bar:
 * one 1-sample glitch used to poison min_width, which shrank the quiet-zone
 * test to 5 samples so a normal 3n space ended the read early - "r=5 ended
 * before 29 elements" on every pass (2 Nov).  4 samples = 8 ms = 1.0 mm at
 * 12 cm/s, well under a 3 mm bar. */
#define BARCODE_MIN_NARROW       BARCODE_MS(8u)
/* Black longer than this is the line, not a bar.  15 s (3 Nov; 2 Nov
 * 1.5 s; before 0.51 s): the large demo label has 21 mm wide bars, which
 * a very slow hand (3 mm/s) needs 7 s to cross. */
#define BARCODE_WIDTH_MAX        BARCODE_MS(15000u)
#define BARCODE_COUNT_MAX        0xFFFFu /* 16-bit saturating counters.  */
/* Debounce (2 Nov): a colour change counts only once it has held this
 * long.  Motor PWM noise on the long arm cable made the black/white
 * decision flicker for a few ms at each bar edge while driving (by hand,
 * motors off, the same label read fine); every flicker was an extra
 * element (r=1) or a tiny "narrowest" that ended the read early (r=5).
 * Every edge is delayed by the same amount, so widths do not change.
 * 4 ms is a third of a 3.5 mm bar at 25 cm/s. */
#define BARCODE_DEBOUNCE         BARCODE_MS(4u)

/* 18 Nov: use the travel widths when the sensor moved at least this far
 * during the read (1/16 mm; a car read crosses about 160 mm).  A label
 * swiped by hand past a parked car has no travel: time widths then. */
#define BARCODE_TRAVEL_MIN       (20u * 16u)     /* 20 mm. */

/* Wide elements must be at least NUM/DEN = 1.5 x the narrow ones. */
#define BARCODE_RATIO_NUM        3u
#define BARCODE_RATIO_DEN        2u

/* Why a read was rejected (kept for the console, see barcode_t). */
#define BARCODE_REASON_NONE       0u    /* Decoded, or nothing to report. */
#define BARCODE_REASON_TOO_MANY   1u    /* More than 29 elements seen.    */
#define BARCODE_REASON_RATIO      2u    /* No clear wide/narrow split.    */
#define BARCODE_REASON_PATTERN    3u    /* 29 elements, not Code 39.      */
#define BARCODE_REASON_BLACK_LONG 4u    /* Black for 255 samples: the line. */
#define BARCODE_REASON_EARLY_END  5u    /* Barcode ended before 29.       */

/* Code 39 character set: index 0..42 fits in 6 bits. */
#define BARCODE_SET_SIZE         43u
#define BARCODE_STAR_PATTERN     0x094u  /* '*' start/stop character. */
#define BARCODE_PATTERN_MSB      8u      /* Bit of element 1 in a pattern. */
#define BARCODE_NO_MATCH         0xFFu

static char const g_symbols[BARCODE_SET_SIZE + 1u] =
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";

static uint16_t const g_patterns[BARCODE_SET_SIZE] =
{
    0x034u, 0x121u, 0x061u, 0x160u, 0x031u,   /* '0' '1' '2' '3' '4' */
    0x130u, 0x070u, 0x025u, 0x124u, 0x064u,   /* '5' '6' '7' '8' '9' */
    0x109u, 0x049u, 0x148u, 0x019u, 0x118u,   /* 'A' 'B' 'C' 'D' 'E' */
    0x058u, 0x00Du, 0x10Cu, 0x04Cu, 0x01Cu,   /* 'F' 'G' 'H' 'I' 'J' */
    0x103u, 0x043u, 0x142u, 0x013u, 0x112u,   /* 'K' 'L' 'M' 'N' 'O' */
    0x052u, 0x007u, 0x106u, 0x046u, 0x016u,   /* 'P' 'Q' 'R' 'S' 'T' */
    0x181u, 0x0C1u, 0x1C0u, 0x091u, 0x190u,   /* 'U' 'V' 'W' 'X' 'Y' */
    0x0D0u, 0x085u, 0x184u, 0x0C4u, 0x0A8u,   /* 'Z' '-' '.' ' ' '$' */
    0x0A2u, 0x08Au, 0x02Au                    /* '/' '+' '%'         */
};

static uint16_t         saturating_increment(uint16_t value);
static void             restart(barcode_t * p_bc, uint16_t quiet);
static bool             debounce(barcode_t * p_bc, bool b_black);
static barcode_result_t check_run(barcode_t * p_bc, uint8_t * p_msg);
static barcode_result_t end_element(barcode_t * p_bc, bool b_black,
                                    uint8_t * p_msg);
static barcode_result_t finish(barcode_t * p_bc, uint8_t * p_msg);
static bool             classify(barcode_t const * p_bc,
                                 uint32_t * p_wide_bits);
static uint16_t         add_sat(uint16_t value, uint16_t add);
static uint16_t         char_pattern(uint32_t wide_bits, uint8_t char_no);
static uint16_t         reverse_pattern(uint16_t pattern);
static uint8_t          lookup(uint16_t pattern);
static uint8_t          match(uint32_t wide_bits, bool b_reversed);


/*!
 * @brief Reset the decoder to WAIT_QUIET with no quiet time seen yet.
 *
 * @param[out] p_bc Decoder to reset.
 */
void
barcode_init (barcode_t * p_bc)
{
    p_bc->reason       = BARCODE_REASON_NONE;
    p_bc->report_count = 0u;
    p_bc->report_min   = 0u;
    p_bc->b_level      = false;
    p_bc->pending      = 0u;
    p_bc->b_used_travel = false;

    {
        uint8_t i = 0u;

        for (i = 0u; i < BARCODE_REPORT_WIDTHS; i++)
        {
            p_bc->report[i] = 0u;
        }
    }

    restart(p_bc, 0u);
}

/*!
 * @brief Feed one sample of the barcode sensor into the decoder.
 *
 * @param[in,out] p_bc    Decoder memory.
 * @param[in]     b_black_raw True if the sensor sees black this sample
 *                            (debounced here, see BARCODE_DEBOUNCE).
 * @param[out]    p_msg   Packed barcode message, written only when the
 *                        result is BARCODE_RESULT_DECODED.
 *
 * @return What happened this sample.
 */
barcode_result_t
barcode_step (barcode_t * p_bc, bool b_black_raw, uint8_t * p_msg)
{
    return (barcode_step_travel(p_bc, b_black_raw, 0u, p_msg));
}

/*!
 * @brief barcode_step() plus how far the barcode sensor moved during this
 *        sample, in 1/16 mm (18 Nov).  Element widths are then kept in
 *        travel as well as in samples; a read during which the sensor
 *        moved at least BARCODE_TRAVEL_MIN is classified on travel, so a
 *        car that weaves (the sensor speeding up and slowing down) still
 *        measures the true bar widths.
 */
barcode_result_t
barcode_step_travel (barcode_t * p_bc, bool b_black_raw,
                     uint16_t travel_16th_mm, uint8_t * p_msg)
{
    barcode_result_t result  = BARCODE_RESULT_NONE;
    bool             b_black = debounce(p_bc, b_black_raw);

    if (BARCODE_WAIT_QUIET == p_bc->state)
    {
        if (!b_black)
        {
            p_bc->quiet = saturating_increment(p_bc->quiet);
        }
        else if (p_bc->quiet >= BARCODE_QUIET_SAMPLES)
        {
            /* The first bar of a possible barcode starts now. */
            p_bc->count       = 0u;
            p_bc->run         = 1u;
            p_bc->run_travel  = travel_16th_mm;
            p_bc->capture_travel = travel_16th_mm;
            p_bc->min_width   = BARCODE_COUNT_MAX;
    p_bc->min_width2  = BARCODE_COUNT_MAX;
            p_bc->min_width2  = BARCODE_COUNT_MAX;
            p_bc->b_run_black = true;
            p_bc->state       = (uint8_t)BARCODE_CAPTURE;  /* Enum fits. */
        }
        else
        {
            /* Black without enough white before it: not a barcode start. */
            p_bc->quiet = 0u;
        }
    }
    else if (b_black == p_bc->b_run_black)
    {
        p_bc->run        = saturating_increment(p_bc->run);
        p_bc->run_travel = add_sat(p_bc->run_travel, travel_16th_mm);
        p_bc->capture_travel += travel_16th_mm;
        result    = check_run(p_bc, p_msg);
    }
    else
    {
        result = end_element(p_bc, b_black, p_msg);
        /* This sample's travel belongs to the element that starts now. */
        p_bc->run_travel = travel_16th_mm;
        p_bc->capture_travel += travel_16th_mm;
    }

    return (result);
}

/*!
 * @brief Convert a character index from a barcode message to ASCII.
 *
 * @param[in] index Index 0..42 into the Code 39 character set.
 *
 * @return The character, or '?' if the index is out of range.
 */
char
barcode_index_to_char (uint8_t index)
{
    char symbol = '?';

    if (index < BARCODE_SET_SIZE)
    {
        symbol = g_symbols[index];
    }

    return (symbol);
}

/*!
 * @brief Runtime character-to-movement table (Project Write-up requirement:
 *        the four characters are announced by the instructor just before the
 *        demo and sent from the PC over MQTT, so they must not be fixed in
 *        the code).
 *
 * One slot per letter A..Z.  The default keeps the Project Write-up example
 * (A left, B right, C straight, D U-turn) so the car still behaves sensibly
 * if no mapping is sent, but every slot can be rewritten at run time.
 */
/* 21 Nov: 36 slots, 0-9 = 0..9, A-Z = 10..35 (the decoder's Code 39
 * order).  The MQTT map frame carries the LETTER index A = 0 .. Z = 25 in
 * 5 bits (subsystem4.h, checked < 26 by comms_codec.c), and
 * barcode_map_set() adds BARCODE_MAP_FIRST_LETTER.  With the table only
 * 32 slots and the frame sending 10 + letter, W-Z could not be sent at all
 * (5 bits) and Q-V were rejected by comms_codec.c (index < 26). */
#define BARCODE_MAP_LETTERS    36u
#define BARCODE_MAP_FIRST_LETTER 10u

static subsystem1_cmd_t g_map[BARCODE_MAP_LETTERS] =
{
    /* Code 39 index space, same as barcode_map_set() and the MQTT map
     * frame: digits 0-9 occupy 0..9, letters start at 10 ('A').  The table
     * used to be letter-indexed while the setter was not, so a map sent for
     * 'F' landed in slot 15 and every read looked in slot 5 - the car never
     * saw a command (19 Nov car log: char "F" -> NONE after mapping). */
    SUBSYSTEM1_CMD_NONE,      /* 0 */
    SUBSYSTEM1_CMD_NONE,      /* 1 */
    SUBSYSTEM1_CMD_NONE,      /* 2 */
    SUBSYSTEM1_CMD_NONE,      /* 3 */
    SUBSYSTEM1_CMD_NONE,      /* 4 */
    SUBSYSTEM1_CMD_NONE,      /* 5 */
    SUBSYSTEM1_CMD_NONE,      /* 6 */
    SUBSYSTEM1_CMD_NONE,      /* 7 */
    SUBSYSTEM1_CMD_NONE,      /* 8 */
    SUBSYSTEM1_CMD_NONE,      /* 9 */
    SUBSYSTEM1_CMD_LEFT,      /* A */
    SUBSYSTEM1_CMD_RIGHT,     /* B */
    SUBSYSTEM1_CMD_STRAIGHT,  /* C */
    SUBSYSTEM1_CMD_UTURN,     /* D */
    SUBSYSTEM1_CMD_NONE,      /* E */
    SUBSYSTEM1_CMD_NONE,      /* F */
    SUBSYSTEM1_CMD_NONE,      /* G */
    SUBSYSTEM1_CMD_NONE,      /* H */
    SUBSYSTEM1_CMD_NONE,      /* I */
    SUBSYSTEM1_CMD_NONE,      /* J */
    SUBSYSTEM1_CMD_NONE,      /* K */
    SUBSYSTEM1_CMD_NONE,      /* L */
    SUBSYSTEM1_CMD_NONE,      /* M */
    SUBSYSTEM1_CMD_NONE,      /* N */
    SUBSYSTEM1_CMD_NONE,      /* O */
    SUBSYSTEM1_CMD_NONE,      /* P */
    SUBSYSTEM1_CMD_NONE,      /* Q */
    SUBSYSTEM1_CMD_NONE,      /* R */
    SUBSYSTEM1_CMD_NONE,      /* S */
    SUBSYSTEM1_CMD_NONE,      /* T */
    SUBSYSTEM1_CMD_NONE,      /* U */
    SUBSYSTEM1_CMD_NONE,      /* V */
    SUBSYSTEM1_CMD_NONE,      /* W */
    SUBSYSTEM1_CMD_NONE,      /* X */
    SUBSYSTEM1_CMD_NONE,      /* Y */
    SUBSYSTEM1_CMD_NONE       /* Z */
};

/*!
 * @brief Point one letter at a movement.  Called from TSK_CTRL when the
 *        dashboard sends a "map" command.
 *
 * @param[in] index Letter index, 0 = 'A' .. 25 = 'Z'.
 * @param[in] cmd   Movement to execute at the next cross-junction.
 */
void
barcode_map_set (uint8_t index, subsystem1_cmd_t cmd)
{
    uint32_t slot = (uint32_t)index + BARCODE_MAP_FIRST_LETTER;

    if ((index < 26u) &&
        (cmd >= SUBSYSTEM1_CMD_LEFT) && (cmd <= SUBSYSTEM1_CMD_UTURN))
    {
        g_map[slot] = cmd;
    }
}

/*!
 * @brief Movement currently mapped to a letter, SUBSYSTEM1_CMD_NONE if any.
 */
subsystem1_cmd_t
barcode_map_get (uint8_t index)
{
    return ((index < 26u) ? g_map[(uint32_t)index + BARCODE_MAP_FIRST_LETTER]
                          : SUBSYSTEM1_CMD_NONE);
}

/*!
 * @brief Code 39 symbol -> map slot: '0'-'9' are 0..9, 'A'-'Z' start at 10.
 */
static int32_t
symbol_to_index (char symbol)
{
    if ((symbol >= '0') && (symbol <= '9'))
    {
        return ((int32_t)(symbol - '0'));
    }

    if ((symbol >= 'A') && (symbol <= 'Z'))
    {
        return (10 + (int32_t)(symbol - 'A'));
    }

    return (-1);
}

/*!
 * @brief Map a decoded character to a navigation command.
 *
 * Looks the character up in the runtime table above, so the four demo
 * characters are whatever the instructor announced.  Any other character is
 * a valid barcode that carries no command.
 *
 * @param[in] symbol Decoded character.
 *
 * @return The navigation command.
 */
subsystem1_cmd_t
barcode_char_to_cmd (char symbol)
{
    subsystem1_cmd_t cmd = SUBSYSTEM1_CMD_NONE;

    {
        int32_t idx = symbol_to_index(symbol);

        if ((idx >= 0) && (idx < (int32_t)BARCODE_MAP_LETTERS))
        {
            cmd = g_map[idx];
        }
    }

    if (('\0' != S1_BARCODE_END_CHAR) && (S1_BARCODE_END_CHAR == symbol))
    {
        cmd = SUBSYSTEM1_CMD_END;
    }

    return (cmd);
}

/*!
 * @brief Add without wrapping past 0xFFFF.
 */
static uint16_t
add_sat (uint16_t value, uint16_t add)
{
    uint32_t sum = (uint32_t)value + (uint32_t)add;

    return ((uint16_t)((sum > BARCODE_COUNT_MAX) ? BARCODE_COUNT_MAX : sum));
}

/*!
 * @brief Add one to an 8-bit count without wrapping past 255.
 */
static uint16_t
saturating_increment (uint16_t value)
{
    uint16_t result = value;

    if (result < BARCODE_COUNT_MAX)
    {
        result++;
    }

    return (result);
}

/*!
 * @brief The sensor colour after debouncing: a change is passed on only
 *        after it has held for BARCODE_DEBOUNCE samples; shorter flickers
 *        are ignored.
 */
static bool
debounce (barcode_t * p_bc, bool b_black)
{
    if (b_black == p_bc->b_level)
    {
        p_bc->pending = 0u;
    }
    else
    {
        p_bc->pending++;

        if (p_bc->pending >= BARCODE_DEBOUNCE)
        {
            p_bc->b_level = b_black;
            p_bc->pending = 0u;
        }
    }

    return (p_bc->b_level);
}

/*!
 * @brief Return to WAIT_QUIET.
 *
 * @param[out] p_bc  Decoder memory.
 * @param[in]  quiet White samples already seen (non-zero when the decoder
 *                   stops while the sensor is already on white).
 */
static void
restart (barcode_t * p_bc, uint16_t quiet)
{
    p_bc->count       = 0u;
    p_bc->run         = 0u;
    p_bc->run_travel  = 0u;
    p_bc->capture_travel = 0u;
    p_bc->min_width   = BARCODE_COUNT_MAX;
    /* 19 Nov: min_width2 was NEVER reset, so a single short glitch in one
     * read became the quiet-zone reference for every read after it and cut
     * them short at 3-13 elements. Both trackers, and the travel pair,
     * must start clean or a reject poisons the next attempt. */
    p_bc->min_width2  = BARCODE_COUNT_MAX;
    p_bc->min_travel  = BARCODE_COUNT_MAX;
    p_bc->min_travel2 = BARCODE_COUNT_MAX;
    p_bc->quiet       = quiet;
    p_bc->b_run_black = false;
    p_bc->state       = (uint8_t)BARCODE_WAIT_QUIET;  /* Enum fits. */
}

/*!
 * @brief Keep the evidence for a rejected read, then restart.
 *
 * restart() clears the counters, so the element count and the first few
 * widths are copied out first: without them a reject says nothing about
 * whether the bars were merged, missing, or simply not Code 39.
 */
static void
note_reject (barcode_t * p_bc, uint8_t reason)
{
    uint8_t i = 0u;

    p_bc->reason       = reason;
    p_bc->b_used_travel = (p_bc->capture_travel >= BARCODE_TRAVEL_MIN);
    p_bc->report_count = p_bc->count;
    p_bc->report_min   = p_bc->min_width;

    for (i = 0u; i < BARCODE_REPORT_WIDTHS; i++)
    {
        p_bc->report[i] = (i < p_bc->count) ? p_bc->widths[i] : 0u;
    }
}

/*!
 * @brief Check the element that is still growing (CAPTURE state).
 *
 * A black element that reaches BARCODE_WIDTH_MAX (15 s) is not a barcode
 * bar (e.g. the
 * sensor is over the line on a curve).  A white element longer than
 * 5 x the narrowest element is the end of a barcode.
 */
static barcode_result_t
check_run (barcode_t * p_bc, uint8_t * p_msg)
{
    barcode_result_t result = BARCODE_RESULT_NONE;
    /* Reference width for the quiet-zone limit: the SECOND narrowest
     * element, so one short glitch cannot shrink the limit and cut the
     * read short (19 Nov: a 5-element read on a 5 mm-bar label ended on a
     * 2.3 s gap because a blip had set the narrowest to ~20 ms). */
    uint32_t         ref    = (p_bc->min_width2 < BARCODE_COUNT_MAX)
                              ? p_bc->min_width2 : p_bc->min_width;
    uint32_t         narrow;
    uint32_t         limit;

    /* 19 Nov: a real label is mostly narrow elements, so the MEDIAN is the
     * narrow width and one short glitch cannot move it.  Minima can: a 57
     * sample blip inside a read with 70 sample bars set the quiet limit to
     * 20 x 57 and cut the read at 11 of 29 elements. */
    if (p_bc->count >= 7u)
    {
        uint16_t sorted[BARCODE_REPORT_WIDTHS];
        uint8_t  n = 0u;
        uint8_t  i;
        uint8_t  j;

        for (i = 0u; (i < p_bc->count) && (i < BARCODE_REPORT_WIDTHS); i++)
        {
            uint16_t v = p_bc->widths[i];

            for (j = n; (j > 0u) && (sorted[j - 1u] > v); j--)
            {
                sorted[j] = sorted[j - 1u];
            }

            sorted[j] = v;
            n++;
        }

        ref = sorted[n / 2u];
    }

    narrow = (ref < BARCODE_MIN_NARROW) ? BARCODE_MIN_NARROW : ref;
    limit  = narrow * BARCODE_END_RATIO;

    if (p_bc->b_run_black)
    {
        if (p_bc->run >= BARCODE_WIDTH_MAX)
        {
            note_reject(p_bc, BARCODE_REASON_BLACK_LONG);
            restart(p_bc, 0u);
            result = BARCODE_RESULT_REJECTED;
        }
    }
    else if (p_bc->run >= limit)
    {
        if (BARCODE_ELEMENTS == p_bc->count)
        {
            result = finish(p_bc, p_msg);
        }
        else
        {
            /* The barcode ended early: a partial or damaged read. */
            note_reject(p_bc, BARCODE_REASON_EARLY_END);
            restart(p_bc, p_bc->run);
            result = BARCODE_RESULT_REJECTED;
        }
    }
    else
    {
        /* Still a normal space inside the barcode. */
    }

    return (result);
}

/*!
 * @brief Store the element that has just ended and start the next one.
 */
static barcode_result_t
end_element (barcode_t * p_bc, bool b_black, uint8_t * p_msg)
{
    barcode_result_t result = BARCODE_RESULT_NONE;

    if (p_bc->count >= BARCODE_ELEMENTS)
    {
        /* More than 29 elements: not a single-character Code 39 barcode. */
        note_reject(p_bc, BARCODE_REASON_TOO_MANY);
        restart(p_bc, 0u);
        result = BARCODE_RESULT_REJECTED;
    }
    else
    {
        p_bc->widths[p_bc->count] = p_bc->run;
        p_bc->travel[p_bc->count] = p_bc->run_travel;
        p_bc->count++;

        if (p_bc->run >= BARCODE_MIN_NARROW)
        {
            if (p_bc->run < p_bc->min_width)
            {
                p_bc->min_width2 = p_bc->min_width;
                p_bc->min_width  = p_bc->run;
            }
            else if (p_bc->run < p_bc->min_width2)
            {
                p_bc->min_width2 = p_bc->run;
            }
        }

        if (0u != p_bc->run_travel)
        {
            if (p_bc->run_travel < p_bc->min_travel)
            {
                p_bc->min_travel2 = p_bc->min_travel;
                p_bc->min_travel  = p_bc->run_travel;
            }
            else if (p_bc->run_travel < p_bc->min_travel2)
            {
                p_bc->min_travel2 = p_bc->run_travel;
            }
        }

        p_bc->run         = 1u;
        p_bc->b_run_black = b_black;

        if ((BARCODE_ELEMENTS == p_bc->count) && (!b_black))
        {
            /* The last bar has just ended: decode now, while the robot is
             * still beside the barcode, not a quiet zone later. */
            result = finish(p_bc, p_msg);
        }
    }

    return (result);
}

/*!
 * @brief VALIDATE, MATCH_FORWARD, MATCH_REVERSED, then DECODED or ERROR.
 */
static barcode_result_t
finish (barcode_t * p_bc, uint8_t * p_msg)
{
    barcode_result_t result     = BARCODE_RESULT_REJECTED;
    uint32_t         wide_bits  = 0u;
    uint8_t          index      = BARCODE_NO_MATCH;
    bool             b_reversed = false;

    p_bc->state = (uint8_t)BARCODE_VALIDATE;  /* Enum fits in 8 bits. */
    p_bc->b_used_travel = (p_bc->capture_travel >= BARCODE_TRAVEL_MIN);

    if (classify(p_bc, &wide_bits))
    {
        p_bc->state = (uint8_t)BARCODE_MATCH_FORWARD;
        index       = match(wide_bits, false);

        if (BARCODE_NO_MATCH == index)
        {
            p_bc->state = (uint8_t)BARCODE_MATCH_REVERSED;
            index       = match(wide_bits, true);
            b_reversed  = true;
        }
    }

    if (BARCODE_NO_MATCH == index)
    {
        note_reject(p_bc, classify(p_bc, &wide_bits) ? BARCODE_REASON_PATTERN
                                                     : BARCODE_REASON_RATIO);
    }

    if (BARCODE_NO_MATCH != index)
    {
        p_bc->reason = BARCODE_REASON_NONE;
        p_bc->state = (uint8_t)BARCODE_DECODED;
        *p_msg      = (uint8_t)(index & S1_BC_INDEX_MASK);

        if (b_reversed)
        {
            *p_msg |= (uint8_t)S1_BC_REVERSED_BIT;  /* Bit 6 fits. */
        }

        result = BARCODE_RESULT_DECODED;
    }
    else
    {
        p_bc->state = (uint8_t)BARCODE_ERROR;
    }

    /* The sensor is already in the white quiet zone after the barcode. */
    restart(p_bc, p_bc->run);

    return (result);
}

/*!
 * @brief Decide which of the 29 elements are wide.
 *
 * @param[in]  p_bc        Decoder holding 29 element widths.
 * @param[out] p_wide_bits Bit i set means element i is wide.
 *
 * @return True if every character has a clear narrow/wide split.
 */
static bool
classify (barcode_t const * p_bc, uint32_t * p_wide_bits)
{
    bool     b_ok      = true;
    uint32_t wide_bits = 0u;
    uint8_t  char_no   = 0u;
    /* 18 Nov: travel widths when the sensor moved (car), else time (hand). */
    uint16_t const * widths = (p_bc->capture_travel >= BARCODE_TRAVEL_MIN)
                              ? p_bc->travel : p_bc->widths;

    for (char_no = 0u; (char_no < BARCODE_CHARS) && b_ok; char_no++)
    {
        uint8_t  base        = char_no * BARCODE_CHAR_PITCH;
        uint16_t picked      = 0u;   /* Bit k: element base+k is wide. */
        uint16_t min_wide    = BARCODE_COUNT_MAX;
        uint16_t max_narrow  = 0u;
        uint8_t  pick        = 0u;
        uint8_t  elem        = 0u;

        /* Pick the 3 widest elements of this character. */
        for (pick = 0u; pick < BARCODE_WIDE_PER_CHAR; pick++)
        {
            uint16_t best      = 0u;
            uint8_t  best_elem = 0u;

            for (elem = 0u; elem < BARCODE_CHAR_ELEMENTS; elem++)
            {
                bool b_taken = (0u != (picked & (1u << elem)));

                if ((!b_taken) && (widths[base + elem] > best))
                {
                    best      = widths[base + elem];
                    best_elem = elem;
                }
            }

            picked = (uint16_t)(picked | (1u << best_elem));

            if (best < min_wide)
            {
                min_wide = best;
            }
        }

        for (elem = 0u; elem < BARCODE_CHAR_ELEMENTS; elem++)
        {
            if (0u != (picked & (1u << elem)))
            {
                wide_bits |= (1uL << (base + elem));
            }
            else if (widths[base + elem] > max_narrow)
            {
                max_narrow = widths[base + elem];
            }
            else
            {
                /* A narrow element no wider than one already seen. */
            }
        }

        /* Reject unless wide >= 1.5 x narrow (integer arithmetic). */
        if (((uint32_t)min_wide * BARCODE_RATIO_DEN) <
            ((uint32_t)max_narrow * BARCODE_RATIO_NUM))
        {
            b_ok = false;
        }
    }

    *p_wide_bits = wide_bits;

    return (b_ok);
}

/*!
 * @brief Extract one character's 9-bit pattern, element 1 as bit 8.
 */
static uint16_t
char_pattern (uint32_t wide_bits, uint8_t char_no)
{
    uint16_t pattern = 0u;
    uint8_t  base    = char_no * BARCODE_CHAR_PITCH;
    uint8_t  elem    = 0u;

    for (elem = 0u; elem < BARCODE_CHAR_ELEMENTS; elem++)
    {
        if (0u != (wide_bits & (1uL << (base + elem))))
        {
            pattern = (uint16_t)(pattern |
                                 (1u << (BARCODE_PATTERN_MSB - elem)));
        }
    }

    return (pattern);
}

/*!
 * @brief Reverse the order of the 9 elements of a pattern.
 */
static uint16_t
reverse_pattern (uint16_t pattern)
{
    uint16_t reversed = 0u;
    uint8_t  elem     = 0u;

    for (elem = 0u; elem < BARCODE_CHAR_ELEMENTS; elem++)
    {
        if (0u != (pattern & (1u << elem)))
        {
            reversed = (uint16_t)(reversed |
                                  (1u << (BARCODE_PATTERN_MSB - elem)));
        }
    }

    return (reversed);
}

/*!
 * @brief Find a data character by its pattern.
 *
 * @return Index 0..42, or BARCODE_NO_MATCH.
 */
static uint8_t
lookup (uint16_t pattern)
{
    uint8_t index = BARCODE_NO_MATCH;
    uint8_t entry = 0u;

    for (entry = 0u; entry < BARCODE_SET_SIZE; entry++)
    {
        if (g_patterns[entry] == pattern)
        {
            index = entry;
            break;
        }
    }

    return (index);
}

/*!
 * @brief Check start/stop characters and look up the data character.
 *
 * Read in reverse, the true first character is the reverse of the last
 * one captured, and every pattern is mirrored.
 *
 * @return Index 0..42 of the data character, or BARCODE_NO_MATCH.
 */
static uint8_t
match (uint32_t wide_bits, bool b_reversed)
{
    uint8_t  index = BARCODE_NO_MATCH;
    uint16_t first = char_pattern(wide_bits, 0u);
    uint16_t data  = char_pattern(wide_bits, BARCODE_DATA_CHAR);
    uint16_t last  = char_pattern(wide_bits, BARCODE_LAST_CHAR);

    if (b_reversed)
    {
        uint16_t true_first = reverse_pattern(last);

        last  = reverse_pattern(first);
        first = true_first;
        data  = reverse_pattern(data);
    }

    if ((BARCODE_STAR_PATTERN == first) && (BARCODE_STAR_PATTERN == last))
    {
        index = lookup(data);
    }

    return (index);
}

/*** end of file ***/
