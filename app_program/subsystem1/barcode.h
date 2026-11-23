/** @file barcode.h
 *
 * @brief Code 39 barcode decoder (Week 6 statechart 6.2).  Private to
 *        subsystem 1.  Pure logic: no kernel calls, so it is unit-tested
 *        on target (subsystem1_test.c) and on a PC (testing/subsystem1).
 */

#ifndef BARCODE_H
#define BARCODE_H

#include <stdint.h>
#include <stdbool.h>

#include "subsystem1.h"

/* A barcode is a start '*', one data character and a stop '*':
 * 3 characters x 9 elements + 2 inter-character gaps = 29 elements.
 */
#define BARCODE_REPORT_WIDTHS    29u     /* Elements kept for a reject.    */

#define BARCODE_ELEMENTS       29u

/* Decoder states (statechart 6.2).  7 states fit in 3 bits. */
typedef enum
{
    BARCODE_WAIT_QUIET = 0,
    BARCODE_CAPTURE,
    BARCODE_VALIDATE,
    BARCODE_MATCH_FORWARD,
    BARCODE_MATCH_REVERSED,
    BARCODE_DECODED,
    BARCODE_ERROR
} barcode_state_t;

/* Result of one decoder step. */
typedef enum
{
    BARCODE_RESULT_NONE = 0,    /* Nothing finished this sample.     */
    BARCODE_RESULT_DECODED,     /* *p_msg holds a new barcode.       */
    BARCODE_RESULT_REJECTED     /* A barcode-like read was rejected. */
} barcode_result_t;

/* Decoder memory.  Widths are 8-bit sample counts that saturate at 255
 * (Change Requirement 1): 29 bytes instead of 116.
 */
typedef struct
{
    /* 16-bit widths (2 Nov): with 1 ms samples and a slow car a wide bar
     * is several hundred samples, more than 8 bits can count. */
    uint16_t widths[BARCODE_ELEMENTS];  /* Finished elements, in samples. */
    /* 18 Nov: the same elements measured in travel of the barcode sensor
     * (1/16 mm, from the wheel encoders).  Weaving changes the sensor's
     * speed - the arm sits beside the left wheel - so a time width is
     * only right at a steady speed.  Used whenever the car moved. */
    uint16_t travel[BARCODE_ELEMENTS];
    uint16_t run_travel;                /* Travel of the current element. */
    uint32_t capture_travel;            /* Travel since the first bar.    */
    bool     b_used_travel;             /* The last finish() used travel. */
    uint8_t  count;                     /* Elements finished so far.      */
    uint16_t run;                       /* Width of the current element.  */
    uint16_t min_width;                 /* Narrowest finished element.    */
    uint16_t min_width2;                /* Second narrowest: a single
                                         * glitch must not set the
                                         * quiet-zone limit.  */
    uint16_t min_travel;                /* Narrowest element, 1/16 mm.
                                         * The car's speed wanders (wheels
                                         * stall, line lost), so a gap
                                         * measured in time can look far
                                         * longer than it is in distance
                                         * and cut the read short (19 Nov). */
    uint16_t min_travel2;               /* Second narrowest, travel.      */
    uint16_t quiet;                     /* White samples seen while idle. */
    uint8_t  state;                     /* A barcode_state_t value.       */
    bool     b_run_black;               /* Colour of the current element. */
    bool     b_level;                   /* Debounced sensor colour.       */
    uint8_t  pending;                   /* Samples the other colour held. */

    /* Why the last read was rejected, kept for the console.  A reject used
     * to be completely silent, so 24 failed reads of a real label looked
     * exactly like a decoder that never saw the bars (2 Nov). */
    uint8_t  reason;                    /* BARCODE_REASON_* value.        */
    uint8_t  report_count;              /* Element count at the reject.   */
    uint16_t report_min;                /* Narrowest element then.        */
    uint16_t report[BARCODE_REPORT_WIDTHS];  /* First elements then.      */
} barcode_t;

void             barcode_init(barcode_t * p_bc);
barcode_result_t barcode_step(barcode_t * p_bc, bool b_black,
                              uint8_t * p_msg);
barcode_result_t barcode_step_travel(barcode_t * p_bc, bool b_black,
                                     uint16_t travel_16th_mm,
                                     uint8_t * p_msg);
char             barcode_index_to_char(uint8_t index);
subsystem1_cmd_t barcode_char_to_cmd(char symbol);

/* Runtime character-to-movement table, set over MQTT before the demo. */
void             barcode_map_set(uint8_t index, subsystem1_cmd_t cmd);
subsystem1_cmd_t barcode_map_get(uint8_t index);

#endif /* BARCODE_H */

/*** end of file ***/
