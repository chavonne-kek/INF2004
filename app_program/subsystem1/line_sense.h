/** @file line_sense.h
 *
 * @brief Line sensing: calibration, normalisation, black/white decisions
 *        with hysteresis, the signed left-edge line error and the
 *        LINE_LOST, LINE_FOUND and JUNCTION events.  Private to
 *        subsystem 1.  Pure logic: no kernel calls, so it can be unit
 *        tested on target and on a PC.
 */

#ifndef LINE_SENSE_H
#define LINE_SENSE_H

#include <stdint.h>
#include <stdbool.h>

#include "subsystem1.h"

#define LINE_SENSORS           3u      /* One per Grove port, in GPIO order. */
#define LINE_LEFT              2u      /* Grove 7 / GP28: barcode arm, the
                                        * physically LEFT-most sensor.      */
#define LINE_CENTRE            1u      /* Grove 6 / GP27: rides the line edge. */
#define LINE_RIGHT             0u      /* Grove 5 / GP26: right of the line.  */

/* The arm doubles as the far-left line guard and as the barcode reader, so
 * the decoder needs the pin by name rather than by "left or right". */
#define LINE_BARCODE           LINE_LEFT

/* Smallest error worth remembering as "which side the line went".  Below
 * this the car is effectively centred, and storing it made every lost
 * sample commit to a hard turn (30 Oct). */
#define LINE_MEMORY_MM         12

/* Barcode reader filter (2 Nov): the arm's reading is averaged over this
 * many samples before its own black/white decision, which only the barcode
 * decoder uses.  8 x 1 ms: 0.16 mm of travel at 2 cm/s, 2 mm at 25 cm/s. */
#define LINE_BC_AVG_SAMPLES    (8u / S1_SAMPLE_MS)

#define LINE_ADC_MAX           4095u   /* 12-bit RP2040 ADC.     */
#define LINE_FULL_SCALE        1000u   /* Normalised: 0 white, 1000 black. */

/* Events reported by line_sense_update() (same values as the public
 * S1_EVT_ bits, so they can be passed straight to the event flag).
 */
#define LINE_EVT_LOST          S1_EVT_LINE_LOST
#define LINE_EVT_FOUND         S1_EVT_LINE_FOUND
#define LINE_EVT_JUNCTION      S1_EVT_JUNCTION

/* Calibration: raw ADC readings on white floor and on the black line. */
typedef struct
{
    uint16_t white[LINE_SENSORS];
    uint16_t black[LINE_SENSORS];
} line_cal_t;

/* Running minimum and maximum, collected while calibrating. */
typedef struct
{
    uint16_t lowest[LINE_SENSORS];
    uint16_t highest[LINE_SENSORS];
} line_cal_run_t;

/* State kept between samples. */
typedef struct
{
    bool     b_black[LINE_SENSORS];    /* Hysteresis output per sensor. */
    int8_t   last_error_mm;            /* Last error while on the line. */
    uint16_t lost_ms;                  /* Time with no sensor on line.  */
    uint8_t  left_mm;                  /* Travel with a left branch.    */
    uint8_t  arm_mm;                   /* Travel with the arm black.    */
    uint16_t bc_hist[LINE_BC_AVG_SAMPLES]; /* Last arm readings, 0..1000. */
    uint32_t bc_sum;                   /* Their sum.                    */
    uint8_t  bc_next;                  /* Oldest entry.                 */
    bool     b_bc_black;               /* Filtered arm: barcode decoder. */
    uint16_t bc_level;                 /* 18 Nov: filtered arm, raw
                                        * (for the barcode trace).      */
    uint16_t bc_min;                   /* raw arm envelope: white end   */
    uint16_t bc_max;                   /* raw arm envelope: ink end     */
    uint8_t  bc_sensor;                /* Input the decoder reads (normally
                                        * LINE_BARCODE; the bench picks the
                                        * one the sensor is plugged into). */
    uint8_t  step_mm;                  /* Travel since the last sample. */
    uint8_t  branch_mm;                /* Travel with a right branch.   */
    uint8_t  dark_mm;                  /* Travel with the centre fully
                                        * on the line (17 Nov ramp).    */
    uint8_t  guard_mm;                 /* Travel with only the right
                                        * guard on the line (ramp).     */
    bool     b_lost;                   /* LINE_LOST is active.          */
    bool     b_junction;               /* JUNCTION is active.           */
} line_sense_t;

void     line_sense_init(line_sense_t * p_sense);
void     line_sense_travel(line_sense_t * p_sense, uint8_t travel_mm);
void     line_sense_cal_default(line_cal_t * p_cal);
void     line_sense_cal_start(line_cal_run_t * p_run);
void     line_sense_cal_track(line_cal_run_t * p_run,
                              uint16_t const raw[LINE_SENSORS]);
bool     line_sense_cal_apply(line_cal_run_t const * p_run,
                              line_cal_t * p_cal);
bool     line_sense_cal_apply_one(line_cal_run_t const * p_run,
                                  line_cal_t * p_cal, uint8_t sensor);
uint16_t line_sense_normalise(uint16_t raw, uint16_t white, uint16_t black);
uint8_t  line_sense_update(line_sense_t * p_sense, line_cal_t const * p_cal,
                           uint16_t const raw[LINE_SENSORS],
                           uint16_t * p_word);
uint16_t line_sense_pack(subsystem1_line_t const * p_line);
void     line_sense_unpack(uint16_t word, subsystem1_line_t * p_line);

void line_sense_hold(bool b_on);
void line_sense_bias(int8_t bias_mm);

#endif /* LINE_SENSE_H */

/*** end of file ***/
