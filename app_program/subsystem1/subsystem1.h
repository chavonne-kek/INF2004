/** @file subsystem1.h
 *
 * @brief Subsystem 1 (Buddy 3): IR line following and Code 39 barcode
 *        decoding.  This is the PUBLIC interface; only what the other
 *        subsystems may use is declared here.
 *
 * Owner: Hao Ming (student ID 2500331).
 *
 * Other subsystems include it as "../subsystem1/subsystem1.h".
 *
 * Interfaces provided (TEAM.md section 5):
 *   IF-01  S1 -> S2  line state, read with subsystem1_line_read().
 *   IF-03  S1 -> S4  barcode messages in the message buffer returned by
 *                    subsystem1_barcode_mbf(), plus event bits set in the
 *                    controller's event flag (see subsystem1_bind_events()).
 *
 * Minimum-bit data (Change Requirement 1):
 *   Line state     one 16-bit word: error 7 bits + 3 flag bits.
 *   Barcode result one byte: character index 6 bits + direction 1 bit.
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include <stdint.h>
#include <stdbool.h>

#include "../app.h"

/* Task, timing and ADC channels (claimed in TEAM.md sections 2 and 4). */
#define S1_PRI_LINE            12      /* Priority band 12-13. */
#define S1_SAMPLE_MS           1u      /* IR sampling period, ms (2 Nov:
                                        * 2 -> 1, twice the samples per
                                        * bar for the barcode debounce).  */
#define S1_ADC_CH_LEFT         0u      /* GP26 (ADC0): outer line sensor.  */
#define S1_ADC_CH_CENTRE       1u      /* GP27 (ADC1): edge line sensor.   */
#define S1_ADC_CH_RIGHT        2u      /* GP28 (ADC2): barcode sensor.     */

/* Which side of the line the car tracks (30 Oct, as built: two line
 * sensors side by side, the barcode sensor further out on one side).
 *   GP26 (Grove 5) = the OUTER line sensor, away from the barcode side
 *   GP27 (Grove 6) = the INNER line sensor, which rides on the line edge
 *   GP28 (Grove 7) = the barcode sensor
 * 0: barcode sensor on the car's RIGHT -> the inner sensor rides on the
 *    line's LEFT edge (the design so far).
 * 1: barcode sensor on the car's LEFT -> it rides on the RIGHT edge; the
 *    line error sign is flipped so the steering is still right.
 * Seen from behind the car, driving away from you.  The S1 check image
 * shows whether it is right: line under the outer sensor = "steer"
 * toward that side. */
#define S1_TRACK_RIGHT_EDGE    0

/* Event bits set in the controller's event flag.  The values must match
 * the bit allocation agreed in TEAM.md section 5 (flg_events).
 */
#define S1_EVT_BARCODE         (1u << 0)
#define S1_EVT_LINE_LOST       (1u << 1)
#define S1_EVT_LINE_FOUND      (1u << 2)
#define S1_EVT_JUNCTION        (1u << 3)

/* Line error range.  Positive error = steer LEFT, negative = steer RIGHT. */
#define S1_LINE_ERR_MAX_MM     40

/* Packed line-state word (IF-01), 16 bits:
 *   bits 0-6  line error + S1_LINE_ERR_OFFSET (0..80 means -40..+40 mm)
 *   bit  7    valid: at least one sensor sees the line
 *   bit  8    lost: no sensor has seen the line for 300 ms
 *   bit  9    junction: left and centre sensors both on black
 *   bits 10-15 spare (always 0)
 */
#define S1_LINE_ERR_OFFSET     40u
#define S1_LINE_ERR_MASK       0x007Fu
#define S1_LINE_VALID_BIT      (1u << 7)
#define S1_LINE_LOST_BIT       (1u << 8)
#define S1_LINE_JUNCTION_BIT   (1u << 9)
/* 22 Nov: the arm (barcode) sensor state was in the struct but never in the
 * word, so subsystem1_line_read() handed TSK_CTRL an uninitialised b_arm -
 * the barcode lock-out then read leftover stack memory: it blocked real
 * boxes on some runs and let a stop through mid-barcode on others. */
#define S1_LINE_ARM_BIT        (1u << 10)

/* Packed barcode message (IF-03), one byte:
 *   bits 0-5  index into the Code 39 character set (0..42)
 *   bit  6    1 = barcode was read in the reverse direction
 *   bit  7    spare (always 0)
 */
/* Raw (un-normalised) IR readings, arm/centre/right, for telemetry. */
void     subsystem1_line_raw(uint16_t out[3]);
void     subsystem1_barcode_reset(void);   /* drop a half-finished read */

#define S1_BC_INDEX_MASK       0x3Fu
#define S1_BC_REVERSED_BIT     (1u << 6)
/* Bit 7: the message is a rejection diagnostic (the reason is in bits 0..2),
 * not a decoded character.  It travels to the console and to
 * robot/telemetry/barcode so a failed read can be diagnosed without USB. */
#define S1_BC_REJECT_BIT       (1u << 7)
#define S1_BC_MSG_SIZE         1u      /* Bytes per message. */

/* Unpacked line state, for the reader's convenience. */
typedef struct
{
    int8_t  error_mm;       /* -40..+40; positive = steer left.   */
    bool    b_valid;        /* At least one sensor sees the line. */
    bool    b_lost;         /* LINE_LOST is active.               */
    bool    b_junction;     /* JUNCTION condition is active.      */
    bool    b_arm;          /* Arm (barcode) sensor sees black.   */
} subsystem1_line_t;

/* End-of-course barcode (Week 6 system statechart: "the end of the course
 * is marked (end barcode or remote STOP)").  TODO(Hao Ming): confirm the
 * character with the lecturer and set it here, e.g. 'E'.  '\0' = no end
 * barcode: unknown characters are only reported, never acted on. */
#define S1_BARCODE_END_CHAR    '\0'

/* Navigation commands carried by barcodes (Project Write-up). */
typedef enum
{
    SUBSYSTEM1_CMD_NONE = 0,    /* Valid barcode, but not a command. */
    SUBSYSTEM1_CMD_LEFT,        /* 'A' */
    SUBSYSTEM1_CMD_RIGHT,       /* 'B' */
    SUBSYSTEM1_CMD_STRAIGHT,    /* 'C' */
    SUBSYSTEM1_CMD_UTURN,       /* 'D' */
    SUBSYSTEM1_CMD_END          /* S1_BARCODE_END_CHAR, if set. */
} subsystem1_cmd_t;

/* Diagnostics, for tests and telemetry. */
typedef struct
{
    uint16_t raw[3];            /* Last ADC readings, 0..4095.        */
    uint32_t samples;           /* Samples taken since start.         */
    uint16_t overruns;          /* Ticks missed because a sample ran late. */
    uint16_t adc_errors;        /* Failed ADC reads.                  */
    uint16_t barcodes;          /* Barcodes decoded.                  */
    uint16_t barcode_errors;    /* Barcode reads rejected.            */
    uint16_t mbf_drops;         /* Barcode messages dropped (buffer full). */
    uint16_t exec_max_us;       /* Worst sample time, us (NFR1: <= 500). */
    uint16_t cal_span[3];       /* Highest - lowest reading of each sensor
                                   in the last calibration sweep (needs
                                   >= 400 to be applied).             */
    bool     b_adc_open;        /* The ADC device opened correctly.   */
    bool     b_calibrated;      /* A calibration has been applied.    */
} subsystem1_diag_t;

/* The kernel calls subsystem1_init() and subsystem1_start(); they are
 * declared in app.h.
 */

void             subsystem1_bind_events(ID flg_id);
bool             subsystem1_line_read(subsystem1_line_t * p_line);
ID               subsystem1_barcode_mbf(void);
char             subsystem1_barcode_char(uint8_t msg);
subsystem1_cmd_t subsystem1_barcode_cmd(uint8_t msg);
bool             subsystem1_barcode_is_reversed(uint8_t msg);
void             subsystem1_calibrate_begin(void);
bool             subsystem1_calibrate_end(void);
void             subsystem1_diag_get(subsystem1_diag_t * p_diag);
uint32_t         subsystem1_line_seen_count(void);   /* 18 Nov */
bool             subsystem1_line_on(void);           /* 18 Nov */
void             subsystem1_line_hint(int8_t error_mm);
void     subsystem1_line_hold(bool b_on);
bool     subsystem1_arm_black(void);
bool     subsystem1_track_black(void);
void     subsystem1_line_bias(int8_t bias_mm);
bool     subsystem1_track_both_black(void); /* 18 Nov */
bool     subsystem1_left_black(void);       /* 20 Nov: GP27 on black */
bool     subsystem1_right_black(void);      /* 20 Nov: GP26 on black */

#endif /* SUBSYSTEM1_H */

/*** end of file ***/
