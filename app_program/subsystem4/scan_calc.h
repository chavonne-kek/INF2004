/** @file scan_calc.h
 *
 * @brief Telemetry helpers for subsystem 4: echo width to the 7-bit range,
 *        5-bit scan index to angle, and the packed 35-bit obstacle profile.
 *        (Scanning, profiling and planning are Buddy 5's b5_*.c.)  Pure
 *        integer logic, tested on a PC (testing/subsystem4/host).
 */

#ifndef SCAN_CALC_H
#define SCAN_CALC_H

#include <stdint.h>
#include <stdbool.h>

#include "subsystem4.h"

#define SCAN_ANGLE_MIN_DEG     30u
#define SCAN_STEP_DEG          5u
#define SCAN_US_PER_CM         58u     /* HC-SR04 datasheet.             */

uint8_t  scan_echo_to_cm(uint32_t echo_us);
uint16_t scan_index_to_deg(uint8_t index);
void     scan_profile_pack(subsystem4_profile_t const * p_profile,
                           uint8_t bytes[S4_PROFILE_BYTES]);
void     scan_profile_unpack(uint8_t const bytes[S4_PROFILE_BYTES],
                             subsystem4_profile_t * p_profile);

#endif /* SCAN_CALC_H */

/*** end of file ***/
