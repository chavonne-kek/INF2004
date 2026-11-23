/**
 * @file b5_scan.h
 * @brief Two-stage adaptive scanning: coarse sweep, then fine sweep on demand.
 */
#ifndef B5_SCAN_H
#define B5_SCAN_H

#include <stdint.h>
#include "b5_types.h"

/** Stage 1: fixed angles 30/60/90/120/150. out must hold B5_COARSE_COUNT. */
uint8_t B5_Scan_Coarse(B5_Sample_t *out);

/**
 * Stage 2: higher-resolution sweep of the window around the coarse hits.
 * out must hold B5_FINE_MAX_SAMPLES. Returns the sample count, 0 if the
 * coarse data held no obstacle. Samples are in ascending angle order.
 */
uint8_t B5_Scan_Fine(const B5_Sample_t *coarse, uint8_t coarseCount, B5_Sample_t *out);

#endif /* B5_SCAN_H */
