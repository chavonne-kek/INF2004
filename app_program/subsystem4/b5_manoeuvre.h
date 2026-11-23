/**
 * @file b5_manoeuvre.h
 * @brief Obstacle bypass, reverse-and-retry and line recovery movements.
 */
#ifndef B5_MANOEUVRE_H
#define B5_MANOEUVRE_H

#include <stdbool.h>
#include <stdint.h>
#include "b5_if.h"
#include "b5_types.h"

void B5_Manoeuvre_Init(const B5_Interfaces_t *io);

/** Stage 4 execution: go round the obstacle on `side`, then head back to line. */
bool B5_Manoeuvre_Bypass(B5_Side_t side, uint8_t shiftCm, uint8_t closestDistCm);

/** Back off so the scan can be retried from further away. */
bool B5_Manoeuvre_Reverse(void);

/** Stage 5: search for the original line. `bypassedSide` is where we went.
 *  IS25 18 Nov: `lineSide` (may be NULL) gets the side of the car the line
 *  is on when it was found. */
bool B5_Manoeuvre_RecoverLine(B5_Side_t bypassedSide, B5_Side_t *lineSide);

#endif /* B5_MANOEUVRE_H */
