/**
 * @file b5_plan.h
 * @brief Avoidance planner: Stop / Continue / Turn left / Turn right / Reverse.
 */
#ifndef B5_PLAN_H
#define B5_PLAN_H

#include <stdint.h>
#include "b5_types.h"

/**
 * Choose an action from an obstacle profile.
 * @param retries  reverse-and-reattempt count so far (0..B5_MAX_RETRIES).
 * Plan.shiftCm is the sideways displacement the bypass must achieve.
 */
B5_Plan_t B5_Plan_Decide(const B5_Profile_t *profile, uint8_t retries);

#endif /* B5_PLAN_H */
