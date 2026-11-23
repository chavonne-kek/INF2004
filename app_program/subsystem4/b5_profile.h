/**
 * @file b5_profile.h
 * @brief Obstacle profiling from scan data: location, width, closest point,
 *        left/right clearance.
 *
 * Coordinates: angle 0 = car's right, 90 = straight ahead, 180 = car's left.
 * Lateral x is positive to the RIGHT, forward y is positive ahead.
 */
#ifndef B5_PROFILE_H
#define B5_PROFILE_H

#include <stdint.h>
#include <stdbool.h>
#include "b5_types.h"

int16_t B5_Geom_LateralCm(uint8_t angleDeg, uint8_t distCm);
int16_t B5_Geom_ForwardCm(uint8_t angleDeg, uint8_t distCm);

/** True if the reading is a real echo inside the corridor and detect range. */
bool B5_Profile_IsObstacle(uint8_t angleDeg, uint8_t distCm);

/**
 * Build a profile from samples sorted by ascending angle.
 * @param stepDeg  angular spacing of the scan (for edge-width correction).
 * @return true if an obstacle was found (profile->detected is set too).
 */
bool B5_Profile_Build(const B5_Sample_t *samples, uint8_t count,
                      uint8_t stepDeg, B5_Profile_t *profile);

/* Bits returned by B5_Profile_OpenEdges(). */
#define B5_EDGE_OPEN_RIGHT  (0x01u)   /* object still there at the first sample */
#define B5_EDGE_OPEN_LEFT   (0x02u)   /* object still there at the last sample  */

/**
 * IS25 17 Nov: which sides of the closest object run to the end of the scan,
 * so its real edge there was not seen (the servo only reaches 60..115 deg on
 * the car).  0 = both edges seen, or no object.
 */
uint8_t B5_Profile_OpenEdges(const B5_Sample_t *samples, uint8_t count);

#endif /* B5_PROFILE_H */
