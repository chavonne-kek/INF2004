/** @file scan_calc.c
 *
 * @brief Telemetry helpers for subsystem 4 (see scan_calc.h).  Pure logic.
 *
 * Angle convention (Buddy 5, b5_config.h): 0 deg = robot's RIGHT, 90 =
 * straight ahead, 180 = LEFT; scan index 0..24 = 30..150 deg.
 */

#include "scan_calc.h"

#define SCAN_FIELD_MASK        0x7Fu
#define SCAN_INDEX_MASK5       0x1Fu



/*!
 * @brief Echo pulse width to range: 7 bits, 127 = nothing in range.
 */
uint8_t
scan_echo_to_cm (uint32_t echo_us)
{
    uint32_t cm = echo_us / SCAN_US_PER_CM;

    return ((uint8_t)((cm >= S4_RANGE_NONE) ? S4_RANGE_NONE : cm));
}

/*!
 * @brief Scan index (0..24) to servo angle (30..150 degrees).
 */
uint16_t
scan_index_to_deg (uint8_t index)
{
    uint8_t limited = (index > S4_SCAN_INDEX_MAX) ? S4_SCAN_INDEX_MAX : index;

    return ((uint16_t)(SCAN_ANGLE_MIN_DEG + ((uint32_t)limited *
                                             SCAN_STEP_DEG)));
}

/*!
 * @brief Pack the 35-bit obstacle profile into 5 bytes, little-endian.
 */
void
scan_profile_pack (subsystem4_profile_t const * p_profile,
                   uint8_t bytes[S4_PROFILE_BYTES])
{
    uint64_t bits  = 0u;
    uint8_t  index = 0u;

    bits |= (uint64_t)(p_profile->width_cm & SCAN_FIELD_MASK);
    bits |= (uint64_t)(p_profile->closest_cm & SCAN_FIELD_MASK) << 7;
    bits |= (uint64_t)(p_profile->closest_index & SCAN_INDEX_MASK5) << 14;
    bits |= (uint64_t)(p_profile->left_clear_cm & SCAN_FIELD_MASK) << 19;
    bits |= (uint64_t)(p_profile->right_clear_cm & SCAN_FIELD_MASK) << 26;
    bits |= (uint64_t)(p_profile->b_left_edge ? 1u : 0u) << 33;
    bits |= (uint64_t)(p_profile->b_right_edge ? 1u : 0u) << 34;

    for (index = 0u; index < S4_PROFILE_BYTES; index++)
    {
        bytes[index] = (uint8_t)(bits >> (8u * index));
    }
}

/*!
 * @brief Unpack the obstacle profile.
 */
void
scan_profile_unpack (uint8_t const bytes[S4_PROFILE_BYTES],
                     subsystem4_profile_t * p_profile)
{
    uint64_t bits  = 0u;
    uint8_t  index = 0u;

    for (index = 0u; index < S4_PROFILE_BYTES; index++)
    {
        bits |= (uint64_t)bytes[index] << (8u * index);
    }

    p_profile->width_cm       = (uint8_t)(bits & SCAN_FIELD_MASK);
    p_profile->closest_cm     = (uint8_t)((bits >> 7) & SCAN_FIELD_MASK);
    p_profile->closest_index  = (uint8_t)((bits >> 14) & SCAN_INDEX_MASK5);
    p_profile->left_clear_cm  = (uint8_t)((bits >> 19) & SCAN_FIELD_MASK);
    p_profile->right_clear_cm = (uint8_t)((bits >> 26) & SCAN_FIELD_MASK);
    p_profile->b_left_edge    = (0u != ((bits >> 33) & 1u));
    p_profile->b_right_edge   = (0u != ((bits >> 34) & 1u));
}

/*** end of file ***/
