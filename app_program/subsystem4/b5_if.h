/**
 * @file b5_if.h
 * @brief Interfaces Buddy 5 needs from the other buddies.
 *
 * Fill this struct once at start-up with thin wrappers around Buddy 2's motion
 * API and Buddy 3's line API. Buddy 5 never includes their headers, so the
 * subsystems stay decoupled.
 *
 * Motion calls are expected to BLOCK until the move finishes and return false
 * if it failed (stall, encoder fault, aborted).
 *
 * IS25 integration (26 Oct): in the team design TSK_CTRL is the ONLY task
 * that commands motion (Week 6, defended at Tribunal 1).  So the motion
 * calls are implemented in scan_task.c as requests to TSK_CTRL, which
 * forwards them to subsystem 2 and answers when MOTION_DONE (or a fault)
 * arrives.  lineFollowEnable() is replaced by takeControl() /
 * releaseControl(): TSK_CTRL brakes, can refuse (e.g. on a hump, or when
 * the robot is not running), and decides what happens after a failure.
 */
#ifndef B5_IF_H
#define B5_IF_H

#include <stdint.h>
#include <stdbool.h>
#include "b5_types.h"

typedef struct
{
    /* Buddy 2: motion control */
    bool (*moveForward)(uint16_t distanceCm);
    bool (*moveBackward)(uint16_t distanceCm);
    bool (*turnLeft)(uint16_t angleDeg);
    bool (*turnRight)(uint16_t angleDeg);
    void (*stop)(void);

    /* Buddy 3: line following */
    bool (*lineDetected)(void);              /* any IR sensor on the line now,
                                                or (IS25 18 Nov) seen at any
                                                time during the last move or
                                                turn */

    /* IS25: hand-over with the vehicle controller (TSK_CTRL) */
    bool (*takeControl)(void);               /* stop car, pause line follower;
                                                false = refused, keep watching */
    void (*releaseControl)(bool success);    /* true: resume line following;
                                                false: avoidance failed */

    /* Buddy 1: telemetry (may be NULL). Payload is B5_PROFILE_WIRE_BYTES. */
    void (*publishProfile)(const uint8_t *payload, uint8_t length);

    /* IS25 18 Nov (may be NULL): before handing back after a bypass, tell
     * the line follower which side of the car the line is on, for the
     * case it loses the line again before it has seen it properly. */
    void (*lineHint)(bool lineOnLeft);
} B5_Interfaces_t;

#endif /* B5_IF_H */
