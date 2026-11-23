/**
 * @file b5_plan.c
 * @brief Avoidance planner (pure logic).
 *
 * Passing on the left means the car centre-line must end up at least
 * (half car + margin) left of the obstacle's left edge; same on the right.
 * The side needing the smaller sideways shift (least deviation from the
 * line) wins, provided the side clearance fits the car.
 */
#include "b5_plan.h"
#include "b5_config.h"
#include "b5_profile.h"

#define NEEDED_CM       ((int16_t)((B5_CAR_WIDTH_CM / 2u) + B5_SAFETY_MARGIN_CM))
#define SIDE_CLEAR_MIN  ((uint8_t)(B5_CAR_WIDTH_CM + (2u * B5_SAFETY_MARGIN_CM)))

static B5_Plan_t MakePlan(B5_Action_t action, int16_t shiftCm)
{
    B5_Plan_t plan;

    plan.action = (uint8_t)((uint8_t)action & 7u);
    plan.shiftCm = (uint8_t)((shiftCm < 0) ? 0 : ((shiftCm > 255) ? 255 : shiftCm));
    return plan;
}

static B5_Plan_t Retreat(uint8_t retries)
{
    return MakePlan((retries < B5_MAX_RETRIES) ? B5_ACT_REVERSE : B5_ACT_STOP, 0);
}

B5_Plan_t B5_Plan_Decide(const B5_Profile_t *profile, uint8_t retries)
{
    int16_t centreX;
    int16_t halfWidth;
    int16_t shiftLeft;
    int16_t shiftRight;
    bool leftOk;
    bool rightOk;

    if (profile->detected == 0u)
    {
        return MakePlan(B5_ACT_CONTINUE, 0);
    }
    if (profile->closestDist <= B5_STOP_DIST_CM)
    {
        return Retreat(retries);     /* too close to manoeuvre: back off */
    }

    centreX = B5_Geom_LateralCm((uint8_t)profile->centreAngle, (uint8_t)profile->closestDist);

    /* 22 Nov car: an under-read width must not shrink the shift to nothing. */
    {
        uint16_t width = profile->width;

        if (width < B5_MIN_WIDTH_CM)
        {
            width = B5_DEFAULT_WIDTH_CM;
        }
        halfWidth = (int16_t)((width + 1u) / 2u);
    }

    /* Left: car centre-line must reach x <= (centreX - halfWidth) - NEEDED. */
    shiftLeft = (int16_t)(NEEDED_CM - (centreX - halfWidth));
    /* Right: car centre-line must reach x >= (centreX + halfWidth) + NEEDED. */
    shiftRight = (int16_t)((centreX + halfWidth) + NEEDED_CM);

    if ((shiftLeft <= 0) || (shiftRight <= 0))
    {
        return MakePlan(B5_ACT_CONTINUE, 0);   /* obstacle already clear of path */
    }

    leftOk = (profile->leftClear >= SIDE_CLEAR_MIN) && (shiftLeft <= (int16_t)B5_MAX_SHIFT_CM);
    rightOk = (profile->rightClear >= SIDE_CLEAR_MIN) && (shiftRight <= (int16_t)B5_MAX_SHIFT_CM);

    /* Always leave the line by at least B5_MIN_SHIFT_CM: a bare minimum of
     * sideways room, so a misread cannot make the bypass graze the box. */
    if (leftOk && (shiftLeft < (int16_t)B5_MIN_SHIFT_CM))
    {
        shiftLeft = (int16_t)B5_MIN_SHIFT_CM;
    }
    if (rightOk && (shiftRight < (int16_t)B5_MIN_SHIFT_CM))
    {
        shiftRight = (int16_t)B5_MIN_SHIFT_CM;
    }

    if (leftOk && rightOk)
    {
        const bool preferLeft = (shiftLeft < shiftRight) ||
                                ((shiftLeft == shiftRight) && (profile->leftClear >= profile->rightClear));

        return preferLeft ? MakePlan(B5_ACT_TURN_LEFT, shiftLeft)
                          : MakePlan(B5_ACT_TURN_RIGHT, shiftRight);
    }
    if (leftOk)
    {
        return MakePlan(B5_ACT_TURN_LEFT, shiftLeft);
    }
    if (rightOk)
    {
        return MakePlan(B5_ACT_TURN_RIGHT, shiftRight);
    }
    return Retreat(retries);
}
