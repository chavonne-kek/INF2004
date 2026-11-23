/**
 * @file b5_manoeuvre.c
 * @brief Bypass and recovery movements built on Buddy 2's motion API.
 *
 * Bypass is a trapezoid: turn 45 deg out, drive the diagonal to gain the
 * lateral shift, turn parallel, drive alongside the obstacle while the servo
 * looks at it sideways (so the pass length adapts to the real obstacle depth),
 * then mirror the manoeuvre back toward the line.
 */
/* IS25 18 Nov: <math.h> removed (unused; its size_t clashes with tk/syslib.h
 * once b5_port.h is included for B5_LOG). */
#include "b5_manoeuvre.h"
#include "b5_config.h"
#include "b5_servo.h"
#include "b5_sonar.h"
#include "b5_profile.h"   /* IS25: B5_Geom_* for the diagonal look */
#include "b5_port.h"      /* IS25: B5_LOG */

#define SQRT2_X1000     (1414u)
#define PERMILLE        (1000u)
#define SEEN_CONFIRM    (2u)

static const B5_Interfaces_t *s_io;

void B5_Manoeuvre_Init(const B5_Interfaces_t *io)
{
    s_io = io;
}

static B5_Side_t Opposite(B5_Side_t side)
{
    return (side == B5_SIDE_LEFT) ? B5_SIDE_RIGHT : B5_SIDE_LEFT;
}

static bool SweepFor(B5_Side_t dir, uint16_t totalDeg);

static bool Turn(B5_Side_t side, uint16_t deg)
{
    return (side == B5_SIDE_LEFT) ? s_io->turnLeft(deg) : s_io->turnRight(deg);
}

/* Drive in short steps and look ahead before each, so a surprise object
 * (or the obstacle itself) cannot be hit. */
static bool GuardedForward(uint16_t cm, bool stopAtLine, bool *lineSeen)
{
    uint16_t done = 0u;

    while (done < cm)
    {
        const uint16_t step = ((uint16_t)(cm - done) < B5_STEP_CM) ? (uint16_t)(cm - done)
                                                                    : (uint16_t)B5_STEP_CM;

#if B5_BYPASS_SONAR_GUARD
        if (B5_Sonar_MeasureCm() <= B5_STOP_DIST_CM)
        {
            s_io->stop();
            B5_Status_RaiseFault(B5_FAULT_BYPASS_ABORT);
            return false;
        }
#endif
        if (!s_io->moveForward(step))
        {
            B5_Status_RaiseFault(B5_FAULT_BYPASS_ABORT);
            return false;
        }
        done = (uint16_t)(done + step);

        if (stopAtLine && s_io->lineDetected())
        {
            *lineSeen = true;
            return true;
        }
    }
    return true;
}

/* Advance beside the obstacle until the side-looking sonar no longer sees it.
 *
 * IS25 17 Nov: on the car the servo only turns about 30 deg right and 25 deg
 * left of straight ahead (b5_config.h), so it looks forward-diagonally, not
 * sideways.  A reading counts as the obstacle when it is within
 * B5_SIDE_SEEN_CM to the side and B5_DETECT_RANGE_CM ahead.  Because the
 * beam meets the obstacle ahead of the car, the car drives on by how far
 * ahead the last hit was (less the step already driven) before the
 * car length that lets the tail clear. */
static bool PassObstacle(B5_Side_t side, uint8_t closestDistCm)
{
    /* Passing on the left puts the obstacle on our right, and vice versa. */
    const uint8_t lookDeg = (side == B5_SIDE_LEFT) ? B5_SERVO_RIGHT_DEG : B5_SERVO_LEFT_DEG;
    uint16_t travelled = 0u;
    uint16_t lastAheadCm = 0u;
    uint8_t clearCount = 0u;
    bool seen = false;
    bool ok = true;

    B5_Servo_MoveTo(lookDeg);

    while ((travelled < B5_PASS_MAX_CM) && (clearCount < SEEN_CONFIRM) && ok)
    {
        const uint8_t d = B5_Sonar_MeasureCm();
        bool hit = false;

        if ((d < B5_DIST_MAX_CM) && (d > B5_SELF_ECHO_CM))   /* not the car */
        {
            int16_t lateral = B5_Geom_LateralCm(lookDeg, d);
            const int16_t ahead = B5_Geom_ForwardCm(lookDeg, d);

            if (lateral < 0)
            {
                lateral = (int16_t)-lateral;
            }
            hit = ((uint16_t)lateral <= B5_SIDE_SEEN_CM) &&
                  (ahead >= 0) && ((uint16_t)ahead <= B5_DETECT_RANGE_CM);
            if (hit)
            {
                lastAheadCm = (uint16_t)ahead;
            }
        }

        if (hit)
        {
            seen = true;
            clearCount = 0u;
        }
        else if (seen || (travelled >= closestDistCm))
        {
            clearCount++;       /* passed the far edge (or never saw it) */
        }
        else
        {
            /* obstacle not alongside yet */
        }

        if (clearCount < SEEN_CONFIRM)
        {
            ok = s_io->moveForward(B5_STEP_CM);
            travelled = (uint16_t)(travelled + B5_STEP_CM);
        }
    }

    B5_Servo_MoveTo(B5_SERVO_FRONT_DEG);
    if (!ok)
    {
        B5_Status_RaiseFault(B5_FAULT_BYPASS_ABORT);
        return false;
    }
    /* The beam met the obstacle lastAheadCm ahead of the sonar; one step of
     * that was already driven after the first clear reading. */
    lastAheadCm = (lastAheadCm > B5_STEP_CM) ? (uint16_t)(lastAheadCm - B5_STEP_CM) : 0u;
    B5_LOG("S4: passed it after %u cm (%s), %u cm more so the tail clears\n",
           (unsigned)travelled, seen ? "seen beside" : "never seen beside",
           (unsigned)(B5_CAR_LENGTH_CM + lastAheadCm));
    /* The sonar sits at the front: drive one car length so the tail clears.
     * IS25 18 Nov: in 5 cm guarded steps.  One long move (20..60 cm at
     * 2-3 cm/s) could outlast TSK_CTRL's 9 s limit per move, which stops
     * it and fails the whole bypass. */
    {
        bool unused = false;

        return GuardedForward((uint16_t)(B5_CAR_LENGTH_CM + lastAheadCm), false,
                              &unused);
    }
}

/*!
 * @brief Fixed-geometry bypass (22 Nov): tuned once for the demo obstacle.
 *
 *        The scan-driven path could not be made repeatable with this sonar,
 *        so the sideways move is a fixed pair of legs and only the way back
 *        is sensor-driven: small turn steps with a creep between them, ending
 *        the moment a track sensor finds the line.  That is what stops the
 *        car parking beside the line instead of on it.
 */
#if B5_FIXED_BYPASS
static bool BypassFixed(B5_Side_t side)
{
    bool lineSeen = false;

    B5_Servo_MoveTo(B5_SERVO_FRONT_DEG);

    /* 1. Away from the line, and far enough out to pass the box. */
    if (!Turn(side, B5_FIXED_OUT_DEG) ||
        !GuardedForward(B5_FIXED_OUT_CM, false, &lineSeen))
    {
        s_io->stop();
        B5_Status_RaiseFault(B5_FAULT_BYPASS_ABORT);
        return false;
    }
    B5_LOG("S4: fixed bypass: out %u deg, %u cm\n",
           (unsigned)B5_FIXED_OUT_DEG, (unsigned)B5_FIXED_OUT_CM);

    /* 2. Back to parallel, still beside the box. */
    if (!Turn(Opposite(side), B5_FIXED_OUT_DEG))
    {
        s_io->stop();
        B5_Status_RaiseFault(B5_FAULT_BYPASS_ABORT);
        return false;
    }

    /* 3. Past the box. */
    if (!GuardedForward(B5_FIXED_PASS_CM, false, &lineSeen))
    {
        s_io->stop();
        B5_Status_RaiseFault(B5_FAULT_BYPASS_ABORT);
        return false;
    }
    B5_LOG("S4: fixed bypass: passed %u cm, turning back to the line\n",
           (unsigned)B5_FIXED_PASS_CM);

    /* 4. 45 deg toward the line, 20 cm across it, then 45 deg to square
     * up (left, back to parallel). */
    if (!Turn(Opposite(side), B5_FIXED_OUT_DEG) ||
        !GuardedForward(B5_FIXED_RETURN_CM, false, &lineSeen))
    {
        s_io->stop();
        B5_Status_RaiseFault(B5_FAULT_BYPASS_ABORT);
        return false;
    }
    B5_LOG("S4: fixed bypass: %u cm across, squaring up %u deg\n",
           (unsigned)B5_FIXED_RETURN_CM, (unsigned)B5_FIXED_OUT_DEG);
    if (s_io->lineDetected())
    {
        B5_LOG("S4: fixed bypass: line found on the way in after %u cm\n",
               (unsigned)B5_FIXED_RETURN_CM);
        return true;
    }
    if (!Turn(side, B5_FIXED_OUT_DEG))
    {
        s_io->stop();
        B5_Status_RaiseFault(B5_FAULT_BYPASS_ABORT);
        return false;
    }
    if (s_io->lineDetected())
    {
        B5_LOG("S4: fixed bypass: line found after squaring up %u deg\n",
               (unsigned)B5_FIXED_OUT_DEG);
        return true;
    }

    /* 5. On the small track there is no room to hunt: rotate on the spot. */
    if (SweepFor(Opposite(side), B5_FIXED_SWEEP_DEG) || s_io->lineDetected())
    {
        B5_LOG("S4: fixed bypass: line found by sweeping up to %u deg\n",
               (unsigned)B5_FIXED_SWEEP_DEG);
        return true;
    }

    B5_LOG("S4: fixed bypass: line not found within %u deg\n",
           (unsigned)B5_FIXED_SWEEP_DEG);
    s_io->stop();
    B5_Status_RaiseFault(B5_FAULT_LINE_LOST);
    return false;
}
#endif

bool B5_Manoeuvre_Bypass(B5_Side_t side, uint8_t shiftCm, uint8_t closestDistCm)
{
    const uint16_t diagonal = (uint16_t)(((uint32_t)shiftCm * SQRT2_X1000) / PERMILLE);
    bool lineSeen = false;

#if B5_FIXED_BYPASS
    (void)diagonal;
    (void)closestDistCm;
    return BypassFixed(side);
#else
    B5_Servo_MoveTo(B5_SERVO_FRONT_DEG);

    if (!Turn(side, B5_BYPASS_TURN_DEG) ||
        !GuardedForward(diagonal, false, &lineSeen) ||
        !Turn(Opposite(side), B5_BYPASS_TURN_DEG) ||
        !PassObstacle(side, closestDistCm) ||
        !Turn(Opposite(side), B5_BYPASS_TURN_DEG))
    {
        s_io->stop();
        B5_Status_RaiseFault(B5_FAULT_BYPASS_ABORT);
        return false;
    }

#if B5_REJOIN_BY_CTRL
    /* IS25 20 Nov: heading back toward the line at 45 deg; TSK_CTRL drives
     * the rest (b5_config.h B5_REJOIN_BY_CTRL). */
    (void)lineSeen;
    B5_LOG("S4: heading back: %s\n", "TSK_CTRL rejoins the line");
    return true;
#endif

    /* Head back toward the line; stop as soon as an IR sensor sees it. */
    if (!GuardedForward((uint16_t)(diagonal + B5_RETURN_OVERSHOOT_CM), true, &lineSeen))
    {
        s_io->stop();
        return false;
    }
    B5_LOG("S4: heading back: %s\n",
           lineSeen ? "line found, squaring up" : "line not found yet, searching");
    if (lineSeen)
    {
        (void)Turn(side, B5_BYPASS_TURN_DEG);   /* square up with the line */
    }
    return true;   /* RECOVER state confirms the line is really there */
#endif
}

bool B5_Manoeuvre_Reverse(void)
{
    return s_io->moveBackward(B5_REVERSE_CM);
}

/* Rotate in 10 degree steps, checking the line after each. */
static bool SweepFor(B5_Side_t dir, uint16_t totalDeg)
{
    uint16_t turned;

    for (turned = 0u; turned < totalDeg; turned = (uint16_t)(turned + B5_RECOVER_SWEEP_STEP_DEG))
    {
        if (!Turn(dir, B5_RECOVER_SWEEP_STEP_DEG))
        {
            return false;
        }
        if (s_io->lineDetected())
        {
            return true;
        }
    }
    return false;
}

bool B5_Manoeuvre_RecoverLine(B5_Side_t bypassedSide, B5_Side_t *lineSide)
{
    /* The line lies on the side opposite to where we detoured. */
    const B5_Side_t toLine = Opposite(bypassedSide);
    bool lineSeen = false;

    if (lineSide != NULL)
    {
        *lineSide = toLine;
    }
    if (s_io->lineDetected())
    {
        return true;
    }

    /* 1. Creep forward a little: the line may be just ahead. */
    if (GuardedForward(B5_RECOVER_CREEP_CM, true, &lineSeen) && lineSeen)
    {
        return true;
    }

    /* 2. Sweep toward the expected line side, then back and the other way. */
    if (SweepFor(toLine, B5_RECOVER_SWEEP_DEG))
    {
        return true;
    }
    (void)Turn(bypassedSide, B5_RECOVER_SWEEP_DEG);          /* undo sweep */
    if (SweepFor(bypassedSide, B5_RECOVER_SWEEP_DEG))
    {
        if (lineSide != NULL)
        {
            *lineSide = bypassedSide;                        /* found there */
        }
        return true;
    }
    (void)Turn(toLine, B5_RECOVER_SWEEP_DEG);                /* restore heading */

    s_io->stop();
    B5_Status_RaiseFault(B5_FAULT_LINE_LOST);
    return false;
}
