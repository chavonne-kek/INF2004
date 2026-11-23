/**
 * @file b5_task.c
 * @brief Buddy 5 main state machine (8 states, 3 bits).
 *
 * IS25 integration (26 Oct): runs inside TSK_SCAN (subsystem 4, core 0,
 * priority 19), the owner of the HC-SR04 and the servo.  Changes, marked
 * "IS25": the hand-over with TSK_CTRL uses takeControl/releaseControl,
 * and a failed avoidance is reported to TSK_CTRL (system FAULT, Week 6)
 * instead of silently resuming line following.
 */
#include "b5_task.h"
#include "b5_config.h"
#include "b5_manoeuvre.h"
#include "b5_plan.h"
#include "b5_port.h"
#include "b5_profile.h"
#include "b5_scan.h"
#include "b5_servo.h"
#include "b5_sonar.h"

static const B5_Interfaces_t *s_io;

/* Working buffers owned by the Buddy 5 task. */
static B5_Sample_t s_coarse[B5_COARSE_COUNT];
static B5_Sample_t s_fine[B5_FINE_MAX_SAMPLES];
static uint8_t     s_fineCount;
static uint8_t     s_openEdges;    /* IS25: B5_EDGE_OPEN_* of the last fine scan */
static B5_Plan_t   s_plan;
static B5_Side_t   s_lastSide;
static uint8_t     s_retries;      /* 0..3 : 2 bits of range */

/* Shared with other tasks: guarded by a critical section. */
static B5_Profile_t s_profile;

static void StoreProfile(const B5_Profile_t *p)
{
    const B5_Irq_t irq = B5_Port_Enter();

    s_profile = *p;
    B5_Port_Exit(irq);
}

void B5_Task_GetProfile(B5_Profile_t *out)
{
    const B5_Irq_t irq = B5_Port_Enter();

    *out = s_profile;
    B5_Port_Exit(irq);
}

uint16_t B5_Task_GetStatus(void)
{
    return B5_Status_Get();
}

void B5_Task_Init(const B5_Interfaces_t *io)
{
    s_io = io;
    s_profile = (B5_Profile_t){0};   /* IS25: no <string.h> (size_t clash with tk/syslib.h) */
    B5_Status_Init();
    B5_Servo_Init();
    B5_Sonar_Init();
    B5_Manoeuvre_Init(io);
}

static void Publish(const B5_Profile_t *p)
{
    uint8_t wire[B5_PROFILE_WIRE_BYTES];

    StoreProfile(p);
    if (s_io->publishProfile != NULL)
    {
        B5_Profile_Pack(p, wire);
        s_io->publishProfile(wire, B5_PROFILE_WIRE_BYTES);
    }
}

static void HandBack(bool success)
{
    s_retries = 0u;
    B5_Status_SetRetries(0u);
    B5_Status_SetObstacle(false);
    B5_Servo_MoveTo(B5_SERVO_FRONT_DEG);
    s_io->releaseControl(success);              /* IS25 */
}

/* IS25 17 Nov night: after a false alarm (the scan found nothing), the
 * trigger stays off until the front reading has been clear once or
 * B5_REARM_MS has passed, so a reading that does not go away (the floor,
 * the car's own bracket) cannot stop the car again and again on the spot. */
#define B5_REARM_MS     (3000u)

static uint8_t s_hits;                        /* 0..B5_TRIGGER_CONFIRM */
static uint8_t s_shortHits;                   /* 0..B5_STOPGAP_CONFIRM (v94) */
static uint8_t s_watch_cm = B5_DIST_NO_ECHO;  /* last front reading, for S5 */

/* 22 Nov: the watch was only ever visible on the USB console.  On a battery
 * run there is no cable, so the same numbers go out over MQTT. */
uint8_t B5_WatchCm(void)
{
    return (s_watch_cm);
}

uint8_t B5_WatchHits(void)
{
    return ((s_hits > s_shortHits) ? s_hits : s_shortHits);
}
static bool    s_rearmWait;
static uint32_t s_rearmStartUs;

static void FalseAlarm(void)
{
    s_rearmWait = true;
    s_rearmStartUs = B5_Port_NowUs();
}

/* IDLE: line follower drives; we only watch straight ahead. */
static void WatchReset(void)
{
    s_hits       = 0u;
    s_shortHits  = 0u;
    s_rearmWait  = false;
}

static B5_State_t StateIdle(void)
{
    uint8_t front;

    /* 21 Nov car: a scan that was interrupted (pause, reset, a state change
     * in TSK_CTRL) left the servo pointing where the sweep stopped - the log
     * showed the watch seeing the box at 115 deg while the car drove
     * straight into it.  The cruise watch only ever looks ahead, so put the
     * servo back to the front before every reading. */
    if (B5_Servo_GetAngle() != B5_SERVO_FRONT_DEG)
    {
        B5_Servo_MoveTo(B5_SERVO_FRONT_DEG);
    }

    front = B5_Sonar_MeasureCm();
    s_watch_cm = front;

    if (s_rearmWait &&
        ((front > B5_TRIGGER_CM) ||
         ((B5_Port_NowUs() - s_rearmStartUs) >= (B5_REARM_MS * 1000u))))
    {
        s_rearmWait = false;                   /* armed again */
    }

    /* IS25 17 Nov night: a reading of B5_SELF_ECHO_CM or less is the car
     * itself, not an obstacle.
     *
     * 21 Nov car: this module (HC-SR04 on 3V3) drops pings - it alternates a
     * real 24 cm with a 1 cm self-echo.  Resetting on every reading meant
     * three in a row almost never happened, so the car avoided the box on
     * some runs and drove into it on others.  Now: a self-echo carries no
     * information and leaves the count alone, a clear reading costs one hit,
     * and only a genuinely open distance (trigger + 10) starts again. */
    if (s_rearmWait)
    {
        s_hits = 0u;
    }
    else if (front <= B5_NEAR_CM)
    {
        /* v94h: short readings are handled by the "1 cm" rule below. */
    }
    else if (front <= B5_SELF_ECHO_CM)
    {
        /* No information (own burst): keep the count as it is. */
    }
    else if (front <= B5_TRIGGER_CM)
    {
        if (s_hits < B5_TRIGGER_CONFIRM)
        {
            s_hits++;
        }
    }
    else if (front > (B5_TRIGGER_CM + 10u))
    {
        s_hits = 0u;
    }
    else if (s_hits > 0u)
    {
        s_hits--;                 /* One miss tolerated near the band. */
    }
    else
    {
        /* Already zero. */
    }

    /* v94 STOPGAP (23 Nov car, static test, HC-SR04 on 3.3 V): empty track
     * reads a steady 24 cm, the 13 cm box at 20 cm reads 3 cm, right in
     * front 1 cm - the module never reports the box's real distance.  A run
     * of B5_STOPGAP_CONFIRM readings of B5_STOPGAP_SHORT_CM or less in a row
     * is therefore taken as "box ahead".  Any other reading restarts the
     * count, so the 1 cm / 24 cm alternation seen on 21-22 Nov cannot fire
     * it.  Cost: a sensor stuck at 1-2 cm while driving looks the same. */
    /* v94h (23 Nov car, team decision): this module answers 2-3 cm for an
     * object that is still far away and 1 cm only when it is close.  Stop
     * only after B5_ONE_CM_COUNT readings of exactly 1 cm IN A ROW; any
     * other reading restarts the count.  (~0.12 s per reading, so 20 in a
     * row is ~2.4 s, ~7 cm of travel at 3 cm/s.) */
    if (s_rearmWait)
    {
        s_shortHits = 0u;
    }
    else if (1u == front)
    {
        if (s_shortHits < 0xFFu)
        {
            s_shortHits++;
        }
    }
    else if (s_shortHits > 0u)
    {
        /* Leaky counter, not a strict run.  This module alternates a 1 with
         * a 2, 3 or 24 on roughly one ping in two, and the log only shows
         * every other ping - a strict run resets invisibly and never fires.
         * One wrong reading now costs one step instead of the whole count. */
        s_shortHits--;
    }
    else
    {
        /* Already zero. */
    }

    /* 21 Nov car: the watch is invisible over MQTT (a profile is only
     * published after a scan), so a trigger that stops working after a
     * junction turn cannot be told apart from a dead sonar.  Print what the
     * watch sees whenever it changes, and every 2 s anyway. */
    {
        static uint8_t  last_front = 0xFFu;
        static uint32_t last_us    = 0u;
        const uint32_t  now_us     = B5_Port_NowUs();

        if ((front != last_front) || ((now_us - last_us) >= 2000000u))
        {
            last_front = front;
            last_us    = now_us;
            B5_LOG("S4: watch front %u cm (trigger <= %u), s_hits %u, "
                   "servo %u deg, rearm %u\n",
                   (unsigned)front, (unsigned)B5_TRIGGER_CM,
                   (unsigned)s_hits, (unsigned)B5_Servo_GetAngle(),
                   (unsigned)(s_rearmWait ? 1u : 0u));
        }
    }
    /* v94h: only the 1 cm rule stops the car (the 11-20 cm band read
     * phantoms on this module). */
    if (s_shortHits < B5_ONE_CM_COUNT)
    {
        B5_Port_DelayMs(B5_CRUISE_PERIOD_MS);
        return B5_STATE_IDLE;
    }
    B5_LOG("S4: watch fired (%u readings of 1 cm in a row)\n",
           (unsigned)s_shortHits);
    s_hits      = 0u;
    s_shortHits = 0u;
    if (!s_io->takeControl())                  /* IS25: TSK_CTRL stops the car */
    {
        B5_Port_DelayMs(B5_CRUISE_PERIOD_MS);  /* refused: keep watching */
        return B5_STATE_IDLE;
    }
    /* 22 Nov (car): a curve stopped the car with nothing in front.  The
     * watch fired on a moving-car artefact, handed over, and the coarse
     * sweep then found a wall off to the side and ordered a bypass around
     * thin air.  Now the front is re-measured while stationary: if it is
     * clear, there was never an obstacle. */
    {
        uint8_t confirm = 0u;
        uint8_t shorts  = 0u;               /* v94 stopgap */
        uint8_t closest = 0xFFu;
        uint8_t reads[B5_CONFIRM_READS];
        uint8_t i;

        B5_Servo_MoveTo(B5_SERVO_FRONT_DEG);
        for (i = 0u; i < B5_CONFIRM_READS; i++)
        {
            uint8_t d = B5_Sonar_MeasureCm();

            reads[i] = d;
            if ((d >= 1u) && (d <= B5_STOPGAP_SHORT_CM))
            {
                shorts++;
            }
            if ((d > B5_SELF_ECHO_CM) && (d <= B5_TRIGGER_CM))
            {
                confirm++;
                if (d < closest)
                {
                    closest = d;
                }
            }
        }
        /* 22 Nov: one run has to say what this sonar really returns for the
         * box, otherwise the thresholds are guesswork. */
        B5_LOG("S4: confirm reads %u %u %u %u %u %u %u %u cm "
               "(band %u..%u, close <= %u)\n",
               (unsigned)reads[0], (unsigned)reads[1], (unsigned)reads[2],
               (unsigned)reads[3], (unsigned)reads[4], (unsigned)reads[5],
               (unsigned)reads[6], (unsigned)reads[7],
               (unsigned)(B5_SELF_ECHO_CM + 1u), (unsigned)B5_TRIGGER_CM,
               (unsigned)B5_CONFIRM_CLOSE_CM);
        /* 22 Nov: the empty floor answers 24 cm every time; a real box
         * always gives at least one reading far shorter than that. */
        if (closest > B5_CONFIRM_CLOSE_CM)
        {
            confirm = 0u;
        }
#if B5_STOPGAP_SHORT
        /* v94 stopgap: the box is confirmed by short readings while
         * stopped.  The scan cannot measure it (every reading of it is
         * 1-3 cm), so no width is published and the car goes straight to
         * the fixed bypass on the B5_STOPGAP_LEFT side. */
        if ((confirm < B5_CONFIRM_HITS) &&
            (shorts >= B5_STOPGAP_CONFIRM_READS))
        {
            s_lastSide  = B5_STOPGAP_LEFT ? B5_SIDE_LEFT : B5_SIDE_RIGHT;
            s_plan.shiftCm = 0u;
            B5_Status_SetObstacle(true);
            B5_LOG("S4: obstacle (stopgap): %u of %u reads <= %u cm, "
                   "width not measurable, fixed bypass on the %s\n",
                   (unsigned)shorts, (unsigned)B5_CONFIRM_READS,
                   (unsigned)B5_STOPGAP_SHORT_CM,
                   B5_STOPGAP_LEFT ? "LEFT" : "RIGHT");
            return B5_STATE_BYPASS;
        }
#endif
        if (confirm < B5_CONFIRM_HITS)
        {
            B5_LOG("S4: false alarm (front clear while stopped): "
                   "%u of %u reads in band, following again\n",
                   (unsigned)confirm, (unsigned)B5_CONFIRM_READS);
            FalseAlarm();
            HandBack(true);
            return B5_STATE_IDLE;
        }
        B5_LOG("S4: obstacle %u cm ahead: %u of %u reads in band, scanning\n",
               (unsigned)front, (unsigned)confirm, (unsigned)B5_CONFIRM_READS);
    }
    return B5_STATE_COARSE;
}

static B5_State_t StateCoarse(void)
{
    B5_Profile_t quick;
    const uint32_t t0 = B5_Port_NowUs();
    uint8_t count = B5_Scan_Coarse(s_coarse);

    (void)B5_Profile_Build(s_coarse, count, B5_COARSE_STEP_DEG, &quick);
    B5_LOG("S4: coarse scan (right 66..114 left): %u %u %u %u %u cm, %u ms\n",
           (unsigned)s_coarse[0].dist, (unsigned)s_coarse[1].dist,
           (unsigned)s_coarse[2].dist, (unsigned)s_coarse[3].dist,
           (unsigned)s_coarse[4].dist,
           (unsigned)((B5_Port_NowUs() - t0) / 1000u));
    if (quick.detected == 0u)
    {
        B5_LOG("S4: nothing in the way after all: %s\n", "following again");
        FalseAlarm();
        HandBack(true);                        /* false alarm: carry on */
        return B5_STATE_IDLE;
    }
    B5_Status_SetObstacle(true);
    return B5_STATE_FINE;
}

static B5_State_t StateFine(void)
{
    B5_Profile_t profile;
    const uint32_t t0 = B5_Port_NowUs();

    s_fineCount = B5_Scan_Fine(s_coarse, B5_COARSE_COUNT, s_fine);
    B5_Servo_MoveTo(B5_SERVO_FRONT_DEG);
    if (!B5_Profile_Build(s_fine, s_fineCount, B5_FINE_STEP_DEG, &profile))
    {
        B5_LOG("S4: fine scan (%u samples, %u ms): obstacle gone\n",
               (unsigned)s_fineCount,
               (unsigned)((B5_Port_NowUs() - t0) / 1000u));
        FalseAlarm();
        HandBack(true);                        /* obstacle gone */
        return B5_STATE_IDLE;
    }
    s_openEdges = B5_Profile_OpenEdges(s_fine, s_fineCount);
    B5_LOG("S4: fine scan (%u samples, %u ms): width %u cm, closest %u cm at "
           "%u deg, clear left %u right %u cm\n", (unsigned)s_fineCount,
           (unsigned)((B5_Port_NowUs() - t0) / 1000u),
           (unsigned)profile.width, (unsigned)profile.closestDist,
           (unsigned)profile.closestAngle, (unsigned)profile.leftClear,
           (unsigned)profile.rightClear);
    if (s_openEdges != 0u)
    {
        B5_LOG("S4: the object goes past the servo's view on the %s: "
               "width is at least that\n",
               (s_openEdges == (B5_EDGE_OPEN_LEFT | B5_EDGE_OPEN_RIGHT)) ? "left and right"
               : ((s_openEdges & B5_EDGE_OPEN_LEFT) != 0u) ? "left" : "right");
    }
    Publish(&profile);
    return B5_STATE_PLAN;
}

static B5_State_t StatePlan(void)
{
    B5_Profile_t profile;
    B5_State_t next = B5_STATE_FAULT;

    B5_Task_GetProfile(&profile);
    /* IS25 17 Nov: the servo only sees 60..115 deg, so an object can run
     * past the edge of the view.  Plan with extra width on each such side
     * (the published width stays the measured one). */
    {
        uint32_t w = (uint32_t)profile.width;

        if ((s_openEdges & B5_EDGE_OPEN_RIGHT) != 0u)
        {
            w += B5_UNSEEN_EDGE_PAD_CM;
        }
        if ((s_openEdges & B5_EDGE_OPEN_LEFT) != 0u)
        {
            w += B5_UNSEEN_EDGE_PAD_CM;
        }
        profile.width = (w > 255u) ? 255u : w;
    }
    s_plan = B5_Plan_Decide(&profile, s_retries);
    B5_Status_SetAction((B5_Action_t)s_plan.action);
    {
        static const char *const names[5] =
        {
            "carry on", "STOP (no room either side)", "go round on the LEFT",
            "go round on the RIGHT", "back off and scan again"
        };
        B5_LOG("S4: plan: %s, shift %u cm\n",
               (s_plan.action < 5u) ? names[s_plan.action] : "?",
               (unsigned)s_plan.shiftCm);
    }

    switch ((B5_Action_t)s_plan.action)
    {
    case B5_ACT_CONTINUE:
        /* IS25 18 Nov: the object is beside the path, not in it.  Same as a
         * false alarm: do not stop for the same reading again at once, or
         * the car would stop and scan on the spot for ever. */
        FalseAlarm();
        HandBack(true);
        next = B5_STATE_IDLE;
        break;
    case B5_ACT_TURN_LEFT:
        s_lastSide = B5_SIDE_LEFT;
        next = B5_STATE_BYPASS;
        break;
    case B5_ACT_TURN_RIGHT:
        s_lastSide = B5_SIDE_RIGHT;
        next = B5_STATE_BYPASS;
        break;
    case B5_ACT_REVERSE:
        next = B5_STATE_REVERSE;
        break;
    case B5_ACT_STOP:
    default:
        B5_Status_RaiseFault(B5_FAULT_BLOCKED);
        next = B5_STATE_FAULT;
        break;
    }
    return next;
}

static B5_State_t StateReverse(void)
{
    s_retries++;                               /* planner caps this at 3 */
    B5_Status_SetRetries(s_retries);
    if (!B5_Manoeuvre_Reverse())
    {
        return B5_STATE_FAULT;
    }
    return B5_STATE_COARSE;                    /* reattempt from further back */
}

static B5_State_t StateBypass(void)
{
    B5_Profile_t profile;

    B5_Task_GetProfile(&profile);
    return B5_Manoeuvre_Bypass(s_lastSide, (uint8_t)s_plan.shiftCm,
                               (uint8_t)profile.closestDist)
               ? B5_STATE_RECOVER : B5_STATE_FAULT;
}

static B5_State_t StateRecover(void)
{
    B5_Side_t lineSide = B5_SIDE_LEFT;

#if B5_REJOIN_BY_CTRL
    /* IS25 20 Nov: TSK_CTRL finds the line (b5_config.h); it lies on the
     * side opposite the detour, and the hint below tells TSK_CTRL so. */
    lineSide = (s_lastSide == B5_SIDE_LEFT) ? B5_SIDE_RIGHT : B5_SIDE_LEFT;
#else
    if (!B5_Manoeuvre_RecoverLine(s_lastSide, &lineSide))
    {
        return B5_STATE_FAULT;
    }
#endif
    /* IS25 18 Nov: the follower's "last side" is from before the obstacle;
     * tell it where the line is now in case it loses it again at once. */
    if (s_io->lineHint != NULL)
    {
        s_io->lineHint(lineSide == B5_SIDE_LEFT);
    }
    B5_Status_ClearFault(B5_FAULT_BYPASS_ABORT | B5_FAULT_LINE_LOST | B5_FAULT_BLOCKED);
#if B5_REJOIN_BY_CTRL
    B5_LOG("S4: past the obstacle: %s\n", "TSK_CTRL rejoins the line");
#else
    B5_LOG("S4: back on the line after the obstacle: %s\n", "following");
#endif
    HandBack(true);                            /* return control to Buddy 3 */
    return B5_STATE_IDLE;
}

static B5_State_t StateFault(void)
{
    B5_LOG("S4: obstacle avoidance FAILED (status 0x%04X): run over\n",
           (unsigned)B5_Status_Get());
    s_io->stop();
    HandBack(false);                           /* IS25: TSK_CTRL -> system FAULT */
    B5_Port_DelayMs(B5_FAULT_HOLD_MS);         /* faults stay visible in status */
    return B5_STATE_IDLE;
}

/* 22 Nov (car): the engine is a task of its own, so a RESET/START in
 * TSK_CTRL did not touch it.  After one hand-over that never completed it
 * sat in a middle state for the whole next run: the console showed no watch
 * line for 28 s while the car drove into the box.  Every new run now pulls
 * it back to watching from scratch. */
static volatile uint32_t s_reset_req = 0u;

void B5_Task_RequestWatchReset(void)
{
    s_reset_req++;
}

void B5_Task_Run(void)
{
    B5_State_t state = B5_STATE_IDLE;
    uint32_t   seen  = 0u;

    /* IS25: TSK_CTRL starts line following; we only watch until it is ours. */
    for (;;)
    {
        if (seen != s_reset_req)
        {
            seen  = s_reset_req;
            state = B5_STATE_IDLE;
            WatchReset();
        }
        B5_Status_SetState(state);
        switch (state)
        {
        case B5_STATE_IDLE:    state = StateIdle();    break;
        case B5_STATE_COARSE:  state = StateCoarse();  break;
        case B5_STATE_FINE:    state = StateFine();    break;
        case B5_STATE_PLAN:    state = StatePlan();    break;
        case B5_STATE_REVERSE: state = StateReverse(); break;
        case B5_STATE_BYPASS:  state = StateBypass();  break;
        case B5_STATE_RECOVER: state = StateRecover(); break;
        case B5_STATE_FAULT:
        default:               state = StateFault();   break;
        }
    }
}
