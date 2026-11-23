/**
 * @file host_test.c
 * @brief PC unit tests for pure logic: profiler, planner, bit packing, status.
 *
 * Build & run:  see test/run_host_tests.sh
 * A tiny ray-caster fakes the ultrasonic sweep so no hardware is needed.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "b5_config.h"
#include "b5_plan.h"
#include "b5_profile.h"
#include "b5_types.h"

static int s_failures;

#define CHECK(cond, msg) \
    do { if (!(cond)) { printf("  FAIL: %s (line %d)\n", msg, __LINE__); s_failures++; } } while (0)

/* Scene: a flat-faced obstacle at forward distance Y spanning lateral
 * [x0, x1], plus optional side walls at x = wallR (right) and wallL (left). */
/* Main box [x0,x1] at forward y, plus an optional second box [bx0,bx1] at by (by<=0 = none). */
typedef struct { double x0, x1, y; double bx0, bx1, by; } Scene;

static uint8_t Ping(const Scene *sc, int angleDeg)
{
    const double th = angleDeg * M_PI / 180.0;
    const double dx = cos(th);
    const double dy = sin(th);
    double best = 1e9;

    if (dy > 1e-6)
    {
        const double t = sc->y / dy;
        const double x = t * dx;
        if (x >= sc->x0 && x <= sc->x1) { best = t; }
    }
    if ((dy > 1e-6) && (sc->by > 0.0))
    {
        const double t = sc->by / dy;
        const double x = t * dx;
        if ((x >= sc->bx0) && (x <= sc->bx1) && (t < best)) { best = t; }
    }
    if (best >= B5_DIST_MAX_CM) { return B5_DIST_NO_ECHO; }
    return (uint8_t)lround(best);
}

static uint8_t Sweep(const Scene *sc, int start, int end, int step, B5_Sample_t *out)
{
    uint8_t n = 0;
    for (int a = start; a <= end; a += step)
    {
        out[n].angle = (uint8_t)a;
        out[n].dist = Ping(sc, a);
        n++;
    }
    return n;
}

static const char *ActName(unsigned a)
{
    static const char *names[] = { "CONTINUE", "STOP", "TURN_LEFT", "TURN_RIGHT", "REVERSE" };
    return (a < 5u) ? names[a] : "?";
}

static void RunScene(const char *name, const Scene *sc, unsigned wantAction,
                     int wantWidth, int widthTol, uint8_t retries)
{
    B5_Sample_t fine[B5_FINE_MAX_SAMPLES];
    B5_Profile_t p;
    uint8_t n = Sweep(sc, 30, 150, (int)B5_FINE_STEP_DEG, fine);
    bool found = B5_Profile_Build(fine, n, B5_FINE_STEP_DEG, &p);
    B5_Plan_t plan = B5_Plan_Decide(&p, retries);

    printf("%s\n  detected=%u centre=%u closest=%ucm@%u width=%u left=%u right=%u -> %s shift=%u\n",
           name, p.detected, p.centreAngle, p.closestDist, p.closestAngle, p.width,
           p.leftClear, p.rightClear, ActName(plan.action), plan.shiftCm);
    CHECK(found == (p.detected != 0u), "detect flag consistent");
    CHECK(plan.action == wantAction, "planner action");
    if (wantWidth >= 0)
    {
        CHECK(abs((int)p.width - wantWidth) <= widthTol, "width within tolerance");
    }
}

static void TestBitPacking(void)
{
    B5_Profile_t a;
    B5_Profile_t b;
    uint8_t wire[B5_PROFILE_WIRE_BYTES];

    a.detected = 1; a.centreAngle = 97; a.closestAngle = 90; a.closestDist = 33;
    a.width = 14; a.leftClear = 255; a.rightClear = 21;
    B5_Profile_Pack(&a, wire);
    B5_Profile_Unpack(wire, &b);
    printf("bit packing round trip\n");
    CHECK((a.detected == b.detected) && (a.centreAngle == b.centreAngle) && (a.closestAngle == b.closestAngle) &&
          (a.closestDist == b.closestDist) && (a.width == b.width) && (a.leftClear == b.leftClear) &&
          (a.rightClear == b.rightClear), "pack/unpack identical");
    CHECK(sizeof(B5_Profile_t) == 8u, "RAM profile is 8 bytes");
    CHECK(sizeof(B5_Sample_t) == 2u, "sample is 2 bytes");
    CHECK(sizeof(B5_Plan_t) == 2u, "plan is 2 bytes");
}

static void TestStatus(void)
{
    uint16_t w;

    printf("shared status word\n");
    B5_Status_Init();
    B5_Status_SetState(B5_STATE_FAULT);
    B5_Status_SetAction(B5_ACT_REVERSE);
    B5_Status_SetObstacle(true);
    B5_Status_SetRetries(3u);
    B5_Status_RaiseFault(B5_FAULT_NO_ECHO | B5_FAULT_BLOCKED);
    B5_Status_ClearFault(B5_FAULT_NO_ECHO);
    w = B5_Status_Get();
    CHECK(B5_ST_GET(w, STATE) == B5_STATE_FAULT, "state");
    CHECK(B5_ST_GET(w, ACTION) == B5_ACT_REVERSE, "action");
    CHECK(B5_ST_GET(w, OBSTACLE) == 1u, "obstacle");
    CHECK(B5_ST_GET(w, RETRIES) == 3u, "retries");
    CHECK(B5_ST_GET(w, FAULTS) == (B5_FAULT_BLOCKED), "faults independent bits");
    B5_Status_SetState(B5_STATE_IDLE);
    CHECK(B5_ST_GET(B5_Status_Get(), RETRIES) == 3u, "setting state keeps other fields");
}

int main(void)
{
    /* Obstacle 12 cm wide, centred on the line, 35 cm ahead, open sides. */
    Scene centred = { -6, 6, 35, 0, 0, 0 };
    /* Second box close on the right (gap 11 cm < car) -> must go left. */
    Scene wallRight = { -6, 6, 35, 17, 30, 45 };
    /* Second box close on the left -> must go right. */
    Scene wallLeft = { -6, 6, 35, -30, -17, 45 };
    /* Obstacle off to the right, clear of the path. */
    Scene offPath = { 30, 42, 35, 0, 0, 0 };
    /* Boxed in both sides: wide gap needed on the left too. */
    Scene boxed = { -6, 6, 35, -30, 30, 48 };
    /* Too close to manoeuvre. */
    Scene tooClose = { -6, 6, 7, 0, 0, 0 };
    /* Nothing there. */
    Scene empty = { 500, 501, 500, 0, 0, 0 };
    /* Obstacle shifted slightly right: left bypass needs the smaller shift. */
    Scene slightRight = { -2, 10, 35, 0, 0, 0 };

    RunScene("centred, open both sides", &centred, B5_ACT_TURN_LEFT, 12, 6, 0);
    RunScene("wall on right", &wallRight, B5_ACT_TURN_LEFT, 12, 6, 0);
    RunScene("wall on left", &wallLeft, B5_ACT_TURN_RIGHT, 12, 6, 0);
    RunScene("off path to the right", &offPath, B5_ACT_CONTINUE, -1, 0, 0);
    RunScene("boxed in, retries left", &boxed, B5_ACT_REVERSE, -1, 0, 0);
    RunScene("boxed in, retries exhausted", &boxed, B5_ACT_STOP, -1, 0, B5_MAX_RETRIES);
    RunScene("too close", &tooClose, B5_ACT_REVERSE, -1, 0, 0);
    RunScene("empty", &empty, B5_ACT_CONTINUE, -1, 0, 0);
    RunScene("slightly right of centre", &slightRight, B5_ACT_TURN_LEFT, 12, 6, 0);

    TestBitPacking();
    TestStatus();

    printf("\n%s (%d failure%s)\n", s_failures == 0 ? "ALL PASSED" : "FAILED",
           s_failures, s_failures == 1 ? "" : "s");
    return s_failures == 0 ? 0 : 1;
}
