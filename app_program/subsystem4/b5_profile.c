/**
 * @file b5_profile.c
 * @brief Obstacle profiler (pure logic, no hardware access).
 *
 * IS25 integration (26 Oct): the firmware does not link libm, so cosf /
 * sinf / sqrtf / lroundf are replaced by a cosine table (1 degree steps,
 * x 10000), integer rounding and an integer square root.  The algorithm
 * is unchanged.
 */
#include "b5_profile.h"
#include "b5_config.h"

#define CLEAR_OPEN_CM   (255)
#define TRIG_SCALE      (10000)
#define PI_NUM          (355)          /* 355 / 113 = pi to 7 digits */
#define PI_DEN          (113)
#define HALF_TURN_DEG   (180)
#define TENTHS          (10)

/* cos(a) x 10000 for a = 0..180 degrees. */
static const int16_t k_cos[B5_ANGLE_MAX_DEG + 1u] =
{
    10000, 9998, 9994, 9986, 9976, 9962, 9945, 9925, 9903, 9877,
    9848, 9816, 9781, 9744, 9703, 9659, 9613, 9563, 9511, 9455,
    9397, 9336, 9272, 9205, 9135, 9063, 8988, 8910, 8829, 8746,
    8660, 8572, 8480, 8387, 8290, 8192, 8090, 7986, 7880, 7771,
    7660, 7547, 7431, 7314, 7193, 7071, 6947, 6820, 6691, 6561,
    6428, 6293, 6157, 6018, 5878, 5736, 5592, 5446, 5299, 5150,
    5000, 4848, 4695, 4540, 4384, 4226, 4067, 3907, 3746, 3584,
    3420, 3256, 3090, 2924, 2756, 2588, 2419, 2250, 2079, 1908,
    1736, 1564, 1392, 1219, 1045, 872, 698, 523, 349, 175,
    0, -175, -349, -523, -698, -872, -1045, -1219, -1392, -1564,
    -1736, -1908, -2079, -2250, -2419, -2588, -2756, -2924, -3090, -3256,
    -3420, -3584, -3746, -3907, -4067, -4226, -4384, -4540, -4695, -4848,
    -5000, -5150, -5299, -5446, -5592, -5736, -5878, -6018, -6157, -6293,
    -6428, -6561, -6691, -6820, -6947, -7071, -7193, -7314, -7431, -7547,
    -7660, -7771, -7880, -7986, -8090, -8192, -8290, -8387, -8480, -8572,
    -8660, -8746, -8829, -8910, -8988, -9063, -9135, -9205, -9272, -9336,
    -9397, -9455, -9511, -9563, -9613, -9659, -9703, -9744, -9781, -9816,
    -9848, -9877, -9903, -9925, -9945, -9962, -9976, -9986, -9994, -9998,
    -10000
};

static int32_t CosX(uint8_t angleDeg)
{
    return k_cos[(angleDeg > B5_ANGLE_MAX_DEG) ? B5_ANGLE_MAX_DEG : angleDeg];
}

/* sin(a) = cos(|90 - a|) for a = 0..180. */
static int32_t SinX(uint8_t angleDeg)
{
    const uint8_t a = (angleDeg > B5_ANGLE_MAX_DEG) ? (uint8_t)B5_ANGLE_MAX_DEG : angleDeg;

    return k_cos[(a > 90u) ? (uint8_t)(a - 90u) : (uint8_t)(90u - a)];
}

/* value / TRIG_SCALE rounded half away from zero (like lroundf). */
static int16_t RoundScaled(int32_t value)
{
    const int32_t half = TRIG_SCALE / 2;

    return (int16_t)((value >= 0) ? ((value + half) / TRIG_SCALE)
                                  : -((-value + half) / TRIG_SCALE));
}

static uint32_t ISqrt(uint32_t value)
{
    uint32_t root = 0u;
    uint32_t bit = 1uL << 30;
    uint32_t rest = value;

    while (bit > rest)
    {
        bit >>= 2;
    }
    while (bit != 0u)
    {
        if (rest >= (root + bit))
        {
            rest -= root + bit;
            root = (root >> 1) + bit;
        }
        else
        {
            root >>= 1;
        }
        bit >>= 2;
    }
    return root;
}

int16_t B5_Geom_LateralCm(uint8_t angleDeg, uint8_t distCm)
{
    return RoundScaled((int32_t)distCm * CosX(angleDeg));
}

int16_t B5_Geom_ForwardCm(uint8_t angleDeg, uint8_t distCm)
{
    return RoundScaled((int32_t)distCm * SinX(angleDeg));
}

bool B5_Profile_IsObstacle(uint8_t angleDeg, uint8_t distCm)
{
    int16_t lateral;
    int16_t forward;

    if (distCm >= B5_DIST_MAX_CM)
    {
        return false;
    }
    lateral = B5_Geom_LateralCm(angleDeg, distCm);
    forward = B5_Geom_ForwardCm(angleDeg, distCm);
    if (lateral < 0)
    {
        lateral = (int16_t)-lateral;
    }
    return ((uint16_t)lateral <= B5_CORRIDOR_HALF_WIDTH_CM) &&
           ((uint16_t)forward <= B5_DETECT_RANGE_CM);
}

static bool IsEcho(const B5_Sample_t *s)
{
    return s->dist < B5_DIST_MAX_CM;
}

static uint8_t AbsDiff(uint8_t a, uint8_t b)
{
    return (a > b) ? (uint8_t)(a - b) : (uint8_t)(b - a);
}

static uint8_t Clamp255(int32_t value)
{
    if (value < 0)
    {
        return 0u;
    }
    return (value > CLEAR_OPEN_CM) ? (uint8_t)CLEAR_OPEN_CM : (uint8_t)value;
}

/* Index of nearest corridor obstacle sample, or -1. */
static int16_t FindClosest(const B5_Sample_t *s, uint8_t count)
{
    int16_t best = -1;
    uint8_t i;

    for (i = 0u; i < count; i++)
    {
        if (B5_Profile_IsObstacle(s[i].angle, s[i].dist) &&
            ((best < 0) || (s[i].dist < s[best].dist)))
        {
            best = (int16_t)i;
        }
    }
    return best;
}

/* Grow the run of samples that belong to the same object as `seed`. */
static void FindEdges(const B5_Sample_t *s, uint8_t count, uint8_t seed,
                      uint8_t *lo, uint8_t *hi)
{
    uint8_t a = seed;
    uint8_t b = seed;

    while ((a > 0u) && IsEcho(&s[a - 1u]) &&
           (AbsDiff((uint8_t)s[a - 1u].dist, (uint8_t)s[a].dist) <= B5_EDGE_JUMP_CM))
    {
        a--;
    }
    while (((uint8_t)(b + 1u) < count) && IsEcho(&s[b + 1u]) &&
           (AbsDiff((uint8_t)s[b + 1u].dist, (uint8_t)s[b].dist) <= B5_EDGE_JUMP_CM))
    {
        b++;
    }
    *lo = a;
    *hi = b;
}

static uint8_t EstimateWidth(const B5_Sample_t *s, uint8_t lo, uint8_t hi, uint8_t stepDeg)
{
    const int32_t dx = (int32_t)B5_Geom_LateralCm(s[hi].angle, s[hi].dist) -
                       (int32_t)B5_Geom_LateralCm(s[lo].angle, s[lo].dist);
    const int32_t dy = (int32_t)B5_Geom_ForwardCm(s[hi].angle, s[hi].dist) -
                       (int32_t)B5_Geom_ForwardCm(s[lo].angle, s[lo].dist);
    /* Chord in tenths of a cm: sqrt(100 (dx^2 + dy^2)). */
    const int32_t chord10 = (int32_t)ISqrt((uint32_t)(100 * ((dx * dx) + (dy * dy))));
    /* Each edge sample only proves the object reaches somewhere inside its
     * angular cell, so add one cell of arc length to the chord:
     * mean distance x step in radians, in tenths of a cm. */
    const int32_t sumDist = (int32_t)s[lo].dist + (int32_t)s[hi].dist;
    const int32_t cell10 = (sumDist * (int32_t)stepDeg * PI_NUM * TENTHS) /
                           (2 * PI_DEN * HALF_TURN_DEG);

    return Clamp255((chord10 + cell10 + (TENTHS / 2)) / TENTHS);
}

/* Free lateral gap between the object's side and the nearest other return. */
static uint8_t SideClearance(const B5_Sample_t *s, uint8_t count,
                             uint8_t lo, uint8_t hi, int16_t edgeX, bool leftSide)
{
    int32_t best = CLEAR_OPEN_CM;
    uint8_t i;
    const uint8_t first = leftSide ? (uint8_t)(hi + 1u) : 0u;
    const uint8_t last = leftSide ? count : lo;

    for (i = first; i < last; i++)
    {
        if (IsEcho(&s[i]) &&
            (B5_Geom_ForwardCm(s[i].angle, s[i].dist) <= (int16_t)B5_LOOKAHEAD_CM))
        {
            const int16_t x = B5_Geom_LateralCm(s[i].angle, s[i].dist);
            int32_t gap = leftSide ? (int32_t)(edgeX - x) : (int32_t)(x - edgeX);

            if (gap < 0)
            {
                gap = 0;
            }
            if (gap < best)
            {
                best = gap;
            }
        }
    }
    return Clamp255(best);
}

bool B5_Profile_Build(const B5_Sample_t *samples, uint8_t count,
                      uint8_t stepDeg, B5_Profile_t *profile)
{
    const int16_t seed = FindClosest(samples, count);
    uint8_t lo;
    uint8_t hi;
    uint8_t i;
    int16_t xMin;
    int16_t xMax;

    *profile = (B5_Profile_t){0};   /* IS25: no memset */
    if (seed < 0)
    {
        return false;
    }

    FindEdges(samples, count, (uint8_t)seed, &lo, &hi);

    xMin = B5_Geom_LateralCm(samples[lo].angle, samples[lo].dist);
    xMax = xMin;
    for (i = lo; i <= hi; i++)
    {
        const int16_t x = B5_Geom_LateralCm(samples[i].angle, samples[i].dist);

        if (x < xMin)
        {
            xMin = x;
        }
        if (x > xMax)
        {
            xMax = x;
        }
    }

    profile->detected = 1u;
    profile->closestAngle = samples[seed].angle;
    profile->closestDist = samples[seed].dist;
    profile->centreAngle = (uint8_t)(((uint16_t)samples[lo].angle + samples[hi].angle) / 2u);
    profile->width = EstimateWidth(samples, lo, hi, stepDeg);
    profile->leftClear = SideClearance(samples, count, lo, hi, xMin, true);
    profile->rightClear = SideClearance(samples, count, lo, hi, xMax, false);
    return true;
}

uint8_t B5_Profile_OpenEdges(const B5_Sample_t *samples, uint8_t count)
{
    const int16_t seed = FindClosest(samples, count);
    uint8_t lo;
    uint8_t hi;
    uint8_t open = 0u;

    if (seed < 0)
    {
        return 0u;
    }
    FindEdges(samples, count, (uint8_t)seed, &lo, &hi);
    if (lo == 0u)
    {
        open |= B5_EDGE_OPEN_RIGHT;
    }
    if ((uint8_t)(hi + 1u) >= count)
    {
        open |= B5_EDGE_OPEN_LEFT;
    }
    return open;
}
