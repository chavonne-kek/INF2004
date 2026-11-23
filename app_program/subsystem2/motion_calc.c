/** @file motion_calc.c
 *
 * @brief Motion maths for subsystem 2 (see motion_calc.h).  Pure logic.
 */

#include "motion_calc.h"

#define MOTION_MILLI           1000
#define MOTION_UM_PER_MM_CALC  1000u
#define MOTION_STRAIGHT_CLAMP_UM 100000u  /* 100 mm: far past saturation. */
#define MOTION_TICKS_PER_S     100         /* 10 ms control period.        */
#define MOTION_PI_NUM          355u        /* 355 / 113 = pi to 7 digits.  */
#define MOTION_PI_DEN          113u
#define MOTION_DEG_PER_TURN    360u
#define MOTION_UM_PER_MM       1000u
#define MOTION_DIST_HALF       0x4000      /* Half of the 15-bit range.    */
#define MOTION_DIST_RANGE      0x8000
#define MOTION_SPEED_RAW_MAX   127


/*!
 * @brief Set the default gains and clear the integrator.
 */
void
motion_pid_init (motion_pid_t * p_pid)
{
    p_pid->kff_milli = MOTION_KFF_MILLI;
    p_pid->kp_milli  = MOTION_KP_MILLI;
    p_pid->ki_milli  = MOTION_KI_MILLI;
    p_pid->kd_milli  = MOTION_KD_MILLI;

    /* Limit the integrator so it can never command the wheel on its own.
     * The original limit let the integral alone reach full duty, so a wheel
     * that fell behind during a turn kept blasting at 100% after the target
     * dropped: measured 315 mm/s against a 50 mm/s cruise, which is what
     * threw the car off the mat at the end of every bend (30 Oct).
     * A quarter of full duty is plenty to trim a steady-state error. */
    p_pid->integral_limit = (MOTION_DUTY_MAX * MOTION_MILLI *
                             MOTION_TICKS_PER_S) / (MOTION_KI_MILLI * 4);
    motion_pid_reset(p_pid);
}

/*!
 * @brief Clear the integrator and the derivative history (on brake, stop
 *        or direction change).
 */
void
motion_pid_reset (motion_pid_t * p_pid)
{
    p_pid->integral      = 0;
    p_pid->last_measured = 0;
    p_pid->b_have_last   = false;
}

/*!
 * @brief One 10 ms step of the wheel-speed controller:
 *        feed-forward + P + I + D (derivative of the measurement).
 *
 * @param[in,out] p_pid         Controller state.
 * @param[in]     target_mm_s   Signed wheel speed set-point.
 * @param[in]     measured_mm_s Signed measured wheel speed.
 *
 * @return Signed duty, -MOTION_DUTY_MAX..+MOTION_DUTY_MAX.
 */
int32_t
motion_pid_step (motion_pid_t * p_pid, int32_t target_mm_s,
                 int32_t measured_mm_s)
{
    int32_t error = target_mm_s - measured_mm_s;
    int32_t slope = 0;      /* Change of the measurement, mm/s per tick. */
    int32_t duty  = 0;

    p_pid->integral = motion_clamp(p_pid->integral + error,
                                   -p_pid->integral_limit,
                                   p_pid->integral_limit);

    if (p_pid->b_have_last)
    {
        slope = measured_mm_s - p_pid->last_measured;
    }

    p_pid->last_measured = measured_mm_s;
    p_pid->b_have_last   = true;

    duty = ((p_pid->kff_milli * target_mm_s) +
            (p_pid->kp_milli * error) +
            ((p_pid->ki_milli * p_pid->integral) / MOTION_TICKS_PER_S) -
            (p_pid->kd_milli * slope * MOTION_TICKS_PER_S)) /
           MOTION_MILLI;

    /* A forward target must never be met by driving the wheel backwards.
     * The measured speed takes its sign from the commanded direction, so
     * once the PI pushed below zero the encoder kept reporting a positive
     * speed, the error stayed negative, and the duty pinned at -100%: the
     * wheel reversed at full speed and the car spun on the spot (30 Oct).
     * Same in reverse for a backward target. */
    /* 20 Nov: this guard used to zero the duty whenever its sign opposed the
     * target, which also killed the braking torque of a wheel that was
     * OVERSHOOTING backwards - so a pivot's reversing wheel had nothing
     * slowing it down and ran to 320 mm/s (car telemetry L +30 / R -320, and
     * the car overshot its 90 deg turn into a spin).  Only refuse torque that
     * would push the wheel further the WRONG way; once the wheel is already
     * turning the commanded way, opposite torque is braking and must stand. */
    if ((target_mm_s > 0) && (duty < 0) && (measured_mm_s <= 0))
    {
        duty = 0;
    }
    else if ((target_mm_s < 0) && (duty > 0) && (measured_mm_s >= 0))
    {
        duty = 0;
    }

    return (motion_clamp(duty, -MOTION_DUTY_MAX, MOTION_DUTY_MAX));
}

/*!
 * @brief Move current toward target by at most step.
 */
int32_t
motion_ramp (int32_t current, int32_t target, int32_t step)
{
    int32_t result = target;

    if (target > (current + step))
    {
        result = current + step;
    }
    else if (target < (current - step))
    {
        result = current - step;
    }
    else
    {
        /* Within one step: jump to the target. */
    }

    return (result);
}

/*!
 * @brief Limit value to low..high.
 */
int32_t
motion_clamp (int32_t value, int32_t low, int32_t high)
{
    int32_t result = value;

    if (value < low)
    {
        result = low;
    }
    else if (value > high)
    {
        result = high;
    }
    else
    {
        /* In range. */
    }

    return (result);
}

/*!
 * @brief Distance in micrometres for a number of encoder edges.
 */
uint32_t
motion_edges_to_um (uint32_t edges)
{
    uint64_t um = ((uint64_t)edges * (uint64_t)MOTION_WHEEL_DIAMETER_UM *
                   MOTION_PI_NUM) /
                  ((uint64_t)MOTION_PI_DEN * (uint64_t)MOTION_EDGES_PER_REV);

    return ((uint32_t)um);  /* Fits: < 4.3 km per call. */
}

/*!
 * @brief Speed in mm/s from the edges counted over a number of ticks.
 */
int32_t
motion_window_speed (uint32_t edges, uint32_t ticks)
{
    uint32_t result = 0u;

    if (0u != ticks)
    {
        result = (motion_edges_to_um(edges) * MOTION_TICKS_PER_S) /
                 (ticks * MOTION_UM_PER_MM);
    }

    return ((int32_t)result);   /* < 1 m/s: fits. */
}

/*!
 * @brief Encoder edges needed to travel mm millimetres.
 */
uint32_t
motion_mm_to_edges (uint32_t mm)
{
    uint64_t edges = ((uint64_t)mm * MOTION_UM_PER_MM * MOTION_PI_DEN *
                      (uint64_t)MOTION_EDGES_PER_REV) /
                     ((uint64_t)MOTION_WHEEL_DIAMETER_UM * MOTION_PI_NUM);

    return ((uint32_t)edges);  /* Fits for any 8-bit cm argument. */
}

/*!
 * @brief Arc each wheel travels for an in-place turn of degrees.
 *
 * Each wheel moves theta x track / 2 (theta in radians), in opposite
 * directions.
 */
uint32_t
motion_turn_arc_mm (uint32_t degrees)
{
    return ((degrees * (uint32_t)MOTION_TRACK_MM * MOTION_PI_NUM) /
            (MOTION_PI_DEN * MOTION_DEG_PER_TURN));
}

/*!
 * @brief Wheel speed difference for a line error (left-edge tracking).
 *
 * Positive error means steer left: the caller subtracts the result from
 * the left wheel and adds it to the right wheel.
 */
/* Runtime steering gain, in mm/s per mm of line error.  Defaults to
 * MOTION_STEER_MILLI and can be changed live from the dashboard with
 * "gain <milli>", so the gain can be tuned while the car is driving
 * instead of reflashing for every value. */
static int32_t g_steer_milli = MOTION_STEER_MILLI;

void
motion_set_steer_milli (int32_t milli)
{
    if (milli < 0)
    {
        milli = 0;
    }
    else if (milli > 20000)
    {
        milli = 20000;
    }

    g_steer_milli = milli;
}

int32_t
motion_steer_milli (void)
{
    return (g_steer_milli);
}

int32_t
motion_steer_mm_s (int8_t line_error_mm)
{
    int32_t err = (int32_t)line_error_mm;
    int32_t mag = (err < 0) ? -err : err;
    int32_t out = (mag * g_steer_milli) / MOTION_MILLI;

    /* 22 Nov: past the knee the bend needs real authority.  Below it the
     * old gain is untouched, so a straight still settles instead of
     * snaking (1200 alone snaked - see MOTION_STEER_MILLI). */
    if (mag > MOTION_STEER_KNEE_MM)
    {
        out += ((mag - MOTION_STEER_KNEE_MM) * MOTION_STEER_HARD_MILLI) / MOTION_MILLI;
    }
    if (out > MOTION_SPEED_MAX_MM_S)
    {
        out = MOTION_SPEED_MAX_MM_S;
    }
    return ((err < 0) ? -out : out);
}

/*!
 * @brief Speed allowed with remaining_mm left to go: never faster than
 *        cruise, never faster than sqrt(2 a d), never below the final
 *        approach speed.
 */
int32_t
motion_profile_speed (uint32_t remaining_mm, int32_t cruise_mm_s)
{
    uint32_t limit = motion_isqrt(2u * (uint32_t)MOTION_DECEL_MM_S2 *
                                  ((remaining_mm > 100000u) ? 100000u
                                                            : remaining_mm));
    int32_t  speed = cruise_mm_s;

    if ((int32_t)limit < speed)
    {
        speed = (int32_t)limit;
    }

    if (speed < MOTION_FINAL_MM_S)
    {
        speed = (cruise_mm_s < MOTION_FINAL_MM_S) ? cruise_mm_s
                                                  : MOTION_FINAL_MM_S;
    }

    return (speed);
}

/*!
 * @brief Straight-line / equal-arc correction from the edge counts.
 *
 * @return mm/s to take from the left wheel's speed and give to the
 *         right one (negative: the right wheel is ahead).
 */
int32_t
motion_straight_trim (uint32_t done_left, uint32_t done_right)
{
    /* In micrometres, so one edge of a fine encoder (0.57 mm at 360 edges
     * per turn) already counts; the difference is clamped first, beyond
     * the point where the trim saturates, so the product fits 32 bits. */
    uint32_t diff_edges = (done_left >= done_right) ?
                          (done_left - done_right) : (done_right - done_left);
    uint32_t diff_um    = motion_edges_to_um(diff_edges);
    int32_t  trim       = 0;

    if (diff_um > MOTION_STRAIGHT_CLAMP_UM)
    {
        diff_um = MOTION_STRAIGHT_CLAMP_UM;
    }

    /* Rounded, not truncated (2 Nov): at 1270 edges per turn one edge is
     * 0.16 mm, worth 0.96 mm/s of trim, which truncation turned into 0. */
    trim = (int32_t)(((diff_um * (uint32_t)MOTION_STRAIGHT_MILLI) +
                      ((MOTION_UM_PER_MM_CALC * (uint32_t)MOTION_MILLI) / 2u)) /
                     (MOTION_UM_PER_MM_CALC * (uint32_t)MOTION_MILLI));
    trim = (done_left >= done_right) ? trim : -trim;

    return (motion_clamp(trim, -MOTION_STRAIGHT_MAX_MM_S,
                         MOTION_STRAIGHT_MAX_MM_S));
}

/*!
 * @brief Integer square root (floor), no libm.
 */
uint32_t
motion_isqrt (uint32_t value)
{
    uint32_t root = 0u;
    uint32_t bit  = 1u << 30u;

    while (bit > value)
    {
        bit >>= 2u;
    }

    while (0u != bit)
    {
        if (value >= (root + bit))
        {
            value -= root + bit;
            root   = (root >> 1u) + bit;
        }
        else
        {
            root >>= 1u;
        }

        bit >>= 2u;
    }

    return (root);
}

/*!
 * @brief Build the 11-bit motion command (IF-04).
 */
uint16_t
motion_cmd_pack (subsystem2_op_t op, uint8_t arg)
{
    return ((uint16_t)(((uint16_t)op & S2_CMD_OP_MASK) |
                       ((uint16_t)arg << S2_CMD_ARG_SHIFT)));
}

/*!
 * @brief Split a motion command.
 *
 * @return False if the spare bits 12-15 are not zero (corrupt message).
 */
bool
motion_cmd_unpack (uint16_t raw, subsystem2_op_t * p_op, uint8_t * p_arg)
{
    *p_op  = (subsystem2_op_t)(raw & S2_CMD_OP_MASK);  /* 4-bit field. */
    *p_arg = (uint8_t)((raw >> S2_CMD_ARG_SHIFT) & S2_CMD_ARG_MASK);

    return (0u == (raw >> (S2_CMD_ARG_SHIFT + 8u)));
}

/*!
 * @brief Pack one wheel speed into 7 bits (5 mm/s steps, offset 64).
 */
static uint32_t
speed_to_raw (int16_t mm_s)
{
    int32_t steps = 0;

    /* Round to the nearest 5 mm/s step. */
    if (mm_s >= 0)
    {
        steps = ((int32_t)mm_s + (S2_SPEED_STEP_MM_S / 2)) /
                S2_SPEED_STEP_MM_S;
    }
    else
    {
        steps = ((int32_t)mm_s - (S2_SPEED_STEP_MM_S / 2)) /
                S2_SPEED_STEP_MM_S;
    }

    /* Saturate (measured values never wrap). */
    return ((uint32_t)motion_clamp(steps + S2_SPEED_OFFSET, 0,
                                   MOTION_SPEED_RAW_MAX));
}

/*!
 * @brief Build the odometry word (IF-02).
 */
uint32_t
motion_odom_pack (subsystem2_odom_t const * p_odom)
{
    return (((uint32_t)p_odom->distance_mm & S2_DIST_MASK) |
            (speed_to_raw(p_odom->left_mm_s) << S2_SPEED_SHIFT_L) |
            (speed_to_raw(p_odom->right_mm_s) << S2_SPEED_SHIFT_R));
}

/*!
 * @brief Unpack the odometry word.
 */
void
motion_odom_unpack (uint32_t word, subsystem2_odom_t * p_odom)
{
    int32_t left  = (int32_t)((word >> S2_SPEED_SHIFT_L) & S2_SPEED_MASK);
    int32_t right = (int32_t)((word >> S2_SPEED_SHIFT_R) & S2_SPEED_MASK);

    p_odom->distance_mm = (uint16_t)(word & S2_DIST_MASK);
    p_odom->left_mm_s   = (int16_t)((left - S2_SPEED_OFFSET) *
                                    S2_SPEED_STEP_MM_S);
    p_odom->right_mm_s  = (int16_t)((right - S2_SPEED_OFFSET) *
                                    S2_SPEED_STEP_MM_S);
}

/*!
 * @brief Signed distance between two 15-bit wrapping distance readings.
 *
 * Correct as long as the robot moves less than 16 m between readings.
 */
int16_t
motion_distance_delta (uint16_t now_mm, uint16_t before_mm)
{
    int32_t delta = ((int32_t)now_mm - (int32_t)before_mm) &
                    (int32_t)S2_DIST_MASK;

    if (delta >= MOTION_DIST_HALF)
    {
        delta -= MOTION_DIST_RANGE;
    }

    return ((int16_t)delta);  /* -16384..16383 fits. */
}

/*** end of file ***/
