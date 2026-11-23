/** @file imu_glue.c
 *
 * @brief IS25 adapter: team inputs -> Buddy 4's IMU_SAMPLE, and his
 *        IMU_STATUS -> the team status word, events and hump record.
 *        See imu_glue.h and BUDDY4_INTEGRATION.md.
 */

#include "imu_glue.h"
#include "imu_math.h"

#define GLUE_HUMP_MM_MAX       127u
#define GLUE_CDEG_PER_Q        25
#define GLUE_PITCH_Q_MIN       (-128)
#define GLUE_PITCH_Q_MAX       127
#define GLUE_UM_PER_MM         1000u
#define GLUE_DEG_FULL          360

static int16_t clamp16(int32_t value);
static int32_t abs32(int32_t value);


/*!
 * @brief Reset the adapter (start of a run, or a test reset).
 */
void
imu_glue_init (imu_glue_t * p_glue)
{
    uint8_t index = 0u;

    for (index = 0u; index < GLUE_ACC_WINDOW; index++)
    {
        p_glue->speed_hist[index] = 0;
    }

    p_glue->hist_index  = 0u;
    p_glue->hist_fill   = 0u;
    p_glue->hump_prev   = IMU_HUMP_IDLE;
    p_glue->count_prev  = 0u;
    p_glue->b_lift_prev = false;
    p_glue->last_mm     = 0u;
    p_glue->max_mm      = 0u;
}

/*!
 * @brief Build Buddy 4's IMU_SAMPLE from one robot-frame reading and the
 *        subsystem 2 odometry.
 *
 * Sign convention: imu.c models ax = -g sin(pitch) - a_car, the negative
 * of the specific force a real LSM303 reports.  Passing -fwd and -left
 * makes his formula sin(pitch) = -(ax + a_enc) / g come out right (nose
 * up = positive pitch, the car's own acceleration cancelled).
 */
void
imu_glue_sample (imu_glue_t * p_glue, imu_glue_in_t const * p_in,
                 IMU_SAMPLE * p_out)
{
    int32_t speed  = ((int32_t)p_in->left_mm_s + p_in->right_mm_s) / 2;
    int32_t oldest = p_glue->speed_hist[p_glue->hist_index];
    int32_t a_mg   = 0;

    p_out->ax_mg = clamp16(-(int32_t)p_in->fwd_mg);
    p_out->ay_mg = clamp16(-(int32_t)p_in->left_mg);
    p_out->az_mg = p_in->up_mg;

    if (!p_in->b_odom_valid)
    {
        p_glue->hist_fill = 0u;     /* No reference: restart the window. */
        p_out->a_enc_mg   = 0;
        p_out->speed_mmps = 0u;
        p_out->odom_valid = 0u;
        p_out->dd_um      = 0u;
        return;
    }

    /* Forward acceleration from the signed speed 50 ms ago. */
    p_glue->speed_hist[p_glue->hist_index] = (int16_t)speed;
    p_glue->hist_index = (uint8_t)((p_glue->hist_index + 1u) %
                                   GLUE_ACC_WINDOW);

    if (p_glue->hist_fill < GLUE_ACC_WINDOW)
    {
        p_glue->hist_fill++;
    }
    else
    {
        a_mg = ((speed - oldest) * GLUE_MG_PER_MM_S2_NUM) /
               GLUE_MG_PER_MM_S2_DEN;
    }

    p_out->a_enc_mg   = clamp16(a_mg);
    p_out->speed_mmps = (uint16_t)abs32(speed);
    p_out->odom_valid = 1u;
    p_out->dd_um      = (uint32_t)abs32(p_in->delta_mm) * GLUE_UM_PER_MM;
}

/*!
 * @brief Edges for the team: EVT_HUMP on start and end (IF-03), a report
 *        when his hump counter moves, and the lifted rising edge.
 */
uint8_t
imu_glue_events (imu_glue_t * p_glue, IMU_STATUS const * p_imu,
                 uint16_t hump_count)
{
    uint8_t events      = 0u;
    bool    b_lifted    = (0u != (p_imu->flags & IMU_FLAG_LIFTED));

    if ((IMU_HUMP_IDLE == p_glue->hump_prev) &&
        (IMU_HUMP_IDLE != p_imu->hump_state))
    {
        events |= GLUE_EVT_HUMP_START;
    }
    else if ((IMU_HUMP_IDLE != p_glue->hump_prev) &&
             (IMU_HUMP_IDLE == p_imu->hump_state))
    {
        events |= GLUE_EVT_HUMP_END;
    }
    else
    {
        /* No change. */
    }

    if (hump_count != p_glue->count_prev)
    {
        uint16_t mm = p_imu->hump_peak_mm;

        p_glue->last_mm = (uint8_t)((mm > GLUE_HUMP_MM_MAX) ?
                                    GLUE_HUMP_MM_MAX : mm);

        if (p_glue->last_mm > p_glue->max_mm)
        {
            p_glue->max_mm = p_glue->last_mm;
        }

        events |= GLUE_EVT_HUMP_REPORT;
    }

    if (b_lifted && (!p_glue->b_lift_prev))
    {
        events |= GLUE_EVT_LIFTED;
    }

    p_glue->hump_prev   = p_imu->hump_state;
    p_glue->count_prev  = hump_count;
    p_glue->b_lift_prev = b_lifted;

    return (events);
}

/*!
 * @brief FR10 "turning": his motion classes leave this to the motion
 *        interface (imu.c, motion_step), so it is decided here from the
 *        left/right wheel speeds.
 */
bool
imu_glue_turning (int16_t left_mm_s, int16_t right_mm_s)
{
    return (abs32((int32_t)left_mm_s - right_mm_s) > GLUE_TURNING_MM_S);
}

/*!
 * @brief His IMU_STATUS -> the team status fields (subsystem3.h).
 *
 *   hump : IDLE -> FLAT, RISING -> CLIMBING, FALLING -> DESCENDING,
 *          sensor fault -> INVALID.  (CRESTING is not produced: his
 *          detector goes straight from rising to falling.)
 *   motion: LIFTED, CLIMBING, DESCENDING first; then TURNING; then
 *          ACCELERATING, DECELERATING, CRUISING, STATIONARY.
 *          UNKNOWN (no odometry) is reported as STATIONARY.
 */
void
imu_glue_status (IMU_STATUS const * p_imu, bool b_turning, bool b_fault,
                 uint16_t heading_deg, subsystem3_status_t * p_status)
{
    subsystem3_motion_t motion = S3_MOTION_STATIONARY;

    p_status->pitch_q       = imu_glue_pitch_q(p_imu->pitch_cdeg);
    p_status->heading_deg   = heading_deg;
    p_status->b_hump_active = (0u != (p_imu->flags & IMU_FLAG_HUMP_ACTIVE));

    if (b_fault)
    {
        p_status->hump = S3_HUMP_INVALID;
    }
    else if (IMU_HUMP_RISING == p_imu->hump_state)
    {
        p_status->hump = S3_HUMP_CLIMBING;
    }
    else if (IMU_HUMP_FALLING == p_imu->hump_state)
    {
        p_status->hump = S3_HUMP_DESCENDING;
    }
    else
    {
        p_status->hump = S3_HUMP_FLAT;
    }

    switch (p_imu->motion)
    {
        case IMU_MOTION_LIFTED:
            motion = S3_MOTION_LIFTED;
            break;

        case IMU_MOTION_CLIMBING:
            motion = S3_MOTION_CLIMBING;
            break;

        case IMU_MOTION_DESCENDING:
            motion = S3_MOTION_DESCENDING;
            break;

        case IMU_MOTION_ACCELERATING:
            motion = S3_MOTION_ACCELERATING;
            break;

        case IMU_MOTION_DECELERATING:
            motion = S3_MOTION_DECELERATING;
            break;

        case IMU_MOTION_CRUISING:
            motion = S3_MOTION_CRUISING;
            break;

        default:
            motion = S3_MOTION_STATIONARY;   /* STATIONARY or UNKNOWN. */
            break;
    }

    if (b_turning && (S3_MOTION_LIFTED != motion) &&
        (S3_MOTION_CLIMBING != motion) && (S3_MOTION_DESCENDING != motion))
    {
        motion = S3_MOTION_TURNING;
    }

    p_status->motion = motion;
}

/*!
 * @brief Centi-degrees to the 0.25 degree field, rounded, saturated.
 */
int16_t
imu_glue_pitch_q (int16_t pitch_cdeg)
{
    int32_t half = (pitch_cdeg >= 0) ? (GLUE_CDEG_PER_Q / 2)
                                     : -(GLUE_CDEG_PER_Q / 2);
    int32_t q    = ((int32_t)pitch_cdeg + half) / GLUE_CDEG_PER_Q;

    if (q < GLUE_PITCH_Q_MIN)
    {
        q = GLUE_PITCH_Q_MIN;
    }
    else if (q > GLUE_PITCH_Q_MAX)
    {
        q = GLUE_PITCH_Q_MAX;
    }
    else
    {
        /* In range. */
    }

    return ((int16_t)q);
}

/*!
 * @brief Pack the IMU status word (see subsystem3.h).
 */
uint32_t
imu_status_pack (subsystem3_status_t const * p_status)
{
    uint32_t pitch_raw = (uint32_t)((int32_t)p_status->pitch_q +
                                    S3_PITCH_OFFSET) & S3_PITCH_MASK;
    uint32_t heading   = (uint32_t)p_status->heading_deg & S3_HEADING_MASK;
    uint32_t word      = pitch_raw |
                         (heading << S3_HEADING_SHIFT) |
                         (((uint32_t)p_status->hump & S3_HUMP_MASK) <<
                          S3_HUMP_SHIFT) |
                         (((uint32_t)p_status->motion & S3_MOTION_MASK) <<
                          S3_MOTION_SHIFT);

    if (p_status->b_hump_active)
    {
        word |= S3_HUMP_ACTIVE_BIT;
    }

    return (word);
}

/*!
 * @brief Unpack the IMU status word.
 */
void
imu_status_unpack (uint32_t word, subsystem3_status_t * p_status)
{
    p_status->pitch_q     = (int16_t)((int32_t)(word & S3_PITCH_MASK) -
                                      S3_PITCH_OFFSET);
    p_status->heading_deg = (uint16_t)((word >> S3_HEADING_SHIFT) &
                                       S3_HEADING_MASK);
    p_status->hump        = (subsystem3_hump_state_t)
                            ((word >> S3_HUMP_SHIFT) & S3_HUMP_MASK);
    p_status->motion      = (subsystem3_motion_t)
                            ((word >> S3_MOTION_SHIFT) & S3_MOTION_MASK);
    p_status->b_hump_active = (0u != (word & S3_HUMP_ACTIVE_BIT));
}

/*!
 * @brief Pack the 14-bit hump record (both heights saturate at 127 mm).
 */
uint16_t
imu_hump_record_pack (uint8_t last_mm, uint8_t max_mm)
{
    uint16_t last = (last_mm > GLUE_HUMP_MM_MAX) ? GLUE_HUMP_MM_MAX : last_mm;
    uint16_t peak = (max_mm > GLUE_HUMP_MM_MAX) ? GLUE_HUMP_MM_MAX : max_mm;

    return ((uint16_t)(last | (uint16_t)(peak << S3_HUMP_MAX_SHIFT)));
}

/*!
 * @brief Magnetic heading 0..359 from the hard-iron corrected X/Y.
 */
uint16_t
imu_heading_deg (float mx, float my)
{
    int32_t deg = (int32_t)(imu_atan2f(my, mx) * IMU_RAD_TO_DEG_F);

    if (deg < 0)
    {
        deg += GLUE_DEG_FULL;
    }

    return ((uint16_t)((uint32_t)deg % (uint32_t)GLUE_DEG_FULL));
}

static int16_t
clamp16 (int32_t value)
{
    int32_t result = value;

    if (result > INT16_MAX)
    {
        result = INT16_MAX;
    }
    else if (result < -INT16_MAX)
    {
        result = -INT16_MAX;
    }
    else
    {
        /* In range. */
    }

    return ((int16_t)result);
}

static int32_t
abs32 (int32_t value)
{
    return ((value < 0) ? -value : value);
}

/*** end of file ***/
