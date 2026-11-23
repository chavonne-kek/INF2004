/** @file subsystem2.c
 *
 * @brief Subsystem 2 (Buddy 2): start-up and the public interface in
 *        subsystem2.h.
 *
 * Owner: Kai Xuan (student ID 2501473).
 */

#include "subsystem2.h"
#include "motion_task.h"
#include "motion_calc.h"
#include "drv_encoder.h"


/*!
 * @brief Phase 1: motors braked, encoders, 10 ms cyclic, mbf_motion_cmd.
 */
ER
subsystem2_init (void)
{
    return (motion_task_init());
}

/*!
 * @brief Phase 2: start TSK_MOTION on core 1.
 */
ER
subsystem2_start (void)
{
    return (motion_task_start());
}

/*!
 * @brief Queue a motion command (IF-04).  Never blocks.  Only TSK_CTRL
 *        may call this (single decision-maker).
 *
 * @return E_OK, or E_TMOUT if the command buffer is full.
 */
ER
subsystem2_command (subsystem2_op_t op, uint8_t arg)
{
    return (motion_task_command(op, arg));
}

/*!
 * @brief moveForward(distance): drive straight, stop on the mark.
 */
ER
subsystem2_move_forward (uint8_t distance_cm)
{
    return (motion_task_command(S2_OP_MOVE_FWD, distance_cm));
}

/*!
 * @brief moveBackward(distance).
 */
ER
subsystem2_move_backward (uint8_t distance_cm)
{
    return (motion_task_command(S2_OP_MOVE_BACK, distance_cm));
}

/*!
 * @brief turnLeft(angle): turn on the spot, encoder based.
 */
ER
subsystem2_turn_left (uint8_t angle_deg)
{
    return (motion_task_command(S2_OP_TURN_LEFT, angle_deg));
}

/*!
 * @brief turnRight(angle).
 */
ER
subsystem2_turn_right (uint8_t angle_deg)
{
    return (motion_task_command(S2_OP_TURN_RIGHT, angle_deg));
}

/*!
 * @brief stop(): brake in the same 10 ms cycle (NFR4).
 */
ER
subsystem2_stop (void)
{
    return (motion_task_command(S2_OP_STOP, 0u));
}

/*!
 * @brief Read the odometry (IF-02): one 32-bit load, then unpack.
 */
void
subsystem2_odometry_read (subsystem2_odom_t * p_odom)
{
    motion_odom_unpack(motion_task_odometry_word(), p_odom);
}

/*!
 * @brief The packed odometry word itself (for telemetry).
 */
uint32_t
subsystem2_odometry_word (void)
{
    return (motion_task_odometry_word());
}

/*!
 * @brief Signed distance between two odometer readings (wrap-safe).
 */
int16_t
subsystem2_distance_delta (uint16_t now_mm, uint16_t before_mm)
{
    return (motion_distance_delta(now_mm, before_mm));
}

/*!
 * @brief Raw 16-bit encoder edge counts (telemetry).
 */
void
subsystem2_encoder_counts (uint16_t * p_left, uint16_t * p_right)
{
    *p_left  = drv_encoder_count(ENC_LEFT);
    *p_right = drv_encoder_count(ENC_RIGHT);
}

/*!
 * @brief Motion statechart state (3 bits).
 */
subsystem2_state_t
subsystem2_state (void)
{
    return ((subsystem2_state_t)motion_task_state());
}

/*!
 * @brief Counts TSK_MOTION cycles.  TSK_CTRL feeds the watchdog only when
 *        this has changed (NFR19).
 */
uint32_t
subsystem2_alive_count (void)
{
    return (motion_task_alive());
}

/*!
 * @brief Attach another subsystem's pin to the shared GPIO interrupt.
 *        Call from subsystemN_init() only.
 */
ER
subsystem2_gpio_attach (uint8_t pin, subsystem2_gpio_cb_t cb)
{
    return (drv_encoder_attach(pin, cb));
}

uint16_t
subsystem2_cmd_pack (subsystem2_op_t op, uint8_t arg)
{
    return (motion_cmd_pack(op, arg));
}

uint32_t
subsystem2_odom_pack (subsystem2_odom_t const * p_odom)
{
    return (motion_odom_pack(p_odom));
}

void
subsystem2_odom_unpack (uint32_t word, subsystem2_odom_t * p_odom)
{
    motion_odom_unpack(word, p_odom);
}

/*** end of file ***/
