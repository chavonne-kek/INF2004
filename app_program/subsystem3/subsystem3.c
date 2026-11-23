/** @file subsystem3.c
 *
 * @brief Subsystem 3 (Buddy 4): start-up and the public interface in
 *        subsystem3.h.
 *
 * Owner: Zi Siang (student ID 2500989).
 */

#include "subsystem3.h"
#include "imu_task.h"
#include "imu_glue.h"


/*!
 * @brief Phase 1: create the 10 ms cyclic handler.
 */
ER
subsystem3_init (void)
{
    return (imu_task_init());
}

/*!
 * @brief Phase 2: start TSK_IMU on core 0.
 */
ER
subsystem3_start (void)
{
    return (imu_task_start());
}

/*!
 * @brief Read the IMU status (one 32-bit load, then unpack).
 */
void
subsystem3_status_read (subsystem3_status_t * p_status)
{
    imu_status_unpack(imu_task_status_word(), p_status);
}

/*!
 * @brief The packed IMU status word itself (telemetry).
 */
uint32_t
subsystem3_status_word (void)
{
    return (imu_task_status_word());
}

/*!
 * @brief True while a hump is being measured (TSK_CTRL then ignores
 *        obstacle readings of 10 cm or more; Week 6 section 5).
 */
bool
subsystem3_hump_active (void)
{
    return (0u != (imu_task_status_word() & S3_HUMP_ACTIVE_BIT));
}

/*!
 * @brief Last hump record (14 bits, see subsystem3.h).
 */
uint16_t
subsystem3_hump_record (void)
{
    return (imu_task_hump_record());
}

/*!
 * @brief Why FAULT_LIFTED was raised (S3_LIFT_*), latched until re-armed.
 */
uint8_t
subsystem3_lift_cause (void)
{
    return (imu_task_lift_cause());
}

/*!
 * @brief A new run starts (TSK_CTRL): clear the latch and arm the lift /
 *        overturn detector again.  TSK_IMU does it at its next 10 ms cycle
 *        (single writer); the detector learns gravity for 1 s first.
 */
void
subsystem3_lift_rearm (void)
{
    imu_task_lift_rearm();
}

void
subsystem3_calibrate_begin (void)
{
    imu_task_cal_begin();
}

bool
subsystem3_calibrate_end (void)
{
    return (imu_task_cal_end());
}

uint32_t
subsystem3_status_pack (subsystem3_status_t const * p_status)
{
    return (imu_status_pack(p_status));
}

void
subsystem3_status_unpack (uint32_t word, subsystem3_status_t * p_status)
{
    imu_status_unpack(word, p_status);
}

/*** end of file ***/
