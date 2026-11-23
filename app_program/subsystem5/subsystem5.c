/** @file subsystem5.c
 *
 * @brief Subsystem 5 (Buddy 1): start-up and the public interface in
 *        subsystem5.h.
 *
 * Owner: Chavonne (student ID 2502481).
 */

#include "subsystem5.h"
#include "comms_task.h"


/*!
 * @brief Phase 1: create mbf_telemetry, mbf_mqtt_tx, mbf_mqtt_rx.
 */
ER
subsystem5_init (void)
{
#if !defined(TM_MQTT)
    tm_printf((UB *)"S5: built without MQTT (make NET=lwip MQTT=1 for the "
              "dashboard); telemetry stays offline\n");
#endif

    return (comms_task_init());
}

/*!
 * @brief Phase 2: start TSK_COMMS on core 0.
 */
ER
subsystem5_start (void)
{
    return (comms_task_start());
}

/*!
 * @brief Post an event telemetry record (IF-06).  Never blocks; a full
 *        buffer drops the record and counts it.
 *
 * @param[in] type   S5_TLM_* record type.
 * @param[in] p_data Packed payload (length fixed per type, subsystem5.h).
 * @param[in] len    Payload length.
 *
 * @return E_OK, E_PAR (wrong length) or E_TMOUT (buffer full).
 */
ER
subsystem5_post (subsystem5_tlm_t type, uint8_t const * p_data, uint8_t len)
{
    return (comms_task_post(type, p_data, len));
}

subsystem5_link_t
subsystem5_link_state (void)
{
    return ((subsystem5_link_t)comms_task_link());
}

uint16_t
subsystem5_drop_count (void)
{
    return (comms_task_drops());
}

uint16_t
subsystem5_bad_command_count (void)
{
    return (comms_task_bad());
}

/*** end of file ***/
