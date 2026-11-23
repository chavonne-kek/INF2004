/** @file comms_bridge.c
 *
 * @brief Service-task side of IF-07 (see comms_bridge.h).  Runs in the
 *        kernel's cyw43_utk_task, never in TSK_COMMS.
 */

#include "subsystem5.h"
#include "comms_bridge.h"
#include "comms_task.h"
#include "comms_codec.h"

#define BRIDGE_QOS_PERIODIC    0u
#define BRIDGE_QOS_EVENT       1u
#define BRIDGE_RSSI_MAX        100


/*!
 * @brief Pop one message from mbf_mqtt_tx (TMO_POL).
 */
int32_t
comms_bridge_next (char const ** pp_topic, uint8_t * p_buf,
                   uint16_t buf_size, uint8_t * p_qos)
{
    uint8_t msg[COMMS_MSG_MAX];
    INT     size   = tk_rcv_mbf(comms_task_tx_mbf(), msg, TMO_POL);
    int32_t result = -1;
    uint8_t index  = 0u;

    if ((size >= 1) && ((size - 1) <= (INT)buf_size) &&
        (msg[0] < (uint8_t)COMMS_TOPIC_COUNT))
    {
        comms_topic_t topic = (comms_topic_t)msg[0];

        for (index = 1u; index < (uint8_t)size; index++)
        {
            p_buf[index - 1u] = msg[index];
        }

        *pp_topic = comms_topic_name(topic);
        *p_qos    = comms_topic_is_event(topic) ? BRIDGE_QOS_EVENT
                                                : BRIDGE_QOS_PERIODIC;
        result    = (int32_t)(size - 1);
    }

    return (result);
}

/*!
 * @brief Forward robot/command payloads to TSK_COMMS (TMO_POL).  Longer
 *        payloads are cut to COMMS_CMD_MAX_LEN, which is still invalid,
 *        so TSK_COMMS counts them as malformed.
 */
void
comms_bridge_received (char const * p_topic, uint8_t const * p_data,
                       uint16_t len)
{
    char const * p_expect = COMMS_BRIDGE_TOPIC_COMMAND;
    uint8_t      index    = 0u;
    bool         b_match  = true;

    while ((p_expect[index] != '\0') || (p_topic[index] != '\0'))
    {
        if (p_expect[index] != p_topic[index])
        {
            b_match = false;
            break;
        }

        index++;
    }

    if (b_match && (0u == len))
    {
        uint8_t empty = 0u;     /* Opcode 0 is invalid: counted as bad. */

        (void)tk_snd_mbf(comms_task_rx_mbf(), &empty, 1, TMO_POL);
    }
    else if (b_match)
    {
        uint16_t size = (len > COMMS_CMD_MAX_LEN) ? COMMS_CMD_MAX_LEN : len;

        (void)tk_snd_mbf(comms_task_rx_mbf(), (void *)p_data, (INT)size,
                         TMO_POL);  /* Cast: kernel API is not const. */
    }
    else
    {
        /* Not our topic. */
    }
}

/*!
 * @brief MQTT session state from the service task.
 */
void
comms_bridge_connected (int32_t b_up)
{
    comms_task_set_connected(0 != b_up);
}

/*!
 * @brief Link RSSI (negative dBm) -> heartbeat signal field (0..100).
 */
void
comms_bridge_rssi (int32_t rssi_dbm)
{
    int32_t magnitude = (rssi_dbm < 0) ? -rssi_dbm : rssi_dbm;

    comms_task_set_signal((uint8_t)((magnitude > BRIDGE_RSSI_MAX) ?
                                    BRIDGE_RSSI_MAX : magnitude));
}

/*** end of file ***/
