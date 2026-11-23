/** @file comms_bridge.h
 *
 * @brief IF-07: functions the kernel WiFi service task (cyw43_utk_task,
 *        inside the extended mqtt_utk.c) calls to exchange messages with
 *        TSK_COMMS.  They only use the two message buffers with TMO_POL,
 *        so the service task never blocks.
 *
 * This header deliberately includes only <stdint.h>: mqtt_utk.c must not
 * include tk/tkernel.h (see the comment at the top of mqtt_utk.c).
 */

#ifndef COMMS_BRIDGE_H
#define COMMS_BRIDGE_H

#include <stdint.h>

#define COMMS_BRIDGE_TOPIC_COMMAND  "robot/command"
#define COMMS_BRIDGE_TOPIC_STATUS   "robot/status"
#define COMMS_BRIDGE_STATUS_ONLINE  "online"    /* Retained.           */
#define COMMS_BRIDGE_STATUS_OFFLINE "offline"   /* Last Will, retained. */
#define COMMS_BRIDGE_KEEPALIVE_S    5           /* NFR18 (5 s, not 2 s). */
#define COMMS_BRIDGE_POLL_MS        20
#define COMMS_BRIDGE_MAX_PER_POLL   4
#define COMMS_BRIDGE_PAYLOAD_MAX    16

/* Next message to publish.  Returns the payload length (>= 0) and sets
 * *pp_topic and *p_qos, or -1 if there is nothing to send. */
int32_t comms_bridge_next(char const ** pp_topic, uint8_t * p_buf,
                          uint16_t buf_size, uint8_t * p_qos);

/* A message arrived on a subscribed topic. */
void    comms_bridge_received(char const * p_topic, uint8_t const * p_data,
                              uint16_t len);

/* MQTT session up (1) or down (0); RSSI in dBm if known (else 0). */
void    comms_bridge_connected(int32_t b_up);
void    comms_bridge_rssi(int32_t rssi_dbm);

#endif /* COMMS_BRIDGE_H */

/*** end of file ***/
