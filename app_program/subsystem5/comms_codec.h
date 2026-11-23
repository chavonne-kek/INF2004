/** @file comms_codec.h
 *
 * @brief Wire formats for subsystem 5 (Change Requirement 1: compact
 *        binary MQTT payloads, little-endian, built with masks and
 *        shifts): topic table, heartbeat, command validation and the
 *        16-entry offline event buffer.  Pure logic, tested on a PC
 *        (testing/subsystem5/host).
 */

#ifndef COMMS_CODEC_H
#define COMMS_CODEC_H

#include <stdint.h>
#include <stdbool.h>

#include "subsystem5.h"

/* Topic IDs: the first byte of every mbf_mqtt_tx message.  The topic
 * identifies the payload layout, so payloads carry no type byte. */
typedef enum
{
    COMMS_TOPIC_HEARTBEAT = 0,  /* 4 B  heartbeat (subsystem5.h)       */
    COMMS_TOPIC_LINE,           /* 2 B  S1 line word                   */
    COMMS_TOPIC_MOTION,         /* 8 B  S2 odometry word + 2 x encoder */
    COMMS_TOPIC_IMU,            /* 3 B  S3 status word (24 bits)       */
    COMMS_TOPIC_BARCODE,        /* 1 B  S1 barcode message     (event) */
    COMMS_TOPIC_HUMP,           /* 2 B  S3 hump record         (event) */
    COMMS_TOPIC_OBSTACLE,       /* 5 B  S4 obstacle profile    (event) */
    COMMS_TOPIC_STATE,          /* 1 B  S4 system state        (event) */
    COMMS_TOPIC_COMMS,          /* 3 B  drops, bad commands, barcode rejects */
    COMMS_TOPIC_IR,             /* 6 B  raw IR: arm, centre, right        */
    COMMS_TOPIC_BCTRACE,        /* 8 B  one part of a barcode trace (18 Nov) */
    COMMS_TOPIC_RANGE,          /* 2 B  sonar watch: front cm, hits (22 Nov) */
    COMMS_TOPIC_COUNT
} comms_topic_t;

#define COMMS_MSG_MAX          (1u + S5_TLM_PAYLOAD_MAX)  /* id + payload */
#define COMMS_CMD_MAX_LEN      (S5_TLM_PAYLOAD_MAX + 1u)
#define COMMS_RING_SIZE        16u     /* NFR13 offline event buffer.     */

typedef struct
{
    uint8_t len;                       /* Bytes used in data (incl. id).  */
    uint8_t data[COMMS_MSG_MAX];
} comms_msg_t;

typedef struct
{
    comms_msg_t slot[COMMS_RING_SIZE];
    uint8_t     head;
    uint8_t     count;
    uint16_t    overwritten;
} comms_ring_t;

char const * comms_topic_name(comms_topic_t topic);
bool         comms_topic_is_event(comms_topic_t topic);
comms_topic_t comms_event_topic(subsystem5_tlm_t type);
uint8_t      comms_event_len(subsystem5_tlm_t type);

uint32_t comms_heartbeat_pack(uint8_t state, uint8_t faults,
                              uint8_t signal, uint32_t uptime_s);
void     comms_put_u16(uint8_t * p_buf, uint16_t value);
void     comms_put_u32(uint8_t * p_buf, uint32_t value);
bool     comms_command_decode(uint8_t const * p_data, uint8_t len,
                              uint8_t * p_cmd);

void     comms_ring_init(comms_ring_t * p_ring);
void     comms_ring_push(comms_ring_t * p_ring, comms_msg_t const * p_msg);
bool     comms_ring_pop(comms_ring_t * p_ring, comms_msg_t * p_msg);

#endif /* COMMS_CODEC_H */

/*** end of file ***/
