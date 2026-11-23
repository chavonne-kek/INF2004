/** @file comms_codec.c
 *
 * @brief Wire formats for subsystem 5 (see comms_codec.h).  Pure logic.
 */

#include "comms_codec.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem4/subsystem4.h"

#define COMMS_BYTE_MASK        0xFFu
#define COMMS_BITS_PER_BYTE    8u

static char const * const g_topic_names[COMMS_TOPIC_COUNT] =
{
    "robot/heartbeat",
    "robot/telemetry/line",
    "robot/telemetry/motion",
    "robot/telemetry/imu",
    "robot/telemetry/barcode",
    "robot/telemetry/hump",
    "robot/telemetry/obstacle",
    "robot/telemetry/state",
    "robot/telemetry/comms",     /* COMMS_TOPIC_COMMS: keep in enum order! */
    "robot/telemetry/ir",        /* COMMS_TOPIC_IR                    */
    "robot/telemetry/bctrace",   /* COMMS_TOPIC_BCTRACE (18 Nov)      */
    "robot/telemetry/range"      /* COMMS_TOPIC_RANGE (22 Nov)        */
};

/* Payload length of each event record type (subsystem5.h). */
static uint8_t const g_event_len[S5_TLM_EVENT_TYPES] = {1u, 2u, 5u, 1u, 8u};


/*!
 * @brief MQTT topic string for a topic ID ("" if out of range).
 */
char const *
comms_topic_name (comms_topic_t topic)
{
    return ((topic < COMMS_TOPIC_COUNT) ? g_topic_names[topic] : "");
}

/*!
 * @brief Events are kept while offline and sent with QoS 1 (NFR13/14).
 */
bool
comms_topic_is_event (comms_topic_t topic)
{
    return ((COMMS_TOPIC_BARCODE == topic) || (COMMS_TOPIC_HUMP == topic) ||
            (COMMS_TOPIC_OBSTACLE == topic) || (COMMS_TOPIC_STATE == topic));
}

/*!
 * @brief Topic for an event record type.
 */
comms_topic_t
comms_event_topic (subsystem5_tlm_t type)
{
    comms_topic_t topic = COMMS_TOPIC_STATE;

    switch (type)
    {
        case S5_TLM_BARCODE:
            topic = COMMS_TOPIC_BARCODE;
        break;

        case S5_TLM_HUMP:
            topic = COMMS_TOPIC_HUMP;
        break;

        case S5_TLM_OBSTACLE:
            topic = COMMS_TOPIC_OBSTACLE;
        break;

        case S5_TLM_BCTRACE:
            topic = COMMS_TOPIC_BCTRACE;
        break;

        case S5_TLM_STATE:
        default:
            topic = COMMS_TOPIC_STATE;
        break;
    }

    return (topic);
}

/*!
 * @brief Required payload length of an event type (0 = unknown type).
 */
uint8_t
comms_event_len (subsystem5_tlm_t type)
{
    return ((type < S5_TLM_EVENT_TYPES) ? g_event_len[type] : 0u);
}

/*!
 * @brief Heartbeat: state 4 + faults 6 + signal 7 + uptime 12 = 29 bits.
 *        Uptime wraps (0..4095 s); signal saturates at 127.
 */
uint32_t
comms_heartbeat_pack (uint8_t state, uint8_t faults, uint8_t signal,
                      uint32_t uptime_s)
{
    uint32_t sig = (signal > S5_HB_SIGNAL_MASK) ? S5_HB_SIGNAL_MASK : signal;

    return (((uint32_t)state & S4_STATE_MASK) |
            (((uint32_t)faults & FAULT_ALL) << S5_HB_FAULT_SHIFT) |
            (sig << S5_HB_SIGNAL_SHIFT) |
            ((uptime_s & S5_HB_UPTIME_MASK) << S5_HB_UPTIME_SHIFT));
}

/*!
 * @brief Store a 16-bit value little-endian.
 */
void
comms_put_u16 (uint8_t * p_buf, uint16_t value)
{
    p_buf[0] = (uint8_t)(value & COMMS_BYTE_MASK);
    p_buf[1] = (uint8_t)(value >> COMMS_BITS_PER_BYTE);
}

/*!
 * @brief Store a 32-bit value little-endian.
 */
void
comms_put_u32 (uint8_t * p_buf, uint32_t value)
{
    uint8_t index = 0u;

    for (index = 0u; index < 4u; index++)
    {
        p_buf[index] = (uint8_t)((value >> (COMMS_BITS_PER_BYTE * index)) &
                                 COMMS_BYTE_MASK);
    }
}

/*!
 * @brief Validate a robot/command payload (FR16 + Change Requirement 1).
 *
 * A command is exactly 1 byte: opcode in bits 0-2 (1..6), speed in bits
 * 3-7 (2..25 for SPEED, 0 for every other opcode).  Anything else is
 * malformed: dropped, never changes state, counted.
 *
 * @param[out] p_cmd The byte to forward to TSK_CTRL if valid.
 *
 * @return True if valid.
 */
bool
comms_command_decode (uint8_t const * p_data, uint8_t len, uint8_t * p_cmd)
{
    bool    b_ok = false;
    uint8_t op   = 0u;
    uint8_t arg  = 0u;

    if (1u == len)
    {
        op  = (uint8_t)(p_data[0] & S4_RCMD_OP_MASK);
        arg = (uint8_t)(p_data[0] >> S4_RCMD_ARG_SHIFT);

        if (S4_RCMD_SPEED == op)
        {
            b_ok = (arg >= S4_RCMD_SPEED_MIN) && (arg <= S4_RCMD_SPEED_MAX);
        }
        else if (S4_RCMD_GAIN == op)
        {
            /* Steering gain in 100 mm/s per mm: 1..25 (100..2500).  Nov
             * 2026: was 1..31, but TSK_CTRL clamps above 25, so 26..31
             * were accepted and then silently changed. */
            b_ok = (arg >= 1u) && (arg <= S4_RCMD_GAIN_MAX);
        }
        else
        {
            b_ok = (op >= S4_RCMD_START) && (op <= S4_RCMD_RESET) &&
                   (0u == arg);
        }
    }
    else if (2u == len)
    {
        /* Two-byte frame, used by the barcode map command. */
        uint8_t nav = (uint8_t)((p_data[1] >> S4_RCMD_MAP_SHIFT) & 0x07u);
        uint8_t idx = (uint8_t)(p_data[1] & S4_RCMD_MAP_INDEX_MASK);

        op  = p_data[0];
        arg = p_data[1];

        if (S4_RCMD_MAP == op)
        {
            /* Movement 1..4 (left, right, straight, U-turn), letter A..Z. */
            b_ok = (nav >= (uint8_t)SUBSYSTEM1_CMD_LEFT) &&
                   (nav <= (uint8_t)SUBSYSTEM1_CMD_UTURN) &&
                   (idx < S4_RCMD_MAP_LETTERS);
        }
    }

    if (b_ok)
    {
        p_cmd[0] = op;
        p_cmd[1] = arg;
    }

    return (b_ok);
}

/*!
 * @brief Empty the offline buffer.
 */
void
comms_ring_init (comms_ring_t * p_ring)
{
    p_ring->head        = 0u;
    p_ring->count       = 0u;
    p_ring->overwritten = 0u;
}

/*!
 * @brief Keep an event while offline; when full, the oldest is replaced.
 */
void
comms_ring_push (comms_ring_t * p_ring, comms_msg_t const * p_msg)
{
    uint8_t tail = (uint8_t)((p_ring->head + p_ring->count) %
                             COMMS_RING_SIZE);

    p_ring->slot[tail] = *p_msg;

    if (p_ring->count < COMMS_RING_SIZE)
    {
        p_ring->count++;
    }
    else
    {
        p_ring->head = (uint8_t)((p_ring->head + 1u) % COMMS_RING_SIZE);

        if (p_ring->overwritten < UINT16_MAX)
        {
            p_ring->overwritten++;
        }
    }
}

/*!
 * @brief Take the oldest kept event.
 *
 * @return False if empty.
 */
bool
comms_ring_pop (comms_ring_t * p_ring, comms_msg_t * p_msg)
{
    bool b_got = false;

    if (p_ring->count > 0u)
    {
        *p_msg       = p_ring->slot[p_ring->head];
        p_ring->head = (uint8_t)((p_ring->head + 1u) % COMMS_RING_SIZE);
        p_ring->count--;
        b_got = true;
    }

    return (b_got);
}

/*** end of file ***/
