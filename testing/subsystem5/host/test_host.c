/** @file test_host.c
 *
 * @brief PC tests for subsystem 5's pure module comms_codec.c.
 */

#include <string.h>

#include "../../host_common/host_check.h"
#include "comms_codec.h"
#include "../../../app_program/subsystem4/subsystem4.h"

int
main (void)
{
    int     valid = 0;
    int     ok    = 1;
    uint8_t cmd[2] = {0u, 0u};

    /* Every possible 1-byte command: exactly 5 opcodes + 24 speeds (2..25)
     * + 25 gains (1..25; Nov 2026: was 1..31, but the robot clamped). */
    for (int b = 0; b < 256; b++)
    {
        uint8_t byte = (uint8_t)b;

        if (comms_command_decode(&byte, 1u, cmd))
        {
            valid++;
            ok = ok && (cmd[0] == (byte & S4_RCMD_OP_MASK)) &&
                 (cmd[1] == (byte >> S4_RCMD_ARG_SHIFT));
        }
    }
    printf("        %d of 256 one-byte payloads are valid\n", valid);
    HOST_CHECK("H-CO01", ok && (54 == valid),
               "commands: 5 opcodes + SPEED 2..25 + GAIN 1..25, nothing else");

    {
        uint8_t two[2] = {1u, 0u};

        HOST_CHECK("H-CO02", (!comms_command_decode(two, 2u, cmd)) &&
                   (!comms_command_decode(two, 0u, cmd)),
                   "wrong length is malformed");
    }

    /* Heartbeat layout. */
    {
        uint32_t hb = comms_heartbeat_pack(SYS_LINE_FOLLOW, FAULT_WIFI, 61u,
                                           5000u);

        HOST_CHECK("H-CO03",
                   ((hb & 0xFu) == SYS_LINE_FOLLOW) &&
                   (((hb >> 4) & 0x3Fu) == FAULT_WIFI) &&
                   (((hb >> 10) & 0x7Fu) == 61u) &&
                   (((hb >> 17) & 0xFFFu) == (5000u % 4096u)) &&
                   (0u == (hb >> 29)),
                   "heartbeat 29 bits, uptime wraps at 4096 s");
    }

    /* Offline ring keeps the newest 16 events. */
    {
        comms_ring_t r;
        comms_msg_t  m;
        int          first = -1;
        int          count = 0;

        comms_ring_init(&r);
        for (int i = 0; i < 20; i++)
        {
            m.len = 2u;
            m.data[0] = (uint8_t)COMMS_TOPIC_HUMP;
            m.data[1] = (uint8_t)i;
            comms_ring_push(&r, &m);
        }
        while (comms_ring_pop(&r, &m))
        {
            if (first < 0) { first = m.data[1]; }
            count++;
        }
        HOST_CHECK("H-CO04", (16 == count) && (4 == first) &&
                   (4u == r.overwritten),
                   "offline buffer: 16 newest kept, oldest overwritten");
    }

    HOST_CHECK("H-CO05",
               (0 == strcmp(comms_topic_name(COMMS_TOPIC_HEARTBEAT),
                            "robot/heartbeat")) &&
               comms_topic_is_event(COMMS_TOPIC_OBSTACLE) &&
               (!comms_topic_is_event(COMMS_TOPIC_MOTION)) &&
               (comms_event_len(S5_TLM_OBSTACLE) == S4_PROFILE_BYTES) &&
               (comms_event_len(S5_TLM_HUMP) == 2u) &&
               (COMMS_TOPIC_BARCODE == comms_event_topic(S5_TLM_BARCODE)),
               "topic table and event sizes");

    {
        uint8_t b[4];

        comms_put_u32(b, 0x11223344u);
        HOST_CHECK("H-CO06", (b[0] == 0x44u) && (b[3] == 0x11u),
                   "payloads are little-endian");
    }

    /* A topic name table that silently disagrees with the enum sends every
     * message under the wrong topic: IR data arrived as "comms" (so the
     * dashboard read a sensor byte as a drop count) and the comms counters
     * arrived as "ir" (too short to decode, so all three levels showed "-")
     * for five hours on 2 Nov.  Names are checked by index, not just by
     * looking for the string somewhere in the table. */
    {
        bool ok = true;

        ok = ok && (0 == strcmp(comms_topic_name(COMMS_TOPIC_IR),
                                "robot/telemetry/ir"));
        ok = ok && (0 == strcmp(comms_topic_name(COMMS_TOPIC_COMMS),
                                "robot/telemetry/comms"));
        ok = ok && (0 == strcmp(comms_topic_name(COMMS_TOPIC_STATE + 1),
                                "robot/telemetry/comms"));

        HOST_CHECK("H-CO07", ok, "topic names match the enum order (IR vs comms)");
    }

    return (HOST_SUMMARY());
}

/*** end of file ***/
