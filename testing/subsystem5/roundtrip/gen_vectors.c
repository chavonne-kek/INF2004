/** @file gen_vectors.c
 *
 * @brief Round-trip test, robot side (Tribunal 1, risk R2): builds every
 *        MQTT payload with the FIRMWARE's own pack functions and writes
 *        one JSON line per payload with the values it was built from.
 *        tools/is25_dashboard.py --selftest must decode every line back
 *        to the same values.
 *
 * Build and run: make run   (in this folder; needs gcc and python3)
 */

#include <stdio.h>
#include <stdlib.h>

#include "../../../app_program/subsystem1/line_sense.h"
#include "../../../app_program/subsystem1/barcode.h"
#include "../../../app_program/subsystem2/motion_calc.h"
#include "../../../app_program/subsystem3/imu_glue.h"
#include "../../../app_program/subsystem4/scan_calc.h"
#include "../../../app_program/subsystem5/comms_codec.h"

#define CASES  200

static FILE * g_out;

static void
hex (uint8_t const * p_data, unsigned len)
{
    unsigned i;

    for (i = 0u; i < len; i++)
    {
        fprintf(g_out, "%02x", p_data[i]);
    }
}

static void
begin (char const * topic, uint8_t const * p_data, unsigned len)
{
    fprintf(g_out, "{\"topic\": \"%s\", \"hex\": \"", topic);
    hex(p_data, len);
    fprintf(g_out, "\", ");
}

static char const *
jbool (bool b)
{
    return (b ? "true" : "false");
}

static char const *
cmd_name (subsystem1_cmd_t cmd)
{
    switch (cmd)
    {
        case SUBSYSTEM1_CMD_LEFT:     return ("TURN_LEFT");
        case SUBSYSTEM1_CMD_RIGHT:    return ("TURN_RIGHT");
        case SUBSYSTEM1_CMD_STRAIGHT: return ("STRAIGHT");
        case SUBSYSTEM1_CMD_UTURN:    return ("U_TURN");
        default:                      return ("NONE");
    }
}

static int
rnd (int lo, int hi)
{
    return (lo + (rand() % (hi - lo + 1)));
}

int
main (int argc, char ** argv)
{
    uint8_t  buf[8];
    unsigned i;

    g_out = (argc > 1) ? fopen(argv[1], "w") : stdout;

    if (NULL == g_out)
    {
        return (1);
    }

    srand(2004u);

    for (i = 0u; i < CASES; i++)
    {
        /* Heartbeat (S5). */
        {
            uint8_t  state  = (uint8_t)rnd(0, 9);
            uint8_t  faults = (uint8_t)rnd(0, 63);
            uint8_t  signal = (uint8_t)rnd(0, 100);
            uint32_t up     = (uint32_t)rnd(0, 100000);

            comms_put_u32(buf, comms_heartbeat_pack(state, faults, signal, up));
            begin("robot/heartbeat", buf, 4u);
            fprintf(g_out, "\"expect\": {\"state\": %u, \"faults\": %u, "
                    "\"uptime_s\": %u}}\n", state, faults,
                    (unsigned)(up % 4096u));
            if (0u != signal)
            {
                begin("robot/heartbeat", buf, 4u);
                fprintf(g_out, "\"expect\": {\"signal_dbm\": -%u}}\n", signal);
            }
        }

        /* Line word (S1). */
        {
            subsystem1_line_t line;

            line.error_mm   = (int8_t)rnd(-40, 40);
            line.b_valid    = (0 != rnd(0, 1));
            line.b_lost     = (0 != rnd(0, 1));
            line.b_junction = (0 != rnd(0, 1));
            comms_put_u16(buf, line_sense_pack(&line));
            begin("robot/telemetry/line", buf, 2u);
            fprintf(g_out, "\"expect\": {\"error_mm\": %d, \"valid\": %s, "
                    "\"lost\": %s, \"junction\": %s}}\n", line.error_mm,
                    jbool(line.b_valid), jbool(line.b_lost),
                    jbool(line.b_junction));
        }

        /* Odometry word + encoder counts (S2). */
        {
            subsystem2_odom_t odom;
            subsystem2_odom_t back;
            uint16_t          left  = (uint16_t)rnd(0, 65535);
            uint16_t          right = (uint16_t)rnd(0, 65535);

            odom.distance_mm = (uint16_t)rnd(0, 32767);
            odom.left_mm_s   = (int16_t)(5 * rnd(-64, 63));
            odom.right_mm_s  = (int16_t)(5 * rnd(-64, 63));
            comms_put_u32(buf, motion_odom_pack(&odom));
            comms_put_u16(&buf[4], left);
            comms_put_u16(&buf[6], right);
            motion_odom_unpack(motion_odom_pack(&odom), &back);
            begin("robot/telemetry/motion", buf, 8u);
            fprintf(g_out, "\"expect\": {\"distance_mm\": %u, \"left_mm_s\": %d, "
                    "\"right_mm_s\": %d, \"left_count\": %u, "
                    "\"right_count\": %u}}\n", back.distance_mm,
                    back.left_mm_s, back.right_mm_s, left, right);
        }

        /* IMU status word (S3). */
        {
            subsystem3_status_t st;
            uint32_t            word;

            st.pitch_q       = (int16_t)rnd(-128, 127);
            st.heading_deg   = (0 == rnd(0, 9)) ? S3_HEADING_NONE
                                                : (uint16_t)rnd(0, 359);
            st.hump          = (subsystem3_hump_state_t)rnd(0, 4);
            st.motion        = (subsystem3_motion_t)rnd(0, 7);
            st.b_hump_active = (0 != rnd(0, 1));
            word = imu_status_pack(&st);
            comms_put_u32(buf, word);
            begin("robot/telemetry/imu", buf, 3u);
            fprintf(g_out, "\"expect\": {\"pitch_q\": %d, \"hump\": %u, "
                    "\"motion\": %u, \"hump_active\": %s",
                    st.pitch_q, (unsigned)st.hump, (unsigned)st.motion,
                    jbool(st.b_hump_active));
            if (S3_HEADING_NONE == st.heading_deg)
            {
                fprintf(g_out, ", \"heading_deg\": null}}\n");
            }
            else
            {
                fprintf(g_out, ", \"heading_deg\": %u}}\n", st.heading_deg);
            }
        }

        /* Barcode message (S1): index 6 bits + reversed bit. */
        {
            uint8_t index = (uint8_t)rnd(0, 42);
            bool    b_rev = (0 != rnd(0, 1));
            char    c     = barcode_index_to_char(index);

            buf[0] = (uint8_t)(index | (b_rev ? S1_BC_REVERSED_BIT : 0u));
            begin("robot/telemetry/barcode", buf, 1u);
            fprintf(g_out, "\"expect\": {\"char\": \"%c\", \"reversed\": %s, "
                    "\"command\": \"%s\"}}\n", c, jbool(b_rev),
                    cmd_name(barcode_char_to_cmd(c)));
        }

        /* Hump record (S3). */
        {
            uint8_t  last = (uint8_t)rnd(0, 200);
            uint8_t  max  = (uint8_t)rnd(0, 200);
            uint16_t rec  = imu_hump_record_pack(last, max);

            comms_put_u16(buf, rec);
            begin("robot/telemetry/hump", buf, 2u);
            fprintf(g_out, "\"expect\": {\"last_mm\": %u, \"max_mm\": %u}}\n",
                    (last > 127u) ? 127u : last, (max > 127u) ? 127u : max);
        }

        /* Obstacle profile (S4). */
        {
            subsystem4_profile_t p;

            p.width_cm       = (uint8_t)rnd(0, 127);
            p.closest_cm     = (uint8_t)rnd(0, 127);
            p.closest_index  = (uint8_t)rnd(0, 24);
            p.left_clear_cm  = (uint8_t)rnd(0, 127);
            p.right_clear_cm = (uint8_t)rnd(0, 127);
            p.b_left_edge    = (0 != rnd(0, 1));
            p.b_right_edge   = false;
            scan_profile_pack(&p, buf);
            begin("robot/telemetry/obstacle", buf, 5u);
            fprintf(g_out, "\"expect\": {\"closest_index\": %u, "
                    "\"closest_angle_deg\": %u, \"detected\": %s",
                    p.closest_index, scan_index_to_deg(p.closest_index),
                    jbool(p.b_left_edge));
            fprintf(g_out, (127u == p.width_cm) ? ", \"width_cm\": null"
                                                : ", \"width_cm\": %u",
                    p.width_cm);
            fprintf(g_out, (127u == p.left_clear_cm) ? ", \"left_clear_cm\": null"
                                                     : ", \"left_clear_cm\": %u",
                    p.left_clear_cm);
            fprintf(g_out, (127u == p.closest_cm) ? ", \"closest_cm\": null}}\n"
                                                  : ", \"closest_cm\": %u}}\n",
                    p.closest_cm);
        }

        /* State event (S4) and comms counters (S5). */
        buf[0] = (uint8_t)rnd(0, 9);
        begin("robot/telemetry/state", buf, 1u);
        fprintf(g_out, "\"expect\": {\"state\": %u}}\n", buf[0]);
        buf[0] = (uint8_t)rnd(0, 255);
        buf[1] = (uint8_t)rnd(0, 255);
        buf[2] = (uint8_t)rnd(0, 255);
        begin("robot/telemetry/comms", buf, 3u);
        fprintf(g_out, "\"expect\": {\"drops\": %u, \"bad_commands\": %u, "
                "\"barcode_rejects\": %u}}\n", buf[0], buf[1], buf[2]);
    }

    /* Commands: the firmware's validity for every possible byte, plus the
     * dashboard's encodings of the named commands. */
    for (i = 0u; i < 256u; i++)
    {
        uint8_t raw = (uint8_t)i;
        uint8_t out[2] = {0u, 0u};
        bool    b_ok = comms_command_decode(&raw, 1u, out);

        begin("robot/command", &raw, 1u);
        fprintf(g_out, "\"valid\": %s", jbool(b_ok));

        if (b_ok && ((raw & S4_RCMD_OP_MASK) == S4_RCMD_SPEED))
        {
            fprintf(g_out, ", \"name\": \"speed\", \"speed\": %u}\n",
                    (unsigned)(raw >> S4_RCMD_ARG_SHIFT));
        }
        else if (b_ok && ((raw & S4_RCMD_OP_MASK) == S4_RCMD_GAIN))
        {
            /* The dashboard sends 100..2500 (the robot clamps above 25):
             * only those must encode back to the same byte. */
            if ((raw >> S4_RCMD_ARG_SHIFT) <= 25u)
            {
                fprintf(g_out, ", \"name\": \"gain\", \"speed\": %u}\n",
                        (unsigned)(raw >> S4_RCMD_ARG_SHIFT) * 100u);
            }
            else
            {
                fprintf(g_out, "}\n");
            }
        }
        else if (b_ok)
        {
            /* Nov 2026: was indexed past its end by GAIN (op 7), which
             * printed "(null)" and failed the round trip. */
            static char const * const names[S4_RCMD_RESET + 1u] =
            {
                "", "start", "pause", "resume", "estop", "reset"
            };
            uint8_t op = (uint8_t)(raw & S4_RCMD_OP_MASK);

            fprintf(g_out, ", \"name\": \"%s\"}\n",
                    (op <= S4_RCMD_RESET) ? names[op] : "?");
        }
        else
        {
            fprintf(g_out, "}\n");
        }
    }

    /* Two-byte barcode map frames: every second byte after S4_RCMD_MAP,
     * and a wrong first byte. */
    for (i = 0u; i < 256u; i++)
    {
        uint8_t frame[2];
        uint8_t out[2] = {0u, 0u};
        bool    b_ok   = false;

        frame[0] = (uint8_t)S4_RCMD_MAP;
        frame[1] = (uint8_t)i;
        b_ok     = comms_command_decode(frame, 2u, out);
        begin("robot/command", frame, 2u);
        fprintf(g_out, "\"valid\": %s", jbool(b_ok));

        if (b_ok)
        {
            fprintf(g_out, ", \"name\": \"map\", \"letter\": \"%c\", "
                    "\"move\": %u}\n",
                    (char)('A' + (frame[1] & S4_RCMD_MAP_INDEX_MASK)),
                    (unsigned)(frame[1] >> S4_RCMD_MAP_SHIFT));
        }
        else
        {
            fprintf(g_out, "}\n");
        }
    }

    {
        uint8_t frame[2] = {(uint8_t)S4_RCMD_START, 0x20u};
        uint8_t out[2]   = {0u, 0u};

        begin("robot/command", frame, 2u);
        fprintf(g_out, "\"valid\": %s}\n",
                jbool(comms_command_decode(frame, 2u, out)));
    }

    if (stdout != g_out)
    {
        fclose(g_out);
    }

    return (0);
}

/*** end of file ***/
