/** @file subsystem5_test.c
 *
 * @brief On-target tests for subsystem 5.  Built only with: make TEST=1
 *        IDs S5-U01 ... must match testing/subsystem5/TEST_LOG.md.
 */

#include "subsystem5.h"
#include "comms_task.h"
#include "comms_codec.h"
#include "comms_bridge.h"
#include "../subsystem1/subsystem1.h"    /* SUBSYSTEM1_CMD_* (S5-U01b). */
#include "../subsystem4/subsystem4.h"
#include "../test_report.h"

#if APP_TEST

void
subsystem5_test (void)
{
    static uint8_t const bad_cmds[5][2] =
    {
        {0x00u, 0u},    /* opcode 0          */
        {0x07u, 0u},    /* opcode 7          */
        {0x09u, 0u},    /* START with speed  */
        {0x0Eu, 0u},    /* SPEED 1 (< 2)     */
        {0xD6u, 0u}     /* SPEED 26 (> 25)   */
    };
    uint8_t  cmd[2] = {0u, 0u};
    uint8_t  index = 0u;
    uint8_t  hump[2] = {5u, 10u};
    uint16_t bad_before = subsystem5_bad_command_count();
    bool     b_ok  = true;

    TEST_INIT();

    /* S5-U01: valid commands accepted, malformed ones rejected. */
    b_ok = comms_command_decode((uint8_t const *)"\x01", 1u, cmd) &&
           (S4_RCMD_START == cmd[0]) &&
           comms_command_decode((uint8_t const *)"\xA6", 1u, cmd);  /* 20 */

    for (index = 0u; index < 5u; index++)
    {
        b_ok = b_ok && (!comms_command_decode(bad_cmds[index], 1u, cmd));
    }

    /* S5-U01b: the 2-byte map frame, valid only for movement 1..4 and a
     * letter in A..Z. */
    {
        uint8_t give[2] = {S4_RCMD_MAP,
                           (uint8_t)((SUBSYSTEM1_CMD_LEFT << S4_RCMD_MAP_SHIFT)
                                     | 2u)};                 /* 'C' -> left */
        uint8_t bad_mv[2] = {S4_RCMD_MAP,
                             (uint8_t)((5u << S4_RCMD_MAP_SHIFT) | 0u)};
        uint8_t bad_ix[2] = {S4_RCMD_MAP,
                             (uint8_t)((SUBSYSTEM1_CMD_LEFT << S4_RCMD_MAP_SHIFT)
                                       | 26u)};

        b_ok = b_ok && comms_command_decode(give, 2u, cmd) &&
               (S4_RCMD_MAP == cmd[0]) &&
               (!comms_command_decode(bad_mv, 2u, cmd)) &&
               (!comms_command_decode(bad_ix, 2u, cmd)) &&
               (!comms_command_decode(give, 3u, cmd));
    }
    TEST_CHECK("S5-U01", b_ok, "command validation (FR16, 1 byte)");

    /* S5-U02: a malformed command through the bridge is counted and
     * changes nothing. */
    comms_bridge_received(COMMS_BRIDGE_TOPIC_COMMAND, bad_cmds[1], 1u);
    (void)tk_dly_tsk(50);
    TEST_CHECK("S5-U02",
               (uint16_t)(bad_before + 1u) == subsystem5_bad_command_count(),
               "malformed command dropped and counted");

    /* S5-U03: heartbeat 29 bits. */
    TEST_CHECK("S5-U03",
               (comms_heartbeat_pack(9u, 0x3Fu, 100u, 4095u) ==
                (9u | (0x3Fu << 4) | (100u << 10) | (4095u << 17))) &&
               (0u == (comms_heartbeat_pack(15u, 0xFFu, 255u, 0xFFFFFu) &
                       0xE0000000u)),
               "heartbeat packs into 29 bits");

    /* S5-U04: posting checks the record length; never blocks. */
    TEST_CHECK("S5-U04",
               (E_PAR == subsystem5_post(S5_TLM_HUMP, hump, 1u)) &&
               (E_OK == subsystem5_post(S5_TLM_HUMP, hump, 2u)),
               "telemetry post: length checked, non-blocking");

    TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */

/*** end of file ***/
