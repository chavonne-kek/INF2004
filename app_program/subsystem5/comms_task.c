/** @file comms_task.c
 *
 * @brief TSK_COMMS (priority 20, core 0): statechart 6.4.
 *
 *   CONNECTING --session up--> ONLINE (or FLUSH if events were kept)
 *   CONNECTING --10 s--> OFFLINE
 *   ONLINE --session down--> OFFLINE (+ FAULT_WIFI)
 *   OFFLINE --session up--> FLUSH --kept events sent--> ONLINE
 *
 * Every 200 ms: drain mbf_telemetry; when ONLINE queue periodic telemetry
 * (line, motion, IMU) and once a second the heartbeat and comms counters.
 * Offline, periodic data is dropped (only fresh data is useful) and event
 * records are kept in a 16-entry buffer (NFR13).
 *
 * Commands from robot/command arrive in mbf_mqtt_rx and wake the task at
 * once; valid ones go to TSK_CTRL (mbf_remote_cmd + EVT_REMOTE_CMD).
 *
 * The robot carries on in every state: a WiFi failure never stops it.
 */

#include "subsystem5.h"
#include "comms_task.h"
#include "comms_codec.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem4/b5_task.h"

#define COMMS_CONNECT_TIMEOUT_MS  10000u
#define COMMS_TLM_MBF_BYTES       256
#define COMMS_TX_MBF_BYTES        512
#define COMMS_RX_MBF_BYTES        64
#define COMMS_FLUSH_PER_TICK      8u
#define COMMS_COUNT_MAX           0xFFFFu
#define COMMS_BYTE_MAX            0xFFu
#define COMMS_MS_PER_S            1000u
/* Serial debug print (Tribunal 1: binary payloads are decoded on the
 * console too, so the demo can be followed without the dashboard).  Off
 * in test builds so it does not interleave with the [TEST] lines. */
#if APP_TEST && (APP_CHECK != 5)    /* The S5 check image shows it. */
#define COMMS_DEBUG_PRINT         0
#else
#define COMMS_DEBUG_PRINT         1
#endif
#define COMMS_DEBUG_EVERY_S       2u

static ID gh_comms_task = 0;
static ID gh_tlm_mbf    = 0;    /* mbf_telemetry: all tasks -> TSK_COMMS. */
static ID gh_tx_mbf     = 0;    /* mbf_mqtt_tx: TSK_COMMS -> service.     */
static ID gh_rx_mbf     = 0;    /* mbf_mqtt_rx: service -> TSK_COMMS.     */

static volatile uint8_t  g_link     = (uint8_t)S5_LINK_CONNECTING;
static volatile bool     gb_session = false;   /* Written by the service. */
static volatile uint8_t  g_signal   = 0u;      /* Written by the service. */
static volatile uint16_t g_drops    = 0u;      /* Posts to a full buffer. */
static volatile uint16_t g_bad      = 0u;      /* Malformed commands.    */
static volatile uint16_t g_tx_full  = 0u;      /* mbf_mqtt_tx full.      */

static comms_ring_t g_ring;

static void comms_thread(INT stacd, void * p_exinf);
static void handle_command(uint8_t const * p_data, INT len);
static void update_link(uint32_t now_ms, uint32_t * p_since_ms);
static void drain_events(void);
static void send_periodic(bool b_second, uint32_t now_ms);
static void send_msg(comms_topic_t topic, uint8_t const * p_payload,
                     uint8_t len);
static void send_raw(comms_msg_t const * p_msg);
static uint32_t now_ms(void);
static uint16_t count_up(uint16_t count);
#if COMMS_DEBUG_PRINT
static void debug_print(uint32_t time_ms);
#endif


/*!
 * @brief Create mbf_telemetry, mbf_mqtt_tx and mbf_mqtt_rx.
 */
ER
comms_task_init (void)
{
    T_CMBF cmbf   = {0};
    ER     result = E_OK;

    comms_ring_init(&g_ring);

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = COMMS_TLM_MBF_BYTES;
    cmbf.maxmsz = (INT)(1u + S5_TLM_PAYLOAD_MAX);
    gh_tlm_mbf  = tk_cre_mbf(&cmbf);

    cmbf.bufsz  = COMMS_TX_MBF_BYTES;
    cmbf.maxmsz = (INT)COMMS_MSG_MAX;
    gh_tx_mbf   = tk_cre_mbf(&cmbf);

    cmbf.bufsz  = COMMS_RX_MBF_BYTES;
    cmbf.maxmsz = (INT)COMMS_CMD_MAX_LEN;
    gh_rx_mbf   = tk_cre_mbf(&cmbf);

    if (gh_tlm_mbf < E_OK)
    {
        result = (ER)gh_tlm_mbf;
    }
    else if (gh_tx_mbf < E_OK)
    {
        result = (ER)gh_tx_mbf;
    }
    else if (gh_rx_mbf < E_OK)
    {
        result = (ER)gh_rx_mbf;
    }
    else
    {
        result = E_OK;
    }

    return (result);
}

/*!
 * @brief Start TSK_COMMS on core 0.
 */
ER
comms_task_start (void)
{
    gh_comms_task = app_start_task((FP)comms_thread, S5_PRI_COMMS, TP_PRC1,
                                   "comms_task");   /* FP: kernel entry. */

    return ((gh_comms_task < E_OK) ? (ER)gh_comms_task : E_OK);
}

/*!
 * @brief Post an event record (IF-06).  Never blocks.
 */
ER
comms_task_post (subsystem5_tlm_t type, uint8_t const * p_data, uint8_t len)
{
    uint8_t record[1u + S5_TLM_PAYLOAD_MAX];
    uint8_t index  = 0u;
    ER      result = E_PAR;

    if ((len == comms_event_len(type)) && (0u != len))
    {
        record[0] = (uint8_t)type;

        for (index = 0u; index < len; index++)
        {
            record[index + 1u] = p_data[index];
        }

        result = tk_snd_mbf(gh_tlm_mbf, record, (INT)(len + 1u), TMO_POL);

        if (E_OK != result)
        {
            g_drops = count_up(g_drops);
        }
    }

    return (result);
}

ID
comms_task_tx_mbf (void)
{
    return (gh_tx_mbf);
}

ID
comms_task_rx_mbf (void)
{
    return (gh_rx_mbf);
}

uint8_t
comms_task_link (void)
{
    return (g_link);
}

uint16_t
comms_task_drops (void)
{
    return (g_drops);
}

uint16_t
comms_task_bad (void)
{
    return (g_bad);
}


void
comms_task_set_connected (bool b_up)
{
    gb_session = b_up;
}

void
comms_task_set_signal (uint8_t signal)
{
    g_signal = signal;
}

/*!
 * @brief TSK_COMMS body.
 */
static void
comms_thread (INT stacd, void * p_exinf)
{
    uint32_t next_tick   = now_ms();
    uint32_t next_second = next_tick;
    uint32_t link_since  = next_tick;

    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        uint8_t  cmd[COMMS_CMD_MAX_LEN];
        uint32_t now  = now_ms();
        TMO      wait = (TMO)(((int32_t)(next_tick - now) > 0) ?
                              (next_tick - now) : 0u);
        INT      size = tk_rcv_mbf(gh_rx_mbf, cmd, wait);

        if (size > 0)
        {
            handle_command(cmd, size);
            continue;   /* Commands first; telemetry at the next tick. */
        }

        now = now_ms();

        if ((int32_t)(now - next_tick) < 0)
        {
            continue;
        }

        next_tick += S5_PERIOD_MS;

        update_link(now, &link_since);
        drain_events();

        if ((int32_t)(now - next_second) >= 0)
        {
            next_second += S5_HEARTBEAT_MS;
            send_periodic(true, now);
#if COMMS_DEBUG_PRINT
            debug_print(now);
#endif
        }
        else
        {
            send_periodic(false, now);
        }
    }
}

/*!
 * @brief Validate a command and forward it to TSK_CTRL.
 */
static void
handle_command (uint8_t const * p_data, INT len)
{
    uint8_t forward[S4_RCMD_REC_LEN];

    if ((len > 0) &&
        comms_command_decode(p_data, (uint8_t)len, forward) &&
        (E_OK == tk_snd_mbf(subsystem4_remote_cmd_mbf(), forward,
                            (INT)S4_RCMD_REC_LEN, TMO_POL)))
    {
        subsystem4_event_set(EVT_REMOTE_CMD);
    }
    else
    {
        g_bad = count_up(g_bad);
        tm_printf((UB *)"S5: malformed command dropped (%u)\n", (UINT)g_bad);
    }
}

/*!
 * @brief Link statechart; FAULT_WIFI is owned (set/cleared) here.
 */
static void
update_link (uint32_t now, uint32_t * p_since_ms)
{
    subsystem5_link_t link   = (subsystem5_link_t)g_link;
    bool              b_up   = gb_session;
    subsystem5_link_t before = link;

    switch (link)
    {
        case S5_LINK_CONNECTING:
            if (b_up)
            {
                link = (g_ring.count > 0u) ? S5_LINK_FLUSH : S5_LINK_ONLINE;
            }
            else if ((now - *p_since_ms) >= COMMS_CONNECT_TIMEOUT_MS)
            {
                link = S5_LINK_OFFLINE;
            }
            else
            {
                /* Still joining WiFi / connecting to the broker. */
            }
        break;

        case S5_LINK_ONLINE:
            if (!b_up)
            {
                link = S5_LINK_OFFLINE;
            }
        break;

        case S5_LINK_OFFLINE:
            if (b_up)
            {
                link = (g_ring.count > 0u) ? S5_LINK_FLUSH : S5_LINK_ONLINE;
            }
        break;

        case S5_LINK_FLUSH:
        default:
            if (!b_up)
            {
                link = S5_LINK_OFFLINE;
            }
            else
            {
                comms_msg_t msg;
                uint8_t     sent = 0u;

                while ((sent < COMMS_FLUSH_PER_TICK) &&
                       comms_ring_pop(&g_ring, &msg))
                {
                    send_raw(&msg);
                    sent++;
                }

                if (0u == g_ring.count)
                {
                    link = S5_LINK_ONLINE;
                }
            }
        break;
    }

    if (link != before)
    {
        *p_since_ms = now;
        g_link      = (uint8_t)link;

        if (S5_LINK_OFFLINE == link)
        {
#if defined(TM_MQTT)
            subsystem4_fault_raise(FAULT_WIFI);
#endif
        }
        else if ((S5_LINK_ONLINE == link) || (S5_LINK_FLUSH == link))
        {
            subsystem4_fault_clear(FAULT_WIFI);
        }
        else
        {
            /* CONNECTING: no change. */
        }

#if defined(TM_MQTT)
        {
            static char const * const names[4] =
            {
                "CONNECTING", "ONLINE", "OFFLINE", "FLUSH"
            };

            tm_printf((UB *)"S5: link %s\n", names[(uint8_t)link & 0x3u]);
        }
#else
        tm_printf((UB *)"S5: link OFFLINE (no MQTT in this build)\n");
#endif
    }
}

/*!
 * @brief Event records: forward when online, keep when not.
 */
static void
drain_events (void)
{
    uint8_t record[1u + S5_TLM_PAYLOAD_MAX];
    INT     size = 0;

    while ((size = tk_rcv_mbf(gh_tlm_mbf, record, TMO_POL)) > 1)
    {
        comms_msg_t msg;
        uint8_t     index = 0u;

        msg.data[0] = (uint8_t)comms_event_topic((subsystem5_tlm_t)record[0]);

        for (index = 1u; index < (uint8_t)size; index++)
        {
            msg.data[index] = record[index];
        }

        msg.len = (uint8_t)size;

        if (S5_LINK_ONLINE == (subsystem5_link_t)g_link)
        {
            send_raw(&msg);
        }
        else if (comms_topic_is_event((comms_topic_t)msg.data[0]))
        {
            comms_ring_push(&g_ring, &msg);     /* NFR13: keep events. */
        }
        else
        {
            /* A barcode trace (18 Nov) is a live view: not kept offline,
             * so it cannot push real events out of the ring. */
        }
    }
}

/*!
 * @brief Periodic telemetry at 5 Hz; heartbeat and counters at 1 Hz.
 *        Read from the other subsystems' single-writer words.
 */
static void
send_periodic (bool b_second, uint32_t now)
{
    uint8_t  payload[S5_TLM_PAYLOAD_MAX];
    uint16_t left  = 0u;
    uint16_t right = 0u;

    if (S5_LINK_ONLINE != (subsystem5_link_t)g_link)
    {
        return;     /* Offline: periodic data is dropped. */
    }

    {
        /* Raw IR levels, sent FIRST: last in the tick meant the transmit
         * queue was already full and it arrived about once every 8 s
         * (2 Nov).  The arm level is the one that matters while a barcode
         * passes, so it gets priority over line and motion. */
        uint16_t ir[3];

        subsystem1_line_raw(ir);
        comms_put_u16(&payload[0], ir[0]);
        comms_put_u16(&payload[2], ir[1]);
        comms_put_u16(&payload[4], ir[2]);
        send_msg(COMMS_TOPIC_IR, payload, 6u);
    }

    {
        /* 22 Nov: sonar watch, so the approach to an obstacle can be followed
         * on the dashboard without a USB cable - front cm (255 = no echo) and
         * how many confirming hits the trigger has. */
        payload[0] = B5_WatchCm();
        payload[1] = B5_WatchHits();
        send_msg(COMMS_TOPIC_RANGE, payload, 2u);
    }

    {
        subsystem1_line_t line;
        uint16_t          word = 0u;

        (void)subsystem1_line_read(&line);
        word = (uint16_t)((uint16_t)((int16_t)line.error_mm +
                                     (int16_t)S1_LINE_ERR_OFFSET) &
                          S1_LINE_ERR_MASK);
        word |= line.b_valid ? S1_LINE_VALID_BIT : 0u;
        word |= line.b_lost ? S1_LINE_LOST_BIT : 0u;
        word |= line.b_junction ? S1_LINE_JUNCTION_BIT : 0u;
        comms_put_u16(payload, word);
        send_msg(COMMS_TOPIC_LINE, payload, 2u);
    }

    comms_put_u32(payload, subsystem2_odometry_word());
    subsystem2_encoder_counts(&left, &right);
    comms_put_u16(&payload[4], left);
    comms_put_u16(&payload[6], right);
    send_msg(COMMS_TOPIC_MOTION, payload, 8u);

    if (b_second)
    {
        /* IMU status once a second, not five times: every message saved
         * here is headroom for the IR and barcode traffic.  The hump record
         * and FAULT_LIFTED (heartbeat) are what the demo needs from S3. */
        comms_put_u32(payload, subsystem3_status_word());
        send_msg(COMMS_TOPIC_IMU, payload, 3u);     /* 24 bits used. */
    }

    if (b_second)
    {
        uint32_t hb = comms_heartbeat_pack((uint8_t)subsystem4_state(),
                                           (uint8_t)subsystem4_faults(),
                                           g_signal, now / COMMS_MS_PER_S);

        comms_put_u32(payload, hb);
        send_msg(COMMS_TOPIC_HEARTBEAT, payload, 4u);

        payload[0] = (uint8_t)((g_drops > COMMS_BYTE_MAX) ? COMMS_BYTE_MAX
                                                          : g_drops);
        payload[1] = (uint8_t)((g_bad > COMMS_BYTE_MAX) ? COMMS_BYTE_MAX
                                                        : g_bad);
        {
            subsystem1_diag_t diag;     /* Statechart 6.2: barcode rejects. */

            subsystem1_diag_get(&diag);
            payload[2] = (uint8_t)((diag.barcode_errors > COMMS_BYTE_MAX) ?
                                   COMMS_BYTE_MAX : diag.barcode_errors);
        }
        send_msg(COMMS_TOPIC_COMMS, payload, 3u);
    }
}

/*!
 * @brief Queue one message for the service task: topic ID + payload.
 */
static void
send_msg (comms_topic_t topic, uint8_t const * p_payload, uint8_t len)
{
    comms_msg_t msg;
    uint8_t     index = 0u;

    msg.data[0] = (uint8_t)topic;

    for (index = 0u; (index < len) && (index < S5_TLM_PAYLOAD_MAX); index++)
    {
        msg.data[index + 1u] = p_payload[index];
    }

    msg.len = (uint8_t)(index + 1u);
    send_raw(&msg);
}

static void
send_raw (comms_msg_t const * p_msg)
{
    if (E_OK != tk_snd_mbf(gh_tx_mbf, (void *)p_msg->data, (INT)p_msg->len,
                           TMO_POL))  /* Cast: kernel API is not const. */
    {
        g_tx_full = count_up(g_tx_full);
    }
}

/*!
 * @brief Milliseconds since start-up (heartbeat uptime, 5 Hz pacing).
 *        tk_get_otm, not tk_get_tim: the system clock counts from the
 *        TRON epoch (1985) and is not zero at start-up.
 */
static uint32_t
now_ms (void)
{
    SYSTIM tim;

    (void)tk_get_otm(&tim);

    return ((uint32_t)tim.lo);
}

static uint16_t
count_up (uint16_t count)
{
    return ((count < COMMS_COUNT_MAX) ? (uint16_t)(count + 1u) : count);
}

#if COMMS_DEBUG_PRINT
/*!
 * @brief Every 2 s, print what the binary payloads carry, decoded with
 *        the same unpack functions the dashboard decoder mirrors.  Runs
 *        in every link state, so it also works with WiFi down.  Example:
 *   S5: up 242 s | FAULT (STALL IMU) | link OFFLINE, signal n/a |
 *       line -40 mm LOST, IR 310 295 302 | odo 0 mm, L 0 R 0 mm/s |
 *       IMU n/a | barcodes 0 (rejected 0) | dropped 0, bad commands 0
 *        "IR" is the last raw ADC reading (0..4095) of the left, centre and
 *        right sensor: test S1-U12 (white floor vs black line) reads it.
 */
static void
debug_print (uint32_t time_ms)
{
    static char const * const link_names[4] =
    {
        "CONNECTING", "ONLINE", "OFFLINE", "FLUSH"
    };
    static char const * const hump_names[8] =
    {
        "FLAT", "CLIMBING", "CRESTING", "DESCENDING", "INVALID", "?", "?", "?"
    };
    static char const * const motion_names[8] =
    {
        "STATIONARY", "ACCELERATING", "TURNING", "CLIMBING", "DESCENDING",
        "LIFTED", "DECELERATING", "CRUISING"
    };
    static uint32_t     count = 0u;
    subsystem1_line_t   line;
    subsystem1_diag_t   diag;
    subsystem2_odom_t   odom;
    subsystem3_status_t imu;
    char                faults[S4_FAULT_TEXT_SIZE];
    UB                  signal[16];
    UB                  imu_text[64];
    uint32_t            fault_mask = subsystem4_faults();
    uint8_t             link       = (uint8_t)(g_link & 0x3u);

    count++;

    if (0u != (count % COMMS_DEBUG_EVERY_S))
    {
        return;
    }

    (void)subsystem1_line_read(&line);
    subsystem1_diag_get(&diag);
    subsystem2_odometry_read(&odom);
    subsystem3_status_read(&imu);

    if (0u == g_signal)
    {
        (void)tm_sprintf(signal, (UB const *)"n/a");
    }
    else
    {
        (void)tm_sprintf(signal, (UB const *)"-%u dBm", (UINT)g_signal);
    }

    if (S3_HUMP_INVALID == imu.hump)
    {
        (void)tm_sprintf(imu_text, (UB const *)"IMU n/a");
    }
    else
    {
        /* Pitch in 0.25 degree steps -> "-1.25". */
        int32_t  q     = imu.pitch_q;
        uint32_t whole = (uint32_t)((q < 0) ? -q : q) / 4u;
        uint32_t frac  = ((uint32_t)((q < 0) ? -q : q) % 4u) * 25u;

        (void)tm_sprintf(imu_text,
                         (UB const *)"pitch %s%u.%02u deg, hump %s, %s",
                         (q < 0) ? "-" : "", (UINT)whole, (UINT)frac,
                         hump_names[imu.hump & 0x7u],
                         motion_names[imu.motion & 0x7u]);
    }

    tm_printf((UB *)"S5: up %u s | %s (%s) | link %s, signal %s"
              " | line %d mm%s%s, IR arm %u C %u R %u | odo %u mm, L %d R %d mm/s"
              " | %s | barcodes %u (rejected %u) | dropped %u, bad commands"
              " %u\n",
              (UINT)(time_ms / COMMS_MS_PER_S),
              subsystem4_state_name(subsystem4_state()),
              subsystem4_fault_text(fault_mask, faults),
#if defined(TM_MQTT)
              link_names[link],
#else
              "off (no MQTT in this build)",
#endif
              signal, (INT)line.error_mm,
              line.b_lost ? " LOST" : "", line.b_junction ? " JUNCTION" : "",
              (UINT)diag.raw[2], (UINT)diag.raw[1], (UINT)diag.raw[0],
              (UINT)odom.distance_mm, (INT)odom.left_mm_s,
              (INT)odom.right_mm_s, imu_text, (UINT)diag.barcodes,
              (UINT)diag.barcode_errors, (UINT)g_drops, (UINT)g_bad);
#if !defined(TM_MQTT)
    (void)link_names;
    (void)link;
#endif
}
#endif

/*** end of file ***/
