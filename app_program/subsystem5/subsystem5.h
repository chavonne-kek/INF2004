/** @file subsystem5.h
 *
 * @brief Subsystem 5 (Buddy 1): WiFi / MQTT telemetry and command bridge
 *        (TSK_COMMS).  PUBLIC interface.
 *
 * Owner: Chavonne (student ID 2502481).
 *
 * TSK_COMMS never calls cyw43, lwIP or MQTT functions (TEAM.md section 3).
 * It exchanges messages with the kernel WiFi service task through two
 * message buffers that the service reads/writes with TMO_POL; the service
 * side is in comms_bridge.c and the IF-07 change to mqtt_utk.c.
 *
 * Interfaces provided:
 *   IF-06  all -> S5  event telemetry records in mbf_telemetry (created
 *                     here, the receiver).  Post with subsystem5_post().
 *                     Never blocks: a full buffer drops the record and
 *                     counts it.
 *   IF-07  S5 <-> kernel WiFi service, through comms_bridge.h.
 *   FAULT_WIFI in flg_faults (S5 owns the bit).
 *
 * Periodic telemetry (line, odometry, IMU status) is NOT posted by the
 * other tasks: TSK_COMMS reads their single-writer words itself at 5 Hz.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include <stdint.h>
#include <stdbool.h>

#include "../app.h"

/* Task (TEAM.md section 4). */
#define S5_PRI_COMMS           20      /* Core 0, lowest application task. */
#define S5_PERIOD_MS           200u    /* 5 Hz telemetry.                  */
#define S5_HEARTBEAT_MS        1000u

/* ---------------------------------------------------------------------
 * Event telemetry record types (IF-06).  A record is 1 type byte plus a
 * fixed-size packed payload, little-endian.
 * ------------------------------------------------------------------- */
typedef enum
{
    S5_TLM_BARCODE = 0,     /* 1 byte: S1 barcode message.          */
    S5_TLM_HUMP,            /* 2 bytes: S3 hump record.             */
    S5_TLM_OBSTACLE,        /* 5 bytes: S4 obstacle profile.        */
    S5_TLM_STATE,           /* 1 byte: new S4 system state.         */
    S5_TLM_BCTRACE,         /* 8 bytes: one part of a barcode trace
                             * (18 Nov, S1; not an event: QoS 0, not
                             * kept offline).  See MQTT_PROTOCOL.md.   */
    S5_TLM_EVENT_TYPES
} subsystem5_tlm_t;

#define S5_TLM_PAYLOAD_MAX     8u      /* Largest payload on the wire. */

/* ---------------------------------------------------------------------
 * Heartbeat (robot/heartbeat), 29 bits in 4 bytes, little-endian:
 *   bits  0-3   system state (subsystem4_state_t)
 *   bits  4-9   fault mask (FAULT_* in subsystem4.h)
 *   bits 10-16  signal: RSSI magnitude in dBm (0..100, 0 = unknown)
 *   bits 17-28  uptime, s (0..4095, wraps)
 *   bits 29-31  zero
 * ------------------------------------------------------------------- */
#define S5_HB_FAULT_SHIFT      4u
#define S5_HB_SIGNAL_SHIFT     10u
#define S5_HB_UPTIME_SHIFT     17u
#define S5_HB_SIGNAL_MASK      0x7Fu
#define S5_HB_UPTIME_MASK      0x0FFFu

/* Link state (statechart 6.4), 2 bits. */
typedef enum
{
    S5_LINK_CONNECTING = 0,
    S5_LINK_ONLINE,
    S5_LINK_OFFLINE,
    S5_LINK_FLUSH
} subsystem5_link_t;

/* --- Telemetry (any task; never blocks) ------------------------------ */
ER   subsystem5_post(subsystem5_tlm_t type, uint8_t const * p_data,
                     uint8_t len);

/* --- Readers ---------------------------------------------------------- */
subsystem5_link_t subsystem5_link_state(void);
uint16_t subsystem5_drop_count(void);
uint16_t subsystem5_bad_command_count(void);

#endif /* SUBSYSTEM5_H */

/*** end of file ***/
