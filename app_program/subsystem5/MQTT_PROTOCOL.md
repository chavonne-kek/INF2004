# IS25 MQTT protocol (subsystem 5, Buddy 1)

All application data is **packed binary**, little-endian, built with masks and
shifts (Change Requirement "Minimum-Bit Data Representation", Tribunal 1).
Topic names stay as text: the topic says which layout the payload has, so
payloads carry no type byte.

The decoder is `tools/is25_dashboard.py`. It is checked against the
firmware's own pack functions by `testing/subsystem5/roundtrip`
(`make run`: 2,273 checks).

## Connection

| Item | Value |
|---|---|
| Broker | Any MQTT 3.1.1 broker, e.g. Mosquitto on a laptop on the same WiFi |
| Settings | `config/wifi_secrets.h`: `WIFI_SSID`, `WIFI_PASSWORD`, `MQTT_BROKER_IP`, `MQTT_BROKER_PORT`, `MQTT_CLIENT_ID` |
| Build | `git apply lib_patches/IF-07_mqtt_utk.patch`, then `make NET=lwip MQTT=1`. Without the patch the build now stops with a message: an unpatched MQTT image only runs the kernel's `hb N` demo on `mtk3smp-pico/heartbeat`, and TSK_COMMS stays OFFLINE with FAULT_WIFI (what the 27 Oct console showed) |
| Keep-alive | 5 s (Tribunal 1: 2 s gave false "offline" on classroom WiFi) |
| Last Will | `robot/status` = `offline`, QoS 1, retained (broker sends it ≤ 7.5 s after a silent drop) |
| On connect | `robot/status` = `online` (retained), subscribe `robot/command` (QoS 1) |
| Reconnect | WiFi rejoins by itself; MQTT retries every 3 s (`mqtt_utk.c`) |
| Offline | Periodic data is dropped; the last 16 **events** are kept and sent on reconnect (statechart 6.4) |
| QoS | Periodic telemetry QoS 0, events QoS 1 |

Only the kernel's WiFi service task touches the radio, lwIP or MQTT.
TSK_COMMS swaps messages with it through two message buffers, always with
`TMO_POL`, so a WiFi problem never blocks the robot.

## Periodic topics (5 Hz; heartbeat and comms 1 Hz)

| Topic | Bytes | Layout (bit 0 = least significant bit of byte 0) |
|---|---|---|
| `robot/heartbeat` | 4 | 0–3 system state · 4–9 fault mask · 10–16 WiFi signal, −dBm (0 = unknown) · 17–28 uptime s since start-up (wraps at 4096) |
| `robot/telemetry/line` | 2 | 0–6 line error + 40 (−40..+40 mm, + = steer left) · 7 valid · 8 lost · 9 junction |
| `robot/telemetry/motion` | 8 | bytes 0–3: 0–14 distance mm (wraps at 32768) · 15–21 left speed (raw − 64) × 5 mm/s · 22–28 right speed; bytes 4–5 left encoder count; bytes 6–7 right encoder count |
| `robot/telemetry/imu` | 3 | 0–7 pitch (raw − 128) × 0.25° · 8–16 heading 0–359° (511 = not calibrated) · 17–19 hump state · 20–22 motion class · 23 hump active |
| `robot/telemetry/comms` | 3 | byte 0 telemetry records dropped · byte 1 malformed commands · byte 2 barcode reads rejected (statechart 6.2); all saturate at 255 |
| `robot/telemetry/bctrace` | 8 | 18 Nov, QoS 0, not kept offline: one part of the element widths of the last barcode read (decoded, or rejected with 4+ elements). Part 0: byte 0 = seq << 4 \| 0, byte 1 element count (≤ 29), byte 2 the `robot/telemetry/barcode` byte, byte 3 bit 0 = widths in travel (unit × 1/16 mm) else time (unit × 1 ms), byte 4 unit, byte 5 darkest arm level / 4, byte 6 lightest / 4 (0..1000), byte 7 width 0. Part k = 1..4: byte 0 = seq << 4 \| k, bytes 1–7 widths 7k−6 .. 7k. Widths are width / unit, rounded; element 0 is black and they alternate |

## Event topics (sent when they happen; kept while offline)

| Topic | Bytes | Layout |
|---|---|---|
| `robot/telemetry/barcode` | 1 | 0–5 Code 39 index into `0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%` · 6 read backwards |
| `robot/telemetry/hump` | 2 | 0–6 last hump mm · 7–13 highest hump of the run mm (both saturate at 127) |
| `robot/telemetry/obstacle` | 5 | 0–6 width cm · 7–13 closest cm · 14–18 angle index (30° + 5° × index) · 19–25 left clearance cm · 26–32 right clearance cm · 33 detected (127 = none / open) |
| `robot/telemetry/state` | 1 | 0–3 new system state |

## Value tables

| Field | Values |
|---|---|
| System state | 0 INIT_CALIBRATE, 1 IDLE, 2 LINE_FOLLOW, 3 TURN_EXECUTE, 4 OBSTACLE_AVOID, 5 LINE_RECOVERY, 6 PAUSED, 7 EMERGENCY_STOP, 8 FAULT, 9 FINISHED |
| Fault bits | 0 STALL, 1 ENCODER, 2 IMU, 3 NO_ECHO, 4 WIFI, 5 LIFTED (robot lifted or overturned: run over; was COLLISION until 17 Nov) |
| Hump state | 0 FLAT, 1 CLIMBING, 2 CRESTING, 3 DESCENDING, 4 INVALID (no IMU data: the rest of the IMU word is not a measurement) |
| Motion class | 0 STATIONARY, 1 ACCELERATING, 2 TURNING, 3 CLIMBING, 4 DESCENDING, 5 IMPACT, 6 DECELERATING, 7 CRUISING |
| Barcode commands | A turn left, B turn right, C straight, D U-turn (at the next junction) |

## Commands: `robot/command`, 1 byte

Bits 0–2 opcode, bits 3–7 argument.

| Command | Byte | Effect |
|---|---|---|
| start | `0x01` | IDLE → LINE_FOLLOW |
| pause | `0x02` | Stop, keep the mission (PAUSED) |
| resume | `0x03` | PAUSED → carry on |
| estop | `0x04` | EMERGENCY_STOP from any state (motors braked) |
| reset | `0x05` | From FAULT / EMERGENCY_STOP / PAUSED / FINISHED → IDLE |
| speed *v* | `0x06 \| v << 3` | Speed limit *v* = 5..25 cm/s (e.g. 15 cm/s = `0x7E`) |

Anything else (wrong length, unknown opcode, non-zero argument, speed out of
range) is ignored and counted in `robot/telemetry/comms` byte 1.

## Dashboard

```sh
pip install paho-mqtt
python tools/is25_dashboard.py --broker <laptop IP> --log run1.csv
```

It prints events as they arrive and a summary line every second (with the
turn rate, from the wheel speeds: (right − left) / track), and logs
every message to the CSV file as evidence. Type `start`, `pause`, `resume`,
`estop`, `reset`, `speed 15`, `bad` (20 malformed commands, which must all be
ignored), `rates` (messages per second per topic) or `quit`.

The robot also prints the same values, decoded, on its USB console every
2 s, so the demo can be followed without WiFi:

```
S5: up 242 s | FAULT (STALL IMU) | link OFFLINE, signal n/a | line -40 mm LOST, IR 310 295 302 | odo 0 mm, L 0 R 0 mm/s | IMU n/a | barcodes 0 (rejected 0) | dropped 0, bad commands 0
```

(`up` is the time since start-up; before 28 Oct the line used the kernel's
calendar clock, which starts at about 3.36 million seconds, and the
heartbeat uptime came from the same wrong clock. `IR` is the console's
extra: the last raw reading, 0–4095, of the left, centre and right IR
sensor, for test S1-U12. It is not in any MQTT message.)
