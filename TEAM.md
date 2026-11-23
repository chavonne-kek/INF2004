# Team and subsystems

IS25 final code (26 Oct): every table below describes the real system. Each
row was taken from the Week 6 design and the Tribunal 1 decisions; change
a row only by team agreement and say so in the commit message.
`IS25_README.md` says what is finished and what each owner still measures.

Fill this in at the start of the project and keep it up to date. It is the
single place that says **who owns what**, so that five people can work in
one firmware image without clashing over pins, priorities, or kernel objects.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | IR line following + Code 39 barcode decoding (Buddy 3, Hao Ming) | 2500331 | | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motors, encoders, PID, odometry (Buddy 2, Kai Xuan) | 2501473 | | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | IMU (LSM303D), pitch, humps, lift / overturn stop (Buddy 4, Zi Siang; his `imu.c` engine) | 2500989 | | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | Ultrasonic + servo scan (Buddy 5 engine `b5_*.c`), vehicle controller (Buddy 5, Soma) | 2402033 | | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | WiFi / MQTT telemetry and commands (Buddy 1, Chavonne) | 2502481 | | `app_program/subsystem5/` | `testing/subsystem5/` |

Shared files (`app_program/app_main.c`, `app_program/app.h`, this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

**Why:** two subsystems using the same pin is one of the most common and
hardest-to-find mistakes in a team project. The code builds without any
error, but the hardware misbehaves. Each subsystem can pass its own tests
and then fail once they are combined, and two outputs driving the same pin
against each other can damage it. Writing the claim down first means a clash
is spotted when someone edits this table, not hours later on the bench.

**No instructor check needed.** The team keeps this table itself. You do not
need the instructor, or the rest of the team, to check or approve a claim:
if the row is empty, fill it in and commit. If two people claim the same
pin, they agree between themselves who moves. Only the *reserved* rows are
fixed.

As built on 26 Oct (`IS25_BUILD_LOG_2026-10-26.md`). Grove cables in this
kit: **white = pin 1 = the port's first GPIO, yellow = pin 2 = second GPIO,
red = 3V3, black = GND**.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | S2 | Left encoder (Grove 1, white), input + pull-up, both-edge IRQ. **So `CRASH_REPORT=1` / `CONSOLE=uart` cannot be used** (build stops) |
| GP1 | S2 | Left encoder (Grove 1, yellow), input + pull-up, both-edge IRQ: the wheel is counted on whichever of GP0/GP1 carries pulses (phase B with `ENC_USE_PHASE_B`) |
| GP2 | S2 | Right encoder (Grove 2, white), input + pull-up, both-edge IRQ |
| GP3 | S2 | Right encoder (Grove 2, yellow), input + pull-up, both-edge IRQ: counted on whichever of GP2/GP3 carries pulses |
| GP4 | S4 | HC-SR04 trigger (Grove 3), `B5_PIN_SONAR_TRIG` |
| GP5 | S4 | HC-SR04 echo (Grove 3, yellow), shared GPIO IRQ, `B5_PIN_SONAR_ECHO`. Module on 3V3, so no divider (a divider is needed if it is ever moved to 5 V) |
| GP6 | | Grove 5 white wire, taped off (Grove 5 is the left IR sensor on GP26) |
| GP7 | | Grove 7 white wire, taped off (Grove 7 is the right IR sensor on GP28) |
| GP8 | S2 | Motor M1A (left), PWM slice 4A 20 kHz |
| GP9 | S2 | Motor M1B (left), PWM slice 4B |
| GP10 | S2 | Motor M2A (right), PWM slice 5A |
| GP11 | S2 | Motor M2B (right), PWM slice 5B |
| GP12 | | |
| GP13 | | |
| GP14 | | |
| GP15 | S4 | Scan servo, PWM slice 7B 50 Hz |
| GP16 | S3 | I2C0 SDA, IMU (Grove 4) |
| GP17 | S3 | I2C0 SCL, IMU (Grove 4) |
| GP18 | | |
| GP19 | | (S1 can use it as a debug timing probe only if `LINE_TIMING_PIN_ENABLE` is set to 1 in `line_task.c`; off by default) |
| GP20 | S4 | Start button (on-board, active low) |
| GP21 | S4 | Reset button (on-board, active low) |
| GP22 | S4 | Piezo buzzer, PWM slice 3A |
| GP26 | S1 | ADC0: left IR sensor AO |
| GP27 | S1 | ADC1: centre IR sensor AO |
| GP28 | S1 | ADC2: right IR sensor AO (also reads the barcode) |

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "environment sensing", "motor control", "user interface" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "environment sensing" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: the "environment sensing" subsystem contains the I2C
sensor code, and owns I2C0. Write it as `drv_<name>.c` / `drv_<name>.h` in
that subsystem's folder, never under `device/`, and call it only from the
peripheral's owning task. The kernel's own drivers in `device/` (serial,
I2C, ADC) may be used instead; switching one on in `config/config_device.h`
changes shared configuration, so record it in the table below first.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** That task can be in any subsystem, and one subsystem may own
several peripherals (one owning task each, or one task owning all of them).
Every other task, in the same subsystem or another one, asks the owning task
through a kernel object (section 5) and never touches that peripheral itself.

*Why:* this port runs tasks on both cores at the same time. If two tasks
drive the same I2C or UART block at once, their transfers get mixed up, and
the drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART1 | | | |
| I2C0 (kernel driver `iica`) | S3 | `imu_task` | GP16 / GP17 (`DEVCNF_USE_IIC` = 1, pins in `i2c_cnf_sysdep.h`) |
| I2C1 | | | |
| SPI0 | | | |
| SPI1 | | | |
| ADC (kernel driver `adca`) | S1 | `line_task` | GP26–GP28 (`DEVCONF_ADC_PIN_INIT_0..2` = TRUE) |
| PWM slices 4, 5 (motors) | S2 | `motion_task` | GP8-GP11 |
| PWM slice 7 (servo), slice 3 (buzzer) | S4 | `scan_task` (servo), `ctrl_task` (buzzer) | GP15, GP22 |
| GPIO bank-0 interrupt (encoders + echo) | S2 | handler enabled by `motion_task` on core 1; S4 attaches GP5 with `subsystem2_gpio_attach()` | GP0–GP3, GP5 |
| Watchdog (and its 1 us tick) | S4 | `ctrl_task` (fed only if `motion_task` is alive) | - |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions, not even from a
"WiFi subsystem".** The calls are not safe from any other task, on either core.

What students *can* do:

- **Read the connection state** from any task, which is always safe:
  ```c
  #include "cyw43_utk.h"
  T_CYW43_UTK_STATUS st;
  cyw43_utk_get_status(&st);   /* st.ready, st.link_status, st.ip_addr, ... */
  ```
- **Send or receive data over the network** by adding code that *runs
  inside the service task*. The MQTT client is in
  `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`. It publishes a heartbeat
  today. A "cloud / reporting" subsystem would add a kernel object
  (e.g. a message buffer) that application tasks fill. The MQTT code then
  reads it with `TMO_POL` (never blocking) and publishes it.
  Because this changes shared library code, **agree it with the team and
  the instructor first**, and record it as an interface in section 5.
  It needs the `NET=lwip MQTT=1` build (README section 3).

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4). Each subsystem gets its own band by default;
change it only by team agreement. Write down every task you create.

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| 1 | 12–13 | line_task – 12 – core 1 (woken every 2 ms by a cyclic handler) |
| 2 | 14–15 | motion_task – 14 – core 1 (cyclic 10 ms) |
| 3 | 16–17 | imu_task – 16 – core 0 (cyclic 10 ms) |
| 4 | 18–19 | ctrl_task – 18 – core 1 (event driven, 50 ms timeout); scan_task – 19 – core 0 (60 ms ping loop) |
| 5 | 20–21 | comms_task – 20 – core 0 (200 ms tick, wakes on commands) |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `subsystemN.h`; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 | S1 → S2, S5 | shared 16-bit word, one writer, one load (owned by S1) | `subsystem1_line_read()` | bits 0–6 error mm + 40 (+ = steer left), 7 valid, 8 LINE_LOST, 9 JUNCTION | Week 6, Tribunal 1 |
| IF-02 | S2 → S1, S3, S4, S5 | shared 32-bit word, one writer, one load (owned by S2) | `subsystem2_odometry_read()` | bits 0–14 distance mm (wraps, use `subsystem2_distance_delta`), 15–21 / 22–28 wheel speeds (raw − 64) × 5 mm/s | Week 6, Tribunal 1 |
| IF-03 | all → S4 | event flag (owned by S4) | `flg_events` via `subsystem4_event_set()` | bits 0–9: BARCODE, LINE_LOST, LINE_FOUND, JUNCTION, MOTION_DONE, HUMP, OBSTACLE, AVOID_REQ, REMOTE_CMD, FAULT | Week 6 |
| IF-03b | S1 → S4 | message buffer, 1 byte (owned by S1) | `subsystem1_barcode_mbf()` | bits 0–5 Code 39 index, bit 6 reversed | Tribunal 1 |
| IF-03c | S2, S3, S4, S5 → S4 | event flag, fault bitmask (owned by S4) | `flg_faults` via `subsystem4_fault_raise/clear()` | bit 0 STALL (S2), 1 ENCODER (S2), 2 IMU (S3), 3 NO_ECHO (S4), 4 WIFI (S5), 5 LIFTED (S3; was COLLISION until 17 Nov). Only the owner sets AND clears its bit; TSK_CTRL only reads | Tribunal 1 Version C |
| IF-04 | S4 (TSK_CTRL only) → S2 | message buffer, 2 bytes (owned by S2) | `subsystem2_command()`; `subsystem2_move_forward/backward()`, `_turn_left/right()`, `_stop()` | bits 0–2 opcode (STOP, FOLLOW, MOVE_FWD, MOVE_BACK, TURN_LEFT, TURN_RIGHT, SPEED, RESET), bits 3–10 argument (cm, degrees, cm/s) | Week 6, Tribunal 1 |
| IF-05 | TSK_SCAN ↔ TSK_CTRL (inside S4) | `mbf_avoid_req` (3 B) + EVT_OBSTACLE / EVT_AVOID_REQ; answers in `mbf_avoid_ack` (1 B) | private to S4 (`scan_task.h`) | Buddy 5 engine asks for control (TAKE), motion (S2 command, 11 bits), DONE_OK / DONE_FAIL; TSK_CTRL stays the only motion commander | Integration 26 Oct |
| IF-06 | all → S5 | message buffer (owned by S5) | `subsystem5_post()` | event records: barcode 1 B, hump 2 B, obstacle 5 B, state 1 B; periodic data is read by S5 from IF-01/02 and the S3 status word | Week 6, Tribunal 1 |
| IF-07 | S5 ↔ kernel WiFi service | message buffers `mbf_mqtt_tx` / `mbf_mqtt_rx`, `comms_bridge.h` | `lib_patches/IF-07_mqtt_utk.patch` (`mqtt_utk.c` + WiFi signal strength in `cyw43_utk.c`) | binary payloads per topic (`subsystem5/MQTT_PROTOCOL.md`), 1-byte commands on `robot/command`, LWT `robot/status`; decoder `tools/is25_dashboard.py` | Needs instructor approval |
| IF-08 | S5 → S4 | message buffer, 1 byte (owned by S4) | `subsystem4_remote_cmd_mbf()` + EVT_REMOTE_CMD | validated command: bits 0–2 opcode, 3–7 speed | Week 6 |
| IF-09 | S3 → S4, S5 | shared 32-bit word, one writer (owned by S3) | `subsystem3_status_read()` | pitch 8 bits (0.25°), heading 9, hump state 3, motion class 3 (0 stationary, 1 accelerating, 2 turning, 3 climbing, 4 descending, 5 impact, 6 decelerating, 7 cruising; 6–7 added 26 Oct), hump active 1 | Tribunal 1, integration 26 Oct |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. The numbers below are the example's usage.

| Object | Limit | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|
| Task | 32 | 1 | 1 | 1 | 2 | 1 |
| Semaphore | 16 | | | | | |
| Event flag | 16 | | | | 2 (`flg_events`, `flg_faults`) | |
| Mailbox | 12 | | | | | |
| Mutex | 12 | | | | | |
| Message buffer | 12 | 1 | 1 | | 3 | 3 |
| Fixed memory pool | 12 | | | | | |
| Variable memory pool | 12 | | | | | |
| Cyclic handler | 12 | 1 | 1 | 1 | | |
| Alarm handler | 12 | | | | | |

The two mutexes of Week 6 (`mtx_line`, `mtx_odom`) are gone: IF-01 and
IF-02 are single-writer words stored and loaded in one access (Change
Requirement 1).  The kernel I2C driver also creates one event flag.

(The kernel's USB and WiFi service tasks also use a few of these.)
