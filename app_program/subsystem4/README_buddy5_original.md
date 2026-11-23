# Buddy 5: Adaptive Servo Ultrasonic Scanning and Obstacle Profiling

Hardware: ultrasonic sensor (HC-SR04 style) on a servo. C, Pico C SDK, micro T-Kernel.

## How it works

```
IDLE  --front ping <= 40 cm x2-->  stop car, pause line follower
COARSE  5 angles (30/60/90/120/150)      -> no obstacle? hand back, IDLE
FINE    4-degree sweep around the hits, median-of-3 per angle
PLAN    profile -> CONTINUE | STOP | TURN_LEFT | TURN_RIGHT | REVERSE
REVERSE back 10 cm, retry (max 3)   BYPASS trapezoid round obstacle
RECOVER creep + sweep for the line, then re-enable Buddy 3's line follower
FAULT   stop, hold 1 s, flags stay visible in the status word, return to IDLE
```

| File | Brief deliverable |
|---|---|
| `b5_servo.c`, `b5_sonar.c`, `b5_scan.c` | Scanning subsystem (Stages 1-2) |
| `b5_profile.c` | Obstacle profile generator (Stage 3) |
| `b5_plan.c` | Avoidance algorithm (Stage 4) |
| `b5_manoeuvre.c` | Bypass + Recovery algorithm (Stage 5) |
| `b5_task.c` | State machine, shared status, telemetry hook |
| `b5_types.c` | Bit packing, race-free status word (Change Req. 1) |
| `b5_if.h` | The only coupling to Buddy 1/2/3 |

## Integration (what you must do)

1. **Fill `B5_Interfaces_t`** with wrappers around Buddy 2 (`moveForward`, `turnLeft`, ... must block until done and return `false` on failure), Buddy 3 (`lineDetected`, `lineFollowEnable`) and Buddy 1 (`publishProfile`, 7-byte binary payload).
2. **Create the task** (micro T-Kernel):
   ```c
   B5_Task_Init(&g_b5Interfaces);
   T_CTSK ctsk = { .tskatr = TA_HLNG | TA_RNG0, .task = (FP)B5TaskEntry,
                   .itskpri = 10, .stksz = 1024 };
   ID id = tk_cre_tsk(&ctsk);  tk_sta_tsk(id, 0);
   /* void B5TaskEntry(INT stacd, void *exinf) { B5_Task_Run(); } */
   ```
3. **GPIO interrupts:** the Pico SDK allows one GPIO callback. If the team already has a shared dispatcher (encoders, IR), call `B5_Sonar_OnGpioEdge(gpio, events)` from it. Otherwise set `B5_SONAR_OWN_IRQ_CALLBACK` to 1.
4. **Check `b5_config.h`:** pins, car width/length, servo pulse range, servo direction (0 deg = right).
5. HC-SR04 echo is 5 V: use a divider or level shifter into the Pico.
6. Telemetry: poll `B5_Task_GetStatus()` (16-bit word, decode with `B5_ST_GET`) and `B5_Task_GetProfile()`.

## Data representation table (Change Requirement 1)

| Field | Valid range | Bits | Implementation |
|---|---|---|---|
| Scan angle | 0..180 deg | 8 | `B5_Sample_t.angle : 8` |
| Scan distance | 1..250 cm, 255 = no echo | 8 | `B5_Sample_t.dist : 8` (sample = 2 B) |
| Profile `detected` | 0/1 | 1 | `B5_Profile_t.detected : 1` |
| Profile centre angle / closest angle | 0..180 | 8 each | bit-field |
| Profile closest distance / width | 0..255 cm | 8 each | bit-field |
| Left / right clearance | 0..255 cm (255 = open) | 8 each | bit-field (RAM 8 B; wire 7 B) |
| Planner action | 5 values | 3 | `B5_Plan_t.action : 3` |
| Planner lateral shift | 0..255 cm | 8 | `B5_Plan_t.shiftCm : 8` |
| Task state | 8 states | 3 | status word bits 0-2 |
| Retry count | 0..3 | 2 | status word bits 7-8 |
| Obstacle present | 0/1 | 1 | status word bit 6 |
| Faults (no echo, line lost, bypass abort, blocked) | independent | 4 (1 each) | status word bits 9-12, `B5_FAULT_*` masks |
| Echo pulse width (ISR to task) | 0..16383 us | 14 + 2 state | single 16-bit word `s_echoWord` |
| MQTT profile payload | 49 bits | 49 -> 7 B | `B5_Profile_Pack` (explicit LSB-first shifts) |

**Shared-data safety**
- Status word (many writers): every field update is a masked read-modify-write inside `B5_Port_Enter/Exit` (T-Kernel `DI/EI`). Reads are single aligned 16-bit loads.
- Sonar ISR to task: one 16-bit word, always written as a whole by exactly one side at a time (task arms, ISR completes, task re-idles), so no read-modify-write exists to race. ISR-only and task-only variables are kept in separate bytes on purpose.
- Latest profile (8 B copy): copied inside a critical section.

## Evidence for Week 13 (what to measure yourself)

- **RAM:** `arm-none-eabi-size` on the `.elf` before/after, or compare `.map` entries for `s_*` symbols.
- **Message size:** profile is **8 B in RAM and 7 B on the wire**. A naive version (seven `int` fields = 28 B; JSON text such as `{"detected":1,"centre":90,...}` = about 96 B) is the "before" baseline. Build it yourself on a branch for the real numbers.
- **Timing:** `time_us_64()` around `B5_Scan_Coarse`, `B5_Scan_Fine`, `B5_Profile_Build`, `B5_Plan_Decide`, and `B5_Profile_Pack`. Expect scans to be dominated by servo settle and ping time, not by the bit packing.
- Host tests (`test/run_host_tests.sh`) show packing round-trips and that planner output is unchanged.

## Known limits and tuning

- Profile values are estimates from a single ultrasonic fan; width and clearance are approximate (cone of about 15 deg). Tune `B5_FINE_STEP_DEG`, `B5_EDGE_JUMP_CM`, `B5_SIDE_SEEN_CM` on the real track.
- During the pass leg the servo looks sideways, so there is no forward guard for that stretch.
- `B5_FAULT_*` bits must be mapped into the team-wide shared fault mask.
- Bit-fields are used for RAM structs (explicitly allowed by the change request). If your Barr-C checker or the template forbids them, replace with the shift/mask style already used in `b5_types.c`.
