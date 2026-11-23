# Buddy 5 code: how it was integrated (26 Oct)

Buddy 5's scanning, profiling, planning, bypass and recovery code
(`b5_*.c` / `b5_*.h`) is now subsystem 4's obstacle engine. It replaces
the skeleton's simpler scan code (`drv_hcsr04.c`, `drv_servo.c`, the
profile builder in `scan_calc.c`, and the fixed bypass in `ctrl_task.c`).
His algorithms are kept as written. Every change is marked `IS25` in the
code, and `BUDDY5_CHANGES.diff` shows them all line by line.

His own host tests still pass (`testing/subsystem4/buddy5_host`, `make
run`). The integer maths that replaced libm gives exactly the same output
as his float version on his test scenes, and matches `cosf`/`sinf` to
within 1 cm on all 45,250 angle/distance pairs.

## Unchanged

`b5_plan.c`, `b5_manoeuvre.c`, `b5_scan.c` and their headers, plus
`b5_types.h`, `b5_profile.h`, `b5_servo.h` and `b5_task.h`.

## What had to change, and why

| File | Change | Why |
|---|---|---|
| `b5_config.h` | Trigger/echo pins GP2/GP3 → **GP4/GP5** | GP2/GP3 are the wheel encoders (TEAM.md). GP4/GP5 is the team's HC-SR04 pin pair (Grove 3). |
| `b5_config.h` | Fine step 4° → **5°**, margin 28° → 30° | Statechart 6.1 and Tribunal 1 (Version C row 5) fixed a 5° grid with a 5-bit scan index. |
| `b5_config.h` | Sonar timeout 30 → **40 ms** | The HC-SR04 out-of-range pulse is about 38 ms. A 30 ms timeout counted open space as "no echo". |
| `b5_config.h`, `b5_sonar.c` | New **60 ms minimum** from one trigger to the next (`B5_SONAR_MIN_CYCLE_MS`) | HC-SR04 datasheet: "use over 60 ms measurement cycle"; Week 6 NFR7 and Tribunal 1 adopted it. Before, pings were echo time + 15 ms apart (about 17 ms for an obstacle at 30 cm), so a late echo could be read as the next one. Fine scans take a little longer. |
| `b5_port.h` | Pico SDK calls → micro T-Kernel (`tk_dly_tsk`, DI/EI, TIMERAWL register) | The team firmware is built with the template makefile, not CMake. It does not link the Pico SDK hardware libraries, so `pico/stdlib.h` is not available. |
| `b5_sonar.c` | `gpio_*` → libbsp; echo edges come from **subsystem 2's shared GPIO interrupt** (`subsystem2_gpio_attach`) | The RP2040 has one GPIO interrupt per core, and S2 owns it (Week 6 shared-resource matrix). This is the dispatcher his README asks for. |
| `b5_sonar.c` | `B5_FAULT_NO_ECHO` also sets/clears **FAULT_NO_ECHO** in the team's `flg_faults` | His README: "map B5_FAULT_* into the team-wide fault mask". |
| `b5_sonar.c` | A far reading (≥ 250 cm) no longer counts as a missing echo | Only a missing pulse means the sensor is faulty. |
| `b5_sonar.c` | New `B5_Sonar_LastCm()` | TSK_CTRL needs the range for the hump rule (ignore obstacles ≥ 10 cm while on a hump). |
| `b5_servo.c` | Pico SDK PWM → libbsp + a direct DIV register write (1 MHz) | Same reason as `b5_port.h`. Same pulse maths and settle times. |
| `b5_profile.c` | `cosf`/`sinf`/`sqrtf`/`lroundf` → cosine table + integer maths | The firmware does not link libm, so the build fails at link time without this. |
| `b5_profile.c`, `b5_types.c`, `b5_task.c` | `memset` → zero assignment | `<string.h>` clashes with the kernel's own `size_t` typedef. |
| `b5_if.h`, `b5_task.c` | `lineFollowEnable(bool)` → **`takeControl()` / `releaseControl(success)`** | See below. |

## The one design change: who drives the motors

In the team design, TSK_CTRL is the only task that commands motion. This
was decided at Week 6 and defended at Tribunal 1, and statechart 6.1 says
TSK_CTRL sends STOP and the motion commands. Buddy 5's engine calls
`moveForward()`, `turnLeft()` and so on directly. So the engine runs
unchanged inside **TSK_SCAN** (core 0, priority 19, the owner of the sonar
and servo), and its motion calls are implemented in `scan_task.c` as
**requests to TSK_CTRL**:

1. The engine sees an obstacle (≤ 40 cm, twice) and calls `takeControl()`.
   TSK_CTRL brakes at once (the NFR4 stop path) and enters OBSTACLE_AVOID.
   It refuses if the robot is not following the line, or if it is on a
   hump and the obstacle is 10 cm or more away. When refused, the engine
   just keeps watching.
2. Each `moveForward` / `turnLeft` / … goes to TSK_CTRL. TSK_CTRL sends it
   to subsystem 2 and answers when MOTION_DONE arrives. A stall, a lift,
   pause or e-stop answers "failed", so the engine aborts exactly as Buddy
   5 designed.
3. `releaseControl(true)` resumes line following. `releaseControl(false)`
   (his FAULT state) puts the system in FAULT, per statechart 6.1 "FAILED
   → system FAULT". The robot then waits for the reset button instead of
   driving on.

## Things for Buddy 5 to check

- **Trigger at 40 cm:** the Week 6 design said 20 cm. 40 cm gives the
  planner room to work, but update the report or change `B5_TRIGGER_CM`.
- **Wire format:** his 7-byte profile is kept in RAM inside TSK_SCAN.
  Telemetry uses the team's 5-byte, 35-bit record (`subsystem4.h`), which
  Chavonne's dashboard decodes. The converter is `io_publish_profile()` in
  `scan_task.c`.
- **Values marked VERIFY** in `b5_config.h`: servo pulse range, servo
  direction (0° = right), car width and length.
- **Bit-fields:** `B5_Profile_t`, `B5_Sample_t` and `B5_Plan_t` stay
  inside TSK_SCAN, which the team rule allows. The shared status word
  already uses masks and shifts.
- On-target tests: S4-U06 (engine idle and watching) and S4-U07 (geometry).
