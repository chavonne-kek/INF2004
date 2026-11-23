# Barcode reader: noise filtering, 1 ms sampling, slower car (2 Nov, evening)

On top of IS25_final_02nov_pm.

## Why the barcode failed while driving but read when pushed slowly by hand

Not the sample rate: at 4 cm/s a 3.5 mm bar already lasted about 44
samples. The arm's black/white decision **flickered** for a few ms at
every bar edge while the motors ran (PWM noise on the long arm cable; the
arm sits higher, so its edges are soft and the reading spends longer near
the threshold). Each flicker was an extra element or a tiny "narrowest
element", which ended the read early: exactly the `r=5 ... min=5` the car
printed. By hand, motors off, there is no noise, so it read.

Reproduced in the simulator (300 counts of noise on the readings):
before 0 of 4 labels, after 4 of 4
(`testing/integration/logs/sim_2026-11-02/`).

## Changes

| What | Where | Before | Now |
|---|---|---|---|
| IR sampling | `subsystem1.h` `S1_SAMPLE_MS` | 2 ms (500/s) | **1 ms (1000/s)** |
| ADC reads per sample | `line_task.c` `LINE_ADC_AVERAGE` | 1 | 2, averaged |
| Arm level for the decoder | `line_sense.c` `update_barcode_level()` | raw decision | **8 ms moving average**, own hysteresis (`b_bc_black`) |
| Decoder debounce | `barcode.c` `BARCODE_DEBOUNCE` | none | a colour change must hold **4 ms** |
| Decoder counters | `barcode.h` | 8-bit (a bar over 0.51 s = "the line") | 16-bit; black limit **1.5 s** (needed below ~2 cm/s with 10.5 mm bars, and with 1 ms samples) |
| Decoder thresholds | `barcode.c` | in samples | in ms (`BARCODE_MS()`), independent of the sample period |
| Line following speed | `ctrl_task.c` | 4 cm/s | **3 cm/s** |
| Recovery / approach | `ctrl_task.c` | 3 cm/s | **2 cm/s** |
| Dashboard `speed` | `subsystem4.h`, dashboard | 5..25 | **2..25** |

Filtering delays both edges of a bar by the same amount, so the widths
(and the wide/narrow ratio) do not change.

## Tests

- New host test **H-BC13**: the track label (3.5/10.5 mm), 5 mm spot,
  noise sigma 350, at 2/3/4/8/12 cm/s: 97-100 % read, 0 wrong. The
  previous decoder read 0 % at 2-8 cm/s in the same test.
- All host suites pass (S1 24, S2 13, S3 15, S4 3, S5 6; H-CO01 now counts
  60 valid one-byte commands because SPEED accepts 2..25).
- On-target self-tests 43/43 in the simulator; S1-U03 now checks
  1000 samples/s and S1-U16 still finds the worst sample under 0.5 ms.

## If it still misses on the car

Look at the `S1: BC reject` line: `n` near 29 with `r=2` means the bars
blur (lower the arm to 2-3 mm); `n` far below 29 means the arm is not
crossing all the bars (move it over the middle of the label).

## IR telemetry fix (2 Nov, 17:10)

The dashboard showed `IR arm - centre - right -` and a nonsense
`drop 160`: in `comms_codec.c` the topic names for COMMS and IR were in
the opposite order to `comms_topic_t`, so the 6-byte IR record went out
on `robot/telemetry/comms` (its first byte read as "drops") and the
3-byte comms record on `robot/telemetry/ir` (too short to decode). Also
`subsystem1_line_raw()` sent the levels in GPIO order (right, centre,
arm) instead of arm, centre, right. Both fixed; H-CO05 now checks the
three topic names.

## Hand-swipe and any-size barcodes (3 Nov)

- `BARCODE_END_RATIO` 5 -> 20: a read is decoded at its last bar, so this
  only resets a partial read; with 5, a hand that slowed down inside the label
  ended the read early.
- `BARCODE_WIDTH_MAX` 1.5 s -> 15 s: the large demo label (6.9 mm narrow,
  20.7 mm wide bars) at a slow hand speed.
- New host test H-BC14: both demo sizes, 0.7-20 cm/s, uneven hand speed,
  both directions: 400/400. The previous limits read 60-75 %.
- New image `make CHECK=6`: barcode bench (barcode sensor on GP28 and a bare
  Pico), barcode-only calibration over the label itself; see
  `docs/BARCODE_BENCH.md`.
