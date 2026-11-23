# Simulation, 17 Nov night (servo 2200 us / 1850..2500 us, gain 500, self-echo filter, TAKE resync)

Single-core simulator images (`SMP=0 WIFI=none`, and `TEST=1` for the
self-tests) built from this commit; `tools/sim/harness.mjs` now maps the
servo pulse from 2200 us = ahead and reports any pulse outside 1850..2500 us.

| Log | Scenario (JSON) | Result |
|---|---|---|
| `test_image.log` | `{}` (TEST=1 image) | 46/46 self-tests (S1 12, S2 9, S3 14, S4 7, S5 4) |
| `barcode_turn.log` | `{"pressStartAt":9,"barcode":"A","branchLeftAtX":800}` | `A` read at 22.0 s, TURN_EXECUTE 33.9 s, follows the branch |
| `obstacle_16cm_150s.log` | `{"pressStartAt":9,"boxX":700,"boxW":160,"boxD":60,"boxY":30,"traceMs":1000}` | OBSTACLE_AVOID 21.6 s, edge beyond view on the left, goes round on the right (25 cm), back on the line 90.1 s, then follows to x = 3.3 m |

Also run (90 s): 10 cm box on the line (`{"pressStartAt":9,"boxX":700}`):
goes round on the left (18 cm), back on the line at 81.5 s.
Servo pulse in every run: 1867..2477 us, never outside 1850..2500 us.
