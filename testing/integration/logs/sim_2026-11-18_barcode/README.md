# Barcode, 18 Nov 03:00–04:00 (commit 4b33f2b)

Simulator (`SMP=0 WIFI=none`), harness with the car's servo trim.

| Log | Scenario | Result |
|---|---|---|
| `bcA.log` | `*A*` then a left branch | `S1: barcode 'A' (travel widths, arm 3..995 of 1000)`, TURN_EXECUTE onto the branch (the 40 cm branch then ends: FAULT at the end of the simulated track, as on 17 Nov) |
| `bcD_m85.log` | `*D*`, left motor 15 % weak | `'D'` read on travel widths, keeps following |
| `obs_c0_w16.log` | 16 cm box on the line | back on the line 78.8 s |
| `test_image.log` | TEST image | 46/46 |
| `before_bcA.log` | `*A*` on the 18 Nov 02:50 image | `'A'` read (time widths) – no regression |

The simulator's steering is steady, so it reads on time widths too; the
weaving case is in the host test H-BC16 (`testing/subsystem1/host`):
arm speed +-60 % every 30–50 mm, time widths 0/100, travel widths 100/100.
