# Obstacle bypass, simulator batch, 18 Nov 02:00–03:00

Single-core simulator images (`SMP=0 WIFI=none`); `tools/sim/harness.mjs`
with the car's servo trim (2200 us ahead, 1850..2500 us). Each run 150 s
(set 2: 170 s), start pressed at 9 s, `traceMs` 1000.

## Before (commit 167d674): 8 / 10 rejoined

`summary_before.txt`. The two failures (`before/`):

- `c0_w16` (16 cm box on the line): crossed the line on the way back
  (LINE_FOUND 73.1 s, LINE_LOST 75.1 s) between two 5 cm steps, so the
  step-end check missed it; drove on to 21 cm past the line, the sweep could
  not reach it: FAULT (LINE_LOST).
- `yR3_w10` (10 cm box 3 cm right of the line): the "tail clears" move was
  one 40 cm move; at 3 cm/s it outlasted TSK_CTRL's 9 s per-move limit,
  which stopped it and failed the bypass: FAULT (BYPASS_ABORT).

## After: 18 / 18 rejoined, no contact with the box

`summary_after1.txt` (same 10 scenarios), `summary_after2.txt` (8 harder
ones: 20 and 30 cm deep boxes, a 20 x 20 cm box, boxes 6 cm off the line,
10 % / 15 % weaker left motor, an obstacle further down the line).
`collision_after.txt` (`collision_check.py`, car body 16 x 15 cm around the
axle against the box, every 1 s): closest gap 64 mm, never inside.

Fixes: S1 keeps a debounced "line seen" count and the bypass checks "seen
at any time during the last move"; the tail-clear move is driven in 5 cm
guarded steps; the line follower is told which side the line is on when
the bypass hands back.

## Final image (commit 769f75e, also re-arms after "carry on")

`final/`: 16 cm box and 30 cm deep box rejoined (86.1 s, 88.2 s);
barcode `A` read and turned onto the branch (identical states and times to
17 Nov night: the branch is a 40 cm dead end, so FAULT at 55.8 s is the end
of the simulated track); TEST image 46/46.
