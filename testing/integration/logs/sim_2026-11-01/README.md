# Simulation logs, 1 Nov (firmware-in-the-loop, rp2040js)

Same firmware as the car (SMP=0 build for the single-core simulator).
Track as in the project specification: 18 mm line, Code 39 `*X*` barcode
(3 mm narrow, 9 mm wide, 141 mm long) 20 mm LEFT of the line from
x = 300 mm, a junction at x = 700 mm. Sensors as on the car: arm (GP28)
40 mm left, centre (GP27) on the line's left edge, right (GP26) 22 mm right;
1270 encoder edges per wheel turn; GP20 pressed at 8 s.

| Log | Barcode | Junction | Result |
|---|---|---|---|
| before_A_left_branch.log | A | left branch | 1 Nov source as received: barcode never decoded, false JUNCTIONs at the bars, line lost beside the barcode, 5 cm/s instead of 12 |
| fixed_A_left_branch.log | A | left branch | read 'A', turned left onto the branch, follows it at 12 cm/s |
| fixed_B_right_branch.log | B | right branch | read 'B', turned right onto the branch |
| fixed_C_crossing.log | C | crossing | read 'C', drove straight across |
| fixed_D_dead_end.log | D | line ends | read 'D', U-turn at the end, follows back |

Also run (not kept): A and B at 8 and 20 cm/s and with a 5 mm sensor spot,
a junction 40 mm after the barcode, and junctions with no barcode (driven
straight through): all as expected.

Start-up sweep (60 deg left, 120 right, 60 left):
- before (1 Nov source as received): heading after the start-up sweep -14 deg (0 = straight along the line)
- after (this version): heading after the start-up sweep 1 deg (0 = straight along the line)
