# Line recovery searches both ways (18 Nov 04:30)

`lineend.log` / `.trace` (`{"pressStartAt":9,"lineEndX":900,"traceMs":1000}`):
the line ends at x = 900 mm. LINE_RECOVERY 37.3 s, turns toward the
remembered side to -94 deg in 5 s, "searching the other way" at 42.4 s,
turns back through the start heading to +89 deg, FAULT at 52.4 s (15 s).
Before: FAULT after 5 s of turning one way only.

`bcA.log`: barcode A read, TURN_EXECUTE onto the branch, follows it; the
40 cm branch ends, the car searches both ways, FAULT (end of the simulated
track). `test_image.log`: 46/46. `*_10s_version.log` (an intermediate 10 s
build, same first 5 s): left and right 90 deg corners followed, 16 cm
obstacle bypassed and rejoined.
