# Simulation after the obstacle trigger change (17 Nov, 19:10)

`B5_TRIGGER_CM` 40 -> 25, `B5_DETECT_RANGE_CM` 60 -> 45, single-core
`SMP=0 WIFI=none` builds of commit "Obstacle trigger 40 -> 25 cm".

| Log | Scenario JSON | Result |
|---|---|---|
| `test_image.log` | (none) | 46/46 PASS |
| `obstacle_10cm.log` | `{"pressStartAt":9,"boxX":700}` | OBSTACLE_AVOID 21.6 s (was 16.7 s at 40 cm), LINE_FOLLOW 80.2 s, ends on the line |
| `obstacle_16cm.log` | `{"pressStartAt":9,"boxX":700,"boxW":160,"boxD":60,"boxY":30}` | OBSTACLE_AVOID 21.6 s, LINE_FOLLOW 80.8 s, on the line |
| `obstacle_near.log` | `{"pressStartAt":9,"boxX":400}` | OBSTACLE_AVOID 11.8 s, LINE_FOLLOW 70.3 s, on the line |
| `noisy_drive.log` | `{"pressStartAt":9,"accNoiseMg":120,"noseUpFromS":20,"noseUpToS":26,"noseUpDeg":18}` | no false stop |
| `barcode_turn.log` | `{"pressStartAt":9,"barcode":"A","branchLeftAtX":800}` | `A`, TURN_EXECUTE, follows the branch |
