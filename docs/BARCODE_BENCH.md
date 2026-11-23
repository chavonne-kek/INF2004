# Barcode bench (make CHECK=6): barcode sensor + bare Pico, swiped by hand

Image: `mtk3pico_smp1_usb_cdc_wifi_test_check6.uf2` (`IS25_BARCODE_BENCH.uf2`).
Same decoder, filters and sampling as the car; no motors, no other sensors.

## Wiring
Barcode sensor (TCRT5000 module): A0 -> GP26, GP27 or GP28 (the bench uses
whichever moves most during calibration). Maker Pi Pico: Grove 6 (GP26/GP27).
Robo Pico: Grove 5/6/7. VCC -> 3V3, GND -> GND.
Hold it 2-3 mm above the paper, pointing straight down.

## Use
1. Flash, open the USB console (115200).
2. Self-tests print `subsystem1 SUMMARY 12 passed`.
3. Step 1, 8 s: slide the sensor back and forth over the label (bars AND white
   paper). `Calibration OK: span ...` = black now means "as dark as these bars".
4. Step 2: swipe over the WHOLE label, from white paper before the first bar to
   white paper after the last, either direction, any speed.
   - good: `S1: barcode 'A'` (+ `(reversed)` the other way)
   - bad: `S1: BC reject r=.. n=.. min=..`

| r | Meaning | Fix |
|---|---|---|
| 1 | more than 29 bars and gaps | noise or a smudge; start on clean white |
| 2 | wide/narrow unclear | blur: lower the sensor, keep it straight down |
| 3 | 29 elements but not Code 39 | not Code 39, or a bar merged and split elsewhere |
| 4 | black for 15 s | sensor resting on black |
| 5 | stopped before 29 | swipe the whole label; keep the sensor over the bars |

A status line every 3 s shows the sensor value and its range (white ~200-500,
bars should reach ~2500+), and how many reads/rejects since the last line.

## What it handles (host test H-BC14)
Both demo label sizes from Barcode_Sample.pdf (3.3 mm narrow / 165 mm and
6.9 mm narrow / 328 mm), 0.7 to 20 cm/s, the hand speeding up or slowing down
2x across the label plus +-40 % wobble, both directions: 400/400 read, 0 wrong.
