# Barcode reading with motor noise (simulation, 2 Nov)

Firmware-in-the-loop (rp2040js, SMP=0 build). Track: 18 mm line, four
Code 39 `*A*` labels as printed for the track (3.5 mm narrow, 10.5 mm
wide) 20 mm left of the line, 300 mm apart. Arm sensor 62 mm left of the
centre sensor with a 5 mm spot (mounted higher than the line sensors);
Gaussian noise of 300 ADC counts on every IR reading (motor PWM noise);
left motor 10 % weak.

| Log | Firmware | Speed | Barcodes read | Rejects |
|---|---|---|---|---|
| barcode_noise_base.log | IS25_final_02nov_pm as received | 4 cm/s | 0 of 4 | 13, all `r=5`/`r=2`, `min` 5-11 samples (flicker) |
| barcode_noise_new.log | this version | 3 cm/s | 4 of 4 | none on the labels |

The "before" log shows the same signature as the car: `r=5 ... min=5`, a
few-sample flicker taken as the narrowest element.
