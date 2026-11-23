> **Historical (26 Oct): the rewiring below is done.** One correction: step 1
> says yellow→GP16 (SDA), white→GP17 (SCL). With this kit's cables, white is
> pin 1 = GP16 = **SDA** and yellow = GP17 = **SCL**, as in
> `CONNECT_IMU_AND_LEFT_ENCODER.md`. The encoders are now on Grove 1 (left,
> GP0/GP1) and Grove 2 (right, GP2/GP3), either signal wire.

# Reconnection list — match the car to IS25_final firmware

Do these **in this order** so nothing is plugged into a pin another device needs.
Power off and unplug USB before each change.

| # | Device | Take the wire OUT of | Put it INTO | Why |
|---|---|---|---|---|
| 1 | IMU (LSM303D module; this doc said GY-511 until 17 Nov) | Grove 3 (GP4/GP5) | **Grove 4** — yellow→GP16 (SDA), white→GP17 (SCL) | firmware uses I2C0 on GP16/17 |
| 2 | Ultrasonic | Grove 5 (GP6/GP26) | **Grove 3** — Trig→GP4, Echo→GP5 | frees GP6/GP26, matches `b5_config.h` |
| 3 | IR left | GP7 (DO) | **GP26 (AO)** | line sensing is analog, ADC0 |
| 4 | IR centre | GP0 (DO) | **GP27 (AO)** | ADC1 |
| 5 | IR right / barcode | GP16 (DO) | **GP28 (AO)** | ADC2 — GP16 already taken by the IMU |
| 6 | Servo signal | GP12 | **GP15** | firmware drives GP15 |
| 7 | Encoders | GP2 / GP3 | *unchanged* | already correct |

## Notes

- **Analog, not digital.** Each IR module now uses its **AO** wire, not **DO**.
  Leave DO disconnected and taped. The firmware reads brightness, so the
  potentiometer setting no longer decides pass/fail — but keep it mid-travel.
- **ADC pins max 3.3 V.** Power the IR modules from 3V3 so AO never exceeds it.
- **GP26 is shared between Grove 5 and Grove 6**, so the cleanest route for the
  three AO wires is the **GPIO breakout header** (pins labelled GP26 / GP27 /
  GP28) with a 3V3 and GND rail, rather than trying to fit them into ports.
- **Ultrasonic echo:** if the module is powered from **3V3**, no divider is
  needed (echo swings 3.3 V). If powered from 5 V, the 1 kΩ + 2 kΩ divider in
  `b5_config.h` is mandatory.
- **Pins freed after this:** GP0, GP7, GP12. **Pins now in use for the IMU:**
  GP16/GP17 — do not put anything else there.
- Update the tape labels: IR-left = GP26, IR-centre = GP27, IR-right = GP28,
  IMU = GP16/17 Grove 4, sonar = GP4/GP5 Grove 3, servo = GP15.

## After rewiring

1. Flash `mtk3pico_smp1_usb_cdc_wifi_test.uf2` (car does not drive — safe on the
   bench).
2. Read the USB console.
3. Only when the test image reports clean, flash
   `mtk3pico_smp1_usb_cdc_wifi.uf2` and drive it.
