# Putting the IS25 code on the Robo Pico car

## 0. What you need

- The car: a Pico W on the Robo Pico, with motors, sensors and batteries
  wired as in `IS25_README.md`.
- A **data** micro-USB cable. A charge-only cable never shows the drive.
- On Windows: **WSL (Ubuntu)** to build, and **PuTTY** (or
  `python tools/read_serial.py`) to read the console.

**Fast path (no building):** the zip's `firmware/` folder has two images
built from exactly this code. You can go straight to step 3.

| File in `firmware/` (name when you build it yourself) | When to use it |
|---|---|
| `IS25_TEST_does-not-drive.uf2` (`mtk3pico_smp1_usb_cdc_wifi_test.uf2`) | **First.** Runs every subsystem's self-test. **The car does not drive.** |
| `IS25_NORMAL_drives.uf2` (`mtk3pico_smp1_usb_cdc_wifi.uf2`) | Normal run: calibrates (turns 60° left, 120° right, 60° left), then waits for the start button. |

You need steps 1–2 once you change code, for example to set the measured
encoder edges, tune the PID, or set the servo angles.

## 1. One-time setup (Windows)

1. Open **PowerShell as administrator**, run `wsl --install`, and restart.
   Open **Ubuntu** from the Start menu and create a user.
2. In Ubuntu:
   ```sh
   sudo apt update && sudo apt install -y git make g++ gcc-arm-none-eabi python3
   mkdir -p ~/pico && cd ~/pico
   git clone --branch 2.2.0 https://github.com/raspberrypi/pico-sdk.git
   cd pico-sdk && git submodule update --init lib/tinyusb lib/cyw43-driver lib/lwip
   echo 'export PICO_SDK_PATH="$HOME/pico/pico-sdk"' >> ~/.bashrc
   ```
   Close Ubuntu and open it again. Check that `arm-none-eabi-gcc --version`
   and `ls $PICO_SDK_PATH` both work.

## 2. Get the code and build

Either copy the repo out of the zip (building inside WSL's own disk is
much faster than building on `/mnt/c`):

```sh
cp -r /mnt/c/Users/<you>/Downloads/IS25_final/INF2004-main ~/pico/
cd ~/pico/INF2004-main
```

or start from the team repo and apply the patch:

```sh
cd ~/pico && git clone https://github.com/chavonne-kek/INF2004.git && cd INF2004
git apply --whitespace=nowarn /mnt/c/Users/<you>/Downloads/IS25_final/IS25_final.patch
```

Then build (keep the default `CRASH_REPORT=0` and USB console: the left
encoder uses GP0, which UART0 would take; such builds stop with an error):

```sh
cd build_make
make TEST=1 -j8      # -> mtk3pico_smp1_usb_cdc_wifi_test.uf2
make -j8             # -> mtk3pico_smp1_usb_cdc_wifi.uf2
cp *.uf2 /mnt/c/Users/<you>/Downloads/     # hand the images to Windows
```

## 3. Flash

1. **Lift the car so the wheels are off the ground.**
2. Hold the white **BOOTSEL** button on the **Pico W** itself.
3. While holding it, plug the USB cable into the Pico W and the computer,
   then let go.
4. A drive called **RPI-RP2** appears in File Explorer.
5. Drag the `.uf2` onto **RPI-RP2**. The drive disappears and the Pico
   restarts with the new firmware.

If RPI-RP2 does not appear, try another cable or port, and hold BOOTSEL
*before* the cable goes in. The Robo Pico's reset button does not enter
BOOTSEL mode.

## 4. Read the console

- `python tools/read_serial.py` (Windows Python, no extra packages) finds
  the Pico and saves everything to `testing/subsystem3/logs/`. Use
  `python tools/read_serial.py auto subsystem1` for your own folder.
- Or open **PuTTY**: Serial, the `COMx` shown in Device Manager → Ports,
  speed 115200.

Press the Robo Pico **reset** button to see the start-up from the
beginning.

## 5. First run: the test image

Every line should read `[TEST] … PASS`, and each subsystem ends with
`SUMMARY x passed, 0 failed`. A FAIL usually means a sensor is unplugged
or on the wrong pin.

- Point the ultrasonic sensor at something within 1 m (S4-U02).
- S3-U01/U02 need the LSM303D IMU on Grove 4 and the car level. S3-U06…U16 use a
  simulated sensor and take about 25 s (U12…U16: lift and overturn stop).
- When `[CAL] S2-M01` appears, **turn each wheel exactly 10 times by
  hand** within 15 s. The wheels coast meanwhile. Put the printed number
  into `MOTION_EDGES_PER_REV` (`app_program/subsystem2/motion_calc.h`)
  **before the first normal run**. The `[CAL] pin activity GP0 … GP3 …`
  and `[CAL] left wheel counted on GPx, right on GPy` lines show which
  pins really toggled. Either wire of each encoder's Grove port works; a
  wheel with no edges on both of its pins is not wired or not powered. If
  it prints `[CAL] NO encoder pulse on GP0..GP3 at all`, turn a wheel
  slowly and watch the small status LEDs next to Grove 1 and Grove 2: no
  blink means that encoder has no power or is not plugged in.
- If the IMU does not answer, the first lines say why
  (`S3: I2C check: …`). `SDA (GP16) LOW, SCL (GP17) LOW … a line is held
  low` usually means the module has no power while its SDA/SCL wires are
  connected: check its VCC (red, 3V3) and GND (black).
- The test run also prints the worst task times
  (`TSK_LINE worst sample … us`, `TSK_MOTION worst cycle … us`). Keep the
  log as evidence.

### Check one subsystem at a time

If the test image shows problems, use the check images (zip folder
`firmware/checks/`, or `make CHECK=1` … `make CHECK=5`). Each runs ONE
subsystem's self-tests, then prints its live readings twice a second
(IR values, encoder pins and a motor test, IMU data and a wiring check,
ultrasonic range with a servo sweep and the buttons, the comms status
line). Steps and expected output: `docs/IS25_Subsystem_Checks.txt`.

In the test and check images the car never drives by itself: START
(GP20 or the dashboard) only beeps and prints `S4: START pressed`.

## 6. Normal run

1. Flash the normal image (`IS25_NORMAL_drives.uf2`, or
   `mtk3pico_smp1_usb_cdc_wifi.uf2` if you built it). It starts at once
   and tries its calibration spin while the car is still lifted and
   perhaps without battery power, so ignore that first run.
2. Put the car **on the line** on the track, switch the **battery on**,
   and press the board's **RUN/RESET** button. (USB alone powers the Pico
   but not the motors: a spin without the battery ends in
   `S2: STALL` → FAULT, as on 27 Oct.)
3. It **beeps, stays still for about 3 s, then sweeps 60° left, 120° right
   and 60° left** (ending where it started) to calibrate
   the IR sensors and the magnetometer. Keep hands clear. The console then
   prints `S4: calibration IR ok (…), IMU ok` and `S4: state IDLE`.
4. Press **GP20 (start)**. It follows the line, executes barcodes A/B/C/D
   at the next junction, measures humps, and avoids obstacles. **GP21
   (reset)** recovers from FAULT or EMERGENCY_STOP.
5. Every 2 s the console prints the decoded state, for example:
   `S5: up 12 s | LINE_FOLLOW (none) | link ONLINE, signal -48 dBm | line -3 mm, IR 812 3012 790 | odo 1820 mm, L 205 R 195 mm/s | pitch 0.25 deg, hump FLAT, CRUISING | barcodes 1 (rejected 0) | dropped 0, bad commands 0`.
   The words in brackets after the state are the active faults. `IR` is
   the raw reading (0–4095) of the left, centre and right IR sensor.
6. If something is the wrong way round or off, fix the matching `TODO`
   constant (see each subsystem's `README.md`), rebuild (step 2) and
   reflash (step 3).

### If the car goes to FAULT at start-up

The console says why. The three messages seen on 27 Oct:

| Console | Meaning | Check |
|---|---|---|
| `S2: STALL - motors driven … no encoder pulse from either wheel` | During the start-up spin the motors were driven but no encoder counted | Battery switch on? Did the wheels turn? If yes: encoder 3V3/GND/signal wires (test image `[CAL] pin activity`) |
| `S3: IMU not found …` and `S3: I2C check: …` | The IMU (LSM303D) does not answer | The check line says what to fix (swapped wires, a line held low, or no answer: power and cable) |
| `S4: calibration IR FAILED (black-white span …)` | The spin did not carry every IR sensor over the line and the floor | Put the car on the line; spans under 400 mean that sensor saw no contrast |

GP21 clears a STALL (back to IDLE, without a new calibration); the board's
RUN/RESET button restarts everything, including the calibration spin.

## 7. WiFi, MQTT and the dashboard (after instructor approval)

```sh
cd ~/pico/INF2004-main
cp config/wifi_secrets.example.h config/wifi_secrets.h   # SSID, password, broker IP
git apply lib_patches/IF-07_mqtt_utk.patch
cd build_make && make NET=lwip MQTT=1 -j8   # -> ..._wifi_lwip_mqtt.uf2, flash it
```

Without the patch the MQTT build stops with a message. (An unpatched
MQTT image only runs the kernel's `mqtt: TX [mtk3smp-pico/heartbeat] "hb N"`
demo, and the robot's own link stays OFFLINE with the WIFI fault.)

On the laptop (same WiFi), run a broker (e.g. Mosquitto) and the
dashboard:

```sh
pip install paho-mqtt
python tools/is25_dashboard.py --broker <laptop IP> --log run1.csv
```

Type `start`, `pause`, `resume`, `estop`, `reset`, `speed 15`, `bad` or
`quit`. `app_program/subsystem5/MQTT_PROTOCOL.md` has every topic and
byte.

## 8. Push the code to the team repo (GitHub)

Do this from the folder you built in. `wifi_secrets.h`, `.uf2` files and
build outputs are in `.gitignore`, so they are never committed.

```sh
cd ~/pico/INF2004                          # the clone from step 2
git checkout -b is25-final                 # work on a branch, not main
git add -A
git status                                 # check: no wifi_secrets.h, no .uf2
git commit -m "IS25 final code: all five subsystems integrated"
git push -u origin is25-final
```

Then open a pull request on GitHub and ask each owner to review their own
folder (TEAM.md: only the owner edits a subsystem's folder). If you started
from the zip copy instead of a clone, first run `git clone` (step 2), copy
the zip's `INF2004-main` contents over it, and then do the steps above.
The first push asks you to sign in to GitHub. Use a personal access token
or the GitHub CLI, not your account password.
