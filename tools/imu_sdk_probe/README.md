# IMU A/B test with the teammate's Pico SDK driver

`lsm303d.c` / `lsm303d.h` are the teammate's LSM303D driver from
`IMU_Subsystem_Integration.zip`, unchanged. `main.c` runs it on the car's IMU
port (I2C0, SDA GP16 / SCL GP17, 100 kHz, internal pull-ups, exactly as her
`imu_subsystem_init()` but with our pins) and prints every 2 s over USB:
line levels, a scan of 0x08-0x77, `lsm303d_init()` and one accelerometer
reading.

Build (Pico SDK 2.x, outside the micro T-Kernel tree):

    mkdir build && cd build
    PICO_SDK_PATH=~/tc/pico-sdk cmake -G Ninja .. && ninja

Result on the car decides where the fault is: chip found here but not by the
IS25 firmware = our firmware; not found here either = module, cable or power.
