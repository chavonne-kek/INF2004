# Car tests, 17 Nov 2026: IMU (LSM303D) on Grove 4, GP16/GP17

Console logs as received, in order. Each run was started with the USB
unplugged first, so the module was powered off in between.

| Log | Image | Result |
|---|---|---|
| `1_check3_kernel_i2c_driver.txt` | CHECK_3, kernel "iica" driver | `IMU not found (-50)`: the chip acknowledged the bit-banged probe, then the kernel driver's read timed out. The GP20/GP21 "devices" are the START/RESET buttons (sweep bug, fixed). |
| `2_check3_kernel_i2c_driver_sweep_fixed.txt` | same, sweep without GP20/GP21 | same `-50` |
| `3_sdk_test_teammate_driver.txt` | `IS25_IMU_SDK_test_teammate_driver.uf2` (Pico SDK, same pins) | LSM303D at 0x1D, WHO_AM_I 0x49, X +0.04 g, Y 0.00 g, Z -0.95 g: module and wiring good, chip faces down |
| `4_check3_polled_i2c.txt` | CHECK_3, polled SDK-style I2C | still `-50` (now a 5 ms limit, S3-U06 passes) |
| `5_check3_polled_i2c_noWiFi_diagnostics.txt` | CHECK_3 without Wi-Fi, failure report | every transfer: STATUS 0x02, TXFLR 1, lines high: the controller never started (it had seen the bit-banged START and took the bus for busy) |
| `6_check3_polled_i2c_reset_after_bitbang_PASS.txt` | CHECK_3, I2C0 reset whenever the bit-banged checks hand the pins back | **LSM303D at 0x1D, S3 14 passed / 0 failed**, about 5,000 reads with 1 failure, level car X +90, Y +60, Z -900 mg |

Still to do on the car: S3-M07..M09 (lift, overturn), the pitch sign (lift
the nose: pitch must go positive), compass calibration (one slow full circle).
In log 6 the bit-banged address scan in the check image prints "found
NOTHING" although the chip reads fine: that scan is a check-image message
only and does not affect the car.
