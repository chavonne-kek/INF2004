/* IS25 IMU A/B test: the teammate's LSM303D driver (lsm303d.c, unchanged,
 * Pico SDK hardware I2C, 100 kHz, internal pull-ups) run on OUR car's IMU
 * port, Grove 4 = I2C0 SDA GP16 / SCL GP17.  Prints over USB every 2 s.
 * If this finds the chip, our firmware's I2C path is at fault; if it does
 * not, the module / cable / power is. */
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "lsm303d.h"

#define SDA_PIN 16
#define SCL_PIN 17

static void lines(void)
{
    /* Bus idle levels with the internal pull-ups on (I2C function). */
    printf("  lines: SDA GP%d %s, SCL GP%d %s\n", SDA_PIN,
           gpio_get(SDA_PIN) ? "high" : "LOW", SCL_PIN,
           gpio_get(SCL_PIN) ? "high" : "LOW");
}

int main(void)
{
    stdio_init_all();
    sleep_ms(3000);                         /* time to open the console */

    /* Same set-up as imu_subsystem_init(), only the pins changed. */
    i2c_init(i2c0, 100000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    for (unsigned run = 1; ; run++) {
        lsm303d_t dev;
        lsm303d_accel_t a;
        int found = 0;

        printf("\n=== IMU SDK test %u (teammate's driver, I2C0 GP%d/GP%d, 100 kHz) ===\n",
               run, SDA_PIN, SCL_PIN);
        lines();

        for (int addr = 0x08; addr <= 0x77; addr++) {
            uint8_t b;
            if (i2c_read_timeout_us(i2c0, (uint8_t)addr, &b, 1, false, 2000) >= 0) {
                printf("  scan: device at 0x%02X%s\n", addr,
                       (addr == 0x1D || addr == 0x1E) ? "  (LSM303D address)" :
                       (addr == 0x19) ? "  (LSM303DLHC accel)" : "");
                found++;
            }
        }
        if (!found) printf("  scan: NOTHING answered (0x08-0x77)\n");

        if (lsm303d_init(&dev, i2c0)) {
            printf("  lsm303d_init: OK, LSM303D at 0x%02X (WHO_AM_I = 0x49)\n", dev.address);
            if (lsm303d_read_accel(&dev, &a))
                printf("  accel: X %+.3f g  Y %+.3f g  Z %+.3f g  (level: Z near +1 or -1)\n",
                       a.x_g, a.y_g, a.z_g);
            else
                printf("  accel read FAILED\n");
        } else {
            uint8_t reg = 0x0F, id = 0;
            for (int i = 0; i < 2; i++) {
                uint8_t addr = (uint8_t)(i ? 0x1E : 0x1D);
                int w = i2c_write_timeout_us(i2c0, addr, &reg, 1, true, 2000);
                int r = (w == 1) ? i2c_read_timeout_us(i2c0, addr, &id, 1, false, 2000) : -9;
                printf("  WHO_AM_I at 0x%02X: write %d, read %d, id 0x%02X\n", addr, w, r, id);
            }
            printf("  lsm303d_init: FAILED (no LSM303D)\n");
        }
        sleep_ms(2000);
    }
}
