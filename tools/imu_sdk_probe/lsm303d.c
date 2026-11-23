#include "lsm303d.h"
#include <stddef.h>

#define WHO_AM_I 0x0F
#define WHO_VALUE 0x49
#define CTRL1 0x20
#define CTRL2 0x21
#define OUT_X_L_A 0x28

static bool reg_read(lsm303d_t *d, uint8_t reg, uint8_t *buf, size_t n) {
    if (i2c_write_blocking(d->bus, d->address, &reg, 1, true) != 1) return false;
    return i2c_read_blocking(d->bus, d->address, buf, n, false) == (int)n;
}
static bool reg_write(lsm303d_t *d, uint8_t reg, uint8_t value) {
    uint8_t bytes[2] = {reg, value};
    return i2c_write_blocking(d->bus, d->address, bytes, 2, false) == 2;
}

bool lsm303d_init(lsm303d_t *dev, i2c_inst_t *bus) {
    if (!dev || !bus) return false;
    dev->bus = bus;
    dev->ready = false;
    const uint8_t addresses[] = {0x1D, 0x1E};
    for (unsigned i = 0; i < 2; ++i) {
        dev->address = addresses[i];
        uint8_t id = 0;
        if (reg_read(dev, WHO_AM_I, &id, 1) && id == WHO_VALUE) {
            /* 50 Hz accelerometer; XYZ enabled; block-data update enabled. */
            if (!reg_write(dev, CTRL1, 0x57)) return false;
            /* +/-2 g full-scale (0x00). */
            if (!reg_write(dev, CTRL2, 0x00)) return false;
            dev->ready = true;
            return true;
        }
    }
    return false;
}

bool lsm303d_read_accel(lsm303d_t *dev, lsm303d_accel_t *out) {
    if (!dev || !dev->ready || !out) return false;
    uint8_t raw[6];
    /* Auto-increment registers for multi-byte I2C read. */
    if (!reg_read(dev, (uint8_t)(OUT_X_L_A | 0x80), raw, 6)) return false;
    int16_t x = (int16_t)((uint16_t)raw[1] << 8 | raw[0]);
    int16_t y = (int16_t)((uint16_t)raw[3] << 8 | raw[2]);
    int16_t z = (int16_t)((uint16_t)raw[5] << 8 | raw[4]);
    /* LSM303D +/-2g sensitivity: 0.061 mg/LSB. */
    out->x_g = (float)x * 0.000061f;
    out->y_g = (float)y * 0.000061f;
    out->z_g = (float)z * 0.000061f;
    return true;
}
