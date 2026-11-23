#ifndef LSM303D_H
#define LSM303D_H
#include <stdbool.h>
#include <stdint.h>
#include "hardware/i2c.h"

typedef struct { float x_g, y_g, z_g; } lsm303d_accel_t;
typedef struct { i2c_inst_t *bus; uint8_t address; bool ready; } lsm303d_t;

/* Pico SDK driver. Caller must initialize I2C and GPIO pullups first. */
bool lsm303d_init(lsm303d_t *dev, i2c_inst_t *bus);
bool lsm303d_read_accel(lsm303d_t *dev, lsm303d_accel_t *out);
#endif
