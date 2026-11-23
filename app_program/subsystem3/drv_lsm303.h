/** @file drv_lsm303.h
 *
 * @brief LSM303D accelerometer + magnetometer on I2C0 (GP16 SDA, GP17
 *        SCL), with the LSM303DLHC (GY-511) as a fallback.  Sensor axes;
 *        imu_task.c maps them to the robot frame.
 */

#ifndef DRV_LSM303_H
#define DRV_LSM303_H

#include "subsystem3.h"

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} lsm303_vec_t;

/* Which chip drv_lsm303_open() found. */
#define LSM303_CHIP_NONE       0u
#define LSM303_CHIP_D          1u      /* LSM303D (the team's module).   */
#define LSM303_CHIP_DLHC       2u      /* LSM303DLHC / GY-511 fallback.  */

ER           drv_lsm303_open(void);
uint8_t      drv_lsm303_chip(void);
char const * drv_lsm303_chip_name(void);
ER           drv_lsm303_read_accel(lsm303_vec_t * p_mg);   /* milli-g    */
ER           drv_lsm303_read_mag(lsm303_vec_t * p_raw);    /* raw counts */
void         drv_lsm303_diagnose(void);
void         drv_lsm303_scan(void);
void         drv_lsm303_sweep(void);

#endif /* DRV_LSM303_H */

/*** end of file ***/
