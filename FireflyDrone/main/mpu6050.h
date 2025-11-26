#pragma once

#include <stdio.h>
#include <driver/i2c.h>

#define MPU6050_ADDR           0x68
#define MPU_6050                0x6B
#define MPU6050_ACCEL_XOUT_H    0x3B

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} mpu6050_raw_dat;
