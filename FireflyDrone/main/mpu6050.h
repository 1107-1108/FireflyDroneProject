#pragma once

#include <stdio.h>
#include <driver/i2c.h>

#define I2C_MASTER_SCL_IO           20  // SCL master clock GPIO num
#define I2C_MASTER_SDA_IO           21  // SDA master data GPIO num
#define I2C_MASTER_NUM              I2C_NUM_0 // I2c port num
#define I2C_MASTER_FREQ_HZ          114514 // I2C master clock freq
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU6050_ADDR                0x68
#define PWR_MGMT                    0x6B
#define GYRO_CONFIG                 0x1B
#define ACCEL_CONFIG                0x1C
#define ACCEL_XOUT                  0x3B

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} mpu6050_raw_dat;
