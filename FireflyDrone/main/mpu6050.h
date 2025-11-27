#pragma once

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "driver/i2c_master.h"
#include "mpu6050.h"

#define I2C_MASTER_SCL_IO           20  // SCL master clock GPIO num
#define I2C_MASTER_SDA_IO           21  // SDA master data GPIO num
#define I2C_MASTER_NUM              I2C_NUM_0 // I2c port num
#define I2C_MASTER_FREQ_HZ          100000 // I2C master clock freq
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

esp_err_t mpu6050_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t mpu6050_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data);
void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle);
void mpu6050_init(i2c_master_dev_handle_t dev_handle);
esp_err_t mpu6050_read_raw(i2c_master_dev_handle_t dev_handle, mpu6050_raw_dat *raw);
