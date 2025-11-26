#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "driver/i2c_master.h"
#include "mpu6050.h"

static const char *TAG = "MPU6050";

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

static esp_err_t mpu6050_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
}

static esp_err_t mpu6050_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
}

static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

static void mpu6050_init(i2c_master_dev_handle_t dev_handle) {
    ESP_ERROR_CHECK(mpu6050_register_write_byte(dev_handle, PWR_MGMT, 0x00)); //起床啦
    ESP_ERROR_CHECK(mpu6050_register_write_byte(dev_handle, ACCEL_CONFIG, 0x00));
    /*
    val | full scale range
    ----|-----------------
      0 |   +- 2g
      1 |   +- 4g
      2 |   +- 8g
      3 |   +- 16g
    */

    ESP_ERROR_CHECK(mpu6050_register_write_byte(dev_handle, GYRO_CONFIG, 0x00));
    /*
    val | full scale range(deg/S)
    ----|-------------------------
      0 |   +- 250
      1 |   +- 500
      2 |   +- 1000
      3 |   +- 2000
    */
   ESP_LOGI(TAG, "MPU6050 initialized");
}

static esp_err_t mpu6050_read_raw(i2c_master_dev_handle_t dev_handle, mpu6050_raw_dat *raw) {
    uint8_t data[14];
    ESP_ERROR_CHECK(mpu6050_register_read(dev_handle, 0x3B, data, 14));

    raw->ax = (data[0] << 8) | data[1];
    raw->ay = (data[2] << 8) | data[3];
    raw->az = (data[4] << 8) | data[5];
    raw->gx = (data[8] << 8) | data[9];
    raw->gy = (data[10] << 8) | data[11];
    raw->gz = (data[12] << 8) | data[13];

    return ESP_OK;
}

void mpu6050_task(void *arg) {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;

    i2c_master_init(&bus, &dev);
    mpu6050_init(dev);

    mpu6050_raw_dat raw;

    while(true) {
        mpu6050_read_raw(dev, &raw);

        ESP_LOGI(TAG, "Accel: [%d %d %d], Gyro: [%d %d %d]",
                 raw.ax, raw.ay, raw.az,
                 raw.gx, raw.gy, raw.gz);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void) {
    xTaskCreate(mpu6050_task, "mpu6050_task", 4096, NULL, 5, NULL);
}