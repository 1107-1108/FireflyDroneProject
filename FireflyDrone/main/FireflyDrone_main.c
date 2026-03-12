#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "driver/i2c_master.h"
#include "mpu6050.h"
#include "FireflyDrone_main.h"
#include "AttitudeCtrl.h"
#include "quaternion.h"
#include "nrf24L01.h"
#include "driver/spi_master.h"

static const char *TAG = "board";

#define MKEF_FILTER_TIME_STEP           0.1 // second
#define PI                              3.1415926575

const double Q[3][3] = {
    {0.005, 0, 0},
    {0, 0.005, 0},
    {0, 0, 0.005}
};

const double R[3][3] = {
    {0.03, 0, 0},
    {0, 0.03, 0},
    {0, 0, 0.03}
};

Quaternion q = {1.0, 0.0, 0.0, 0.0};
double roll, pitch, yaw;
double gyro_sens = 131.0;


void mpu6050_task(void *arg) {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;

    i2c_master_init(&bus, &dev);
    mpu6050_init(dev);

    mpu6050_raw_dat raw;

    while(true) {
        mpu6050_read_raw(dev, &raw);
        // quatGyroUpdate(&q, raw.gx / gyro_sens * (PI / 180.0), raw.gy, raw.gz, 0.01);

        ESP_LOGI(TAG, "Accel: [%d %d %d], Gyro: [%d %d %d]",
                 raw.ax, raw.ay, raw.az,
                 raw.gx, raw.gy, raw.gz);
        
        MEKF_filter(MKEF_FILTER_TIME_STEP, Q, R, &raw, &roll, &pitch, &yaw);
        
        // quatToEulerFloat(q, &roll, &pitch, &yaw);
        
        ESP_LOGI(TAG, "roll:  %.2f, pitch:  %.2f, yaw:  %.2f]", roll, pitch, yaw);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void nrf24_task(void *arg) {
    nrf24_config_t cfg = {
        .pin_miso = 13,
        .pin_mosi = 11,
        .pin_sck = 12,
        .pin_csn = 10,
        .pin_ce = 9,
        .pin_irq = -1,
        .spi_host = SPI2_HOST,
    };

    uint8_t addr[5] = {'N', 'R', 'F', '2', '4'};


    ESP_ERROR_CHECK(nrf24_init(&cfg));
    ESP_ERROR_CHECK(nrf24_set_channel(76));
    ESP_ERROR_CHECK(nrf24_set_tx_addr(addr, 5));
    ESP_ERROR_CHECK(nrf24_set_rx_addr_p0(addr, 5)); 
    ESP_ERROR_CHECK(nrf24_print_regs());

    uint32_t cnt = 0;
    uint8_t tx_buf[32];

    while(true) {
        memset(tx_buf, 0, sizeof(tx_buf));
        snprintf((char *)tx_buf, sizeof(tx_buf), "hello %04lu", (unsigned long)cnt++);

        esp_err_t ret = nrf24_send(tx_buf, 32);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "send ok: %s", tx_buf);
        } else {
            ESP_LOGE(TAG, "send failed: %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void) {
    //xTaskCreate(mpu6050_task, "mpu6050_task", 4096, NULL, 5, NULL);
    xTaskCreate(nrf24_task, "nrf24_task", 4096, NULL, 5, NULL);
}