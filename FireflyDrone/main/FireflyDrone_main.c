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

static const char *TAG = "board";

#define MKEF_FILTER_TIME_STEP           0.1 // second
#define PI                              3.1415926575

const double Q[3][3] = {
    {0.001, 0, 0},
    {0, 0.001, 0},
    {0, 0, 0.001}
};

const double R[3][3] = {
    {0.01, 0, 0},
    {0, 0.01, 0},
    {0, 0, 0.01}
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

void app_main(void) {
    xTaskCreate(mpu6050_task, "mpu6050_task", 4096, NULL, 5, NULL);
}