#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "nrf24L01.h"
#include "esp_log.h"
#include "esp_mac.h"
#include <string.h>


void nrf24_task(void *arg) {
    nrf24_config_t cfg = {
        .pin_miso = 13,
        .pin_mosi = 11,
        .pin_sck  = 12,
        .pin_csn  = 10,
        .pin_ce   = 9,
        .pin_irq  = -1,
        .spi_host = SPI2_HOST,
    };

    uint8_t addr[5] = {'N','R','F','2','4'};
    uint8_t buf[32];
    uint8_t len;

    nrf24_init(&cfg);
    nrf24_set_channel(76);
    nrf24_set_rx_addr_p0(addr, 5);
    nrf24_set_tx_addr(addr, 5);
    nrf24_start_listen();

    while (true) {
        if (nrf24_data_ready()) {
            memset(buf, 0, sizeof(buf));
            nrf24_recv(buf, &len);
            printf("recv: %s\n", buf);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}



void app_main(void)
{
    xTaskCreate(nrf24_task, "nrf24_task", 4096, NULL, 5, NULL);
}