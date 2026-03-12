#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    int pin_miso;
    int pin_mosi;
    int pin_sck;
    int pin_csn;
    int pin_ce;
    int pin_irq;      // 可为 -1
    int spi_host;     // SPI2_HOST / SPI3_HOST
} nrf24_config_t;

esp_err_t nrf24_init(const nrf24_config_t *cfg);
esp_err_t nrf24_set_channel(uint8_t ch);
esp_err_t nrf24_set_tx_addr(const uint8_t *addr, uint8_t len);
esp_err_t nrf24_set_rx_addr_p0(const uint8_t *addr, uint8_t len);
esp_err_t nrf24_start_listen(void);
esp_err_t nrf24_stop_listen(void);
esp_err_t nrf24_send(const uint8_t *data, uint8_t len);
esp_err_t nrf24_recv(uint8_t *data, uint8_t *len);
bool nrf24_data_ready(void);
esp_err_t nrf24_print_regs(void);
