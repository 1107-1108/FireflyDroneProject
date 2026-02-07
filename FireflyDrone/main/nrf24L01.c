#include "nrf24L01.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

uint8_t NRF24L01_init(void) {
    uint8_t buf[5] = {0XA5, 0XA5, 0XA5, 0XA5, 0XA5};
    uint8_t i;
    
}



