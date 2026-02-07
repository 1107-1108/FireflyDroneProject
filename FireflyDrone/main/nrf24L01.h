#define NRF24L01_HOST   SPI1_HOST
#define PIN_NUM_MISO    7
#define PIN_NUM_MOSI    8
#define PIN_NUM_CLK     6
#define PIN_NUM_CS      13

// registers
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//NRF24L01寄存器操作命令
#define NRF_READ_REG    0x00
#define NRF_WRITE_REG   0x20
#define RD_RX_PLOAD     0x61
#define WR_TX_PLOAD     0xA0
#define FLUSH_TX        0xE1
#define FLUSH_RX        0xE2
#define REUSE_TX_PL     0xE3
#define NOP             0xFF
//SPI(NRF24L01)寄存器地址
#define CONFIG          0x00
#define EN_AA           0x01
#define EN_RXADDR       0x02
#define SETUP_AW        0x03
#define SETUP_RETR      0x04
#define RF_CH           0x05
#define RF_SETUP        0x06
#define STATUS          0x07
#define MAX_TX  		0x10
#define TX_OK   		0x20
#define RX_OK   		0x40

