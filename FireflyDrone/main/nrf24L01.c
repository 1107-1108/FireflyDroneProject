#include "nrf24L01.h"
#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

static const char *TAG = "NRF24";

static spi_device_handle_t s_dev;
static nrf24_config_t s_cfg;

// SPI commands
#define NRF_R_REGISTER         0x00
#define NRF_W_REGISTER         0x20
#define NRF_R_RX_PAYLOAD       0x61
#define NRF_W_TX_PAYLOAD       0xA0
#define NRF_FLUSH_TX           0xE1
#define NRF_FLUSH_RX           0xE2
#define NRF_REUSE_TX_PL        0xE3
#define NRF_NOP                0xFF

// Registers
#define REG_CONFIG             0x00
#define REG_EN_AA              0x01
#define REG_EN_RXADDR          0x02
#define REG_SETUP_AW           0x03
#define REG_SETUP_RETR         0x04
#define REG_RF_CH              0x05
#define REG_RF_SETUP           0x06
#define REG_STATUS             0x07
#define REG_OBSERVE_TX         0x08
#define REG_RPD                0x09
#define REG_RX_ADDR_P0         0x0A
#define REG_TX_ADDR            0x10
#define REG_RX_PW_P0           0x11
#define REG_FIFO_STATUS        0x17
#define REG_DYNPD              0x1C
#define REG_FEATURE            0x1D

// CONFIG bits
#define CONFIG_MASK_RX_DR      (1 << 6)
#define CONFIG_MASK_TX_DS      (1 << 5)
#define CONFIG_MASK_MAX_RT     (1 << 4)
#define CONFIG_EN_CRC          (1 << 3)
#define CONFIG_CRCO            (1 << 2)
#define CONFIG_PWR_UP          (1 << 1)
#define CONFIG_PRIM_RX         (1 << 0)

// STATUS bits
#define STATUS_RX_DR           (1 << 6)
#define STATUS_TX_DS           (1 << 5)
#define STATUS_MAX_RT          (1 << 4)

// FIFO_STATUS bits
#define FIFO_TX_REUSE          (1 << 6)
#define FIFO_TX_FULL           (1 << 5)
#define FIFO_TX_EMPTY          (1 << 4)
#define FIFO_RX_FULL           (1 << 1)
#define FIFO_RX_EMPTY          (1 << 0)

static inline void nrf_ce(int level)
{
    gpio_set_level(s_cfg.pin_ce, level);
}

static esp_err_t nrf_cmd(uint8_t cmd, uint8_t *status)
{
    uint8_t tx[1] = { cmd };
    uint8_t rx[1] = { 0 };

    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    ESP_RETURN_ON_ERROR(spi_device_transmit(s_dev, &t), TAG, "cmd failed");
    if (status) *status = rx[0];
    return ESP_OK;
}

static esp_err_t nrf_read_reg(uint8_t reg, uint8_t *val)
{
    uint8_t tx[2] = { (uint8_t)(NRF_R_REGISTER | (reg & 0x1F)), NRF_NOP };
    uint8_t rx[2] = { 0 };

    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    ESP_RETURN_ON_ERROR(spi_device_transmit(s_dev, &t), TAG, "read reg failed");
    *val = rx[1];
    return ESP_OK;
}

static esp_err_t nrf_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(NRF_W_REGISTER | (reg & 0x1F)), val };

    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
    };

    return spi_device_transmit(s_dev, &t);
}

static esp_err_t nrf_read_buf(uint8_t reg, uint8_t *buf, size_t len)
{
    uint8_t cmd = NRF_R_REGISTER | (reg & 0x1F);

    spi_transaction_t t1 = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    spi_transaction_t t2 = {
        .length = len * 8,
        .rx_buffer = buf,
    };

    ESP_RETURN_ON_ERROR(spi_device_acquire_bus(s_dev, portMAX_DELAY), TAG, "acquire bus failed");
    esp_err_t ret = spi_device_polling_transmit(s_dev, &t1);
    if (ret == ESP_OK) ret = spi_device_polling_transmit(s_dev, &t2);
    spi_device_release_bus(s_dev);

    return ret;
}

static esp_err_t nrf_write_buf(uint8_t reg, const uint8_t *buf, size_t len)
{
    uint8_t cmd = NRF_W_REGISTER | (reg & 0x1F);

    spi_transaction_t t1 = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    spi_transaction_t t2 = {
        .length = len * 8,
        .tx_buffer = buf,
    };

    ESP_RETURN_ON_ERROR(spi_device_acquire_bus(s_dev, portMAX_DELAY), TAG, "acquire bus failed");
    esp_err_t ret = spi_device_polling_transmit(s_dev, &t1);
    if (ret == ESP_OK) ret = spi_device_polling_transmit(s_dev, &t2);
    spi_device_release_bus(s_dev);

    return ret;
}

static esp_err_t nrf_write_payload(const uint8_t *data, size_t len)
{
    uint8_t cmd = NRF_W_TX_PAYLOAD;

    spi_transaction_t t1 = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    spi_transaction_t t2 = {
        .length = len * 8,
        .tx_buffer = data,
    };

    ESP_RETURN_ON_ERROR(spi_device_acquire_bus(s_dev, portMAX_DELAY), TAG, "acquire bus failed");
    esp_err_t ret = spi_device_polling_transmit(s_dev, &t1);
    if (ret == ESP_OK) ret = spi_device_polling_transmit(s_dev, &t2);
    spi_device_release_bus(s_dev);

    return ret;
}

static esp_err_t nrf_read_payload(uint8_t *data, size_t len)
{
    uint8_t cmd = NRF_R_RX_PAYLOAD;

    spi_transaction_t t1 = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    spi_transaction_t t2 = {
        .length = len * 8,
        .rx_buffer = data,
    };

    ESP_RETURN_ON_ERROR(spi_device_acquire_bus(s_dev, portMAX_DELAY), TAG, "acquire bus failed");
    esp_err_t ret = spi_device_polling_transmit(s_dev, &t1);
    if (ret == ESP_OK) ret = spi_device_polling_transmit(s_dev, &t2);
    spi_device_release_bus(s_dev);

    return ret;
}

static esp_err_t nrf_flush_tx(void) { return nrf_cmd(NRF_FLUSH_TX, NULL); }
static esp_err_t nrf_flush_rx(void) { return nrf_cmd(NRF_FLUSH_RX, NULL); }
static esp_err_t nrf_clear_irq(void)
{
    // 写1清除 RX_DR / TX_DS / MAX_RT
    return nrf_write_reg(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);
}

esp_err_t nrf24_init(const nrf24_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg, ESP_ERR_INVALID_ARG, TAG, "cfg null");
    s_cfg = *cfg;

    spi_bus_config_t buscfg = {
        .mosi_io_num = cfg->pin_mosi,
        .miso_io_num = cfg->pin_miso,
        .sclk_io_num = cfg->pin_sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2 * 1000 * 1000,   // nRF24L01 SPI 上限 10MHz，先用 2MHz 稳一点
        .mode = 0,
        .spics_io_num = cfg->pin_csn,
        .queue_size = 4,
    };

    ESP_RETURN_ON_ERROR(spi_bus_initialize(cfg->spi_host, &buscfg, SPI_DMA_CH_AUTO), TAG, "spi bus init failed");
    ESP_RETURN_ON_ERROR(spi_bus_add_device(cfg->spi_host, &devcfg, &s_dev), TAG, "spi add dev failed");

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << cfg->pin_ce,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "ce gpio failed");

    if (cfg->pin_irq >= 0) {
        gpio_config_t irq_conf = {
            .pin_bit_mask = 1ULL << cfg->pin_irq,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = 1,
            .pull_down_en = 0,
            .intr_type = GPIO_INTR_NEGEDGE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&irq_conf), TAG, "irq gpio failed");
    }

    nrf_ce(0);
    esp_rom_delay_us(5000);

    ESP_RETURN_ON_ERROR(nrf_write_reg(REG_CONFIG, CONFIG_EN_CRC | CONFIG_CRCO), TAG, "config failed");
    ESP_RETURN_ON_ERROR(nrf_write_reg(REG_EN_AA, 0x01), TAG, "en_aa failed");          // 只开 pipe0 自动应答
    ESP_RETURN_ON_ERROR(nrf_write_reg(REG_EN_RXADDR, 0x01), TAG, "en_rxaddr failed");  // 只开 pipe0
    ESP_RETURN_ON_ERROR(nrf_write_reg(REG_SETUP_AW, 0x03), TAG, "setup_aw failed");    // 5字节地址
    ESP_RETURN_ON_ERROR(nrf_write_reg(REG_SETUP_RETR, 0x3F), TAG, "setup_retr failed");// 自动重发
    ESP_RETURN_ON_ERROR(nrf_write_reg(REG_RF_CH, 76), TAG, "rf_ch failed");            // 2.476GHz
    ESP_RETURN_ON_ERROR(nrf_write_reg(REG_RF_SETUP, 0x06), TAG, "rf_setup failed");    // 1Mbps, 0dBm（常见配置）
    ESP_RETURN_ON_ERROR(nrf_write_reg(REG_RX_PW_P0, 32), TAG, "rx_pw_p0 failed");      // 固定32字节payload
    ESP_RETURN_ON_ERROR(nrf_flush_tx(), TAG, "flush tx failed");
    ESP_RETURN_ON_ERROR(nrf_flush_rx(), TAG, "flush rx failed");
    ESP_RETURN_ON_ERROR(nrf_clear_irq(), TAG, "clear irq failed");

    uint8_t addr[5] = { 'N', 'R', 'F', '2', '4' };
    ESP_RETURN_ON_ERROR(nrf_write_buf(REG_TX_ADDR, addr, 5), TAG, "tx addr failed");
    ESP_RETURN_ON_ERROR(nrf_write_buf(REG_RX_ADDR_P0, addr, 5), TAG, "rx addr p0 failed");

    // PWR_UP
    uint8_t cfg_reg = CONFIG_EN_CRC | CONFIG_CRCO | CONFIG_PWR_UP;
    ESP_RETURN_ON_ERROR(nrf_write_reg(REG_CONFIG, cfg_reg), TAG, "power up failed");
    esp_rom_delay_us(5000); // 上电到 standby 需要时间

    return ESP_OK;
}

esp_err_t nrf24_start_listen(void)
{
    uint8_t cfg;
    ESP_RETURN_ON_ERROR(nrf_read_reg(REG_CONFIG, &cfg), TAG, "read config failed");
    cfg |= CONFIG_PWR_UP | CONFIG_PRIM_RX;
    ESP_RETURN_ON_ERROR(nrf_write_reg(REG_CONFIG, cfg), TAG, "write config failed");
    ESP_RETURN_ON_ERROR(nrf_clear_irq(), TAG, "clear irq failed");
    ESP_RETURN_ON_ERROR(nrf_flush_rx(), TAG, "flush rx failed");
    esp_rom_delay_us(150);
    nrf_ce(1);
    return ESP_OK;
}

esp_err_t nrf24_stop_listen(void)
{
    nrf_ce(0);
    uint8_t cfg;
    ESP_RETURN_ON_ERROR(nrf_read_reg(REG_CONFIG, &cfg), TAG, "read config failed");
    cfg &= ~CONFIG_PRIM_RX;
    cfg |= CONFIG_PWR_UP;
    ESP_RETURN_ON_ERROR(nrf_write_reg(REG_CONFIG, cfg), TAG, "write config failed");
    esp_rom_delay_us(150);
    return ESP_OK;
}

esp_err_t nrf24_send(const uint8_t *data, uint8_t len)
{
    ESP_RETURN_ON_FALSE(data && len > 0 && len <= 32, ESP_ERR_INVALID_ARG, TAG, "bad payload");

    ESP_RETURN_ON_ERROR(nrf24_stop_listen(), TAG, "stop listen failed");
    ESP_RETURN_ON_ERROR(nrf_flush_tx(), TAG, "flush tx failed");
    ESP_RETURN_ON_ERROR(nrf_clear_irq(), TAG, "clear irq failed");
    ESP_RETURN_ON_ERROR(nrf_write_payload(data, len), TAG, "write payload failed");

    // CE 高脉冲启动发送
    nrf_ce(1);
    esp_rom_delay_us(15);   // >10us 更稳
    nrf_ce(0);

    uint8_t status = 0;
    for (int i = 0; i < 200; i++) {
        ESP_RETURN_ON_ERROR(nrf_read_reg(REG_STATUS, &status), TAG, "read status failed");
        if (status & STATUS_TX_DS) {
            nrf_clear_irq();
            return ESP_OK;
        }
        if (status & STATUS_MAX_RT) {
            nrf_clear_irq();
            nrf_flush_tx();
            return ESP_FAIL;
        }
        esp_rom_delay_us(100);
    }

    return ESP_ERR_TIMEOUT;
}

bool nrf24_data_ready(void)
{
    uint8_t status = 0;
    if (nrf_read_reg(REG_STATUS, &status) != ESP_OK) {
        return false;
    }
    return (status & STATUS_RX_DR) != 0;
}

esp_err_t nrf24_recv(uint8_t *data, uint8_t *len)
{
    ESP_RETURN_ON_FALSE(data && len, ESP_ERR_INVALID_ARG, TAG, "bad arg");

    uint8_t status = 0;
    ESP_RETURN_ON_ERROR(nrf_read_reg(REG_STATUS, &status), TAG, "read status failed");
    if (!(status & STATUS_RX_DR)) {
        return ESP_ERR_NOT_FOUND;
    }

    // 这里按固定 payload 32 字节处理；如果你要动态 payload，驱动要再扩展
    ESP_RETURN_ON_ERROR(nrf_read_payload(data, 32), TAG, "read payload failed");
    *len = 32;

    ESP_RETURN_ON_ERROR(nrf_clear_irq(), TAG, "clear irq failed");

    uint8_t fifo = 0;
    ESP_RETURN_ON_ERROR(nrf_read_reg(REG_FIFO_STATUS, &fifo), TAG, "read fifo failed");
    if (!(fifo & FIFO_RX_EMPTY)) {
        // 还有包没读，外层继续调 recv 即可
    }

    return ESP_OK;
}

esp_err_t nrf24_set_channel(uint8_t ch)
{
    if (ch > 125) return ESP_ERR_INVALID_ARG;
    return nrf_write_reg(REG_RF_CH, ch);
}

esp_err_t nrf24_set_tx_addr(const uint8_t *addr, uint8_t len)
{
    if (!addr || len < 3 || len > 5) return ESP_ERR_INVALID_ARG;
    return nrf_write_buf(REG_TX_ADDR, addr, len);
}

esp_err_t nrf24_set_rx_addr_p0(const uint8_t *addr, uint8_t len)
{
    if (!addr || len < 3 || len > 5) return ESP_ERR_INVALID_ARG;
    return nrf_write_buf(REG_RX_ADDR_P0, addr, len);
}

esp_err_t nrf24_print_regs(void)
{
    uint8_t v = 0;
    const uint8_t regs[] = {
        REG_CONFIG, REG_EN_AA, REG_EN_RXADDR, REG_SETUP_AW, REG_SETUP_RETR,
        REG_RF_CH, REG_RF_SETUP, REG_STATUS, REG_RX_PW_P0, REG_FIFO_STATUS
    };

    for (size_t i = 0; i < sizeof(regs); i++) {
        ESP_RETURN_ON_ERROR(nrf_read_reg(regs[i], &v), TAG, "read reg failed");
        ESP_LOGI(TAG, "REG 0x%02X = 0x%02X", regs[i], v);
    }
    return ESP_OK;
}