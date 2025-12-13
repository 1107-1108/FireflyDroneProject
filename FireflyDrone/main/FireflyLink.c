#include <stdio.h>
#include <stdint.h>

#define MAX_PAYLOAD 256
#define FIREFLYLINK_VERSION 101

typedef struct {
    uint8_t header; //Version...
    uint8_t payload[MAX_PAYLOAD];    // 数据
    int payload_len;    // 数据长度
    uint8_t ecc[4];     // 海明码
    uint16_t crc;       // CRC16
    uint8_t footer;
} FireflyLink_Protocol_Packet;


uint16_t crc16(uint8_t *data, int len) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}

void generate_ecc(uint8_t *payload, int len, uint8_t *ecc_out) {
    int ecc_index = 0;
    memset(ecc_out, 0, 4);

    for(int i = 0; i < len; i++) {
        uint8_t byte = payload[i];

        uint8_t d1 = (byte >> 7) & 1;
        uint8_t d2 = (byte >> 6) & 1;
        uint8_t d3 = (byte >> 5) & 1;
        uint8_t d4 = (byte >> 4) & 1;

        uint8_t p1 = d1 ^ d2 ^ d4;
        uint8_t p2 = d1 ^ d3 ^ d4;
        uint8_t p3 = d2 ^ d3 ^ d4;

        ecc_out[ecc_index++] = (p1 << 2) | (p2 << 1) | p3;

        d1 = (byte >> 3) & 1;
        d2 = (byte >> 2) & 1;
        d3 = (byte >> 1) & 1;
        d4 = byte & 1;

        p1 = d1 ^ d2 ^ d4;
        p2 = d1 ^ d3 ^ d4;
        p3 = d2 ^ d3 ^ d4;

        ecc_out[ecc_index++] = (p1 << 2) | (p2 << 1) | p3;
    }
}
