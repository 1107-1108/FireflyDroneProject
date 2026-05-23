#ifndef FIREFLYLINK_H
#define FIREFLYLINK_H

#include <stdint.h>

/* Max payload that fits in a 32-byte nRF24 packet:
 *   1 (header) + 1 (seq) + 1 (len) + N (payload) + N (ECC) + 2 (CRC) ≤ 32
 *   N ≤ 13  */
#define FIREFLYLINK_MAX_PAYLOAD 13
#define FIREFLYLINK_HEADER      0xFD

typedef enum {
    FIREFLYLINK_OK       =  0,
    FIREFLYLINK_ERR_CRC  = -1,
    FIREFLYLINK_ERR_LEN  = -2,
    FIREFLYLINK_ERR_HEAD = -3,
    FIREFLYLINK_ERR_SIZE = -4,
} fireflylink_status_t;

typedef struct {
    uint8_t seq;
    uint8_t payload_len;
    uint8_t payload[FIREFLYLINK_MAX_PAYLOAD];
    uint8_t ecc[FIREFLYLINK_MAX_PAYLOAD];
    uint16_t crc;
} FireflyLink_Packet;

/* Init packet with sequence number. */
void fireflylink_init(FireflyLink_Packet *pkt, uint8_t seq);

/* Pack payload into packet: computes ECC and CRC. Returns bytes_in_wire_format. */
int fireflylink_pack(FireflyLink_Packet *pkt, const uint8_t *data, uint8_t len);

/* Unpack: verify CRC then attempt Hamming single-bit correction. */
int fireflylink_unpack(FireflyLink_Packet *pkt, uint8_t *data_out, uint8_t *len_out);

/* Serialize packet to wire-format buffer. Returns bytes written or negative error. */
int fireflylink_serialize(const FireflyLink_Packet *pkt, uint8_t *buf, int buf_size);

/* Parse wire-format buffer into packet. Returns status code. */
int fireflylink_parse(const uint8_t *buf, int buf_size, FireflyLink_Packet *pkt_out);

/* Wire-format byte count for a packet. */
int fireflylink_wire_size(const FireflyLink_Packet *pkt);

/* CRC16-CCITT (polynomial 0x1021). */
uint16_t fireflylink_crc16(const uint8_t *data, int len);

#endif
