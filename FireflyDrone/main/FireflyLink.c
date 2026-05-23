#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "FireflyLink.h"

/* -------------------------------------------------------------------------- */
/*  CRC16-CCITT                                                              */
/* -------------------------------------------------------------------------- */

uint16_t fireflylink_crc16(const uint8_t *data, int len) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/* -------------------------------------------------------------------------- */
/*  Hamming(7,4) — encode one nibble into 3 parity bits                       */
/* -------------------------------------------------------------------------- */

static uint8_t hamming_encode_nibble(uint8_t nibble) {
    uint8_t d1 = (nibble >> 3) & 1;
    uint8_t d2 = (nibble >> 2) & 1;
    uint8_t d3 = (nibble >> 1) & 1;
    uint8_t d4 =  nibble       & 1;

    uint8_t p1 = d1 ^ d2 ^ d4;
    uint8_t p2 = d1 ^ d3 ^ d4;
    uint8_t p3 = d2 ^ d3 ^ d4;

    return (p1 << 2) | (p2 << 1) | p3;
}

/* -------------------------------------------------------------------------- */
/*  Hamming(7,4) — decode + correct one nibble                                */
/*  Returns true if a data bit was flipped, false otherwise.                  */
/* -------------------------------------------------------------------------- */

static bool hamming_decode_nibble(uint8_t parity, uint8_t *nibble) {
    uint8_t rec_p1 = (parity >> 2) & 1;
    uint8_t rec_p2 = (parity >> 1) & 1;
    uint8_t rec_p3 =  parity       & 1;

    uint8_t d1 = (*nibble >> 3) & 1;
    uint8_t d2 = (*nibble >> 2) & 1;
    uint8_t d3 = (*nibble >> 1) & 1;
    uint8_t d4 = *nibble        & 1;

    uint8_t calc_p1 = d1 ^ d2 ^ d4;
    uint8_t calc_p2 = d1 ^ d3 ^ d4;
    uint8_t calc_p3 = d2 ^ d3 ^ d4;

    uint8_t s1 = rec_p1 ^ calc_p1;
    uint8_t s2 = rec_p2 ^ calc_p2;
    uint8_t s3 = rec_p3 ^ calc_p3;

    if (s1 == 0 && s2 == 0 && s3 == 0)
        return false;

    /* syndrome s3:s2:s1 maps to bit position in [p1,p2,d1,p3,d2,d3,d4] */
    uint8_t pos = (s3 << 2) | (s2 << 1) | s1;

    switch (pos) {
    case 3: *nibble ^= 0x8; return true;  /* d1 */
    case 5: *nibble ^= 0x4; return true;  /* d2 */
    case 6: *nibble ^= 0x2; return true;  /* d3 */
    case 7: *nibble ^= 0x1; return true;  /* d4 */
    default: return false;                 /* parity-bit error, data ok */
    }
}

/* -------------------------------------------------------------------------- */
/*  Encode entire payload → 1 ECC byte per payload byte                       */
/*  ECC byte layout: [p1_hi p2_hi p3_hi p1_lo p2_lo p3_lo 0 0]               */
/* -------------------------------------------------------------------------- */

static void encode_ecc(const uint8_t *payload, int len, uint8_t *ecc_out) {
    for (int i = 0; i < len; i++) {
        uint8_t hi_nibble = payload[i] >> 4;
        uint8_t lo_nibble = payload[i] & 0x0F;
        uint8_t p_hi = hamming_encode_nibble(hi_nibble);
        uint8_t p_lo = hamming_encode_nibble(lo_nibble);
        ecc_out[i] = (p_hi << 3) | p_lo;
    }
}

/* -------------------------------------------------------------------------- */
/*  Decode + correct payload using ECC.                                       */
/*  Returns number of bits corrected, or -1 if uncorrectable error suspected. */
/* -------------------------------------------------------------------------- */

static int decode_ecc(uint8_t *payload, int len, const uint8_t *ecc) {
    int corrections = 0;
    for (int i = 0; i < len; i++) {
        uint8_t hi_nibble = payload[i] >> 4;
        uint8_t lo_nibble = payload[i] & 0x0F;

        uint8_t p_hi = (ecc[i] >> 3) & 0x07;
        uint8_t p_lo =  ecc[i]       & 0x07;

        if (hamming_decode_nibble(p_hi, &hi_nibble)) corrections++;
        if (hamming_decode_nibble(p_lo, &lo_nibble)) corrections++;

        payload[i] = (hi_nibble << 4) | lo_nibble;
    }
    return corrections;
}

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */

void fireflylink_init(FireflyLink_Packet *pkt, uint8_t seq) {
    memset(pkt, 0, sizeof(*pkt));
    pkt->seq = seq;
}

int fireflylink_pack(FireflyLink_Packet *pkt, const uint8_t *data, uint8_t len) {
    if (len > FIREFLYLINK_MAX_PAYLOAD)
        return FIREFLYLINK_ERR_SIZE;

    pkt->payload_len = len;
    memcpy(pkt->payload, data, len);
    encode_ecc(pkt->payload, len, pkt->ecc);

    /* CRC over [seq, payload_len, payload, ecc] */
    uint8_t crc_buf[1 + 1 + FIREFLYLINK_MAX_PAYLOAD * 2];
    crc_buf[0] = pkt->seq;
    crc_buf[1] = pkt->payload_len;
    memcpy(crc_buf + 2, pkt->payload, len);
    memcpy(crc_buf + 2 + len, pkt->ecc, len);
    pkt->crc = fireflylink_crc16(crc_buf, 2 + len + len);

    return fireflylink_wire_size(pkt);
}

int fireflylink_unpack(FireflyLink_Packet *pkt, uint8_t *data_out, uint8_t *len_out) {
    /* --- CRC check before correction --- */
    uint8_t crc_buf[1 + 1 + FIREFLYLINK_MAX_PAYLOAD * 2];
    crc_buf[0] = pkt->seq;
    crc_buf[1] = pkt->payload_len;
    memcpy(crc_buf + 2, pkt->payload, pkt->payload_len);
    memcpy(crc_buf + 2 + pkt->payload_len, pkt->ecc, pkt->payload_len);

    uint16_t computed = fireflylink_crc16(crc_buf, 2 + pkt->payload_len * 2);

    /* CRC match — no errors, fast path */
    if (computed == pkt->crc) {
        memcpy(data_out, pkt->payload, pkt->payload_len);
        *len_out = pkt->payload_len;
        return FIREFLYLINK_OK;
    }

    /* CRC mismatch — attempt Hamming single-bit correction */
    int n = decode_ecc(pkt->payload, pkt->payload_len, pkt->ecc);

    /* Recompute CRC after correction */
    memcpy(crc_buf + 2, pkt->payload, pkt->payload_len);
    memcpy(crc_buf + 2 + pkt->payload_len, pkt->ecc, pkt->payload_len);
    computed = fireflylink_crc16(crc_buf, 2 + pkt->payload_len * 2);

    if (computed == pkt->crc) {
        memcpy(data_out, pkt->payload, pkt->payload_len);
        *len_out = pkt->payload_len;
        return n;  /* success, return number of bits corrected */
    }

    return FIREFLYLINK_ERR_CRC;
}

int fireflylink_serialize(const FireflyLink_Packet *pkt, uint8_t *buf, int buf_size) {
    int size = fireflylink_wire_size(pkt);
    if (size > buf_size)
        return FIREFLYLINK_ERR_SIZE;

    int pos = 0;
    buf[pos++] = FIREFLYLINK_HEADER;
    buf[pos++] = pkt->seq;
    buf[pos++] = pkt->payload_len;
    memcpy(buf + pos, pkt->payload, pkt->payload_len);  pos += pkt->payload_len;
    memcpy(buf + pos, pkt->ecc, pkt->payload_len);      pos += pkt->payload_len;
    buf[pos++] = (uint8_t)(pkt->crc >> 8);
    buf[pos++] = (uint8_t)(pkt->crc & 0xFF);

    return pos;
}

int fireflylink_parse(const uint8_t *buf, int buf_size, FireflyLink_Packet *pkt_out) {
    if (buf_size < 5)
        return FIREFLYLINK_ERR_SIZE;
    if (buf[0] != FIREFLYLINK_HEADER)
        return FIREFLYLINK_ERR_HEAD;

    int pos = 1;
    memset(pkt_out, 0, sizeof(*pkt_out));
    pkt_out->seq         = buf[pos++];
    pkt_out->payload_len = buf[pos++];

    if (pkt_out->payload_len > FIREFLYLINK_MAX_PAYLOAD)
        return FIREFLYLINK_ERR_LEN;

    int expected = 1 + 1 + 1 + 2 * pkt_out->payload_len + 2;
    if (buf_size < expected)
        return FIREFLYLINK_ERR_SIZE;

    memcpy(pkt_out->payload, buf + pos, pkt_out->payload_len);  pos += pkt_out->payload_len;
    memcpy(pkt_out->ecc,     buf + pos, pkt_out->payload_len);  pos += pkt_out->payload_len;
    pkt_out->crc = ((uint16_t)buf[pos] << 8) | buf[pos + 1];

    return FIREFLYLINK_OK;
}

int fireflylink_wire_size(const FireflyLink_Packet *pkt) {
    return 1 + 1 + 1 + 2 * pkt->payload_len + 2;
}
