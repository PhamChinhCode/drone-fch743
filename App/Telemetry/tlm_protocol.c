/**
 * @file    tlm_protocol.c
 * @brief   Hiện thực đóng gói / giải mã khung telemetry.
 */

#include "tlm_protocol.h"

/* ==========================================================================
 * CRC-16/CCITT-FALSE
 * ========================================================================== */

uint16_t tlm_crc16_update(uint16_t crc, uint8_t byte)
{
    crc ^= (uint16_t)byte << 8;
    for (int i = 0; i < 8; i++) {
        crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                              : (uint16_t)(crc << 1);
    }
    return crc;
}

uint16_t tlm_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0; i < len; i++) {
        crc = tlm_crc16_update(crc, data[i]);
    }
    return crc;
}

/* ==========================================================================
 * Đóng gói
 * ========================================================================== */

uint16_t tlm_frame_encode(uint8_t *dst, uint16_t dst_size,
                          uint8_t id, uint8_t seq,
                          const void *payload, uint8_t len)
{
    const uint16_t total = (uint16_t)len + TLM_OVERHEAD;

    if (dst == NULL || dst_size < total || len > TLM_MAX_PAYLOAD) {
        return 0;
    }
    if (len > 0 && payload == NULL) {
        return 0;
    }

    dst[0] = TLM_SYNC0;
    dst[1] = TLM_SYNC1;
    dst[2] = len;
    dst[3] = id;
    dst[4] = seq;

    if (len > 0) {
        memcpy(&dst[TLM_HEADER_SIZE], payload, len);
    }

    /* CRC tính từ byte LEN tới hết payload (bỏ qua 2 byte đồng bộ). */
    const uint16_t crc = tlm_crc16(&dst[2], (uint16_t)(len + 3u));

    dst[TLM_HEADER_SIZE + len]      = (uint8_t)(crc & 0xFFu);
    dst[TLM_HEADER_SIZE + len + 1u] = (uint8_t)(crc >> 8);

    return total;
}

/* ==========================================================================
 * Giải mã
 * ========================================================================== */

void tlm_parser_init(tlm_parser_t *p)
{
    if (p == NULL) {
        return;
    }
    memset(p, 0, sizeof(*p));
    p->state = TLM_RX_SYNC0;
}

bool tlm_parser_push(tlm_parser_t *p, uint8_t byte)
{
    if (p == NULL) {
        return false;
    }

    switch (p->state) {

    case TLM_RX_SYNC0:
        if (byte == TLM_SYNC0) {
            p->state = TLM_RX_SYNC1;
        }
        break;

    case TLM_RX_SYNC1:
        /* Nếu không khớp, có thể byte này lại là SYNC0 của khung sau. */
        p->state = (byte == TLM_SYNC1) ? TLM_RX_LEN
                 : (byte == TLM_SYNC0) ? TLM_RX_SYNC1
                                       : TLM_RX_SYNC0;
        break;

    case TLM_RX_LEN:
        if (byte > TLM_MAX_PAYLOAD) {
            p->overruns++;
            p->state = TLM_RX_SYNC0;
            break;
        }
        p->len      = byte;
        p->crc_calc = tlm_crc16_update(0xFFFFu, byte);
        p->state    = TLM_RX_ID;
        break;

    case TLM_RX_ID:
        p->id       = byte;
        p->crc_calc = tlm_crc16_update(p->crc_calc, byte);
        p->state    = TLM_RX_SEQ;
        break;

    case TLM_RX_SEQ:
        p->seq      = byte;
        p->crc_calc = tlm_crc16_update(p->crc_calc, byte);
        p->index    = 0;
        p->state    = (p->len > 0) ? TLM_RX_PAYLOAD : TLM_RX_CRC_LO;
        break;

    case TLM_RX_PAYLOAD:
        p->payload[p->index++] = byte;
        p->crc_calc = tlm_crc16_update(p->crc_calc, byte);
        if (p->index >= p->len) {
            p->state = TLM_RX_CRC_LO;
        }
        break;

    case TLM_RX_CRC_LO:
        p->crc_recv = byte;
        p->state    = TLM_RX_CRC_HI;
        break;

    case TLM_RX_CRC_HI:
        p->crc_recv |= (uint16_t)byte << 8;
        p->state = TLM_RX_SYNC0;
        if (p->crc_recv == p->crc_calc) {
            p->frames_ok++;
            return true;
        }
        p->crc_errors++;
        break;

    default:
        p->state = TLM_RX_SYNC0;
        break;
    }

    return false;
}
