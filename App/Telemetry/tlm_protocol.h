/**
 * @file    tlm_protocol.h
 * @brief   Khung gói tin telemetry — đóng gói, kiểm tra CRC, giải mã.
 *
 * ĐỊNH DẠNG KHUNG (little-endian toàn bộ):
 *
 *   Byte  0 : SYNC0 = 0xFE
 *   Byte  1 : SYNC1 = 0x5A
 *   Byte  2 : LEN   — số byte payload (0..TLM_MAX_PAYLOAD)
 *   Byte  3 : ID    — mã bản tin, xem tlm_msg_id_t
 *   Byte  4 : SEQ   — bộ đếm tăng dần, giúp phát hiện mất gói
 *   Byte  5..(4+LEN) : PAYLOAD
 *   2 byte cuối      : CRC-16/CCITT-FALSE, tính trên byte 2 .. hết payload
 *
 * Tổng phụ phí: 7 byte mỗi gói.
 *
 * VÌ SAO KHÔNG DÙNG MAVLink / MSP:
 *   Khung này đủ nhẹ để chạy ở 4 kHz mà vẫn có CRC và số thứ tự. Nếu sau này
 *   cần ghép nối với QGroundControl thì viết thêm một lớp chuyển đổi, phần
 *   dữ liệu trong App/State không phải đổi.
 */
#ifndef TLM_PROTOCOL_H
#define TLM_PROTOCOL_H

#include "fc_types.h"
#include "fc_config.h"

#define TLM_SYNC0            0xFEu
#define TLM_SYNC1            0x5Au
#define TLM_HEADER_SIZE      5u
#define TLM_CRC_SIZE         2u
#define TLM_OVERHEAD         (TLM_HEADER_SIZE + TLM_CRC_SIZE)
#define TLM_FRAME_MAX        (TLM_MAX_PAYLOAD + TLM_OVERHEAD)

/* ==========================================================================
 * CRC-16/CCITT-FALSE  (đa thức 0x1021, giá trị khởi tạo 0xFFFF)
 * ========================================================================== */

/** Cập nhật CRC với một byte. Dùng để tính dần khi nhận từng byte. */
uint16_t tlm_crc16_update(uint16_t crc, uint8_t byte);

/** Tính CRC cho cả khối dữ liệu. */
uint16_t tlm_crc16(const uint8_t *data, uint16_t len);

/* ==========================================================================
 * Đóng gói
 * ========================================================================== */

/**
 * Ghép một khung hoàn chỉnh vào bộ đệm.
 *
 * @param dst      bộ đệm đích, phải chứa được ít nhất (len + TLM_OVERHEAD) byte
 * @param dst_size kích thước bộ đệm đích
 * @param id       mã bản tin
 * @param seq      số thứ tự (người gọi tự tăng)
 * @param payload  dữ liệu, có thể NULL nếu len == 0
 * @param len      độ dài payload
 * @return         số byte đã ghi, hoặc 0 nếu bộ đệm không đủ chỗ
 */
uint16_t tlm_frame_encode(uint8_t *dst, uint16_t dst_size,
                          uint8_t id, uint8_t seq,
                          const void *payload, uint8_t len);

/* ==========================================================================
 * Giải mã theo từng byte (dùng cho lệnh gửi từ máy tính xuống)
 * ========================================================================== */

typedef enum {
    TLM_RX_SYNC0 = 0,
    TLM_RX_SYNC1,
    TLM_RX_LEN,
    TLM_RX_ID,
    TLM_RX_SEQ,
    TLM_RX_PAYLOAD,
    TLM_RX_CRC_LO,
    TLM_RX_CRC_HI
} tlm_rx_state_t;

typedef struct {
    tlm_rx_state_t state;
    uint8_t  id;
    uint8_t  seq;
    uint8_t  len;
    uint8_t  index;
    uint16_t crc_calc;
    uint16_t crc_recv;
    uint8_t  payload[TLM_MAX_PAYLOAD];

    /* Thống kê, hữu ích khi gỡ lỗi đường truyền */
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t overruns;
} tlm_parser_t;

/** Đưa parser về trạng thái chờ byte đồng bộ. Gọi một lần lúc khởi tạo. */
void tlm_parser_init(tlm_parser_t *p);

/**
 * Nạp một byte vào parser.
 * @return true khi vừa nhận xong một khung hợp lệ (đọc p->id, p->payload,
 *         p->len). Khung sẽ bị ghi đè ở byte kế tiếp nên phải xử lý ngay.
 */
bool tlm_parser_push(tlm_parser_t *p, uint8_t byte);

#endif /* TLM_PROTOCOL_H */
