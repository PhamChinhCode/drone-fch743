/**
 * @file    mtf01p.h
 * @brief   Driver cho MicoAir MTF-01P — optical flow + đo khoảng cách laser.
 *
 * ĐẤU NỐI (FCH743_V1.0):
 *   PA0  UART4_TX  (không dùng — cảm biến tự phát, không cần hỏi)
 *   PA1  UART4_RX  <- chân TX của MTF-01P
 *   Cấp nguồn 5 V và GND chung.
 *
 * GIAO THỨC:
 *   Mặc định MTF-01P xuất **MSP V2** ở 115200 8N1, hai bản tin lặp lại:
 *     0x1F01  MSP2_SENSOR_RANGEFINDER   payload 5 byte
 *     0x1F02  MSP2_SENSOR_OPTIC_FLOW    payload 9 byte
 *
 *   Khung MSP V2:
 *     '$' 'X' '<' flag func_lo func_hi size_lo size_hi payload... crc8
 *   CRC8 kiểu DVB-S2 (đa thức 0xD5) tính từ byte `flag` tới hết payload.
 *
 *   Dòng MTF-01 còn cấu hình được sang MAVLink bằng phần mềm của hãng. Nếu
 *   parser không ra dữ liệu, bật DBG_MODE_FLOW_RAW trên console để xem byte
 *   thô cảm biến đang gửi rồi đối chiếu lại.
 *
 * CÁCH HOẠT ĐỘNG:
 *   UART4 chạy DMA vòng tròn liên tục. mtf01p_update() gọi từ vòng lặp chính
 *   sẽ rút byte mới ra khỏi đệm và nạp vào bộ phân tích. Không dùng ngắt nào
 *   ngoài ngắt lỗi UART, nên không có rủi ro tranh chấp dữ liệu.
 *
 *   Đệm 256 byte ứng với ~160 ms dữ liệu (100 Hz × ~16 byte/gói), vòng lặp
 *   chính chỉ cần gọi update nhanh hơn mức đó là không mất byte nào.
 */
#ifndef MTF01P_H
#define MTF01P_H

#include "fc_types.h"
#include "fc_config.h"

/** Mã bản tin MSP V2 mà cảm biến phát ra. */
#define MSP2_SENSOR_RANGEFINDER   0x1F01u
#define MSP2_SENSOR_OPTIC_FLOW    0x1F02u

/** Số byte thô giữ lại để soi bằng console khi cần dò giao thức. */
#define MTF01P_RAW_SNAPSHOT_LEN   32u

/**
 * Khởi động UART4 ở chế độ DMA vòng tròn.
 * Gọi sau MX_UART4_Init() và fc_time_init().
 * @return false nếu HAL từ chối khởi động DMA.
 */
bool mtf01p_init(void);

/**
 * Rút dữ liệu mới từ đệm DMA và phân tích. Gọi đều đặn trong vòng lặp chính
 * (tối thiểu 20 Hz, càng nhanh càng tốt). Cập nhật thẳng vào g_fc.flow.
 * @return số khung MSP hợp lệ vừa xử lý xong
 */
uint8_t mtf01p_update(void);

/* --- Thống kê phục vụ chẩn đoán ---------------------------------------- */

uint32_t mtf01p_bytes_received(void);
uint32_t mtf01p_frames_ok(void);
uint32_t mtf01p_crc_errors(void);

/**
 * Chép ra các byte thô nhận gần nhất, dùng cho chế độ hexdump của console.
 * @return số byte đã chép (tối đa MTF01P_RAW_SNAPSHOT_LEN)
 */
uint8_t mtf01p_peek_raw(uint8_t *dst, uint8_t max_len);

/** Gọi khi UART4 báo lỗi — driver sẽ khởi động lại DMA. */
void mtf01p_uart_error_isr(void);

#endif /* MTF01P_H */
