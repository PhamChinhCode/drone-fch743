/**
 * @file    crsf.h
 * @brief   Driver nhận CRSF từ máy thu ExpressLRS trên USART2.
 *
 * ĐẤU NỐI (FCH743_V1.0):
 *   PA2  USART2_TX  -> chân RX của máy thu (chỉ dùng khi gửi telemetry ngược)
 *   PA3  USART2_RX  <- chân TX của máy thu
 *   Cấp nguồn 5 V và GND chung.
 *
 *   CHÚ Ý dây chéo: TX của mạch nối vào RX của máy thu và ngược lại. Nối
 *   thẳng TX-TX là lỗi thường gặp nhất, console sẽ không thấy byte nào.
 *
 * GIAO THỨC:
 *   CRSF chạy 420000 8N1 (đã đặt sẵn trong MX_USART2_UART_Init).
 *   Khung:
 *     [địa chỉ] [độ dài] [kiểu] [payload...] [crc8]
 *   `độ dài` đếm từ byte `kiểu` tới hết crc, nên tổng khung = độ dài + 2.
 *   CRC8 kiểu DVB-S2 (đa thức 0xD5) tính từ byte `kiểu` tới hết payload.
 *
 *   Hai khung driver này quan tâm:
 *     0x16  RC_CHANNELS_PACKED   22 byte — 16 kênh, mỗi kênh 11 bit
 *     0x14  LINK_STATISTICS      10 byte — RSSI, chất lượng đường truyền
 *   Các khung khác (telemetry, tham số) được kiểm CRC rồi bỏ qua.
 *
 *   Vài bản ELRS đời cũ chạy 400000 thay vì 420000. Nếu console ở
 *   DBG_MODE_RC_RAW thấy byte chạy nhưng lệch lung tung, đổi
 *   huart2.Init.BaudRate trong CubeMX rồi thử lại.
 *
 * CÁCH HOẠT ĐỘNG:
 *   USART2 chạy DMA vòng tròn liên tục (DMA1_Stream3). crsf_update() gọi từ
 *   vòng lặp chính sẽ rút byte mới ra khỏi đệm và nạp vào bộ phân tích.
 *   Không dùng ngắt nào ngoài ngắt lỗi UART, nên không có rủi ro tranh chấp
 *   dữ liệu — cùng một mô hình với driver MTF-01P.
 *
 * FAILSAFE:
 *   ELRS mất sóng thì đơn giản là ngừng phát, không có cờ báo riêng. Quá
 *   RC_FAILSAFE_TIMEOUT_MS không có khung hợp lệ thì driver bật cờ failsafe,
 *   ĐỒNG THỜI ép ga về 0 và ba trục về giữa. Giữ nguyên giá trị cần cũ trong
 *   lúc mất sóng là cách nhanh nhất để máy bay lao đi mất kiểm soát.
 *
 * CHƯA LÀM:
 *   Gửi telemetry ngược về tay điều khiển (điện áp pin, GPS, chế độ bay).
 *   CRSF dùng chung một đường cho cả hai chiều, muốn thêm thì phát khung
 *   0x08 / 0x02 ra PA2 vào đúng khe thời gian giữa hai khung nhận được.
 */
#ifndef CRSF_H
#define CRSF_H

#include "fc_types.h"
#include "fc_config.h"

/* ==========================================================================
 * Hằng số giao thức
 * ========================================================================== */

/** Địa chỉ thiết bị đứng đầu mỗi khung. */
#define CRSF_ADDR_BROADCAST         0x00u
#define CRSF_ADDR_FLIGHT_CONTROLLER 0xC8u
#define CRSF_ADDR_RADIO_TRANSMITTER 0xEAu
#define CRSF_ADDR_RECEIVER          0xECu
#define CRSF_ADDR_TRANSMITTER       0xEEu

/** Kiểu khung. */
#define CRSF_FRAMETYPE_GPS                0x02u
#define CRSF_FRAMETYPE_BATTERY_SENSOR     0x08u
#define CRSF_FRAMETYPE_LINK_STATISTICS    0x14u
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16u
#define CRSF_FRAMETYPE_ATTITUDE           0x1Eu
#define CRSF_FRAMETYPE_FLIGHT_MODE        0x21u

/** Kích thước khung: byte `độ dài` chỉ nhận giá trị trong khoảng này. */
#define CRSF_FRAME_LEN_MIN          2u
#define CRSF_FRAME_LEN_MAX          62u

/** Số byte payload của hai khung driver xử lý. */
#define CRSF_RC_CHANNELS_PAYLOAD    22u
#define CRSF_LINK_STATS_PAYLOAD     10u

/** Số byte thô giữ lại để soi bằng console khi cần dò giao thức. */
#define CRSF_RAW_SNAPSHOT_LEN       32u

/* ==========================================================================
 * API
 * ========================================================================== */

/**
 * Khởi động USART2 ở chế độ DMA vòng tròn.
 * Gọi sau MX_USART2_UART_Init() và fc_time_init().
 * @return false nếu HAL từ chối khởi động DMA.
 */
bool crsf_init(void);

/**
 * Rút dữ liệu mới từ đệm DMA và phân tích. Gọi đều đặn trong vòng lặp chính
 * (tối thiểu 100 Hz, càng nhanh càng tốt). Cập nhật thẳng vào g_fc.rc.
 * @return số khung CRSF hợp lệ vừa xử lý xong
 */
uint8_t crsf_update(void);

/* --- Thống kê phục vụ chẩn đoán ---------------------------------------- */

uint32_t crsf_bytes_received(void);
uint32_t crsf_frames_ok(void);
uint32_t crsf_crc_errors(void);

/** Số khung RC_CHANNELS_PACKED nhận được — dùng để tính tần số khung thực tế. */
uint32_t crsf_rc_frames(void);

/** Chu kỳ giữa hai khung RC gần nhất, micro giây (500 Hz -> ~2000). */
uint32_t crsf_frame_interval_us(void);

/**
 * Chép ra các byte thô nhận gần nhất, dùng cho chế độ hexdump của console.
 * @return số byte đã chép (tối đa CRSF_RAW_SNAPSHOT_LEN)
 */
uint8_t crsf_peek_raw(uint8_t *dst, uint8_t max_len);

/** Gọi khi USART2 báo lỗi — driver sẽ khởi động lại DMA. */
void crsf_uart_error_isr(void);

#endif /* CRSF_H */
