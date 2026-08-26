/**
 * @file    tlm_port.h
 * @brief   Lớp truyền tải cho telemetry — USART3 (DMA) hoặc USB CDC.
 *
 * Tách riêng khỏi tlm_stream.c để sau này thêm đường truyền mới (ví dụ gửi
 * kèm vào khung CRSF về tay điều khiển) mà không phải sửa phần đóng gói.
 *
 * ĐƯỜNG TRUYỀN HIỆN TẠI:
 *   USART3 (PD8 = TX, PD9 = RX) @921600 -> ESP32 "air" gắn trên máy bay
 *   -> ESP-NOW 2.4 GHz -> ESP32 "ground" cắm USB ở máy tính -> phần mềm PC.
 *   Cặp ESP32 hoàn toàn trong suốt: chúng chỉ gom byte thành gói rồi trả lại
 *   nguyên vẹn, nên firmware không cần biết có chúng ở giữa.
 *
 * BỘ ĐỆM GỬI:
 *   Dữ liệu được chép vào một ring buffer tĩnh nằm trong .bss (AXI SRAM
 *   0x24000000) rồi mới đẩy đi bằng DMA. Bắt buộc phải qua bước chép này vì
 *   khung gói tin do tlm_stream.c dựng nằm trên stack, mà stack thuộc DTCMRAM
 *   — vùng DMA1/DMA2 không truy cập được.
 *
 * LUỒNG NGẮT:
 *   tlm_port_write() có thể gọi từ vòng lặp chính. Việc nạp DMA tiếp theo
 *   diễn ra trong HAL_UART_TxCpltCallback (ưu tiên 10), nên vùng cập nhật
 *   con trỏ ring buffer được bảo vệ bằng khoá ngắt ngắn.
 */
#ifndef TLM_PORT_H
#define TLM_PORT_H

#include "fc_types.h"
#include "fc_config.h"

typedef enum {
    TLM_PORT_NONE = 0,
    TLM_PORT_UART,      /**< USART3, 921600 baud, TX DMA1_S7 / RX DMA1_S6 */
    TLM_PORT_USB        /**< USB CDC (cổng COM ảo)                       */
} tlm_port_type_t;

/** Chọn và khởi tạo đường truyền. Gọi sau MX_USART3_UART_Init(). */
void tlm_port_init(tlm_port_type_t port);

/** Đổi đường truyền lúc đang chạy (ví dụ khi phát hiện cắm USB). */
void tlm_port_set(tlm_port_type_t port);

tlm_port_type_t tlm_port_get(void);

/**
 * Đưa dữ liệu vào hàng đợi gửi.
 * @return false nếu bộ đệm không còn đủ chỗ (gói bị bỏ, không gửi một phần).
 */
bool tlm_port_write(const uint8_t *data, uint16_t len);

/** Kích hoạt truyền nếu đang rảnh. Gọi đều đặn trong vòng lặp chính. */
void tlm_port_flush(void);

/** Số byte còn trống trong bộ đệm gửi. */
uint16_t tlm_port_tx_free(void);

/** Số gói đã bị bỏ do đầy đệm — theo dõi để biết đang phát quá tải. */
uint32_t tlm_port_dropped(void);

/**
 * Đọc dữ liệu nhận được (uplink).
 * @return số byte đã lấy ra, 0 nếu chưa có gì.
 */
uint16_t tlm_port_read(uint8_t *dst, uint16_t max_len);

/** Gọi từ HAL_UART_TxCpltCallback khi huart == &huart3. */
void tlm_port_tx_complete_isr(void);

/**
 * Gọi từ HAL_UART_ErrorCallback khi huart == &huart3.
 * Lỗi khung / tràn đệm khiến HAL huỷ DMA RX; không khởi động lại thì đường
 * uplink đứng vĩnh viễn mà không báo gì.
 */
void tlm_port_uart_error_isr(void);

#endif /* TLM_PORT_H */
