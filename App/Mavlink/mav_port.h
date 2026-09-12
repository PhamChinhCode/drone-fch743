/**
 * @file    mav_port.h
 * @brief   Lớp truyền tải MAVLink — UART8 (PE0/PE1) tới máy tính nhúng ROS2.
 *
 * Tách khỏi mav_link.c theo đúng cách tlm_port tách khỏi tlm_stream: file này
 * chỉ biết byte, không biết khung MAVLink là gì.
 *
 * ĐƯỜNG TRUYỀN:
 *   UART8  PE1 = TX -> RX của máy tính nhúng
 *          PE0 = RX <- TX của máy tính nhúng
 *   921600 baud, 8N1, không bắt tay phần cứng. Dây board-to-board, cả hai đầu
 *   đều 3,3 V nên nối thẳng, chỉ cần chung GND.
 *
 * VÌ SAO KHÔNG DÙNG CHUNG USART3 VỚI TELEMETRY CŨ:
 *   USART3 đang chở khung nhị phân riêng của dự án tới ESP32. Trộn hai giao
 *   thức trên một dây thì cả hai parser đều phải bỏ qua rác của bên kia — làm
 *   được nhưng vô ích khi UART8 còn trống nguyên.
 *
 * BỘ ĐỆM GỬI:
 *   Ring buffer tĩnh trong .dma_buffer (AXI SRAM 0x24000000). Bắt buộc: khung
 *   MAVLink được dựng trên stack, mà stack nằm ở DTCMRAM — vùng DMA1/DMA2
 *   không với tới được. Xem FC_DMA_BUFFER trong fc_types.h.
 *
 * LUỒNG NGẮT:
 *   mav_port_write() gọi từ vòng lặp chính. TX DMA chạy chế độ Normal nên khi
 *   xong, HAL bật cờ TCIE và HAL_UART_TxCpltCallback chạy trong NGẮT UART8
 *   (ưu tiên 10) — không phải trong ngắt DMA. Vì vậy UART8 global interrupt
 *   BẮT BUỘC phải bật trong CubeMX, nếu không đường gửi chết sau đúng một gói.
 */
#ifndef MAV_PORT_H
#define MAV_PORT_H

#include "fc_types.h"
#include "fc_config.h"

/**
 * Ring buffer gửi. 1 KB chứa được ~25 khung ATTITUDE, thừa sức hấp thụ một
 * nhịp vòng lặp bị chậm. Ở 921600 baud, làm rỗng 1 KB mất khoảng 11 ms.
 */
#define MAV_TX_BUFFER_SIZE   1024u

/** Đệm DMA vòng tròn nhận. Uplink chỉ là lệnh lác đác nên 256 byte là dư. */
#define MAV_RX_BUFFER_SIZE   256u

/** Khởi động UART8: xoá đệm, bật DMA RX vòng tròn. Gọi sau MX_UART8_Init(). */
void mav_port_init(void);

/**
 * Đưa trọn một khung vào hàng đợi gửi.
 * @return false nếu không đủ chỗ. Gói bị BỎ HẲN, không bao giờ gửi một nửa —
 *         nửa khung tới nơi làm parser đầu kia mất công đồng bộ lại.
 */
bool mav_port_write(const uint8_t *data, uint16_t len);

/** Kích DMA nếu đang rảnh. Gọi đều đặn ở vòng lặp chính. */
void mav_port_flush(void);

/** Rút byte đã nhận từ đệm DMA vòng tròn. Trả về số byte lấy được. */
uint16_t mav_port_read(uint8_t *dst, uint16_t max_len);

/** Số byte còn trống trong đệm gửi. */
uint16_t mav_port_tx_free(void);

/** Số gói đã bỏ vì đầy đệm — theo dõi để biết đang phát quá tải. */
uint32_t mav_port_dropped(void);

/** Gọi từ HAL_UART_TxCpltCallback khi huart->Instance == UART8. */
void mav_port_tx_complete_isr(void);

/**
 * Gọi từ HAL_UART_ErrorCallback khi huart->Instance == UART8.
 * Lỗi khung / tràn đệm khiến HAL huỷ DMA RX; không khởi động lại thì đường
 * uplink đứng vĩnh viễn mà không báo gì.
 */
void mav_port_error_isr(void);

#endif /* MAV_PORT_H */
