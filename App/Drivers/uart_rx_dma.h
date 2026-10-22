/**
 * @file    uart_rx_dma.h
 * @brief   Tự chữa UART nhận bằng DMA vòng tròn khi HAL làm nó chết im.
 *
 * VÌ SAO CẦN:
 *   Các driver CRSF, GPS, MTF-01P, telemetry đều nhận bằng DMA vòng tròn chạy
 *   mãi, và đều trông vào HAL_UART_ErrorCallback để khởi động lại khi có lỗi.
 *   Đo trên FC thật thì có hai đường làm RX chết mà callback đó không cứu:
 *
 *   1. Lỗi khung (cắm/rút dây, thiết bị bên kia khởi động lại): HAL tắt DMAR,
 *      gọi HAL_DMA_Abort_IT — stream dừng (EN = 0) nhưng cờ TCIF không bao giờ
 *      lên, nên pha huỷ không bao giờ xong: hdmarx kẹt State = ABORT,
 *      Lock = LOCKED, callback lỗi không bao giờ được gọi, và mọi lần
 *      HAL_UART_Receive_DMA sau đó đều thất bại. (Đo 2026-10-22 trên UART8.)
 *
 *   2. Lúc khởi động, thiết bị đã phát từ trước nên ORE treo sẵn. Trong
 *      HAL_UART_Receive_DMA, ngắt lỗi bắn ngay khi vừa bật EIE, TRƯỚC khi bật
 *      DMAR; callback khởi động lại thất bại vì DMA đang chạy, rồi HAL bật
 *      DMAR. Kết quả: DMA vẫn chạy nhưng RxState = READY, EIE tắt — HAL coi
 *      như đã thôi nhận. (Đo 2026-10-22 trên USART2, mỗi lần khởi động.)
 *
 * CÁCH DÙNG:
 *   Gọi đầu mỗi vòng update của driver (vòng lặp chính, không phải ngắt).
 *   Trả về true khi vừa khởi động lại RX — DMA ghi lại từ đầu đệm, nên người
 *   gọi phải đưa đuôi đọc về 0 và xoá bộ phân tích khung.
 */

#ifndef UART_RX_DMA_H
#define UART_RX_DMA_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * Thấy UART đã thôi nhận (RxState = READY) thì dừng hẳn DMA, gỡ khoá HAL
 * và chạy lại HAL_UART_Receive_DMA. Đang nhận bình thường thì không làm gì.
 * Chạy lại mà thất bại thì RxState vẫn READY, vòng sau tự thử tiếp.
 *
 * @return true nếu vừa chạy lại RX.
 */
bool uart_rx_dma_heal(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t size);

#endif /* UART_RX_DMA_H */
