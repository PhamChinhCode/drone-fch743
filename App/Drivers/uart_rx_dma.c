/**
 * @file    uart_rx_dma.c
 * @brief   Tự chữa UART nhận bằng DMA vòng tròn. Xem uart_rx_dma.h.
 */

#include "uart_rx_dma.h"

/*
 * Stream dừng sau byte đang chuyển dở, cỡ vài chu kỳ bus. Đặt trần chỉ để
 * phần cứng có hỏng cũng không treo vòng lặp chính trong lúc khoá ngắt.
 */
#define STREAM_STOP_SPIN_MAX  1000u

bool uart_rx_dma_heal(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t size)
{
    if (huart->RxState != HAL_UART_STATE_READY) {
        return false;                       /* đang nhận bình thường */
    }

    /*
     * Khoá ngắt cả đoạn: ngắt UART/DMA chen vào giữa chính là thứ gây ra
     * trạng thái nửa vời ở trường hợp 2 trong uart_rx_dma.h.
     */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    bool restarted = false;

    if (huart->RxState == HAL_UART_STATE_READY) {
        DMA_HandleTypeDef  *dma    = huart->hdmarx;
        DMA_Stream_TypeDef *stream = (DMA_Stream_TypeDef *)dma->Instance;

        /*
         * Dừng hẳn, bất kể HAL đang tưởng DMA ở trạng thái nào: kẹt ABORT
         * (trường hợp 1) hay vẫn BUSY và đang chạy (trường hợp 2). Cờ ngắt
         * của stream thì HAL_DMA_Start_IT tự xoá.
         */
        ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);
        __HAL_DMA_DISABLE(dma);
        for (uint32_t i = 0; (stream->CR & DMA_SxCR_EN) != 0U && i < STREAM_STOP_SPIN_MAX; i++) {
        }

        if ((stream->CR & DMA_SxCR_EN) == 0U) {
            dma->State = HAL_DMA_STATE_READY;
            __HAL_UNLOCK(dma);

            /*
             * ORE còn treo thì vừa bật lại EIE là ngắt lỗi bắn ngay. Byte cũ
             * trong RDR cũng bỏ luôn, kẻo DMA chép nó vào đầu đệm mới.
             */
            __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_FEF |
                                         UART_CLEAR_NEF | UART_CLEAR_PEF);
            __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);

            restarted = (HAL_UART_Receive_DMA(huart, buf, size) == HAL_OK);
        }
    }

    __set_PRIMASK(primask);
    return restarted;
}
