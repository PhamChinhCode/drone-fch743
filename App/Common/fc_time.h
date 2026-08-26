/**
 * @file    fc_time.h
 * @brief   Bộ đếm thời gian độ phân giải micro giây, dựa trên TIM2.
 *
 * TIM2 là timer 32-bit trên APB1 (clock 240 MHz), CubeMX đã đặt
 * Prescaler = 239 nên mỗi tick đúng 1 µs, và Period = 0xFFFFFFFF nên bộ đếm
 * chạy tự do, tràn sau khoảng 71,6 phút.
 *
 * micros() đọc thẳng thanh ghi TIM2->CNT, không gọi HAL, nên đủ nhẹ để dùng
 * trong ISR ở tần số 8 kHz.
 *
 * CẢNH BÁO VỀ TRÀN SỐ:
 *   Không bao giờ so sánh hai mốc thời gian bằng '<' hay '>'. Luôn dùng
 *   fc_elapsed_us(now, past) trong fc_types.h — phép trừ unsigned cho kết quả
 *   đúng kể cả khi bộ đếm vừa tràn, miễn là khoảng cách dưới 71 phút.
 */
#ifndef FC_TIME_H
#define FC_TIME_H

#include "fc_types.h"
#include "stm32h7xx.h"

/** Khởi động TIM2. Gọi một lần sau MX_TIM2_Init(). */
void fc_time_init(void);

/** Số micro giây kể từ khi fc_time_init() được gọi (tràn sau ~71,6 phút). */
static inline uint32_t micros(void)
{
    return TIM2->CNT;
}

/** Số mili giây kể từ lúc khởi động, lấy từ SysTick của HAL. */
uint32_t millis(void);

/** Chờ bận trong khoảng micro giây. Chỉ dùng lúc khởi tạo, không dùng trong ISR. */
void delay_us(uint32_t us);

#endif /* FC_TIME_H */
