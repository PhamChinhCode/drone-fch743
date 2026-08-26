/**
 * @file    fc_time.c
 */

#include "fc_time.h"
#include "main.h"

extern TIM_HandleTypeDef htim2;

void fc_time_init(void)
{
    /* Đưa bộ đếm về 0 rồi cho chạy tự do. Không bật ngắt update: bộ đếm
     * 32-bit ở 1 MHz chỉ tràn sau ~71,6 phút, và mọi phép đo khoảng cách
     * đều dùng phép trừ unsigned nên tràn không gây sai số. */
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_TIM_Base_Start(&htim2);
}

uint32_t millis(void)
{
    return HAL_GetTick();
}

void delay_us(uint32_t us)
{
    const uint32_t start = micros();
    while (fc_elapsed_us(micros(), start) < us) {
        /* chờ bận */
    }
}
