/**
 * @file    drv_hal_callbacks.c
 * @brief   Nơi tập trung toàn bộ hàm callback của HAL.
 *
 * VÌ SAO GOM VỀ MỘT CHỖ:
 *   Các hàm HAL_xxx_Callback trong thư viện ST khai báo `weak`, mỗi hàm chỉ
 *   được định nghĩa đúng một lần trong cả chương trình. Nếu để rải rác ở
 *   từng driver thì hai driver dùng chung một loại ngoại vi sẽ xung đột lúc
 *   liên kết. Gom hết vào đây rồi phân phối theo instance là cách gọn nhất.
 *
 *   Nhờ file này mà Core/Src/main.c KHÔNG cần thêm dòng nào trong khối
 *   USER CODE để xử lý ngắt — CubeMX generate lại bao nhiêu lần cũng không
 *   ảnh hưởng.
 *
 * THÊM THIẾT BỊ MỚI:
 *   Chỉ cần thêm một nhánh `if` vào hàm tương ứng. Ví dụ khi có driver
 *   CRSF trên USART2:
 *
 *       void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t n)
 *       {
 *           if (huart->Instance == USART2) { crsf_rx_event_isr(n); }
 *       }
 *
 * LƯU Ý VỀ ƯU TIÊN NGẮT:
 *   Các hàm dưới đây chạy trong ngữ cảnh ngắt. Tuyệt đối không gọi
 *   HAL_Delay() hay bất kỳ API HAL nào có timeout — SysTick ở mức ưu tiên 15
 *   (thấp nhất) nên sẽ bị chặn và chương trình treo cứng.
 */

#include "main.h"
#include "icm20602.h"
#include "lsm6dsv.h"
#include "bmp388.h"
#include "mtf01p.h"
#include "crsf.h"
#include "dshot.h"
#include "tlm_port.h"
#include "dbg_console.h"

/* ==========================================================================
 * GPIO — ngắt ngoài
 * ========================================================================== */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    /* PC4 — chân DRDY của ICM20602, sườn lên, ưu tiên 0. */
    if (GPIO_Pin == SPI1_INT_Pin) {
        icm20602_drdy_isr();
    }

    /* PD7 - chan DRDY cua LSM6DSV (IMU phu), suon len, uu tien 4.
     * Chung vector EXTI9_5 voi EXTI5..EXTI9; hien chi PD7 dung. */
    if (GPIO_Pin == SPI3_INT_Pin) {
        lsm6dsv_drdy_isr();
    }
}

/* ==========================================================================
 * SPI
 * ========================================================================== */

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        icm20602_spi_complete_isr();
    }
    if (hspi->Instance == SPI3) {
        lsm6dsv_spi_complete_isr();
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        icm20602_spi_error_isr();
    }
    if (hspi->Instance == SPI3) {
        lsm6dsv_spi_error_isr();
    }
}

/* ==========================================================================
 * TIM
 * ========================================================================== */

/*
 * Với DMA burst trên TIM_DMA_UPDATE, HAL báo "truyền xong" qua đúng hàm này
 * chứ không phải HAL_TIM_PWM_PulseFinishedCallback — xem TIM_DMAPeriodElapsedCplt
 * trong stm32h7xx_hal_tim.c.
 *
 * Hàm này cũng là callback của ngắt tràn timer thông thường, nên phải lọc
 * theo instance. TIM2 (fc_time) chạy bằng HAL_TIM_Base_Start không ngắt nên
 * không bao giờ vào đây.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* TIM1 — DShot600 phát xong một khung cho cả 4 motor. */
    if (htim->Instance == TIM1) {
        dshot_dma_complete_isr();
    }
}

void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        dshot_dma_error_isr();
    }
}

/* ==========================================================================
 * I2C
 * ========================================================================== */

/*
 * Callback của HAL_I2C_Mem_Read_IT — đúng hàm này chứ không phải
 * HAL_I2C_MasterRxCpltCallback, vì lượt truyền có pha ghi địa chỉ thanh ghi
 * rồi mới lặp lại START để đọc.
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    /* I2C1 — BMP388. Chưa có thiết bị I2C nào khác trên bus này. */
    if (hi2c->Instance == I2C1) {
        bmp388_i2c_complete_isr();
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        bmp388_i2c_error_isr();
    }
}

/* ==========================================================================
 * UART
 * ========================================================================== */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /*
     * Console dạng chữ được hỏi trước vì nó nhận handle lúc chạy (có thể là
     * USART1, USART3 hay bất kỳ UART nào). Hai module không được trỏ vào cùng
     * một UART — nếu trùng, console thắng và telemetry nhị phân sẽ đứng.
     */
    if (dbg_console_owns(huart)) {
        dbg_console_tx_complete_isr();
        return;
    }

    /* USART3 — telemetry nhị phân -> ESP32 air -> ESP-NOW -> máy tính. */
    if (huart->Instance == USART3) {
        tlm_port_tx_complete_isr();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /*
     * UART4 — MTF-01P. Lỗi khung hoặc tràn đệm khiến HAL huỷ DMA; phải khởi
     * động lại, nếu không luồng dữ liệu đứng vĩnh viễn mà không báo gì.
     */
    if (huart->Instance == UART4) {
        mtf01p_uart_error_isr();
    }

    /*
     * USART2 — máy thu ExpressLRS. Cùng lý do như trên, và ở đây còn gấp hơn:
     * DMA đứng nghĩa là mất đường điều khiển.
     */
    if (huart->Instance == USART2) {
        crsf_uart_error_isr();
    }

    /*
     * USART3 — telemetry. Lỗi ở đây không ảnh hưởng chuyến bay, nhưng nếu
     * không khởi động lại RX DMA thì đường lệnh từ máy tính im luôn. Hay gặp
     * nhất là lúc cắm/rút nguồn ESP32 làm chân TX của nó thả nổi một nhịp.
     */
    if (huart->Instance == USART3) {
        tlm_port_uart_error_isr();
    }
}
