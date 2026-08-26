/**
 * @file    icm20602.h
 * @brief   Driver cho IMU 6 trục InvenSense ICM-20602 trên SPI1.
 *
 * ĐẤU NỐI (FCH743_V1.0):
 *   PA5  SPI1_SCK        PA6  SPI1_MISO       PA7  SPI1_MOSI
 *   PA4  CS   (GPIO, mức thấp là chọn chip)
 *   PC4  INT  (DRDY, EXTI4, sườn lên, ưu tiên 0)
 *
 * CÁCH HOẠT ĐỘNG:
 *   1. icm20602_init()  — cấu hình chip bằng SPI chế độ hỏi vòng (blocking),
 *      chạy ở 1 MHz cho chắc, xong thì chuyển về 8 MHz.
 *   2. icm20602_start() — cho phép ngắt DRDY.
 *   3. Mỗi khi có mẫu mới, chip kéo chân INT lên -> EXTI4 -> driver phát
 *      lệnh đọc liên tiếp 14 byte bằng DMA (không chiếm CPU).
 *   4. DMA xong -> ngắt -> tách số, đổi thang, xoay trục, lọc, rồi ghi thẳng
 *      vào g_fc.imu.
 *
 *   Toàn bộ đường dữ liệu chạy trong ngắt, vòng lặp chính chỉ việc đọc
 *   g_fc.imu. Không có hàm nào chặn (blocking) sau khi init xong.
 *
 * GIỚI HẠN PHẦN CỨNG:
 *   ICM-20602 chịu được SPI tối đa 10 MHz cho mọi thanh ghi. Cấu hình hiện
 *   tại là 8 MHz (kernel PLL1Q 64 MHz chia 8) — nằm trong giới hạn.
 */
#ifndef ICM20602_H
#define ICM20602_H

#include "fc_types.h"
#include "fc_config.h"

/* ==========================================================================
 * Bản đồ thanh ghi (trích phần dùng tới)
 * ========================================================================== */

#define ICM_REG_SMPLRT_DIV        0x19u
#define ICM_REG_CONFIG            0x1Au
#define ICM_REG_GYRO_CONFIG       0x1Bu
#define ICM_REG_ACCEL_CONFIG      0x1Cu
#define ICM_REG_ACCEL_CONFIG2     0x1Du
#define ICM_REG_FIFO_EN           0x23u
#define ICM_REG_INT_PIN_CFG       0x37u
#define ICM_REG_INT_ENABLE        0x38u
#define ICM_REG_INT_STATUS        0x3Au
#define ICM_REG_ACCEL_XOUT_H      0x3Bu   /**< đầu khối 14 byte dữ liệu */
#define ICM_REG_TEMP_OUT_H        0x41u
#define ICM_REG_GYRO_XOUT_H       0x43u
#define ICM_REG_SIGNAL_PATH_RESET 0x68u
#define ICM_REG_USER_CTRL         0x6Au
#define ICM_REG_PWR_MGMT_1        0x6Bu
#define ICM_REG_PWR_MGMT_2        0x6Cu
#define ICM_REG_WHO_AM_I          0x75u

#define ICM_WHO_AM_I_VALUE        0x12u

/** Bit đọc: địa chỉ thanh ghi phải OR với 0x80 khi muốn đọc. */
#define ICM_SPI_READ_BIT          0x80u

/** Số byte đọc liên tiếp: accel(6) + temp(2) + gyro(6). */
#define ICM_BURST_DATA_LEN        14u

/* ==========================================================================
 * Trạng thái driver
 * ========================================================================== */

typedef enum {
    ICM_STATE_UNINIT = 0,   /**< chưa gọi init                        */
    ICM_STATE_IDLE,         /**< init xong, chưa bật ngắt DRDY        */
    ICM_STATE_RUNNING,      /**< đang đọc dữ liệu liên tục            */
    ICM_STATE_CALIBRATING,  /**< đang lấy bias gyro                   */
    ICM_STATE_ERROR         /**< lỗi giao tiếp, không dùng được       */
} icm20602_state_t;

/* ==========================================================================
 * API
 * ========================================================================== */

/**
 * Đặt lại và cấu hình cảm biến. Hàm CHẶN khoảng 150 ms.
 * Gọi sau MX_SPI1_Init() và fc_time_init().
 * @return true nếu WHO_AM_I đúng và mọi thanh ghi ghi thành công.
 */
bool icm20602_init(void);

/** Cho phép ngắt DRDY, bắt đầu luồng dữ liệu. */
void icm20602_start(void);

/** Tạm dừng luồng dữ liệu (ví dụ trước khi ghi cấu hình lại). */
void icm20602_stop(void);

/**
 * Bắt đầu lấy bias gyro. Không chặn — quá trình diễn ra trong ngắt và mất
 * khoảng IMU_CALIB_SAMPLE_COUNT / IMU_SAMPLE_RATE_HZ giây (mặc định 250 ms).
 * Máy bay phải nằm yên; nếu phát hiện rung quá IMU_CALIB_MOVE_LIMIT_DPS thì
 * quá trình tự khởi động lại từ đầu.
 */
void icm20602_start_gyro_calibration(void);

/** Trạng thái hiện tại của driver. */
icm20602_state_t icm20602_get_state(void);

/** Phần trăm tiến độ hiệu chuẩn (0..100). */
uint8_t icm20602_calibration_progress(void);

/* --- Hàm gọi từ ngắt, xem App/Drivers/drv_hal_callbacks.c --------------- */

/** Do lech chuan gyro cua cua so 1 giay vua xong, do/giay.
 *  Cong cu cua giai doan 2 - xem App/Common/imu_noise.h. */
float   icm20602_gyro_sigma_dps(void);
vec3f_t icm20602_gyro_sigma_axes_dps(void);

/* --- Ham goi tu ngat --------------------------------------------------- */

/** Gọi khi chân DRDY (PC4) có sườn lên. */
void icm20602_drdy_isr(void);

/** Gọi khi DMA của SPI1 truyền nhận xong. */
void icm20602_spi_complete_isr(void);

/** Gọi khi SPI1 báo lỗi. */
void icm20602_spi_error_isr(void);

#endif /* ICM20602_H */
