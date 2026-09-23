/**
 * @file    icm42688.h
 * @brief   Driver cho IMU 6 trục TDK InvenSense ICM-42688-P trên SPI1.
 *
 * Thay cho ICM-20602 cùng vị trí, cùng chân:
 *   PA5  SPI1_SCK        PA6  SPI1_MISO       PA7  SPI1_MOSI
 *   PA4  CS   (GPIO, mức thấp là chọn chip)
 *   PC4  INT1 (DRDY, EXTI4, sườn lên, ưu tiên 0)
 *
 * CÁCH HOẠT ĐỘNG — giữ nguyên khuôn của driver ICM-20602:
 *   1. icm42688_init()  — cấu hình chip bằng SPI hỏi vòng ở 1 MHz, xong thì
 *      chuyển về 8 MHz.
 *   2. icm42688_start() — cho phép xử lý ngắt DRDY.
 *   3. Mỗi mẫu mới, chip phát xung trên INT1 -> EXTI4 -> đọc liên tiếp 14 byte
 *      bằng DMA.
 *   4. DMA xong -> tách số, đổi thang, xoay trục, lọc, ghi vào g_fc.imu.
 *
 * KHÁC BIỆT SO VỚI ICM-20602 — những chỗ dễ sai:
 *   - WHO_AM_I = 0x47 (ICM-20602 là 0x12).
 *   - Khối dữ liệu bắt đầu từ TEMP_DATA1 (0x1D): nhiệt(2) accel(6) gyro(6).
 *     Cùng 14 byte nhưng NHIỆT ĐỘ ĐỨNG ĐẦU, không nằm giữa như ICM-20602.
 *   - Mã dải đo NGƯỢC chiều: FS_SEL = 0 là dải LỚN NHẤT (±2000 °/s, ±16 g).
 *   - Nhiệt độ: T = raw / 132,48 + 25.
 *   - Thanh ghi chia thành nhiều bank (REG_BANK_SEL = 0x76). Bộ lọc chống
 *     răng cưa (AAF) nằm ở bank 1 (gyro) và bank 2 (accel).
 *   - Ở ODR >= 4 kHz datasheet bắt buộc INT_TPULSE_DURATION = 8 µs và
 *     INT_TDEASSERT_DISABLE = 1, đồng thời phải xoá INT_ASYNC_RESET.
 *
 * GIỚI HẠN PHẦN CỨNG:
 *   SPI tối đa 24 MHz. Giữ 8 MHz như trước — đủ nhanh (15 byte ~ 15 µs) và
 *   không phải đổi gì trong CubeMX.
 */
#ifndef ICM42688_H
#define ICM42688_H

#include "fc_types.h"
#include "fc_config.h"

/* ==========================================================================
 * Bản đồ thanh ghi (trích phần dùng tới)
 * ========================================================================== */

/* --- Bank 0 --- */
#define ICM42_REG_DEVICE_CONFIG       0x11u
#define ICM42_REG_INT_CONFIG          0x14u
#define ICM42_REG_FIFO_CONFIG         0x16u
#define ICM42_REG_TEMP_DATA1          0x1Du   /**< đầu khối 14 byte dữ liệu */
#define ICM42_REG_INT_STATUS          0x2Du
#define ICM42_REG_INTF_CONFIG0        0x4Cu
#define ICM42_REG_INTF_CONFIG1        0x4Du
#define ICM42_REG_PWR_MGMT0           0x4Eu
#define ICM42_REG_GYRO_CONFIG0        0x4Fu
#define ICM42_REG_ACCEL_CONFIG0       0x50u
#define ICM42_REG_GYRO_ACCEL_CONFIG0  0x52u
#define ICM42_REG_INT_CONFIG1         0x64u
#define ICM42_REG_INT_SOURCE0         0x65u
#define ICM42_REG_WHO_AM_I            0x75u
#define ICM42_REG_BANK_SEL            0x76u   /**< có mặt ở mọi bank */

/* --- Bank 1 — AAF gyro --- */
#define ICM42_REG_GYRO_CONFIG_STATIC2 0x0Bu
#define ICM42_REG_GYRO_CONFIG_STATIC3 0x0Cu
#define ICM42_REG_GYRO_CONFIG_STATIC4 0x0Du
#define ICM42_REG_GYRO_CONFIG_STATIC5 0x0Eu

/* --- Bank 2 — AAF accel --- */
#define ICM42_REG_ACCEL_CONFIG_STATIC2 0x03u
#define ICM42_REG_ACCEL_CONFIG_STATIC3 0x04u
#define ICM42_REG_ACCEL_CONFIG_STATIC4 0x05u

#define ICM42_WHO_AM_I_VALUE          0x47u

/** Bit đọc: địa chỉ thanh ghi phải OR với 0x80 khi muốn đọc. */
#define ICM42_SPI_READ_BIT            0x80u

/** Số byte đọc liên tiếp: temp(2) + accel(6) + gyro(6). */
#define ICM42_BURST_DATA_LEN          14u

/* ==========================================================================
 * Trạng thái driver
 * ========================================================================== */

typedef enum {
    ICM_STATE_UNINIT = 0,   /**< chưa gọi init                        */
    ICM_STATE_IDLE,         /**< init xong, chưa bật ngắt DRDY        */
    ICM_STATE_RUNNING,      /**< đang đọc dữ liệu liên tục            */
    ICM_STATE_CALIBRATING,  /**< đang lấy bias gyro                   */
    ICM_STATE_ERROR         /**< lỗi giao tiếp, không dùng được       */
} icm42688_state_t;

/* ==========================================================================
 * API — cùng hình dạng với driver ICM-20602 cũ
 * ========================================================================== */

/**
 * Đặt lại và cấu hình cảm biến. Hàm CHẶN khoảng 100 ms.
 * Gọi sau MX_SPI1_Init() và fc_time_init().
 * @return true nếu WHO_AM_I đúng và mọi thanh ghi ghi thành công.
 */
bool icm42688_init(void);

/** Cho phép ngắt DRDY, bắt đầu luồng dữ liệu. */
void icm42688_start(void);

/** Tạm dừng luồng dữ liệu. */
void icm42688_stop(void);

/**
 * Bắt đầu lấy bias gyro. Không chặn — diễn ra trong ngắt. Máy bay phải nằm
 * yên; rung quá imu_calib_move_sd_dps thì tự khởi động lại từ đầu.
 */
void icm42688_start_gyro_calibration(void);

/** Trạng thái hiện tại của driver. */
icm42688_state_t icm42688_get_state(void);

/** Phần trăm tiến độ hiệu chuẩn (0..100). */
uint8_t icm42688_calibration_progress(void);

/** WHO_AM_I đọc được lúc init — để in ra khi init thất bại. */
uint8_t icm42688_who_am_i(void);

/** Do lech chuan gyro cua cua so 1 giay vua xong, do/giay.
 *  Xem App/Common/imu_noise.h. */
float   icm42688_gyro_sigma_dps(void);
vec3f_t icm42688_gyro_sigma_axes_dps(void);

/* --- Hàm gọi từ ngắt, xem App/Drivers/drv_hal_callbacks.c --------------- */

/** Gọi khi chân INT1 (PC4) có sườn lên. */
void icm42688_drdy_isr(void);

/** Gọi khi DMA của SPI1 truyền nhận xong. */
void icm42688_spi_complete_isr(void);

/** Gọi khi SPI1 báo lỗi. */
void icm42688_spi_error_isr(void);

#endif /* ICM42688_H */
