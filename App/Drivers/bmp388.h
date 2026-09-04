/**
 * @file    bmp388.h
 * @brief   Driver cho cảm biến áp suất khí quyển Bosch BMP388 trên I2C1.
 *
 * ĐẤU NỐI (FCH743_V1.0):
 *   PB8  I2C1_SCL   ->  SCK/SCL của BMP388
 *   PB7  I2C1_SDA   ->  SDI/SDA
 *   CS   -> 3V3     chọn giao diện I2C (nối GND là SPI)
 *   SDO  -> 3V3     chọn địa chỉ 0x77 (nối GND là 0x76)
 *   VDD/VDDIO -> 3V3, GND chung.
 *
 *   CubeMX cấu hình PB7/PB8 ở chế độ GPIO_NOPULL, nên BUỘC phải có điện trở
 *   kéo lên ngoài (2,2k – 4,7k lên 3V3) trên cả hai đường. Thiếu chúng thì
 *   HAL_I2C_IsDeviceReady() luôn trả về lỗi.
 *
 * CÁCH HOẠT ĐỘNG:
 *   1. bmp388_init() — đặt lại chip, đọc 21 byte hệ số hiệu chuẩn từ NVM,
 *      ghi cấu hình đo rồi bật chế độ NORMAL. Hàm CHẶN khoảng 15 ms, chỉ
 *      chạy một lần lúc khởi động.
 *   2. bmp388_update() gọi từ vòng lặp chính. Nó phát lệnh đọc I2C ở chế độ
 *      NGẮT (I2C1 không có kênh DMA trong project này) rồi trả về ngay.
 *   3. Ngắt I2C1_EV báo truyền xong -> driver chỉ bật một cờ, KHÔNG tính
 *      toán gì trong ngắt.
 *   4. Lần gọi bmp388_update() kế tiếp sẽ bù nhiệt, bù áp, đổi ra độ cao và
 *      ghi vào g_fc.baro.
 *
 *   Phép bù của Bosch dùng đa thức bậc 3 và luỹ thừa bậc 3 của số đo thô,
 *   nên tính bằng `double` (Cortex-M7 build với fpv5-d16 nên có FPU 64-bit,
 *   chi phí không đáng kể ở 50 Hz). Để bên ngoài ngắt cho vòng lặp gyro
 *   4 kHz không bị chen ngang.
 *
 * ĐỒNG BỘ MẪU:
 *   Mỗi lần đọc lấy liền 7 byte từ thanh ghi STATUS (0x03) tới DATA_5 (0x09).
 *   Bit drdy_press trong STATUS cho biết đây có phải mẫu mới hay không, nhờ
 *   vậy không cần chân ngắt DRDY và cũng không đếm nhầm mẫu cũ hai lần.
 *   Driver hỏi vòng ở BARO_POLL_RATE_HZ (gấp đôi ODR) nên không bỏ sót mẫu.
 */
#ifndef BMP388_H
#define BMP388_H

#include "fc_types.h"
#include "fc_config.h"

/* ==========================================================================
 * Bản đồ thanh ghi (trích phần dùng tới)
 * ========================================================================== */

#define BMP388_REG_CHIP_ID        0x00u
#define BMP388_REG_ERR            0x02u
#define BMP388_REG_STATUS         0x03u
#define BMP388_REG_DATA_0         0x04u   /**< áp suất XLSB, đầu khối 6 byte */
#define BMP388_REG_EVENT          0x10u
#define BMP388_REG_INT_STATUS     0x11u
#define BMP388_REG_INT_CTRL       0x19u
#define BMP388_REG_IF_CONF        0x1Au
#define BMP388_REG_PWR_CTRL       0x1Bu
#define BMP388_REG_OSR            0x1Cu
#define BMP388_REG_ODR            0x1Du
#define BMP388_REG_CONFIG         0x1Fu
#define BMP388_REG_CALIB_00       0x31u   /**< đầu khối 21 byte hệ số NVM   */
#define BMP388_REG_CMD            0x7Eu

/** Mã nhận dạng chip. BMP390 dùng chung phép bù, chỉ khác mã này. */
#define BMP388_CHIP_ID            0x50u
#define BMP390_CHIP_ID            0x60u

/** Lệnh ghi vào thanh ghi CMD. */
#define BMP388_CMD_SOFTRESET      0xB6u
#define BMP388_CMD_FIFO_FLUSH     0xB0u

/** Bit trong thanh ghi STATUS (0x03). */
#define BMP388_STATUS_CMD_RDY     (1u << 4)
#define BMP388_STATUS_DRDY_PRESS  (1u << 5)
#define BMP388_STATUS_DRDY_TEMP   (1u << 6)

/** Bit trong thanh ghi ERR (0x02). */
#define BMP388_ERR_FATAL          (1u << 0)
#define BMP388_ERR_CMD            (1u << 1)
#define BMP388_ERR_CONF           (1u << 2)

/** Số byte hệ số hiệu chuẩn đọc từ NVM. */
#define BMP388_CALIB_LEN          21u

/** Số byte đọc mỗi chu kỳ: STATUS + áp suất(3) + nhiệt độ(3). */
#define BMP388_BURST_LEN          7u

/* ==========================================================================
 * Trạng thái driver
 * ========================================================================== */

typedef enum {
    BMP_STATE_UNINIT = 0,   /**< chưa gọi init                          */
    BMP_STATE_RUNNING,      /**< đang đo liên tục ở chế độ NORMAL       */
    BMP_STATE_CALIBRATING,  /**< đang lấy mốc áp suất mặt đất           */
    BMP_STATE_ERROR         /**< lỗi I2C hoặc sai chip, không dùng được */
} bmp388_state_t;

/* ==========================================================================
 * API
 * ========================================================================== */

/**
 * Đặt lại và cấu hình cảm biến. Hàm CHẶN khoảng 15 ms.
 * Gọi sau MX_I2C1_Init() và fc_time_init().
 * @return true nếu CHIP_ID đúng, đọc được hệ số NVM và ghi cấu hình thành công.
 */
bool bmp388_init(void);

/**
 * Xử lý mẫu vừa nhận rồi phát lệnh đọc kế tiếp. KHÔNG chặn.
 * Gọi đều đặn trong vòng lặp chính, tối thiểu BARO_POLL_RATE_HZ lần/giây.
 * Cập nhật thẳng vào g_fc.baro.
 * @return true nếu vừa xử lý xong một mẫu mới
 */
bool bmp388_update(void);

/**
 * Bắt đầu lấy mốc áp suất mặt đất. Không chặn — cộng dồn
 * BARO_CALIB_SAMPLE_COUNT mẫu (mặc định 50 mẫu ≈ 1 giây ở 50 Hz) rồi ghi
 * trung bình vào g_fc.baro.ground_pressure_pa và bật cờ calibrated.
 * Máy bay phải nằm yên trên mặt đất, tránh gió lùa và quạt gió.
 */
void bmp388_start_ground_calibration(void);

/** Đặt lại mốc áp suất mặt đất bằng đúng mẫu hiện tại (hiệu chuẩn tức thì). */
void bmp388_reset_ground_level(void);

/** Trạng thái hiện tại của driver. */
bmp388_state_t bmp388_get_state(void);

/** Phần trăm tiến độ hiệu chuẩn (0..100). */
uint8_t bmp388_calibration_progress(void);

/** Mã chip đọc được lúc init — 0x50 là BMP388, 0x60 là BMP390. */
uint8_t bmp388_chip_id(void);

/* --- Thống kê phục vụ chẩn đoán ---------------------------------------- */

uint32_t bmp388_i2c_errors(void);
uint32_t bmp388_stale_reads(void);

/**
 * Số lần bỏ lượt hỏi vòng vì từ kế đang giữ bus I2C1 dùng chung.
 * KHÔNG phải lỗi — xem ghi chú trong start_read(). Vài lần là bình thường;
 * tăng liên tục và nhanh thì hai driver đang hỏi vòng quá gần pha nhau.
 */
uint32_t bmp388_bus_lost(void);

/* --- Hàm gọi từ ngắt, xem App/Drivers/drv_hal_callbacks.c --------------- */

/** Gọi khi I2C1 đọc xong khối thanh ghi. */
void bmp388_i2c_complete_isr(void);

/** Gọi khi I2C1 báo lỗi (NACK, bus error, arbitration lost). */
void bmp388_i2c_error_isr(void);

#endif /* BMP388_H */
