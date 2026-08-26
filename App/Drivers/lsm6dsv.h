/**
 * @file    lsm6dsv.h
 * @brief   Driver cho IMU 6 trục ST LSM6DSV trên SPI3 (IMU phụ).
 *
 * ĐẤU NỐI (FCH743_V1.0):
 *   PB3  SPI3_SCK        PB4  SPI3_MISO       PD6  SPI3_MOSI
 *   PA15 CS   (GPIO, mức thấp là chọn chip)
 *   PD7  INT1 (DRDY, EXTI9_5, sườn lên, ưu tiên 4)
 *
 * VAI TRÒ:
 *   Đây là IMU THỨ HAI. Nó ghi vào g_fc.imu2 và KHÔNG đụng tới g_fc.imu.
 *   Vòng điều khiển 4 kHz và bộ ước lượng vẫn chạy hoàn toàn bằng ICM20602.
 *   Xem App/Docs/KE_HOACH_LSM6DSV.md — giai đoạn 5 mới là lúc hợp nhất, và
 *   chỉ khi số đo ở giai đoạn 2 cho thấy đáng làm.
 *
 *   Trên module còn có từ kế QMC6309 nối vào sensor hub của chip này. Phần
 *   đó là giai đoạn 3, chưa hiện thực ở đây.
 *
 * KHÁC BIỆT SO VỚI ICM20602 — những chỗ dễ sai:
 *   - SPI chế độ 3 (CPOL=1, CPHA=1), trong khi ICM dùng chế độ 0.
 *   - Byte thấp ĐỨNG TRƯỚC (little-endian); ICM thì ngược lại.
 *   - Khối dữ liệu xếp theo thứ tự temp -> gyro -> accel; ICM là accel ->
 *     temp -> gyro.
 *   - Phải bật IF_INC trong CTRL3 thì đọc liên tiếp mới tự tăng địa chỉ.
 *   - Nhiệt độ quy đổi bằng raw/256 + 25, không phải raw/326,8 + 25.
 *
 * VÌ SAO ODR CHỈ 1920 Hz CHỨ KHÔNG PHẢI 8 kHz:
 *   Dữ liệu bị lọc xuống 100 Hz trước khi dùng nên tốc độ cao hơn vô ích,
 *   mà mỗi ngắt đều tốn CPU. Vòng PID 4 kHz là thứ không được phép chậm lại.
 *
 * NGUỒN BẢN ĐỒ THANH GHI:
 *   Driver chính thức của ST — github.com/STMicroelectronics/lsm6dsv-pid,
 *   file lsm6dsv_reg.h và lsm6dsv_reg.c. Không viết theo trí nhớ.
 */
#ifndef LSM6DSV_H
#define LSM6DSV_H

#include "fc_types.h"
#include "fc_config.h"

/* ==========================================================================
 * Bản đồ thanh ghi (trích phần dùng tới)
 * ========================================================================== */

#define LSM_REG_FUNC_CFG_ACCESS   0x01u   /**< bit6 mở bank sensor hub  */
#define LSM_REG_PIN_CTRL          0x02u
#define LSM_REG_IF_CFG            0x03u   /**< bit6 = pull-up bus I2C phụ */
#define LSM_REG_INT1_CTRL         0x0Du
#define LSM_REG_INT2_CTRL         0x0Eu
#define LSM_REG_WHO_AM_I          0x0Fu
#define LSM_REG_CTRL1             0x10u   /**< bit0-3 odr_xl, bit4-6 mode */
#define LSM_REG_CTRL2             0x11u   /**< bit0-3 odr_g,  bit4-6 mode */
#define LSM_REG_CTRL3             0x12u
#define LSM_REG_CTRL4             0x13u
#define LSM_REG_CTRL5             0x14u
#define LSM_REG_CTRL6             0x15u   /**< bit0-3 fs_g              */
#define LSM_REG_CTRL7             0x16u
#define LSM_REG_CTRL8             0x17u   /**< bit0-1 fs_xl             */
#define LSM_REG_CTRL9             0x18u
#define LSM_REG_CTRL10            0x19u
#define LSM_REG_STATUS            0x1Eu
#define LSM_REG_OUT_TEMP_L        0x20u   /**< đầu khối 14 byte dữ liệu */
#define LSM_REG_OUTX_L_G          0x22u
#define LSM_REG_OUTX_L_A          0x28u

#define LSM_WHO_AM_I_VALUE        0x70u

/* --- Bit trong IF_CFG (0x03) --- */
#define LSM_IF_CFG_I2C_I3C_DIS    0x01u
#define LSM_IF_CFG_SIM            0x04u
#define LSM_IF_CFG_PP_OD          0x08u
#define LSM_IF_CFG_H_LACTIVE      0x10u
#define LSM_IF_CFG_ASF_CTRL       0x20u
#define LSM_IF_CFG_SHUB_PU_EN     0x40u   /**< kéo lên SDX/SCX của bus phụ */
#define LSM_IF_CFG_SDA_PU_EN      0x80u

/* --- Bit trong CTRL3 --- */
#define LSM_CTRL3_SW_RESET        0x01u
#define LSM_CTRL3_IF_INC          0x04u
#define LSM_CTRL3_BDU             0x40u
#define LSM_CTRL3_BOOT            0x80u

/* --- Bit trong CTRL4 --- */
#define LSM_CTRL4_INT2_IN_LH      0x01u
#define LSM_CTRL4_DRDY_PULSED     0x02u   /**< BẮT BUỘC — xem ghi chú dưới */
#define LSM_CTRL4_INT2_DRDY_TEMP  0x04u
#define LSM_CTRL4_DRDY_MASK       0x08u
#define LSM_CTRL4_INT2_ON_INT1    0x10u

/* --- Bit trong INT1_CTRL --- */
#define LSM_INT1_DRDY_XL          0x01u
#define LSM_INT1_DRDY_G           0x02u

/* --- Bit trong STATUS_REG --- */
#define LSM_STATUS_XLDA           0x01u   /**< accel có mẫu mới          */
#define LSM_STATUS_GDA            0x02u   /**< gyro có mẫu mới           */
#define LSM_STATUS_TDA            0x04u   /**< nhiệt độ có mẫu mới       */

/** Bit đọc: địa chỉ thanh ghi phải OR với 0x80 khi muốn đọc. */
#define LSM_SPI_READ_BIT          0x80u

/**
 * Số byte đọc liên tiếp: temp(2) + gyro(6) + accel(6), tức 0x20..0x2D.
 * Trùng con số 14 của ICM20602 nhưng THỨ TỰ BÊN TRONG KHÁC HẲN.
 */
#define LSM_BURST_DATA_LEN        14u


/* ==========================================================================
 * Sensor hub — bus I2C phụ, nơi QMC6309 nằm
 *
 * Các thanh ghi dưới đây nằm ở BANK RIÊNG. Phải bật bit shub_reg_access
 * (bit 6 của FUNC_CFG_ACCESS = 0x01) mới truy cập được, và PHẢI TẮT LẠI sau
 * khi xong — để bật thì các thanh ghi thường trong dải 0x02..0x22 sẽ đọc ra
 * nội dung của bank sensor hub chứ không phải nội dung thật.
 *
 * Nguồn: driver chính thức của ST, các struct lsm6dsv_master_config_t,
 * lsm6dsv_slv0_add_t, lsm6dsv_slv0_config_t, lsm6dsv_status_master_t.
 * ========================================================================== */

#define LSM_FUNC_CFG_SHUB_ACCESS  0x40u   /**< bit6 của FUNC_CFG_ACCESS   */

#define LSM_SH_SENSOR_HUB_1       0x02u   /**< 0x02..0x07 = 6 byte đọc về */
#define LSM_SH_MASTER_CONFIG      0x14u
#define LSM_SH_SLV0_ADD           0x15u
#define LSM_SH_SLV0_SUBADD        0x16u
#define LSM_SH_SLV0_CONFIG        0x17u
#define LSM_SH_DATAWRITE_SLV0     0x21u
#define LSM_SH_STATUS_MASTER      0x22u

/* --- MASTER_CONFIG (0x14) --- */
#define LSM_SH_AUX_SENS_ONE       0x00u   /**< bit1..0 = 00 -> đúng 1 slave */
#define LSM_SH_MASTER_ON          0x04u
#define LSM_SH_PASS_THROUGH       0x10u
#define LSM_SH_START_CONFIG       0x20u   /**< 0 = kích theo DRDY của accel */
#define LSM_SH_WRITE_ONCE         0x40u
#define LSM_SH_RST_MASTER_REGS    0x80u

/* --- SLV0_ADD (0x15): bit0 = hướng, bit7..1 = địa chỉ 7 bit --- */
#define LSM_SH_SLV0_READ          0x01u

/* --- STATUS_MASTER (0x22) --- */
#define LSM_SH_STATUS_ENDOP       0x01u   /**< một lượt giao dịch đã xong  */
#define LSM_SH_STATUS_SLV0_NACK   0x08u   /**< slave KHÔNG trả lời         */
#define LSM_SH_STATUS_WR_ONCE     0x80u   /**< lệnh ghi-một-lần đã chạy    */

/* ==========================================================================
 * Trạng thái driver
 * ========================================================================== */

typedef enum {
    LSM_STATE_UNINIT = 0,   /**< chưa gọi init                        */
    LSM_STATE_IDLE,         /**< init xong, chưa bật ngắt DRDY        */
    LSM_STATE_RUNNING,      /**< đang đọc dữ liệu liên tục            */
    LSM_STATE_CALIBRATING,  /**< đang lấy bias gyro                   */
    LSM_STATE_ERROR         /**< lỗi giao tiếp, không dùng được       */
} lsm6dsv_state_t;

/* ==========================================================================
 * API
 * ========================================================================== */

/**
 * Đặt lại và cấu hình cảm biến. Hàm CHẶN khoảng 60 ms.
 * Gọi sau MX_SPI3_Init() và fc_time_init().
 * @return true nếu WHO_AM_I đúng và mọi thanh ghi ghi thành công.
 */
bool lsm6dsv_init(void);

/** Cho phép ngắt DRDY, bắt đầu luồng dữ liệu. */
void lsm6dsv_start(void);

/** Tạm dừng luồng dữ liệu. */
void lsm6dsv_stop(void);

/**
 * Bắt đầu lấy bias gyro. Không chặn — diễn ra trong ngắt, mất khoảng
 * IMU2_CALIB_SAMPLE_COUNT / IMU2_ODR_HZ giây. Máy bay phải nằm yên; phát
 * hiện rung quá IMU2_CALIB_MOVE_LIMIT_DPS thì tự khởi động lại từ đầu.
 */
void lsm6dsv_start_gyro_calibration(void);

/** Trạng thái hiện tại của driver. */
lsm6dsv_state_t lsm6dsv_get_state(void);

/** Phần trăm tiến độ hiệu chuẩn (0..100). */
uint8_t lsm6dsv_calibration_progress(void);

/** Giá trị WHO_AM_I đọc được lúc init — để in ra console khi dò lỗi. */
uint8_t lsm6dsv_who_am_i(void);

/** Tần số lấy mẫu THỰC TẾ đo được, Hz. Phải bám sát IMU2_ODR_HZ. */
uint32_t lsm6dsv_sample_rate_hz(void);

/** Số lần DRDY tới lúc DMA chưa xong (mẫu bị bỏ). Phải đứng yên ở 0. */
uint32_t lsm6dsv_overruns(void);

/**
 * Số lần quá trình hiệu chuẩn bias gyro bị huỷ và làm lại.
 *
 * Tăng không ngừng nghĩa là biên độ dao động của gyro vượt
 * IMU2_CALIB_MOVE_LIMIT_DPS ngay cả khi máy bay nằm yên — hiệu chuẩn sẽ lặp
 * vô hạn và bias không bao giờ được trừ. Không có con số này thì lỗi đó diễn
 * ra hoàn toàn im lặng.
 */
uint32_t lsm6dsv_calib_restarts(void);

/* --- Chẩn đoán khi KHÔNG có mẫu nào về ------------------------------------
 *
 * Ba số dưới đây tách bạch ba tầng có thể hỏng, chỉ cần một lần nhìn:
 *
 *   edges = 0  -> EXTI trên PD7 không hề kích. Chip không phát xung INT1,
 *                 hoặc dây INT chưa nối, hoặc nối vào INT2 chứ không phải INT1.
 *   edges > 0 nhưng count = 0
 *              -> EXTI có kích nhưng driver chặn (sai trạng thái) hoặc DMA hỏng.
 *   status bit GDA/XLDA = 0
 *              -> CHIP KHÔNG LẤY MẪU. Lỗi ở ODR hoặc chế độ hoạt động,
 *                 không phải ở đường ngắt.
 *   pd7 = 1 dai dẳng
 *              -> INT1 bị CHỐT ở mức cao. Đúng triệu chứng thiếu DRDY_PULSED.
 */

/** Số sườn lên đã bắt được trên PD7, đếm TRƯỚC mọi kiểm tra trạng thái. */
uint32_t lsm6dsv_drdy_edges(void);

/** STATUS_REG đọc được ở lần chẩn đoán gần nhất. */
uint8_t lsm6dsv_diag_status(void);

/** Mức logic hiện tại của chân PD7. */
bool lsm6dsv_diag_int_level(void);

/**
 * Đọc STATUS_REG và mức chân PD7. Gọi từ VÒNG LẶP CHÍNH, không phải từ ngắt.
 *
 * Tự vô hiệu hoá ngay khi có mẫu đầu tiên về (sample_count > 0), nên không
 * bao giờ đụng vào bus SPI trong lúc DMA đang chạy.
 */
void lsm6dsv_diag_poll(void);

/** Độ lệch chuẩn gyro của cửa sổ 1 giây vừa xong, °/s. Xem imu_noise.h. */
float lsm6dsv_gyro_sigma_dps(void);
vec3f_t lsm6dsv_gyro_sigma_axes_dps(void);


/* ==========================================================================
 * Từ kế QMC6309 qua sensor hub — giai đoạn 3
 * ========================================================================== */

/**
 * Dò và cấu hình QMC6309 qua bus I2C phụ, rồi bật chế độ đọc liên tục.
 *
 * Gọi SAU lsm6dsv_init() và TRƯỚC lsm6dsv_start(). Hàm CHẶN khoảng 60 ms.
 *
 * Thất bại ở đây KHÔNG làm hỏng đường IMU — chỉ mất từ kế. Máy bay vẫn bay
 * bình thường, chỉ là không có giữ hướng.
 *
 * @return true nếu đọc được chip ID 0x90 và mọi lệnh ghi đều được nhận.
 */
bool lsm6dsv_mag_init(void);

/**
 * Đọc một mẫu từ trường. Gọi từ VÒNG LẶP CHÍNH, không phải từ ngắt.
 *
 * Tự giới hạn nhịp theo MAG_UPDATE_RATE_HZ nên gọi mỗi vòng lặp cũng được.
 * Mỗi lần đọc thật chiếm bus SPI3 khoảng 40 µs và tạm khoá ngắt DRDY, tức
 * bỏ một hai mẫu IMU — ở 50 Hz thì đó là ~0,1% số mẫu.
 *
 * @return true nếu vừa nạp một mẫu mới vào g_fc.mag.
 */
bool lsm6dsv_mag_update(uint32_t now_us);

/** Tần số cập nhật từ kế đo được, Hz. */
uint32_t lsm6dsv_mag_rate_hz(void);

/** Số lần QMC6309 không trả lời trên bus I2C phụ. Phải đứng yên ở 0. */
uint32_t lsm6dsv_mag_nacks(void);

/** Số lần phải bỏ một lượt đọc vì không giành được bus SPI3. */
uint32_t lsm6dsv_mag_bus_busy(void);

/**
 * Vì sao lsm6dsv_mag_init() thất bại. Tách bạch từng tầng để không phải đoán:
 * bus phụ chết, từ kế không trả lời, hay đọc được mà sai chip.
 */
typedef enum {
    MAG_INIT_OK = 0,
    MAG_INIT_NOT_IDLE,      /**< gọi sai lúc — driver chưa ở trạng thái IDLE */
    MAG_INIT_PU_FAIL,       /**< không ghi được IF_CFG để bật pull-up        */
    MAG_INIT_HUB_TIMEOUT,   /**< sensor hub không báo xong — accel có chạy?  */
    MAG_INIT_NACK,          /**< không ai trả lời ở địa chỉ 0x7C             */
    MAG_INIT_BAD_ID,        /**< đọc được nhưng chip ID khác 0x90            */
    MAG_INIT_WRITE_FAIL,    /**< ghi thanh ghi cấu hình thất bại             */
    MAG_INIT_CFG_FAIL       /**< bật chế độ đọc liên tục thất bại            */
} mag_init_result_t;

/** Kết quả lần init gần nhất. */
mag_init_result_t lsm6dsv_mag_init_result(void);

/** Tên dạng chuỗi của kết quả trên, để in thẳng ra console. */
const char *lsm6dsv_mag_init_result_name(void);

/** Giá trị STATUS_MASTER đọc được ở lượt giao dịch cuối cùng. */
uint8_t lsm6dsv_mag_last_status(void);

/** Địa chỉ I2C 7 bit mà từ kế thực sự trả lời. 0 nếu chưa dò được. */
uint8_t lsm6dsv_mag_addr(void);

/**
 * Đổ toàn bộ thanh ghi then chốt của sensor hub ra console.
 *
 * Gọi khi lsm6dsv_mag_init() thất bại. Hàm CHẶN khoảng 500 ms và in ra:
 * cấu hình chính, nội dung bank sensor hub đọc lại được, và diễn biến
 * STATUS_MASTER theo thời gian trong một lượt kích thử.
 *
 * Mục đích: phân biệt "ghi không vào thanh ghi" với "ghi vào rồi nhưng bộ
 * máy không chạy" — hai lỗi trông giống hệt nhau từ bên ngoài.
 */
void lsm6dsv_mag_dump(void);

/* --- Hàm gọi từ ngắt, xem App/Drivers/drv_hal_callbacks.c --------------- */

/** Gọi khi chân DRDY (PD7) có sườn lên. */
void lsm6dsv_drdy_isr(void);

/** Gọi khi DMA của SPI3 truyền nhận xong. */
void lsm6dsv_spi_complete_isr(void);

/** Gọi khi SPI3 báo lỗi. */
void lsm6dsv_spi_error_isr(void);

#endif /* LSM6DSV_H */
