/**
 * @file    lsm6dsv.c
 * @brief   Hiện thực driver LSM6DSV (IMU phụ trên SPI3).
 */

#include "lsm6dsv.h"
#include "qmc6309.h"
#include "imu_noise.h"
#include "fc_state.h"
#include "fc_time.h"
#include "dbg_console.h"
#include "main.h"

extern SPI_HandleTypeDef hspi3;

/* ==========================================================================
 * Chuyển tham số cấu hình thành giá trị thanh ghi
 *
 * Các mã dưới đây lấy từ driver chính thức của ST (lsm6dsv_reg.c, các hàm
 * lsm6dsv_*_data_rate_get / lsm6dsv_*_full_scale_get).
 * ========================================================================== */

#if   IMU2_ODR_HZ == 120
  #define LSM_ODR_SEL   0x06u
#elif IMU2_ODR_HZ == 240
  #define LSM_ODR_SEL   0x07u
#elif IMU2_ODR_HZ == 480
  #define LSM_ODR_SEL   0x08u
#elif IMU2_ODR_HZ == 960
  #define LSM_ODR_SEL   0x09u
#elif IMU2_ODR_HZ == 1920
  #define LSM_ODR_SEL   0x0Au
#else
  #error "IMU2_ODR_HZ chi nhan 120, 240, 480, 960 hoac 1920"
#endif

/*
 * HỆ SỐ ĐỔI THANG — HAI HẰNG SỐ CẦN KIỂM CHỨNG THỰC TẾ
 *
 * Lấy theo bảng độ nhạy chuẩn của họ LSM6DS. Lưu ý con số của ST KHÔNG bằng
 * FS/32768 như ICM20602: dải ±2000 °/s có độ nhạy 70 mdps/LSB, tức 32768 LSB
 * ứng với ~2293 °/s. Đó là chủ ý của ST để có dư địa, không phải lỗi.
 *
 * CÁCH KIỂM (giai đoạn 2 trong App/Docs/KE_HOACH_LSM6DSV.md):
 *   Thang accel -> để yên trên bàn, cột |a|/g trong DBG_MODE_IMU2 phải ≈ 1,00.
 *   Thang gyro  -> DBG_MODE_IMU_CMP, xoay bo bằng tay, hai cột gyro phải bám
 *                  nhau. Lệch theo tỉ lệ CỐ ĐỊNH nghĩa là hằng số này sai.
 */
#if   IMU2_GYRO_FS_DPS == 125
  #define LSM_GYRO_FS_SEL        0x00u
  #define LSM_GYRO_MDPS_PER_LSB  4.375f
#elif IMU2_GYRO_FS_DPS == 250
  #define LSM_GYRO_FS_SEL        0x01u
  #define LSM_GYRO_MDPS_PER_LSB  8.75f
#elif IMU2_GYRO_FS_DPS == 500
  #define LSM_GYRO_FS_SEL        0x02u
  #define LSM_GYRO_MDPS_PER_LSB  17.50f
#elif IMU2_GYRO_FS_DPS == 1000
  #define LSM_GYRO_FS_SEL        0x03u
  #define LSM_GYRO_MDPS_PER_LSB  35.0f
#elif IMU2_GYRO_FS_DPS == 2000
  #define LSM_GYRO_FS_SEL        0x04u
  #define LSM_GYRO_MDPS_PER_LSB  70.0f
#else
  #error "IMU2_GYRO_FS_DPS chi nhan 125, 250, 500, 1000 hoac 2000"
#endif

#if   IMU2_ACCEL_FS_G == 2
  #define LSM_ACCEL_FS_SEL       0x00u
  #define LSM_ACCEL_MG_PER_LSB   0.061f
#elif IMU2_ACCEL_FS_G == 4
  #define LSM_ACCEL_FS_SEL       0x01u
  #define LSM_ACCEL_MG_PER_LSB   0.122f
#elif IMU2_ACCEL_FS_G == 8
  #define LSM_ACCEL_FS_SEL       0x02u
  #define LSM_ACCEL_MG_PER_LSB   0.244f
#elif IMU2_ACCEL_FS_G == 16
  #define LSM_ACCEL_FS_SEL       0x03u
  #define LSM_ACCEL_MG_PER_LSB   0.488f
#else
  #error "IMU2_ACCEL_FS_G chi nhan 2, 4, 8 hoac 16"
#endif

#define LSM_GYRO_SCALE   (LSM_GYRO_MDPS_PER_LSB * 0.001f)
#define LSM_ACCEL_SCALE  (LSM_ACCEL_MG_PER_LSB  * 0.001f * FC_GRAVITY_MPS2)

/* Nhiệt độ: T[°C] = raw / 256 + 25. Khác ICM20602 (chia 326,8). */
#define LSM_TEMP_SCALE   (1.0f / 256.0f)
#define LSM_TEMP_OFFSET  25.0f

/* Chế độ hiệu năng cao cho cả accel lẫn gyro là mã 0 ở bit 4-6. */
#define LSM_CTRL1_VALUE  ((uint8_t)(LSM_ODR_SEL))
#define LSM_CTRL2_VALUE  ((uint8_t)(LSM_ODR_SEL))

/*
 * Điều khiển chân CS bằng thanh ghi BSRR — một lệnh ghi duy nhất, không đọc
 * lại, an toàn khi bị ngắt cắt ngang. Giống hệt cách icm20602.c làm.
 */
#define CS_LOW()   (SPI3_SS_GPIO_Port->BSRR = (uint32_t)SPI3_SS_Pin << 16u)
#define CS_HIGH()  (SPI3_SS_GPIO_Port->BSRR = (uint32_t)SPI3_SS_Pin)

/* ==========================================================================
 * Biến nội bộ
 *
 * Hai bộ đệm SPI là nguồn/đích trực tiếp của DMA2_Stream2 và DMA2_Stream3
 * nên BẮT BUỘC dùng FC_DMA_BUFFER. Để mặc định chúng rơi vào DTCMRAM mà
 * DMA1/DMA2 trên H743 không truy cập được — DMA sẽ im lặng không chạy.
 * ========================================================================== */

FC_DMA_BUFFER static uint8_t s_tx[1u + LSM_BURST_DATA_LEN];
FC_DMA_BUFFER static uint8_t s_rx[1u + LSM_BURST_DATA_LEN];

static volatile lsm6dsv_state_t s_state = LSM_STATE_UNINIT;
static volatile bool     s_busy;            /* đang có transfer DMA chạy   */
static volatile uint32_t s_sample_us;       /* mốc thời gian lúc DRDY      */
static uint32_t          s_prev_sample_us;
static uint32_t          s_overrun_count;
static uint8_t           s_who;             /* WHO_AM_I đọc lúc init       */
static uint32_t          s_cal_restarts;    /* hieu chuan bi huy bao nhieu lan */

/* --- Chẩn đoán, chỉ hoạt động khi chưa có mẫu nào về --- */
static volatile uint32_t s_drdy_edges;      /* sườn lên bắt được trên PD7  */
static uint8_t           s_diag_status;     /* STATUS_REG lần đọc gần nhất */
static bool              s_diag_pin;        /* mức logic PD7               */

static float s_gyro_alpha;
static float s_accel_alpha;

/* Đo tần số lấy mẫu thực tế, mỗi giây một lần. */
static uint32_t s_hz;
static uint32_t s_hz_count;
static uint32_t s_hz_mark_us;

static imu_noise_t s_noise;

/** Bộ tích luỹ cho quá trình lấy bias gyro. */
static struct {
    uint32_t count;
    float    sum[AXIS_COUNT];
    float    sumsq[AXIS_COUNT];   /* để tính độ lệch chuẩn, xem calibration_feed */
} s_cal;

/* ==========================================================================
 * Truy cập thanh ghi ở chế độ hỏi vòng (chỉ dùng lúc init)
 *
 * HAL_SPI_Transmit / TransmitReceive bản chặn dùng CPU chép dữ liệu, không
 * qua DMA, nên bộ đệm nằm trên stack (DTCMRAM) vẫn hợp lệ ở đây.
 * ========================================================================== */

static bool reg_write(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7Fu), value };

    CS_LOW();
    const HAL_StatusTypeDef st =
        HAL_SPI_Transmit(&hspi3, tx, sizeof(tx), IMU2_SPI_TIMEOUT_MS);
    CS_HIGH();

    delay_us(2);
    return (st == HAL_OK);
}

static bool reg_read(uint8_t reg, uint8_t *value)
{
    uint8_t tx[2] = { (uint8_t)(reg | LSM_SPI_READ_BIT), 0x00u };
    uint8_t rx[2] = { 0, 0 };

    CS_LOW();
    const HAL_StatusTypeDef st =
        HAL_SPI_TransmitReceive(&hspi3, tx, rx, sizeof(tx), IMU2_SPI_TIMEOUT_MS);
    CS_HIGH();

    delay_us(2);
    *value = rx[1];
    return (st == HAL_OK);
}

/** Ghi rồi đọc lại để chắc chắn thanh ghi đã nhận đúng giá trị. */
static bool reg_write_verify(uint8_t reg, uint8_t value)
{
    uint8_t readback = 0;

    if (!reg_write(reg, value)) {
        return false;
    }
    if (!reg_read(reg, &readback)) {
        return false;
    }
    return (readback == value);
}

/** Đổi tốc độ SPI3 giữa lúc chạy. HAL_SPI_Init tự tắt/bật lại ngoại vi và
 *  KHÔNG gọi lại MspInit khi trạng thái đã khác RESET, nên liên kết DMA
 *  vẫn giữ nguyên. */
static bool spi_set_baud(uint32_t prescaler)
{
    hspi3.Init.BaudRatePrescaler = prescaler;
    return (HAL_SPI_Init(&hspi3) == HAL_OK);
}

/* ==========================================================================
 * Khởi tạo
 * ========================================================================== */

bool lsm6dsv_init(void)
{
    s_state = LSM_STATE_UNINIT;
    s_busy  = false;
    s_who   = 0;
    memset(s_tx, 0, sizeof(s_tx));
    memset(s_rx, 0, sizeof(s_rx));
    memset(&s_cal, 0, sizeof(s_cal));

    /* Hệ số lọc tính sẵn theo ODR danh định, tránh chia trong ISR. */
    const float dt = 1.0f / (float)IMU2_ODR_HZ;
    s_gyro_alpha  = fc_lpf_alpha(IMU2_GYRO_LPF_HZ,  dt);
    s_accel_alpha = fc_lpf_alpha(IMU2_ACCEL_LPF_HZ, dt);

    imu_noise_reset(&s_noise, micros());
    s_hz            = 0;
    s_hz_count      = 0;
    s_hz_mark_us    = micros();
    s_overrun_count = 0;
    s_drdy_edges    = 0;
    s_diag_status   = 0;
    s_diag_pin      = false;
    s_cal_restarts  = 0;

    CS_HIGH();

    /* Ghi thanh ghi cấu hình ở 1 MHz cho chắc, tuy chip chịu được 10 MHz. */
    if (!spi_set_baud(SPI_BAUDRATEPRESCALER_64)) {
        goto fail;
    }
    HAL_Delay(1);

    /*
     * --- Đặt lại toàn bộ chip ---
     * Đặt lại TRƯỚC khi đọc WHO_AM_I: nếu lần nạp trước để chip ở một trạng
     * thái lạ (ví dụ sensor hub đang mở bank), thanh ghi thường sẽ đọc ra rác
     * và WHO_AM_I sai một cách khó hiểu.
     */
    if (!reg_write(LSM_REG_CTRL3, LSM_CTRL3_SW_RESET)) {
        goto fail;
    }

    /* Chờ bit reset tự xoá. Datasheet nói khoảng 50 µs, cho rộng rãi. */
    {
        bool cleared = false;

        for (int i = 0; i < 50; i++) {
            uint8_t c3 = 0xFFu;

            HAL_Delay(1);
            if (reg_read(LSM_REG_CTRL3, &c3) && (c3 & LSM_CTRL3_SW_RESET) == 0u) {
                cleared = true;
                break;
            }
        }
        if (!cleared) {
            goto fail;
        }
    }

    /* --- Xác minh đúng chip --- */
    if (!reg_read(LSM_REG_WHO_AM_I, &s_who) || s_who != LSM_WHO_AM_I_VALUE) {
        goto fail;
    }

    /*
     * --- Cấu hình đo ---
     *
     * CTRL3: BDU giữ cặp byte thấp/cao thuộc CÙNG một mẫu cho tới khi đọc
     * xong — không có nó thì đọc burst có thể ghép byte thấp của mẫu này với
     * byte cao của mẫu sau. IF_INC cho phép địa chỉ tự tăng khi đọc liên tiếp,
     * bắt buộc phải có để đọc một phát 14 byte.
     */
    if (!reg_write_verify(LSM_REG_CTRL3, (uint8_t)(LSM_CTRL3_BDU | LSM_CTRL3_IF_INC))) {
        goto fail;
    }

    /* Dải đo trước, tốc độ sau — đổi dải khi đang chạy sẽ có vài mẫu rác. */
    if (!reg_write_verify(LSM_REG_CTRL6, LSM_GYRO_FS_SEL) ||
        !reg_write_verify(LSM_REG_CTRL8, LSM_ACCEL_FS_SEL)) {
        goto fail;
    }

    if (!reg_write_verify(LSM_REG_CTRL1, LSM_CTRL1_VALUE) ||
        !reg_write_verify(LSM_REG_CTRL2, LSM_CTRL2_VALUE)) {
        goto fail;
    }

    /*
     * --- DRDY PHẢI Ở CHẾ ĐỘ PHÁT XUNG ---
     *
     * Mặc định của chip (CTRL4 = 0x00) là chế độ CHỐT: INT1 lên cao khi có
     * mẫu mới và chỉ hạ xuống KHI DỮ LIỆU ĐƯỢC ĐỌC. Với EXTI bắt sườn lên,
     * cách đó khoá chết ngay từ mẫu đầu:
     *
     *     có mẫu -> INT1 lên cao và GIỮ NGUYÊN
     *            -> không ai đọc dữ liệu vì ISR chưa từng chạy
     *            -> INT1 không bao giờ hạ -> không còn sườn lên nào nữa
     *
     * Triệu chứng: init báo OK, mọi thanh ghi đọc lại đúng, nhưng count đứng
     * yên ở 0 và err cũng 0 — tức ISR chưa vào lần nào. Đây chính là lỗi đã
     * gặp ở lần chạy đầu tiên.
     *
     * DRDY_PULSED cho INT1 phát một xung ngắn mỗi mẫu rồi tự về thấp, giống
     * cách ICM20602 hoạt động với INT_PIN_CFG = 0x00. Quan trọng hơn: nếu lỡ
     * bỏ một mẫu thì mẫu sau vẫn sinh sườn mới, không khoá chết vĩnh viễn.
     */
    if (!reg_write_verify(LSM_REG_CTRL4, LSM_CTRL4_DRDY_PULSED)) {
        goto fail;
    }

    /*
     * Chân INT1 báo gyro có dữ liệu mới. Chọn gyro chứ không phải accel vì
     * hai bên cùng ODR mà gyro mới là thứ vòng điều khiển cần — lấy nhịp
     * theo nó thì mẫu gyro luôn tươi nhất.
     *
     * Mặc định của chip là tích cực mức cao, đẩy kéo. Khớp với PD7 cấu hình
     * bắt sườn lên, không kéo trở.
     */
    if (!reg_write_verify(LSM_REG_INT1_CTRL, LSM_INT1_DRDY_G)) {
        goto fail;
    }

    /* Chip cần vài mẫu để ổn định sau khi bật ODR. */
    HAL_Delay(20);

    /* Xong phần cấu hình, chuyển sang 8 MHz cho đường dữ liệu. */
    if (!spi_set_baud(SPI_BAUDRATEPRESCALER_8)) {
        goto fail;
    }

    s_tx[0] = (uint8_t)(LSM_REG_OUT_TEMP_L | LSM_SPI_READ_BIT);

    /*
     * Đọc mồi một lượt khối dữ liệu để xoá mọi trạng thái DRDY còn treo từ
     * trước khi bật DRDY_PULSED. Không có bước này thì mẫu đầu tiên có thể
     * bị lỡ. Dùng đường hỏi vòng, chưa động tới DMA.
     */
    {
        uint8_t tx[1u + LSM_BURST_DATA_LEN];
        uint8_t rx[1u + LSM_BURST_DATA_LEN];

        memset(tx, 0, sizeof(tx));
        tx[0] = (uint8_t)(LSM_REG_OUT_TEMP_L | LSM_SPI_READ_BIT);

        CS_LOW();
        (void)HAL_SPI_TransmitReceive(&hspi3, tx, rx, sizeof(tx),
                                      IMU2_SPI_TIMEOUT_MS);
        CS_HIGH();
        delay_us(2);
    }

    g_fc.imu2.calibrated = false;
    g_fc.imu2.healthy    = false;
    s_state              = LSM_STATE_IDLE;
    return true;

fail:
    s_state = LSM_STATE_ERROR;
    g_fc.imu2.healthy = false;
    fc_state_set_error(FC_ERR_IMU2_SPI);
    return false;
}

void lsm6dsv_start(void)
{
    if (s_state == LSM_STATE_IDLE) {
        s_prev_sample_us = micros();
        s_state = LSM_STATE_RUNNING;
    }
}

void lsm6dsv_stop(void)
{
    if (s_state == LSM_STATE_RUNNING || s_state == LSM_STATE_CALIBRATING) {
        s_state = LSM_STATE_IDLE;
    }
}

/* ==========================================================================
 * Hiệu chuẩn bias gyro
 * ========================================================================== */

void lsm6dsv_start_gyro_calibration(void)
{
    if (s_state != LSM_STATE_RUNNING && s_state != LSM_STATE_CALIBRATING) {
        return;
    }

    memset(&s_cal, 0, sizeof(s_cal));

    g_fc.imu2.calibrated    = false;
    g_fc.imu2.gyro_bias_dps = (vec3f_t){ 0.0f, 0.0f, 0.0f };
    s_state = LSM_STATE_CALIBRATING;
}

uint8_t lsm6dsv_calibration_progress(void)
{
    if (s_state != LSM_STATE_CALIBRATING) {
        return g_fc.imu2.calibrated ? 100u : 0u;
    }
    return (uint8_t)((s_cal.count * 100u) / IMU2_CALIB_SAMPLE_COUNT);
}

/** Nạp một mẫu vào bộ tích luỹ bias. Gọi từ ISR. */
static void calibration_feed(const float gyro_body[AXIS_COUNT])
{
    for (int i = 0; i < AXIS_COUNT; i++) {
        s_cal.sum[i]   += gyro_body[i];
        s_cal.sumsq[i] += gyro_body[i] * gyro_body[i];
    }
    s_cal.count++;

    /*
     * Máy bay chưa đứng yên thì bias tính ra sẽ lệch. Đo bằng ĐỘ LỆCH CHUẨN,
     * không phải biên độ đỉnh-đỉnh — xem IMU2_CALIB_MOVE_SD_DPS trong
     * fc_config.h.
     *
     * Bản cũ dùng biên độ đỉnh-đỉnh với ngưỡng 2,0 °/s, và vì nhiễu nền của
     * chính con này đã cho biên độ ~8,2 °/s nên hiệu chuẩn lặp VÔ HẠN — đo
     * được 174 lần huỷ liên tiếp trong khi máy bay nằm im.
     */
    if (s_cal.count >= IMU_CALIB_SD_MIN_SAMPLES) {
        const float inv = 1.0f / (float)s_cal.count;

        for (int i = 0; i < AXIS_COUNT; i++) {
            const float mean = s_cal.sum[i] * inv;
            const float var  = s_cal.sumsq[i] * inv - mean * mean;

            if (var > (IMU2_CALIB_MOVE_SD_DPS * IMU2_CALIB_MOVE_SD_DPS)) {
                /*
                 * Bộ đếm này tăng KHÔNG NGỪNG nghĩa là nền nhiễu của chính con
                 * quay đã vượt ngưỡng, chứ không phải máy bay đang rung.
                 * Console báo ra để không phải ngồi đoán.
                 */
                s_cal_restarts++;
                memset(&s_cal, 0, sizeof(s_cal));
                return;
            }
        }
    }

    if (s_cal.count < IMU2_CALIB_SAMPLE_COUNT) {
        return;
    }

    const float inv = 1.0f / (float)s_cal.count;

    g_fc.imu2.gyro_bias_dps = (vec3f_t){
        s_cal.sum[AXIS_ROLL]  * inv,
        s_cal.sum[AXIS_PITCH] * inv,
        s_cal.sum[AXIS_YAW]   * inv
    };
    g_fc.imu2.calibrated = true;
    s_state = LSM_STATE_RUNNING;
}

/* ==========================================================================
 * Xử lý mẫu
 * ========================================================================== */

/** Ghép hai byte theo thứ tự BYTE THẤP TRƯỚC — quy ước của ST. */
static inline int16_t le16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/** Xoay trục cảm biến sang trục thân theo IMU2_AXIS_MAP/SIGN. */
static void align_axes(const float in[3], float out[AXIS_COUNT])
{
    out[AXIS_ROLL]  = (float)(IMU2_AXIS_SIGN_X) * in[IMU2_AXIS_MAP_X];
    out[AXIS_PITCH] = (float)(IMU2_AXIS_SIGN_Y) * in[IMU2_AXIS_MAP_Y];
    out[AXIS_YAW]   = (float)(IMU2_AXIS_SIGN_Z) * in[IMU2_AXIS_MAP_Z];
}

static void process_sample(void)
{
    /*
     * --- Tách dữ liệu thô ---
     * Byte 0 của s_rx là byte rác ứng với lúc gửi địa chỉ. Sau đó khối 14
     * byte xếp theo thứ tự của ST: temp -> gyro -> accel, mỗi giá trị là
     * 16 bit có dấu, BYTE THẤP TRƯỚC.
     */
    const int16_t t_raw  = le16(&s_rx[1]);
    const int16_t gx_raw = le16(&s_rx[3]);
    const int16_t gy_raw = le16(&s_rx[5]);
    const int16_t gz_raw = le16(&s_rx[7]);
    const int16_t ax_raw = le16(&s_rx[9]);
    const int16_t ay_raw = le16(&s_rx[11]);
    const int16_t az_raw = le16(&s_rx[13]);

    /*
     * Toàn bộ bằng 0 thường là dấu hiệu mất kết nối (MISO treo thấp) chứ
     * không phải số đo thật.
     */
    if ((gx_raw | gy_raw | gz_raw | ax_raw | ay_raw | az_raw) == 0) {
        g_fc.imu2.error_count++;
        return;
    }

    /* --- Đổi thang rồi xoay trục --- */
    const float gyro_sensor[3] = {
        (float)gx_raw * LSM_GYRO_SCALE,
        (float)gy_raw * LSM_GYRO_SCALE,
        (float)gz_raw * LSM_GYRO_SCALE
    };
    const float accel_sensor[3] = {
        (float)ax_raw * LSM_ACCEL_SCALE,
        (float)ay_raw * LSM_ACCEL_SCALE,
        (float)az_raw * LSM_ACCEL_SCALE
    };

    float gyro_body[AXIS_COUNT];
    float accel_body[AXIS_COUNT];
    align_axes(gyro_sensor,  gyro_body);
    align_axes(accel_sensor, accel_body);

    /* --- Đang hiệu chuẩn thì chỉ tích luỹ, chưa xuất số liệu --- */
    if (s_state == LSM_STATE_CALIBRATING) {
        calibration_feed(gyro_body);
    }

    /* --- Trừ bias --- */
    const vec3f_t bias = g_fc.imu2.gyro_bias_dps;
    const vec3f_t gyro = {
        gyro_body[AXIS_ROLL]  - bias.x,
        gyro_body[AXIS_PITCH] - bias.y,
        gyro_body[AXIS_YAW]   - bias.z
    };
    const vec3f_t accel = {
        accel_body[AXIS_ROLL],
        accel_body[AXIS_PITCH],
        accel_body[AXIS_YAW]
    };

    /* --- Lọc thông thấp --- */
    vec3f_t filt = g_fc.imu2.gyro_filtered_dps;
    filt.x = fc_lpf(filt.x, gyro.x, s_gyro_alpha);
    filt.y = fc_lpf(filt.y, gyro.y, s_gyro_alpha);
    filt.z = fc_lpf(filt.z, gyro.z, s_gyro_alpha);

    vec3f_t accel_filt = g_fc.imu2.accel_mps2;
    accel_filt.x = fc_lpf(accel_filt.x, accel.x, s_accel_alpha);
    accel_filt.y = fc_lpf(accel_filt.y, accel.y, s_accel_alpha);
    accel_filt.z = fc_lpf(accel_filt.z, accel.z, s_accel_alpha);

    /* --- Nền nhiễu: nạp gyro CHƯA LỌC, đó mới là nhiễu thật --- */
    imu_noise_feed(&s_noise, gyro, s_sample_us);

    /* --- Đo tần số lấy mẫu thực tế --- */
    s_hz_count++;
    if (fc_elapsed_us(s_sample_us, s_hz_mark_us) >= 1000000u) {
        s_hz         = s_hz_count;
        s_hz_count   = 0;
        s_hz_mark_us = s_sample_us;
    }

    /* --- Ghi vào khối trạng thái chung --- */
    g_fc.imu2.gyro_raw          = (vec3i16_t){ gx_raw, gy_raw, gz_raw };
    g_fc.imu2.accel_raw         = (vec3i16_t){ ax_raw, ay_raw, az_raw };
    g_fc.imu2.gyro_dps          = gyro;
    g_fc.imu2.gyro_filtered_dps = filt;
    g_fc.imu2.accel_mps2        = accel_filt;
    g_fc.imu2.temperature_c     = (float)t_raw * LSM_TEMP_SCALE + LSM_TEMP_OFFSET;

    g_fc.imu2.dt_us        = fc_elapsed_us(s_sample_us, s_prev_sample_us);
    s_prev_sample_us       = s_sample_us;
    g_fc.imu2.timestamp_us = s_sample_us;
    g_fc.imu2.sample_count++;
    g_fc.imu2.healthy = true;
}

/* ==========================================================================
 * Hàm gọi từ ngắt
 * ========================================================================== */

void lsm6dsv_drdy_isr(void)
{
    /*
     * Đếm TRƯỚC mọi kiểm tra. Nhờ vậy phân biệt được "EXTI không hề kích"
     * với "EXTI có kích nhưng driver chặn lại" — hai lỗi khác hẳn nhau.
     */
    s_drdy_edges++;

    if (s_state != LSM_STATE_RUNNING && s_state != LSM_STATE_CALIBRATING) {
        return;
    }

    /*
     * Mẫu trước chưa đọc xong mà mẫu mới đã tới. Ở 1920 Hz mỗi transfer
     * 15 byte ở 8 MHz chỉ mất ~15 µs trên tổng 521 µs, nên chuyện này chỉ
     * xảy ra khi có ngắt khác giữ CPU rất lâu. Bỏ mẫu, không xếp hàng chờ.
     */
    if (s_busy) {
        s_overrun_count++;
        g_fc.imu2.error_count++;
        return;
    }

    s_busy      = true;
    s_sample_us = micros();
    s_tx[0]     = (uint8_t)(LSM_REG_OUT_TEMP_L | LSM_SPI_READ_BIT);

    CS_LOW();
    if (HAL_SPI_TransmitReceive_DMA(&hspi3, s_tx, s_rx,
                                    1u + LSM_BURST_DATA_LEN) != HAL_OK) {
        CS_HIGH();
        s_busy = false;
        g_fc.imu2.error_count++;
        fc_state_set_error(FC_ERR_IMU2_SPI);
    }
}

void lsm6dsv_spi_complete_isr(void)
{
    CS_HIGH();
    s_busy = false;
    process_sample();
}

void lsm6dsv_spi_error_isr(void)
{
    CS_HIGH();
    HAL_SPI_Abort(&hspi3);
    s_busy = false;

    g_fc.imu2.error_count++;
    g_fc.imu2.healthy = false;
    fc_state_set_error(FC_ERR_IMU2_SPI);
}

/* ==========================================================================
 * API đọc
 * ========================================================================== */

lsm6dsv_state_t lsm6dsv_get_state(void)   { return s_state; }
uint8_t         lsm6dsv_who_am_i(void)    { return s_who; }
uint32_t        lsm6dsv_sample_rate_hz(void) { return s_hz; }
uint32_t        lsm6dsv_overruns(void)    { return s_overrun_count; }
uint32_t        lsm6dsv_calib_restarts(void) { return s_cal_restarts; }
uint32_t        lsm6dsv_drdy_edges(void)  { return s_drdy_edges; }
uint8_t         lsm6dsv_diag_status(void) { return s_diag_status; }
bool            lsm6dsv_diag_int_level(void) { return s_diag_pin; }

void lsm6dsv_diag_poll(void)
{
    /*
     * Chỉ chẩn đoán khi CHƯA có mẫu nào. Một khi dữ liệu đã chảy thì hàm này
     * không đụng gì tới bus SPI nữa — tránh hẳn nguy cơ chen ngang DMA.
     */
    if (g_fc.imu2.sample_count != 0u || s_state == LSM_STATE_UNINIT) {
        return;
    }

    s_diag_pin = (HAL_GPIO_ReadPin(SPI3_INT_GPIO_Port, SPI3_INT_Pin) == GPIO_PIN_SET);

    if (!s_busy) {
        uint8_t st = 0;

        if (reg_read(LSM_REG_STATUS, &st)) {
            s_diag_status = st;
        }
    }
}

float lsm6dsv_gyro_sigma_dps(void)
{
    return imu_noise_sigma_max(&s_noise);
}

vec3f_t lsm6dsv_gyro_sigma_axes_dps(void)
{
    return (vec3f_t){ s_noise.sigma[0], s_noise.sigma[1], s_noise.sigma[2] };
}

/* ==========================================================================
 * TỪ KẾ QMC6309 QUA SENSOR HUB — giai đoạn 3
 *
 * QMC6309 nằm trên bus I2C phụ của LSM6DSV (chân SDX/SCX), không có dây nào
 * ra MCU. LSM6DSV làm I2C master, tự đọc từ kế theo nhịp, rồi để kết quả vào
 * dãy SENSOR_HUB_1..6 để host lấy qua SPI3.
 *
 * Bộ máy sensor hub được KÍCH BỞI DRDY CỦA ACCEL, nên accel phải đang chạy
 * thì các lệnh dưới đây mới thực thi. Ở ODR 1920 Hz một lượt xong sau ~0,5 ms.
 * ========================================================================== */

/* --- Đổi tham số cấu hình thành giá trị thanh ghi của QMC6309 --- */

#if   MAG_RANGE_G == 8
  #define MAG_RNG_SEL        QMC_RNG_8G
  #define MAG_LSB_PER_GAUSS  QMC_LSB_PER_GAUSS_8G
#elif MAG_RANGE_G == 16
  #define MAG_RNG_SEL        QMC_RNG_16G
  #define MAG_LSB_PER_GAUSS  QMC_LSB_PER_GAUSS_16G
#elif MAG_RANGE_G == 32
  #define MAG_RNG_SEL        QMC_RNG_32G
  #define MAG_LSB_PER_GAUSS  QMC_LSB_PER_GAUSS_32G
#else
  #error "MAG_RANGE_G chi nhan 8, 16 hoac 32"
#endif

#if   MAG_ODR_HZ == 1
  #define MAG_ODR_SEL   QMC_ODR_1HZ
#elif MAG_ODR_HZ == 10
  #define MAG_ODR_SEL   QMC_ODR_10HZ
#elif MAG_ODR_HZ == 50
  #define MAG_ODR_SEL   QMC_ODR_50HZ
#elif MAG_ODR_HZ == 100
  #define MAG_ODR_SEL   QMC_ODR_100HZ
#elif MAG_ODR_HZ == 200
  #define MAG_ODR_SEL   QMC_ODR_200HZ
#else
  #error "MAG_ODR_HZ chi nhan 1, 10, 50, 100 hoac 200"
#endif

#if   MAG_OSR1 == 8
  #define MAG_OSR1_SEL  QMC_OSR1_8
#elif MAG_OSR1 == 4
  #define MAG_OSR1_SEL  QMC_OSR1_4
#elif MAG_OSR1 == 2
  #define MAG_OSR1_SEL  QMC_OSR1_2
#elif MAG_OSR1 == 1
  #define MAG_OSR1_SEL  QMC_OSR1_1
#else
  #error "MAG_OSR1 chi nhan 1, 2, 4 hoac 8"
#endif

#if   MAG_OSR2 == 1
  #define MAG_OSR2_SEL  QMC_OSR2_1
#elif MAG_OSR2 == 2
  #define MAG_OSR2_SEL  QMC_OSR2_2
#elif MAG_OSR2 == 4
  #define MAG_OSR2_SEL  QMC_OSR2_4
#elif MAG_OSR2 == 8
  #define MAG_OSR2_SEL  QMC_OSR2_8
#elif MAG_OSR2 == 16
  #define MAG_OSR2_SEL  QMC_OSR2_16
#else
  #error "MAG_OSR2 chi nhan 1, 2, 4, 8 hoac 16"
#endif

/* Giá trị cuối cùng ghi vào hai thanh ghi điều khiển của QMC6309. */
#define MAG_CTRL2_VALUE  ((uint8_t)(((MAG_ODR_SEL) << QMC_CTRL2_ODR_SHIFT) |  \
                                    ((MAG_RNG_SEL) << QMC_CTRL2_RNG_SHIFT) |  \
                                     (QMC_SETRESET_ON)))
#define MAG_CTRL1_VALUE  ((uint8_t)(((MAG_OSR2_SEL) << QMC_CTRL1_OSR2_SHIFT) | \
                                    ((MAG_OSR1_SEL) << QMC_CTRL1_OSR1_SHIFT) | \
                                     (QMC_MODE_NORMAL)))

/* Ba pha cho mot mau, nen nhip pha = 3 x nhip mau mong muon. */
#define MAG_PHASE_US     (1000000UL / (MAG_UPDATE_RATE_HZ * 3u))

static uint32_t s_mag_last_us;
static uint32_t s_mag_hz;
static uint32_t s_mag_hz_count;
static uint32_t s_mag_hz_mark_us;
static uint32_t s_mag_nacks;
static uint32_t s_mag_bus_busy;
static mag_init_result_t s_mag_result = MAG_INIT_NOT_IDLE;
static uint8_t  s_mag_status;      /* STATUS_MASTER luot cuoi */
static bool     s_mag_timeout;     /* luot cuoi co bi qua han khong */
static uint8_t  s_mag_addr;        /* dia chi 7 bit tu ke tra loi   */

/*
 * Ba pha cua mot luot doc tu ke. Moi pha chi lam mot chum SPI ngan (~40 us)
 * roi tra quyen dieu khien; thoi gian cho nam GIUA cac lan goi tu vong lap
 * chinh, nen khong co cho nao chan lau.
 */
typedef enum {
    MAG_PH_TRIGGER = 0,  /* cau hinh SLV0 + bat master, roi tat accel */
    MAG_PH_KICK,         /* bat lai accel -> sinh suon kich hub       */
    MAG_PH_COLLECT       /* doc STATUS_MASTER va sau byte du lieu     */
} mag_phase_t;

static mag_phase_t s_mag_phase;

/* ==========================================================================
 * Giành bus SPI3 cho một giao dịch hỏi vòng từ vòng lặp chính
 *
 * Đường dữ liệu IMU chạy bằng DMA, được kích bởi ngắt DRDY ở 1920 Hz. Nếu
 * vòng lặp chính chen một giao dịch hỏi vòng vào giữa thì hai bên đâm nhau
 * trên cùng một ngoại vi SPI.
 *
 * Cách xử lý: tạm KHOÁ ngắt DRDY, đợi nốt transfer đang chạy (nếu có), làm
 * việc của mình, rồi mở lại. Vài mẫu IMU bị bỏ trong lúc đó là chấp nhận
 * được — ở 50 Hz thì chỉ mất khoảng 0,1% số mẫu.
 *
 * KHÔNG xoá cờ EXTI đang treo lúc mở lại: một sườn xảy ra trong lúc khoá vẫn
 * là mẫu hợp lệ, để nó chạy thì đỡ mất một mẫu. Và nhờ DRDY_PULSED nên việc
 * khoá/mở ngắt không gây kẹt cứng như chế độ chốt mức.
 * ========================================================================== */

static bool bus_acquire(void)
{
    HAL_NVIC_DisableIRQ(SPI3_INT_EXTI_IRQn);

    /* Một transfer 15 byte ở 8 MHz mất ~15 µs. Trần 200 µs là rất rộng. */
    for (int i = 0; i < 40; i++) {
        if (!s_busy) {
            return true;
        }
        delay_us(5);
    }

    HAL_NVIC_EnableIRQ(SPI3_INT_EXTI_IRQn);
    s_mag_bus_busy++;
    return false;
}

static void bus_release(void)
{
    HAL_NVIC_EnableIRQ(SPI3_INT_EXTI_IRQn);
}

/* ==========================================================================
 * Truy cập bank sensor hub
 * ========================================================================== */

/* ==========================================================================
 * Bật/tắt gia tốc kế — đây là CÔNG TẮC KÍCH của sensor hub
 *
 * Bộ máy sensor hub chỉ khởi động ở sườn DRDY ĐẦU TIÊN của accel sau khi
 * master được bật. Nếu accel đã chạy sẵn từ trước thì không có sườn "đầu
 * tiên" nào và hub nằm im — STATUS_MASTER đọc mãi ra 0x00.
 *
 * Đây chính là lỗi đã gặp ở lần chạy thứ hai, và là lý do mọi ví dụ chính
 * thức của ST đều theo đúng thứ tự:
 *
 *     tắt accel  ->  bật master  ->  BẬT LẠI accel  ->  đợi cờ xong
 * ========================================================================== */

static void xl_off(void)
{
    (void)reg_write(LSM_REG_CTRL1, 0x00u);
    delay_us(500);
}

static void xl_on(void)
{
    (void)reg_write(LSM_REG_CTRL1, LSM_CTRL1_VALUE);
}

static bool shub_bank(bool on)
{
    return reg_write_verify(LSM_REG_FUNC_CFG_ACCESS,
                            on ? LSM_FUNC_CFG_SHUB_ACCESS : 0x00u);
}

/*
 * Đợi bộ máy sensor hub chạy xong.
 *
 * KHÔNG hỏi vòng. Đây là bài học đắt nhất của giai đoạn này, và bằng chứng
 * đo được trên chính bo này:
 *
 *   - Hỏi mỗi 1 ms  -> cờ ENDOP KHÔNG BAO GIỜ xuất hiện.
 *   - Hỏi mỗi 20 ms -> lúc được lúc không.
 *   - Hỏi mỗi 30 ms -> ENDOP xuất hiện ĐÚNG MỘT LẦN rồi thôi, sau đó
 *                      STATUS_MASTER về 0x00 mãi mãi.
 *
 * Lý do: muốn đọc STATUS_MASTER thì phải MỞ bank sensor hub, mà chính việc
 * bật/tắt FUNC_CFG_ACCESS lại cắt ngang bộ máy I2C master. Càng hỏi nhiều
 * càng phá. Ta đang phá hỏng đúng thứ mình đang chờ.
 *
 * Cách chắc chắn: đợi một khoảng CỐ ĐỊNH đủ rộng, tuyệt đối không đụng vào
 * bank trong lúc đó, rồi mở ra đúng MỘT LẦN để đọc cả cờ lẫn dữ liệu.
 */
/*
 * ĐÃ ĐO trên bo này: với MAG_SHUB_ODR = 0, cờ ENDOP chỉ xuất hiện mỗi
 * khoảng 600 ms. 150 ms là quá ngắn nên init hỏng lúc được lúc không.
 * Nới MAG_SHUB_ODR cho hub chạy nhanh hơn thì có thể rút con số này xuống.
 */
#define SHUB_SETTLE_MS 400

/**
 * Tắt I2C master và ĐẶT LẠI bộ máy sensor hub.
 *
 * Xoá MASTER_CONFIG về 0 là chưa đủ. ĐÃ ĐO trên bo này: sau một lượt
 * ghi-một-lần, mọi lệnh ĐỌC tiếp theo đều hỏng — dù chính hàm đó vừa đọc
 * được chip ID ngay trước đó. Cơ chế write_once để lại bộ máy ở trạng thái
 * kẹt, và chỉ xung rst_master_regs mới gỡ ra được.
 *
 * Đây là lý do chế độ đọc liên tục không bao giờ chạy sau khi cấu hình từ
 * kế thành công: hub đã kẹt từ lệnh ghi cuối cùng.
 */
static void shub_master_off(void)
{
    if (shub_bank(true)) {
        (void)reg_write(LSM_SH_MASTER_CONFIG, LSM_SH_RST_MASTER_REGS);
        (void)reg_write(LSM_SH_MASTER_CONFIG, 0x00u);
        (void)shub_bank(false);
    }
    delay_us(500);
}

/**
 * Ghi MỘT byte vào thanh ghi của từ kế qua kênh ghi-một-lần.
 *
 * Cờ write_once làm lệnh ghi chạy đúng một lần rồi dừng, thay vì lặp lại mỗi
 * chu kỳ — nếu không thì mỗi nhịp accel lại ghi đè thanh ghi từ kế.
 */
static bool shub_write_reg(uint8_t addr7, uint8_t slave_reg, uint8_t value)
{
    bool    ok;
    uint8_t st = 0;

    if (!shub_bank(true)) {
        return false;
    }
    ok = reg_write(LSM_SH_SLV0_ADD, (uint8_t)(addr7 << 1))   /* bit0=0 -> ghi */
      && reg_write(LSM_SH_SLV0_SUBADD, slave_reg)
      && reg_write(LSM_SH_DATAWRITE_SLV0, value)
      /*
       * numop không dùng khi ghi, NHƯNG trường shub_odr ở bit 7..5 thì có —
       * và để 0 là đặt hub về nhịp CHẬM NHẤT (~600 ms một chu kỳ). Đó chính
       * là lý do đường ghi hỏng trong khi đường đọc chạy tốt: cùng một thời
       * gian chờ, đường đọc đã nới shub_odr còn đường ghi thì chưa.
       */
      && reg_write(LSM_SH_SLV0_CONFIG,
                   (uint8_t)((MAG_SHUB_ODR & 0x07u) << 5))
      && reg_write(LSM_SH_MASTER_CONFIG,
                   (uint8_t)(LSM_SH_WRITE_ONCE | LSM_SH_MASTER_ON |
                             LSM_SH_AUX_SENS_ONE));
    (void)shub_bank(false);

    if (!ok) {
        shub_master_off();
        return false;
    }

    /* Tắt rồi bật accel để sinh sườn kích, sau đó ĐỂ YÊN cho hub chạy. */
    xl_off();
    xl_on();
    HAL_Delay(SHUB_SETTLE_MS);

    if (shub_bank(true)) {
        ok = reg_read(LSM_SH_STATUS_MASTER, &st);
        (void)shub_bank(false);
    } else {
        ok = false;
    }
    /*
     * Chờ ENDOP, KHÔNG phải WR_ONCE.
     *
     * Tôi từng chọn WR_ONCE vì tên nó nghe hợp lý cho lệnh ghi-một-lần, và
     * đường ghi hỏng trong khi đường đọc chạy tốt. Ví dụ chính thức của ST
     * chờ sens_hub_endop cho CẢ HAI chiều — wr_once_done là cờ khác, không
     * phải thứ báo "giao dịch đã xong".
     */
    s_mag_status  = st;
    s_mag_timeout = (ok && (st & LSM_SH_STATUS_ENDOP) == 0u);

    shub_master_off();

    if (!ok || s_mag_timeout) {
        return false;
    }
    if ((st & LSM_SH_STATUS_SLV0_NACK) != 0u) {
        s_mag_nacks++;
        return false;
    }
    return true;
}

/**
 * Đọc n byte liên tiếp từ từ kế vào dst. Dùng lúc init, có chặn.
 *
 * Mở bank ĐÚNG MỘT LẦN ở cuối để lấy cả cờ trạng thái lẫn dữ liệu — xem ghi
 * chú dài ở SHUB_SETTLE_MS về việc vì sao không được hỏi vòng.
 */
static bool shub_read_regs(uint8_t addr7, uint8_t slave_reg, uint8_t *dst, uint8_t n)
{
    bool    ok;
    uint8_t st = 0;

    if (!shub_bank(true)) {
        return false;
    }
    ok = reg_write(LSM_SH_SLV0_ADD, (uint8_t)((addr7 << 1) | LSM_SH_SLV0_READ))
      && reg_write(LSM_SH_SLV0_SUBADD, slave_reg)
      && reg_write(LSM_SH_SLV0_CONFIG,
                   (uint8_t)((n & 0x07u) | ((MAG_SHUB_ODR & 0x07u) << 5)))
      && reg_write(LSM_SH_MASTER_CONFIG,
                   (uint8_t)(LSM_SH_MASTER_ON | LSM_SH_AUX_SENS_ONE));
    (void)shub_bank(false);

    if (!ok) {
        shub_master_off();
        return false;
    }

    xl_off();
    xl_on();
    HAL_Delay(SHUB_SETTLE_MS);

    if (shub_bank(true)) {
        ok = reg_read(LSM_SH_STATUS_MASTER, &st);
        for (uint8_t i = 0; i < n; i++) {
            ok = ok && reg_read((uint8_t)(LSM_SH_SENSOR_HUB_1 + i), &dst[i]);
        }
        (void)shub_bank(false);
    } else {
        ok = false;
    }
    s_mag_status  = st;
    s_mag_timeout = (ok && (st & LSM_SH_STATUS_ENDOP) == 0u);

    shub_master_off();

    if (!ok || s_mag_timeout) {
        return false;
    }
    if ((st & LSM_SH_STATUS_SLV0_NACK) != 0u) {
        s_mag_nacks++;
        return false;
    }
    return true;
}

/* ==========================================================================
 * Khởi tạo từ kế
 * ========================================================================== */

bool lsm6dsv_mag_init(void)
{
    uint8_t id = 0;

    g_fc.mag.healthy    = false;
    g_fc.mag.calibrated = false;
    g_fc.mag.chip_id    = 0;
    s_mag_last_us       = micros();
    s_mag_hz            = 0;
    s_mag_hz_count      = 0;
    s_mag_hz_mark_us    = s_mag_last_us;
    s_mag_nacks         = 0;
    s_mag_bus_busy      = 0;

    /*
     * Accel PHẢI đang chạy — bộ máy sensor hub lấy nhịp từ DRDY của nó.
     * lsm6dsv_init() đã bật ODR trước khi hàm này được gọi.
     */
    s_mag_result  = MAG_INIT_OK;
    s_mag_status  = 0;
    s_mag_timeout = false;

    if (s_state != LSM_STATE_IDLE) {
        s_mag_result = MAG_INIT_NOT_IDLE;
        return false;
    }

    /*
     * --- BẬT ĐIỆN TRỞ KÉO LÊN CHO BUS I2C PHỤ ---
     *
     * Hai chân SDX/SCX nối tới QMC6309 là một bus I2C thật, và I2C cần điện
     * trở kéo lên mới hoạt động. Module tích hợp thường KHÔNG có điện trở
     * ngoài cho đường này vì nó nằm hoàn toàn bên trong — nên phải dùng điện
     * trở nội của LSM6DSV.
     *
     * Không bật thì bus chết câm: mọi giao dịch đều "thành công" nhưng đọc ra
     * toàn 0x00, vì không có gì kéo đường lên mức cao. Đây đúng là triệu
     * chứng đã gặp ở lần chạy đầu.
     *
     * Đọc-sửa-ghi để không đụng các bit khác của IF_CFG.
     */
    {
        uint8_t ifc = 0;

        if (!reg_read(LSM_REG_IF_CFG, &ifc) ||
            !reg_write_verify(LSM_REG_IF_CFG,
                              (uint8_t)(ifc | LSM_IF_CFG_SHUB_PU_EN))) {
            s_mag_result = MAG_INIT_PU_FAIL;
            return false;
        }
    }
    HAL_Delay(2);

    /* --- Xoá cấu hình sensor hub còn sót từ lần nạp trước --- */
    if (shub_bank(true)) {
        (void)reg_write(LSM_SH_MASTER_CONFIG, LSM_SH_RST_MASTER_REGS);
        (void)reg_write(LSM_SH_MASTER_CONFIG, 0x00u);
        (void)shub_bank(false);
    }
    HAL_Delay(2);

    /*
     * --- Dò địa chỉ rồi xác minh đúng chip ---
     *
     * Thử lần lượt hai địa chỉ ứng viên (xem ghi chú ở qmc6309.h). Địa chỉ nào
     * trả về đúng 0x90 thì dùng địa chỉ đó cho toàn bộ phần sau.
     */
    {
        static const uint8_t cand[2] = { QMC_I2C_ADDR_7BIT, QMC_I2C_ADDR_7BIT_ALT };
        bool found = false;

        s_mag_addr = 0;

        for (int i = 0; i < 2 && !found; i++) {
            if (shub_read_regs(cand[i], QMC_REG_CHIP_ID, &id, 1u)) {
                g_fc.mag.chip_id = id;
                if (id == QMC_CHIP_ID_VALUE) {
                    s_mag_addr = cand[i];
                    found = true;
                }
            } else if (s_mag_timeout) {
                /* Hub không chạy — thử địa chỉ khác cũng vô ích. */
                s_mag_result = MAG_INIT_HUB_TIMEOUT;
                return false;
            }
        }

        if (!found) {
            s_mag_result = (g_fc.mag.chip_id != 0u) ? MAG_INIT_BAD_ID
                                                    : MAG_INIT_NACK;
            return false;
        }
    }

    /*
     * --- Đặt lại từ kế ---
     * SOFT_RST không tự xoá, datasheet mục 9.2.4 nói rõ phải ghi 0x80 rồi
     * ghi tiếp 0x00.
     */
    if (!shub_write_reg(s_mag_addr, QMC_REG_CTRL2, QMC_CTRL2_SOFT_RST) ||
        !shub_write_reg(s_mag_addr, QMC_REG_CTRL2, 0x00u)) {
        s_mag_result = MAG_INIT_WRITE_FAIL;
        return false;
    }
    HAL_Delay(10);

    /*
     * --- Cấu hình đo ---
     * CTRL2 trước (dải đo, ODR, chế độ set/reset), CTRL1 sau (bộ lọc và chế
     * độ chạy). Đặt chế độ chạy ở bước cuối để chip không lấy mẫu nào bằng
     * cấu hình dở dang.
     */
    if (!shub_write_reg(s_mag_addr, QMC_REG_CTRL2, MAG_CTRL2_VALUE) ||
        !shub_write_reg(s_mag_addr, QMC_REG_CTRL1, MAG_CTRL1_VALUE)) {
        s_mag_result = MAG_INIT_WRITE_FAIL;
        return false;
    }
    HAL_Delay(10);

    /*
     * --- Đọc ngược lại để chắc chắn lệnh ghi TỚI ĐƯỢC từ kế ---
     *
     * Cờ ENDOP chỉ nói bộ máy sensor hub đã chạy xong một lượt, KHÔNG nói
     * byte có vào đúng thanh ghi của chip hay không. Đọc lại là cách duy
     * nhất biết chắc.
     */
    {
        uint8_t back[2] = { 0, 0 };

        if (shub_read_regs(s_mag_addr, QMC_REG_CTRL1, &back[0], 1u)) {
            dbg_print_hex("  QMC CTRL1 doc lai", back[0], 2);
        } else {
            dbg_println("  QMC CTRL1 doc lai: THAT BAI");
        }
        if (shub_read_regs(s_mag_addr, QMC_REG_CTRL2, &back[1], 1u)) {
            dbg_print_hex("  QMC CTRL2 doc lai", back[1], 2);
        } else {
            dbg_println("  QMC CTRL2 doc lai: THAT BAI");
        }
        dbg_print_hex("  mong doi CTRL1", MAG_CTRL1_VALUE, 2);
        dbg_print_hex("  mong doi CTRL2", MAG_CTRL2_VALUE, 2);
    }

    /*
     * --- Bật chế độ đọc liên tục ---
     * Từ đây hub tự đọc 6 byte dữ liệu của QMC6309 theo nhịp và giữ trong
     * SENSOR_HUB_1..6. Vòng lặp chính chỉ việc lấy ra.
     */
    /*
     * KHÔNG dùng chế độ đọc liên tục.
     *
     * Đã thử và đo kỹ: cấu hình SLV0 đọc 6 byte rồi để master bật liên tục
     * thì dãy SENSOR_HUB KHÔNG BAO GIỜ được nạp lại — mọi thanh ghi đều đúng
     * (MASTER_CONFIG=0x04, SLV0_ADD=0xF9, SLV0_SUBADD=0x01, SLV0_CONFIG=0x86)
     * nhưng STATUS_MASTER đứng ở 0x00 và dữ liệu giữ nguyên giá trị cũ.
     *
     * Trong khi đó đường đọc MỘT LƯỢT (shub_read_regs) chạy hoàn toàn tin
     * cậy. Nên lsm6dsv_mag_update() dùng đúng đường đó, chia thành ba pha
     * để không chặn vòng lặp chính — xem chú thích ở hàm đó.
     */
    s_mag_phase = MAG_PH_TRIGGER;

    /* Hiệu chuẩn: giai đoạn 4 mới đo, giờ chỉ nạp giá trị mặc định. */
    g_fc.mag.calibrated = false;
    return true;
}

/* ==========================================================================
 * Đọc định kỳ
 * ========================================================================== */

/*
 * Một lượt đọc từ kế, chia làm BA PHA để không chặn vòng lặp chính.
 *
 * Vì sao không dùng chế độ đọc liên tục: xem chú thích trong lsm6dsv_mag_init().
 * Tóm lại là nó không quay vòng, còn đường đọc một lượt thì tin cậy.
 *
 * Mỗi pha chỉ tốn khoảng 40 µs trên bus SPI3. Khoảng nghỉ giữa các pha chính
 * là thời gian để bộ máy sensor hub chạy — ta không ngồi đợi nó, mà quay lại
 * lấy kết quả ở lần gọi sau. Ba pha × MAG_PHASE_US cho ra nhịp cuối cùng.
 */
bool lsm6dsv_mag_update(uint32_t now_us)
{
    uint8_t buf[QMC_DATA_LEN];

    if (g_fc.mag.chip_id != QMC_CHIP_ID_VALUE) {
        return false;                        /* init chưa thành công */
    }
    if (fc_elapsed_us(now_us, s_mag_last_us) < MAG_PHASE_US) {
        return false;
    }
    s_mag_last_us = now_us;

    if (!bus_acquire()) {
        return false;
    }

    /* ------------------------------------------------ pha 1: nạp lệnh đọc */
    if (s_mag_phase == MAG_PH_TRIGGER) {
        if (shub_bank(true)) {
            (void)reg_write(LSM_SH_SLV0_ADD,
                            (uint8_t)((s_mag_addr << 1) | LSM_SH_SLV0_READ));
            (void)reg_write(LSM_SH_SLV0_SUBADD, QMC_REG_XOUT_L);
            (void)reg_write(LSM_SH_SLV0_CONFIG,
                            (uint8_t)((QMC_DATA_LEN & 0x07u) |
                                      ((MAG_SHUB_ODR & 0x07u) << 5)));
            (void)reg_write(LSM_SH_MASTER_CONFIG,
                            (uint8_t)(LSM_SH_MASTER_ON | LSM_SH_AUX_SENS_ONE));
            (void)shub_bank(false);
        }
        (void)reg_write(LSM_REG_CTRL1, 0x00u);   /* tắt accel, không chờ */
        bus_release();
        s_mag_phase = MAG_PH_KICK;
        return false;
    }

    /* ------------------------------------------- pha 2: bật lại accel */
    if (s_mag_phase == MAG_PH_KICK) {
        (void)reg_write(LSM_REG_CTRL1, LSM_CTRL1_VALUE);
        bus_release();
        s_mag_phase = MAG_PH_COLLECT;
        return false;
    }

    /* ------------------------------------------------- pha 3: lấy kết quả */
    {
        uint8_t st = 0;
        bool    ok = shub_bank(true);

        if (ok) {
            ok = reg_read(LSM_SH_STATUS_MASTER, &st);
            for (uint8_t i = 0; i < QMC_DATA_LEN; i++) {
                ok = ok && reg_read((uint8_t)(LSM_SH_SENSOR_HUB_1 + i), &buf[i]);
            }
            (void)shub_bank(false);
        }
        /* Xung rst_master_regs — không có nó thì lượt sau sẽ kẹt. */
        if (shub_bank(true)) {
            (void)reg_write(LSM_SH_MASTER_CONFIG, LSM_SH_RST_MASTER_REGS);
            (void)reg_write(LSM_SH_MASTER_CONFIG, 0x00u);
            (void)shub_bank(false);
        }
        bus_release();

        s_mag_status = st;
        s_mag_phase  = MAG_PH_TRIGGER;

        if (!ok) {
            g_fc.mag.error_count++;
            g_fc.mag.healthy = false;
            return false;
        }
        if ((st & LSM_SH_STATUS_SLV0_NACK) != 0u) {
            s_mag_nacks++;
            return false;
        }
        if ((st & LSM_SH_STATUS_ENDOP) == 0u) {
            return false;                    /* chưa xong, bỏ lượt này */
        }
    }

    /* --- Tách số: 16 bit bù hai, BYTE THẤP TRƯỚC (datasheet mục 9.2.1) --- */
    const int16_t rx = le16(&buf[0]);
    const int16_t ry = le16(&buf[2]);
    const int16_t rz = le16(&buf[4]);

    if ((rx | ry | rz) == 0) {
        g_fc.mag.error_count++;
        return false;
    }

    /* --- Đổi thang, vẫn ở hệ CẢM BIẾN --- */
    const vec3f_t sensor_g = {
        (float)rx / MAG_LSB_PER_GAUSS,
        (float)ry / MAG_LSB_PER_GAUSS,
        (float)rz / MAG_LSB_PER_GAUSS
    };

    /*
     * --- Hiệu chuẩn sắt cứng rồi sắt mềm, TRONG HỆ CẢM BIẾN ---
     *
     * Thứ tự này quan trọng: méo từ trường do sắt cứng và sắt mềm gắn với
     * CHIP và với khối kim loại quanh nó, không gắn với hướng lắp. Hiệu chuẩn
     * ở hệ cảm biến rồi mới xoay trục nghĩa là sau này sửa MAG_AXIS_* cũng
     * không làm hỏng bộ số hiệu chuẩn.
     *
     * Giai đoạn 4 mới đo bộ số này; tới lúc đó offset = 0 và scale = 1 nên
     * hai dòng dưới không đổi gì.
     */
    const float cal[3] = {
        (sensor_g.x - MAG_OFFSET_X_G) * MAG_SCALE_X,
        (sensor_g.y - MAG_OFFSET_Y_G) * MAG_SCALE_Y,
        (sensor_g.z - MAG_OFFSET_Z_G) * MAG_SCALE_Z
    };

    /* --- Rồi mới xoay sang hệ thân --- */
    const vec3f_t field = {
        (float)(MAG_AXIS_SIGN_X) * cal[MAG_AXIS_MAP_X],
        (float)(MAG_AXIS_SIGN_Y) * cal[MAG_AXIS_MAP_Y],
        (float)(MAG_AXIS_SIGN_Z) * cal[MAG_AXIS_MAP_Z]
    };

    s_mag_hz_count++;
    if (fc_elapsed_us(now_us, s_mag_hz_mark_us) >= 1000000u) {
        s_mag_hz         = s_mag_hz_count;
        s_mag_hz_count   = 0;
        s_mag_hz_mark_us = now_us;
    }

    g_fc.mag.raw             = (vec3i16_t){ rx, ry, rz };
    g_fc.mag.raw_gauss       = sensor_g;
    g_fc.mag.field_gauss     = field;
    g_fc.mag.magnitude_gauss = vec3f_norm(field);
    g_fc.mag.overflow = (rx > 32000 || rx < -32000 ||
                         ry > 32000 || ry < -32000 ||
                         rz > 32000 || rz < -32000);
    g_fc.mag.timestamp_us = now_us;
    g_fc.mag.sample_count++;
    g_fc.mag.healthy = true;
    return true;
}

mag_init_result_t lsm6dsv_mag_init_result(void) { return s_mag_result; }
uint8_t           lsm6dsv_mag_last_status(void) { return s_mag_status; }

const char *lsm6dsv_mag_init_result_name(void)
{
    switch (s_mag_result) {
    case MAG_INIT_OK:          return "OK";
    case MAG_INIT_NOT_IDLE:    return "goi sai luc (driver chua IDLE)";
    case MAG_INIT_PU_FAIL:     return "khong bat duoc pull-up bus phu";
    case MAG_INIT_HUB_TIMEOUT: return "sensor hub khong bao xong - accel co chay khong?";
    case MAG_INIT_NACK:        return "khong ai tra loi o dia chi 0x7C";
    case MAG_INIT_BAD_ID:      return "doc duoc nhung chip ID khac 0x90";
    case MAG_INIT_WRITE_FAIL:  return "ghi thanh ghi cau hinh that bai";
    case MAG_INIT_CFG_FAIL:    return "bat che do doc lien tuc that bai";
    default:                   return "?";
    }
}

uint8_t  lsm6dsv_mag_addr(void)      { return s_mag_addr; }
uint32_t lsm6dsv_mag_rate_hz(void)   { return s_mag_hz; }
uint32_t lsm6dsv_mag_nacks(void)     { return s_mag_nacks; }
uint32_t lsm6dsv_mag_bus_busy(void)  { return s_mag_bus_busy; }

/* ==========================================================================
 * Đổ thanh ghi để gỡ lỗi sensor hub
 * ========================================================================== */

void lsm6dsv_mag_dump(void)
{
    uint8_t v = 0;

    dbg_println("");
    dbg_println("--- DUMP SENSOR HUB ---");

    /* --- Thanh ghi thường: chip có sống và accel có chạy không --- */
    if (reg_read(LSM_REG_WHO_AM_I, &v))  { dbg_print_hex("WHO_AM_I ", v, 2); }
    if (reg_read(LSM_REG_CTRL1, &v))     { dbg_print_hex("CTRL1 (accel ODR)", v, 2); }
    if (reg_read(LSM_REG_CTRL2, &v))     { dbg_print_hex("CTRL2 (gyro ODR) ", v, 2); }
    if (reg_read(LSM_REG_CTRL3, &v))     { dbg_print_hex("CTRL3            ", v, 2); }
    if (reg_read(LSM_REG_CTRL4, &v))     { dbg_print_hex("CTRL4            ", v, 2); }
    if (reg_read(LSM_REG_IF_CFG, &v))    { dbg_print_hex("IF_CFG (bit6=PU) ", v, 2); }
    if (reg_read(LSM_REG_STATUS, &v))    { dbg_print_hex("STATUS_REG       ", v, 2); }
    if (reg_read(LSM_REG_FUNC_CFG_ACCESS, &v)) {
        dbg_print_hex("FUNC_CFG_ACCESS  ", v, 2);
    }

    /* --- Kiểm tra bank sensor hub có chuyển được không --- */
    dbg_println("");
    if (!shub_bank(true)) {
        dbg_println("KHONG MO DUOC BANK SHUB - ghi FUNC_CFG_ACCESS that bai");
        return;
    }
    if (reg_read(LSM_REG_FUNC_CFG_ACCESS, &v)) {
        dbg_print_hex("FUNC_CFG_ACCESS sau khi mo bank", v, 2);
    }
    if (reg_read(LSM_SH_MASTER_CONFIG, &v)) { dbg_print_hex("MASTER_CONFIG", v, 2); }
    if (reg_read(LSM_SH_SLV0_ADD, &v))      { dbg_print_hex("SLV0_ADD     ", v, 2); }
    if (reg_read(LSM_SH_SLV0_SUBADD, &v))   { dbg_print_hex("SLV0_SUBADD  ", v, 2); }
    if (reg_read(LSM_SH_SLV0_CONFIG, &v))   { dbg_print_hex("SLV0_CONFIG  ", v, 2); }
    if (reg_read(LSM_SH_STATUS_MASTER, &v)) { dbg_print_hex("STATUS_MASTER", v, 2); }
    (void)shub_bank(false);

    /*
     * --- Dò chip ID NHIỀU LẦN ---
     *
     * Phân biệt hai chuyện rất khác nhau:
     *   NACK ở MỌI lần   -> từ kế không có nguồn, hoặc chết hẳn
     *   NACK LÚC CÓ LÚC KHÔNG -> nhiễu trên bus I2C phụ, điện trở kéo lên
     *                            nội của LSM6DSV quá yếu để chống lại
     */
    dbg_println("");
    dbg_println("Do chip ID 10 lan lien tiep @ 0x7C:");

    {
        int ok_cnt = 0;

        for (int i = 0; i < 10; i++) {
            uint8_t id = 0;
            const bool ok = shub_read_regs(QMC_I2C_ADDR_7BIT,
                                           QMC_REG_CHIP_ID, &id, 1u);

            if (ok && id == QMC_CHIP_ID_VALUE) { ok_cnt++; }

            dbg_print_hex("  id ", id, 2);
            dbg_print_hex("     STATUS_MASTER", s_mag_status, 2);
            HAL_Delay(50);
        }
        dbg_print_int("  so lan doc DUNG (0x90) tren 10", ok_cnt);

        /*
         * Quet mot vai dia chi hay gap cua tu ke QST/Honeywell.
         * Neu KHONG dia chi nao tra loi thi bus song nhung khong co thiet bi
         * nao co dien - loai tru han kha nang "sai dia chi".
         */
        {
            static const uint8_t cand[] = {
                0x7C, 0x3E, 0x0C, 0x0D, 0x1C, 0x2C, 0x1E, 0x30
            };
            int found = 0;

            dbg_println("  Quet dia chi tren bus phu:");
            for (unsigned i = 0; i < sizeof(cand); i++) {
                uint8_t id = 0;

                if (shub_read_regs(cand[i], 0x00u, &id, 1u)) {
                    dbg_print_hex("    CO TRA LOI @", cand[i], 2);
                    dbg_print_hex("       doc duoc", id, 2);
                    found++;
                }
            }
            if (found == 0) {
                dbg_println("    khong dia chi nao tra loi.");
            }
        }

        if (ok_cnt == 0) {
            dbg_println("  -> KHONG LAN NAO. Tu ke mat nguon hoac chet.");
        } else if (ok_cnt < 10) {
            dbg_println("  -> CHAP CHON. Nhieu tren bus I2C phu; dien tro keo");
            dbg_println("     len noi cua LSM6DSV qua yeu. Can thu lai nhieu lan.");
        } else {
            dbg_println("  -> LAN NAO CUNG DUOC. Loi chi xay ra luc khoi dong.");
        }
    }

    shub_master_off();
    dbg_println("--- HET DUMP ---");
    dbg_println("");
}
