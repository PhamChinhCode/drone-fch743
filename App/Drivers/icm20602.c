/**
 * @file    icm20602.c
 * @brief   Hiện thực driver ICM-20602.
 */

#include "icm20602.h"
#include "imu_noise.h"
#include "fc_state.h"
#include "param_table.h"
#include "fc_time.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

/* ==========================================================================
 * Chuyển tham số cấu hình thành giá trị thanh ghi
 * ========================================================================== */

/*
 * Dai do gio la THAM SO RUNTIME nen viec chon bit thanh ghi phai lam luc
 * chay, khong con #error luc bien dich duoc nua.
 *
 * Doi lai bang cach KHAC: gia tri khong hop le KHONG bi tu choi im lang ma
 * duoc lam tron XUONG muc hop le gan nhat, va ham tra ve luon di doi voi
 * ham tinh thang do ben duoi - hai thu nay doc cung mot bang nen khong the
 * lech nhau. Dat 700 dps thi chip chay 500 dps VA thang do cung la 500.
 *
 * (min/max trong param_list.h da chan ngoai dai; day chan not cac gia tri
 * nam trong dai nhung khong phai muc chip ho tro.)
 */
typedef struct {
    uint16_t value;   /* dps hoac g */
    uint8_t  fs_sel;  /* bit 4..3 cua thanh ghi CONFIG */
} icm_fs_entry_t;

static const icm_fs_entry_t ICM_GYRO_FS[] = {
    { 250u, 0u }, { 500u, 1u }, { 1000u, 2u }, { 2000u, 3u },
};
static const icm_fs_entry_t ICM_ACCEL_FS[] = {
    { 2u, 0u }, { 4u, 1u }, { 8u, 2u }, { 16u, 3u },
};

/** Muc ho tro lon nhat KHONG vuot qua `want`; khong co thi lay muc thap nhat. */
static const icm_fs_entry_t *fs_pick(const icm_fs_entry_t *tab, size_t n,
                                     uint16_t want)
{
    const icm_fs_entry_t *best = &tab[0];

    for (size_t i = 0; i < n; i++) {
        if (tab[i].value <= want) {
            best = &tab[i];
        }
    }
    return best;
}

/*
 * DLPF_CFG = 0 kèm FCHOICE_B = 00: băng thông gyro 250 Hz, tốc độ ra 8 kHz.
 * Đây là cấu hình chuẩn cho máy bay điều khiển ở 4-8 kHz.
 */
#define ICM_GYRO_DLPF_CFG   0u
#define ICM_ACCEL_DLPF_CFG  0u    /* băng thông accel 218,1 Hz, ODR 1 kHz */

/*
 * He so doi thang: gia tri tho 16-bit co dau -> don vi vat ly.
 *
 * Truoc day la macro hang so. Gio la bien tinh san trong icm20602_init(),
 * lay tu CHINH muc dai do da ghi vao chip - khong phai tu tham so nguoi dung
 * yeu cau. Neu hai thu nay lech nhau thi moi so do deu sai theo mot ti le co
 * dinh, va do la kieu sai rat kho phat hien vi may bay van bay, chi la sai
 * he so PID.
 */
static float s_gyro_scale;
static float s_accel_scale;

/* Nhiệt độ: T[°C] = raw / 326,8 + 25 (theo datasheet mục 4.20). */
#define ICM_TEMP_SCALE   (1.0f / 326.8f)
#define ICM_TEMP_OFFSET  25.0f

/*
 * Điều khiển chân CS bằng thanh ghi BSRR: một lệnh ghi duy nhất, không đọc
 * lại, an toàn khi bị ngắt cắt ngang. Nhanh hơn HAL_GPIO_WritePin và đường
 * dữ liệu này chạy ở 8 kHz nên đáng để tối ưu.
 */
#define CS_LOW()   (SPI1_SS_GPIO_Port->BSRR = (uint32_t)SPI1_SS_Pin << 16u)
#define CS_HIGH()  (SPI1_SS_GPIO_Port->BSRR = (uint32_t)SPI1_SS_Pin)

/* ==========================================================================
 * Biến nội bộ
 *
 * Hai bộ đệm SPI là nguồn/đích trực tiếp của DMA1_Stream0 và DMA1_Stream1
 * nên BẮT BUỘC dùng FC_DMA_BUFFER (xem chú thích ở fc_types.h). Nếu để mặc
 * định chúng rơi vào DTCMRAM và DMA không đọc ghi được.
 * ========================================================================== */

FC_DMA_BUFFER static uint8_t s_tx[1u + ICM_BURST_DATA_LEN];
FC_DMA_BUFFER static uint8_t s_rx[1u + ICM_BURST_DATA_LEN];

static volatile icm20602_state_t s_state = ICM_STATE_UNINIT;
static volatile bool     s_busy;            /* đang có transfer DMA chạy   */
static volatile uint32_t s_sample_us;       /* mốc thời gian lúc DRDY      */
static uint32_t          s_prev_sample_us;
static uint32_t          s_overrun_count;   /* DRDY tới khi DMA chưa xong  */

static float s_gyro_alpha;                  /* hệ số lọc gyro              */
static float s_accel_alpha;
static uint8_t s_gyro_fs_sel;               /* bit dai do da ghi vao chip  */
static uint8_t s_accel_fs_sel;

#if IMU_NOISE_STATS_ENABLE
/* Nen nhieu gyro, phuc vu so sanh voi LSM6DSV o giai doan 2.
 * Xem App/Common/imu_noise.h. Tat bang IMU_NOISE_STATS_ENABLE = 0. */
static imu_noise_t s_noise;
#endif

/** Bộ tích luỹ cho quá trình lấy bias gyro. */
static struct {
    uint32_t count;
    float    sum[AXIS_COUNT];
    float    sumsq[AXIS_COUNT];   /* để tính độ lệch chuẩn, xem calibration_feed */
} s_cal;

/* ==========================================================================
 * Truy cập thanh ghi ở chế độ hỏi vòng (chỉ dùng lúc init)
 *
 * HAL_SPI_TransmitReceive bản chặn dùng CPU chép dữ liệu, không qua DMA,
 * nên bộ đệm nằm trên stack (DTCMRAM) vẫn hoàn toàn hợp lệ ở đây.
 * ========================================================================== */

static bool reg_write(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7Fu), value };

    CS_LOW();
    const HAL_StatusTypeDef st =
        HAL_SPI_Transmit(&hspi1, tx, sizeof(tx), IMU_SPI_TIMEOUT_MS);
    CS_HIGH();

    /* Datasheet yêu cầu tối thiểu 100 ns giữa hai lần chọn chip. */
    delay_us(2);
    return (st == HAL_OK);
}

static bool reg_read(uint8_t reg, uint8_t *value)
{
    uint8_t tx[2] = { (uint8_t)(reg | ICM_SPI_READ_BIT), 0x00u };
    uint8_t rx[2] = { 0, 0 };

    CS_LOW();
    const HAL_StatusTypeDef st =
        HAL_SPI_TransmitReceive(&hspi1, tx, rx, sizeof(tx), IMU_SPI_TIMEOUT_MS);
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

/** Đổi tốc độ SPI1 giữa lúc chạy. HAL_SPI_Init tự tắt/bật lại ngoại vi. */
static bool spi_set_baud(uint32_t prescaler)
{
    hspi1.Init.BaudRatePrescaler = prescaler;
    return (HAL_SPI_Init(&hspi1) == HAL_OK);
}

/* ==========================================================================
 * Khởi tạo
 * ========================================================================== */

bool icm20602_init(void)
{
    uint8_t who = 0;

    s_state = ICM_STATE_UNINIT;
    s_busy  = false;
    memset(s_tx, 0, sizeof(s_tx));
    memset(s_rx, 0, sizeof(s_rx));
    memset(&s_cal, 0, sizeof(s_cal));

    /* Hệ số lọc tính sẵn theo tốc độ lấy mẫu danh định, tránh chia trong ISR. */
    const float dt = 1.0f / (float)IMU_SAMPLE_RATE_HZ;
    s_gyro_alpha  = fc_lpf_alpha(g_params.imu_gyro_lpf_hz,  dt);
    s_accel_alpha = fc_lpf_alpha(g_params.imu_accel_lpf_hz, dt);

    /*
     * Chot dai do MOT LAN o day. Thang do tinh tu muc that su duoc chon, nen
     * no khong the lech khoi thanh ghi da ghi xuong chip.
     */
    const icm_fs_entry_t *gfs = fs_pick(ICM_GYRO_FS,
                                        sizeof(ICM_GYRO_FS) / sizeof(ICM_GYRO_FS[0]),
                                        g_params.imu_gyro_fs_dps);
    const icm_fs_entry_t *afs = fs_pick(ICM_ACCEL_FS,
                                        sizeof(ICM_ACCEL_FS) / sizeof(ICM_ACCEL_FS[0]),
                                        g_params.imu_accel_fs_g);

    s_gyro_fs_sel  = gfs->fs_sel;
    s_accel_fs_sel = afs->fs_sel;
    s_gyro_scale   = (float)gfs->value / 32768.0f;
    s_accel_scale  = ((float)afs->value / 32768.0f) * FC_GRAVITY_MPS2;

#if IMU_NOISE_STATS_ENABLE
    imu_noise_reset(&s_noise, micros());
#endif

    CS_HIGH();

    /* Ghi thanh ghi cấu hình ở 1 MHz cho chắc ăn, tuy chip chịu được 10 MHz. */
    if (!spi_set_baud(SPI_BAUDRATEPRESCALER_64)) {
        goto fail;
    }
    HAL_Delay(1);

    /* --- Đặt lại toàn bộ chip --- */
    if (!reg_write(ICM_REG_PWR_MGMT_1, 0x80u)) {   /* DEVICE_RESET */
        goto fail;
    }
    HAL_Delay(100);

    /* Chờ bit reset tự xoá. */
    for (int i = 0; i < 50; i++) {
        uint8_t pwr = 0xFFu;
        if (reg_read(ICM_REG_PWR_MGMT_1, &pwr) && (pwr & 0x80u) == 0) {
            break;
        }
        HAL_Delay(2);
    }

    /* Đặt lại đường tín hiệu của gyro, accel và cảm biến nhiệt. */
    if (!reg_write(ICM_REG_SIGNAL_PATH_RESET, 0x07u)) {
        goto fail;
    }
    HAL_Delay(100);

    /*
     * Tắt hẳn giao diện I2C. Bắt buộc với ICM-20602 nối SPI: nếu bỏ qua,
     * chip có thể tự chuyển sang chế độ I2C khi thấy nhiễu trên bus và
     * ngừng đáp ứng lệnh SPI.
     */
    if (!reg_write(ICM_REG_USER_CTRL, 0x10u)) {     /* I2C_IF_DIS */
        goto fail;
    }

    /* Rời chế độ ngủ, chọn nguồn clock tự động (PLL theo gyro nếu sẵn sàng). */
    if (!reg_write(ICM_REG_PWR_MGMT_1, 0x01u)) {
        goto fail;
    }
    HAL_Delay(15);

    /* --- Xác minh đúng chip --- */
    if (!reg_read(ICM_REG_WHO_AM_I, &who) || who != ICM_WHO_AM_I_VALUE) {
        goto fail;
    }

    /* --- Cấu hình đo --- */
    if (!reg_write_verify(ICM_REG_PWR_MGMT_2,    0x00u) ||   /* bật đủ 6 trục   */
        !reg_write_verify(ICM_REG_CONFIG,        ICM_GYRO_DLPF_CFG) ||
        !reg_write_verify(ICM_REG_SMPLRT_DIV,    0x00u) ||
        !reg_write_verify(ICM_REG_GYRO_CONFIG,   (uint8_t)(s_gyro_fs_sel << 3)) ||
        !reg_write_verify(ICM_REG_ACCEL_CONFIG,  (uint8_t)(s_accel_fs_sel << 3)) ||
        !reg_write_verify(ICM_REG_ACCEL_CONFIG2, ICM_ACCEL_DLPF_CFG) ||
        !reg_write_verify(ICM_REG_FIFO_EN,       0x00u)) {   /* không dùng FIFO */
        goto fail;
    }

    /*
     * INT_PIN_CFG = 0x00: chân INT tích cực mức cao, đẩy kéo, phát xung 50 µs
     * rồi tự về thấp. Không cần đọc INT_STATUS để xoá cờ, hợp với EXTI4 bắt
     * sườn lên và điện trở kéo xuống trên PC4.
     */
    if (!reg_write_verify(ICM_REG_INT_PIN_CFG, 0x00u) ||
        !reg_write_verify(ICM_REG_INT_ENABLE,  0x01u)) {     /* DATA_RDY_INT_EN */
        goto fail;
    }

    /* Xong phần cấu hình, chuyển sang 8 MHz cho đường dữ liệu. */
    if (!spi_set_baud(SPI_BAUDRATEPRESCALER_8)) {
        goto fail;
    }

    /* Xoá cờ ngắt còn treo trước khi cho phép DRDY. */
    {
        uint8_t dummy = 0;
        (void)reg_read(ICM_REG_INT_STATUS, &dummy);
    }

    s_tx[0] = (uint8_t)(ICM_REG_ACCEL_XOUT_H | ICM_SPI_READ_BIT);

    g_fc.imu.calibrated = false;
    g_fc.imu.healthy    = false;
    s_state             = ICM_STATE_IDLE;
    return true;

fail:
    s_state = ICM_STATE_ERROR;
    g_fc.imu.healthy = false;
    fc_state_set_error(FC_ERR_IMU_SPI);
    return false;
}

void icm20602_start(void)
{
    if (s_state == ICM_STATE_IDLE) {
        s_prev_sample_us = micros();
        s_state = ICM_STATE_RUNNING;
    }
}

void icm20602_stop(void)
{
    if (s_state == ICM_STATE_RUNNING || s_state == ICM_STATE_CALIBRATING) {
        s_state = ICM_STATE_IDLE;
    }
}

icm20602_state_t icm20602_get_state(void)
{
    return s_state;
}

/* ==========================================================================
 * Hiệu chuẩn bias gyro
 * ========================================================================== */

void icm20602_start_gyro_calibration(void)
{
    if (s_state != ICM_STATE_RUNNING && s_state != ICM_STATE_CALIBRATING) {
        return;
    }

    memset(&s_cal, 0, sizeof(s_cal));

    g_fc.imu.calibrated    = false;
    g_fc.imu.gyro_bias_dps = (vec3f_t){ 0.0f, 0.0f, 0.0f };
    s_state = ICM_STATE_CALIBRATING;
}

uint8_t icm20602_calibration_progress(void)
{
    if (s_state != ICM_STATE_CALIBRATING) {
        return g_fc.imu.calibrated ? 100u : 0u;
    }
    return (uint8_t)((s_cal.count * 100u) / g_params.imu_calib_sample_count);
}

/** Nạp một mẫu vào bộ tích luỹ. Trả về true khi đã đủ số mẫu. */
static void calibration_feed(const float gyro[AXIS_COUNT])
{
    for (int i = 0; i < AXIS_COUNT; i++) {
        s_cal.sum[i]   += gyro[i];
        s_cal.sumsq[i] += gyro[i] * gyro[i];
    }
    s_cal.count++;

    /*
     * Máy bay chưa đứng yên thì bias tính ra sẽ lệch. Đo bằng ĐỘ LỆCH CHUẨN,
     * không phải biên độ đỉnh-đỉnh — xem imu_calib_move_sd_dps trong
     * fc_config.h để biết vì sao.
     *
     * So bình phương với bình phương để khỏi phải gọi sqrtf trong ISR 8 kHz.
     */
    if (s_cal.count >= IMU_CALIB_SD_MIN_SAMPLES) {
        const float inv = 1.0f / (float)s_cal.count;

        for (int i = 0; i < AXIS_COUNT; i++) {
            const float mean = s_cal.sum[i] * inv;
            const float var  = s_cal.sumsq[i] * inv - mean * mean;

            if (var > (g_params.imu_calib_move_sd_dps *
                       g_params.imu_calib_move_sd_dps)) {
                memset(&s_cal, 0, sizeof(s_cal));
                return;
            }
        }
    }

    if (s_cal.count >= g_params.imu_calib_sample_count) {
        const float inv = 1.0f / (float)s_cal.count;
        g_fc.imu.gyro_bias_dps.x = s_cal.sum[AXIS_ROLL]  * inv;
        g_fc.imu.gyro_bias_dps.y = s_cal.sum[AXIS_PITCH] * inv;
        g_fc.imu.gyro_bias_dps.z = s_cal.sum[AXIS_YAW]   * inv;
        g_fc.imu.calibrated      = true;
        s_state                  = ICM_STATE_RUNNING;
    }
}

/* ==========================================================================
 * Xử lý một mẫu
 * ========================================================================== */

/** Đọc số nguyên 16-bit có dấu, kiểu big-endian như chip xuất ra. */
static inline int16_t be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

/**
 * Xoay từ hệ trục cảm biến sang hệ trục thân máy bay.
 * Cau hinh bang imu_axis_map_* va imu_axis_sign_*, doi duoc luc chay.
 *
 * Doc thang g_params trong ham chay o 8 kHz: doc mot truong toan cuc dung
 * bang chi phi doc mot hang so tu flash, va doi lai la khong co ban sao nao
 * de quen dong bo.
 */
static inline void align_axes(const float in[3], float out[3])
{
    out[AXIS_ROLL]  = (float)g_params.imu_axis_sign_x * in[g_params.imu_axis_map_x];
    out[AXIS_PITCH] = (float)g_params.imu_axis_sign_y * in[g_params.imu_axis_map_y];
    out[AXIS_YAW]   = (float)g_params.imu_axis_sign_z * in[g_params.imu_axis_map_z];
}

static void process_sample(void)
{
    /* --- Tách dữ liệu thô (byte 0 là byte rác ứng với lúc gửi địa chỉ) --- */
    const int16_t ax_raw = be16(&s_rx[1]);
    const int16_t ay_raw = be16(&s_rx[3]);
    const int16_t az_raw = be16(&s_rx[5]);
    const int16_t t_raw  = be16(&s_rx[7]);
    const int16_t gx_raw = be16(&s_rx[9]);
    const int16_t gy_raw = be16(&s_rx[11]);
    const int16_t gz_raw = be16(&s_rx[13]);

    /*
     * Toàn bộ giá trị bằng 0 hoặc bằng 0xFFFF thường là dấu hiệu mất kết nối
     * (MISO treo cao hoặc thấp) chứ không phải số đo thật.
     */
    if ((gx_raw | gy_raw | gz_raw | ax_raw | ay_raw | az_raw) == 0) {
        g_fc.imu.error_count++;
        return;
    }

    /* --- Đổi thang rồi xoay trục --- */
    const float gyro_sensor[3] = {
        (float)gx_raw * s_gyro_scale,
        (float)gy_raw * s_gyro_scale,
        (float)gz_raw * s_gyro_scale
    };
    const float accel_sensor[3] = {
        (float)ax_raw * s_accel_scale,
        (float)ay_raw * s_accel_scale,
        (float)az_raw * s_accel_scale
    };

    float gyro_body[3];
    float accel_body[3];
    align_axes(gyro_sensor,  gyro_body);
    align_axes(accel_sensor, accel_body);

    /* --- Đang hiệu chuẩn thì chỉ tích luỹ, chưa xuất số liệu --- */
    if (s_state == ICM_STATE_CALIBRATING) {
        calibration_feed(gyro_body);
    }

    /* --- Trừ bias --- */
    const vec3f_t bias = g_fc.imu.gyro_bias_dps;
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

#if IMU_NOISE_STATS_ENABLE
    /* Nap gyro CHUA LOC - do moi la nhieu that. */
    imu_noise_feed(&s_noise, gyro, s_sample_us);
#endif

    /* --- Lọc thông thấp --- */
    vec3f_t filt = g_fc.imu.gyro_filtered_dps;
    filt.x = fc_lpf(filt.x, gyro.x, s_gyro_alpha);
    filt.y = fc_lpf(filt.y, gyro.y, s_gyro_alpha);
    filt.z = fc_lpf(filt.z, gyro.z, s_gyro_alpha);

    vec3f_t accel_filt = g_fc.imu.accel_mps2;
    accel_filt.x = fc_lpf(accel_filt.x, accel.x, s_accel_alpha);
    accel_filt.y = fc_lpf(accel_filt.y, accel.y, s_accel_alpha);
    accel_filt.z = fc_lpf(accel_filt.z, accel.z, s_accel_alpha);

    /* --- Ghi vào khối trạng thái chung --- */
    g_fc.imu.gyro_raw          = (vec3i16_t){ gx_raw, gy_raw, gz_raw };
    g_fc.imu.accel_raw         = (vec3i16_t){ ax_raw, ay_raw, az_raw };
    g_fc.imu.gyro_dps          = gyro;
    g_fc.imu.gyro_filtered_dps = filt;
    g_fc.imu.accel_mps2        = accel_filt;
    g_fc.imu.temperature_c     = (float)t_raw * ICM_TEMP_SCALE + ICM_TEMP_OFFSET;

    g_fc.imu.dt_us        = fc_elapsed_us(s_sample_us, s_prev_sample_us);
    s_prev_sample_us      = s_sample_us;
    g_fc.imu.timestamp_us = s_sample_us;
    g_fc.imu.sample_count++;
    g_fc.imu.healthy = true;
}

/* ==========================================================================
 * Hàm gọi từ ngắt
 * ========================================================================== */

void icm20602_drdy_isr(void)
{
    if (s_state != ICM_STATE_RUNNING && s_state != ICM_STATE_CALIBRATING) {
        return;
    }

    /*
     * Mẫu trước chưa đọc xong mà mẫu mới đã tới. Ở 8 kHz mỗi transfer 15 byte
     * ở 8 MHz chỉ mất ~15 µs trên tổng 125 µs nên chuyện này chỉ xảy ra khi
     * có ngắt khác giữ CPU quá lâu. Bỏ mẫu này, không xếp hàng chờ.
     */
    if (s_busy) {
        s_overrun_count++;
        g_fc.imu.error_count++;
        return;
    }

    s_busy      = true;
    s_sample_us = micros();
    s_tx[0]     = (uint8_t)(ICM_REG_ACCEL_XOUT_H | ICM_SPI_READ_BIT);

    CS_LOW();
    if (HAL_SPI_TransmitReceive_DMA(&hspi1, s_tx, s_rx,
                                    1u + ICM_BURST_DATA_LEN) != HAL_OK) {
        CS_HIGH();
        s_busy = false;
        g_fc.imu.error_count++;
        fc_state_set_error(FC_ERR_IMU_SPI);
    }
}

void icm20602_spi_complete_isr(void)
{
    CS_HIGH();
    s_busy = false;
    process_sample();
}

void icm20602_spi_error_isr(void)
{
    CS_HIGH();
    HAL_SPI_Abort(&hspi1);
    s_busy = false;

    g_fc.imu.error_count++;
    g_fc.imu.healthy = false;
    fc_state_set_error(FC_ERR_IMU_SPI);
}

/* ==========================================================================
 * Nền nhiễu — công cụ của giai đoạn 2
 * ========================================================================== */

float icm20602_gyro_sigma_dps(void)
{
#if IMU_NOISE_STATS_ENABLE
    return imu_noise_sigma_max(&s_noise);
#else
    return 0.0f;
#endif
}

vec3f_t icm20602_gyro_sigma_axes_dps(void)
{
#if IMU_NOISE_STATS_ENABLE
    return (vec3f_t){ s_noise.sigma[0], s_noise.sigma[1], s_noise.sigma[2] };
#else
    return (vec3f_t){ 0.0f, 0.0f, 0.0f };
#endif
}
