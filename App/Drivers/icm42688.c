/**
 * @file    icm42688.c
 * @brief   Hiện thực driver ICM-42688-P.
 *
 * Dựng lại từ icm20602.c: đường dữ liệu (DRDY -> DMA -> xử lý trong ngắt),
 * bộ lọc, notch, hiệu chuẩn bias giữ NGUYÊN. Chỉ khác phần cấu hình thanh ghi
 * và thứ tự byte trong khối dữ liệu — xem đầu icm42688.h.
 */

#include "icm42688.h"
#include "imu_noise.h"
#include "vib_stream.h"
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
    uint8_t  fs_sel;  /* bit 7..5 cua GYRO_CONFIG0 / ACCEL_CONFIG0 */
} icm_fs_entry_t;

/* Ma NGUOC so voi ICM-20602: 0 la dai LON nhat. Bang van xep tang dan theo
 * value vi fs_pick() can thu tu do. */
static const icm_fs_entry_t ICM_GYRO_FS[] = {
    { 250u, 3u }, { 500u, 2u }, { 1000u, 1u }, { 2000u, 0u },
};
static const icm_fs_entry_t ICM_ACCEL_FS[] = {
    { 2u, 3u }, { 4u, 2u }, { 8u, 1u }, { 16u, 0u },
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

/* ODR 8 kHz cho ca gyro lan accel (ma 3 trong bit 3..0 cua *_CONFIG0). */
#define ICM42_ODR_8KHZ      0x03u

/*
 * Bo loc chong rang cua (AAF) ~258 Hz cho CA gyro lan accel — gan nhat voi
 * DLPF 250 Hz cua ICM-20602 truoc day, de cac bo loc phia sau (PT1 100 Hz,
 * notch 220 Hz) va he so PID khong doi y nghia. Bo ba so lay tu bang AAF
 * trong datasheet: DELT = 6, DELTSQR = 36, BITSHIFT = 10.
 */
#define ICM42_AAF_DELT      6u
#define ICM42_AAF_DELTSQR   36u
#define ICM42_AAF_BITSHIFT  10u

/*
 * Bo loc UI sau AAF: che do "low latency" (ma 14) cho ca hai — chi lay mau
 * thua dau ra Dec2, khong them bac loc nao. Loc da co AAF va PT1/notch phia
 * sau lo; them mot tang nua chi them tre pha cho vong rate.
 */
#define ICM42_UI_FILT_LOW_LATENCY  0xEEu

/*
 * He so doi thang: gia tri tho 16-bit co dau -> don vi vat ly.
 *
 * Truoc day la macro hang so. Gio la bien tinh san trong icm42688_init(),
 * lay tu CHINH muc dai do da ghi vao chip - khong phai tu tham so nguoi dung
 * yeu cau. Neu hai thu nay lech nhau thi moi so do deu sai theo mot ti le co
 * dinh, va do la kieu sai rat kho phat hien vi may bay van bay, chi la sai
 * he so PID.
 */
static float s_gyro_scale;
static float s_accel_scale;

/* Nhiệt độ: T[°C] = raw / 132,48 + 25 (datasheet ICM-42688-P). */
#define ICM_TEMP_SCALE   (1.0f / 132.48f)
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

FC_DMA_BUFFER static uint8_t s_tx[1u + ICM42_BURST_DATA_LEN];
FC_DMA_BUFFER static uint8_t s_rx[1u + ICM42_BURST_DATA_LEN];

static volatile icm42688_state_t s_state = ICM_STATE_UNINIT;
static volatile bool     s_busy;            /* đang có transfer DMA chạy   */
static volatile uint32_t s_sample_us;       /* mốc thời gian lúc DRDY      */
static uint32_t          s_prev_sample_us;
static uint32_t          s_overrun_count;   /* DRDY tới khi DMA chưa xong  */

static float s_gyro_alpha;                  /* hệ số lọc gyro              */
static float s_accel_alpha;

/*
 * Trạng thái LPF gyro tách riêng khỏi g_fc.imu.gyro_filtered_dps: giờ sau LPF
 * còn notch, nên số xuất ra không còn là trạng thái của LPF nữa.
 */
static vec3f_t s_gyro_lpf;

/* Notch biquad (RBJ), dạng trực tiếp II chuyển vị. Hệ số đã chia a0. */
static struct {
    bool  on;
    float b0, b1, b2, a1, a2;
    float z1[AXIS_COUNT], z2[AXIS_COUNT];
} s_notch;

static void notch_init(float f0_hz, float q, float fs_hz)
{
    memset(&s_notch, 0, sizeof(s_notch));
    if (f0_hz <= 0.0f || q <= 0.0f || f0_hz >= 0.5f * fs_hz) {
        return;                              /* tắt */
    }
    const float w0    = 2.0f * FC_PI * f0_hz / fs_hz;
    const float cw    = cosf(w0);
    const float alpha = sinf(w0) / (2.0f * q);
    const float a0    = 1.0f + alpha;

    s_notch.b0 = 1.0f / a0;
    s_notch.b1 = -2.0f * cw / a0;
    s_notch.b2 = 1.0f / a0;
    s_notch.a1 = -2.0f * cw / a0;
    s_notch.a2 = (1.0f - alpha) / a0;
    s_notch.on = true;
}

static inline float notch_apply(int axis, float x)
{
    if (!s_notch.on) {
        return x;
    }
    const float y = s_notch.b0 * x + s_notch.z1[axis];
    s_notch.z1[axis] = s_notch.b1 * x - s_notch.a1 * y + s_notch.z2[axis];
    s_notch.z2[axis] = s_notch.b2 * x - s_notch.a2 * y;
    return y;
}
static uint8_t s_gyro_fs_sel;               /* bit dai do da ghi vao chip  */
static uint8_t s_who_am_i;
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
    uint8_t tx[2] = { (uint8_t)(reg | ICM42_SPI_READ_BIT), 0x00u };
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

bool icm42688_init(void)
{
    uint8_t who = 0;

    s_who_am_i = 0;

    s_state = ICM_STATE_UNINIT;
    s_busy  = false;
    memset(s_tx, 0, sizeof(s_tx));
    memset(s_rx, 0, sizeof(s_rx));
    memset(&s_cal, 0, sizeof(s_cal));

    /* Hệ số lọc tính sẵn theo tốc độ lấy mẫu danh định, tránh chia trong ISR. */
    const float dt = 1.0f / (float)IMU_SAMPLE_RATE_HZ;
    s_gyro_alpha  = fc_lpf_alpha(g_params.imu_gyro_lpf_hz,  dt);
    s_accel_alpha = fc_lpf_alpha(g_params.imu_accel_lpf_hz, dt);
    s_gyro_lpf    = (vec3f_t){ 0.0f, 0.0f, 0.0f };
    notch_init(IMU_GYRO_NOTCH_HZ, IMU_GYRO_NOTCH_Q, (float)IMU_SAMPLE_RATE_HZ);

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

    /* Ghi thanh ghi cấu hình ở 1 MHz cho chắc ăn, tuy chip chịu được 24 MHz. */
    if (!spi_set_baud(SPI_BAUDRATEPRESCALER_64)) {
        goto fail;
    }
    HAL_Delay(1);

    /* --- Đặt lại toàn bộ chip --- */
    (void)reg_write(ICM42_REG_BANK_SEL, 0x00u);
    if (!reg_write(ICM42_REG_DEVICE_CONFIG, 0x01u)) {   /* SOFT_RESET_CONFIG */
        goto fail;
    }
    HAL_Delay(2);                                        /* datasheet: >= 1 ms */

    /* --- Xác minh đúng chip --- */
    if (!reg_read(ICM42_REG_WHO_AM_I, &who)) {
        goto fail;
    }
    s_who_am_i = who;
    if (who != ICM42_WHO_AM_I_VALUE) {
        goto fail;
    }

    /*
     * Tắt hẳn giao diện I2C (UI_SIFS_CFG = 11), giữ dữ liệu big-endian như
     * mặc định (bit 5..4). Cùng lý do như ICM-20602: nhiễu trên bus có thể
     * làm chip nhảy sang I2C và ngừng đáp SPI.
     */
    if (!reg_write_verify(ICM42_REG_INTF_CONFIG0, 0x33u)) {
        goto fail;
    }

    /*
     * Tắt AFSR (bit 7..6 = 01). Ở chế độ mặc định chip tự đổi dải đo bên
     * trong, và ngay lúc đổi dữ liệu gyro đứng yên vài mẫu — Betaflight/PX4
     * đều gặp và đều tắt. Các bit khác (nguồn clock) giữ nguyên.
     */
    {
        uint8_t v = 0;
        if (!reg_read(ICM42_REG_INTF_CONFIG1, &v) ||
            !reg_write_verify(ICM42_REG_INTF_CONFIG1, (uint8_t)((v & 0x3Fu) | 0x40u))) {
            goto fail;
        }
    }

    /* --- AAF gyro, bank 1 --- */
    {
        uint8_t v = 0;
        bool ok = reg_write(ICM42_REG_BANK_SEL, 0x01u);
        /* STATIC2: bit1 AAF_DIS = 0 (bật AAF), bit0 NF_DIS = 1 (tắt notch
         * trong chip — tần số notch đó không biết, notch của ta ở phía sau). */
        ok = ok && reg_read(ICM42_REG_GYRO_CONFIG_STATIC2, &v);
        ok = ok && reg_write_verify(ICM42_REG_GYRO_CONFIG_STATIC2,
                                    (uint8_t)((v & ~0x03u) | 0x01u));
        ok = ok && reg_write_verify(ICM42_REG_GYRO_CONFIG_STATIC3, ICM42_AAF_DELT);
        ok = ok && reg_write_verify(ICM42_REG_GYRO_CONFIG_STATIC4,
                                    (uint8_t)(ICM42_AAF_DELTSQR & 0xFFu));
        ok = ok && reg_write_verify(ICM42_REG_GYRO_CONFIG_STATIC5,
                                    (uint8_t)((ICM42_AAF_BITSHIFT << 4) |
                                              (ICM42_AAF_DELTSQR >> 8)));

        /* --- AAF accel, bank 2 --- */
        ok = ok && reg_write(ICM42_REG_BANK_SEL, 0x02u);
        ok = ok && reg_write_verify(ICM42_REG_ACCEL_CONFIG_STATIC2,
                                    (uint8_t)(ICM42_AAF_DELT << 1));   /* bit0 AAF_DIS = 0 */
        ok = ok && reg_write_verify(ICM42_REG_ACCEL_CONFIG_STATIC3,
                                    (uint8_t)(ICM42_AAF_DELTSQR & 0xFFu));
        ok = ok && reg_write_verify(ICM42_REG_ACCEL_CONFIG_STATIC4,
                                    (uint8_t)((ICM42_AAF_BITSHIFT << 4) |
                                              (ICM42_AAF_DELTSQR >> 8)));

        /* Luôn trả về bank 0, kể cả khi hỏng giữa chừng. */
        ok = reg_write(ICM42_REG_BANK_SEL, 0x00u) && ok;
        if (!ok) {
            goto fail;
        }
    }

    /* --- Cấu hình đo --- */
    if (!reg_write_verify(ICM42_REG_GYRO_CONFIG0,
                          (uint8_t)((s_gyro_fs_sel << 5) | ICM42_ODR_8KHZ)) ||
        !reg_write_verify(ICM42_REG_ACCEL_CONFIG0,
                          (uint8_t)((s_accel_fs_sel << 5) | ICM42_ODR_8KHZ)) ||
        !reg_write_verify(ICM42_REG_GYRO_ACCEL_CONFIG0, ICM42_UI_FILT_LOW_LATENCY) ||
        !reg_write_verify(ICM42_REG_FIFO_CONFIG, 0x00u)) {      /* không dùng FIFO */
        goto fail;
    }

    /*
     * INT1: xung (không chốt), đẩy kéo, tích cực mức cao — hợp với EXTI4 bắt
     * sườn lên và điện trở kéo xuống trên PC4, giống ICM-20602.
     *
     * INT_CONFIG1 = 0x60: xung 8 µs + TDEASSERT_DISABLE (bắt buộc ở ODR >= 4 kHz),
     * INT_ASYNC_RESET = 0 (datasheet yêu cầu xoá thì chân INT mới chạy đúng).
     *
     * INT_SOURCE0 = 0x08: chỉ UI_DRDY lên INT1 (mặc định còn RESET_DONE).
     */
    if (!reg_write_verify(ICM42_REG_INT_CONFIG,  0x03u) ||
        !reg_write_verify(ICM42_REG_INT_CONFIG1, 0x60u) ||
        !reg_write_verify(ICM42_REG_INT_SOURCE0, 0x08u)) {
        goto fail;
    }

    /*
     * Bật gyro và accel ở chế độ low-noise. Datasheet: sau lệnh này không ghi
     * thanh ghi nào trong 200 µs, và gyro cần ~30 ms mới ra số đúng.
     */
    if (!reg_write(ICM42_REG_PWR_MGMT0, 0x0Fu)) {
        goto fail;
    }
    HAL_Delay(50);
    {
        uint8_t pwr = 0;
        if (!reg_read(ICM42_REG_PWR_MGMT0, &pwr) || pwr != 0x0Fu) {
            goto fail;
        }
    }

    /* Xong phần cấu hình, chuyển sang 8 MHz cho đường dữ liệu. */
    if (!spi_set_baud(SPI_BAUDRATEPRESCALER_8)) {
        goto fail;
    }

    /* Xoá cờ ngắt còn treo trước khi cho phép DRDY. */
    {
        uint8_t dummy = 0;
        (void)reg_read(ICM42_REG_INT_STATUS, &dummy);
    }

    s_tx[0] = (uint8_t)(ICM42_REG_TEMP_DATA1 | ICM42_SPI_READ_BIT);

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

void icm42688_start(void)
{
    if (s_state == ICM_STATE_IDLE) {
        s_prev_sample_us = micros();
        s_state = ICM_STATE_RUNNING;
    }
}

void icm42688_stop(void)
{
    if (s_state == ICM_STATE_RUNNING || s_state == ICM_STATE_CALIBRATING) {
        s_state = ICM_STATE_IDLE;
    }
}

icm42688_state_t icm42688_get_state(void)
{
    return s_state;
}

uint8_t icm42688_who_am_i(void)
{
    return s_who_am_i;
}

/* ==========================================================================
 * Hiệu chuẩn bias gyro
 * ========================================================================== */

void icm42688_start_gyro_calibration(void)
{
    if (s_state != ICM_STATE_RUNNING && s_state != ICM_STATE_CALIBRATING) {
        return;
    }

    memset(&s_cal, 0, sizeof(s_cal));

    g_fc.imu.calibrated    = false;
    g_fc.imu.gyro_bias_dps = (vec3f_t){ 0.0f, 0.0f, 0.0f };
    s_state = ICM_STATE_CALIBRATING;
}

uint8_t icm42688_calibration_progress(void)
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

/** Đọc số nguyên 16-bit có dấu, big-endian (INTF_CONFIG0 mặc định). */
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
    /*
     * Tách dữ liệu thô (byte 0 là byte rác ứng với lúc gửi địa chỉ).
     * Thứ tự của ICM-42688: NHIỆT ĐỘ trước, rồi accel, rồi gyro.
     */
    const int16_t t_raw  = be16(&s_rx[1]);
    const int16_t ax_raw = be16(&s_rx[3]);
    const int16_t ay_raw = be16(&s_rx[5]);
    const int16_t az_raw = be16(&s_rx[7]);
    const int16_t gx_raw = be16(&s_rx[9]);
    const int16_t gy_raw = be16(&s_rx[11]);
    const int16_t gz_raw = be16(&s_rx[13]);

    /*
     * Toàn bộ bằng 0 thường là MISO treo thấp (mất kết nối). Gyro bằng đúng
     * -32768 cả ba trục là cách ICM-42688 báo "chưa có số hợp lệ" (cảm biến
     * đang khởi động hoặc bị tắt).
     */
    if ((gx_raw | gy_raw | gz_raw | ax_raw | ay_raw | az_raw) == 0 ||
        (gx_raw == INT16_MIN && gy_raw == INT16_MIN && gz_raw == INT16_MIN)) {
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

    /* Luồng rung 8 kHz qua USB, cũng lấy số CHƯA LỌC. Tắt thì chỉ tốn một phép kiểm cờ. */
    vib_stream_push(&gyro, &accel, s_sample_us);

    /* --- Lọc thông thấp --- */
    s_gyro_lpf.x = fc_lpf(s_gyro_lpf.x, gyro.x, s_gyro_alpha);
    s_gyro_lpf.y = fc_lpf(s_gyro_lpf.y, gyro.y, s_gyro_alpha);
    s_gyro_lpf.z = fc_lpf(s_gyro_lpf.z, gyro.z, s_gyro_alpha);

    /* --- Notch rung motor, xem IMU_GYRO_NOTCH_HZ --- */
    const vec3f_t filt = {
        notch_apply(AXIS_ROLL,  s_gyro_lpf.x),
        notch_apply(AXIS_PITCH, s_gyro_lpf.y),
        notch_apply(AXIS_YAW,   s_gyro_lpf.z)
    };

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

void icm42688_drdy_isr(void)
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
    s_tx[0]     = (uint8_t)(ICM42_REG_TEMP_DATA1 | ICM42_SPI_READ_BIT);

    CS_LOW();
    if (HAL_SPI_TransmitReceive_DMA(&hspi1, s_tx, s_rx,
                                    1u + ICM42_BURST_DATA_LEN) != HAL_OK) {
        CS_HIGH();
        s_busy = false;
        g_fc.imu.error_count++;
        fc_state_set_error(FC_ERR_IMU_SPI);
    }
}

void icm42688_spi_complete_isr(void)
{
    CS_HIGH();
    s_busy = false;
    process_sample();
}

void icm42688_spi_error_isr(void)
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

float icm42688_gyro_sigma_dps(void)
{
#if IMU_NOISE_STATS_ENABLE
    return imu_noise_sigma_max(&s_noise);
#else
    return 0.0f;
#endif
}

vec3f_t icm42688_gyro_sigma_axes_dps(void)
{
#if IMU_NOISE_STATS_ENABLE
    return (vec3f_t){ s_noise.sigma[0], s_noise.sigma[1], s_noise.sigma[2] };
#else
    return (vec3f_t){ 0.0f, 0.0f, 0.0f };
#endif
}
