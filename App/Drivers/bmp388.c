/**
 * @file    bmp388.c
 * @brief   Hiện thực driver BMP388 (I2C1, đọc bằng ngắt).
 */

#include "bmp388.h"
#include "fc_state.h"
#include "param_table.h"
#include "fc_time.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

/* HAL nhận địa chỉ 8 bit, tức địa chỉ 7 bit đã dịch trái một nhịp. */
#define BMP_I2C_ADDR        ((uint16_t)(BARO_I2C_ADDR_7BIT << 1))

/* ==========================================================================
 * Chuyển tham số cấu hình thành giá trị thanh ghi
 * ========================================================================== */

/*
 * Sieu lay mau gio la THAM SO RUNTIME nen khong con #error luc bien dich.
 * Thay bang tra bang luc chay: chi nhan dung luy thua cua 2 tu 1 den 32, gia
 * tri khac duoc lam tron XUONG muc hop le gan nhat.
 *
 * osr_index() tra ve chi so trong bang, va CHINH chi so do vua la bit thanh
 * ghi vua la so mu de suy nguoc ra he so that (1 << i). Nho vay he so dung
 * cho phep tinh thoi gian do o duoi luon khop voi thu da ghi xuong chip.
 */
#define BMP_OSR_LEVELS  6u   /* 1, 2, 4, 8, 16, 32 */

static uint8_t osr_index(uint8_t want)
{
    uint8_t idx = 0;

    for (uint8_t i = 0; i < BMP_OSR_LEVELS; i++) {
        if ((1u << i) <= want) {
            idx = i;
        }
    }
    return idx;
}

/* He so IIR hop le: 0, 1, 3, 7, 15, 31, 63, 127 - tuc (1 << i) - 1. */
#define BMP_IIR_LEVELS  8u

static uint8_t iir_index(uint8_t want)
{
    uint8_t idx = 0;

    for (uint8_t i = 0; i < BMP_IIR_LEVELS; i++) {
        if (((1u << i) - 1u) <= want) {
            idx = i;
        }
    }
    return idx;
}

/* ODR của chip = 200 Hz / 2^odr_sel. */
#if   BARO_SAMPLE_RATE_HZ == 200
  #define BMP_ODR_SEL     0u
#elif BARO_SAMPLE_RATE_HZ == 100
  #define BMP_ODR_SEL     1u
#elif BARO_SAMPLE_RATE_HZ == 50
  #define BMP_ODR_SEL     2u
#elif BARO_SAMPLE_RATE_HZ == 25
  #define BMP_ODR_SEL     3u
#elif BARO_SAMPLE_RATE_HZ == 12
  #define BMP_ODR_SEL     4u     /* thực tế 12,5 Hz */
#elif BARO_SAMPLE_RATE_HZ == 6
  #define BMP_ODR_SEL     5u     /* thực tế 6,25 Hz */
#else
  #error "BARO_SAMPLE_RATE_HZ chi nhan 200, 100, 50, 25, 12 hoac 6"
#endif

#define BMP_ODR_PERIOD_US  (1000000u / BARO_SAMPLE_RATE_HZ)

/*
 * Thoi gian do PHAI lot trong chu ky ODR, cong thuc datasheet muc 3.9.2 (us):
 *   t_meas = 234 + (392 + osr_p * 2020) + (163 + osr_t * 2020)
 *
 * Truoc day day la #error luc bien dich. Gio osr la tham so runtime nen phep
 * kiem chuyen sang luc chay - va no VAN PHAI CO: vuot chu ky thi chip bat bit
 * conf_err va lang le bo mau, bieu hien ra ngoai chi la "baro thinh thoang
 * chet" chu khong bao gi.
 *
 * Khong hop le thi HA osr_p cho toi khi vua - ha do phan giai con hon mat han
 * nguon do do cao.
 */
static uint8_t fit_osr_pressure(uint8_t osr_p_idx, uint8_t osr_t_idx)
{
    while (osr_p_idx > 0u) {
        const uint32_t t_meas = 234u
                              + (392u + (1u << osr_p_idx) * 2020u)
                              + (163u + (1u << osr_t_idx) * 2020u);

        if (t_meas < BMP_ODR_PERIOD_US) {
            break;
        }
        osr_p_idx--;
    }
    return osr_p_idx;
}

/* PWR_CTRL: bật đo áp suất + nhiệt độ, chế độ NORMAL (đo liên tục). */
#define BMP_PWR_CTRL_NORMAL   0x33u
#define BMP_PWR_CTRL_SLEEP    0x00u

/* Dung trong bmp388_init(), sau khi da chot chi so osr. */
#define BMP_ODR_VALUE      (uint8_t)(BMP_ODR_SEL)


/*
 * IF_CONF: bật bộ canh giờ (watchdog) của giao diện I2C với ngưỡng 40 ms.
 * Nếu bus kẹt giữa chừng — chuyện thường gặp khi dây dài hoặc nhiễu ESC —
 * chip tự đặt lại phần giao tiếp thay vì treo cứng mãi.
 */
#define BMP_IF_CONF_VALUE  0x06u

/* Chân INT của cảm biến không đấu vào mạch nên tắt hẳn ngắt của chip. */
#define BMP_INT_CTRL_VALUE 0x00u

/* Chu kỳ hỏi vòng và hạn chờ một lượt truyền I2C. */
#define BMP_POLL_PERIOD_US  (1000000u / BARO_POLL_RATE_HZ)
#define BMP_XFER_TIMEOUT_US 20000u

/* Dải đo hợp lệ của BMP388 theo datasheet: 300 hPa .. 1250 hPa. */
#define BMP_PRESSURE_MIN_PA  30000.0f
#define BMP_PRESSURE_MAX_PA  125000.0f

/* ==========================================================================
 * Biến nội bộ
 *
 * s_buf KHÔNG cần macro FC_DMA_BUFFER: I2C1 trong project này chạy ở chế độ
 * ngắt, chính CPU chép từng byte ra khỏi thanh ghi RXDR, không có DMA nào
 * đụng tới vùng nhớ. Để ở DTCMRAM như biến static thường là đúng và nhanh
 * nhất. (Nếu sau này bật DMA cho I2C1 thì BẮT BUỘC phải thêm macro — xem
 * chú thích trong fc_types.h.)
 * ========================================================================== */

static uint8_t s_buf[BMP388_BURST_LEN];

static volatile bmp388_state_t s_state = BMP_STATE_UNINIT;
static volatile bool     s_busy;          /* đang có lượt đọc I2C chạy    */
static volatile bool     s_new_raw;       /* ISR báo có khối byte mới     */
static volatile uint32_t s_rx_us;         /* mốc thời gian lúc nhận xong  */

static uint32_t s_last_poll_us;
static uint32_t s_xfer_start_us;
static uint32_t s_i2c_errors;
static uint32_t s_stale_reads;
static uint8_t  s_chip_id;

static float s_alt_alpha;                 /* hệ số lọc độ cao             */
static float s_alt_rel_m;                 /* độ cao tương đối đã lọc      */
static bool  s_alt_primed;                /* đã nạp mẫu đầu vào bộ lọc    */

/** Bộ tích luỹ khi lấy mốc áp suất mặt đất. */
static struct {
    uint32_t count;
    double   sum_pa;
} s_ground;

/**
 * Hệ số hiệu chuẩn đã quy đổi sang số thực, theo bảng "quantized" của Bosch.
 * Dùng double vì phép bù có số hạng bậc 3 của giá trị thô (cỡ 10^18) nhân
 * với hệ số cỡ 10^-19 — float 32 bit không đủ dải để giữ chính xác.
 */
static struct {
    double t1, t2, t3;
    double p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
    double t_lin;          /**< nhiệt độ tuyến tính, dùng lại khi bù áp    */
} s_cal;

/* ==========================================================================
 * Truy cập thanh ghi ở chế độ hỏi vòng (chỉ dùng lúc init)
 * ========================================================================== */

static bool reg_write(uint8_t reg, uint8_t value)
{
    return (HAL_I2C_Mem_Write(&hi2c1, BMP_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                              &value, 1u, BARO_I2C_TIMEOUT_MS) == HAL_OK);
}

static bool reg_read(uint8_t reg, uint8_t *dst, uint16_t len)
{
    return (HAL_I2C_Mem_Read(&hi2c1, BMP_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             dst, len, BARO_I2C_TIMEOUT_MS) == HAL_OK);
}

/** Ghi rồi đọc lại để chắc chắn thanh ghi đã nhận đúng giá trị. */
static bool reg_write_verify(uint8_t reg, uint8_t value)
{
    uint8_t readback = 0;

    if (!reg_write(reg, value)) {
        return false;
    }
    if (!reg_read(reg, &readback, 1u)) {
        return false;
    }
    return (readback == value);
}

/* ==========================================================================
 * Hệ số hiệu chuẩn
 *
 * 21 byte đọc từ 0x31 xếp theo thứ tự little-endian:
 *   T1(u16) T2(u16) T3(i8) P1(i16) P2(i16) P3(i8) P4(i8) P5(u16)
 *   P6(u16) P7(i8) P8(i8) P9(i16) P10(i8) P11(i8)
 *
 * Mẫu số là các luỹ thừa của 2 lấy từ bảng 3.11.1 của datasheet.
 * ========================================================================== */

static void parse_calibration(const uint8_t *b)
{
    const uint16_t nvm_t1  = (uint16_t)(b[0]  | ((uint16_t)b[1]  << 8));
    const uint16_t nvm_t2  = (uint16_t)(b[2]  | ((uint16_t)b[3]  << 8));
    const int8_t   nvm_t3  = (int8_t)b[4];
    const int16_t  nvm_p1  = (int16_t)(b[5]  | ((uint16_t)b[6]  << 8));
    const int16_t  nvm_p2  = (int16_t)(b[7]  | ((uint16_t)b[8]  << 8));
    const int8_t   nvm_p3  = (int8_t)b[9];
    const int8_t   nvm_p4  = (int8_t)b[10];
    const uint16_t nvm_p5  = (uint16_t)(b[11] | ((uint16_t)b[12] << 8));
    const uint16_t nvm_p6  = (uint16_t)(b[13] | ((uint16_t)b[14] << 8));
    const int8_t   nvm_p7  = (int8_t)b[15];
    const int8_t   nvm_p8  = (int8_t)b[16];
    const int16_t  nvm_p9  = (int16_t)(b[17] | ((uint16_t)b[18] << 8));
    const int8_t   nvm_p10 = (int8_t)b[19];
    const int8_t   nvm_p11 = (int8_t)b[20];

    s_cal.t1  = (double)nvm_t1 * 256.0;                        /* / 2^-8  */
    s_cal.t2  = (double)nvm_t2 / 1073741824.0;                 /* / 2^30  */
    s_cal.t3  = (double)nvm_t3 / 281474976710656.0;            /* / 2^48  */

    s_cal.p1  = ((double)nvm_p1 - 16384.0) / 1048576.0;        /* / 2^20  */
    s_cal.p2  = ((double)nvm_p2 - 16384.0) / 536870912.0;      /* / 2^29  */
    s_cal.p3  = (double)nvm_p3 / 4294967296.0;                 /* / 2^32  */
    s_cal.p4  = (double)nvm_p4 / 137438953472.0;               /* / 2^37  */
    s_cal.p5  = (double)nvm_p5 * 8.0;                          /* / 2^-3  */
    s_cal.p6  = (double)nvm_p6 / 64.0;                         /* / 2^6   */
    s_cal.p7  = (double)nvm_p7 / 256.0;                        /* / 2^8   */
    s_cal.p8  = (double)nvm_p8 / 32768.0;                      /* / 2^15  */
    s_cal.p9  = (double)nvm_p9 / 281474976710656.0;            /* / 2^48  */
    s_cal.p10 = (double)nvm_p10 / 281474976710656.0;           /* / 2^48  */
    s_cal.p11 = (double)nvm_p11 / 36893488147419103232.0;      /* / 2^65  */

    s_cal.t_lin = 0.0;
}

/**
 * Toàn bộ hệ số bằng 0 nghĩa là chưa đọc được NVM (thường do nhầm địa chỉ
 * hoặc thiếu điện trở kéo lên) — phải coi là lỗi, nếu không áp suất tính ra
 * sẽ luôn bằng 0 mà không có dấu hiệu gì.
 */
static bool calibration_looks_valid(void)
{
    return (s_cal.t1 != 0.0) && (s_cal.t2 != 0.0) && (s_cal.p5 != 0.0);
}

/* ==========================================================================
 * Phép bù của Bosch (bản số thực, datasheet mục 9.2 / API bmp3)
 * ========================================================================== */

/** Trả về nhiệt độ °C, đồng thời lưu t_lin cho phép bù áp suất. */
static double compensate_temperature(uint32_t raw_temp)
{
    const double d1 = (double)raw_temp - s_cal.t1;
    const double d2 = d1 * s_cal.t2;

    s_cal.t_lin = d2 + (d1 * d1) * s_cal.t3;
    return s_cal.t_lin;
}

/** Trả về áp suất Pa. Phải gọi compensate_temperature() trước. */
static double compensate_pressure(uint32_t raw_press)
{
    const double t  = s_cal.t_lin;
    const double t2 = t * t;
    const double t3 = t2 * t;

    const double p  = (double)raw_press;
    const double p2 = p * p;
    const double p3 = p2 * p;

    const double out1 = s_cal.p5 + s_cal.p6 * t + s_cal.p7 * t2 + s_cal.p8 * t3;
    const double out2 = p * (s_cal.p1 + s_cal.p2 * t + s_cal.p3 * t2 + s_cal.p4 * t3);
    const double out3 = p2 * (s_cal.p9 + s_cal.p10 * t) + p3 * s_cal.p11;

    return out1 + out2 + out3;
}

/* ==========================================================================
 * Áp suất -> độ cao
 *
 * Công thức khí quyển chuẩn quốc tế cho tầng đối lưu:
 *   h = (T0 / L) * (1 - (p / p_ref)^(R*L / (g*M)))
 * với T0 = 288,15 K, L = 0,0065 K/m  ->  T0/L = 44330,77 m
 * và số mũ R*L/(g*M) = 0,190263.
 *
 * Lấy p_ref là áp suất mặt đất thì kết quả chính là độ cao so với điểm cất
 * cánh — đúng thứ vòng giữ độ cao cần.
 * ========================================================================== */

static float pressure_to_altitude_m(float pressure_pa, float reference_pa)
{
    if (pressure_pa <= 0.0f || reference_pa <= 0.0f) {
        return 0.0f;
    }
    return 44330.77f * (1.0f - powf(pressure_pa / reference_pa, 0.190263f));
}

/* ==========================================================================
 * Ghi nhận lỗi
 * ========================================================================== */

static void record_error(void)
{
    s_i2c_errors++;
    g_fc.baro.error_count++;
    g_fc.baro.healthy = false;
    fc_state_set_error(FC_ERR_BARO_I2C);
}

/* ==========================================================================
 * Khởi tạo
 * ========================================================================== */

bool bmp388_init(void)
{
    uint8_t calib[BMP388_CALIB_LEN];
    uint8_t err_reg = 0;

    s_state        = BMP_STATE_UNINIT;
    s_busy         = false;
    s_new_raw      = false;
    s_rx_us        = 0;
    s_last_poll_us = 0;
    s_i2c_errors   = 0;
    s_stale_reads  = 0;
    s_chip_id      = 0;
    s_alt_rel_m    = 0.0f;
    s_alt_primed   = false;

    memset(s_buf, 0, sizeof(s_buf));
    memset(&s_cal, 0, sizeof(s_cal));
    memset(&s_ground, 0, sizeof(s_ground));

    g_fc.baro.healthy    = false;
    g_fc.baro.calibrated = false;

    /* Hệ số lọc tính sẵn theo ODR danh định, tránh chia trong đường dữ liệu. */
    s_alt_alpha = fc_lpf_alpha(g_params.baro_alt_lpf_hz,
                               1.0f / (float)BARO_SAMPLE_RATE_HZ);

    /* Chip cần tối đa 2 ms kể từ lúc có nguồn mới trả lời được. */
    HAL_Delay(5);

    if (HAL_I2C_IsDeviceReady(&hi2c1, BMP_I2C_ADDR, 3u,
                              BARO_I2C_TIMEOUT_MS) != HAL_OK) {
        goto fail;      /* sai địa chỉ, thiếu điện trở kéo lên, hoặc mất nguồn */
    }

    if (!reg_read(BMP388_REG_CHIP_ID, &s_chip_id, 1u)) {
        goto fail;
    }
    if (s_chip_id != BMP388_CHIP_ID && s_chip_id != BMP390_CHIP_ID) {
        goto fail;
    }

    /* --- Đặt lại toàn bộ chip --- */
    if (!reg_write(BMP388_REG_CMD, BMP388_CMD_SOFTRESET)) {
        goto fail;
    }
    HAL_Delay(10);      /* datasheet yêu cầu 2 ms, lấy dư cho chắc */

    /* Chờ chip báo sẵn sàng nhận lệnh tiếp theo. */
    bool ready = false;
    for (int i = 0; i < 20; i++) {
        uint8_t status = 0;
        if (reg_read(BMP388_REG_STATUS, &status, 1u) &&
            (status & BMP388_STATUS_CMD_RDY) != 0u) {
            ready = true;
            break;
        }
        HAL_Delay(2);
    }
    if (!ready) {
        goto fail;
    }

    /* Lỗi nghiêm trọng ngay sau reset nghĩa là chip hỏng, không dùng được. */
    if (!reg_read(BMP388_REG_ERR, &err_reg, 1u) ||
        (err_reg & BMP388_ERR_FATAL) != 0u) {
        goto fail;
    }

    /* --- Hệ số hiệu chuẩn nằm trong NVM, đọc một lần rồi giữ luôn --- */
    if (!reg_read(BMP388_REG_CALIB_00, calib, BMP388_CALIB_LEN)) {
        goto fail;
    }
    parse_calibration(calib);
    if (!calibration_looks_valid()) {
        goto fail;
    }

    /*
     * --- Cau hinh do ---
     * Ghi het thong so truoc, bat che do NORMAL sau cung. Doi OSR/ODR khi
     * chip dang do se lam chip bo do mau hien tai va bat co conf_err.
     *
     * Ba gia tri duoi day tinh TU g_params luc chay (truoc kia la hang so
     * bien dich). fit_osr_pressure() ha osr_p neu bo tham so nguoi dung chon
     * lam thoi gian do vuot chu ky ODR - ha do phan giai con hon de chip bat
     * conf_err roi lang le bo mau.
     */
    const uint8_t osr_p_want = osr_index(g_params.baro_osr_pressure);
    const uint8_t osr_t_idx  = osr_index(g_params.baro_osr_temperature);
    const uint8_t osr_p_idx  = fit_osr_pressure(osr_p_want, osr_t_idx);

    const uint8_t osr_value    = (uint8_t)((osr_t_idx << 3) | osr_p_idx);
    const uint8_t config_value = (uint8_t)(iir_index(g_params.baro_iir_coef) << 1);

    if (osr_p_idx != osr_p_want) {
        /*
         * Da phai ha xuong. Ghi nguoc vao tham so de `get baro_osr_pressure`
         * in ra con so DANG CHAY chu khong phai con so da bi bo qua - khong
         * co dong nay thi bang tham so noi doi mot cach im lang.
         */
        const uint16_t idx = param_find("baro_osr_pressure");
        if (idx != PARAM_INDEX_NONE) {
            (void)param_set_f32(idx, (float)(1u << osr_p_idx));
        }
    }

    if (!reg_write_verify(BMP388_REG_PWR_CTRL,  BMP_PWR_CTRL_SLEEP)    ||
        !reg_write_verify(BMP388_REG_IF_CONF,   BMP_IF_CONF_VALUE)     ||
        !reg_write_verify(BMP388_REG_INT_CTRL,  BMP_INT_CTRL_VALUE)    ||
        !reg_write_verify(BMP388_REG_OSR,       osr_value)             ||
        !reg_write_verify(BMP388_REG_ODR,       BMP_ODR_VALUE)         ||
        !reg_write_verify(BMP388_REG_CONFIG,    config_value)          ||
        !reg_write_verify(BMP388_REG_PWR_CTRL,  BMP_PWR_CTRL_NORMAL)) {
        goto fail;
    }

    /*
     * Chip tự kiểm tra tính hợp lệ của bộ OSR/ODR vừa ghi. Bit conf_err bật
     * nghĩa là thời gian đo dài hơn chu kỳ ODR — kiểm tra ở tầng biên dịch
     * phía trên đã chặn trường hợp này, nên nếu vẫn bật thì có gì đó sai
     * nghiêm trọng và không nên tin số đo.
     */
    HAL_Delay(5);
    if (!reg_read(BMP388_REG_ERR, &err_reg, 1u) ||
        (err_reg & (BMP388_ERR_FATAL | BMP388_ERR_CONF)) != 0u) {
        goto fail;
    }

    s_state        = BMP_STATE_RUNNING;
    s_last_poll_us = micros();
    return true;

fail:
    s_state = BMP_STATE_ERROR;
    fc_state_set_error(FC_ERR_BARO_I2C);
    return false;
}

/* ==========================================================================
 * Xử lý một mẫu
 * ========================================================================== */

static bool process_sample(void)
{
    const uint8_t status = s_buf[0];

    /*
     * Chưa có mẫu mới: sáu byte vừa đọc chính là mẫu lần trước. Bỏ qua để
     * sample_count không đếm trùng và để dt giữa hai mẫu vẫn đúng.
     */
    if ((status & BMP388_STATUS_DRDY_PRESS) == 0u) {
        s_stale_reads++;
        return false;
    }

    const uint32_t raw_press = (uint32_t)s_buf[1]
                             | ((uint32_t)s_buf[2] << 8)
                             | ((uint32_t)s_buf[3] << 16);
    const uint32_t raw_temp  = (uint32_t)s_buf[4]
                             | ((uint32_t)s_buf[5] << 8)
                             | ((uint32_t)s_buf[6] << 16);

    if (raw_press == 0u || raw_temp == 0u) {
        s_stale_reads++;
        return false;
    }

    const float temp_c   = (float)compensate_temperature(raw_temp);
    const float press_pa = (float)compensate_pressure(raw_press);

    /* Ra ngoài dải đo của chip thì chắc chắn là số rác — không ghi vào state. */
    if (press_pa < BMP_PRESSURE_MIN_PA || press_pa > BMP_PRESSURE_MAX_PA) {
        record_error();
        return false;
    }

    g_fc.baro.temperature_c = temp_c;
    g_fc.baro.pressure_pa   = press_pa;
    g_fc.baro.altitude_m    = pressure_to_altitude_m(press_pa, g_params.baro_sea_level_pa);

    if (s_state == BMP_STATE_CALIBRATING) {
        s_ground.sum_pa += (double)press_pa;
        s_ground.count++;

        /* Chưa có mốc thì độ cao tương đối chưa có nghĩa. */
        g_fc.baro.altitude_rel_m = 0.0f;

        if (s_ground.count >= g_params.baro_calib_sample_count) {
            g_fc.baro.ground_pressure_pa =
                (float)(s_ground.sum_pa / (double)s_ground.count);
            g_fc.baro.calibrated = true;
            s_alt_primed = false;
            s_state      = BMP_STATE_RUNNING;
        }
    } else {
        const float alt_rel =
            pressure_to_altitude_m(press_pa, g_fc.baro.ground_pressure_pa);

        /*
         * Mẫu đầu tiên nạp thẳng vào bộ lọc. Nếu để bộ lọc bò từ 0 lên thì
         * ngay sau khi hiệu chuẩn xong độ cao sẽ trườn dần cả giây — đủ để
         * vòng giữ độ cao hiểu nhầm là máy bay đang rơi.
         */
        if (!s_alt_primed) {
            s_alt_rel_m  = alt_rel;
            s_alt_primed = true;
        } else {
            s_alt_rel_m = fc_lpf(s_alt_rel_m, alt_rel, s_alt_alpha);
        }
        g_fc.baro.altitude_rel_m = s_alt_rel_m;
    }

    g_fc.baro.timestamp_us = s_rx_us;
    g_fc.baro.sample_count++;
    g_fc.baro.healthy = true;
    return true;
}

/** Phát lệnh đọc STATUS + 6 byte dữ liệu, không chặn. */
static void start_read(void)
{
    /*
     * Đặt cờ bận TRƯỚC khi gọi HAL: ngắt I2C1_EV ở mức ưu tiên 5 có thể chen
     * vào ngay giữa chừng, và ISR là nơi hạ cờ này xuống.
     */
    s_xfer_start_us = micros();
    s_busy          = true;

    if (HAL_I2C_Mem_Read_IT(&hi2c1, BMP_I2C_ADDR, BMP388_REG_STATUS,
                            I2C_MEMADD_SIZE_8BIT, s_buf,
                            BMP388_BURST_LEN) != HAL_OK) {
        s_busy = false;
        record_error();
    }
}

/* ==========================================================================
 * API
 * ========================================================================== */

bool bmp388_update(void)
{
    if (s_state == BMP_STATE_UNINIT || s_state == BMP_STATE_ERROR) {
        return false;
    }

    bool got_sample = false;

    /*
     * Xử lý mẫu cũ TRƯỚC khi phát lệnh đọc mới. Nhờ thứ tự này mà s_buf chỉ
     * có đúng một chủ tại mỗi thời điểm: vòng lặp chính khi s_busy == false,
     * ngắt I2C khi s_busy == true. Không cần khoá ngắt ở đâu cả.
     */
    if (s_new_raw) {
        s_new_raw  = false;
        got_sample = process_sample();
    }

    const uint32_t now_us = micros();

    /*
     * Lượt truyền treo — thường do cảm biến bị rút giữa chừng hoặc bus kẹt ở
     * mức thấp. Huỷ để giải phóng ngoại vi, lần hỏi vòng sau sẽ thử lại.
     */
    if (s_busy && fc_elapsed_us(now_us, s_xfer_start_us) > BMP_XFER_TIMEOUT_US) {
        (void)HAL_I2C_Master_Abort_IT(&hi2c1, BMP_I2C_ADDR);
        s_busy = false;
        record_error();
    }

    /*
     * Điều kiện !s_new_raw chặn một trường hợp hiếm nhưng có thật: ngắt I2C
     * kết thúc ngay giữa hàm này, sau khi cờ s_new_raw đã được đọc là false.
     * Không có nó, lệnh đọc mới sẽ ghi đè s_buf khi mẫu vừa nhận chưa kịp xử
     * lý và mẫu đó mất trắng.
     */
    if (!s_busy && !s_new_raw &&
        fc_elapsed_us(now_us, s_last_poll_us) >= BMP_POLL_PERIOD_US) {
        s_last_poll_us = now_us;
        start_read();
    }

    /* Quá lâu không có mẫu hợp lệ thì hạ cờ khoẻ để tầng ước lượng bỏ qua. */
    if (fc_elapsed_us(now_us, g_fc.baro.timestamp_us)
            > (BARO_TIMEOUT_MS * 1000u)) {
        g_fc.baro.healthy = false;
    }

    return got_sample;
}

void bmp388_start_ground_calibration(void)
{
    if (s_state == BMP_STATE_UNINIT || s_state == BMP_STATE_ERROR) {
        return;
    }

    s_ground.count  = 0;
    s_ground.sum_pa = 0.0;
    s_alt_primed    = false;

    g_fc.baro.calibrated     = false;
    g_fc.baro.altitude_rel_m = 0.0f;

    s_state = BMP_STATE_CALIBRATING;
}

void bmp388_reset_ground_level(void)
{
    if (!g_fc.baro.healthy) {
        return;
    }

    g_fc.baro.ground_pressure_pa = g_fc.baro.pressure_pa;
    g_fc.baro.altitude_rel_m     = 0.0f;
    g_fc.baro.calibrated         = true;

    s_alt_rel_m  = 0.0f;
    s_alt_primed = true;
}

bmp388_state_t bmp388_get_state(void)
{
    return s_state;
}

uint8_t bmp388_calibration_progress(void)
{
    if (s_state != BMP_STATE_CALIBRATING) {
        return g_fc.baro.calibrated ? 100u : 0u;
    }
    return (uint8_t)((s_ground.count * 100u) / g_params.baro_calib_sample_count);
}

uint8_t  bmp388_chip_id(void)     { return s_chip_id; }
uint32_t bmp388_i2c_errors(void)  { return s_i2c_errors; }
uint32_t bmp388_stale_reads(void) { return s_stale_reads; }

/* ==========================================================================
 * Ngắt
 *
 * Hai hàm dưới đây chạy trong I2C1_EV_IRQHandler / I2C1_ER_IRQHandler ở mức
 * ưu tiên 5 — thấp hơn gyro (0) và DShot (1) nên không đụng đường điều khiển.
 * Cố tình giữ thật ngắn: mọi phép tính số thực để cho bmp388_update() làm.
 * ========================================================================== */

void bmp388_i2c_complete_isr(void)
{
    s_rx_us   = micros();
    s_busy    = false;
    s_new_raw = true;
}

void bmp388_i2c_error_isr(void)
{
    s_busy = false;
    record_error();
}
