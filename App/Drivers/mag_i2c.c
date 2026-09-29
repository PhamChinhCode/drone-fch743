/**
 * @file    mag_i2c.c
 * @brief   Hiện thực driver từ kế rời trên I2C1 (dò HMC5883L/QMC5883L, đọc
 *          bằng ngắt). Xem mag_i2c.h để biết kiến trúc chung.
 */

#include "mag_i2c.h"
#include "hmc5883.h"
#include "qmc5883.h"
#include "qmc5883p.h"
#include "ist8310.h"
#include "fc_state.h"
#include "param_table.h"
#include "fc_time.h"
#include "dbg_console.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c2;

/* HAL nhận địa chỉ 8 bit, tức địa chỉ 7 bit đã dịch trái một nhịp. */
#define MAG_I2C_HAL_ADDR(addr7)  ((uint16_t)((addr7) << 1))

#define MAG_I2C_POLL_PERIOD_US   (1000000u / MAG_I2C_UPDATE_RATE_HZ)
#define MAG_I2C_XFER_TIMEOUT_US  20000u

/* ==========================================================================
 * Biến nội bộ — cùng khuôn với bmp388.c (đọc ngắt, không DMA).
 * ========================================================================== */

/* Du cho burst dai nhat trong ba chip (QMC5883P doc 9 byte: 0x01..0x09). */
static uint8_t s_buf[QMC5883P_BURST_LEN];

static volatile mag_i2c_state_t s_state = MAG_I2C_STATE_UNINIT;
static volatile bool     s_busy;
static volatile bool     s_new_raw;
static volatile uint32_t s_rx_us;

static uint32_t s_last_poll_us;
static uint32_t s_xfer_start_us;
static uint32_t s_i2c_errors;
static uint32_t s_stale_reads;
static uint32_t s_bus_lost;    /* so lan GAP bus ban (gom ca thu lai)   */

static mag_i2c_variant_t s_variant = MAG_I2C_VARIANT_NONE;
static uint16_t s_hal_addr;        /* dia chi 8 bit da dich, dung cho HAL */
static uint8_t  s_data_base_reg;   /* thanh ghi dau khoi 6 byte du lieu   */
static uint8_t  s_burst_len;       /* so byte doc moi luot                */
static uint8_t  s_status_idx;      /* vi tri byte STATUS trong s_buf      */
static float    s_lsb_per_gauss;

/*
 * Rieng IST8310: chip chi do DON, nen sau moi luot doc phai ghi lai lenh do.
 * Lenh ghi [CNTL1, 0x01] chay bang ngat nhu luot doc, nen s_cmd phai song toi
 * khi ngat xong.
 */
static volatile bool s_need_trigger;
static uint8_t       s_ist_stale_run;  /* so luot doc lien tiep chua co mau moi */
static uint8_t       s_cmd[2] = { IST8310_REG_CNTL1, IST8310_CNTL1_SINGLE };

/* ==========================================================================
 * Truy cập thanh ghi ở chế độ hỏi vòng (chỉ dùng lúc init)
 * ========================================================================== */

static bool reg_write(uint8_t reg, uint8_t value)
{
    return (HAL_I2C_Mem_Write(&hi2c2, s_hal_addr, reg, I2C_MEMADD_SIZE_8BIT,
                              &value, 1u, MAG_I2C_TIMEOUT_MS) == HAL_OK);
}

static bool reg_read(uint8_t reg, uint8_t *dst, uint16_t len)
{
    return (HAL_I2C_Mem_Read(&hi2c2, s_hal_addr, reg, I2C_MEMADD_SIZE_8BIT,
                             dst, len, MAG_I2C_TIMEOUT_MS) == HAL_OK);
}

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
 * Ghi nhận lỗi — theo đúng quy ước hiện có của g_fc.mag: mất từ kế KHÔNG
 * đặt cờ lỗi toàn cục, vì chưa có gì phụ thuộc vào nó (xem fc_state.c).
 * ========================================================================== */

static void record_error(void)
{
    s_i2c_errors++;
    g_fc.mag.error_count++;
    g_fc.mag.healthy = false;
}

/* ==========================================================================
 * Chọn dải đo theo tham số dùng chung mag_range_g — cùng lối "làm tròn
 * xuống mức hợp lệ gần nhất" mà mag_pick_range() trong lsm6dsv.c đang dùng
 * cho QMC6309, để một tham số áp được cho cả ba chip.
 * ========================================================================== */

static uint8_t hmc_pick_gain(uint8_t want_g)
{
    if (want_g >= 8u) {
        s_lsb_per_gauss = HMC_LSB_PER_GAUSS_8_1GA;
        return HMC_GN_8_1GA;
    }
    s_lsb_per_gauss = HMC_LSB_PER_GAUSS_1_3GA;   /* mac dinh nha may */
    return HMC_GN_1_3GA;
}

static uint8_t qmc5883_pick_range(uint8_t want_g)
{
    if (want_g >= 8u) {
        s_lsb_per_gauss = QMC5883_LSB_PER_GAUSS_8G;
        return QMC5883_RNG_8G;
    }
    s_lsb_per_gauss = QMC5883_LSB_PER_GAUSS_2G;
    return QMC5883_RNG_2G;
}

/* ==========================================================================
 * Khởi tạo
 * ========================================================================== */

static bool init_hmc5883(void)
{
    uint8_t id[3];

    if (!reg_read(HMC_REG_ID_A, id, 3u)) {
        return false;
    }
    if (id[0] != HMC_ID_A_VALUE || id[1] != HMC_ID_B_VALUE ||
        id[2] != HMC_ID_C_VALUE) {
        return false;
    }

    const uint8_t gain = hmc_pick_gain(g_params.mag_range_g);
    const uint8_t config_a = (uint8_t)((HMC_MA_8 << HMC_CONFIG_A_MA_SHIFT) |
                                        (HMC_DO_75HZ << HMC_CONFIG_A_DO_SHIFT) |
                                        HMC_MS_NORMAL);
    const uint8_t config_b = (uint8_t)(gain << HMC_CONFIG_B_GN_SHIFT);

    if (!reg_write_verify(HMC_REG_CONFIG_A, config_a) ||
        !reg_write_verify(HMC_REG_CONFIG_B, config_b) ||
        !reg_write_verify(HMC_REG_MODE, HMC_MODE_CONTINUOUS)) {
        return false;
    }

    s_variant       = MAG_I2C_VARIANT_HMC5883L;
    s_data_base_reg = HMC_REG_DATA_X_MSB;
    s_burst_len     = HMC_BURST_LEN;
    s_status_idx    = HMC_BURST_STATUS_IDX;
    return true;
}

static bool init_qmc5883(void)
{
    uint8_t id = 0;

    if (!reg_read(QMC5883_REG_CHIP_ID, &id, 1u) || id != QMC5883_CHIP_ID_VALUE) {
        return false;
    }

    const uint8_t rng = qmc5883_pick_range(g_params.mag_range_g);
    const uint8_t ctrl1 = (uint8_t)((QMC5883_OSR_512 << QMC5883_CTRL1_OSR_SHIFT) |
                                     (rng << QMC5883_CTRL1_RNG_SHIFT) |
                                     (QMC5883_ODR_100HZ << QMC5883_CTRL1_ODR_SHIFT) |
                                     QMC5883_MODE_CONTINUOUS);

    /* Chan INT khong dau vao mach nen tat han ngat cua chip. */
    if (!reg_write_verify(QMC5883_REG_SET_RESET, QMC5883_SET_RESET_VALUE) ||
        !reg_write_verify(QMC5883_REG_CTRL2, QMC5883_CTRL2_INT_ENB)       ||
        !reg_write_verify(QMC5883_REG_CTRL1, ctrl1)) {
        return false;
    }

    s_variant       = MAG_I2C_VARIANT_QMC5883L;
    s_data_base_reg = QMC5883_REG_DATA_X_LSB;
    s_burst_len     = QMC5883_BURST_LEN;
    s_status_idx    = QMC5883_BURST_STATUS_IDX;
    return true;
}

/*
 * QMC5883P chi dung dai 8 G: day la dai duy nhat co he so doi thang da
 * kiem chung (3000 LSB/Gauss). Xem ghi chu dai o cuoi qmc5883p.h — CO Y
 * bo qua g_params.mag_range_g cho rieng chip nay thay vi suy doan.
 */
static bool init_qmc5883p(void)
{
    uint8_t id = 0;

    if (!reg_read(QMC5883P_REG_CHIP_ID, &id, 1u) || id != QMC5883P_CHIP_ID_VALUE) {
        return false;
    }

    const uint8_t ctrl1 = (uint8_t)((QMC5883P_OSR1_8 << QMC5883P_CTRL1_OSR1_SHIFT) |
                                     (QMC5883P_ODR_100HZ << QMC5883P_CTRL1_ODR_SHIFT) |
                                     QMC5883P_MODE_CONTINUOUS);
    const uint8_t ctrl2 = (uint8_t)(QMC5883P_RNG_8G << QMC5883P_CTRL2_RNG_SHIFT);

    /*
     * 0x29 la GIA TRI, 0x06 la THANH GHI - khong phai nguoc lai. Phai ghi
     * truoc khi bat che do lien tuc, neu khong dau ba truc khong xac dinh.
     *
     * KHONG dung reg_write_verify o day: thanh ghi 0x06 von la byte cao
     * truc Z, doc lai se ra du lieu tu truong chu khong ra 0x29.
     */
    if (!reg_write(QMC5883P_REG_DATA_Z_MSB, QMC5883P_SET_XYZ_SIGN)) {
        return false;
    }
    if (!reg_write_verify(QMC5883P_REG_CTRL2, ctrl2) ||
        !reg_write_verify(QMC5883P_REG_CTRL1, ctrl1)) {
        return false;
    }

    s_lsb_per_gauss = QMC5883P_LSB_PER_GAUSS_8G;
    s_variant       = MAG_I2C_VARIANT_QMC5883P;
    s_data_base_reg = QMC5883P_REG_DATA_X_LSB;
    s_burst_len     = QMC5883P_BURST_LEN;
    s_status_idx    = QMC5883P_BURST_STATUS_IDX;
    return true;
}

#if MAG_I2C_USE_GPS_MAG
/*
 * IST8310 tren module GPS. Thu tu theo driver PX4: dat lai mem, xac nhan
 * WAI, trung binh 16 lan, do rong xung chuan, roi ra lenh do dau tien.
 * Chip co dinh mot dai do nen bo qua g_params.mag_range_g.
 */
static bool init_ist8310(void)
{
    uint8_t id = 0;

    if (!reg_read(IST8310_REG_WAI, &id, 1u) || id != IST8310_WAI_VALUE) {
        return false;
    }
    if (!reg_write(IST8310_REG_CNTL2, IST8310_CNTL2_SRST)) {
        return false;
    }
    HAL_Delay(10);

    if (!reg_read(IST8310_REG_WAI, &id, 1u) || id != IST8310_WAI_VALUE ||
        !reg_write_verify(IST8310_REG_AVGCNTL, IST8310_AVGCNTL_16X) ||
        !reg_write_verify(IST8310_REG_PDCNTL, IST8310_PDCNTL_NORMAL) ||
        !reg_write(IST8310_REG_CNTL1, IST8310_CNTL1_SINGLE)) {
        return false;
    }

    s_lsb_per_gauss = IST8310_LSB_PER_GAUSS;
    s_variant       = MAG_I2C_VARIANT_IST8310;
    s_data_base_reg = IST8310_REG_STAT1;
    s_burst_len     = IST8310_BURST_LEN;
    s_status_idx    = IST8310_BURST_STATUS_IDX;
    return true;
}

/** Thu dung mot dia chi: co tra loi thi khoi tao, xong thi chay. */
static bool try_addr(uint8_t addr7, bool (*init_fn)(void), uint8_t chip_id)
{
    if (HAL_I2C_IsDeviceReady(&hi2c2, MAG_I2C_HAL_ADDR(addr7), 3u,
                              MAG_I2C_TIMEOUT_MS) != HAL_OK) {
        return false;
    }
    s_hal_addr = MAG_I2C_HAL_ADDR(addr7);
    if (!init_fn()) {
        return false;
    }
    g_fc.mag.chip_id = chip_id;
    s_state = MAG_I2C_STATE_RUNNING;
    s_last_poll_us = micros();
    return true;
}
#endif /* MAG_I2C_USE_GPS_MAG */

bool mag_i2c_init(void)
{
    s_state         = MAG_I2C_STATE_UNINIT;
    s_busy          = false;
    s_new_raw       = false;
    s_rx_us         = 0;
    s_last_poll_us  = 0;
    s_i2c_errors    = 0;
    s_stale_reads   = 0;
    s_bus_lost      = 0;
    s_variant       = MAG_I2C_VARIANT_NONE;
    s_lsb_per_gauss = 1.0f;
    s_burst_len     = 0;
    s_status_idx    = 0;
    s_need_trigger  = false;
    s_ist_stale_run = 0;

    memset(s_buf, 0, sizeof(s_buf));

    g_fc.mag.healthy = false;
    g_fc.mag.chip_id = 0;

    /*
     * "Da hieu chuan" = bo tham so KHONG con la ma tran don vi.
     *
     * Truoc day co nay khong bao gio duoc dat true (ca driver nay lan nhanh
     * SHUB), nen console bao "CHUA HIEU CHUAN" vinh vien ke ca sau khi da do
     * xong - mot cai nhan sai lam nguoi doc ket luan sai.
     *
     * Hieu chuan nam trong g_params chu khong nam trong driver, nen day la
     * dinh nghia trung thuc duy nhat co the kiem tra duoc tu day.
     */
    g_fc.mag.calibrated =
        (fabsf(g_params.mag_offset_x_g) > 1.0e-6f) ||
        (fabsf(g_params.mag_offset_y_g) > 1.0e-6f) ||
        (fabsf(g_params.mag_offset_z_g) > 1.0e-6f) ||
        (fabsf(g_params.mag_scale_x - 1.0f) > 1.0e-6f) ||
        (fabsf(g_params.mag_scale_y - 1.0f) > 1.0e-6f) ||
        (fabsf(g_params.mag_scale_z - 1.0f) > 1.0e-6f);

    /* Chip can toi da vai ms ke tu luc co nguon moi tra loi duoc. */
    HAL_Delay(5);

#if MAG_I2C_USE_GPS_MAG
    /*
     * CHI do IST8310, KHONG lui ve chip tren bo khi khong thay. Bo tham so
     * mag_offset_* / mag_scale_* / mag_axis_* thuoc ve DUNG MOT chip; lui
     * ve chip khac luc day GPS long ra se chay voi hieu chuan cua chip kia
     * va keo yaw sai mot cach tu tin — te hon la khong co tu ke.
     */
    if (try_addr(IST8310_I2C_ADDR_7BIT, init_ist8310, IST8310_WAI_VALUE) ||
        try_addr(IST8310_I2C_ADDR_ALT_7BIT, init_ist8310, IST8310_WAI_VALUE)) {
        return true;
    }
    goto fail;
#endif

    if (HAL_I2C_IsDeviceReady(&hi2c2, MAG_I2C_HAL_ADDR(HMC_I2C_ADDR_7BIT), 3u,
                              MAG_I2C_TIMEOUT_MS) == HAL_OK) {
        s_hal_addr = MAG_I2C_HAL_ADDR(HMC_I2C_ADDR_7BIT);
        if (init_hmc5883()) {
            g_fc.mag.chip_id = HMC_ID_A_VALUE;
            s_state = MAG_I2C_STATE_RUNNING;
            s_last_poll_us = micros();
            return true;
        }
        goto fail;
    }

    if (HAL_I2C_IsDeviceReady(&hi2c2, MAG_I2C_HAL_ADDR(QMC5883_I2C_ADDR_7BIT), 3u,
                              MAG_I2C_TIMEOUT_MS) == HAL_OK) {
        s_hal_addr = MAG_I2C_HAL_ADDR(QMC5883_I2C_ADDR_7BIT);
        if (init_qmc5883()) {
            g_fc.mag.chip_id = QMC5883_CHIP_ID_VALUE;
            s_state = MAG_I2C_STATE_RUNNING;
            s_last_poll_us = micros();
            return true;
        }
        goto fail;
    }

    if (HAL_I2C_IsDeviceReady(&hi2c2, MAG_I2C_HAL_ADDR(QMC5883P_I2C_ADDR_7BIT), 3u,
                              MAG_I2C_TIMEOUT_MS) == HAL_OK) {
        s_hal_addr = MAG_I2C_HAL_ADDR(QMC5883P_I2C_ADDR_7BIT);
        if (init_qmc5883p()) {
            g_fc.mag.chip_id = QMC5883P_CHIP_ID_VALUE;
            s_state = MAG_I2C_STATE_RUNNING;
            s_last_poll_us = micros();
            return true;
        }
        goto fail;
    }

fail:
    s_state    = MAG_I2C_STATE_ERROR;
    s_hal_addr = 0;
    return false;
}

/* ==========================================================================
 * Xử lý một mẫu
 * ========================================================================== */

static bool process_sample(void)
{
    const uint8_t status = s_buf[s_status_idx];

    /* Ca hai chip deu dat bit0 khi du lieu moi da san sang. */
    if ((status & 0x01u) == 0u) {
        s_stale_reads++;
        return false;
    }

    int16_t rx, ry, rz;
    bool overflow;

    if (s_variant == MAG_I2C_VARIANT_IST8310) {
        /*
         * STAT1 o byte 0, du lieu LSB truoc tu byte 1. Dao Z de he truc tay
         * TRAI cua chip (X toi, Y phai, Z len) thanh tay phai — xem ist8310.h.
         * -32768 khong dao duoc trong int16 nen chan ve 32767.
         */
        rx = (int16_t)(((uint16_t)s_buf[2] << 8) | s_buf[1]);
        ry = (int16_t)(((uint16_t)s_buf[4] << 8) | s_buf[3]);
        const int16_t z_chip = (int16_t)(((uint16_t)s_buf[6] << 8) | s_buf[5]);
        rz = (z_chip == INT16_MIN) ? INT16_MAX : (int16_t)(-z_chip);
        overflow = false;
    } else if (s_variant == MAG_I2C_VARIANT_HMC5883L) {
        /* MSB truoc, thu tu THANH GHI la X, Z, Y (khong phai X,Y,Z). */
        rx = (int16_t)(((uint16_t)s_buf[0] << 8) | s_buf[1]);
        rz = (int16_t)(((uint16_t)s_buf[2] << 8) | s_buf[3]);
        ry = (int16_t)(((uint16_t)s_buf[4] << 8) | s_buf[5]);
        overflow = (rx == HMC_OVERFLOW_CODE || ry == HMC_OVERFLOW_CODE ||
                    rz == HMC_OVERFLOW_CODE);
    } else {
        /*
         * QMC5883L va QMC5883P dung CHUNG dinh dang: LSB truoc, thu tu
         * X, Y, Z, va bit OVL o cung vi tri 0x02 trong STATUS. Chi khac
         * dia chi, thanh ghi cau hinh va do dai burst.
         */
        rx = (int16_t)(((uint16_t)s_buf[1] << 8) | s_buf[0]);
        ry = (int16_t)(((uint16_t)s_buf[3] << 8) | s_buf[2]);
        rz = (int16_t)(((uint16_t)s_buf[5] << 8) | s_buf[4]);
        overflow = (status & QMC5883_STATUS_OVL) != 0u;
    }

    if (rx == 0 && ry == 0 && rz == 0) {
        s_stale_reads++;
        return false;
    }

    /* --- Doi thang, van o he CAM BIEN --- */
    const vec3f_t sensor_g = {
        (float)rx / s_lsb_per_gauss,
        (float)ry / s_lsb_per_gauss,
        (float)rz / s_lsb_per_gauss
    };

    /*
     * --- Hieu chuan sat cung roi sat mem, TRONG HE CAM BIEN ---
     * Dung chung cong thuc va tham so voi nguon SHUB (lsm6dsv.c) de doi
     * nguon khong lam hong duong xu ly phia sau. Bo so nay phai do lai cho
     * dung module vua lap - xem GD4 trong App/Docs/KE_HOACH_LA_BAN_I2C.md.
     */
    /*
     * cal = S (raw - offset), S doi xung: duong cheo mag_scale_*, ngoai duong
     * cheo mag_soft_* (2026-09-22). mag_soft_* = 0 thi dung nhu ban thang truc.
     */
    const float dx = sensor_g.x - g_params.mag_offset_x_g;
    const float dy = sensor_g.y - g_params.mag_offset_y_g;
    const float dz = sensor_g.z - g_params.mag_offset_z_g;
    const float cal[3] = {
        g_params.mag_scale_x * dx + g_params.mag_soft_xy * dy + g_params.mag_soft_xz * dz,
        g_params.mag_soft_xy * dx + g_params.mag_scale_y * dy + g_params.mag_soft_yz * dz,
        g_params.mag_soft_xz * dx + g_params.mag_soft_yz * dy + g_params.mag_scale_z * dz
    };

    /* --- Roi moi xoay sang he than --- */
    const vec3f_t field = {
        (float)g_params.mag_axis_sign_x * cal[g_params.mag_axis_map_x],
        (float)g_params.mag_axis_sign_y * cal[g_params.mag_axis_map_y],
        (float)g_params.mag_axis_sign_z * cal[g_params.mag_axis_map_z]
    };

    g_fc.mag.raw             = (vec3i16_t){ rx, ry, rz };
    g_fc.mag.raw_gauss       = sensor_g;
    g_fc.mag.field_gauss     = field;
    g_fc.mag.magnitude_gauss = vec3f_norm(field);
    g_fc.mag.overflow        = overflow;
    g_fc.mag.timestamp_us    = s_rx_us;
    g_fc.mag.sample_count++;
    g_fc.mag.healthy         = true;
    return true;
}

/**
 * Phát lệnh đọc STATUS + 6 byte dữ liệu, không chặn.
 *
 * HAL_I2C_Mem_Read_IT() trả về khác HAL_OK trong đúng hai trường hợp ở đây:
 * BMP388 đang giữ bus (hi2c->State != READY), hoặc cờ BUSY phần cứng còn bật
 * dù State đã READY (bus vừa nhả, chưa kịp ổn định). CẢ HAI đều là tranh
 * chấp bình thường trên bus dùng chung, không phải lỗi thật — không thử phân
 * biệt bằng State sau khi gọi (State giữ nguyên READY ở nhánh cờ BUSY, dễ
 * suy diễn nhầm thành "lỗi thật"). Cứ bỏ lượt và thử lại ở vòng hỏi kế tiếp;
 * nếu bus kẹt thật sự thì MAG_TIMEOUT_MS trong mag_i2c_update() sẽ hạ cờ
 * healthy đúng lúc, không cần đếm lỗi ở đây.
 */
static bool start_read(void)
{
    s_xfer_start_us = micros();
    s_busy          = true;

    if (HAL_I2C_Mem_Read_IT(&hi2c2, s_hal_addr, s_data_base_reg,
                            I2C_MEMADD_SIZE_8BIT, s_buf,
                            (uint16_t)s_burst_len) != HAL_OK) {
        s_busy = false;
        s_bus_lost++;
        return false;
    }
    return true;
}

/* ==========================================================================
 * API
 * ========================================================================== */

bool mag_i2c_update(uint32_t now_us)
{
    if (s_state != MAG_I2C_STATE_RUNNING) {
        return false;
    }

    bool got_sample = false;

    if (s_new_raw) {
        s_new_raw  = false;
        got_sample = process_sample();
        /*
         * IST8310: ra lenh do khi vua lay duoc mau moi. Mau cu nghia la chip
         * con dang do — ra lenh luc do chi bat no do lai tu dau.
         *
         * Nhung neu lenh truoc bi mat (thua bus, loi I2C) thi chip dung im
         * mai, nen cu 5 luot doc (100 ms) lien tiep khong co mau moi thi ra
         * lenh lai.
         */
        if (s_variant == MAG_I2C_VARIANT_IST8310) {
            if (got_sample) {
                s_ist_stale_run = 0;
                s_need_trigger  = true;
            } else if (++s_ist_stale_run >= 5u) {
                s_ist_stale_run = 0;
                s_need_trigger  = true;
            }
        }
    }

    /*
     * Lenh do cua IST8310. Thua bus (BMP388 dang truyen) thi giu co va thu
     * lai o vong lap ke tiep, giong luot doc. Chua gui duoc lenh thi khong
     * doc — doc luc nay chi ra mau cu.
     */
    if (s_need_trigger && !s_busy) {
        /*
         * Moc la now_us, KHONG phai micros(): phep kiem timeout ngay ben duoi
         * tinh now_us - s_xfer_start_us trong CUNG lan goi nay. Lay micros()
         * (muon hon now_us) thi hieu so am, thanh so khong dau khong lo, va
         * luot ghi bi huy ngay sau khi phat START — da gap that 2026-09-21:
         * 4/5 lenh do bi huy, mau tu ke dung im.
         */
        s_xfer_start_us = now_us;
        s_busy          = true;
        if (HAL_I2C_Master_Transmit_IT(&hi2c2, s_hal_addr, s_cmd, 2u) == HAL_OK) {
            s_need_trigger = false;
        } else {
            s_busy = false;
            s_bus_lost++;
        }
    }

    /* Luot truyen treo - huy de giai phong ngoai vi, thu lai lan sau. */
    if (s_busy && fc_elapsed_us(now_us, s_xfer_start_us) > MAG_I2C_XFER_TIMEOUT_US) {
        (void)HAL_I2C_Master_Abort_IT(&hi2c2, s_hal_addr);
        s_busy = false;
        record_error();
    }

    /*
     * Chi doi dau moc thoi gian khi luot doc THAT SU khoi phat duoc.
     *
     * Neu dong dau trong ca truong hop thua bus thi moi lan nhuong BMP388
     * mat tron mot chu ky 20 ms, va te hon: hai bo hoi vong deu chay tu
     * cung vong lap chinh voi chu ky co dinh (baro 10 ms, tu ke 20 ms) nen
     * chung KHOA PHA duoc - tu ke cu roi dung vao luc baro dang truyen va
     * thua lien tuc. DA QUAN SAT: ba luot lien tiep cung mot mau, dt_ms leo
     * 32 -> 52 -> 72 ms.
     *
     * De nguyen moc cu thi lan thu lai roi vao vong lap chinh ke tiep
     * (duoi 1 ms sau), luc do bus gan nhu chac chan da ranh - vua pha vo
     * khoa pha vua bien mot khoang trong 20 ms thanh khong dang ke.
     */
    if (!s_busy && !s_new_raw && !s_need_trigger &&
        fc_elapsed_us(now_us, s_last_poll_us) >= MAG_I2C_POLL_PERIOD_US) {
        if (start_read()) {
            s_last_poll_us = now_us;
        }
    }

    if (fc_elapsed_us(now_us, g_fc.mag.timestamp_us) > (MAG_TIMEOUT_MS * 1000u)) {
        g_fc.mag.healthy = false;
    }

    return got_sample;
}

mag_i2c_state_t mag_i2c_get_state(void) { return s_state; }
mag_i2c_variant_t mag_i2c_variant(void) { return s_variant; }

const char *mag_i2c_variant_name(void)
{
    switch (s_variant) {
    case MAG_I2C_VARIANT_HMC5883L: return "HMC5883L @ 0x1E";
    case MAG_I2C_VARIANT_QMC5883L: return "QMC5883L @ 0x0D";
    case MAG_I2C_VARIANT_QMC5883P: return "QMC5883P @ 0x2C";
    case MAG_I2C_VARIANT_IST8310:
        return (s_hal_addr == MAG_I2C_HAL_ADDR(IST8310_I2C_ADDR_7BIT))
               ? "IST8310 @ 0x0E (la ban tren GPS)" : "IST8310 @ 0x0C (la ban tren GPS)";
#if MAG_I2C_USE_GPS_MAG
    default:                       return "khong tim thay IST8310 (0x0E, 0x0C deu khong tra loi)";
#else
    default:                       return "khong tim thay (0x1E, 0x0D, 0x2C deu khong tra loi)";
#endif
    }
}

uint32_t mag_i2c_errors(void)      { return s_i2c_errors; }
uint32_t mag_i2c_stale_reads(void) { return s_stale_reads; }
uint32_t mag_i2c_bus_lost(void)    { return s_bus_lost; }

/* ==========================================================================
 * Ngắt
 *
 * Chạy trong I2C1_EV_IRQHandler / I2C1_ER_IRQHandler, đã được
 * drv_hal_callbacks.c lọc theo hi2c->Devaddress nên chỉ vào đây khi đúng là
 * lượt đọc của từ kế. Cố tình giữ thật ngắn như bmp388_i2c_complete_isr().
 * ========================================================================== */

void mag_i2c_complete_isr(void)
{
    s_rx_us   = micros();
    s_busy    = false;
    s_new_raw = true;
}

void mag_i2c_write_complete_isr(void)
{
    s_busy = false;
}

void mag_i2c_error_isr(void)
{
    s_busy = false;
    record_error();
}

/* ==========================================================================
 * Chẩn đoán: quét bus
 * ========================================================================== */

void mag_i2c_scan_dump(void)
{
    uint8_t found = 0;

    dbg_println("  Quet bus I2C2 (0x08..0x77):");

    for (uint8_t addr = 0x08u; addr <= 0x77u; addr++) {
        if (HAL_I2C_IsDeviceReady(&hi2c2, MAG_I2C_HAL_ADDR(addr), 1u, 5u) != HAL_OK) {
            continue;
        }
        found++;

        /*
         * Goi ten nhung dia chi da biet de khong phai tra bang. Neu module
         * la mot chip khac hoan toan thi it nhat ta biet NO CO O DAY, va
         * dia chi nao - du hon nhieu so "khong thay gi".
         */
        const char *who = "?";
        switch (addr) {
        case 0x0Cu: who = "IST8310 (dia chi phu) hoac AK8975";              break;
        case 0x0Du: who = "QMC5883L";                                        break;
        case 0x0Eu: who = "IST8310 - la ban tren GPS MG-F10-A";              break;
        case 0x1Cu: who = "LIS3MDL (chua co driver)";                        break;
        case 0x1Eu: who = "HMC5883L hoac LIS3MDL (dia chi phu)";             break;
        case 0x2Cu: who = "QMC5883P - la ban tren bo";                       break;
        case 0x30u: who = "MMC5883MA (chua co driver)";                      break;
        case 0x76u: who = "BMP388 (SDO noi GND)";                            break;
        case 0x77u: who = "BMP388 - DOI CHUNG, bus va dien tro keo len OK";   break;
        default:                                                             break;
        }

        dbg_print_hex("    tra loi @ 0x", addr, 2);
        dbg_println(who);
    }

    if (found == 0u) {
        dbg_println("    KHONG CO GI TRA LOI - ke ca BMP388 @ 0x77.");
        dbg_println("    => Loi o BUS, khong phai o module tu ke:");
        dbg_println("       kiem tra dien tro keo len PB7/PB8, hoac bus dang bi keo thap.");
    } else {
        dbg_print_int("    tong so thiet bi tra loi", (int32_t)found);
    }
}
