/**
 * @file    fc_state.c
 * @brief   Hiện thực cho khối trạng thái trung tâm.
 */

#include "fc_state.h"
#include "param_table.h"
#include "usb_msc.h"
#include "stm32h7xx.h"   /* chỉ cần cho __get_PRIMASK / __set_PRIMASK */

/* ==========================================================================
 * Thực thể toàn cục.
 *
 * Không gán giá trị khởi tạo -> nằm trong .bss -> theo STM32H743XX_FLASH.ld
 * thì .bss thuộc vùng RAM tại 0x24000000 (AXI SRAM). DMA1/DMA2 truy cập được
 * vùng này. Tuyệt đối KHÔNG đặt struct dùng cho DMA lên stack, vì stack nằm
 * trong DTCMRAM (0x20000000) mà DMA1/DMA2 không với tới.
 * ========================================================================== */
fc_t g_fc;

/* Ngưỡng coi là mất tín hiệu (mili giây). */
#define HEALTH_TIMEOUT_IMU_MS     10U
#define HEALTH_TIMEOUT_BARO_MS    200U
#define HEALTH_TIMEOUT_FLOW_MS    200U
#define HEALTH_TIMEOUT_POWER_MS   500U

/* ==========================================================================
 * Vùng loại trừ ngắt
 * ========================================================================== */

static inline uint32_t critical_enter(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void critical_exit(uint32_t primask)
{
    __set_PRIMASK(primask);
}

/* ==========================================================================
 * Khởi tạo
 * ========================================================================== */

void fc_state_init(void)
{
    memset(&g_fc, 0, sizeof(g_fc));

    g_fc.mode       = FC_MODE_INIT;
    g_fc.ctrl.mode  = FLIGHT_MODE_ANGLE;   /* mặc định an toàn nhất */

    /* Quaternion đơn vị: chưa nghiêng. */
    g_fc.est.attitude_q = (quatf_t){ 1.0f, 0.0f, 0.0f, 0.0f };

    /* Mốc áp suất mặc định để altitude không ra giá trị vô lý trước hiệu chuẩn. */
    g_fc.baro.ground_pressure_pa = g_params.baro_sea_level_pa;

    /* Kênh RC về giữa, ga về thấp nhất. */
    for (int i = 0; i < RC_CHANNEL_COUNT; i++) {
        g_fc.rc.channel_raw[i] = RC_CRSF_CHANNEL_MID;
    }
    g_fc.rc.channel_raw[2] = RC_CRSF_CHANNEL_MIN;  /* kênh ga */
    g_fc.rc.failsafe       = true;

    /* Chưa có RC hợp lệ -> chặn arm ngay từ đầu. */
    g_fc.sys.arm_block_flags = ARM_BLOCK_NO_RC | ARM_BLOCK_GYRO_CALIB;
}

/* ==========================================================================
 * Chụp trạng thái
 * ========================================================================== */

void fc_state_snapshot(fc_t *dst)
{
    if (dst == NULL) {
        return;
    }

    const uint32_t primask = critical_enter();
    memcpy(dst, &g_fc, sizeof(fc_t));
    critical_exit(primask);
}

/* ==========================================================================
 * Theo dõi sức khoẻ cảm biến
 * ========================================================================== */

/** Trả về true nếu mẫu còn mới và module tự báo là khoẻ. */
static bool sensor_alive(uint32_t now_us, uint32_t stamp_us,
                         uint32_t timeout_ms, bool healthy_flag)
{
    if (!healthy_flag) {
        return false;
    }
    return fc_elapsed_us(now_us, stamp_us) < (timeout_ms * 1000U);
}

void fc_state_update_health(uint32_t now_us)
{
    uint32_t mask = 0;

    if (sensor_alive(now_us, g_fc.imu.timestamp_us,
                     HEALTH_TIMEOUT_IMU_MS, g_fc.imu.healthy)) {
        mask |= SENSOR_GYRO | SENSOR_ACCEL;
    } else {
        fc_state_set_error(FC_ERR_IMU_TIMEOUT);
    }

    /*
     * IMU phu LSM6DSV. Mat no KHONG dat co loi va KHONG chan arm - may
     * bay van bay binh thuong bang ICM20602. Chi bao trong sensor_health.
     */
    if (sensor_alive(now_us, g_fc.imu2.timestamp_us,
                     HEALTH_TIMEOUT_IMU_MS, g_fc.imu2.healthy)) {
        mask |= SENSOR_IMU2;
    }

    /* Tu ke. Mat no cung KHONG dat co loi - chua co gi phu thuoc vao no. */
    if (sensor_alive(now_us, g_fc.mag.timestamp_us,
                     MAG_TIMEOUT_MS, g_fc.mag.healthy)) {
        mask |= SENSOR_MAG;
    }

    if (sensor_alive(now_us, g_fc.baro.timestamp_us,
                     HEALTH_TIMEOUT_BARO_MS, g_fc.baro.healthy)) {
        mask |= SENSOR_BARO;
    }

    if (sensor_alive(now_us, g_fc.flow.timestamp_us,
                     HEALTH_TIMEOUT_FLOW_MS, g_fc.flow.healthy)) {
        mask |= SENSOR_FLOW;
        if (g_fc.flow.range_valid) {
            mask |= SENSOR_RANGE;
        }
    }

    if (sensor_alive(now_us, g_fc.rc.last_frame_us,
                     RC_FAILSAFE_TIMEOUT_MS, g_fc.rc.healthy)) {
        mask |= SENSOR_RC;
    } else {
        g_fc.rc.failsafe = true;
        fc_state_set_error(FC_ERR_RC_TIMEOUT);
    }

    if (sensor_alive(now_us, g_fc.power.timestamp_us,
                     HEALTH_TIMEOUT_POWER_MS, g_fc.power.healthy)) {
        mask |= SENSOR_POWER;
    }

    if (g_fc.sys.sdcard_mounted) {
        mask |= SENSOR_SDCARD;
    }

    g_fc.sys.sensor_health = mask;
}

/* ==========================================================================
 * Điều kiện arm
 * ========================================================================== */

bool fc_state_check_arm(void)
{
    uint32_t block = ARM_BLOCK_NONE;

    /* Che do doc the qua USB thi khong bay, xet truoc moi thu khac. */
    if (usb_msc_active()) {
        block |= ARM_BLOCK_USB_MSC;
    }

    /* Phải có tín hiệu RC hợp lệ và không đang failsafe. */
    if ((g_fc.sys.sensor_health & SENSOR_RC) == 0) {
        block |= ARM_BLOCK_NO_RC;
    }
    if (g_fc.rc.failsafe) {
        block |= ARM_BLOCK_FAILSAFE;
    }

    /* Cần gạt ga xuống thấp trước khi arm. */
    if (g_fc.rc.throttle > g_params.arm_throttle_max_norm) {
        block |= ARM_BLOCK_THROTTLE_HIGH;
    }

    /* Gyro phải hiệu chuẩn xong. */
    if (!g_fc.imu.calibrated) {
        block |= ARM_BLOCK_GYRO_CALIB;
    }

    /* Con quay và gia tốc kế bắt buộc phải khoẻ. */
    if ((g_fc.sys.sensor_health & (SENSOR_GYRO | SENSOR_ACCEL))
            != (SENSOR_GYRO | SENSOR_ACCEL)) {
        block |= ARM_BLOCK_SENSOR_FAIL;
    }

    /* Máy bay phải đang nằm tương đối phẳng. */
    const float tilt_limit_rad = g_params.arm_max_tilt_deg * FC_DEG_TO_RAD;
    if (fabsf(g_fc.est.attitude_rad.roll)  > tilt_limit_rad ||
        fabsf(g_fc.est.attitude_rad.pitch) > tilt_limit_rad) {
        block |= ARM_BLOCK_NOT_LEVEL;
    }

    /* Pin không được ở mức nguy hiểm. */
    if (g_fc.power.critical) {
        block |= ARM_BLOCK_LOW_BATTERY;
    }

    g_fc.sys.arm_block_flags = block;
    return (block == ARM_BLOCK_NONE);
}

/* ==========================================================================
 * Cờ lỗi
 * ========================================================================== */

void fc_state_set_error(fc_error_flag_t flag)
{
    const uint32_t primask = critical_enter();
    g_fc.sys.error_flags |= (uint32_t)flag;
    critical_exit(primask);
}

void fc_state_clear_error(fc_error_flag_t flag)
{
    const uint32_t primask = critical_enter();
    g_fc.sys.error_flags &= ~(uint32_t)flag;
    critical_exit(primask);
}

/* ==========================================================================
 * Máy trạng thái
 * ========================================================================== */

/** Bảng chuyển trạng thái hợp lệ: transition_ok[từ][sang]. */
static bool transition_allowed(fc_mode_t from, fc_mode_t to)
{
    if (to == FC_MODE_FAULT) {
        return true;                    /* luôn được phép báo lỗi nặng */
    }

    switch (from) {
    case FC_MODE_INIT:
        return (to == FC_MODE_CALIBRATING);

    case FC_MODE_CALIBRATING:
        return (to == FC_MODE_DISARMED);

    case FC_MODE_DISARMED:
        return (to == FC_MODE_ARMED) || (to == FC_MODE_CALIBRATING);

    case FC_MODE_ARMED:
        return (to == FC_MODE_DISARMED) || (to == FC_MODE_FAILSAFE);

    case FC_MODE_FAILSAFE:
        return (to == FC_MODE_DISARMED) || (to == FC_MODE_ARMED);

    case FC_MODE_FAULT:
        return false;                   /* trạng thái cuối, chỉ reset mới thoát */

    default:
        return false;
    }
}

bool fc_state_set_mode(fc_mode_t mode)
{
    if (mode >= FC_MODE_COUNT) {
        return false;
    }
    if (mode == g_fc.mode) {
        return true;
    }
    if (!transition_allowed(g_fc.mode, mode)) {
        return false;
    }

    /* Chỉ cho arm khi mọi điều kiện an toàn đã thoả. */
    if (mode == FC_MODE_ARMED && !fc_state_check_arm()) {
        return false;
    }

    const uint32_t primask = critical_enter();

    g_fc.mode        = mode;
    g_fc.motor.armed = (mode == FC_MODE_ARMED);

    if (mode != FC_MODE_ARMED) {
        /* Rời trạng thái bay: cắt toàn bộ đầu ra và xả tích phân PID. */
        for (int i = 0; i < FC_MOTOR_COUNT; i++) {
            g_fc.motor.output_norm[i] = 0.0f;
            g_fc.motor.throttle[i]    = 0;
        }
        for (int i = 0; i < AXIS_COUNT; i++) {
            g_fc.ctrl.rate_pid[i].integral  = 0.0f;
            g_fc.ctrl.angle_pid[i].integral = 0.0f;
        }
        g_fc.ctrl.altitude_pid.integral = 0.0f;
        g_fc.ctrl.throttle_cmd          = 0.0f;
    }

    critical_exit(primask);
    return true;
}

/* ==========================================================================
 * Chuỗi mô tả
 * ========================================================================== */

const char *fc_mode_name(fc_mode_t mode)
{
    static const char *const names[FC_MODE_COUNT] = {
        [FC_MODE_INIT]        = "INIT",
        [FC_MODE_CALIBRATING] = "CALIB",
        [FC_MODE_DISARMED]    = "DISARMED",
        [FC_MODE_ARMED]       = "ARMED",
        [FC_MODE_FAILSAFE]    = "FAILSAFE",
        [FC_MODE_FAULT]       = "FAULT",
    };
    return (mode < FC_MODE_COUNT) ? names[mode] : "?";
}

const char *fc_flight_mode_name(flight_mode_t mode)
{
    static const char *const names[FLIGHT_MODE_COUNT] = {
        [FLIGHT_MODE_ACRO]    = "ACRO",
        [FLIGHT_MODE_ANGLE]   = "ANGLE",
        [FLIGHT_MODE_ALTHOLD] = "ALTHOLD",
        [FLIGHT_MODE_POSHOLD] = "POSHOLD",
    };
    return (mode < FLIGHT_MODE_COUNT) ? names[mode] : "?";
}
