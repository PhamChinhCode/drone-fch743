/**
 * @file    servo.c
 * @brief   Hiện thực hai servo trên TIM4. Xem servo.h về quyền điều khiển.
 *
 * TIM4: 240 MHz / (239+1) = 1 MHz, chu kỳ 20000 tick -> 50 Hz, CCR = µs.
 */

#include "servo.h"
#include "fc_state.h"
#include "param_table.h"
#include "ctrl_offboard.h"
#include "main.h"
#include <stdlib.h>

extern TIM_HandleTypeDef htim4;

#define SERVO_CH_GRIP  TIM_CHANNEL_3   /* PD14 */
#define SERVO_CH_CAM   TIM_CHANNEL_4   /* PD15 */

static uint16_t s_grip_us;
static uint16_t s_cam_us;

static bool     s_pi;            /* Pi có quyền ở nhịp trước              */

/* Sau khi Pi trả quyền: chờ người lái động vào kênh mới bám theo RC. */
static bool     s_grip_wait;
static bool     s_cam_wait;
static uint16_t s_grip_ref;
static uint16_t s_cam_ref;

/**
 * Kẹp xung vào dải [a, b] rồi ghi ra kênh. a > b là hợp lệ (đảo chiều), nên
 * kẹp theo cặp đã sắp.
 */
static uint16_t write_us(uint32_t channel, uint16_t us, int32_t a, int32_t b)
{
    const uint16_t out = (uint16_t)fc_constrain_i32(us, (a < b) ? a : b, (a < b) ? b : a);
    __HAL_TIM_SET_COMPARE(&htim4, channel, out);
    return out;
}

static void write_grip(uint16_t us)
{
    s_grip_us = write_us(SERVO_CH_GRIP, us,
                         g_params.servo_grip_open_us, g_params.servo_grip_close_us);
}

static void write_cam(uint16_t us)
{
    s_cam_us = write_us(SERVO_CH_CAM, us,
                        g_params.servo_cam_min_us, g_params.servo_cam_max_us);
}

/** Đổi giá trị kênh CRSF 172..1811 thành xung trong [a, b], a ứng với 172. */
static uint16_t rc_to_us(uint16_t raw, int32_t a, int32_t b)
{
    const int32_t r = fc_constrain_i32(raw, RC_CRSF_CHANNEL_MIN, RC_CRSF_CHANNEL_MAX);

    return (uint16_t)(a + (r - RC_CRSF_CHANNEL_MIN) * (b - a)
                          / (RC_CRSF_CHANNEL_MAX - RC_CRSF_CHANNEL_MIN));
}

/**
 * Kênh bị tắt (-1), ngoài dải, hoặc tay cầm không gửi kênh đó thì trả false.
 *
 * ELRS ở chế độ ít hơn 16 kênh gửi các kênh thiếu bằng 0 (đo 2026-10-22: GX12
 * CH13 = CH14 = 0). Không chặn thì 0 bị kẹp về 172 và servo bị kéo hết về một
 * đầu. Kênh thật luôn >= 172; dưới một nửa số đó coi như không có kênh.
 */
static bool rc_raw(int8_t ch, uint16_t *raw)
{
    if (ch < 0 || ch >= RC_CHANNEL_COUNT) {
        return false;
    }
    *raw = g_fc.rc.channel_raw[ch];
    return *raw >= RC_CRSF_CHANNEL_MIN / 2;
}

/** Người lái đã động vào kênh kể từ lúc nhận lại quyền chưa. */
static bool picked_up(bool *wait, uint16_t ref, uint16_t raw)
{
    if (*wait && abs((int32_t)raw - (int32_t)ref) < SERVO_RC_PICKUP_DELTA) {
        return false;
    }
    *wait = false;
    return true;
}

void servo_init(void)
{
    s_pi        = false;
    s_grip_wait = false;
    s_cam_wait  = false;

    write_grip(g_params.servo_grip_close_us);
    write_cam((uint16_t)((g_params.servo_cam_min_us + g_params.servo_cam_max_us) / 2));

    (void)HAL_TIM_PWM_Start(&htim4, SERVO_CH_GRIP);
    (void)HAL_TIM_PWM_Start(&htim4, SERVO_CH_CAM);
}

void servo_update(void)
{
    if (ctrl_offboard_pi_has_authority()) {
        /* Servo chỉ đổi khi Pi gửi lệnh — xem servo_link_*(). */
        s_pi = true;
        return;
    }

    /* Mất sóng: giữ vị trí cuối. Hàng đang kẹp không được rơi vì mất sóng. */
    if (!g_fc.rc.healthy || g_fc.rc.failsafe) {
        return;
    }

    uint16_t grip_raw = 0;
    uint16_t cam_raw  = 0;
    const bool grip_on = rc_raw(g_params.servo_grip_channel, &grip_raw);
    const bool cam_on  = rc_raw(g_params.servo_cam_channel,  &cam_raw);

    /*
     * Vừa nhận lại quyền từ Pi: chốt giá trị kênh lúc này làm mốc. Đặt ở đây
     * chứ không phải lúc Pi mất quyền, vì mất quyền do mất sóng thì phải chờ
     * sóng về mới đọc được kênh.
     */
    if (s_pi) {
        s_pi        = false;
        s_grip_wait = true;
        s_cam_wait  = true;
        s_grip_ref  = grip_raw;
        s_cam_ref   = cam_raw;
    }

    /* Núm xoay: đầu 172 = mở / cam_min, đầu 1811 = đóng / cam_max. */
    if (grip_on && picked_up(&s_grip_wait, s_grip_ref, grip_raw)) {
        write_grip(rc_to_us(grip_raw, g_params.servo_grip_open_us,
                            g_params.servo_grip_close_us));
    }

    if (cam_on && picked_up(&s_cam_wait, s_cam_ref, cam_raw)) {
        write_cam(rc_to_us(cam_raw, g_params.servo_cam_min_us,
                           g_params.servo_cam_max_us));
    }
}

bool servo_link_grip(bool close)
{
    if (!ctrl_offboard_pi_has_authority()) {
        return false;
    }
    servo_set_grip(close);
    return true;
}

bool servo_link_grip_us(uint16_t pulse_us)
{
    if (!ctrl_offboard_pi_has_authority()) {
        return false;
    }
    write_grip(pulse_us);
    return true;
}

bool servo_link_cam_us(uint16_t pulse_us)
{
    if (!ctrl_offboard_pi_has_authority()) {
        return false;
    }
    write_cam(pulse_us);
    return true;
}

void servo_set_grip(bool close)
{
    write_grip(close ? g_params.servo_grip_close_us : g_params.servo_grip_open_us);
}

void servo_set_grip_us(uint16_t pulse_us) { write_grip(pulse_us); }
void servo_set_cam_us(uint16_t pulse_us)  { write_cam(pulse_us); }
uint16_t servo_grip_us(void)              { return s_grip_us; }
uint16_t servo_cam_us(void)               { return s_cam_us;  }
