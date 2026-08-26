/**
 * @file    ctrl_angle.c
 * @brief   Hiện thực vòng ngoài và bộ chọn chế độ bay.
 */

#include "ctrl_angle.h"
#include "ctrl_poshold.h"
#include "fc_state.h"
#include "fc_time.h"

#define ANGLE_PERIOD_US (1000000UL / FC_ATTITUDE_RATE_HZ)

static uint32_t      s_last_us;
static bool          s_started;
static flight_mode_t s_mode = FLIGHT_MODE_ANGLE;
static bool          s_fallback;

void ctrl_angle_init(void)
{
    s_last_us  = micros();
    s_started  = false;
    s_mode     = FLIGHT_MODE_ANGLE;
    s_fallback = false;
    ctrl_poshold_init();

    g_fc.ctrl.mode               = FLIGHT_MODE_ANGLE;
    g_fc.ctrl.setpoint_rate_dps  = (vec3f_t){ 0.0f, 0.0f, 0.0f };
    g_fc.ctrl.setpoint_angle_rad = (euler_t){ 0.0f, 0.0f, 0.0f };
    g_fc.ctrl.throttle_cmd       = 0.0f;
}

/**
 * Đọc công tắc chọn chế độ.
 *
 * Dưới ngưỡng là ANGLE, trên ngưỡng là ACRO — chiều này có chủ ý: kênh chưa
 * gán hoặc mất tín hiệu đều cho giá trị thấp, tức rơi về chế độ an toàn hơn.
 */
static flight_mode_t read_mode_switch(void)
{
#if RC_MODE_CHANNEL < 0
    return FLIGHT_MODE_ANGLE;
#else
    const uint16_t raw = g_fc.rc.channel_raw[RC_MODE_CHANNEL];

    /*
     * Công tắc ba nấc. Thứ tự nấc theo mức độ AN TOÀN giảm dần, để kênh chưa
     * gán hoặc mất tín hiệu (giá trị thấp) rơi vào chế độ an toàn nhất.
     */
    if (raw >= RC_MODE_ACRO_THRESHOLD)    { return FLIGHT_MODE_ACRO;    }
    if (raw >= RC_MODE_POSHOLD_THRESHOLD) { return FLIGHT_MODE_POSHOLD; }
    return FLIGHT_MODE_ANGLE;
#endif
}

bool ctrl_angle_update(uint32_t now_us)
{
    if (fc_elapsed_us(now_us, s_last_us) < ANGLE_PERIOD_US) {
        return false;
    }
    s_last_us = now_us;

    if (!s_started) {
        s_started = true;
        return false;
    }

    /* --- Chọn chế độ --- */
    flight_mode_t want = read_mode_switch();

    /*
     * Đòi ANGLE nhưng bộ ước lượng chưa có góc tin cậy: lùi về ACRO. Bám theo
     * một góc chưa hội tụ còn tệ hơn không bám gì cả — nó sẽ lái máy bay về
     * một tư thế sai với toàn bộ thẩm quyền của vòng ngoài.
     */
    s_fallback = false;
    if ((want == FLIGHT_MODE_ANGLE || want == FLIGHT_MODE_POSHOLD) &&
        !g_fc.est.attitude_valid) {
        want       = FLIGHT_MODE_ACRO;
        s_fallback = true;
    }

    /* Rời POSHOLD thì xoá tích phân, nếu không lần vào lại nó bung ra ngay. */
    if (s_mode == FLIGHT_MODE_POSHOLD && want != FLIGHT_MODE_POSHOLD) {
        ctrl_poshold_reset();
    }

    s_mode        = want;
    g_fc.ctrl.mode = want;

    /* --- Ga: ở cả hai chế độ đều đi thẳng từ cần xuống khâu trộn --- */
    g_fc.ctrl.throttle_cmd = g_fc.rc.throttle;

    /*
     * --- Trục YAW: luôn điều khiển theo tốc độ ---
     * Không có la bàn thì yaw ước lượng trôi dần, giữ hướng theo nó là tự làm
     * máy bay quay đi. Xem chú thích về yaw trong ekf_attitude.h.
     */
    const float yaw_rate = g_fc.rc.yaw * RATE_MAX_YAW_DPS;

    if (s_mode == FLIGHT_MODE_ACRO) {
        /* Cần ra thẳng tốc độ quay. */
        g_fc.ctrl.setpoint_rate_dps = (vec3f_t){
            g_fc.rc.roll  * RATE_MAX_ROLL_DPS,
            g_fc.rc.pitch * RATE_MAX_PITCH_DPS,
            yaw_rate
        };
        g_fc.ctrl.setpoint_angle_rad = (euler_t){ 0.0f, 0.0f, 0.0f };
        return true;
    }

    /* --- ANGLE / POSHOLD: cần góc mục tiêu, rồi vòng P ra tốc độ --- */

    float target_roll  = g_fc.rc.roll  * ANGLE_MAX_LEAN_DEG * FC_DEG_TO_RAD;
    float target_pitch = g_fc.rc.pitch * ANGLE_MAX_LEAN_DEG * FC_DEG_TO_RAD;

    if (s_mode == FLIGHT_MODE_POSHOLD) {
        /*
         * Góc mục tiêu do vòng giữ vận tốc quyết định thay vì cần điều khiển.
         * Mất flow thì nó trả false — lùi về ANGLE ngay tại đây, giữ nguyên
         * góc mục tiêu tính từ cần ở trên.
         */
        const float dt = (float)ANGLE_PERIOD_US * 1.0e-6f;
        if (ctrl_poshold_update(dt, &target_roll, &target_pitch)) {
            /* dùng góc từ vòng giữ vận tốc */
        } else {
            s_mode         = FLIGHT_MODE_ANGLE;
            g_fc.ctrl.mode = FLIGHT_MODE_ANGLE;
            s_fallback     = true;
        }
    }

    g_fc.ctrl.setpoint_angle_rad.roll  = target_roll;
    g_fc.ctrl.setpoint_angle_rad.pitch = target_pitch;
    g_fc.ctrl.setpoint_angle_rad.yaw   = 0.0f;

    /* Sai lệch góc, đổi sang độ vì ANGLE_PID_KP tính theo độ. */
    const float err_roll_deg  =
        (target_roll  - g_fc.est.attitude_rad.roll)  * FC_RAD_TO_DEG;
    const float err_pitch_deg =
        (target_pitch - g_fc.est.attitude_rad.pitch) * FC_RAD_TO_DEG;

    /*
     * Chỉ khâu P, và chặn trần tốc độ yêu cầu. Không chặn thì bật ANGLE lúc
     * đang nghiêng 60° sẽ đòi 300 °/s và máy bay giật nảy rất mạnh.
     */
    const float rate_roll = fc_constrainf(ANGLE_PID_KP * err_roll_deg,
                                          -ANGLE_MAX_RATE_DPS,
                                          ANGLE_MAX_RATE_DPS);
    const float rate_pitch = fc_constrainf(ANGLE_PID_KP * err_pitch_deg,
                                           -ANGLE_MAX_RATE_DPS,
                                           ANGLE_MAX_RATE_DPS);

    g_fc.ctrl.setpoint_rate_dps = (vec3f_t){ rate_roll, rate_pitch, yaw_rate };
    return true;
}

flight_mode_t ctrl_angle_active_mode(void) { return s_mode; }
bool          ctrl_angle_fallback(void)    { return s_fallback; }
