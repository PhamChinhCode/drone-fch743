/**
 * @file    ctrl_althold.c
 * @brief   Hiện thực vòng giữ độ cao. Xem ctrl_althold.h.
 */
#include "ctrl_althold.h"
#include "mixer.h"
#include "fc_state.h"
#include "param_table.h"
#include "ctrl_offboard.h"

/*
 * Cần ga quy về khoảng [-1, +1] quanh điểm giữa.
 *
 * Điểm giữa lấy từ tham số chứ không cứng bằng 0,5: nhiều tay điều khiển có
 * cần ga không lò xo, và người lái quen đặt điểm "giữ" ở chỗ khác.
 *
 * Tách ra hàm riêng để ctrl_althold_stick_centred() dùng đúng phép tính này —
 * hai chỗ tính lệch nhau một chút là đủ để OFFBOARD thoát trong khi vòng giữ
 * độ cao vẫn tưởng cần đang ở giữa.
 */
static float stick_dev(void)
{
    const float centre = g_params.althold_stick_centre;
    const float span   = (centre > 0.5f) ? centre : (1.0f - centre);

    const float dev = (g_fc.rc.throttle - centre) / ((span > 0.05f) ? span : 0.05f);
    return fc_constrainf(dev, -1.0f, 1.0f);
}

#include <math.h>

static float s_target_m;        /* mốc độ cao đang giữ            */
static float s_climb_target;    /* tốc độ lên mong muốn, m/s      */
static float s_integral;        /* phần tích phân của ga          */
static float s_prev_climb;      /* cho khâu vi phân               */
static float s_dterm;           /* đạo hàm đã lọc                 */
static bool  s_primed;          /* đã có mẫu trước để lấy đạo hàm */
static float s_reset_seen;      /* altitude_reset_sum_m đã áp vào mốc */

/**
 * Phần lực đẩy theo phương THẲNG ĐỨNG trên mỗi đơn vị ga: cos(roll)·cos(pitch),
 * kẹp dưới ở cos(ALTHOLD_TILT_COMP_MAX_DEG). Xem chú thích ở fc_config.h.
 */
static float tilt_cos(void)
{
    const float c = cosf(g_fc.est.attitude_rad.roll) *
                    cosf(g_fc.est.attitude_rad.pitch);
    const float c_min = cosf(ALTHOLD_TILT_COMP_MAX_DEG * FC_DEG_TO_RAD);

    return (c > c_min) ? c : c_min;
}

void ctrl_althold_reset(void)
{
    s_reset_seen   = g_fc.est.altitude_reset_sum_m;
    s_target_m     = 0.0f;
    s_climb_target = 0.0f;
    s_integral     = 0.0f;
    s_prev_climb   = 0.0f;
    s_dterm        = 0.0f;
    s_primed       = false;
}

void ctrl_althold_init(void)
{
    ctrl_althold_reset();
}

void ctrl_althold_enter(float throttle_now)
{
    ctrl_althold_reset();

    s_target_m = g_fc.est.altitude_m;

    /*
     * Nạp tích phân sao cho đầu ra ban đầu bằng ĐÚNG mức ga đang giữ.
     *
     * Lúc vừa vào chế độ, sai số độ cao bằng 0 (mốc vừa đặt bằng độ cao hiện
     * tại) nên P và D đều bằng 0. Đầu ra khi đó là hover + I, vậy muốn nó
     * bằng throttle_now thì I phải bằng phần chênh.
     *
     * Kẹp trong giới hạn tích phân: nếu người lái đang giữ ga sát trần thì
     * phần chênh có thể vượt, và để nó vượt sẽ làm khâu chống bão hoà mất tác
     * dụng ngay từ nhịp đầu.
     */
    const float lim = g_params.althold_i_limit;

    /*
     * Đầu ra có bù nghiêng = (hover + I) / cos, nên để nó bằng throttle_now
     * thì hover + I phải bằng throttle_now · cos. Vào chế độ lúc đang nghiêng
     * mà quên nhân cos thì ga nhảy bậc đúng một lần 1/cos.
     */
    s_integral = fc_constrainf(throttle_now * tilt_cos() - g_params.althold_hover_thr,
                               -lim, lim);
}

bool ctrl_althold_update(float dt, float *throttle_out)
{
    /*
     * Không tin được độ cao thì KHÔNG giữ. Bên gọi trả ga về cần.
     *
     * dt bất thường cũng bỏ qua: chạy khâu vi phân với dt sai còn tệ hơn bỏ
     * hẳn một nhịp — cùng lý lẽ với ctrl_rate.c.
     */
    if (!g_fc.est.altitude_valid || dt <= 0.0f || dt > 0.1f) {
        return false;
    }

    /*
     * Bộ ước lượng vừa neo lại độ cao theo laser: đó là đổi GỐC đo, không phải
     * máy bay lên/xuống. Dời mốc đúng lượng đó để sai số độ cao không nhảy bậc
     * — thiếu bước này thì cú dời 0,84 m ngày 09-19 thành vọt ga rồi cắt ga.
     */
    {
        const float rs = g_fc.est.altitude_reset_sum_m;
        s_target_m  += rs - s_reset_seen;
        s_reset_seen = rs;
    }

    const float alt   = g_fc.est.altitude_m;
    const float climb = g_fc.est.climb_rate_mps;

    /* ================= Vòng ngoài: cần ga -> tốc độ lên mong muốn ========= */

    /* Cần ga quy về [-1, +1] quanh điểm giữa — xem stick_dev(). */
    const float dev = stick_dev();
    const float db  = g_params.althold_stick_deadband;

    if (ctrl_offboard_is_active() && fabsf(dev) <= db) {
        /*
         * OFFBOARD và người lái chưa chạm cần ga: tốc độ lên do máy tính
         * nhúng đặt. Mốc độ cao bám theo chỗ hiện tại, nên lúc rời OFFBOARD
         * là chốt ngay tại đó chứ không bò về độ cao cũ.
         */
        s_climb_target = ctrl_offboard_climb_mps();
        s_target_m     = alt;
    } else if (fabsf(dev) <= db) {
        /*
         * Cần ở giữa: GIỮ. Sai số độ cao đổi ra tốc độ lên mong muốn bằng
         * khâu P, rồi kẹp lại — không có kẹp thì lệch 10 m sẽ đòi một tốc độ
         * lên mà máy bay không thể đạt, và tích phân dồn trong lúc đó.
         */
        s_climb_target = fc_constrainf(
            g_params.althold_alt_kp * (s_target_m - alt),
            -g_params.althold_max_climb_mps,
             g_params.althold_max_climb_mps);
    } else {
        /*
         * Cần ga ra khỏi vùng chết. Nếu đang OFFBOARD thì đây là người lái
         * giành lại quyền trên trục đứng — thoát hẳn OFFBOARD, đừng chia đôi
         * thẩm quyền giữa người và máy. Hàm này vô hại khi không ở OFFBOARD.
         *
         * Cần ga xử lý ở đây chứ không ở stick_override() của ctrl_offboard:
         * nó không về giữa theo lò xo nên chỉ có vùng chết của chính vòng này
         * mới biết thế nào là "đang chạm".
         */
        ctrl_offboard_request_exit(OFFBOARD_EXIT_STICK);

        /*
         * Cần ra khỏi vùng chết: người lái ra lệnh tốc độ lên/xuống.
         *
         * Trừ đi vùng chết rồi chuẩn hoá lại, để ngay khi vừa ra khỏi vùng
         * chết thì lệnh bắt đầu từ 0 chứ không nhảy bậc.
         */
        const float sign = (dev > 0.0f) ? 1.0f : -1.0f;
        const float mag  = (fabsf(dev) - db) / ((1.0f - db) > 0.01f
                                                ? (1.0f - db) : 0.01f);

        s_climb_target = sign * mag * g_params.althold_max_climb_mps;

        /*
         * Mốc BÁM THEO độ cao hiện tại trong lúc đang đẩy cần. Thả cần ra là
         * chốt ngay tại chỗ vừa tới — xem ghi chú ở header.
         */
        s_target_m = alt;
    }

    /* ================= Vòng trong: PID trên tốc độ lên ==================== */

    const float error = s_climb_target - climb;

    const float p_term = g_params.althold_climb_kp * error;

    /*
     * Tích phân đóng băng khi khâu trộn đã bão hoà — cùng cơ chế với
     * ctrl_rate.c. Khâu trộn hy sinh ga để giữ quyền điều khiển tư thế, nên
     * lúc đó ga yêu cầu không tới được motor và dồn tích phân là vô nghĩa.
     */
    if (!mixer_saturated()) {
        s_integral += g_params.althold_climb_ki * error * dt;
        s_integral  = fc_constrainf(s_integral,
                                    -g_params.althold_i_limit,
                                     g_params.althold_i_limit);
    }

    /*
     * D lấy trên SỐ ĐO chứ không trên sai số, và đảo dấu — gạt cần ga không
     * sinh xung vi phân. Bỏ qua nhịp đầu vì chưa có mẫu trước để so.
     */
    float raw_d = 0.0f;
    if (s_primed) {
        raw_d = -(climb - s_prev_climb) / dt;
    }
    s_prev_climb = climb;
    s_primed     = true;

    const float alpha = fc_lpf_alpha(g_params.althold_dterm_lpf_hz, dt);
    s_dterm = fc_lpf(s_dterm, raw_d, alpha);

    const float d_term = g_params.althold_climb_kd * s_dterm;

    /*
     * Ga = ga treo + PID.
     *
     * Ga treo là số hạng NUÔI TIẾN (feedforward) và nó gánh phần lớn công
     * việc. Không có nó thì tích phân phải tự dựng lên toàn bộ lực nâng, mất
     * vài giây — trong đó máy bay rơi.
     */
    float thr = g_params.althold_hover_thr + p_term + s_integral + d_term;

    /*
     * Bù nghiêng: PID ở trên tính lực đẩy THẲNG ĐỨNG cần có; chia cos để ra
     * lực đẩy dọc trục thân. Chia TRƯỚC khi kẹp trần/sàn ga.
     */
    thr /= tilt_cos();

    *throttle_out = fc_constrainf(thr, g_params.althold_thr_min,
                                       g_params.althold_thr_max);
    return true;
}

bool ctrl_althold_stick_centred(void)
{
    return fabsf(stick_dev()) <= g_params.althold_stick_deadband;
}

float ctrl_althold_target_m(void)     { return s_target_m; }
float ctrl_althold_climb_target(void) { return s_climb_target; }
float ctrl_althold_integral(void)     { return s_integral; }
