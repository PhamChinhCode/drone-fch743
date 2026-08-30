/**
 * @file    ctrl_angle.c
 * @brief   Hiện thực vòng ngoài và bộ chọn chế độ bay.
 */

#include "ctrl_angle.h"
#include "ctrl_poshold.h"
#include "ctrl_althold.h"
#include "fc_state.h"
#include "param_table.h"
#include "fc_time.h"

#define ANGLE_PERIOD_US (1000000UL / FC_ATTITUDE_RATE_HZ)

static uint32_t      s_last_us;
static bool          s_started;
static flight_mode_t s_mode = FLIGHT_MODE_ANGLE;
static bool          s_fallback;
static bool          s_alt_active;   /* dang giu do cao hay khong */

void ctrl_angle_init(void)
{
    s_last_us  = micros();
    s_started  = false;
    s_mode     = FLIGHT_MODE_ANGLE;
    s_fallback   = false;
    s_alt_active = false;
    ctrl_poshold_init();
    ctrl_althold_init();

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
    /*
     * Kenh am = luon ANGLE. Truoc day day la #if nen doi y phai build lai;
     * gio la phep so sanh luc chay, doi bang `set rc_mode_channel=-1`.
     */
    if (g_params.rc_mode_channel < 0) {
        return FLIGHT_MODE_ANGLE;
    }

    const uint16_t raw = g_fc.rc.channel_raw[g_params.rc_mode_channel];

    /*
     * Công tắc ba nấc. Thứ tự nấc theo mức độ AN TOÀN giảm dần, để kênh chưa
     * gán hoặc mất tín hiệu (giá trị thấp) rơi vào chế độ an toàn nhất.
     */
    /*
     * ACRO bị GỠ KHỎI CÔNG TẮC theo mặc định.
     *
     * Ở ACRO máy bay không tự cân bằng: buông cần là nó giữ nguyên góc
     * nghiêng đang có và tiếp tục lật. Gạt nhầm vào đó giữa chuyến bay thì
     * chỉ có vài giây để nhận ra và gạt về.
     *
     * Nó KHÔNG bị xoá khỏi firmware — vẫn là chế độ DỰ PHÒNG tự động khi bộ
     * ước lượng chưa có góc tin cậy (xem chỗ đặt s_fallback bên dưới). Đó là
     * lựa chọn đúng cho tình huống ấy, vì ACRO là chế độ duy nhất không cần
     * biết góc.
     *
     * Bật lại bằng `set rc_mode_acro_enable=1` khi thật sự muốn tập bay ACRO.
     */
    if (g_params.rc_mode_acro_enable != 0u &&
        raw >= g_params.rc_mode_acro_threshold) {
        return FLIGHT_MODE_ACRO;
    }
    if (raw >= g_params.rc_mode_poshold_threshold) { return FLIGHT_MODE_POSHOLD; }
    if (raw >= g_params.rc_mode_althold_threshold) { return FLIGHT_MODE_ALTHOLD; }
    return FLIGHT_MODE_ANGLE;
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
    if ((want == FLIGHT_MODE_ANGLE || want == FLIGHT_MODE_ALTHOLD ||
         want == FLIGHT_MODE_POSHOLD) &&
        !g_fc.est.attitude_valid) {
        want       = FLIGHT_MODE_ACRO;
        s_fallback = true;
    }

    /* Rời POSHOLD thì xoá tích phân, nếu không lần vào lại nó bung ra ngay. */
    if (s_mode == FLIGHT_MODE_POSHOLD && want != FLIGHT_MODE_POSHOLD) {
        ctrl_poshold_reset();
    }

    /*
     * CHƯA ARM cũng phải xoá.
     *
     * Vòng này chạy bất kể đã arm hay chưa, nên máy bay nằm trên bàn ở chế độ
     * POSHOLD thì tích phân vận tốc vẫn dồn — đo được I_y = −0,98 độ chỉ sau
     * ít phút, và trần là ±8 độ. Arm lúc đó là máy bay nghiêng ngay lập tức
     * theo một lệnh tích cóp từ khi còn nằm im.
     *
     * Đây đúng là chốt mà ctrl_rate.c đã có từ đầu và ctrl_althold.c cũng có;
     * riêng chỗ này thiếu.
     */
    if (!g_fc.motor.armed || g_fc.mode != FC_MODE_ARMED) {
        ctrl_poshold_reset();
    }

    s_mode        = want;
    g_fc.ctrl.mode = want;

    /* ==================== Quyền điều khiển ga ==========================
     *
     * ACRO và ANGLE: cần đi thẳng xuống khâu trộn, như trước.
     * ALTHOLD và POSHOLD: vòng giữ độ cao nắm quyền.
     *
     * POSHOLD BAO GỒM giữ độ cao — giữ được vị trí ngang mà độ cao vẫn phải
     * rà tay thì mới xong một nửa việc.
     * ================================================================== */
    {
        const bool want_alt = (s_mode == FLIGHT_MODE_ALTHOLD ||
                               s_mode == FLIGHT_MODE_POSHOLD);
        const bool armed    = g_fc.motor.armed && (g_fc.mode == FC_MODE_ARMED);

        if (!want_alt || !armed) {
            /*
             * Xoá trạng thái khi rời chế độ HOẶC khi chưa arm. Không xoá lúc
             * chưa arm thì tích phân dồn sẵn trên bàn, và ngay giây đầu sau
             * khi arm nó bung ra thành một cú vọt ga.
             */
            if (s_alt_active) {
                ctrl_althold_reset();
                s_alt_active = false;
            }
            g_fc.ctrl.throttle_cmd = g_fc.rc.throttle;
        } else {
            /*
             * Vừa vào chế độ: nạp tích phân theo mức ga NGƯỜI LÁI ĐANG GIỮ để
             * đầu ra không nhảy bậc. throttle_cmd lúc này còn giữ giá trị của
             * nhịp trước, tức đúng mức ga tay.
             */
            if (!s_alt_active) {
                ctrl_althold_enter(g_fc.ctrl.throttle_cmd);
                s_alt_active = true;
            }

            const float dt = (float)ANGLE_PERIOD_US * 1.0e-6f;
            float       thr;

            if (ctrl_althold_update(dt, &thr)) {
                g_fc.ctrl.throttle_cmd = thr;
            } else {
                /*
                 * Mất tin cậy độ cao. Trả ga về cần NGAY — giữ độ cao theo
                 * một ước lượng sai là cách chắc chắn nhất để đâm xuống đất.
                 */
                g_fc.ctrl.throttle_cmd = g_fc.rc.throttle;
                s_fallback             = true;
            }
        }
    }

    /*
     * --- Trục YAW: luôn điều khiển theo tốc độ ---
     * Không có la bàn thì yaw ước lượng trôi dần, giữ hướng theo nó là tự làm
     * máy bay quay đi. Xem chú thích về yaw trong ekf_attitude.h.
     */
    const float yaw_rate = g_fc.rc.yaw * g_params.rate_max_yaw_dps;

    if (s_mode == FLIGHT_MODE_ACRO) {
        /* Cần ra thẳng tốc độ quay. */
        g_fc.ctrl.setpoint_rate_dps = (vec3f_t){
            g_fc.rc.roll  * g_params.rate_max_roll_dps,
            g_fc.rc.pitch * g_params.rate_max_pitch_dps,
            yaw_rate
        };
        g_fc.ctrl.setpoint_angle_rad = (euler_t){ 0.0f, 0.0f, 0.0f };
        return true;
    }

    /* --- ANGLE / POSHOLD: cần góc mục tiêu, rồi vòng P ra tốc độ --- */

    float target_roll  = g_fc.rc.roll  * g_params.angle_max_lean_deg * FC_DEG_TO_RAD;
    float target_pitch = g_fc.rc.pitch * g_params.angle_max_lean_deg * FC_DEG_TO_RAD;

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

    /*
     * --- Bu do lech lap dat IMU ---
     *
     * Bo uoc luong can bang theo TRONG LUC, nhung no chi biet trong luc qua
     * con chip. Chip dan lech 1 do so voi khung thi "nam ngang" cua EKF la
     * nghieng 1 do cua khung, va may bay troi deu mot huong mai mai.
     *
     * Lech 0,9 do cho gia toc ngang 0,154 m/s^2 - sau muoi giay la 1,5 m/s va
     * di duoc gan 8 met. Khong cach nao chinh PID cho het, vi bo dieu khien
     * dang BAM DUNG mot muc tieu SAI.
     *
     * DAT O DAY, KHONG DAT O CHO TINH TU CAN:
     *   Nhanh POSHOLD ben tren GHI DE target_roll/target_pitch bang goc do
     *   vong giu van toc quyet dinh. Cong trim truoc do thi POSHOLD xoa mat.
     *   Cong o day thi ca ANGLE, ALTHOLD lan POSHOLD deu duoc bu.
     *
     *   Rieng ACRO da thoat som phia tren - dung vay, ACRO khong co goc muc
     *   tieu de ma bu.
     *
     * Trong POSHOLD, khau tich phan cua vong van toc von cung tu bu duoc do
     * lech nay. Nhung de no phai bu nghia la tich phan luon dung o mot gia
     * tri khac 0, an mat du dia danh cho gio. Bu san o day thi tich phan bat
     * dau tu gan 0.
     *
     * CACH DO: dat may bay len mat DA KIEM PHANG bang nivo, doc roll/pitch o
     * mode 16, roi dat trim bang DUNG so doc duoc doi dau.
     */
    target_roll  += g_params.angle_trim_roll_deg  * FC_DEG_TO_RAD;
    target_pitch += g_params.angle_trim_pitch_deg * FC_DEG_TO_RAD;

    g_fc.ctrl.setpoint_angle_rad.roll  = target_roll;
    g_fc.ctrl.setpoint_angle_rad.pitch = target_pitch;
    g_fc.ctrl.setpoint_angle_rad.yaw   = 0.0f;

    /* Sai lệch góc, đổi sang độ vì angle_pid_kp tính theo độ. */
    const float err_roll_deg  =
        (target_roll  - g_fc.est.attitude_rad.roll)  * FC_RAD_TO_DEG;
    const float err_pitch_deg =
        (target_pitch - g_fc.est.attitude_rad.pitch) * FC_RAD_TO_DEG;

    /*
     * Chỉ khâu P, và chặn trần tốc độ yêu cầu. Không chặn thì bật ANGLE lúc
     * đang nghiêng 60° sẽ đòi 300 °/s và máy bay giật nảy rất mạnh.
     */
    const float rate_roll = fc_constrainf(g_params.angle_pid_kp * err_roll_deg,
                                          -g_params.angle_max_rate_dps,
                                          g_params.angle_max_rate_dps);
    const float rate_pitch = fc_constrainf(g_params.angle_pid_kp * err_pitch_deg,
                                           -g_params.angle_max_rate_dps,
                                           g_params.angle_max_rate_dps);

    g_fc.ctrl.setpoint_rate_dps = (vec3f_t){ rate_roll, rate_pitch, yaw_rate };
    return true;
}

flight_mode_t ctrl_angle_active_mode(void) { return s_mode; }
bool          ctrl_angle_fallback(void)    { return s_fallback; }
