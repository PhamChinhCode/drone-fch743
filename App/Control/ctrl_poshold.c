/**
 * @file    ctrl_poshold.c
 * @brief   Hiện thực vòng giữ vận tốc bằng optical flow.
 */

#include "ctrl_poshold.h"
#include "ekf_velocity.h"
#include "fc_state.h"
#include "param_table.h"
#include "ctrl_offboard.h"

static float s_int_x;           /* tích phân trục thân X, độ */
static float s_int_y;

static vec3f_t s_v_body;        /* vận tốc thân đang đo, để soi bằng mắt */
static vec3f_t s_target_body;

/*
 * Mốc vị trí đang giữ, hệ NED, mét. Chỉ có nghĩa khi s_pos_locked = true.
 *
 * Đây là thứ biến vòng này từ GIỮ VẬN TỐC thành GIỮ VỊ TRÍ. Giữ vận tốc bằng
 * 0 nghe như là đứng yên, nhưng sai số vận tốc dù nhỏ vẫn tích luỹ thành trôi
 * — không có gì kéo máy bay về chỗ cũ. Có mốc vị trí thì mỗi mét trôi đi sinh
 * ra một lệnh vận tốc ngược chiều để bò về.
 */
static float s_tgt_n, s_tgt_e;
static bool  s_pos_locked;

void ctrl_poshold_reset(void)
{
    s_int_x      = 0.0f;
    s_int_y      = 0.0f;
    s_pos_locked = false;   /* mốc sẽ được chốt lại ở nhịp chạy kế tiếp */
}

void ctrl_poshold_init(void)
{
    ctrl_poshold_reset();
    s_v_body      = (vec3f_t){ 0.0f, 0.0f, 0.0f };
    s_target_body = (vec3f_t){ 0.0f, 0.0f, 0.0f };
}

bool ctrl_poshold_update(float dt, float *roll_rad, float *pitch_rad)
{
    /*
     * Không có ước lượng vận tốc tin cậy thì không có gì để bám. Trả false để
     * bên gọi lùi về ANGLE — bám một vận tốc không đo được là cách chắc chắn
     * nhất để máy bay lao đi mất.
     */
    if (!ekf_velocity_is_valid() || dt <= 0.0f || dt > 0.1f) {
        return false;
    }

    /*
     * --- Xoay vận tốc từ hệ NED về hệ THÂN ---
     * Bộ ước lượng cho vận tốc hệ NED, nhưng cần điều khiển và góc nghiêng
     * đều là khái niệm hệ thân. Tính toàn bộ ở hệ thân cho gọn.
     */
    const float yaw = g_fc.est.attitude_rad.yaw;
    const float cy  = cosf(yaw);
    const float sy  = sinf(yaw);

    const float vn = g_fc.est.velocity_mps.x;
    const float ve = g_fc.est.velocity_mps.y;

    const float v_fwd   =  vn * cy + ve * sy;   /* dương = đang bay tới  */
    const float v_right = -vn * sy + ve * cy;   /* dương = đang bay phải */

    s_v_body = (vec3f_t){ v_fwd, v_right, 0.0f };

    /*
     * --- Vận tốc mong muốn từ cần điều khiển ---
     * Cần ở giữa -> mong muốn 0, tức GHÌ ĐỨNG YÊN. Đẩy cần thì bay theo hướng
     * đó với tốc độ tỉ lệ.
     *
     * rc.pitch dương là NGÓC MŨI, tức bay LÙI, nên phải đảo dấu để cần đẩy tới
     * ra vận tốc tới dương.
     */
    float tgt_fwd, tgt_right;

    /*
     * rc.roll/rc.pitch đã bị trừ vùng chết ở tầng CRSF nên chúng bằng ĐÚNG 0
     * khi cần ở giữa. Không cần thêm ngưỡng ở đây.
     */
    const bool stick_active = (g_fc.rc.pitch != 0.0f) || (g_fc.rc.roll != 0.0f);

    if (ctrl_offboard_is_active()) {
        /*
         * --- OFFBOARD: mục tiêu vận tốc do máy tính nhúng đặt ---
         *
         * Lệnh đã qua đủ lớp lọc và giới hạn bao trong ctrl_offboard, nên tới
         * đây nó chỉ còn là một mục tiêu vận tốc hệ thân bình thường — đúng
         * thứ mà phần còn lại của hàm này vốn đã biết xử lý.
         *
         * Mốc vị trí BÁM THEO chỗ hiện tại, giống hệt nhánh cần điều khiển.
         * Không bám thì lúc rời OFFBOARD máy bay quay đầu bò về điểm đã vào
         * chế độ — đúng kiểu hỏng khiến người lái hoảng, và nó xảy ra đúng
         * vào lúc vừa có sự cố.
         */
        const vec3f_t ext = ctrl_offboard_velocity_body();

        tgt_fwd   = ext.x;
        tgt_right = ext.y;

        s_tgt_n      = g_fc.est.position_m.x;
        s_tgt_e      = g_fc.est.position_m.y;
        s_pos_locked = g_fc.est.position_valid;
    } else if (stick_active || !g_fc.est.position_valid) {
        /*
         * --- Người lái đang cầm lái, hoặc chưa tin được vị trí ---
         *
         * Cần ra lệnh VẬN TỐC trực tiếp, và mốc vị trí BÁM THEO chỗ hiện tại.
         * Không bám theo thì lúc thả cần, máy bay quay đầu bò ngược về điểm
         * xuất phát của cả chuyến — đúng kiểu hỏng khiến người lái hoảng.
         *
         * rc.pitch dương là NGÓC MŨI tức bay LÙI, nên đảo dấu.
         */
        tgt_fwd   = -g_fc.rc.pitch * g_params.poshold_max_vel_mps;
        tgt_right =  g_fc.rc.roll  * g_params.poshold_max_vel_mps;

        s_tgt_n      = g_fc.est.position_m.x;
        s_tgt_e      = g_fc.est.position_m.y;
        s_pos_locked = g_fc.est.position_valid;
    } else {
        /*
         * --- Buông cần: GIỮ CHỖ ---
         *
         * Vòng ngoài chỉ có P, giống hệt vòng góc: sai số vị trí đổi ra vận
         * tốc mong muốn, rồi kẹp lại. Kẹp là bắt buộc — lệch 20 m mà không
         * kẹp thì nó đòi lao về với tốc độ không điều khiển nổi.
         */
        if (!s_pos_locked) {
            s_tgt_n      = g_fc.est.position_m.x;
            s_tgt_e      = g_fc.est.position_m.y;
            s_pos_locked = true;
        }

        const float vmax = g_params.poshold_max_vel_mps;

        const float vd_n = fc_constrainf(
            g_params.poshold_pos_kp * (s_tgt_n - g_fc.est.position_m.x),
            -vmax, vmax);
        const float vd_e = fc_constrainf(
            g_params.poshold_pos_kp * (s_tgt_e - g_fc.est.position_m.y),
            -vmax, vmax);

        /* Vận tốc mong muốn đang ở hệ NED, xoay về hệ thân cho vòng trong. */
        tgt_fwd   =  vd_n * cy + vd_e * sy;
        tgt_right = -vd_n * sy + vd_e * cy;
    }

    s_target_body = (vec3f_t){ tgt_fwd, tgt_right, 0.0f };

    const float err_fwd   = tgt_fwd   - v_fwd;
    const float err_right = tgt_right - v_right;

    /* --- Tích phân, chống gió --- */
    s_int_x = fc_constrainf(s_int_x + g_params.poshold_vel_ki * err_fwd   * dt,
                            -g_params.poshold_i_limit_deg,
                            g_params.poshold_i_limit_deg);
    s_int_y = fc_constrainf(s_int_y + g_params.poshold_vel_ki * err_right * dt,
                            -g_params.poshold_i_limit_deg,
                            g_params.poshold_i_limit_deg);

    /*
     * --- Sai số vận tốc -> góc nghiêng ---
     *
     * Muốn tăng tốc TỚI thì phải CHÚC MŨI XUỐNG, tức pitch ÂM — nên có dấu
     * trừ. Muốn tăng tốc SANG PHẢI thì nghiêng phải, tức roll DƯƠNG.
     */
    const float pitch_deg = -(g_params.poshold_vel_kp * err_fwd   + s_int_x);
    const float roll_deg  =  (g_params.poshold_vel_kp * err_right + s_int_y);

    /*
     * Trần nghiêng đặt THẤP hơn est_flow_max_tilt_deg là có chủ ý: nghiêng
     * vượt ngưỡng kia thì bộ ước lượng từ chối mẫu flow, tức vòng này tự cắt
     * mất nguồn đo của chính nó rồi lùi về ANGLE giữa chừng.
     */
    *pitch_rad = fc_constrainf(pitch_deg, -g_params.poshold_max_tilt_deg,
                               g_params.poshold_max_tilt_deg) * FC_DEG_TO_RAD;
    *roll_rad  = fc_constrainf(roll_deg, -g_params.poshold_max_tilt_deg,
                               g_params.poshold_max_tilt_deg) * FC_DEG_TO_RAD;
    return true;
}

vec3f_t ctrl_poshold_velocity_body(void) { return s_v_body; }
vec3f_t ctrl_poshold_target_body(void)   { return s_target_body; }

vec3f_t ctrl_poshold_target_ned(void)
{
    return (vec3f_t){ s_tgt_n, s_tgt_e, 0.0f };
}

bool ctrl_poshold_position_locked(void) { return s_pos_locked; }

vec3f_t ctrl_poshold_integral_deg(void)
{
    return (vec3f_t){ s_int_x, s_int_y, 0.0f };
}
