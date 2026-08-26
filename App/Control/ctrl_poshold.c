/**
 * @file    ctrl_poshold.c
 * @brief   Hiện thực vòng giữ vận tốc bằng optical flow.
 */

#include "ctrl_poshold.h"
#include "ekf_velocity.h"
#include "fc_state.h"

static float s_int_x;           /* tích phân trục thân X, độ */
static float s_int_y;

static vec3f_t s_v_body;        /* vận tốc thân đang đo, để soi bằng mắt */
static vec3f_t s_target_body;

void ctrl_poshold_reset(void)
{
    s_int_x = 0.0f;
    s_int_y = 0.0f;
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
    const float tgt_fwd   = -g_fc.rc.pitch * POSHOLD_MAX_VEL_MPS;
    const float tgt_right =  g_fc.rc.roll  * POSHOLD_MAX_VEL_MPS;

    s_target_body = (vec3f_t){ tgt_fwd, tgt_right, 0.0f };

    const float err_fwd   = tgt_fwd   - v_fwd;
    const float err_right = tgt_right - v_right;

    /* --- Tích phân, chống gió --- */
    s_int_x = fc_constrainf(s_int_x + POSHOLD_VEL_KI * err_fwd   * dt,
                            -POSHOLD_I_LIMIT_DEG, POSHOLD_I_LIMIT_DEG);
    s_int_y = fc_constrainf(s_int_y + POSHOLD_VEL_KI * err_right * dt,
                            -POSHOLD_I_LIMIT_DEG, POSHOLD_I_LIMIT_DEG);

    /*
     * --- Sai số vận tốc -> góc nghiêng ---
     *
     * Muốn tăng tốc TỚI thì phải CHÚC MŨI XUỐNG, tức pitch ÂM — nên có dấu
     * trừ. Muốn tăng tốc SANG PHẢI thì nghiêng phải, tức roll DƯƠNG.
     */
    const float pitch_deg = -(POSHOLD_VEL_KP * err_fwd   + s_int_x);
    const float roll_deg  =  (POSHOLD_VEL_KP * err_right + s_int_y);

    /*
     * Trần nghiêng đặt THẤP hơn EST_FLOW_MAX_TILT_DEG là có chủ ý: nghiêng
     * vượt ngưỡng kia thì bộ ước lượng từ chối mẫu flow, tức vòng này tự cắt
     * mất nguồn đo của chính nó rồi lùi về ANGLE giữa chừng.
     */
    *pitch_rad = fc_constrainf(pitch_deg, -POSHOLD_MAX_TILT_DEG,
                               POSHOLD_MAX_TILT_DEG) * FC_DEG_TO_RAD;
    *roll_rad  = fc_constrainf(roll_deg, -POSHOLD_MAX_TILT_DEG,
                               POSHOLD_MAX_TILT_DEG) * FC_DEG_TO_RAD;
    return true;
}

vec3f_t ctrl_poshold_velocity_body(void) { return s_v_body; }
vec3f_t ctrl_poshold_target_body(void)   { return s_target_body; }

vec3f_t ctrl_poshold_integral_deg(void)
{
    return (vec3f_t){ s_int_x, s_int_y, 0.0f };
}
