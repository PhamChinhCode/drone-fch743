/**
 * @file    mixer.c
 * @brief   Hiện thực khâu trộn quad X.
 */

#include "mixer.h"
#include "fc_state.h"
#include "param_table.h"

/*
 * Bảng trộn. Mỗi hàng là một motor, ba cột là hệ số của roll, pitch, yaw.
 *
 * Thứ tự hàng bám đúng số motor trên console (M1..M4), và vị trí vật lý của
 * chúng là số đo thực tế của bo này chứ không phải quy ước mặc định:
 *
 *   M1 trước-phải : bên phải -> roll âm    ; phía trước -> pitch dương
 *   M2 trước-trái : bên trái -> roll dương ; phía trước -> pitch dương
 *   M3 sau-trái   : bên trái -> roll dương ; phía sau   -> pitch âm
 *   M4 sau-phải   : bên phải -> roll âm    ; phía sau   -> pitch âm
 *
 * DẤU PITCH TỪNG BỊ VIẾT NGƯỢC Ở ĐÂY — ghi lại để không tái phạm.
 * Motor đẩy LÊN, nên nó hoạt động như bập bênh: tăng lực ở phía TRƯỚC thì đầu
 * trước bị nâng lên, tức mũi NGÓC LÊN. Trực giác "muốn ngóc mũi thì đẩy đuôi
 * xuống" là sai, vì motor không kéo xuống được, chỉ đẩy lên.
 *
 * Triệu chứng khi sai dấu: ngẩng mũi bằng tay thì hai motor TRƯỚC quay nhanh
 * lên — tức PID phụ hoạ cho nhiễu loạn thay vì chống lại. Lắp cánh vào là lộn
 * nhào ngay giây đầu.
 *
 * Cột yaw: hai motor cùng đường chéo nhận cùng dấu. {M1,M3} một dấu,
 * {M2,M4} dấu ngược lại. mix_yaw_sign quyết định đường chéo nào dương.
 *
 * KHONG con `const`: cot yaw duoc dung lai tu g_params trong
 * mixer_apply_params(). Giu bang dung san thay vi nhan dau o cho dung de
 * vong nong khong doi mot phep nhan nao - bang nay duoc doc 4000 lan moi
 * giay cho moi motor.
 */
static float s_mix[FC_MOTOR_COUNT][AXIS_COUNT] = {
    /*         roll    pitch     yaw   */
    /* M1 */ { -1.0f, +1.0f, +1.0f },
    /* M2 */ { +1.0f, +1.0f, -1.0f },
    /* M3 */ { +1.0f, -1.0f, +1.0f },
    /* M4 */ { -1.0f, -1.0f, -1.0f },
};

void mixer_apply_params(void)
{
    /*
     * Dau cua ca cot yaw. Cac o mang giu dang +1/-1 co dinh (quan he giua bon
     * motor la co dinh - do la hinh hoc cua khung X), chi rieng DAU CHUNG la
     * doi duoc, vi no phu thuoc chieu quay that cua canh quat.
     */
    const float sign = (g_params.mix_yaw_sign < 0) ? -1.0f : +1.0f;

    s_mix[0][AXIS_YAW] = sign * +1.0f;
    s_mix[1][AXIS_YAW] = sign * -1.0f;
    s_mix[2][AXIS_YAW] = sign * +1.0f;
    s_mix[3][AXIS_YAW] = sign * -1.0f;
}

static bool  s_saturated;
static float s_scale = 1.0f;

void mixer_init(void)
{
    mixer_apply_params();   /* dung bang tron theo mix_yaw_sign dang co */

    s_saturated = false;
    s_scale     = 1.0f;

    for (int i = 0; i < FC_MOTOR_COUNT; i++) {
        g_fc.motor.output_norm[i] = 0.0f;
    }
    g_fc.motor.saturated = false;
}

void mixer_update(void)
{
    /*
     * Chưa arm thì không trộn gì cả. Đây KHÔNG phải chốt an toàn — chốt thật
     * nằm ở dshot.c và arming.c — nhưng để output_norm bằng 0 khi disarm giúp
     * console đọc đúng và tránh giá trị cũ đọng lại.
     */
    if (!g_fc.motor.armed || g_fc.mode != FC_MODE_ARMED) {
        mixer_init();
        return;
    }

    float roll  = g_fc.ctrl.pid_output.x;
    float pitch = g_fc.ctrl.pid_output.y;
    float yaw   = g_fc.ctrl.pid_output.z;
    float thr   = g_fc.ctrl.throttle_cmd;

#if MIX_STICK_PASSTHROUGH
    /*
     * Chế độ bring-up: cần điều khiển đi thẳng vào khâu trộn, không qua PID.
     * Chỉ để kiểm dấu khi tháo cánh quạt — xem MIX_STICK_PASSTHROUGH.
     */
    roll  = g_fc.rc.roll  * MIX_PASSTHROUGH_GAIN;
    pitch = g_fc.rc.pitch * MIX_PASSTHROUGH_GAIN;
    yaw   = g_fc.rc.yaw   * MIX_PASSTHROUGH_GAIN;
    thr   = g_fc.rc.throttle;
#endif

    /* --- Phần lệnh điều khiển, chưa có ga --- */
    float mix[FC_MOTOR_COUNT];
    float mix_min = 0.0f;
    float mix_max = 0.0f;

    for (int i = 0; i < FC_MOTOR_COUNT; i++) {
        mix[i] = roll  * s_mix[i][AXIS_ROLL]
               + pitch * s_mix[i][AXIS_PITCH]
               + yaw   * s_mix[i][AXIS_YAW];

        if (mix[i] < mix_min) { mix_min = mix[i]; }
        if (mix[i] > mix_max) { mix_max = mix[i]; }
    }

    /*
     * --- Thu nhỏ khi lệnh vượt quá dư địa ---
     *
     * Biên độ của phần điều khiển là mix_max - mix_min. Vượt quá 1 thì dù đặt
     * ga ở đâu cũng có motor tràn. Thu nhỏ CẢ BỐN theo cùng hệ số để giữ
     * nguyên tỉ lệ giữa chúng: máy bay nghiêng đúng hướng, chỉ yếu đi.
     */
    const float range = mix_max - mix_min;

    s_scale     = 1.0f;
    s_saturated = false;

    if (range > 1.0f) {
        s_scale     = 1.0f / range;
        s_saturated = true;

        for (int i = 0; i < FC_MOTOR_COUNT; i++) {
            mix[i] *= s_scale;
        }
        mix_min *= s_scale;
        mix_max *= s_scale;
    }

    /*
     * --- Đặt ga vào phần còn trống ---
     *
     * Sau khi thu nhỏ, phần điều khiển chiếm dải [mix_min, mix_max]. Ga chỉ
     * được nằm trong khoảng còn lại thì mới không motor nào tràn:
     *     -mix_min <= ga <= 1 - mix_max
     *
     * Ép ga vào khoảng đó, tức GA bị hy sinh chứ không phải quyền điều khiển.
     * Mất độ cao còn cứu được; mất điều khiển tư thế thì không.
     */
    const float thr_min = -mix_min;
    const float thr_max = 1.0f - mix_max;

    if (thr < thr_min) { thr = thr_min; s_saturated = true; }
    if (thr > thr_max) { thr = thr_max; s_saturated = true; }

    for (int i = 0; i < FC_MOTOR_COUNT; i++) {
        /* Chặn lần cuối: sau các bước trên thì không được cắt gì nữa, nhưng
         * để đây phòng sai số làm tròn đẩy giá trị ra ngoài vài phần triệu. */
        g_fc.motor.output_norm[i] = fc_constrainf(thr + mix[i], 0.0f, 1.0f);
    }

    g_fc.motor.saturated    = s_saturated;
    g_fc.motor.timestamp_us = g_fc.imu.timestamp_us;
}

bool  mixer_saturated(void) { return s_saturated; }
float mixer_scale(void)     { return s_scale; }
