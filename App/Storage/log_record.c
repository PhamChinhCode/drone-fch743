/**
 * @file    log_record.c
 * @brief   Hiện thực bản ghi log. Xem log_record.h.
 */

#include "log_record.h"
#include "fc_state.h"
#include "ctrl_poshold.h"
#include "ctrl_althold.h"
#include "ekf_altitude.h"

#include <math.h>

/* ==========================================================================
 * Định dạng số — tự viết thay vì dùng printf
 *
 * snprintf với %f kéo theo cả bộ định dạng dấu phẩy động của thư viện C,
 * tốn vài chục KB flash và chậm. Ở đây chỉ cần in số nguyên có dấu và số
 * thập phân tỉ lệ cố định, viết tay vài chục dòng là xong.
 * ========================================================================== */

/** Số nguyên có dấu. Trả về số ký tự đã ghi. */
static int wr_int(char *p, int32_t v)
{
    char    tmp[12];
    int     n = 0;
    int     len = 0;
    uint32_t u;

    if (v < 0) {
        p[len++] = '-';
        u = (uint32_t)(-(int64_t)v);
    } else {
        u = (uint32_t)v;
    }

    do {
        tmp[n++] = (char)('0' + (u % 10u));
        u /= 10u;
    } while (u != 0u);

    while (n > 0) {
        p[len++] = tmp[--n];
    }
    return len;
}

/**
 * Số thập phân tỉ lệ cố định: in v/10^dec.
 * Ví dụ wr_fixed(p, -125, 1) cho ra "-12.5".
 */
static int wr_fixed(char *p, int32_t v, int dec)
{
    int32_t div = 1;
    int     len = 0;

    for (int i = 0; i < dec; i++) { div *= 10; }

    if (v < 0) {
        p[len++] = '-';
        v = -v;
    }
    len += wr_int(&p[len], v / div);

    if (dec > 0) {
        int32_t frac = v % div;

        p[len++] = '.';
        for (int i = dec - 1; i >= 0; i--) {
            int32_t d = frac;

            for (int k = 0; k < i; k++) { d /= 10; }
            p[len++] = (char)('0' + (d % 10));
        }
    }
    return len;
}

const char g_log_csv_header[] =
    "t_ms,gx,gy,gz,az,sp_x,sp_y,sp_z,pid_x,pid_y,pid_z,"
    "m1,m2,m3,m4,roll,pitch,yaw,alt_m,thr,mode,armed,sat,"
    "sp_roll,sp_pitch,v_fwd,v_right,vt_fwd,vt_right,range_m,flow_q,est_flags,"
    "rc_thr,climb,climb_tgt,alt_err,alt_i\r\n";

int log_record_to_csv(char *out, const bb_record_t *r)
{
    int n = 0;

    n += wr_int(&out[n], (int32_t)r->t_ms);
    for (int i = 0; i < 3; i++) { out[n++] = ','; n += wr_fixed(&out[n], r->gyro[i], 1); }
    out[n++] = ','; n += wr_fixed(&out[n], r->accel_z, 3);
    for (int i = 0; i < 3; i++) { out[n++] = ','; n += wr_fixed(&out[n], r->sp[i], 1); }
    for (int i = 0; i < 3; i++) { out[n++] = ','; n += wr_fixed(&out[n], r->pid[i], 4); }
    for (int i = 0; i < 4; i++) { out[n++] = ','; n += wr_int(&out[n], (int32_t)r->motor[i]); }
    for (int i = 0; i < 3; i++) { out[n++] = ','; n += wr_fixed(&out[n], r->att[i], 2); }
    out[n++] = ','; n += wr_fixed(&out[n], r->alt_cm, 2);
    out[n++] = ','; n += wr_fixed(&out[n], (int32_t)r->thr, 4);
    out[n++] = ','; n += wr_int(&out[n], (r->flags & LOG_FLAG_MODE_MASK) >> LOG_FLAG_MODE_SHIFT);
    out[n++] = ','; n += wr_int(&out[n], (r->flags & 0x01u) ? 1 : 0);
    out[n++] = ','; n += wr_int(&out[n], (r->flags & 0x02u) ? 1 : 0);

    for (int i = 0; i < 2; i++) { out[n++] = ','; n += wr_fixed(&out[n], r->sp_angle[i], 2); }
    for (int i = 0; i < 2; i++) { out[n++] = ','; n += wr_fixed(&out[n], r->vel_body[i], 2); }
    for (int i = 0; i < 2; i++) { out[n++] = ','; n += wr_fixed(&out[n], r->vel_tgt[i], 2); }
    out[n++] = ','; n += wr_fixed(&out[n], (int32_t)r->range_cm, 2);
    out[n++] = ','; n += wr_int(&out[n], r->flow_q);
    out[n++] = ','; n += wr_int(&out[n], r->est_flags);

    out[n++] = ','; n += wr_fixed(&out[n], (int32_t)r->rc_thr * 5, 3);
    out[n++] = ','; n += wr_fixed(&out[n], (int32_t)r->climb * 5, 2);
    out[n++] = ','; n += wr_fixed(&out[n], (int32_t)r->climb_tgt * 5, 2);
    out[n++] = ','; n += wr_fixed(&out[n], (int32_t)r->alt_err * 2, 2);
    out[n++] = ','; n += wr_fixed(&out[n], (int32_t)r->alt_i * 5, 3);
    out[n++] = '\r';
    out[n++] = '\n';

    return n;
}

/* ==========================================================================
 * Chụp trạng thái
 * ========================================================================== */

static int16_t clamp16(int32_t v)
{
    if (v >  32767) { return  32767; }
    if (v < -32768) { return -32768; }
    return (int16_t)v;
}

/** Làm tròn rồi kẹp vào int8. */
static int8_t clamp8(float v)
{
    const float r = roundf(v);
    if (r >  127.0f) { return  127; }
    if (r < -128.0f) { return -128; }
    return (int8_t)r;
}

void log_record_fill(bb_record_t *r, uint32_t t_ms)
{
    r->t_ms = t_ms;

    r->gyro[0] = clamp16((int32_t)(g_fc.imu.gyro_filtered_dps.x * 10.0f));
    r->gyro[1] = clamp16((int32_t)(g_fc.imu.gyro_filtered_dps.y * 10.0f));
    r->gyro[2] = clamp16((int32_t)(g_fc.imu.gyro_filtered_dps.z * 10.0f));

    r->accel_z = clamp16((int32_t)(g_fc.imu.accel_mps2.z / FC_GRAVITY_MPS2 * 1000.0f));

    r->sp[0] = clamp16((int32_t)(g_fc.ctrl.setpoint_rate_dps.x * 10.0f));
    r->sp[1] = clamp16((int32_t)(g_fc.ctrl.setpoint_rate_dps.y * 10.0f));
    r->sp[2] = clamp16((int32_t)(g_fc.ctrl.setpoint_rate_dps.z * 10.0f));

    r->pid[0] = clamp16((int32_t)(g_fc.ctrl.pid_output.x * 10000.0f));
    r->pid[1] = clamp16((int32_t)(g_fc.ctrl.pid_output.y * 10000.0f));
    r->pid[2] = clamp16((int32_t)(g_fc.ctrl.pid_output.z * 10000.0f));

    for (int i = 0; i < FC_MOTOR_COUNT && i < 4; i++) {
        r->motor[i] = g_fc.motor.throttle[i];
    }

    r->att[0] = clamp16((int32_t)(g_fc.est.attitude_rad.roll  * 100.0f));
    r->att[1] = clamp16((int32_t)(g_fc.est.attitude_rad.pitch * 100.0f));
    r->att[2] = clamp16((int32_t)(g_fc.est.attitude_rad.yaw   * 100.0f));

    r->alt_cm = clamp16((int32_t)(g_fc.est.altitude_m * 100.0f));
    r->thr    = (uint16_t)clamp16((int32_t)(g_fc.ctrl.throttle_cmd * 10000.0f));
    r->flags  = (uint8_t)((g_fc.motor.armed ? 0x01u : 0u) |
                          (g_fc.motor.saturated ? 0x02u : 0u) |
                          (((uint32_t)g_fc.ctrl.mode << LOG_FLAG_MODE_SHIFT) &
                           LOG_FLAG_MODE_MASK));

    /* ------------------------------------------------------------------
     * Chẩn đoán giữ vị trí
     * ------------------------------------------------------------------ */

    /*
     * Góc mà tầng trên RA LỆNH. Đây là ranh giới quan trọng nhất: so nó với
     * att[] là biết vòng góc có thực hiện nổi lệnh không, mà so nó với
     * vel_tgt[] là biết poshold có ra lệnh đúng không. Thiếu nó thì hai loại
     * lỗi ấy nhìn giống hệt nhau.
     */
    r->sp_angle[0] = clamp16((int32_t)(g_fc.ctrl.setpoint_angle_rad.roll  * 100.0f));
    r->sp_angle[1] = clamp16((int32_t)(g_fc.ctrl.setpoint_angle_rad.pitch * 100.0f));

    /*
     * Vận tốc thân tính LẠI từ ước lượng NED, chứ không lấy qua
     * ctrl_poshold_velocity_body().
     *
     * Cùng công thức, cùng đầu vào, nên lúc poshold đang chạy thì hai bên ra
     * đúng một số. Khác ở chỗ: ctrl_poshold về 0 khi poshold không chạy, còn
     * cách này vẫn cho số thật — tức vẫn dùng được đúng vào lúc cần biết vì
     * sao poshold từ chối vào.
     */
    {
        const float yaw = g_fc.est.attitude_rad.yaw;
        const float cy  = cosf(yaw);
        const float sy  = sinf(yaw);
        const float vn  = g_fc.est.velocity_mps.x;
        const float ve  = g_fc.est.velocity_mps.y;

        r->vel_body[0] = clamp16((int32_t)((  vn * cy + ve * sy) * 100.0f));
        r->vel_body[1] = clamp16((int32_t)(( -vn * sy + ve * cy) * 100.0f));
    }

    /* Vận tốc poshold ĐANG ĐÒI. Bằng 0 khi poshold không chạy - đúng như vậy. */
    {
        const vec3f_t tgt = ctrl_poshold_target_body();

        r->vel_tgt[0] = clamp16((int32_t)(tgt.x * 100.0f));
        r->vel_tgt[1] = clamp16((int32_t)(tgt.y * 100.0f));
    }

    /* Hệ số quy đổi của optical flow tỉ lệ thuận với độ cao, nên range sai
     * bao nhiêu phần trăm thì vận tốc sai bấy nhiêu. */
    r->range_cm = (uint16_t)(g_fc.flow.range_mm / 10u);
    r->flow_q   = g_fc.flow.flow_quality;

    r->est_flags = (uint8_t)(
        (g_fc.est.position_valid ? LOG_EST_POS_VALID   : 0u) |
        (g_fc.est.altitude_valid ? LOG_EST_ALT_VALID   : 0u) |
        (g_fc.est.attitude_valid ? LOG_EST_ATT_VALID   : 0u) |
        (g_fc.flow.healthy       ? LOG_EST_FLOW_OK     : 0u) |
        (g_fc.flow.range_valid   ? LOG_EST_RANGE_VALID : 0u) |
        (ctrl_poshold_position_locked() ? LOG_EST_POS_LOCKED : 0u) |
        (ekf_altitude_vibe_active()     ? LOG_EST_ALT_VIBE   : 0u) |
        (ctrl_althold_sat_guard()       ? LOG_EST_SAT_GUARD  : 0u));

    /* ------------------------------------------------------------------
     * Chẩn đoán giữ độ cao
     * ------------------------------------------------------------------ */

    /*
     * Mốc độ cao chỉ có nghĩa khi vòng giữ độ cao đang chạy; lúc khác
     * ctrl_althold_reset() đặt nó về 0 và hiệu số thành -độ_cao, vô nghĩa.
     * Tốc độ đòi và tích phân thì reset về 0 sẵn nên không cần chặn.
     */
    const flight_mode_t m = g_fc.ctrl.mode;
    const bool alt_on = g_fc.motor.armed &&
                        (m == FLIGHT_MODE_ALTHOLD || m == FLIGHT_MODE_POSHOLD ||
                         m == FLIGHT_MODE_OFFBOARD);

    r->rc_thr    = (uint8_t)fc_constrainf(g_fc.rc.throttle * 200.0f + 0.5f, 0.0f, 255.0f);
    r->climb     = clamp8(g_fc.est.climb_rate_mps * 20.0f);
    r->climb_tgt = clamp8(ctrl_althold_climb_target() * 20.0f);
    r->alt_err   = alt_on ? clamp8((ctrl_althold_target_m() - g_fc.est.altitude_m) * 50.0f) : 0;
    r->alt_i     = clamp8(ctrl_althold_integral() * 200.0f);
}
