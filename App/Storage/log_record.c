/**
 * @file    log_record.c
 * @brief   Hiện thực bản ghi log. Xem log_record.h.
 */

#include "log_record.h"
#include "fc_state.h"

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
    "t_ms,gx,gy,gz,ax,ay,az,sp_x,sp_y,sp_z,pid_x,pid_y,pid_z,"
    "m1,m2,m3,m4,roll,pitch,yaw,alt_m,thr,mode,armed,sat\r\n";

int log_record_to_csv(char *out, const bb_record_t *r)
{
    int n = 0;

    n += wr_int(&out[n], (int32_t)r->t_ms);
    for (int i = 0; i < 3; i++) { out[n++] = ','; n += wr_fixed(&out[n], r->gyro[i], 1); }
    for (int i = 0; i < 3; i++) { out[n++] = ','; n += wr_fixed(&out[n], r->accel[i], 3); }
    for (int i = 0; i < 3; i++) { out[n++] = ','; n += wr_fixed(&out[n], r->sp[i], 1); }
    for (int i = 0; i < 3; i++) { out[n++] = ','; n += wr_fixed(&out[n], r->pid[i], 4); }
    for (int i = 0; i < 4; i++) { out[n++] = ','; n += wr_int(&out[n], (int32_t)r->motor[i]); }
    for (int i = 0; i < 3; i++) { out[n++] = ','; n += wr_fixed(&out[n], r->att[i], 2); }
    out[n++] = ','; n += wr_fixed(&out[n], r->alt_cm, 2);
    out[n++] = ','; n += wr_fixed(&out[n], (int32_t)r->thr, 4);
    out[n++] = ','; n += wr_int(&out[n], r->mode);
    out[n++] = ','; n += wr_int(&out[n], (r->flags & 0x01u) ? 1 : 0);
    out[n++] = ','; n += wr_int(&out[n], (r->flags & 0x02u) ? 1 : 0);
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

void log_record_fill(bb_record_t *r, uint32_t t_ms)
{
    r->t_ms = t_ms;

    r->gyro[0] = clamp16((int32_t)(g_fc.imu.gyro_filtered_dps.x * 10.0f));
    r->gyro[1] = clamp16((int32_t)(g_fc.imu.gyro_filtered_dps.y * 10.0f));
    r->gyro[2] = clamp16((int32_t)(g_fc.imu.gyro_filtered_dps.z * 10.0f));

    r->accel[0] = clamp16((int32_t)(g_fc.imu.accel_mps2.x / FC_GRAVITY_MPS2 * 1000.0f));
    r->accel[1] = clamp16((int32_t)(g_fc.imu.accel_mps2.y / FC_GRAVITY_MPS2 * 1000.0f));
    r->accel[2] = clamp16((int32_t)(g_fc.imu.accel_mps2.z / FC_GRAVITY_MPS2 * 1000.0f));

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
    r->mode   = (uint8_t)g_fc.ctrl.mode;
    r->flags  = (uint8_t)((g_fc.motor.armed ? 0x01u : 0u) |
                          (g_fc.motor.saturated ? 0x02u : 0u));
}
