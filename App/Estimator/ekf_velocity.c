/**
 * @file    ekf_velocity.c
 * @brief   Hiện thực ước lượng vận tốc ngang từ optical flow.
 */

#include "ekf_velocity.h"
#include "ekf_attitude.h"

/* Một bộ lọc 2 trạng thái cho mỗi trục ngang: [vận tốc, bias gia tốc]. */
typedef struct {
    float x[2];
    float P[2][2];
} axis_kf_t;

static axis_kf_t s_n;           /* trục Bắc */
static axis_kf_t s_e;           /* trục Đông */

static vec3f_t  s_body_meas;    /* vận tốc thân đo được, để soi bằng mắt */

/* Số liệu thô của lần cập nhật gần nhất, phục vụ chẩn đoán phần bù quay. */
static float s_dbg_wx, s_dbg_wy, s_dbg_gx, s_dbg_gy;
static bool     s_valid;
static uint32_t s_accepted;
static uint32_t s_rejected;

/* ==========================================================================
 * Khởi tạo
 * ========================================================================== */

static void axis_init(axis_kf_t *a)
{
    a->x[0] = 0.0f;
    a->x[1] = 0.0f;
    a->P[0][0] = 1.0f;  a->P[0][1] = 0.0f;
    a->P[1][0] = 0.0f;  a->P[1][1] = 1.0f;
}

void ekf_velocity_init(void)
{
    axis_init(&s_n);
    axis_init(&s_e);

    s_body_meas = (vec3f_t){ 0.0f, 0.0f, 0.0f };
    s_valid     = false;
    s_accepted  = 0;
    s_rejected  = 0;
}

/* ==========================================================================
 * Dự báo
 * ========================================================================== */

static void axis_predict(axis_kf_t *a, float accel, float dt)
{
    const float acc = accel - a->x[1];

    a->x[0] += acc * dt;
    /* bias giữ nguyên: mô hình bước ngẫu nhiên */

    /*      [ 1  -dt ]
     *  F = [ 0   1  ]      cột thứ hai âm vì bias bị TRỪ khỏi gia tốc
     */
    float FP[2][2];
    for (int j = 0; j < 2; j++) {
        FP[0][j] = a->P[0][j] - dt * a->P[1][j];
        FP[1][j] =              a->P[1][j];
    }

    float Pn[2][2];
    for (int i = 0; i < 2; i++) {
        Pn[i][0] = FP[i][0] - dt * FP[i][1];
        Pn[i][1] =                 FP[i][1];
    }

    const float sa2 = EST_ACC_XY_NOISE_MPS2 * EST_ACC_XY_NOISE_MPS2;
    const float sb2 = EST_ACC_XY_BIAS_WALK * EST_ACC_XY_BIAS_WALK;

    Pn[0][0] += sa2 * dt * dt;
    Pn[1][1] += sb2 * dt;

    memcpy(a->P, Pn, sizeof(Pn));
}

void ekf_velocity_predict(float accel_n, float accel_e, float dt)
{
    if (dt <= 0.0f || dt > 0.1f) {
        return;
    }
    axis_predict(&s_n, accel_n, dt);
    axis_predict(&s_e, accel_e, dt);
}

/* ==========================================================================
 * Cập nhật vô hướng — đo trực tiếp trạng thái vận tốc, nên H = [1 0]
 * ========================================================================== */

static void axis_update(axis_kf_t *a, float z, float r)
{
    const float S = a->P[0][0] + r;
    if (S < 1e-9f) {
        return;
    }

    const float K[2] = { a->P[0][0] / S, a->P[1][0] / S };
    const float y    = z - a->x[0];

    a->x[0] += K[0] * y;
    a->x[1] += K[1] * y;

    /* Dạng Joseph, giữ P xác định dương qua nhiều giờ chạy. */
    const float IKH[2][2] = { { 1.0f - K[0], 0.0f },
                              {      -K[1], 1.0f } };

    float tmp[2][2], Pn[2][2];
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            tmp[i][j] = IKH[i][0] * a->P[0][j] + IKH[i][1] * a->P[1][j];
        }
    }
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            Pn[i][j] = tmp[i][0] * IKH[j][0] + tmp[i][1] * IKH[j][1]
                     + K[i] * r * K[j];
        }
    }
    memcpy(a->P, Pn, sizeof(Pn));
}

/* ==========================================================================
 * Cập nhật bằng optical flow
 * ========================================================================== */

bool ekf_velocity_update_flow(float flow_x_rad, float flow_y_rad,
                              vec3f_t gyro_dps, float dt,
                              float range_m, float tilt_cos,
                              uint8_t quality)
{
    if (dt <= 0.0f || dt > 0.5f) {
        s_rejected++;
        return false;
    }

    /* Chất lượng thấp nghĩa là mặt sàn không đủ kết cấu để bám. */
    if (quality < FLOW_QUALITY_MIN) {
        s_rejected++;
        return false;
    }

    /*
     * Nghiêng nhiều thì tia laser bắn xiên nên độ cao sai, và phép chiếu vận
     * tốc từ hệ thân sang NED cũng mất chính xác. Thà bỏ còn hơn tin số sai.
     */
    if (tilt_cos < cosf(EST_FLOW_MAX_TILT_DEG * FC_DEG_TO_RAD)) {
        s_rejected++;
        return false;
    }

    /* Độ cao thẳng đứng tới đúng mặt sàn mà flow đang nhìn. */
    const float h = range_m * tilt_cos;
    if (h < EST_FLOW_MIN_HEIGHT_M || h > EST_FLOW_MAX_HEIGHT_M) {
        s_rejected++;
        return false;
    }

    /*
     * --- BÙ QUAY ---
     * Trừ đi phần dịch ảnh do máy bay quay, chỉ giữ lại phần do tịnh tiến.
     * Xem phần dẫn giải trong ekf_velocity.h.
     */
    const float wx = flow_x_rad / dt;            /* rad/s đo được */
    const float wy = flow_y_rad / dt;

    const float gx = gyro_dps.x * FC_DEG_TO_RAD;
    const float gy = gyro_dps.y * FC_DEG_TO_RAD;

    /*
     * Dấu CỘNG, không phải trừ — chỗ này tôi từng làm sai và triệu chứng rất
     * dễ nhầm với "bù thiếu". Dẫn lại từ hình học:
     *
     *   Lăn phải (gyro_x > 0) làm điểm mặt đất dịch về +Y trong hệ thân.
     *   Bay phải (vy > 0)     làm điểm mặt đất lùi về -Y.
     *   Đo trên phần cứng: bay phải cho flow_x DƯƠNG, nên flow_x đo
     *   "điểm đi về -Y" là dương, tức đóng góp của lăn phải vào flow_x là ÂM:
     *
     *       flow_x = +vy/h - gyro_x      ->   vy =  h·(flow_x + gyro_x)
     *       flow_y = -vx/h - gyro_y      ->   vx = -h·(flow_y + gyro_y)
     *
     * Làm ngược dấu thì thay vì triệt tiêu, nó CỘNG THÊM phần quay — sai số
     * dư đúng bằng 2 lần tốc độ quay. Nghiêng tay ~110 °/s ở độ cao 0,8 m sẽ
     * cho vận tốc ảo ±3 m/s, và bộ giữ vị trí sẽ tự kích ngay lập tức.
     */
    const float vy_body =  h * (wx + gx);
    const float vx_body = -h * (wy + gy);

    s_body_meas = (vec3f_t){ vx_body, vy_body, 0.0f };
    s_dbg_wx = wx;  s_dbg_wy = wy;
    s_dbg_gx = gx;  s_dbg_gy = gy;

    /*
     * Xoay sang hệ NED. Thành phần thẳng đứng của vận tốc thân coi như 0 —
     * xấp xỉ này chỉ đúng khi gần thăng bằng, mà cổng nghiêng phía trên đã
     * bảo đảm điều đó.
     */
    const vec3f_t v_ned =
        ekf_attitude_body_to_ned((vec3f_t){ vx_body, vy_body, 0.0f });

    const float r = EST_FLOW_NOISE_MPS * EST_FLOW_NOISE_MPS;
    axis_update(&s_n, v_ned.x, r);
    axis_update(&s_e, v_ned.y, r);

    s_valid = true;
    s_accepted++;
    return true;
}

/* ==========================================================================
 * API đọc
 * ========================================================================== */

float   ekf_velocity_north(void)        { return s_n.x[0]; }
float   ekf_velocity_east(void)         { return s_e.x[0]; }
vec3f_t ekf_velocity_body_measured(void){ return s_body_meas; }
bool    ekf_velocity_is_valid(void)     { return s_valid; }
uint32_t ekf_velocity_accepted(void)    { return s_accepted; }
uint32_t ekf_velocity_rejected(void)    { return s_rejected; }

void ekf_velocity_debug_rates(float *wx, float *wy, float *gx, float *gy)
{
    *wx = s_dbg_wx;  *wy = s_dbg_wy;
    *gx = s_dbg_gx;  *gy = s_dbg_gy;
}

float ekf_velocity_uncertainty_mps(void)
{
    const float v = 0.5f * (s_n.P[0][0] + s_e.P[0][0]);
    return sqrtf(fmaxf(v, 0.0f));
}
