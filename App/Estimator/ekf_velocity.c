/**
 * @file    ekf_velocity.c
 * @brief   Hiện thực ước lượng vận tốc ngang từ optical flow.
 */

#include "ekf_velocity.h"
#include "param_table.h"
#include "ekf_attitude.h"
#include "fc_time.h"

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

/*
 * Moc thoi gian cua lan CHAP NHAN mau flow gan nhat.
 *
 * VI SAO CAN: truoc day s_valid bat mot lan roi KHONG BAO GIO tat. Chi co
 * ekf_velocity_init() dat no ve false. Hau qua day chuyen, da do duoc that:
 *
 *   1. Nghieng qua est_flow_max_tilt_deg (20 do) -> cong loc chan mau flow
 *   2. Bo loc mat phep do, chi con SUY TINH tu gia toc ke. O 15 do nghieng,
 *      gia toc ngang ~2,6 m/s^2 nen chi nua giay da tich luy 1,3 m/s sai so
 *   3. ekf_velocity_is_valid() VAN tra ve true
 *   4. ctrl_poshold tin vao con so do va nghieng may bay theo no
 *   5. Nghieng nhieu hon -> chan nhieu hon -> vong phan hoi DUONG
 *
 *   Nguoi dung bao: POSHOLD lac theo vong tron voi ban kinh LON DAN. Buoc 5
 *   chinh la cai lam ban kinh lon dan.
 *
 *   Va duong lui ve ANGLE ma ctrl_poshold tu nhan la co - "mat flow thi tra
 *   false" - CHUA BAO GIO chay duoc, vi dieu kien cua no la !is_valid().
 */
static uint32_t s_last_ok_us;
static bool     s_have_ok;

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
    s_valid      = false;
    s_accepted   = 0;
    s_rejected   = 0;
    s_last_ok_us = 0;
    s_have_ok    = false;
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

    const float sa2 = g_params.est_acc_xy_noise_mps2 * g_params.est_acc_xy_noise_mps2;
    const float sb2 = g_params.est_acc_xy_bias_walk * g_params.est_acc_xy_bias_walk;

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
    if (quality < g_params.flow_quality_min) {
        s_rejected++;
        return false;
    }

    /*
     * Nghiêng nhiều thì tia laser bắn xiên nên độ cao sai, và phép chiếu vận
     * tốc từ hệ thân sang NED cũng mất chính xác. Thà bỏ còn hơn tin số sai.
     */
    if (tilt_cos < cosf(g_params.est_flow_max_tilt_deg * FC_DEG_TO_RAD)) {
        s_rejected++;
        return false;
    }

    /* Độ cao thẳng đứng tới đúng mặt sàn mà flow đang nhìn. */
    const float h = range_m * tilt_cos;
    if (h < g_params.est_flow_min_height_m || h > g_params.est_flow_max_height_m) {
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
    /*
     * --- BÙ CÁNH TAY ĐÒN ---
     * Số trên là vận tốc của ĐIỂM ĐẶT cảm biến. Trừ ω × r để ra vận tốc của
     * tâm máy bay (r = (FLOW_OFFSET_X_M, 0, FLOW_OFFSET_Z_M), r_y = 0):
     *     (ω × r)_x = gy·r_z
     *     (ω × r)_y = gz·r_x − gx·r_z
     * Kiểm dấu: ngóc mũi (gy > 0) thì điểm dưới tâm đi TỚI; mũi quay phải
     * (gz > 0) thì đuôi quét sang TRÁI.
     */
    const float gz = gyro_dps.z * FC_DEG_TO_RAD;

    const float vy_body =  h * (wx + gx) - (gz * FLOW_OFFSET_X_M - gx * FLOW_OFFSET_Z_M);
    const float vx_body = -h * (wy + gy) - gy * FLOW_OFFSET_Z_M;

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

    const float r = g_params.est_flow_noise_mps * g_params.est_flow_noise_mps;
    axis_update(&s_n, v_ned.x, r);
    axis_update(&s_e, v_ned.y, r);

    /*
     * Flow QUAY LAI sau mot khoang mat: nap lai bo loc thay vi hoa tron mau
     * moi voi mot trang thai da troi.
     *
     * Trong khoang mat, bo loc chi tich phan gia toc ke nen van toc uoc luong
     * co the da lech vai m/s. Hoa mau tot voi trang thai rac chi lam ban mau
     * tot; vut trang thai cu di roi bat dau lai tu phep do moi thi dung hon.
     *
     * axis_init() dat van toc ve 0 va phuong sai ve lon, nen phep cap nhat
     * ngay sau day se keo thang toi gia tri do duoc.
     */
    if (s_have_ok &&
        fc_elapsed_us(micros(), s_last_ok_us) >
            (uint32_t)g_params.est_flow_timeout_ms * 1000u) {
        axis_init(&s_n);
        axis_init(&s_e);
    }

    s_valid      = true;
    s_last_ok_us = micros();
    s_have_ok    = true;
    s_accepted++;
    return true;
}

/* ==========================================================================
 * API đọc
 * ========================================================================== */

float   ekf_velocity_north(void)        { return s_n.x[0]; }
float   ekf_velocity_east(void)         { return s_e.x[0]; }
vec3f_t ekf_velocity_body_measured(void){ return s_body_meas; }
bool ekf_velocity_is_valid(void)
{
    if (!s_valid) {
        return false;
    }

    /*
     * HET HAN neu da lau khong chap nhan duoc mau flow nao.
     *
     * Khong co phep do thi bo loc chi con tich phan gia toc ke, va cai do
     * troi rat nhanh - xem giai thich dai o cho khai bao s_last_ok_us.
     *
     * Nguong dat theo est_flow_timeout_ms. Qua ngan thi co chop tat lien tuc
     * moi khi nghieng nhe; qua dai thi khong cat duoc vong phan hoi duong.
     */
    if (!s_have_ok) {
        return false;
    }

    const uint32_t age_us = fc_elapsed_us(micros(), s_last_ok_us);

    if (age_us > (uint32_t)g_params.est_flow_timeout_ms * 1000u) {
        return false;
    }
    return true;
}

/** Da bao lau ke tu mau flow duoc chap nhan gan nhat, mili giay. */
uint32_t ekf_velocity_age_ms(void)
{
    if (!s_have_ok) {
        return 0xFFFFFFFFu;
    }
    return fc_elapsed_us(micros(), s_last_ok_us) / 1000u;
}
uint32_t ekf_velocity_accepted(void)    { return s_accepted; }
uint32_t ekf_velocity_rejected(void)    { return s_rejected; }

void ekf_velocity_bias_ne(float *bn, float *be)
{
    *bn = s_n.x[1];
    *be = s_e.x[1];
}

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
