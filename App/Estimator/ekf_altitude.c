/**
 * @file    ekf_altitude.c
 * @brief   Hiện thực EKF độ cao 3 trạng thái.
 */

#include "ekf_altitude.h"
#include "param_table.h"
#include "fc_time.h"

/* Chỉ số trạng thái, đặt tên cho dễ đọc phần đại số bên dưới. */
enum { ST_H = 0, ST_V = 1, ST_B = 2, ST_N = 3 };

static float s_x[ST_N];            /* [độ cao, tốc độ lên, bias accel] */
static float s_P[ST_N][ST_N];
static bool  s_valid;

static bool     s_range_tilt_blocked;   /* đang ở phía "quá nghiêng" của vòng trễ */
static bool     s_range_used_once;
static uint32_t s_range_last_used_us;   /* lần gần nhất laser được DÙNG (cập nhật/neo) */

static float    s_step_ref;             /* mức laser ứng viên, trung bình chạy */
static uint8_t  s_step_n;               /* số mẫu liên tiếp nằm quanh mức đó   */
static float    s_reset_sum_m;          /* tổng các lần dời độ cao do neo lại  */

/* ==========================================================================
 * Khởi tạo
 * ========================================================================== */

void ekf_altitude_init(void)
{
    memset(s_x, 0, sizeof(s_x));
    memset(s_P, 0, sizeof(s_P));

    /*
     * Bắt đầu ở mặt đất nên độ cao và tốc độ gần như chắc chắn bằng 0; bias
     * thì chưa biết gì, để rộng cho baro kéo về.
     */
    s_P[ST_H][ST_H] = 1.0f;
    s_P[ST_V][ST_V] = 1.0f;
    s_P[ST_B][ST_B] = 1.0f;

    s_valid = false;

    s_range_tilt_blocked = false;
    s_range_used_once    = false;
    s_range_last_used_us = 0;

    s_step_n      = 0;
    s_reset_sum_m = 0.0f;
}

/* ==========================================================================
 * Dự báo
 * ========================================================================== */

void ekf_altitude_predict(float accel_up_mps2, float dt)
{
    if (dt <= 0.0f || dt > 0.1f) {
        return;
    }

    const float a = accel_up_mps2 - s_x[ST_B];

    /* --- Trạng thái --- */
    s_x[ST_H] += s_x[ST_V] * dt + 0.5f * a * dt * dt;
    s_x[ST_V] += a * dt;
    /* bias giữ nguyên: mô hình bước ngẫu nhiên */

    /*
     * --- Hiệp phương sai ---
     *      [ 1  dt  -dt²/2 ]
     *  F = [ 0   1   -dt   ]
     *      [ 0   0    1    ]
     * Cột thứ ba mang dấu âm vì bias bị TRỪ khỏi gia tốc.
     */
    const float dt2 = dt * dt;
    const float F02 = -0.5f * dt2;
    const float F12 = -dt;

    float FP[ST_N][ST_N];
    for (int j = 0; j < ST_N; j++) {
        FP[0][j] = s_P[0][j] + dt * s_P[1][j] + F02 * s_P[2][j];
        FP[1][j] =             s_P[1][j] + F12 * s_P[2][j];
        FP[2][j] =                               s_P[2][j];
    }

    float Pn[ST_N][ST_N];
    for (int i = 0; i < ST_N; i++) {
        Pn[i][0] = FP[i][0] + dt * FP[i][1] + F02 * FP[i][2];
        Pn[i][1] =            FP[i][1] + F12 * FP[i][2];
        Pn[i][2] =                             FP[i][2];
    }

    /*
     * Nhiễu quá trình. Nhiễu gia tốc bơm vào cả h lẫn v và hai đại lượng này
     * tương quan với nhau (cùng một nguồn), nên các số hạng chéo Q[0][1]
     * KHÔNG được bỏ — bỏ đi thì bộ lọc tự tin quá mức vào độ cao.
     */
    const float sa2 = g_params.est_acc_z_noise_mps2 * g_params.est_acc_z_noise_mps2;
    const float sb2 = g_params.est_acc_z_bias_walk * g_params.est_acc_z_bias_walk;

    Pn[0][0] += sa2 * dt2 * dt2 * 0.25f;
    Pn[0][1] += sa2 * dt2 * dt * 0.5f;
    Pn[1][0] += sa2 * dt2 * dt * 0.5f;
    Pn[1][1] += sa2 * dt2;
    Pn[2][2] += sb2 * dt;

    memcpy(s_P, Pn, sizeof(s_P));
}

/* ==========================================================================
 * Cập nhật vô hướng
 *
 * Cả baro lẫn laser đều đo trực tiếp trạng thái h, tức H = [1 0 0]. Nhờ vậy
 * phép cập nhật rút gọn thành vài dòng, không cần nghịch đảo ma trận nào.
 * ========================================================================== */

static void update_height(float z, float r)
{
    const float S = s_P[ST_H][ST_H] + r;
    if (S < 1e-9f) {
        return;
    }

    const float K[ST_N] = {
        s_P[0][ST_H] / S,
        s_P[1][ST_H] / S,
        s_P[2][ST_H] / S
    };

    const float y = z - s_x[ST_H];

    s_x[0] += K[0] * y;
    s_x[1] += K[1] * y;
    s_x[2] += K[2] * y;

    /*
     * Dạng Joseph: P = (I-KH) P (I-KH)ᵀ + K R Kᵀ.
     * Với H = [1 0 0] thì (I-KH) chỉ khác đơn vị ở cột đầu.
     */
    float IKH[ST_N][ST_N];
    for (int i = 0; i < ST_N; i++) {
        for (int j = 0; j < ST_N; j++) {
            IKH[i][j] = ((i == j) ? 1.0f : 0.0f) - ((j == ST_H) ? K[i] : 0.0f);
        }
    }

    float tmp[ST_N][ST_N];
    for (int i = 0; i < ST_N; i++) {
        for (int j = 0; j < ST_N; j++) {
            float s = 0.0f;
            for (int k = 0; k < ST_N; k++) {
                s += IKH[i][k] * s_P[k][j];
            }
            tmp[i][j] = s;
        }
    }

    for (int i = 0; i < ST_N; i++) {
        for (int j = 0; j < ST_N; j++) {
            float s = 0.0f;
            for (int k = 0; k < ST_N; k++) {
                s += tmp[i][k] * IKH[j][k];
            }
            s_P[i][j] = s + K[i] * r * K[j];
        }
    }

    s_valid = true;
}

void ekf_altitude_update_baro(float altitude_m)
{
    update_height(altitude_m, g_params.est_baro_noise_m * g_params.est_baro_noise_m);
}

bool ekf_altitude_update_range(float range_m, float tilt_cos)
{
    /*
     * Nghiêng nhiều thì tia laser bắn xiên, chạm vào chỗ xa hơn hẳn điểm
     * ngay dưới bụng — chiếu hình học không cứu được vì mặt đất bên đó có
     * thể cao thấp khác. Thà bỏ hẳn số đo còn hơn tin một con số sai.
     */
    const float max_tilt_deg = s_range_tilt_blocked
                             ? g_params.est_range_max_tilt_deg - EST_RANGE_TILT_HYST_DEG
                             : g_params.est_range_max_tilt_deg;
    s_range_tilt_blocked = tilt_cos < cosf(max_tilt_deg * FC_DEG_TO_RAD);
    if (s_range_tilt_blocked) {
        return false;
    }

    if (range_m < EST_RANGE_MIN_M || range_m > g_params.est_range_max_m) {
        return false;
    }

    /* Chiếu khoảng cách nghiêng xuống phương thẳng đứng. */
    const float height = range_m * tilt_cos;
    const float r      = g_params.est_range_noise_m * g_params.est_range_noise_m;
    const uint32_t now_us = micros();

    /*
     * Laser vừa quay lại sau một quãng không dùng được: trong quãng đó độ cao
     * chỉ bám baro và đã có thể lệch cả mét. Cập nhật Kalman thường sẽ kéo cả
     * tốc độ lên theo cú nhảy (K[ST_V] lớn vì P đã nở) và vòng giữ độ cao giật
     * theo. Neo thẳng độ cao về laser.
     *
     * v CŨNG PHẢI coi là KHÔNG BIẾT: quãng mất laser chính là lúc v trôi (đo 09-18
     * khi cầm tay nghiêng > 25 độ: v tới -3 m/s). Giữ v cũ với P_vv nhỏ và xoá
     * P_hv thì K[ST_V] ~ 0 - laser chỉ kéo h, không sửa được v; v sai làm h trôi,
     * cổng phần dư chặn, 0,5 s sau lại neo h... vòng lặp khoá v ở -10 m/s ngay cả
     * khi nằm yên trên bàn. Đặt v = 0 với P_vv lớn để vài mẫu laser kế tiếp dựng
     * lại v, và nới P_bb vì bias cũng có thể đã bị kéo lệch trong quãng đó.
     */
    const bool lost = !s_range_used_once ||
        fc_elapsed_us(now_us, s_range_last_used_us) > EST_RANGE_REANCHOR_MS * 1000u;

    /*
     * Cổng phần dư: lệch quá EST_RANGE_GATE_SIGMA sigma HOẶC quá EST_RANGE_STEP_M
     * mét thì không cập nhật. Ngưỡng mét là bắt buộc: ngay sau khi dựng lại, P
     * nở nhanh và một mẫu lệch cả nửa mét vẫn lọt cổng sigma với K lớn.
     */
    const float y = height - s_x[ST_H];
    const float S = s_P[ST_H][ST_H] + r;
    const bool jump = (fabsf(y) > EST_RANGE_STEP_M) ||
                      (y * y > EST_RANGE_GATE_SIGMA * EST_RANGE_GATE_SIGMA * S);

    if (!lost && !jump) {
        s_step_n = 0;
        update_height(height, r);
        s_range_last_used_us = now_us;
        return true;
    }

    /*
     * Ứng viên mức mới (bậc địa hình, laser vừa quay lại, hoặc gai). Chỉ tin khi
     * laser ỔN ĐỊNH ở đó EST_RANGE_STEP_N mẫu liền — gai lẻ bị bỏ, và kéo dài
     * thì không khoá laser vĩnh viễn.
     */
    if (s_step_n == 0 || fabsf(height - s_step_ref) > EST_RANGE_STEP_TOL_M) {
        s_step_ref = height;
        s_step_n   = 1;
        return false;
    }
    s_step_n++;
    s_step_ref += (height - s_step_ref) / (float)s_step_n;
    if (s_step_n < EST_RANGE_STEP_N) {
        return false;
    }

    /*
     * NEO LẠI. Δh công bố ra ngoài để vòng giữ độ cao dời mốc theo: với nó đây
     * là đổi GỐC đo, không phải máy bay vừa lên/xuống.
     */
    s_reset_sum_m += s_step_ref - s_x[ST_H];
    s_x[ST_H]      = s_step_ref;

    if (lost) {
        /*
         * Mất laser lâu: v CŨNG PHẢI coi là KHÔNG BIẾT. Quãng mất laser chính là
         * lúc v trôi (đo 09-18 khi cầm tay nghiêng > 25 độ: v tới -3 m/s). Giữ v
         * cũ với P_vv nhỏ và xoá P_hv thì laser không sửa được v; v sai làm h
         * trôi, cổng chặn, lại neo... vòng lặp khoá v ở -10 m/s ngay cả khi nằm
         * yên trên bàn. Đặt v = 0 với P_vv lớn để laser dựng lại v, và nới P_bb.
         */
        s_x[ST_V] = 0.0f;
        for (int i = 0; i < ST_N; i++) {
            s_P[ST_H][i] = s_P[i][ST_H] = 0.0f;
            s_P[ST_V][i] = s_P[i][ST_V] = 0.0f;
        }
        s_P[ST_H][ST_H] = r;
        s_P[ST_V][ST_V] = EST_RANGE_REANCHOR_VEL_VAR;
        s_P[ST_B][ST_B] = fmaxf(s_P[ST_B][ST_B], EST_RANGE_REANCHOR_BIAS_VAR);
    }
    /* Bậc địa hình khi laser vẫn chạy: chỉ dời h, v và P vẫn đúng nên giữ nguyên. */

    s_step_n             = 0;
    s_valid              = true;
    s_range_used_once    = true;
    s_range_last_used_us = now_us;
    return true;
}

/* ==========================================================================
 * API đọc
 * ========================================================================== */

float ekf_altitude_m(void)              { return s_x[ST_H]; }
float ekf_altitude_climb_rate_mps(void) { return s_x[ST_V]; }
float ekf_altitude_accel_bias(void)     { return s_x[ST_B]; }
bool  ekf_altitude_is_valid(void)       { return s_valid; }
float ekf_altitude_reset_sum_m(void)    { return s_reset_sum_m; }

bool ekf_altitude_range_recent(uint32_t max_ms)
{
    return s_range_used_once &&
           fc_elapsed_us(micros(), s_range_last_used_us) <= max_ms * 1000u;
}

float ekf_altitude_uncertainty_m(void)
{
    return sqrtf(fmaxf(s_P[ST_H][ST_H], 0.0f));
}

float ekf_altitude_climb_uncertainty_mps(void)
{
    return sqrtf(fmaxf(s_P[ST_V][ST_V], 0.0f));
}
