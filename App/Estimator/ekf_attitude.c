/**
 * @file    ekf_attitude.c
 * @brief   Hiện thực EKF dạng sai số cho ước lượng góc.
 */

#include "ekf_attitude.h"

#define N  EKF_ATT_STATES        /* 6 */

/* ==========================================================================
 * Trạng thái nội bộ
 * ========================================================================== */

static quatf_t s_q;              /* trạng thái danh nghĩa: thân -> NED     */
static vec3f_t s_bias;           /* bias gyro, rad/s                        */
static float   s_P[N][N];        /* hiệp phương sai của trạng thái sai số   */
static float   s_R[3][3];        /* ma trận xoay hiện tại, thân -> NED      */
static bool    s_valid;

/* ==========================================================================
 * Đại số ma trận cỡ nhỏ
 *
 * Kích thước cố định nên trình biên dịch mở được vòng lặp; cả bước chạy tốn
 * khoảng 1500 phép nhân, tức vài micro giây trên M7 480 MHz có FPU. Cố ý
 * viết dạng tổng quát thay vì khai thác cấu trúc thưa của F và H: nhanh hơn
 * chút nhưng rất dễ sai, mà đây là chỗ sai thì cả bộ lọc vô nghĩa.
 * ========================================================================== */

/** out = A * B */
static void m66_mul(const float A[N][N], const float B[N][N], float out[N][N])
{
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            float s = 0.0f;
            for (int k = 0; k < N; k++) {
                s += A[i][k] * B[k][j];
            }
            out[i][j] = s;
        }
    }
}

/** out = A * Bᵀ */
static void m66_mul_bt(const float A[N][N], const float B[N][N], float out[N][N])
{
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            float s = 0.0f;
            for (int k = 0; k < N; k++) {
                s += A[i][k] * B[j][k];
            }
            out[i][j] = s;
        }
    }
}

/** Ép P đối xứng trở lại. Sai số làm tròn tích luỹ sẽ phá tính đối xứng,
 *  và một P không đối xứng sớm muộn cho ra phương sai âm rồi vỡ bộ lọc. */
static void m66_symmetrize(float P[N][N])
{
    for (int i = 0; i < N; i++) {
        for (int j = i + 1; j < N; j++) {
            const float m = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = m;
            P[j][i] = m;
        }
    }
}

/** Nghịch đảo ma trận 3x3. false nếu suy biến. */
static bool m33_inv(const float m[3][3], float out[3][3])
{
    const float c00 =  (m[1][1] * m[2][2] - m[1][2] * m[2][1]);
    const float c01 = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]);
    const float c02 =  (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

    const float det = m[0][0] * c00 + m[0][1] * c01 + m[0][2] * c02;
    if (fabsf(det) < 1e-12f) {
        return false;
    }
    const float inv_det = 1.0f / det;

    out[0][0] = c00 * inv_det;
    out[1][0] = c01 * inv_det;
    out[2][0] = c02 * inv_det;

    out[0][1] = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]) * inv_det;
    out[1][1] =  (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_det;
    out[2][1] = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]) * inv_det;

    out[0][2] =  (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_det;
    out[1][2] = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]) * inv_det;
    out[2][2] =  (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_det;

    return true;
}

/* ==========================================================================
 * Quaternion
 * ========================================================================== */

static quatf_t quat_normalize(quatf_t q)
{
    const float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n < 1e-9f) {
        return (quatf_t){ 1.0f, 0.0f, 0.0f, 0.0f };
    }
    const float inv = 1.0f / n;
    return (quatf_t){ q.w * inv, q.x * inv, q.y * inv, q.z * inv };
}

/** a ⊗ b (Hamilton). */
static quatf_t quat_mul(quatf_t a, quatf_t b)
{
    return (quatf_t){
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    };
}

/** Dựng ma trận xoay thân -> NED từ quaternion. */
static void quat_to_matrix(quatf_t q, float R[3][3])
{
    const float w = q.w, x = q.x, y = q.y, z = q.z;

    R[0][0] = 1.0f - 2.0f * (y * y + z * z);
    R[0][1] =        2.0f * (x * y - w * z);
    R[0][2] =        2.0f * (x * z + w * y);

    R[1][0] =        2.0f * (x * y + w * z);
    R[1][1] = 1.0f - 2.0f * (x * x + z * z);
    R[1][2] =        2.0f * (y * z - w * x);

    R[2][0] =        2.0f * (x * z - w * y);
    R[2][1] =        2.0f * (y * z + w * x);
    R[2][2] = 1.0f - 2.0f * (x * x + y * y);
}

/* ==========================================================================
 * Khởi tạo
 * ========================================================================== */

void ekf_attitude_init(void)
{
    s_q     = (quatf_t){ 1.0f, 0.0f, 0.0f, 0.0f };
    s_bias  = (vec3f_t){ 0.0f, 0.0f, 0.0f };
    s_valid = false;

    memset(s_P, 0, sizeof(s_P));

    /*
     * Phương sai ban đầu. Góc: coi như chưa biết gì (1 rad ≈ 57°) để bộ lọc
     * kéo nhanh về hướng accel chỉ trong vài chục mẫu đầu. Bias: hẹp hơn
     * nhiều, vì driver IMU đã hiệu chuẩn bias tĩnh rồi — phần còn lại chỉ là
     * trôi theo nhiệt.
     */
    const float bias0 = (2.0f * FC_DEG_TO_RAD) * (2.0f * FC_DEG_TO_RAD);
    for (int i = 0; i < 3; i++) {
        s_P[i][i]         = 1.0f;
        s_P[i + 3][i + 3] = bias0;
    }

    quat_to_matrix(s_q, s_R);
}

/** Dựng quaternion ban đầu từ hướng trọng lực. Yaw đặt bằng 0 vì không có
 *  cảm biến nào biết hướng mũi máy bay. */
static void init_from_accel(vec3f_t a)
{
    const float norm = vec3f_norm(a);
    if (norm < 1.0f) {
        return;                     /* rơi tự do hoặc số rác */
    }

    const float ax = a.x / norm;
    const float ay = a.y / norm;
    const float az = a.z / norm;

    /* R[2][0] = -sin(pitch), R[2][1] = cos(pitch)sin(roll), R[2][2] = cos(pitch)cos(roll) */
    const float pitch = asinf(fc_constrainf(-ax, -1.0f, 1.0f));
    const float roll  = atan2f(ay, az);

    const float cr = cosf(roll * 0.5f),  sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);

    /* yaw = 0 nên các số hạng chứa yaw rút gọn hết. */
    s_q = quat_normalize((quatf_t){ cr * cp, sr * cp, cr * sp, -sr * sp });
    quat_to_matrix(s_q, s_R);
    s_valid = true;
}

/* ==========================================================================
 * Dự báo
 * ========================================================================== */

static void predict(vec3f_t gyro_rad, float dt)
{
    /* --- Tích phân quaternion danh nghĩa --- */
    const float wx = gyro_rad.x, wy = gyro_rad.y, wz = gyro_rad.z;
    const float angle = sqrtf(wx * wx + wy * wy + wz * wz) * dt;

    quatf_t dq;
    if (angle > 1e-7f) {
        const float s = sinf(angle * 0.5f) / (angle / dt);
        dq = (quatf_t){ cosf(angle * 0.5f), wx * s, wy * s, wz * s };
    } else {
        /* Góc quá nhỏ: xấp xỉ bậc nhất, tránh chia cho số gần 0. */
        dq = (quatf_t){ 1.0f, wx * dt * 0.5f, wy * dt * 0.5f, wz * dt * 0.5f };
    }

    s_q = quat_normalize(quat_mul(s_q, dq));
    quat_to_matrix(s_q, s_R);

    /*
     * --- Truyền hiệp phương sai ---
     * Động học sai số:  δθ̇ = -[ω]× δθ - b_g,   ḃ_g = 0
     * Rời rạc hoá bậc nhất:
     *     F = [ I - [ω]× dt   -I dt ]
     *         [      0          I   ]
     */
    float F[N][N];
    memset(F, 0, sizeof(F));
    for (int i = 0; i < N; i++) {
        F[i][i] = 1.0f;
    }

    F[0][1] =  wz * dt;  F[0][2] = -wy * dt;
    F[1][0] = -wz * dt;  F[1][2] =  wx * dt;
    F[2][0] =  wy * dt;  F[2][1] = -wx * dt;

    F[0][3] = -dt;  F[1][4] = -dt;  F[2][5] = -dt;

    float tmp[N][N];
    float Pn[N][N];
    m66_mul(F, s_P, tmp);
    m66_mul_bt(tmp, F, Pn);

    /* Nhiễu quá trình: gyro bơm vào góc, bước ngẫu nhiên bơm vào bias. */
    const float gyro_sd  = EST_GYRO_NOISE_DPS * FC_DEG_TO_RAD;
    const float bias_sd  = EST_GYRO_BIAS_WALK_DPS * FC_DEG_TO_RAD;
    const float q_theta  = gyro_sd * gyro_sd * dt;
    const float q_bias   = bias_sd * bias_sd * dt;

    for (int i = 0; i < 3; i++) {
        Pn[i][i]         += q_theta;
        Pn[i + 3][i + 3] += q_bias;
    }

    memcpy(s_P, Pn, sizeof(s_P));
    m66_symmetrize(s_P);
}

/* ==========================================================================
 * Cập nhật bằng accel
 * ========================================================================== */

static void update_accel(vec3f_t a)
{
    const float g = FC_GRAVITY_MPS2;

    /*
     * Trọng lực do quaternion dự đoán, biểu diễn trong hệ thân:
     *     g_body = Rᵀ · (0,0,g) = g · (R[2][0], R[2][1], R[2][2])
     * Lúc nằm phẳng ra đúng (0, 0, +g) — khớp quy ước của fc_config.h.
     */
    const vec3f_t gb = {
        g * s_R[2][0],
        g * s_R[2][1],
        g * s_R[2][2]
    };

    /*
     * Độ tin cậy thích nghi. Accel chỉ chỉ đúng hướng trọng lực khi máy bay
     * không tăng tốc; |a| lệch khỏi g bao nhiêu thì phần lệch đó chắc chắn là
     * gia tốc chuyển động, không phải trọng lực. Nới R theo bình phương độ
     * lệch — nới liên tục chứ không cắt cứng, vì cắt cứng làm bộ lọc giật mỗi
     * lần đi qua ngưỡng.
     */
    const float dev   = fabsf(vec3f_norm(a) - g);
    const float scale = 1.0f + (dev / EST_ACCEL_REJECT_MPS2)
                             * (dev / EST_ACCEL_REJECT_MPS2);
    const float r     = EST_ACCEL_NOISE_MPS2 * EST_ACCEL_NOISE_MPS2 * scale;

    /*
     * H = ∂h/∂δθ = [gb]×  (ba cột đầu), phần bias bằng 0.
     * Ma trận này hạng 2: nhân không gian nằm dọc gb, tức thành phần yaw.
     * Đó chính là lý do accel không sửa được hướng mũi máy bay.
     */
    float H[3][N];
    memset(H, 0, sizeof(H));
    H[0][1] = -gb.z;  H[0][2] =  gb.y;
    H[1][0] =  gb.z;  H[1][2] = -gb.x;
    H[2][0] = -gb.y;  H[2][1] =  gb.x;

    /* PHt = P Hᵀ  (6x3) */
    float PHt[N][3];
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < N; k++) {
                s += s_P[i][k] * H[j][k];
            }
            PHt[i][j] = s;
        }
    }

    /* S = H PHt + R  (3x3) */
    float S[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < N; k++) {
                s += H[i][k] * PHt[k][j];
            }
            S[i][j] = s + ((i == j) ? r : 0.0f);
        }
    }

    float Sinv[3][3];
    if (!m33_inv(S, Sinv)) {
        return;                     /* bỏ qua lượt này, đừng làm hỏng P */
    }

    /* K = PHt S⁻¹  (6x3) */
    float K[N][3];
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) {
                s += PHt[i][k] * Sinv[k][j];
            }
            K[i][j] = s;
        }
    }

    /* Sai lệch giữa đo và dự đoán. */
    const float y[3] = { a.x - gb.x, a.y - gb.y, a.z - gb.z };

    float dx[N];
    for (int i = 0; i < N; i++) {
        dx[i] = K[i][0] * y[0] + K[i][1] * y[1] + K[i][2] * y[2];
    }

    /*
     * --- Cập nhật P theo dạng Joseph ---
     *     P = (I - KH) P (I - KH)ᵀ + K R Kᵀ
     * Dạng rút gọn P = (I-KH)P rẻ hơn nhưng chỉ đúng khi K là nghiệm tối ưu
     * chính xác; sai số làm tròn tích luỹ có thể đẩy P mất tính xác định
     * dương. Dạng Joseph giữ P đối xứng nửa xác định dương ngay cả khi K
     * không hoàn hảo — đáng vài trăm phép nhân cho một bộ lọc phải chạy hàng
     * giờ không được vỡ.
     */
    float IKH[N][N];
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) {
                s += K[i][k] * H[k][j];
            }
            IKH[i][j] = ((i == j) ? 1.0f : 0.0f) - s;
        }
    }

    float tmp[N][N];
    float Pn[N][N];
    m66_mul(IKH, s_P, tmp);
    m66_mul_bt(tmp, IKH, Pn);

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            Pn[i][j] += r * (K[i][0] * K[j][0] + K[i][1] * K[j][1] + K[i][2] * K[j][2]);
        }
    }
    memcpy(s_P, Pn, sizeof(s_P));
    m66_symmetrize(s_P);

    /*
     * --- Tiêm sai số vào trạng thái danh nghĩa rồi xoá ---
     * δq ≈ (1, δθ/2) với góc nhỏ. Nhân bên PHẢI vì δθ định nghĩa trong hệ thân.
     */
    const quatf_t dq = quat_normalize(
        (quatf_t){ 1.0f, 0.5f * dx[0], 0.5f * dx[1], 0.5f * dx[2] });

    s_q = quat_normalize(quat_mul(s_q, dq));
    quat_to_matrix(s_q, s_R);

    s_bias.x += dx[3];
    s_bias.y += dx[4];
    s_bias.z += dx[5];

    /*
     * Chặn bias. Bias trục Z không quan sát được nên nếu để tự do nó sẽ trôi
     * theo nhiễu cho tới khi thành số vô lý và kéo cả ước lượng góc đi theo.
     */
    const float lim = EST_GYRO_BIAS_MAX_DPS * FC_DEG_TO_RAD;
    s_bias.x = fc_constrainf(s_bias.x, -lim, lim);
    s_bias.y = fc_constrainf(s_bias.y, -lim, lim);
    s_bias.z = fc_constrainf(s_bias.z, -lim, lim);

    /* Chặn luôn phương sai của bias yaw, vì nó chỉ tăng chứ không bao giờ giảm. */
    if (s_P[5][5] > lim * lim) {
        s_P[5][5] = lim * lim;
    }
}

/* ==========================================================================
 * API
 * ========================================================================== */

void ekf_attitude_update(vec3f_t gyro_dps, vec3f_t accel_mps2, float dt_s)
{
    if (dt_s <= 0.0f || dt_s > 0.1f) {
        return;                     /* nhịp bất thường, bỏ qua */
    }

    /* Quy ước accel của mạch, xem EST_ACCEL_Z_SIGN trong fc_config.h. */
    const vec3f_t a = vec3f_scale(accel_mps2, (float)EST_ACCEL_Z_SIGN);

    if (!s_valid) {
        init_from_accel(a);
        return;
    }

    const vec3f_t gyro_rad = {
        gyro_dps.x * FC_DEG_TO_RAD - s_bias.x,
        gyro_dps.y * FC_DEG_TO_RAD - s_bias.y,
        gyro_dps.z * FC_DEG_TO_RAD - s_bias.z
    };

    predict(gyro_rad, dt_s);

    /* Rơi tự do hoặc va đập mạnh: accel không mang thông tin trọng lực nào. */
    const float norm = vec3f_norm(a);
    if (norm > 1.0f && norm < 30.0f) {
        update_accel(a);
    }
}

quatf_t ekf_attitude_quaternion(void) { return s_q; }
bool    ekf_attitude_is_valid(void)   { return s_valid; }
float   ekf_attitude_tilt_cos(void)   { return s_R[2][2]; }

euler_t ekf_attitude_euler(void)
{
    euler_t e;
    e.roll  = atan2f(s_R[2][1], s_R[2][2]);
    e.pitch = asinf(fc_constrainf(-s_R[2][0], -1.0f, 1.0f));
    e.yaw   = atan2f(s_R[1][0], s_R[0][0]);
    return e;
}

vec3f_t ekf_attitude_gyro_bias_dps(void)
{
    return vec3f_scale(s_bias, FC_RAD_TO_DEG);
}

vec3f_t ekf_attitude_body_to_ned(vec3f_t v)
{
    return (vec3f_t){
        s_R[0][0] * v.x + s_R[0][1] * v.y + s_R[0][2] * v.z,
        s_R[1][0] * v.x + s_R[1][1] * v.y + s_R[1][2] * v.z,
        s_R[2][0] * v.x + s_R[2][1] * v.y + s_R[2][2] * v.z
    };
}

float ekf_attitude_uncertainty_deg(void)
{
    /* Chỉ lấy hai trục roll/pitch: yaw không quan sát được nên phương sai của
     * nó luôn lớn và sẽ che mất thông tin hữu ích nếu gộp vào. */
    const float var = 0.5f * (s_P[0][0] + s_P[1][1]);
    return sqrtf(fmaxf(var, 0.0f)) * FC_RAD_TO_DEG;
}
