/**
 * @file    imu_noise.h
 * @brief   Đo độ lệch chuẩn của gyro theo cửa sổ trượt, chạy ngay trong ngắt.
 *
 * VÌ SAO CẦN:
 *   Console chạy 50 Hz nên không thể đo nhiễu của tín hiệu 2-8 kHz — nó chỉ
 *   nhìn thấy giá trị tại đúng thời điểm in. Muốn biết nhiễu thật thì phải
 *   cộng dồn NGAY TRONG ISR, ở tốc độ đầy đủ.
 *
 *   Đây là công cụ chính của giai đoạn 2 trong App/Docs/KE_HOACH_LSM6DSV.md:
 *   so nền nhiễu của ICM20602 và LSM6DSV để quyết định có đáng hợp nhất hai
 *   IMU hay không.
 *
 * CHI PHÍ:
 *   3 phép nhân + 6 phép cộng mỗi mẫu. Trên M7 có FPU đơn chính xác là vài
 *   chục chu kỳ, tức khoảng 0,03% ngân sách 125 µs của vòng 8 kHz.
 *
 * GIỚI HẠN CẦN BIẾT:
 *   Dùng công thức sigma² = E[x²] - E[x]². Công thức này mất chính xác khi
 *   giá trị trung bình LỚN so với độ lệch — hai số lớn gần bằng nhau trừ đi
 *   nhau. Ở đây gyro đã trừ bias nên trung bình ≈ 0 lúc đứng yên, và đó đúng
 *   là lúc ta quan tâm.
 *
 *   Hệ quả: số đọc CHỈ có nghĩa khi máy bay đứng yên hoặc giữ ga ổn định.
 *   Đang xoay máy bay thì con số này vô nghĩa, đừng đọc.
 */
#ifndef IMU_NOISE_H
#define IMU_NOISE_H

#include "fc_types.h"

/** Cửa sổ tính, tính bằng micro giây. Một giây là đủ mượt mà vẫn kịp phản ứng. */
#define IMU_NOISE_WINDOW_US 1000000u

typedef struct {
    float    sum[AXIS_COUNT];
    float    sumsq[AXIS_COUNT];
    uint32_t count;

    float    sigma[AXIS_COUNT];  /**< kết quả của cửa sổ VỪA XONG      */
    uint32_t samples;            /**< số mẫu trong cửa sổ vừa xong     */
    uint32_t mark_us;
} imu_noise_t;

static inline void imu_noise_reset(imu_noise_t *n, uint32_t now_us)
{
    for (int i = 0; i < AXIS_COUNT; i++) {
        n->sum[i]   = 0.0f;
        n->sumsq[i] = 0.0f;
        n->sigma[i] = 0.0f;
    }
    n->count   = 0;
    n->samples = 0;
    n->mark_us = now_us;
}

/**
 * Nạp một mẫu gyro. Gọi từ ISR, sau khi đã trừ bias và xoay trục.
 *
 * @param g       tốc độ góc theo hệ trục THÂN, đơn vị °/s
 * @param now_us  mốc thời gian của mẫu
 */
static inline void imu_noise_feed(imu_noise_t *n, vec3f_t g, uint32_t now_us)
{
    const float v[AXIS_COUNT] = { g.x, g.y, g.z };

    for (int i = 0; i < AXIS_COUNT; i++) {
        n->sum[i]   += v[i];
        n->sumsq[i] += v[i] * v[i];
    }
    n->count++;

    if (fc_elapsed_us(now_us, n->mark_us) < IMU_NOISE_WINDOW_US) {
        return;
    }

    /* --- Hết cửa sổ: chốt kết quả rồi bắt đầu cửa sổ mới --- */
    if (n->count > 1u) {
        const float inv = 1.0f / (float)n->count;

        for (int i = 0; i < AXIS_COUNT; i++) {
            const float mean = n->sum[i] * inv;
            const float var  = n->sumsq[i] * inv - mean * mean;

            /* Sai số làm tròn có thể đẩy phương sai xuống âm một chút. */
            n->sigma[i] = (var > 0.0f) ? sqrtf(var) : 0.0f;
        }
        n->samples = n->count;
    }

    for (int i = 0; i < AXIS_COUNT; i++) {
        n->sum[i]   = 0.0f;
        n->sumsq[i] = 0.0f;
    }
    n->count   = 0;
    n->mark_us = now_us;
}

/** Độ lệch chuẩn lớn nhất trong ba trục — một con số để so sánh nhanh. */
static inline float imu_noise_sigma_max(const imu_noise_t *n)
{
    float m = n->sigma[0];

    if (n->sigma[1] > m) { m = n->sigma[1]; }
    if (n->sigma[2] > m) { m = n->sigma[2]; }
    return m;
}

#endif /* IMU_NOISE_H */
