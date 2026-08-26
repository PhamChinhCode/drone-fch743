/**
 * @file    fc_types.h
 * @brief   Kiểu dữ liệu cơ sở và hàm tiện ích dùng chung.
 *
 * QUY ƯỚC ĐƠN VỊ (áp dụng cho toàn bộ struct nội bộ):
 *   - Tốc độ góc  : độ/giây   (dps)   — quen thuộc khi chỉnh PID
 *   - Góc         : radian            — hậu tố _rad; đổi sang độ khi hiển thị
 *   - Gia tốc     : m/s^2
 *   - Vận tốc     : m/s
 *   - Vị trí/cao  : mét
 *   - Áp suất     : Pascal
 *   - Nhiệt độ    : độ C
 *   - Điện áp     : Volt, Dòng: Ampe
 *   - Thời gian   : micro giây (uint32_t) hoặc mili giây (uint32_t)
 *
 * Tên trường LUÔN mang hậu tố đơn vị (vd: alt_m, rate_dps) để tránh nhầm lẫn.
 */
#ifndef FC_TYPES_H
#define FC_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/* ========================================================================== */
/*  Vector / Quaternion                                                       */
/* ========================================================================== */

typedef struct { float x, y, z; }      vec3f_t;
typedef struct { int16_t x, y, z; }    vec3i16_t;
typedef struct { int32_t x, y, z; }    vec3i32_t;
typedef struct { float w, x, y, z; }   quatf_t;

/** Góc Euler theo quy ước hàng không (ZYX): yaw -> pitch -> roll. */
typedef struct { float roll, pitch, yaw; } euler_t;

/** Chỉ số trục — dùng cho mảng PID, setpoint, rate... */
typedef enum {
    AXIS_ROLL  = 0,
    AXIS_PITCH = 1,
    AXIS_YAW   = 2,
    AXIS_COUNT = 3
} fc_axis_t;

/* ==========================================================================
 *  Bộ đệm DMA — BẮT BUỘC đọc trước khi viết driver
 *
 *  Trong STM32H743XX_FLASH.ld, `.bss` và `.data` được đặt ở DTCMRAM
 *  (0x20000000). DTCM chỉ nối trực tiếp với lõi Cortex-M7 — **DMA1 và DMA2
 *  KHÔNG truy cập được vùng này**. Biến toàn cục hay `static` thông thường,
 *  cũng như mọi mảng cục bộ (nằm trên stack), đều không dùng làm bộ đệm DMA
 *  được: transfer sẽ ra dữ liệu rác hoặc gây bus fault.
 *
 *  Mọi mảng làm nguồn / đích cho DMA1 hoặc DMA2 phải khai báo như sau:
 *
 *      FC_DMA_BUFFER static uint8_t rx_buf[256];
 *
 *  Macro đặt biến vào section `.dma_buffer` -> AXI SRAM (0x24000000), vùng
 *  DMA truy cập bình thường và luôn được cấp clock.
 *
 *  Hai lưu ý:
 *    - Section là NOLOAD nên biến KHÔNG được zero lúc khởi động. Hãy memset
 *      trong hàm init của driver nếu cần bắt đầu từ 0.
 *    - Không gán giá trị khởi tạo cho biến dùng macro này.
 *
 *  (SDMMC1 dùng IDMA riêng và QUADSPI dùng MDMA — hai khối này truy cập được
 *   DTCM, nên bộ đệm của chúng không bắt buộc dùng macro.)
 * ========================================================================== */

#define FC_DMA_BUFFER  __attribute__((section(".dma_buffer"), aligned(32)))

/* ========================================================================== */
/*  Hằng số toán học                                                          */
/* ========================================================================== */

#define FC_PI            3.14159265358979f
#define FC_DEG_TO_RAD    (FC_PI / 180.0f)
#define FC_RAD_TO_DEG    (180.0f / FC_PI)
#define FC_GRAVITY_MPS2  9.80665f

/* ========================================================================== */
/*  Hàm tiện ích                                                              */
/* ========================================================================== */

static inline float fc_constrainf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline int32_t fc_constrain_i32(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/** Ánh xạ tuyến tính v từ khoảng [in_lo,in_hi] sang [out_lo,out_hi]. */
static inline float fc_mapf(float v, float in_lo, float in_hi,
                            float out_lo, float out_hi)
{
    if (in_hi == in_lo) return out_lo;
    return out_lo + (v - in_lo) * (out_hi - out_lo) / (in_hi - in_lo);
}

/** Vùng chết đối xứng quanh 0; giá trị ngoài vùng chết được kéo giãn lại. */
static inline float fc_deadbandf(float v, float band)
{
    if (v > band)  return (v - band) / (1.0f - band);
    if (v < -band) return (v + band) / (1.0f - band);
    return 0.0f;
}

/** Bộ lọc thông thấp bậc 1: y += alpha * (x - y). */
static inline float fc_lpf(float y, float x, float alpha)
{
    return y + alpha * (x - y);
}

/** Hệ số alpha cho LPF theo tần số cắt và chu kỳ lấy mẫu. */
static inline float fc_lpf_alpha(float cutoff_hz, float dt_s)
{
    const float rc = 1.0f / (2.0f * FC_PI * cutoff_hz);
    return dt_s / (rc + dt_s);
}

/* --- Vector ------------------------------------------------------------- */

static inline vec3f_t vec3f_add(vec3f_t a, vec3f_t b)
{
    return (vec3f_t){ a.x + b.x, a.y + b.y, a.z + b.z };
}

static inline vec3f_t vec3f_sub(vec3f_t a, vec3f_t b)
{
    return (vec3f_t){ a.x - b.x, a.y - b.y, a.z - b.z };
}

static inline vec3f_t vec3f_scale(vec3f_t a, float s)
{
    return (vec3f_t){ a.x * s, a.y * s, a.z * s };
}

static inline float vec3f_norm(vec3f_t a)
{
    return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
}

/* --- Thời gian ---------------------------------------------------------- */
/*
 * Bộ đếm micro giây kiểu uint32_t tràn sau ~71,6 phút. Phép trừ unsigned
 * dưới đây vẫn cho kết quả đúng khi tràn, MIỄN LÀ khoảng cách < 71 phút.
 * Không bao giờ so sánh trực tiếp hai mốc thời gian bằng '<' hoặc '>'.
 */

static inline uint32_t fc_elapsed_us(uint32_t now_us, uint32_t past_us)
{
    return now_us - past_us;
}

static inline uint32_t fc_elapsed_ms(uint32_t now_ms, uint32_t past_ms)
{
    return now_ms - past_ms;
}

/** true nếu đã tới hạn (an toàn với tràn số). */
static inline bool fc_timeout_us(uint32_t now_us, uint32_t start_us, uint32_t period_us)
{
    return (now_us - start_us) >= period_us;
}

#endif /* FC_TYPES_H */
