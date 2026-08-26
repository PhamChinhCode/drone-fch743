/**
 * @file    ctrl_rate.c
 * @brief   Hiện thực vòng PID tốc độ góc.
 */

#include "ctrl_rate.h"
#include "mixer.h"
#include "fc_state.h"
#include "fc_time.h"
#include "stm32h7xx.h"

/*
 * Chia nhịp từ tốc độ mẫu IMU xuống tốc độ vòng điều khiển.
 * 8000 / 4000 = 2, tức lấy một mẫu bỏ một mẫu.
 */
#define RATE_LOOP_DIVIDER (IMU_SAMPLE_RATE_HZ / FC_LOOP_RATE_HZ)

#if RATE_LOOP_DIVIDER < 1
  #error "FC_LOOP_RATE_HZ khong duoc lon hon IMU_SAMPLE_RATE_HZ"
#endif

static uint32_t s_last_sample;      /* sample_count của lần chạy trước    */
static uint32_t s_last_us;
static uint32_t s_loops;
static uint32_t s_skips;
static uint32_t s_hz;

static float s_dterm_alpha;
static float s_dterm[AXIS_COUNT];   /* đạo hàm đã lọc                     */
static bool  s_dterm_primed;        /* đã có mẫu gyro đầu tiên chưa       */

/* Đo tần số thực tế của vòng, mỗi giây một lần. */
static uint32_t s_hz_mark_us;
static uint32_t s_hz_count;

/* ==========================================================================
 * Khởi tạo
 * ========================================================================== */

static void load_gains(void)
{
    pid_gains_t *g;

    g = &g_fc.ctrl.rate_pid[AXIS_ROLL].gains;
    g->kp = RATE_PID_ROLL_KP;
    g->ki = RATE_PID_ROLL_KI;
    g->kd = RATE_PID_ROLL_KD;
    g->kff = 0.0f;
    g->i_limit   = RATE_PID_I_LIMIT;
    g->out_limit = RATE_PID_OUT_LIMIT;

    g = &g_fc.ctrl.rate_pid[AXIS_PITCH].gains;
    g->kp = RATE_PID_PITCH_KP;
    g->ki = RATE_PID_PITCH_KI;
    g->kd = RATE_PID_PITCH_KD;
    g->kff = 0.0f;
    g->i_limit   = RATE_PID_I_LIMIT;
    g->out_limit = RATE_PID_OUT_LIMIT;

    g = &g_fc.ctrl.rate_pid[AXIS_YAW].gains;
    g->kp = RATE_PID_YAW_KP;
    g->ki = RATE_PID_YAW_KI;
    g->kd = RATE_PID_YAW_KD;
    g->kff = 0.0f;
    g->i_limit   = RATE_PID_I_LIMIT;
    g->out_limit = RATE_PID_OUT_LIMIT;
}

void ctrl_rate_reset(void)
{
    for (int i = 0; i < AXIS_COUNT; i++) {
        g_fc.ctrl.rate_pid[i].integral         = 0.0f;
        g_fc.ctrl.rate_pid[i].prev_measurement = 0.0f;
        g_fc.ctrl.rate_pid[i].derivative       = 0.0f;
        g_fc.ctrl.rate_pid[i].output           = 0.0f;
        s_dterm[i] = 0.0f;
    }
    s_dterm_primed = false;

    g_fc.ctrl.pid_output       = (vec3f_t){ 0.0f, 0.0f, 0.0f };
    g_fc.ctrl.setpoint_rate_dps = (vec3f_t){ 0.0f, 0.0f, 0.0f };
}

void ctrl_rate_init(void)
{
    load_gains();
    ctrl_rate_reset();

    s_last_sample = 0;
    s_last_us     = micros();
    s_loops       = 0;
    s_skips       = 0;
    s_hz          = 0;
    s_hz_mark_us  = s_last_us;
    s_hz_count    = 0;

    /* Hệ số lọc D tính sẵn theo nhịp danh định, tránh chia trong vòng nóng. */
    s_dterm_alpha = fc_lpf_alpha(RATE_DTERM_LPF_HZ, 1.0f / (float)FC_LOOP_RATE_HZ);
}

/* ==========================================================================
 * Đọc IMU nhất quán
 *
 * Driver IMU ghi từ ngắt ở 8 kHz. Đọc ba trục gyro cùng với sample_count và
 * timestamp mà không khoá ngắt thì có thể lấy được gyro của mẫu này ghép với
 * mốc thời gian của mẫu sau — dt sai thì D sai theo.
 * ========================================================================== */

static void imu_snapshot(vec3f_t *gyro, uint32_t *count, uint32_t *ts)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    *gyro  = g_fc.imu.gyro_filtered_dps;
    *count = g_fc.imu.sample_count;
    *ts    = g_fc.imu.timestamp_us;

    __set_PRIMASK(primask);
}

/* ==========================================================================
 * Vòng điều khiển
 * ========================================================================== */

bool ctrl_rate_update(void)
{
    vec3f_t  gyro;
    uint32_t count, ts;
    imu_snapshot(&gyro, &count, &ts);

    /* Chưa có mẫu mới, hoặc chưa tới lượt theo bộ chia nhịp. */
    if (count == s_last_sample || (count % RATE_LOOP_DIVIDER) != 0u) {
        return false;
    }

    const uint32_t dt_us = fc_elapsed_us(ts, s_last_us);
    s_last_sample = count;
    s_last_us     = ts;

    /*
     * Nhịp bất thường: mất mẫu, tràn bộ đếm, hoặc lần chạy đầu tiên. Chạy PID
     * với dt sai còn tệ hơn bỏ hẳn một bước — nhất là khâu vi phân.
     */
    if (dt_us == 0u || dt_us > 10000u) {
        s_skips++;
        return false;
    }
    const float dt = (float)dt_us * 1.0e-6f;

    /* --- Đo tần số thực tế --- */
    s_hz_count++;
    if (fc_elapsed_us(ts, s_hz_mark_us) >= 1000000u) {
        s_hz         = s_hz_count;
        s_hz_count   = 0;
        s_hz_mark_us = ts;
    }

    /*
     * Chưa arm: xoá sạch rồi thoát. Không để tích phân dồn sẵn trong lúc máy
     * bay còn nằm trên bàn — dồn rồi thì ngay giây đầu sau khi arm nó bung ra.
     */
    if (!g_fc.motor.armed || g_fc.mode != FC_MODE_ARMED) {
        ctrl_rate_reset();
        return false;
    }

    /*
     * --- Setpoint đến từ VÒNG NGOÀI, không tự sinh ở đây ---
     *
     * ctrl_angle.c ghi setpoint_rate_dps: chế độ ACRO thì nó lấy thẳng từ cần,
     * chế độ ANGLE thì nó chạy vòng P trên góc rồi ra tốc độ. Vòng này không
     * cần biết đang ở chế độ nào — nó chỉ có đúng một việc: bám cho được tốc
     * độ được giao. Tách bạch như vậy nên thêm chế độ mới (ALTHOLD, POSHOLD)
     * về sau không phải đụng vào đây.
     */
    const vec3f_t sp_v = g_fc.ctrl.setpoint_rate_dps;
    const float sp[AXIS_COUNT]   = { sp_v.x, sp_v.y, sp_v.z };
    const float meas[AXIS_COUNT] = { gyro.x, gyro.y, gyro.z };

    /*
     * Khâu trộn của bước TRƯỚC có bão hoà không. Dùng kết quả cũ là đúng thứ
     * tự: khâu trộn chạy sau vòng này, nên tại đây chỉ có thông tin của bước
     * trước — và trễ một bước ở 4 kHz là 0,25 ms, không đáng kể.
     */
    const bool saturated = mixer_saturated();

    vec3f_t out;
    float  *out_axis[AXIS_COUNT] = { &out.x, &out.y, &out.z };

    for (int i = 0; i < AXIS_COUNT; i++) {
        pid_t *p = &g_fc.ctrl.rate_pid[i];
        const pid_gains_t *g = &p->gains;

        const float error = sp[i] - meas[i];

        /* --- P --- */
        const float p_term = g->kp * error;

        /* --- I --- */
        if (!saturated) {
            p->integral += g->ki * error * dt;
            p->integral  = fc_constrainf(p->integral, -g->i_limit, g->i_limit);
        }

        /*
         * --- D, lấy trên SỐ ĐO và đảo dấu ---
         * Đạo hàm của số đo, không phải của sai số: gạt cần không sinh xung.
         * Dấu âm vì số đo tăng nghĩa là sai số giảm.
         *
         * Mẫu ĐẦU TIÊN sau khi arm không có mẫu trước để so, mà
         * prev_measurement lại đang bằng 0 trong khi gyro thực tế thì không.
         * Tính đạo hàm lúc đó là lấy đạo hàm của một bước nhảy giả: gyro 5°/s
         * cho ra 20000 °/s² và D giật 4,5 % dải ga ngay lúc arm. Nạp mẫu đầu
         * vào rồi bỏ qua một bước là hết.
         */
        float raw_d = 0.0f;
        if (s_dterm_primed) {
            raw_d = -(meas[i] - p->prev_measurement) / dt;
        }
        p->prev_measurement = meas[i];

        /* Lọc trước khi nhân hệ số — đạo hàm thô gần như toàn nhiễu. */
        s_dterm[i] = fc_lpf(s_dterm[i], raw_d, s_dterm_alpha);
        const float d_term = g->kd * s_dterm[i];

        p->derivative = s_dterm[i];
        p->output     = fc_constrainf(p_term + p->integral + d_term,
                                      -g->out_limit, g->out_limit);
        *out_axis[i]  = p->output;
    }

    s_dterm_primed = true;

    g_fc.ctrl.pid_output  = out;
    g_fc.ctrl.timestamp_us = ts;

    s_loops++;
    return true;
}

uint32_t ctrl_rate_loops(void) { return s_loops; }
uint32_t ctrl_rate_hz(void)    { return s_hz; }
uint32_t ctrl_rate_skips(void) { return s_skips; }
