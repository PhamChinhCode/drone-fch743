/**
 * @file    estimator.c
 * @brief   Hiện thực bộ điều phối ước lượng.
 */

#include "estimator.h"
#include "ekf_attitude.h"
#include "ekf_altitude.h"
#include "ekf_velocity.h"
#include "fc_state.h"
#include "fc_time.h"
#include "stm32h7xx.h"

static uint32_t s_last_us;
static bool     s_started;

static uint32_t s_steps;
static uint32_t s_baro_updates;
static uint32_t s_range_updates;
static uint32_t s_range_rejected;

/* Đếm mẫu đã nạp, để chỉ nạp mỗi mẫu cảm biến đúng một lần. */
static uint32_t s_baro_seen;
static uint32_t s_flow_seen;
static uint32_t s_flow_us;
static bool     s_flow_started;

/* ==========================================================================
 * Đọc IMU nhất quán
 *
 * Driver IMU ghi vào g_fc.imu từ ngắt ở 8 kHz, trong khi hàm này chạy ở tầng
 * vòng lặp chính. Đọc sáu số float mà không khoá ngắt thì có thể lấy được ba
 * trục gyro của mẫu này ghép với ba trục accel của mẫu sau — vector trọng lực
 * ghép sai mẫu sẽ thành một cú giật giả mà bộ lọc tưởng là chuyển động thật.
 *
 * Vùng khoá chỉ dài vài chục chu kỳ nên không ảnh hưởng ngắt gyro.
 * ========================================================================== */

static void imu_snapshot(vec3f_t *gyro, vec3f_t *accel, bool *healthy)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    *gyro    = g_fc.imu.gyro_dps;
    *accel   = g_fc.imu.accel_mps2;
    *healthy = g_fc.imu.healthy;

    __set_PRIMASK(primask);
}

/* ==========================================================================
 * API
 * ========================================================================== */

void estimator_init(void)
{
    ekf_attitude_init();
    ekf_altitude_init();
    ekf_velocity_init();

    s_last_us        = micros();
    s_started        = false;
    s_steps          = 0;
    s_baro_updates   = 0;
    s_range_updates  = 0;
    s_range_rejected = 0;
    s_baro_seen      = 0;
    s_flow_seen      = 0;
    s_flow_us        = 0;
    s_flow_started   = false;

    g_fc.est.attitude_q      = (quatf_t){ 1.0f, 0.0f, 0.0f, 0.0f };
    g_fc.est.attitude_valid  = false;
    g_fc.est.altitude_valid  = false;
    g_fc.est.position_valid  = false;
}

bool estimator_update(uint32_t now_us)
{
    if (fc_elapsed_us(now_us, s_last_us) < EST_PERIOD_US) {
        return false;
    }

    const float dt = (float)fc_elapsed_us(now_us, s_last_us) * 1.0e-6f;
    s_last_us = now_us;

    /*
     * Nhịp đầu tiên sau khi khởi động có dt vô nghĩa (bằng khoảng thời gian
     * từ lúc init tới lúc vòng lặp chạy tới đây). Bỏ qua nó.
     */
    if (!s_started) {
        s_started = true;
        return false;
    }

    vec3f_t gyro, accel;
    bool    imu_ok;
    imu_snapshot(&gyro, &accel, &imu_ok);

    if (!imu_ok) {
        g_fc.est.attitude_valid = false;
        return false;
    }

    /* ---------------- Góc ---------------- */
    ekf_attitude_update(gyro, accel, dt);

    if (!ekf_attitude_is_valid()) {
        return false;               /* chưa dựng được quaternion ban đầu */
    }

    /* ---------------- Độ cao ---------------- */

    /*
     * Tách gia tốc chuyển động ra khỏi số đo accel.
     *
     * Quy ước của mạch: accel đọc ra (0,0,+g) khi nằm yên, tức nó chỉ theo
     * HƯỚNG trọng lực. Xoay sang hệ NED rồi trừ đi g sẽ còn lại phần gia tốc
     * thật; đảo dấu để lấy chiều dương là lên (NED có Z hướng xuống).
     *
     *   a_ned  = R · a_body
     *   a_lên  = a_ned.z - g
     * Nằm yên: a_ned.z = g nên a_lên = 0. Bay lên 1 m/s²: a_ned.z = g + 1.
     */
    const vec3f_t a_body = vec3f_scale(accel, (float)EST_ACCEL_Z_SIGN);
    const vec3f_t a_ned  = ekf_attitude_body_to_ned(a_body);
    const float   a_up   = a_ned.z - FC_GRAVITY_MPS2;

    ekf_altitude_predict(a_up, dt);

    /* Gia tốc ngang hệ NED cho bộ lọc vận tốc — cùng nguồn a_ned ở trên. */
    ekf_velocity_predict(a_ned.x, a_ned.y, dt);

    /* Baro: chỉ nạp mẫu mới, và chỉ khi đã có mốc mặt đất. */
    if (g_fc.baro.healthy && g_fc.baro.calibrated &&
        g_fc.baro.sample_count != s_baro_seen) {

        s_baro_seen = g_fc.baro.sample_count;
        ekf_altitude_update_baro(g_fc.baro.altitude_rel_m);
        s_baro_updates++;
    }

    /* Laser: chỉ nạp mẫu mới và khi driver báo số đo hợp lệ. */
    if (g_fc.flow.healthy && g_fc.flow.range_valid &&
        g_fc.flow.sample_count != s_flow_seen) {

        s_flow_seen = g_fc.flow.sample_count;

        const float range_m = (float)g_fc.flow.range_mm * 0.001f;
        if (ekf_altitude_update_range(range_m, ekf_attitude_tilt_cos())) {
            s_range_updates++;
        } else {
            s_range_rejected++;
        }

        /*
         * Cùng một gói MTF-01P mang cả khoảng cách lẫn optical flow, nên xử lý
         * luôn ở đây. dt lấy từ khoảng cách giữa hai mốc thời gian gói, không
         * lấy nhịp danh định — gói UART tới không đều.
         */
        const uint32_t flow_dt_us = fc_elapsed_us(g_fc.flow.timestamp_us, s_flow_us);
        s_flow_us = g_fc.flow.timestamp_us;

        if (s_flow_started) {
            const float fdt = (float)flow_dt_us * 1.0e-6f;
            const float fx  = (float)g_fc.flow.flow_x_raw * FLOW_RAD_PER_COUNT;
            const float fy  = (float)g_fc.flow.flow_y_raw * FLOW_RAD_PER_COUNT;

            (void)ekf_velocity_update_flow(fx, fy, g_fc.imu.gyro_dps, fdt,
                                           range_m, ekf_attitude_tilt_cos(),
                                           g_fc.flow.flow_quality);
        }
        s_flow_started = true;
    }

    /* ---------------- Ghi ra khối trạng thái chung ---------------- */

    const euler_t e = ekf_attitude_euler();

    g_fc.est.attitude_q     = ekf_attitude_quaternion();
    g_fc.est.attitude_rad   = e;
    g_fc.est.rate_dps       = g_fc.imu.gyro_filtered_dps;
    g_fc.est.attitude_valid = true;

    g_fc.est.altitude_m     = ekf_altitude_m();
    g_fc.est.climb_rate_mps = ekf_altitude_climb_rate_mps();
    g_fc.est.altitude_valid = ekf_altitude_is_valid();

    /* NED có Z hướng xuống, nên độ cao vào với dấu âm. */
    g_fc.est.position_m.z = -g_fc.est.altitude_m;
    g_fc.est.velocity_mps.z = -g_fc.est.climb_rate_mps;

    /*
     * Vận tốc ngang từ optical flow. Vị trí thì CHƯA — tích phân vận tốc sẽ
     * trôi, và chưa có gì kiểm chứng được nó, nên position_valid vẫn là false
     * cho tới khi bộ giữ vị trí được xây và kiểm.
     */
    g_fc.est.velocity_mps.x = ekf_velocity_north();
    g_fc.est.velocity_mps.y = ekf_velocity_east();
    g_fc.est.position_valid = false;

    g_fc.est.timestamp_us = now_us;
    s_steps++;
    return true;
}

uint32_t estimator_steps(void)          { return s_steps; }
uint32_t estimator_baro_updates(void)   { return s_baro_updates; }
uint32_t estimator_range_updates(void)  { return s_range_updates; }
uint32_t estimator_range_rejected(void) { return s_range_rejected; }
