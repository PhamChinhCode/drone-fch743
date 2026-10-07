/**
 * @file    tlm_stream.c
 * @brief   Bảng đăng ký luồng telemetry, hàm đóng gói và bộ lập lịch.
 */

#include "tlm_stream.h"
#include "tlm_port.h"
#include "param_msg.h"
#include "param_table.h"
#include "param_apply.h"
#include "icm42688.h"

/** Số gói tối đa phát trong một lần gọi update, tránh dồn cục gây tràn đệm. */
#define TLM_MAX_PACKETS_PER_UPDATE  4

static uint8_t      s_seq;
static tlm_parser_t s_parser;

/* ==========================================================================
 * Hàm chuyển thang đo
 * ========================================================================== */

static inline int16_t sat_i16(float v)
{
    if (v >  32767.0f) return  32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}

static inline uint16_t sat_u16(float v)
{
    if (v > 65535.0f) return 65535;
    if (v < 0.0f)     return 0;
    return (uint16_t)v;
}

static inline int16_t rad_to_cdeg(float rad)
{
    return sat_i16(rad * FC_RAD_TO_DEG * 100.0f);
}

static inline int16_t dps_to_ddps(float dps)
{
    return sat_i16(dps * 10.0f);
}

/* ==========================================================================
 * Hàm đóng gói — mỗi hàm ứng với một bản tin
 * ========================================================================== */

static void pack_heartbeat(const fc_t *fc, void *dst)
{
    tlm_heartbeat_t *m = (tlm_heartbeat_t *)dst;

    m->uptime_ms       = fc->sys.uptime_ms;
    m->error_flags     = fc->sys.error_flags;
    m->sensor_health   = (uint16_t)fc->sys.sensor_health;
    m->arm_block_flags = (uint16_t)fc->sys.arm_block_flags;
    m->mode            = (uint8_t)fc->mode;
    m->flight_mode     = (uint8_t)fc->ctrl.mode;
    m->cpu_load_pct    = fc->sys.cpu_load_pct;
    m->reserved        = 0;
}

static void pack_attitude(const fc_t *fc, void *dst)
{
    tlm_attitude_t *m = (tlm_attitude_t *)dst;

    m->roll_cdeg       = rad_to_cdeg(fc->est.attitude_rad.roll);
    m->pitch_cdeg      = rad_to_cdeg(fc->est.attitude_rad.pitch);
    m->yaw_cdeg        = rad_to_cdeg(fc->est.attitude_rad.yaw);
    m->rate_roll_ddps  = dps_to_ddps(fc->est.rate_dps.x);
    m->rate_pitch_ddps = dps_to_ddps(fc->est.rate_dps.y);
    m->rate_yaw_ddps   = dps_to_ddps(fc->est.rate_dps.z);
}

static void pack_imu(const fc_t *fc, void *dst)
{
    tlm_imu_t *m = (tlm_imu_t *)dst;

    m->gyro_ddps[0] = dps_to_ddps(fc->imu.gyro_dps.x);
    m->gyro_ddps[1] = dps_to_ddps(fc->imu.gyro_dps.y);
    m->gyro_ddps[2] = dps_to_ddps(fc->imu.gyro_dps.z);

    /* m/s^2 -> mili-g */
    const float k = 1000.0f / FC_GRAVITY_MPS2;
    m->accel_mg[0] = sat_i16(fc->imu.accel_mps2.x * k);
    m->accel_mg[1] = sat_i16(fc->imu.accel_mps2.y * k);
    m->accel_mg[2] = sat_i16(fc->imu.accel_mps2.z * k);

    m->temperature_cdeg = sat_i16(fc->imu.temperature_c * 100.0f);
    m->error_count      = fc->imu.error_count;
}

static void pack_baro(const fc_t *fc, void *dst)
{
    tlm_baro_t *m = (tlm_baro_t *)dst;

    m->pressure_pa      = (int32_t)fc->baro.pressure_pa;
    m->altitude_cm      = (int32_t)(fc->baro.altitude_rel_m * 100.0f);
    m->climb_rate_cms   = sat_i16(fc->est.climb_rate_mps * 100.0f);
    m->temperature_cdeg = sat_i16(fc->baro.temperature_c * 100.0f);
    m->error_count      = fc->baro.error_count;
}

static void pack_flow(const fc_t *fc, void *dst)
{
    tlm_flow_t *m = (tlm_flow_t *)dst;

    m->flow_x_raw     = fc->flow.flow_x_raw;
    m->flow_y_raw     = fc->flow.flow_y_raw;
    m->velocity_x_cms = sat_i16(fc->flow.velocity_mps.x * 100.0f);
    m->velocity_y_cms = sat_i16(fc->flow.velocity_mps.y * 100.0f);
    m->range_mm       = fc->flow.range_mm;
    m->flow_quality   = fc->flow.flow_quality;
    m->flags          = (uint8_t)((fc->flow.range_valid ? 0x01u : 0u) |
                                  (fc->flow.healthy     ? 0x02u : 0u));
}

static void pack_rc(const fc_t *fc, void *dst)
{
    tlm_rc_t *m = (tlm_rc_t *)dst;

    for (int i = 0; i < 8; i++) {
        m->channel[i] = fc->rc.channel_raw[i];
    }
    m->link_quality   = fc->rc.link_quality;
    m->rssi_dbm       = fc->rc.rssi_dbm;
    m->frame_loss_pct = fc->rc.frame_loss_pct;
    m->flags          = (uint8_t)((fc->rc.failsafe ? 0x01u : 0u) |
                                  (fc->rc.healthy  ? 0x02u : 0u));
}

static void pack_motor(const fc_t *fc, void *dst)
{
    tlm_motor_t *m = (tlm_motor_t *)dst;

    for (int i = 0; i < 4; i++) {
        m->throttle[i] = (i < FC_MOTOR_COUNT) ? fc->motor.throttle[i] : 0;
        m->erpm[i]     = (i < FC_MOTOR_COUNT)
                       ? (uint16_t)(fc->motor.erpm[i] / 100u) : 0;
    }
    m->flags = (uint8_t)((fc->motor.armed     ? 0x01u : 0u) |
                         (fc->motor.saturated ? 0x02u : 0u));
}

static void pack_power(const fc_t *fc, void *dst)
{
    tlm_power_t *m = (tlm_power_t *)dst;

    m->voltage_mv      = sat_u16(fc->power.voltage_v * 1000.0f);
    m->current_ca      = sat_u16(fc->power.current_a * 100.0f);
    m->consumed_mah    = sat_u16(fc->power.consumed_mah);
    m->cell_voltage_mv = sat_u16(fc->power.cell_voltage_v * 1000.0f);
    m->cell_count      = fc->power.cell_count;
    m->flags           = (uint8_t)((fc->power.warning  ? 0x01u : 0u) |
                                   (fc->power.critical ? 0x02u : 0u));
}

static void pack_pid(const fc_t *fc, void *dst)
{
    tlm_pid_t *m = (tlm_pid_t *)dst;

    const float sp[AXIS_COUNT] = {
        fc->ctrl.setpoint_rate_dps.x,
        fc->ctrl.setpoint_rate_dps.y,
        fc->ctrl.setpoint_rate_dps.z
    };
    const float meas[AXIS_COUNT] = {
        fc->imu.gyro_filtered_dps.x,
        fc->imu.gyro_filtered_dps.y,
        fc->imu.gyro_filtered_dps.z
    };
    const float out[AXIS_COUNT] = {
        fc->ctrl.pid_output.x,
        fc->ctrl.pid_output.y,
        fc->ctrl.pid_output.z
    };

    for (int i = 0; i < AXIS_COUNT; i++) {
        m->setpoint_ddps[i]   = dps_to_ddps(sp[i]);
        m->measured_ddps[i]   = dps_to_ddps(meas[i]);
        m->output_permille[i] = sat_i16(out[i] * 1000.0f);
    }
    m->reserved = (uint16_t)(icm42688_notch_hz() + 0.5f);   /* notch gyro dang dung, Hz */
}

static void pack_system(const fc_t *fc, void *dst)
{
    tlm_system_t *m = (tlm_system_t *)dst;

    m->loop_count       = fc->sys.loop_count;
    m->loop_overruns    = fc->sys.loop_overruns;
    m->loop_time_us     = (uint16_t)fc->sys.loop_time_us;
    m->loop_time_max_us = (uint16_t)fc->sys.loop_time_max_us;
    m->imu_dt_us        = (uint16_t)fc->imu.dt_us;
    m->cpu_load_pct     = fc->sys.cpu_load_pct;
    m->flags            = (uint8_t)((fc->sys.sdcard_mounted ? 0x01u : 0u) |
                                    (fc->sys.logging_active ? 0x02u : 0u) |
                                    (fc->sys.usb_connected  ? 0x04u : 0u));
}

/* ==========================================================================
 * Bảng đăng ký
 *
 * period_ms ở đây là giá trị mặc định lúc khởi động (hồ sơ FLIGHT).
 * Thêm luồng mới = thêm một dòng vào bảng này.
 * ========================================================================== */

static tlm_stream_t g_tlm_streams[] = {
    /*  id                 size                     period  hàm đóng gói   */
    { TLM_MSG_HEARTBEAT, sizeof(tlm_heartbeat_t), 1000, pack_heartbeat, 0 },
    { TLM_MSG_ATTITUDE,  sizeof(tlm_attitude_t),   100, pack_attitude,  0 },
    { TLM_MSG_IMU,       sizeof(tlm_imu_t),          0, pack_imu,       0 },
    { TLM_MSG_BARO,      sizeof(tlm_baro_t),       500, pack_baro,      0 },
    { TLM_MSG_FLOW,      sizeof(tlm_flow_t),         0, pack_flow,      0 },
    { TLM_MSG_RC,        sizeof(tlm_rc_t),         500, pack_rc,        0 },
    { TLM_MSG_MOTOR,     sizeof(tlm_motor_t),        0, pack_motor,     0 },
    { TLM_MSG_POWER,     sizeof(tlm_power_t),     1000, pack_power,     0 },
    { TLM_MSG_PID,       sizeof(tlm_pid_t),          0, pack_pid,       0 },
    { TLM_MSG_SYSTEM,    sizeof(tlm_system_t),    2000, pack_system,    0 },
};

#define TLM_STREAM_COUNT  (sizeof(g_tlm_streams) / sizeof(g_tlm_streams[0]))

static tlm_stream_t *stream_find(uint8_t msg_id)
{
    for (size_t i = 0; i < TLM_STREAM_COUNT; i++) {
        if (g_tlm_streams[i].id == msg_id) {
            return &g_tlm_streams[i];
        }
    }
    return NULL;
}

/* ==========================================================================
 * Phát một gói
 * ========================================================================== */

static bool stream_emit(const tlm_stream_t *st, const fc_t *fc)
{
    uint8_t payload[TLM_MAX_PAYLOAD];
    uint8_t frame[TLM_FRAME_MAX];

    if (st->size > TLM_MAX_PAYLOAD || st->pack == NULL) {
        return false;
    }

    st->pack(fc, payload);

    const uint16_t n = tlm_frame_encode(frame, sizeof(frame),
                                        st->id, s_seq, payload, st->size);
    if (n == 0) {
        return false;
    }

    if (!tlm_port_write(frame, n)) {
        return false;   /* đệm đầy — bỏ gói này, lần sau gửi lại */
    }

    s_seq++;
    return true;
}

/* ==========================================================================
 * API
 * ========================================================================== */

void tlm_stream_init(void)
{
    s_seq = 0;
    tlm_parser_init(&s_parser);

    for (size_t i = 0; i < TLM_STREAM_COUNT; i++) {
        g_tlm_streams[i].next_due_ms = 0;
    }
}

uint8_t tlm_stream_update(uint32_t now_ms)
{
    uint8_t sent = 0;

    for (size_t i = 0; i < TLM_STREAM_COUNT; i++) {
        tlm_stream_t *st = &g_tlm_streams[i];

        if (st->period_ms == 0) {
            continue;                       /* luồng đang tắt */
        }
        /* Phép so sánh có dấu này an toàn khi bộ đếm mili giây tràn. */
        if ((int32_t)(now_ms - st->next_due_ms) < 0) {
            continue;                       /* chưa tới hạn */
        }

        if (stream_emit(st, &g_fc)) {
            sent++;
        }
        st->next_due_ms = now_ms + st->period_ms;

        if (sent >= TLM_MAX_PACKETS_PER_UPDATE) {
            break;
        }
    }

    return sent;
}

bool tlm_stream_set_period(uint8_t msg_id, uint16_t period_ms)
{
    tlm_stream_t *st = stream_find(msg_id);
    if (st == NULL) {
        return false;
    }
    st->period_ms   = period_ms;
    st->next_due_ms = 0;
    return true;
}

void tlm_stream_apply_profile(tlm_profile_t profile)
{
    /* Bảng chu kỳ theo từng hồ sơ, cùng thứ tự với g_tlm_streams[]. */
    static const uint16_t table[TLM_PROFILE_COUNT][TLM_STREAM_COUNT] = {
        /*            HB   ATT  IMU  BARO FLOW RC   MOT  PWR  PID  SYS  */
        [TLM_PROFILE_SILENT] = { 1000,   0,   0,   0,   0,   0,   0,   0,   0,   0 },
        [TLM_PROFILE_FLIGHT] = { 1000, 100,   0, 500,   0, 500,   0, 1000,  0, 2000 },
        [TLM_PROFILE_TUNING] = {  500,  20,  20,   0,   0, 100,  20,  500, 10, 1000 },
        [TLM_PROFILE_DEBUG]  = {  200,  20,  20, 100, 100,  50,  20,  200, 20,  500 },
    };

    if (profile >= TLM_PROFILE_COUNT) {
        return;
    }
    for (size_t i = 0; i < TLM_STREAM_COUNT; i++) {
        g_tlm_streams[i].period_ms   = table[profile][i];
        g_tlm_streams[i].next_due_ms = 0;
    }
}

bool tlm_stream_send_payload(uint8_t msg_id, const void *payload, uint8_t len)
{
    uint8_t frame[TLM_FRAME_MAX];

    const uint16_t n = tlm_frame_encode(frame, sizeof(frame),
                                        msg_id, s_seq, payload, len);
    if (n == 0 || !tlm_port_write(frame, n)) {
        return false;
    }

    s_seq++;
    return true;
}

bool tlm_stream_send_now(uint8_t msg_id)
{
    const tlm_stream_t *st = stream_find(msg_id);
    return (st != NULL) && stream_emit(st, &g_fc);
}

bool tlm_stream_send_text(uint8_t severity, const char *text)
{
    tlm_text_t msg;
    uint8_t    frame[TLM_FRAME_MAX];

    if (text == NULL) {
        return false;
    }

    memset(&msg, 0, sizeof(msg));
    msg.severity = severity;

    size_t n = strlen(text);
    if (n > sizeof(msg.text)) {
        n = sizeof(msg.text);
    }
    memcpy(msg.text, text, n);

    /* Chỉ gửi đúng phần chữ có thật, không gửi thừa byte 0. */
    const uint8_t len = (uint8_t)(1u + n);

    const uint16_t sz = tlm_frame_encode(frame, sizeof(frame),
                                         TLM_MSG_TEXT, s_seq, &msg, len);
    if (sz == 0 || !tlm_port_write(frame, sz)) {
        return false;
    }

    s_seq++;
    return true;
}

/* ==========================================================================
 * Xử lý lệnh từ máy tính
 *
 * Các hàm hook dưới đây khai báo `weak`: mặc định không làm gì. Khi bạn viết
 * xong module hiệu chuẩn / lưu cấu hình, chỉ cần định nghĩa lại hàm cùng tên
 * ở file khác là trình liên kết tự thay thế, không phải sửa file này.
 * ========================================================================== */

__attribute__((weak)) void fc_hook_calibrate_gyro(void)  { }
__attribute__((weak)) void fc_hook_calibrate_accel(void) { }
__attribute__((weak)) void fc_hook_calibrate_baro(void)  { }
__attribute__((weak)) void fc_hook_save_config(void)     { }
__attribute__((weak)) void fc_hook_reboot(void)          { }
__attribute__((weak)) void fc_hook_log_start(void)       { }
__attribute__((weak)) void fc_hook_log_stop(void)        { }

/*
 * 0x41 — nạp hệ số PID.
 *
 * LỆNH CŨ, GIỮ LẠI CHO TƯƠNG THÍCH. Phần mềm mới nên dùng
 * TLM_MSG_CMD_PARAM_SET: nó ghi được mọi tham số, có ACK, và trả về giá trị
 * firmware thực sự nhận.
 *
 * VÌ SAO PHẢI VIẾT LẠI: bản cũ ghi THẲNG vào g_fc.ctrl.rate_pid[]. Từ khi hệ
 * số PID sống trong g_params, làm vậy tạo ra HAI nguồn ghi cho cùng một giá
 * trị — và lần fc_params_apply() kế tiếp (do một lệnh `set` bất kỳ kích
 * hoạt) sẽ lặng lẽ xoá sạch những gì lệnh này vừa ghi. Nay nó đi qua đúng
 * đường như mọi thứ khác.
 */
static bool set_pid_param(const char *prefix, const char *suffix, float value)
{
    char name[PARAM_NAME_MAX];
    size_t n = 0;

    while (*prefix && n < sizeof(name) - 1u) name[n++] = *prefix++;
    while (*suffix && n < sizeof(name) - 1u) name[n++] = *suffix++;
    name[n] = '\0';

    const uint16_t idx = param_find(name);
    return (idx != PARAM_INDEX_NONE) && param_set_f32(idx, value);
}

static bool handle_set_pid(const uint8_t *payload, uint8_t len)
{
    static const char *const AXIS_PREFIX[AXIS_COUNT] = {
        "rate_pid_roll_", "rate_pid_pitch_", "rate_pid_yaw_"
    };

    if (len < sizeof(tlm_cmd_set_pid_t)) {
        return false;
    }

    tlm_cmd_set_pid_t cmd;
    memcpy(&cmd, payload, sizeof(cmd));

    if (cmd.axis >= AXIS_COUNT || cmd.loop > 1) {
        return false;
    }

    /* Không cho đổi hệ số khi đang bay — tránh mất kiểm soát giữa chừng. */
    if (g_fc.mode == FC_MODE_ARMED) {
        return false;
    }

    bool ok;

    if (cmd.loop == 0) {
        ok  = set_pid_param(AXIS_PREFIX[cmd.axis], "kp", cmd.kp);
        ok &= set_pid_param(AXIS_PREFIX[cmd.axis], "ki", cmd.ki);
        ok &= set_pid_param(AXIS_PREFIX[cmd.axis], "kd", cmd.kd);
    } else {
        /*
         * Vòng góc cố ý CHỈ CÓ P và dùng CHUNG một hệ số cho cả roll lẫn
         * pitch — xem ghi chú dài trong fc_config.h về việc vì sao không có
         * I và D ở đây. Nên ki/kd/kff của lệnh này bị bỏ qua, và ba trục đều
         * ghi vào cùng một tham số.
         */
        const uint16_t idx = param_find("angle_pid_kp");
        ok = (idx != PARAM_INDEX_NONE) && param_set_f32(idx, cmd.kp);
    }

    if (ok) {
        fc_params_apply();
    }
    return ok;
}

static bool handle_action(const uint8_t *payload, uint8_t len)
{
    if (len < sizeof(tlm_cmd_action_t)) {
        return false;
    }

    tlm_cmd_action_t cmd;
    memcpy(&cmd, payload, sizeof(cmd));

    /* Mọi lệnh dưới đây đều nguy hiểm nếu đang bay. */
    if (g_fc.mode == FC_MODE_ARMED) {
        return false;
    }

    switch (cmd.action) {
    case TLM_ACTION_CALIB_GYRO:  fc_hook_calibrate_gyro();  return true;
    case TLM_ACTION_CALIB_ACCEL: fc_hook_calibrate_accel(); return true;
    case TLM_ACTION_CALIB_BARO:  fc_hook_calibrate_baro();  return true;
    case TLM_ACTION_SAVE_CONFIG: fc_hook_save_config();     return true;
    case TLM_ACTION_REBOOT:      fc_hook_reboot();          return true;
    case TLM_ACTION_LOG_START:   fc_hook_log_start();       return true;
    case TLM_ACTION_LOG_STOP:    fc_hook_log_stop();        return true;
    default:                                                return false;
    }
}

/*
 * Mỗi lần gọi rút tối đa 64 byte. Ở 921600 baud mà vòng lặp chính chạy trên
 * 1 kHz thì con số này thừa sức theo kịp, còn khi vòng lặp bị kẹt một nhịp
 * thì nó cũng chặn không cho việc phân tích uplink ăn hết một chu kỳ điều
 * khiển. Phần chưa lấy vẫn nằm nguyên trong đệm DMA vòng tròn.
 */
#define TLM_RX_BYTES_PER_UPDATE  64u

uint8_t tlm_stream_rx_update(void)
{
    uint8_t buf[TLM_RX_BYTES_PER_UPDATE];
    uint8_t handled = 0;

    const uint16_t n = tlm_port_read(buf, sizeof(buf));

    for (uint16_t i = 0; i < n; i++) {
        if (tlm_parser_push(&s_parser, buf[i]) &&
            tlm_stream_handle_command(&s_parser)) {
            handled++;
        }
    }
    return handled;
}

const tlm_parser_t *tlm_stream_rx_stats(void)
{
    return &s_parser;
}

bool tlm_stream_handle_command(const tlm_parser_t *p)
{
    if (p == NULL) {
        return false;
    }

    switch (p->id) {

    case TLM_MSG_CMD_SET_RATE: {
        if (p->len < sizeof(tlm_cmd_set_rate_t)) {
            return false;
        }
        tlm_cmd_set_rate_t cmd;
        memcpy(&cmd, p->payload, sizeof(cmd));
        return tlm_stream_set_period(cmd.msg_id, cmd.period_ms);
    }

    case TLM_MSG_CMD_SET_PID:
        return handle_set_pid(p->payload, p->len);

    case TLM_MSG_CMD_ACTION:
        return handle_action(p->payload, p->len);

    default:
        /*
         * Nhom lenh tham so / CLI / motor test do param_msg.c lo. De o day
         * thay vi liet ke tung ma lenh: them mot lenh moi chi phai sua mot
         * file, va file nay khong can biet nhom do co bao nhieu lenh.
         */
        return param_msg_handle(p);
    }
}
