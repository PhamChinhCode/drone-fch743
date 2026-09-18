/**
 * @file    mav_link.c
 * @brief   Hiện thực lớp bản tin MAVLink v2.
 */

/*
 * Chỉ dùng MỘT kênh MAVLink (UART8). Mặc định thư viện cấp 4 kênh, mỗi kênh
 * giữ tĩnh một mavlink_message_t + mavlink_status_t — khai báo 1 tiết kiệm
 * khoảng 900 byte RAM mà không mất gì.
 *
 * PHẢI đặt TRƯỚC khi include mavlink.h.
 */
#define MAVLINK_COMM_NUM_BUFFERS 1

#include "mav_link.h"
#include "mav_port.h"
#include "fc_state.h"
#include "fc_time.h"
#include "arming.h"
#include "ctrl_offboard.h"
#include "param_table.h"
#include "main.h"
#include "fc_git_version.h"   /* sinh tu cmake/fc_git_version.cmake */
#include "ekf_attitude.h"
#include "ekf_altitude.h"
#include "ekf_velocity.h"

/*
 * Thư viện sinh tự động nên không sửa được, và nó đọc/ghi các trường trong
 * struct đóng gói chặt. Chặn đúng hai cảnh báo đó quanh chỗ include, để cảnh
 * báo của code mình vẫn hiện ra bình thường.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

/* ==========================================================================
 * Trạng thái module
 * ========================================================================== */

static mavlink_status_t  s_rx_status;
static mavlink_message_t s_rx_msg;

static uint32_t s_link_last_ms;     /**< HEARTBEAT gần nhất từ máy tính nhúng */
static uint32_t s_rx_frames;
static uint32_t s_rx_errors;
static uint8_t  s_parse_err_last;   /**< ảnh chụp trước của bộ đếm 8 bit      */
static uint32_t s_tx_dropped;

/* ==========================================================================
 * Gửi một khung đã đóng gói
 * ========================================================================== */

/**
 * Chuyển mavlink_message_t thành byte rồi đẩy vào ring buffer của mav_port.
 *
 * Đệm nằm trên STACK và cỡ 280 byte. Không sao: stack mọc ngược từ cuối
 * DTCMRAM và còn dư hơn 100 KB. mav_port_write() chép sang .dma_buffer nên
 * đệm này chết đi cũng không ảnh hưởng gì tới DMA.
 */
static void send_msg(const mavlink_message_t *msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    const uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

    if (!mav_port_write(buf, len)) {
        s_tx_dropped++;
    }
}

/* ==========================================================================
 * Ánh xạ trạng thái nội bộ sang từ vựng MAVLink
 * ========================================================================== */

static uint8_t mav_system_status(void)
{
    switch (g_fc.mode) {
    case FC_MODE_INIT:        return MAV_STATE_BOOT;
    case FC_MODE_CALIBRATING: return MAV_STATE_CALIBRATING;
    case FC_MODE_DISARMED:    return MAV_STATE_STANDBY;
    case FC_MODE_ARMED:       return MAV_STATE_ACTIVE;
    case FC_MODE_FAILSAFE:    return MAV_STATE_CRITICAL;
    case FC_MODE_FAULT:       return MAV_STATE_EMERGENCY;
    default:                  return MAV_STATE_UNINIT;
    }
}

static uint8_t mav_base_mode(void)
{
    /*
     * CUSTOM_MODE_ENABLED bật vì custom_mode chở flight_mode_t của dự án —
     * ACRO / ANGLE / ALTHOLD / POSHOLD không có mã MAVLink tiêu chuẩn tương
     * ứng. MAVROS sẽ hiện chúng dưới dạng số.
     */
    uint8_t base = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                 | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

    if (g_fc.motor.armed) {
        base |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    /* ACRO là điều khiển tốc độ góc trần, mọi chế độ còn lại đều có cân bằng. */
    if (g_fc.ctrl.mode != FLIGHT_MODE_ACRO) {
        base |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    }

    return base;
}

/** Đổi bitmask cảm biến của dự án sang bitmask MAV_SYS_STATUS_SENSOR_*. */
static uint32_t mav_sensor_bits(uint32_t mask)
{
    uint32_t out = 0;

    if (mask & SENSOR_GYRO)  { out |= MAV_SYS_STATUS_SENSOR_3D_GYRO; }
    if (mask & SENSOR_ACCEL) { out |= MAV_SYS_STATUS_SENSOR_3D_ACCEL; }
    if (mask & SENSOR_MAG)   { out |= MAV_SYS_STATUS_SENSOR_3D_MAG; }
    if (mask & SENSOR_BARO)  { out |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE; }
    if (mask & SENSOR_FLOW)  { out |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW; }
    if (mask & SENSOR_RANGE) { out |= MAV_SYS_STATUS_SENSOR_LASER_POSITION; }
    if (mask & SENSOR_RC)    { out |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER; }
    if (mask & SENSOR_POWER) { out |= MAV_SYS_STATUS_SENSOR_BATTERY; }

    return out;
}

/* ==========================================================================
 * Các bản tin downlink
 * ========================================================================== */

static void send_heartbeat(void)
{
    mavlink_message_t msg;

    mavlink_msg_heartbeat_pack(MAV_SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, &msg,
                               MAV_TYPE_QUADROTOR,
                               MAV_AUTOPILOT_GENERIC,
                               mav_base_mode(),
                               (uint32_t)g_fc.ctrl.mode,
                               mav_system_status());
    send_msg(&msg);
}

static void send_sys_status(void)
{
    mavlink_message_t msg;

    /*
     * present: những cảm biến bo mạch này CÓ. Lấy từ danh sách cứng chứ không
     * từ sensor_health — nếu lấy từ health thì một cảm biến hỏng sẽ biến mất
     * khỏi danh sách thay vì hiện ra là "có nhưng hỏng", mà đó đúng là thứ ta
     * cần nhìn thấy nhất.
     */
    const uint32_t present = mav_sensor_bits(SENSOR_GYRO | SENSOR_ACCEL |
                                             SENSOR_BARO | SENSOR_FLOW |
                                             SENSOR_RANGE | SENSOR_RC |
                                             SENSOR_POWER | SENSOR_MAG);
    uint32_t health = mav_sensor_bits(g_fc.sys.sensor_health);

    /*
     * --- Bit OPTICAL_FLOW noi ve BO UOC LUONG, khong noi ve driver ---
     *
     * sys.sensor_health tra loi "MTF01P con noi chuyen khong". Do KHONG phai
     * cau hoi ma may tinh nhung can tra loi. Cai no can biet la "vx/vy trong
     * LOCAL_POSITION_NED co dung duoc khong", va hai thu do khac han nhau.
     *
     * Da do that tren ban: sensor_health = 0x051F, tuc bit SENSOR_FLOW DANG
     * BAT, trong khi ekf_velocity_is_valid() sai va bo uoc luong dang phat ra
     * -0,133 m/s tu tich phan gia toc ke thuan tuy voi may bay nam im. Ben Pi
     * doc bit cu se ket luan van toc dung duoc va nap ca so rac do vao EKF.
     *
     * est.position_valid duoc gan dung bang ekf_velocity_is_valid() trong
     * estimator.c, va no da nam san trong blackboard nen khong phai keo them
     * phu thuoc vao module uoc luong.
     *
     * Bit nay doi CUNG KHUNG voi covariance 1e6 cua ODOMETRY (331) — hai kenh
     * doc cung mot bien. (Truoc 1.5 bit nay con phuc vu LOCAL_POSITION_NED,
     * nay da ngung phat.)
     */
    if (g_fc.est.position_valid) {
        health |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    } else {
        health &= ~(uint32_t)MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }

    /*
     * battery_remaining = -1 nghĩa là "không biết". Dự án đo điện áp chứ không
     * đếm dung lượng, mà suy phần trăm từ điện áp lúc đang tải thì sai lệch
     * lớn — báo không biết trung thực hơn là báo một con số bịa.
     */
    mavlink_msg_sys_status_pack(MAV_SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, &msg,
                                present,
                                present,
                                health,
                                (uint16_t)(g_fc.sys.cpu_load_pct * 10u),
                                (uint16_t)(g_fc.power.voltage_v * 1000.0f),
                                (int16_t)(g_fc.power.current_a * 100.0f),
                                -1,
                                0, 0,
                                g_fc.imu.error_count,
                                g_fc.rc.error_count,
                                g_fc.baro.error_count,
                                g_fc.flow.error_count,
                                0, 0, 0);
    send_msg(&msg);
}

static void send_attitude(uint32_t now_ms)
{
    mavlink_message_t msg;

    /* est.rate_dps là độ/giây, MAVLink đòi radian/giây. */
    mavlink_msg_attitude_pack(MAV_SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, &msg,
                              now_ms,
                              g_fc.est.attitude_rad.roll,
                              g_fc.est.attitude_rad.pitch,
                              g_fc.est.attitude_rad.yaw,
                              g_fc.est.rate_dps.x * FC_DEG_TO_RAD,
                              g_fc.est.rate_dps.y * FC_DEG_TO_RAD,
                              g_fc.est.rate_dps.z * FC_DEG_TO_RAD);
    send_msg(&msg);
}

/*
 * --- GLOBAL_POSITION_INT (33) ---
 *
 * Bo mach nay KHONG co GPS. Tu hop dong 1.5 chi con 1 Hz (GIAO_UOC 11.1 #14):
 * optical_flow_node ben Pi da chuyen sang lay do cao tu laser (DISTANCE_SENSOR),
 * ban tin nay chi con cho telemetry_aggregator hien thi len GCS.
 *
 * lat/lon = 0 va hdg = 65535 la cach bao "khong biet" theo dung dac ta. Chi
 * alt, relative_alt va vx/vy/vz mang thong tin that.
 */
static void send_global_position_int(uint32_t now_ms)
{
    mavlink_message_t msg;

    /* Van toc he NED, don vi cm/s. vz duong la di xuong — trung quy uoc MAVLink. */
    mavlink_msg_global_position_int_pack(MAV_SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, &msg,
                                         now_ms,
                                         0, 0,
                                         (int32_t)(g_fc.baro.altitude_m * 1000.0f),
                                         (int32_t)(g_fc.est.altitude_m * 1000.0f),
                                         (int16_t)(g_fc.est.velocity_mps.x * 100.0f),
                                         (int16_t)(g_fc.est.velocity_mps.y * 100.0f),
                                         (int16_t)(g_fc.est.velocity_mps.z * 100.0f),
                                         UINT16_MAX);
    send_msg(&msg);
}

/*
 * --- LOCAL_POSITION_NED (32) va VFR_HUD (74): NGUNG PHAT tu hop dong 1.5 ---
 *
 * GIAO_UOC 11.1 #14: Pi 4 bao hoa CPU, mavros_node giai ma MOI ban tin toi du
 * khong plugin nao dung. LOCAL_POSITION_NED trung du lieu voi ODOMETRY (331,
 * co covariance) — da chay song song tu 1.2 dung nhu 10.4; VFR_HUD chi de hien
 * thi. Pi tat ca hai plugin 09-14. So 32, 74 khong tai su dung cho viec khac.
 */

/*
 * --- ODOMETRY (331) --- hợp đồng 1.2, GIAO_UOC mục 11.2 "Cách điền ODOMETRY".
 *
 * Đã chạy song song LOCAL_POSITION_NED từ 1.2 tới 1.4; từ 1.5 là bản tin vị trí/
 * vận tốc DUY NHẤT (LOCAL_POSITION_NED ngừng, GIAO_UOC 11.1 #14).
 *
 * VẬN TỐC LÀ HỆ THÂN FRD, KHÔNG PHẢI NED: MAVROS không đọc child_frame_id mà
 * luôn coi vx/vy/vz là thân FRD. Điền nhầm NED thì ROS nhận sai im lặng. Xoay
 * bằng CẢ ma trận thái độ (không chỉ yaw) — đó mới đúng là hệ thân; khi nằm
 * phẳng hai cách cho cùng kết quả.
 *
 * Covariance là tam giác trên 6×6 theo hàng: đường chéo ở chỉ số 0,6,11,15,18,20.
 * 1e6 = "đừng dùng". Vì MAVROS bỏ trường quality, 1e6 là cờ hiệu lực DUY NHẤT
 * Pi thấy — nên nó đọc CÙNG g_fc.est.position_valid với bit flow của
 * SYS_STATUS, tức hai bên đổi trong cùng một vòng lặp ước lượng.
 *
 * reset_counter luôn 0: vị trí chỉ đặt về gốc ở estimator_init(), lúc khởi động.
 * Mất flow rồi có lại thì vị trí được GIỮ, không đặt lại.
 */
#define MAV_COV_UNKNOWN   1.0e6f
/* Laser không được dùng quá lâu hơn chừng này thì z/vz của ODOMETRY là "đừng dùng". Laser ~100 Hz. */
#define MAV_ODOM_RANGE_MAX_AGE_MS 300u
#define MAV_ODOM_ATT_VAR  ((1.0f * FC_DEG_TO_RAD) * (1.0f * FC_DEG_TO_RAD))  /* (1°)², mục 8.4 */

static const uint8_t k_cov_diag[6] = { 0u, 6u, 11u, 15u, 18u, 20u };

/*
 * Mốc bắt đầu tích phân vị trí ngang, ms. Vị trí là dẫn đường suy tính và
 * không bao giờ được đặt lại sau khởi động, nên sai số tích luỹ từ lần ĐẦU TIÊN
 * có vận tốc hợp lệ — kể cả những quãng mất flow ở giữa.
 */
static bool     s_pos_started;
static uint32_t s_pos_start_ms;

static void send_odometry(uint32_t now_ms)
{
    mavlink_message_t msg;
    float pose_cov[21];
    float vel_cov[21];

    const bool vel_ok = g_fc.est.position_valid;
    if (vel_ok && !s_pos_started) {
        s_pos_started  = true;
        s_pos_start_ms = now_ms;
    }

    /* Ngoài đường chéo: không có số, điền 0 (đặc tả: phần tử đầu NaN mới là "không biết"). */
    memset(pose_cov, 0, sizeof(pose_cov));
    memset(vel_cov,  0, sizeof(vel_cov));

    /* --- pose: x, y, z, roll, pitch, yaw --- */
    const float sig_v = ekf_velocity_uncertainty_mps();
    if (vel_ok) {
        const float t_s = (float)(now_ms - s_pos_start_ms) * 1.0e-3f;
        const float sig_p = sig_v * t_s;
        pose_cov[k_cov_diag[0]] = sig_p * sig_p;
        pose_cov[k_cov_diag[1]] = sig_p * sig_p;
    } else {
        pose_cov[k_cov_diag[0]] = MAV_COV_UNKNOWN;
        pose_cov[k_cov_diag[1]] = MAV_COV_UNKNOWN;
    }
    /*
     * Hợp đồng 1.7: z và vz chỉ hợp lệ khi laser vừa được DÙNG. Thiếu laser (nghiêng > 25 độ,
     * ngoài tầm, bị che) độ cao chỉ còn baro + gia tốc, trôi tới ~0,9 m/s mà P vẫn báo sigma
     * ~0,06 m/s - EKF Pi tin theo và z lao xuống -1,9 m khi cầm tay nghiêng (đo 09-18).
     */
    const bool z_ok = g_fc.est.altitude_valid &&
                      ekf_altitude_range_recent(MAV_ODOM_RANGE_MAX_AGE_MS);
    const float sig_h = ekf_altitude_uncertainty_m();
    pose_cov[k_cov_diag[2]] = z_ok ? sig_h * sig_h : MAV_COV_UNKNOWN;
    pose_cov[k_cov_diag[3]] = MAV_ODOM_ATT_VAR;
    pose_cov[k_cov_diag[4]] = MAV_ODOM_ATT_VAR;
    pose_cov[k_cov_diag[5]] = MAV_COV_UNKNOWN;       /* yaw: chờ kiểm chứng từ kế (10.6a) */

    /*
     * --- twist: vx, vy, vz (thân), rollspeed, pitchspeed, yawspeed ---
     * Phương sai giữ nguyên theo trục của bộ lọc (ngang / đứng), không xoay theo
     * thái độ: lúc bay nghiêng vài độ, phần trộn giữa hai trục nhỏ hơn nhiều so
     * với độ tự tin thái quá của chính các con số này.
     */
    const float sig_vz = ekf_altitude_climb_uncertainty_mps();
    vel_cov[k_cov_diag[0]] = vel_ok ? sig_v * sig_v : MAV_COV_UNKNOWN;
    vel_cov[k_cov_diag[1]] = vel_ok ? sig_v * sig_v : MAV_COV_UNKNOWN;
    vel_cov[k_cov_diag[2]] = z_ok ? sig_vz * sig_vz : MAV_COV_UNKNOWN;
    vel_cov[k_cov_diag[3]] = MAV_COV_UNKNOWN;        /* tốc độ góc: chưa có số đo sai số */
    vel_cov[k_cov_diag[4]] = MAV_COV_UNKNOWN;
    vel_cov[k_cov_diag[5]] = MAV_COV_UNKNOWN;

    /*
     * vn/ve không hợp lệ (mất flow, vd nằm đất < est_flow_min_height_m) thì bộ lọc vận tốc chỉ
     * còn tích phân gia tốc và trôi tới vài m/s. Xoay nguyên vector sang hệ thân thì phần trôi đó
     * LỌT vào vz (nghiêng 1-2 độ: vz +0,12 m/s khi nằm yên, đo 09-18) dù vz đang báo hợp lệ.
     */
    const vec3f_t v_ned = { vel_ok ? g_fc.est.velocity_mps.x : 0.0f,
                            vel_ok ? g_fc.est.velocity_mps.y : 0.0f,
                            g_fc.est.velocity_mps.z };
    const vec3f_t v_body = ekf_attitude_ned_to_body(v_ned);
    const float q[4] = { g_fc.est.attitude_q.w, g_fc.est.attitude_q.x,
                         g_fc.est.attitude_q.y, g_fc.est.attitude_q.z };

    mavlink_msg_odometry_pack(MAV_SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, &msg,
                              (uint64_t)now_ms * 1000u,
                              MAV_FRAME_LOCAL_NED,
                              MAV_FRAME_BODY_FRD,
                              g_fc.est.position_m.x,
                              g_fc.est.position_m.y,
                              g_fc.est.position_m.z,
                              q,
                              v_body.x, v_body.y, v_body.z,
                              g_fc.est.rate_dps.x * FC_DEG_TO_RAD,
                              g_fc.est.rate_dps.y * FC_DEG_TO_RAD,
                              g_fc.est.rate_dps.z * FC_DEG_TO_RAD,
                              pose_cov,
                              vel_cov,
                              0u,
                              MAV_ESTIMATOR_TYPE_UNKNOWN,
                              vel_ok ? 100 : 0);
    send_msg(&msg);
}

/*
 * --- RC_CHANNELS (65) --- hợp đồng 1.2, GIAO_UOC mục 11.2 "Cách điền RC_CHANNELS".
 *
 * CHỈ ĐỂ HIỂN THỊ. Pi không được suy quyền từ đây — quyền là OB_AUTH (mục 6.3).
 * Chiều ngược lại, RC_CHANNELS_OVERRIDE, FC CỐ Ý không xử lý: tay cầm là lưới
 * an toàn cuối cùng, không để thứ gì trên dây giả được nó.
 */
_Static_assert(RC_CHANNEL_COUNT >= 16, "RC_CHANNELS can du 16 kenh");

/* µs = (raw − 992) × 5/8 + 1500, làm tròn: 172 → 988, 992 → 1500, 1811 → 2012. */
static uint16_t crsf_to_us(uint16_t raw)
{
    const int32_t d5 = ((int32_t)raw - RC_CRSF_CHANNEL_MID) * 5;
    /* Cộng 8000 cho số bị chia luôn dương: phép chia C cắt về 0, không làm tròn xuống. */
    const int32_t us = 1500 + ((d5 + 4 + 8000) / 8) - 1000;
    return (uint16_t)((us < 0) ? 0 : ((us > 65534) ? 65534 : us));
}

static void send_rc_channels(uint32_t now_ms)
{
    mavlink_message_t msg;
    uint16_t us[16];

    for (uint32_t i = 0; i < 16u; i++) {
        us[i] = crsf_to_us(g_fc.rc.channel_raw[i]);
    }

    /* rssi: 0..254 theo link quality, 255 = không biết (đặc tả) — dùng khi mất sóng. */
    const bool rc_ok = g_fc.rc.healthy && !g_fc.rc.failsafe;
    const uint8_t lq = (g_fc.rc.link_quality > 100u) ? 100u : g_fc.rc.link_quality;
    const uint8_t rssi = rc_ok ? (uint8_t)((lq * 254u) / 100u) : UINT8_MAX;

    mavlink_msg_rc_channels_pack(MAV_SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, &msg,
                                 now_ms, 16u,
                                 us[0],  us[1],  us[2],  us[3],
                                 us[4],  us[5],  us[6],  us[7],
                                 us[8],  us[9],  us[10], us[11],
                                 us[12], us[13], us[14], us[15],
                                 UINT16_MAX, UINT16_MAX,
                                 rssi);
    send_msg(&msg);
}

/*
 * --- HIGHRES_IMU (105) ---
 *
 * Chon HIGHRES_IMU chu khong phai SCALED_IMU (26): SCALED_IMU nhet gyro vao
 * int16 don vi mrad/s, ma 2000 dps = 34900 mrad/s — tran kieu ngay trong dai
 * do binh thuong cua ICM20602. HIGHRES_IMU dung float nen khong co bay do, va
 * cho luon ca tu ke lan khi ap trong cung mot khung.
 *
 * MAVROS sinh /mavros/imu/data_raw tu day.
 */
#define MAV_HIGHRES_ACC_GYRO  0x003Fu   /**< bit 0..5    : xacc..zgyro           */
#define MAV_HIGHRES_MAG       0x01C0u   /**< bit 6..8    : xmag..zmag            */
#define MAV_HIGHRES_BARO      0x1A00u   /**< bit 9,11,12 : ap suat, do cao, nhiet */

static void send_highres_imu(uint32_t now_ms)
{
    mavlink_message_t msg;

    /*
     * Chi bat bit tu ke khi no that su dung duoc. Tu ke chua hieu chuan lech
     * huong hang chuc do; bao "co du lieu" luc do te hon la bao "khong co".
     */
    uint16_t fields = MAV_HIGHRES_ACC_GYRO | MAV_HIGHRES_BARO;
    if (g_fc.mag.healthy && g_fc.mag.calibrated) {
        fields |= MAV_HIGHRES_MAG;
    }

    /*
     * Moc thoi gian lay tu mili giay nhan 1000, khong dung micros(): micros()
     * doc TIM2 32 bit nen tran vong sau 71 phut, du de EKF ben Pi sap sai thu
     * tu phep do.
     *
     * gyro_dps da tru bias nhung CHUA qua bo loc — dung nghia "raw" ma
     * /mavros/imu/data_raw can.
     */
    mavlink_msg_highres_imu_pack(MAV_SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, &msg,
                                 (uint64_t)now_ms * 1000u,
                                 g_fc.imu.accel_mps2.x,
                                 g_fc.imu.accel_mps2.y,
                                 g_fc.imu.accel_mps2.z,
                                 g_fc.imu.gyro_dps.x * FC_DEG_TO_RAD,
                                 g_fc.imu.gyro_dps.y * FC_DEG_TO_RAD,
                                 g_fc.imu.gyro_dps.z * FC_DEG_TO_RAD,
                                 g_fc.mag.field_gauss.x,
                                 g_fc.mag.field_gauss.y,
                                 g_fc.mag.field_gauss.z,
                                 g_fc.baro.pressure_pa * 0.01f,   /* Pa -> hPa        */
                                 0.0f,                            /* khong co ong pitot */
                                 g_fc.baro.altitude_m,
                                 g_fc.imu.temperature_c,
                                 fields, 0);
    send_msg(&msg);
}

/*
 * --- BATTERY_STATUS (147) ---
 *
 * Du an do dien ap tong va dong dien, khong do tung cell. Dien ap cell la
 * dien ap tong chia so cell — dien vao dung N o dau cua voltages[] de MAVROS
 * hien dung so cell, cac o con lai de UINT16_MAX ("khong dung").
 */
static void send_battery_status(void)
{
    mavlink_message_t msg;

    uint16_t cells[10];
    uint16_t cells_ext[4] = { 0, 0, 0, 0 };

    const uint16_t cell_mv = (uint16_t)(g_fc.power.cell_voltage_v * 1000.0f);
    const uint8_t  n_cell  = (g_fc.power.cell_count > 10u) ? 10u : g_fc.power.cell_count;

    for (uint8_t i = 0; i < 10u; i++) {
        cells[i] = (i < n_cell) ? cell_mv : UINT16_MAX;
    }

    /* battery_remaining = -1: cung ly do da ghi o send_sys_status(). */
    mavlink_msg_battery_status_pack(MAV_SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, &msg,
                                    0,
                                    MAV_BATTERY_FUNCTION_ALL,
                                    MAV_BATTERY_TYPE_LIPO,
                                    INT16_MAX,      /* khong do nhiet do pin        */
                                    cells,
                                    (int16_t)(g_fc.power.current_a * 100.0f),
                                    (int32_t)g_fc.power.consumed_mah,
                                    -1,             /* khong tinh nang luong        */
                                    -1,             /* phan tram con lai: khong biet */
                                    0,              /* thoi gian con lai: khong biet */
                                    MAV_BATTERY_CHARGE_STATE_UNDEFINED,
                                    cells_ext, 0, 0);
    send_msg(&msg);
}

/*
 * --- EXTENDED_SYS_STATE (245) ---
 *
 * Chua arm thi chac chan dang nam dat. Da arm thi dua vao do cao, nguong
 * 0,5 m — du cao de nhieu baro khong lam nhay trang thai, du thap de bat duoc
 * luc vua roi mat dat.
 */
static void send_extended_sys_state(void)
{
    mavlink_message_t msg;

    uint8_t landed;
    if (!g_fc.motor.armed) {
        landed = MAV_LANDED_STATE_ON_GROUND;
    } else if (!g_fc.est.altitude_valid) {
        landed = MAV_LANDED_STATE_UNDEFINED;
    } else {
        landed = (g_fc.est.altitude_m > 0.5f) ? MAV_LANDED_STATE_IN_AIR
                                              : MAV_LANDED_STATE_ON_GROUND;
    }

    mavlink_msg_extended_sys_state_pack(MAV_SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, &msg,
                                        MAV_VTOL_STATE_UNDEFINED, landed);
    send_msg(&msg);
}

/*
 * --- AUTOPILOT_VERSION (148) ---
 *
 * Tra loi MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES. MAVROS hoi luc ket noi, thu
 * 5 lan roi bo cuoc va in canh bao.
 *
 * Chi khai MAVLINK2 va SET_POSITION_TARGET_LOCAL_NED (tu hop dong 1.4). Khai
 * thua (MISSION_FLOAT, FTP...) se khien MAVROS bat cac plugin ma firmware chua
 * ho tro, roi cho phan hoi khong bao gio toi.
 */
static void send_autopilot_version(void)
{
    mavlink_message_t msg;

    /*
     * capabilities: MAVLINK2 | SET_POSITION_TARGET_LOCAL_NED = 0x2080.
     *
     * 0x0080 khai từ hợp đồng 1.4 (GIAO_UOC 11.1 #13a): giao ước mục 9.5 và
     * 10.3 bước 7 chỉ cho khai cờ SAU KHI nghiệm thu trên dây — Pi đã kiểm dấu
     * 3.4 bốn trục và đo đầu-cuối qua position_controller_node ngày 09-14.
     *
     * flight_sw_version: 4 byte MAJOR.MINOR.PATCH.TYPE theo mã hoá chuẩn
     * MAVLink, TYPE = DEV.
     *
     * flight_custom_version: 8 byte đầu SHA git lúc build, sinh MỖI LẦN build,
     * ghi như một uint64 LITTLE-ENDIAN (hợp đồng 1.2) để MAVROS in đúng chuỗi
     * hash — thứ tự byte do cmake/fc_git_version.cmake quyết định.
     * Build từ cây có thay đổi chưa commit thì hash là của HEAD, KHÔNG đại diện
     * cho bản build — trên dây báo bằng NAMED_VALUE_INT FC_DIRTY (mục 9.5).
     */
    static const uint8_t k_git_hash[8] = FC_GIT_HASH_BYTES;

    const uint32_t sw_version = ((uint32_t)FC_FIRMWARE_VERSION_MAJOR << 24)
                              | ((uint32_t)FC_FIRMWARE_VERSION_MINOR << 16)
                              | ((uint32_t)FC_FIRMWARE_VERSION_PATCH << 8)
                              | (uint32_t)FIRMWARE_VERSION_TYPE_DEV;

    /* Con tro NULL duoc _mav_put_uint8_t_array() dien 0 — dung y do. */
    mavlink_msg_autopilot_version_pack(MAV_SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, &msg,
                                       MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                       MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED,
                                       sw_version, 0, 0, 0,
                                       k_git_hash, NULL, NULL,
                                       0, 0, 0, NULL);
    send_msg(&msg);
}

/* ==========================================================================
 * D#3 — DISTANCE_SENSOR (132). Cách điền: GIAO_UOC mục 11.2.
 * ========================================================================== */

/*
 * min_distance = 15 cm (chốt 09-13).
 *
 * Máy bay nằm trên mặt đất thì laser đọc ~17 cm (17-19 cm, signal_quality 100)
 * — đó là khoảng cách từ laser tới đất theo cách lắp, và máy bay KHÔNG THỂ
 * xuống thấp hơn. Nên laser không bao giờ phải đo dưới ~17 cm: cận dưới thật
 * của MTF01P không còn quan trọng với dự án này.
 *
 * Đặt 15 chứ không 17: MAVROS/ROS coi mẫu dưới min_range là không hợp lệ, và
 * nhiễu 1-2 cm lúc nằm đất sẽ làm mất mẫu đúng lúc cần nhất (chạm đất).
 */
/*
 * 09-18: hạ 15 -> 10. Nằm đất laser đọc 0,10-0,14 m tuỳ chỗ đặt (không chỉ 17-19 cm như 09-13),
 * dưới 15 thì Pi bỏ laser -> mất cả flow camera lẫn vz FC và EKF trôi. 10 cm khớp EST_RANGE_MIN_M:
 * dưới đó là laser bị che (tay che đọc 0,05-0,10 m).
 */
#define MAV_DIST_MIN_CM       10u
#define MAV_DIST_COVARIANCE   25u   /* cm², tức sigma 5 cm = EST_RANGE_NOISE_M */

static void send_distance_sensor(uint32_t now_ms)
{
    mavlink_message_t msg;

    const uint16_t max_cm = (uint16_t)(g_params.flow_range_max_mm / 10u);

    uint32_t cur_cm = (uint32_t)g_fc.flow.range_mm / 10u;
    if (cur_cm > UINT16_MAX) {
        cur_cm = UINT16_MAX;
    }

    /*
     * Cờ hiệu lực nằm ở signal_quality — đặc tả định nghĩa rõ 1 = invalid
     * signal. range_valid đã gom cả "ngoài tầm" lẫn "quá hạn không có mẫu"
     * (mtf01p.c). Khi hợp lệ, quy range_quality 0..255 về 2..100 để không bao
     * giờ rơi vào 0 (không biết) hay 1 (không hợp lệ).
     */
    uint8_t quality;
    if (!g_fc.flow.range_valid) {
        quality = 1u;
        /*
         * Hợp đồng 1.6: MAVROS bỏ qua signal_quality, và driver giữ range_mm là
         * số đo HỢP LỆ CUỐI. Gửi nguyên số đó thì Pi thấy một độ cao đóng băng
         * mà vẫn nằm trong [min, max]. 0 < min_distance nên Pi tự loại.
         */
        cur_cm = 0u;
    } else {
        quality = (uint8_t)(2u + ((uint32_t)g_fc.flow.range_quality * 98u) / 255u);
    }

    mavlink_msg_distance_sensor_pack(MAV_SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, &msg,
                                     now_ms,
                                     (uint16_t)MAV_DIST_MIN_CM,
                                     max_cm,
                                     (uint16_t)cur_cm,
                                     MAV_DISTANCE_SENSOR_LASER,
                                     0u,                              /* id */
                                     MAV_SENSOR_ROTATION_PITCH_270,   /* nhìn xuống */
                                     (uint8_t)MAV_DIST_COVARIANCE,
                                     0.0f, 0.0f,                      /* FOV: không biết */
                                     NULL,                            /* quaternion: chỉ dùng khi CUSTOM */
                                     quality);
    send_msg(&msg);
}

/* ==========================================================================
 * D#1 — Trạng thái OFFBOARD lên dây. GIAO_UOC mục 6.3, 9.3, 9.4, 10.2 R5.
 * ========================================================================== */

/*
 * Các con số dưới đây đi thẳng lên dây và Pi so khớp nguyên văn. Chúng là
 * HỢP ĐỒNG, không phải chi tiết nội bộ: đổi thứ tự enum trong ctrl_offboard.h
 * mà không ai hay là Pi đọc sai lý do rời OFFBOARD. Chặn ngay lúc biên dịch.
 */
_Static_assert(OFFBOARD_EXIT_NONE    == 0, "OB_EXIT lech hop dong");
_Static_assert(OFFBOARD_EXIT_BOOT    == 1, "OB_EXIT lech hop dong");
_Static_assert(OFFBOARD_EXIT_SWITCH  == 2, "OB_EXIT lech hop dong");
_Static_assert(OFFBOARD_EXIT_TIMEOUT == 3, "OB_EXIT lech hop dong");
_Static_assert(OFFBOARD_EXIT_STICK   == 4, "OB_EXIT lech hop dong");
_Static_assert(OFFBOARD_EXIT_DISARM  == 5, "OB_EXIT lech hop dong");
_Static_assert(OFFBOARD_EXIT_CLAMP   == 6, "OB_EXIT lech hop dong");
_Static_assert(OFFBOARD_EXIT_NO_ATT  == 7, "OB_EXIT lech hop dong");
_Static_assert(OFFBOARD_EXIT_NO_RC   == 8, "OB_EXIT lech hop dong");
_Static_assert(OFFBOARD_EXIT_MODE_SW == 9, "OB_EXIT lech hop dong");

/* Bảng bit OB_ARM_BLK — GIAO_UOC mục 9.3. Cùng lý do như trên. */
_Static_assert(ARM_BLOCK_NO_RC            == (1u << 0),  "OB_ARM_BLK lech hop dong");
_Static_assert(ARM_BLOCK_THROTTLE_HIGH    == (1u << 1),  "OB_ARM_BLK lech hop dong");
_Static_assert(ARM_BLOCK_NOT_LEVEL        == (1u << 2),  "OB_ARM_BLK lech hop dong");
_Static_assert(ARM_BLOCK_GYRO_CALIB       == (1u << 3),  "OB_ARM_BLK lech hop dong");
_Static_assert(ARM_BLOCK_SENSOR_FAIL      == (1u << 4),  "OB_ARM_BLK lech hop dong");
_Static_assert(ARM_BLOCK_LOW_BATTERY      == (1u << 5),  "OB_ARM_BLK lech hop dong");
_Static_assert(ARM_BLOCK_FAILSAFE         == (1u << 6),  "OB_ARM_BLK lech hop dong");
_Static_assert(ARM_BLOCK_SWITCH           == (1u << 7),  "OB_ARM_BLK lech hop dong");
_Static_assert(ARM_BLOCK_USB_MSC          == (1u << 8),  "OB_ARM_BLK lech hop dong");
_Static_assert(ARM_BLOCK_THR_NOT_CENTRE   == (1u << 9),  "OB_ARM_BLK lech hop dong");
_Static_assert(ARM_BLOCK_PI_NO_AUTH       == (1u << 10), "OB_ARM_BLK lech hop dong");
_Static_assert(ARM_BLOCK_PI_NOT_POSHOLD   == (1u << 11), "OB_ARM_BLK lech hop dong");
_Static_assert(ARM_BLOCK_PI_WAIT_CH5      == (1u << 12), "OB_ARM_BLK lech hop dong");

static offboard_state_t s_ob_last_state;
static offboard_exit_t  s_ob_last_exit;

/**
 * Gửi một NAMED_VALUE_INT.
 *
 * Trường name đúng 10 byte và KHÔNG bắt buộc kết thúc 0 khi đủ 10 ký tự. Hàm
 * pack chép nguyên 10 byte, nên đưa thẳng chuỗi ngắn hơn vào là đọc lố bộ nhớ
 * sau chuỗi — phải đệm vào mảng đủ 10 byte trước.
 */
static void send_named_int(uint32_t now_ms, const char *name, int32_t value)
{
    mavlink_message_t msg;
    char n[MAVLINK_MSG_NAMED_VALUE_INT_FIELD_NAME_LEN];

    memset(n, 0, sizeof(n));
    size_t len = strlen(name);
    if (len > sizeof(n)) {
        len = sizeof(n);
    }
    memcpy(n, name, len);

    mavlink_msg_named_value_int_pack(MAV_SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, &msg,
                                     now_ms, n, value);
    send_msg(&msg);
}

/*
 * Kênh MÁY ĐỌC, phát định kỳ 2 Hz — kể cả khi chưa arm, ch8 xuống, hay
 * offboard_switch_channel = -1 (giao ước mục 12.A4). Định kỳ chứ không theo sự
 * kiện: node Pi restart lúc nào cũng biết trạng thái trong vòng 0,5 s (R5).
 */
static void send_offboard_state(uint32_t now_ms)
{
    int32_t state;
    switch (ctrl_offboard_state()) {
    case OFFBOARD_ACTIVE:   state = 2; break;
    case OFFBOARD_DISABLED: state = 1; break;
    default:                state = 0; break;   /* KHOA — và mọi giá trị lạ coi như khoá */
    }

    send_named_int(now_ms, "OB_STATE", state);
    send_named_int(now_ms, "OB_AUTH",  ctrl_offboard_pi_has_authority() ? 1 : 0);
    send_named_int(now_ms, "OB_EXIT",  (int32_t)ctrl_offboard_last_exit());
    send_named_int(now_ms, "FC_CTR_VER",
                   (int32_t)(MAV_CONTRACT_MAJOR * 10000 + MAV_CONTRACT_MINOR * 100));

    /*
     * C — firmware có build từ cây chưa commit không (GIAO_UOC mục 9.5). 1 nghĩa
     * là git hash trong AUTOPILOT_VERSION KHÔNG đại diện cho bản đang chạy.
     */
    send_named_int(now_ms, "FC_DIRTY", FC_GIT_DIRTY ? 1 : 0);

    /* A — sẵn sàng arm (GIAO_UOC mục 6.3). Xem arming_link_arm_ready(). */
    send_named_int(now_ms, "OB_ARM_RDY", arming_link_arm_ready() ? 1 : 0);
    send_named_int(now_ms, "OB_ARM_BLK", (int32_t)arming_link_arm_block());

    /* Hợp đồng 1.3 — DISARM thường lúc này có được nhận không (GIAO_UOC 11.1 #12). */
    send_named_int(now_ms, "OB_DIS_RDY", arming_link_disarm_ready() ? 1 : 0);
}

/** Như send_named_int(), cho NAMED_VALUE_FLOAT. Cùng lý do đệm tên 10 byte. */
static void send_named_float(uint32_t now_ms, const char *name, float value)
{
    mavlink_message_t msg;
    char n[MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN];

    memset(n, 0, sizeof(n));
    size_t len = strlen(name);
    if (len > sizeof(n)) {
        len = sizeof(n);
    }
    memcpy(n, name, len);

    mavlink_msg_named_value_float_pack(MAV_SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, &msg,
                                       now_ms, n, value);
    send_msg(&msg);
}

static int32_t clamp_counter(uint32_t v)
{
    return (v > (uint32_t)INT32_MAX) ? INT32_MAX : (int32_t)v;
}

/*
 * B — để Pi TỰ kiểm dấu mục 3.4, không cần console USART1.
 *
 * VÌ SAO LÀ NAMED_VALUE_FLOAT MÀ KHÔNG PHẢI POSITION_TARGET_LOCAL_NED (85):
 *   bản tin vị trí đi qua MAVROS nên bị đổi hệ quy chiếu CẢ HAI CHIỀU — đổi đi
 *   và đổi về triệt tiêu nhau, số Pi đọc lại luôn khớp số Pi gửi kể cả khi
 *   MAVROS đổi sai dấu. NAMED_VALUE_* thì MAVROS chuyển nguyên văn, nên Pi so
 *   được ý định vật lý của mình với nhãn vật lý của FC.
 *
 * Mục tiêu là số SAU KHI kẹp dải, đúng từ vựng của lệnh CLI `offboard`. Phát cả
 * khi chưa nhận setpoint nào (giá trị 0) để Pi phân biệt "FC chưa nhận" với
 * "FC chưa phát" — theo đề nghị của Pi ở mục 3.4.
 *
 * Nhóm này tách khỏi send_offboard_state() và phát lệch pha: gộp 14 khung vào
 * cùng một nhịp là dồn ~420 byte vào đệm TX 1 KB trong một lần.
 */
static void send_offboard_rx(uint32_t now_ms)
{
    const vec3f_t v = ctrl_offboard_velocity_body();

    send_named_int(now_ms, "OB_RX_OK",  clamp_counter(ctrl_offboard_accepted()));
    send_named_int(now_ms, "OB_RX_REJ", clamp_counter(ctrl_offboard_rejected()));
    send_named_int(now_ms, "OB_RX_CLP", clamp_counter(ctrl_offboard_clamped()));

    send_named_float(now_ms, "OB_T_FWD",  v.x);                          /* m/s, + = tới   */
    send_named_float(now_ms, "OB_T_RGT",  v.y);                          /* m/s, + = phải  */
    send_named_float(now_ms, "OB_T_UP",   ctrl_offboard_climb_mps());    /* m/s, + = lên   */
    send_named_float(now_ms, "OB_T_YAWR", ctrl_offboard_yaw_rate_dps()); /* °/s, + = phải  */
}

static void text_append(char *buf, size_t cap, const char *s)
{
    size_t n = strlen(buf);
    while (*s != '\0' && (n + 1u) < cap) {
        buf[n++] = *s++;
    }
    buf[n] = '\0';
}

/*
 * Kênh NGƯỜI ĐỌC: STATUSTEXT đúng một lần mỗi khi chuyển trạng thái.
 *
 * Chỉ để log và màn hình. Không có gì quyết định dựa vào nó: gói có thể rơi
 * khi đệm TX đầy, và node restart thì lỡ mất. Kênh định kỳ ở trên mới là nguồn
 * sự thật.
 */
static void check_offboard_transition(void)
{
    const offboard_state_t st = ctrl_offboard_state();
    const offboard_exit_t  ex = ctrl_offboard_last_exit();

    if (st == s_ob_last_state && ex == s_ob_last_exit) {
        return;
    }
    s_ob_last_state = st;
    s_ob_last_exit  = ex;

    /* Hàm pack chép nguyên 50 byte — đệm đủ 50 và xoá 0 trước. */
    char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1u];
    memset(text, 0, sizeof(text));

    text_append(text, sizeof(text), "OFFBOARD: ");
    text_append(text, sizeof(text), ctrl_offboard_state_name(st));
    if (st != OFFBOARD_ACTIVE) {
        text_append(text, sizeof(text), " (");
        text_append(text, sizeof(text), ctrl_offboard_exit_name(ex));
        text_append(text, sizeof(text), ")");
    }

    /* Giao ước mục 6.3: NOTICE cho DANG_CHAY và TAT, WARNING cho mọi lần KHOA. */
    const uint8_t severity = (st == OFFBOARD_LOCKED) ? MAV_SEVERITY_WARNING
                                                     : MAV_SEVERITY_NOTICE;

    mavlink_message_t msg;
    mavlink_msg_statustext_pack(MAV_SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, &msg,
                                severity, text, 0u, 0u);
    send_msg(&msg);
}

/* ==========================================================================
 * Bảng lịch phát
 *
 * Cùng cách làm với tlm_stream: mỗi dòng một bản tin, quét mỗi vòng lặp, cái
 * nào tới hạn thì phát. Chu kỳ tính bằng mili giây.
 * ========================================================================== */

typedef struct {
    uint16_t period_ms;
    uint32_t next_ms;
    void   (*send)(uint32_t now_ms);
} mav_sched_t;

static void tick_heartbeat  (uint32_t now_ms) { (void)now_ms; send_heartbeat();        }
static void tick_sys_status (uint32_t now_ms) { (void)now_ms; send_sys_status();       }
static void tick_attitude   (uint32_t now_ms) { send_attitude(now_ms);                 }
static void tick_global_pos (uint32_t now_ms) { send_global_position_int(now_ms);      }
static void tick_odometry   (uint32_t now_ms) { send_odometry(now_ms);                 }
static void tick_rc_channels(uint32_t now_ms) { send_rc_channels(now_ms);              }
static void tick_highres_imu(uint32_t now_ms) { send_highres_imu(now_ms);              }
static void tick_battery    (uint32_t now_ms) { (void)now_ms; send_battery_status();   }
static void tick_ext_state  (uint32_t now_ms) { (void)now_ms; send_extended_sys_state(); }
static void tick_offboard   (uint32_t now_ms) { send_offboard_state(now_ms);           }
static void tick_offboard_rx(uint32_t now_ms) { send_offboard_rx(now_ms);              }
static void tick_distance   (uint32_t now_ms) { send_distance_sensor(now_ms);          }

static mav_sched_t s_rates[] = {
    { 1000, 0, tick_heartbeat   },   /*  1 Hz */
    {  500, 0, tick_sys_status  },   /*  2 Hz */
    {   33, 0, tick_attitude    },   /* 30 Hz — 1.5: 50 -> 30, EKF Pi chạy 30 Hz (11.1 #14) */
    { 1000, 0, tick_global_pos  },   /*  1 Hz — 1.5: 10 -> 1, chỉ còn hiển thị GCS */
    {   33, 0, tick_odometry    },   /* 30 Hz — ODOMETRY */
    {   33, 0, tick_highres_imu },   /* 30 Hz — 1.5: 50 -> 30 */
    { 1000, 0, tick_battery     },   /*  1 Hz */
    { 1000, 0, tick_ext_state   },   /*  1 Hz */
    {  500, 0, tick_offboard    },   /*  2 Hz — OB_STATE/AUTH/EXIT, FC_CTR_VER/DIRTY, OB_ARM_RDY/BLK */
    {  500, 0, tick_offboard_rx },   /*  2 Hz — OB_RX_OK/REJ/CLP, OB_T_FWD/RGT/UP/YAWR */
    {   50, 0, tick_distance    },   /* 20 Hz — DISTANCE_SENSOR */
    {  200, 0, tick_rc_channels },   /*  5 Hz — RC_CHANNELS, chỉ hiển thị */
};

#define MAV_SCHED_COUNT  (sizeof(s_rates) / sizeof(s_rates[0]))

/* ==========================================================================
 * Xử lý uplink
 * ========================================================================== */

static void handle_command_long(const mavlink_message_t *msg)
{
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(msg, &cmd);

    /* Không phải nói với ta thì im lặng bỏ qua. 0 nghĩa là phát cho tất cả. */
    if (cmd.target_system != 0 && cmd.target_system != MAV_SYSTEM_ID) {
        return;
    }

    uint8_t result = MAV_RESULT_UNSUPPORTED;

    if (cmd.command == MAV_CMD_COMPONENT_ARM_DISARM) {
        /*
         * --- ARM / DISARM từ máy tính nhúng ---
         *
         * Trước đây ARM luôn bị từ chối: arm phải là chủ ý của người đứng cạnh
         * máy bay. Quy tắc đó VẪN GIỮ, chỉ đổi cách thể hiện — người lái đồng
         * ý bằng cách gạt CẢ ch8 lẫn ch5 lên, đúng thứ tự. Toàn bộ quy trình và
         * điều kiện quyền nằm trong arming.h, file này chỉ dịch kết quả.
         *
         * Ánh xạ kết quả — ba mã khác nhau để phía Pi phân biệt được việc cần
         * làm, thay vì một chữ "thất bại" chung chung:
         *   OK            -> ACCEPTED
         *   NO_AUTHORITY  -> DENIED               ch8/ch5 chưa đúng, hoặc người
         *                                         lái đã giành lái. Pi KHÔNG
         *                                         nên thử lại — phải chờ người.
         *   BLOCKED       -> TEMPORARILY_REJECTED có quyền nhưng chưa đủ điều
         *                                         kiện (ga, cảm biến...). Thử
         *                                         lại sau là hợp lý.
         *
         * KHÔNG dùng "kết quả ? ACCEPTED : FAILED": ARMING_LINK_OK bằng 0, nên
         * phép thử kiểu bool sẽ báo thành công thành thất bại mà trình biên
         * dịch không cảnh báo gì.
         */
        /*
         * DISARM: param2 = 21196 là mã ép của MAVLink — cắt ở mọi độ cao. Không
         * có mã ép thì FC chỉ nhận khi máy bay gần đất (GIAO_UOC 11.1 #12),
         * đang bay trả TEMPORARILY_REJECTED.
         */
        const arming_link_result_t r = (cmd.param1 == 0.0f)
                                     ? arming_request_disarm(cmd.param2 == LINK_DISARM_FORCE_MAGIC)
                                     : arming_request_arm_link();

        switch (r) {
        case ARMING_LINK_OK:           result = MAV_RESULT_ACCEPTED;             break;
        case ARMING_LINK_NO_AUTHORITY: result = MAV_RESULT_DENIED;               break;
        case ARMING_LINK_BLOCKED:      result = MAV_RESULT_TEMPORARILY_REJECTED; break;
        default:                       result = MAV_RESULT_FAILED;               break;
        }
    }
    else if (cmd.command == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES) {
        /*
         * MAVROS hoi cau nay ngay sau khi bat tay. Tra loi truoc roi moi ACK:
         * thu tu nguoc lai van chay duoc nhung lam log ben Pi kho doc.
         */
        send_autopilot_version();
        result = MAV_RESULT_ACCEPTED;
    }
    else if (cmd.command == MAV_CMD_REQUEST_MESSAGE) {
        /*
         * Cách hỏi mới của MAVLink, thay cho lệnh 520. Giao ước mục 5.1 chỉ
         * chốt param1 = 148; mọi ID khác trả UNSUPPORTED (vẫn có ACK — mục 7).
         * So bằng float để khỏi ép kiểu một NaN sang số nguyên.
         */
        if (cmd.param1 == (float)MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
            send_autopilot_version();
            result = MAV_RESULT_ACCEPTED;
        }
    }

    mavlink_message_t ack;
    mavlink_msg_command_ack_pack(MAV_SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, &ack,
                                 cmd.command, result, 0, 0,
                                 msg->sysid, msg->compid);
    send_msg(&ack);
}

/*
 * --- SET_POSITION_TARGET_LOCAL_NED (84) ---
 *
 * Cua vao duy nhat cua duong dieu khien tu may tinh nhung. File nay lam phan
 * kiem tra thuoc ve GIAO THUC (dia chi, he toa do, type_mask); phan thuoc ve
 * DIEU KHIEN (NaN, gioi han bao) do ctrl_offboard lo — xem ctrl_offboard.h.
 *
 * VI SAO TU CHOI CA KHUNG THAY VI LAM NGO TUNG TRUONG:
 *   Khung sai type_mask nghia la hai ben dang hieu hop dong khac nhau. Nhan
 *   bua phan minh doc duoc, bo qua phan con lai, la bay theo mot lenh ma minh
 *   khong hieu. Tu choi thi ctrl_offboard het han roi ve POSHOLD — may bay
 *   phanh lai va treo, va nguyen nhan hien ro trong bo dem 'rejected'.
 */
static void handle_set_position_target(const mavlink_message_t *msg, uint32_t now_ms)
{
    mavlink_set_position_target_local_ned_t sp;
    mavlink_msg_set_position_target_local_ned_decode(msg, &sp);

    if (sp.target_system != 0 && sp.target_system != MAV_SYSTEM_ID) {
        return;                              /* khong noi voi ta, im lang bo */
    }

    /*
     * Giai doan 1 chi nhan he THAN. ctrl_poshold von tinh toan o he than, va
     * sai so AprilTag cung sinh ra o he camera — dung he than bo duoc mot phep
     * xoay o moi dau. He LOCAL_NED can yaw tin cay, mo sau khi tu ke da chung
     * minh duoc (xem App/Docs/GIAO_UOC_FC_ROS2.md muc 5.2 va 10.6a).
     */
    if (sp.coordinate_frame != MAV_FRAME_BODY_NED) {
        ctrl_offboard_note_reject();
        return;
    }

    const uint16_t mask = sp.type_mask;

    /* Phai BO QUA vi tri. Bit nay bang 0 nghia la Pi dang ra lenh vi tri —
     * FC khong co vong vi tri, nhan vao la lai theo so khong kiem chung duoc. */
    const uint16_t need_ignore = POSITION_TARGET_TYPEMASK_X_IGNORE
                               | POSITION_TARGET_TYPEMASK_Y_IGNORE
                               | POSITION_TARGET_TYPEMASK_Z_IGNORE
                               | POSITION_TARGET_TYPEMASK_AX_IGNORE
                               | POSITION_TARGET_TYPEMASK_AY_IGNORE
                               | POSITION_TARGET_TYPEMASK_AZ_IGNORE
                               | POSITION_TARGET_TYPEMASK_YAW_IGNORE;

    /* Phai DUNG van toc. Bit ignore bang 1 nghia la khong co lenh nao ca. */
    const uint16_t need_use = POSITION_TARGET_TYPEMASK_VX_IGNORE
                            | POSITION_TARGET_TYPEMASK_VY_IGNORE
                            | POSITION_TARGET_TYPEMASK_VZ_IGNORE;

    /*
     * Bit 9 (FORCE_SET) KHONG xet — co y. Giao uoc chot type_mask = 0x07C7, ma
     * 0x07C7 BAT bit 9. Truoc 2026-09-13 ham nay loai khung khi bit 9 bat, tuc
     * loai 100% setpoint dung hop dong — `loai` tang, `nhan` dung yen o 0.
     *
     * Bit 9 chi noi "afx/afy/afz mang luc thay vi gia toc". need_ignore da bat
     * buoc bo qua ca ba truong gia toc, nen bit 9 khong con gi de ap vao. Va
     * MAVROS lan PX4 deu hay dat no chung nhom voi cac bit bo qua gia toc.
     * Xem App/Docs/GIAO_UOC_FC_ROS2.md muc 5.2.
     */
    if (((mask & need_ignore) != need_ignore) ||
        ((mask & need_use) != 0)) {
        ctrl_offboard_note_reject();
        return;
    }

    /*
     * yaw_rate duoc phep vang mat: bit 11 bat = Pi khong ra lenh yaw, giu
     * nguyen huong. Day la truong hop hop le, khong phai loi.
     */
    const float yaw_rate =
        ((mask & POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE) != 0) ? 0.0f : sp.yaw_rate;

    (void)ctrl_offboard_set_target(sp.vx, sp.vy, sp.vz, yaw_rate, now_ms);
}

static void handle_message(const mavlink_message_t *msg, uint32_t now_ms)
{
    s_rx_frames++;

    switch (msg->msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT:
        s_link_last_ms = now_ms;
        break;

    case MAVLINK_MSG_ID_COMMAND_LONG:
        handle_command_long(msg);
        break;

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
        handle_set_position_target(msg, now_ms);
        break;

    default:
        /* Mức 1 không quan tâm tới bản tin nào khác. */
        break;
    }
}

static void rx_update(uint32_t now_ms)
{
    uint8_t  buf[64];
    uint16_t n;

    /*
     * Rút hết những gì DMA đã nhận, theo từng khối. Vòng while dừng khi đệm
     * DMA cạn — uplink chỉ là lệnh lác đác nên bình thường nó chạy đúng một
     * lượt rồi thoát.
     */
    while ((n = mav_port_read(buf, sizeof(buf))) > 0) {
        for (uint16_t i = 0; i < n; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &s_rx_msg, &s_rx_status)) {
                handle_message(&s_rx_msg, now_ms);
            }
        }
    }

    /*
     * status.parse_error là bộ đếm 8 BIT của thư viện, nó tràn vòng ở 255.
     * Cộng dồn phần chênh lệch để con số đưa ra ngoài không tụt về sau mỗi
     * 256 lỗi. Phép trừ uint8_t xử lý đúng lúc tràn.
     */
    const uint8_t pe = s_rx_status.parse_error;
    s_rx_errors     += (uint8_t)(pe - s_parse_err_last);
    s_parse_err_last = pe;
}

/* ==========================================================================
 * API
 * ========================================================================== */

void mav_init(void)
{
    mav_port_init();

    s_link_last_ms   = 0;
    s_rx_frames      = 0;
    s_rx_errors      = 0;
    s_parse_err_last = 0;
    s_tx_dropped     = 0;

    memset(&s_rx_status, 0, sizeof(s_rx_status));
    memset(&s_rx_msg, 0, sizeof(s_rx_msg));

    /*
     * Chụp trạng thái OFFBOARD hiện có làm mốc, để lúc khởi động không bắn một
     * STATUSTEXT "chuyển trạng thái" giả. ctrl_angle_init() (khởi tạo
     * ctrl_offboard) chạy trước mav_init() trong main.c.
     */
    s_ob_last_state = ctrl_offboard_state();
    s_ob_last_exit  = ctrl_offboard_last_exit();

    /*
     * Rải điểm phát đầu tiên của 4 bản tin ra 4 mili giây khác nhau. Để chung
     * một mốc thì vòng lặp đầu tiên phải đóng gói cả 4 khung cùng lúc — không
     * nguy hiểm, nhưng tạo một đỉnh thời gian vô cớ, mà ta có ngân sách 125 us
     * mỗi vòng để giữ.
     */
    /*
     * Cách nhau 23 ms chứ không 1 ms: từ hợp đồng 1.2 có hai nhóm 7 khung ở 2 Hz
     * và ODOMETRY ~244 byte. Lệch 1 ms thì các nhóm cùng chu kỳ vẫn dồn vào
     * đệm TX gần như cùng lúc, mà UART chỉ rút được ~92 byte mỗi mili giây.
     */
    const uint32_t t0 = millis();
    for (uint32_t i = 0; i < MAV_SCHED_COUNT; i++) {
        s_rates[i].next_ms = t0 + i * 23u;
    }
}

void mav_update(uint32_t now_ms)
{
    rx_update(now_ms);
    check_offboard_transition();

    for (uint32_t i = 0; i < MAV_SCHED_COUNT; i++) {
        /*
         * So sánh có dấu trên hiệu, không so sánh trực tiếp hai mốc — cách này
         * vẫn đúng khi bộ đếm mili giây tràn vòng sau 49 ngày.
         */
        if ((int32_t)(now_ms - s_rates[i].next_ms) >= 0) {
            s_rates[i].send(now_ms);
            s_rates[i].next_ms = now_ms + s_rates[i].period_ms;
        }
    }

    mav_port_flush();
}

bool mav_link_ok(void)
{
    return (s_link_last_ms != 0) &&
           (fc_elapsed_ms(millis(), s_link_last_ms) < MAV_LINK_TIMEOUT_MS);
}

uint32_t mav_rx_frames(void)  { return s_rx_frames; }
uint32_t mav_rx_errors(void)  { return s_rx_errors; }
uint32_t mav_tx_dropped(void) { return s_tx_dropped + mav_port_dropped(); }
