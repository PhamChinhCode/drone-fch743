/**
 * @file    tlm_messages.h
 * @brief   Định nghĩa payload của từng bản tin telemetry.
 *
 * NGUYÊN TẮC THIẾT KẾ:
 *   - Dùng SỐ NGUYÊN CÓ THANG ĐO thay vì float. Gói nhỏ hơn một nửa, và
 *     đường telemetry (ELRS ~10 kB/s) không kham nổi float cho mọi trường.
 *     Thang đo ghi ngay cạnh mỗi trường, ví dụ `_cdeg` = 1/100 độ.
 *   - Mọi struct đều `packed` để bố cục byte giống hệt nhau ở mọi trình biên
 *     dịch. Bên máy tính chỉ cần khai báo struct tương ứng là đọc được.
 *   - Little-endian (khớp với Cortex-M và x86, không cần đảo byte).
 *
 * CÁCH THÊM BẢN TIN MỚI:
 *   1. Cấp một mã ID mới ở CUỐI danh sách tlm_msg_id_t (không dùng lại mã cũ).
 *   2. Khai báo struct payload `packed` ở đây.
 *   3. Viết hàm đóng gói trong tlm_stream.c và thêm một dòng vào bảng
 *      `g_tlm_streams[]`. Không cần sửa gì khác.
 */
#ifndef TLM_MESSAGES_H
#define TLM_MESSAGES_H

#include "fc_types.h"

/* ==========================================================================
 * Mã bản tin
 * ========================================================================== */

typedef enum {
    /* --- Bay xuống máy tính (downlink) --- */
    TLM_MSG_HEARTBEAT = 0x01,   /**< nhịp sống, trạng thái tổng quát  */
    TLM_MSG_ATTITUDE  = 0x02,   /**< góc nghiêng + tốc độ góc         */
    TLM_MSG_IMU       = 0x03,   /**< gyro + accel đã hiệu chỉnh       */
    TLM_MSG_BARO      = 0x04,   /**< áp suất + độ cao                 */
    TLM_MSG_FLOW      = 0x05,   /**< optical flow + khoảng cách       */
    TLM_MSG_RC        = 0x06,   /**< kênh điều khiển + chất lượng sóng */
    TLM_MSG_MOTOR     = 0x07,   /**< đầu ra 4 động cơ                 */
    TLM_MSG_POWER     = 0x08,   /**< điện áp, dòng, dung lượng        */
    TLM_MSG_PID       = 0x09,   /**< setpoint và đầu ra PID (tuning)  */
    TLM_MSG_SYSTEM    = 0x0A,   /**< thời gian vòng lặp, tải CPU      */
    TLM_MSG_TEXT      = 0x0B,   /**< thông báo dạng chữ               */
    TLM_MSG_LINK      = 0x0C,   /**< chất lượng đường ESP-NOW         */

    /* --- Máy tính gửi lên (uplink) --- */
    TLM_MSG_CMD_SET_RATE  = 0x40,  /**< bật/tắt và đặt chu kỳ 1 luồng */
    TLM_MSG_CMD_SET_PID   = 0x41,  /**< nạp hệ số PID                 */
    TLM_MSG_CMD_ACTION    = 0x42,  /**< hiệu chuẩn, lưu, khởi động lại */

    TLM_MSG_ID_MAX = 0xFF
} tlm_msg_id_t;

/* ==========================================================================
 * Payload — downlink
 * ========================================================================== */

/** 0x01 — nhịp sống. Bản tin quan trọng nhất, luôn bật. (16 byte) */
typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;
    uint32_t error_flags;       /**< fc_error_flag_t                  */
    uint16_t sensor_health;     /**< fc_sensor_mask_t                 */
    uint16_t arm_block_flags;   /**< fc_arm_block_t                   */
    uint8_t  mode;              /**< fc_mode_t                        */
    uint8_t  flight_mode;       /**< flight_mode_t                    */
    uint8_t  cpu_load_pct;
    uint8_t  reserved;
} tlm_heartbeat_t;

/** 0x02 — tư thế bay. Luồng hay dùng nhất khi tinh chỉnh. (12 byte) */
typedef struct __attribute__((packed)) {
    int16_t roll_cdeg;          /**< 1/100 độ                         */
    int16_t pitch_cdeg;
    int16_t yaw_cdeg;
    int16_t rate_roll_ddps;     /**< 1/10 độ/giây                     */
    int16_t rate_pitch_ddps;
    int16_t rate_yaw_ddps;
} tlm_attitude_t;

/** 0x03 — số liệu IMU đã hiệu chỉnh. (16 byte) */
typedef struct __attribute__((packed)) {
    int16_t gyro_ddps[3];       /**< 1/10 độ/giây                     */
    int16_t accel_mg[3];        /**< mili-g (1/1000 g)                */
    int16_t temperature_cdeg;   /**< 1/100 độ C                       */
    uint16_t error_count;
} tlm_imu_t;

/** 0x04 — khí áp kế. (14 byte) */
typedef struct __attribute__((packed)) {
    int32_t  pressure_pa;
    int32_t  altitude_cm;       /**< so với mặt đất, đơn vị cm        */
    int16_t  climb_rate_cms;    /**< cm/giây                          */
    int16_t  temperature_cdeg;
    uint16_t error_count;
} tlm_baro_t;

/** 0x05 — optical flow và cảm biến khoảng cách. (12 byte) */
typedef struct __attribute__((packed)) {
    int16_t  flow_x_raw;
    int16_t  flow_y_raw;
    int16_t  velocity_x_cms;    /**< cm/giây, hệ toạ độ thân          */
    int16_t  velocity_y_cms;
    uint16_t range_mm;
    uint8_t  flow_quality;
    uint8_t  flags;             /**< bit0 = range_valid, bit1 = healthy */
} tlm_flow_t;

/** 0x06 — điều khiển từ xa. Chỉ gửi 8 kênh đầu cho gọn. (20 byte) */
typedef struct __attribute__((packed)) {
    uint16_t channel[8];        /**< giá trị thô CRSF 172..1811       */
    uint8_t  link_quality;      /**< 0..100 %                         */
    int8_t   rssi_dbm;
    uint8_t  frame_loss_pct;
    uint8_t  flags;             /**< bit0 = failsafe, bit1 = healthy  */
} tlm_rc_t;

/** 0x07 — đầu ra động cơ. (17 byte) */
typedef struct __attribute__((packed)) {
    uint16_t throttle[4];       /**< giá trị DShot 0..2047            */
    uint16_t erpm[4];           /**< eRPM/100, 0 nếu chưa có telemetry */
    uint8_t  flags;             /**< bit0 = armed, bit1 = saturated   */
} tlm_motor_t;

/** 0x08 — nguồn điện. (10 byte) */
typedef struct __attribute__((packed)) {
    uint16_t voltage_mv;
    uint16_t current_ca;        /**< 1/100 Ampe                       */
    uint16_t consumed_mah;
    uint16_t cell_voltage_mv;
    uint8_t  cell_count;
    uint8_t  flags;             /**< bit0 = warning, bit1 = critical  */
} tlm_power_t;

/** 0x09 — dữ liệu tinh chỉnh PID. Chỉ bật khi đang chỉnh. (20 byte) */
typedef struct __attribute__((packed)) {
    int16_t setpoint_ddps[3];   /**< mục tiêu tốc độ góc, 1/10 dps    */
    int16_t measured_ddps[3];   /**< giá trị đo được                  */
    int16_t output_permille[3]; /**< đầu ra PID, 1/1000 (-1000..1000) */
    uint16_t reserved;
} tlm_pid_t;

/** 0x0A — hiệu năng hệ thống. (16 byte) */
typedef struct __attribute__((packed)) {
    uint32_t loop_count;
    uint32_t loop_overruns;
    uint16_t loop_time_us;
    uint16_t loop_time_max_us;
    uint16_t imu_dt_us;
    uint8_t  cpu_load_pct;
    uint8_t  flags;             /**< bit0 = sd_mounted, bit1 = logging,
                                     bit2 = usb_connected             */
} tlm_system_t;

/** 0x0B — thông báo dạng chữ. Độ dài thay đổi, tối đa 60 ký tự. */
typedef struct __attribute__((packed)) {
    uint8_t severity;           /**< 0=info 1=warn 2=error            */
    char    text[60];           /**< không cần ký tự kết thúc chuỗi   */
} tlm_text_t;

/**
 * 0x0C — chất lượng đường ESP-NOW. (16 byte)
 *
 * KHÁC MỌI BẢN TIN CÒN LẠI: firmware bay KHÔNG sinh ra bản tin này. Trạm mặt
 * đất (ESP32 cắm ở máy tính) tự dựng nó rồi chèn vào luồng, vì chỉ đầu nhận
 * mới biết được có bao nhiêu gói đã rơi trên đường và RSSI bằng bao nhiêu.
 * Nhờ đi chung một khung nên phần mềm PC chỉ cần một bộ giải mã duy nhất.
 */
typedef struct __attribute__((packed)) {
    uint32_t packets_rx;        /**< tổng gói ESP-NOW trạm nhận được  */
    uint32_t packets_lost;      /**< suy ra từ chỗ đứt số thứ tự      */
    uint32_t air_frames_drop;   /**< khung ESP32 trên máy bay đã bỏ   */
    uint32_t air_crc_bad;       /**< khung hỏng CRC ngay trên dây UART */
    uint16_t bytes_per_sec;     /**< lưu lượng thực tế đo trong 1 s   */
    uint16_t gap_ms;            /**< khoảng lặng dài nhất trong 1 s   */
    int8_t   rssi_dbm;          /**< 0 nếu SDK không cung cấp         */
    uint8_t  loss_pct;          /**< 0..100, tính trên 1 giây gần nhất */
    uint8_t  flags;             /**< bit0 = đã ghép đôi (unicast),
                                     bit1 = có số liệu từ phía máy bay */
    uint8_t  air_q_peak;        /**< hàng đợi sâu nhất phía máy bay   */
} tlm_link_t;                   /* 24 byte */

/* ==========================================================================
 * Payload — uplink (máy tính điều khiển firmware)
 * ========================================================================== */

/** 0x40 — bật/tắt hoặc đổi chu kỳ phát của một luồng. */
typedef struct __attribute__((packed)) {
    uint8_t  msg_id;            /**< luồng cần chỉnh                  */
    uint16_t period_ms;         /**< 0 = tắt luồng                    */
} tlm_cmd_set_rate_t;

/** 0x41 — nạp hệ số PID cho một trục. */
typedef struct __attribute__((packed)) {
    uint8_t axis;               /**< fc_axis_t                        */
    uint8_t loop;               /**< 0 = rate, 1 = angle              */
    float   kp, ki, kd, kff;
} tlm_cmd_set_pid_t;

/** Mã lệnh dùng cho TLM_MSG_CMD_ACTION. */
typedef enum {
    TLM_ACTION_CALIB_GYRO  = 1,
    TLM_ACTION_CALIB_ACCEL = 2,
    TLM_ACTION_CALIB_BARO  = 3,
    TLM_ACTION_SAVE_CONFIG = 4,
    TLM_ACTION_REBOOT      = 5,
    TLM_ACTION_LOG_START   = 6,
    TLM_ACTION_LOG_STOP    = 7,
} tlm_action_t;

/** 0x42 — lệnh không tham số. */
typedef struct __attribute__((packed)) {
    uint8_t  action;            /**< tlm_action_t                     */
    uint32_t argument;          /**< tuỳ lệnh, thường bằng 0          */
} tlm_cmd_action_t;

#endif /* TLM_MESSAGES_H */
