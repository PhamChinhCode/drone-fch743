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
    TLM_MSG_PARAM_VALUE = 0x0D, /**< mô tả + giá trị một tham số      */
    TLM_MSG_ACK       = 0x0E,   /**< trả lời cho một lệnh uplink      */
    TLM_MSG_FC_INFO   = 0x0F,   /**< nhận dạng mạch bay, chữ ký bảng  */
    TLM_MSG_CLI_LINE  = 0x10,   /**< một dòng chữ trả lời lệnh CLI    */
    TLM_MSG_VIB       = 0x11,   /**< gyro+accel CHƯA LỌC 8 kHz, xem vib_stream.h */

    /* --- Máy tính gửi lên (uplink) --- */
    TLM_MSG_CMD_SET_RATE  = 0x40,  /**< bật/tắt và đặt chu kỳ 1 luồng */
    TLM_MSG_CMD_SET_PID   = 0x41,  /**< nạp hệ số PID                 */
    TLM_MSG_CMD_ACTION    = 0x42,  /**< hiệu chuẩn, lưu, khởi động lại */
    TLM_MSG_CMD_PARAM_REQ_LIST = 0x43, /**< xin toàn bộ bảng tham số  */
    TLM_MSG_CMD_PARAM_READ     = 0x44, /**< xin lại một tham số        */
    TLM_MSG_CMD_PARAM_SET      = 0x45, /**< đặt một tham số            */
    TLM_MSG_CMD_MOTOR_TEST     = 0x46, /**< quay thử motor (có canh chừng) */
    TLM_MSG_CMD_CLI            = 0x47, /**< gửi một dòng lệnh CLI      */
    TLM_MSG_CMD_FC_INFO_REQ    = 0x48, /**< xin bản tin nhận dạng      */

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
    uint16_t reserved;          /**< tần số notch gyro đang dùng, Hz (0 = tắt) */
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
 * Phiên bản giao thức
 *
 * Tăng số này khi đổi bố cục hoặc ý nghĩa của BẤT KỲ struct nào ở trên. App
 * PC so nó ngay lúc bắt tay và TỪ CHỐI kết nối nếu lệch — thà không kết nối
 * còn hơn đọc sai offset rồi hiển thị số vô nghĩa một cách tự tin.
 * ========================================================================== */
#define TLM_PROTOCOL_VERSION  2u

/**
 * Độ dài tên tham số trên đường truyền. PHẢI bằng PARAM_NAME_MAX trong
 * param_table.h — param_msg.c có _Static_assert kiểm điều đó lúc biên dịch.
 *
 * Nhân đôi hằng số ở đây là có chủ ý: file này được chép nguyên sang phía máy
 * tính, mà bên đó không có param_table.h.
 */
#define TLM_PARAM_NAME_MAX  28

/**
 * 0x0D — mô tả đầy đủ một tham số. (50 byte)
 *
 * ĐÂY LÀ BẢN TIN LÀM NÊN PHẦN MỀM CẤU HÌNH: app PC không cần biên dịch sẵn
 * danh sách tham số nào cả, nó hỏi và firmware tự khai báo tên, kiểu, giới
 * hạn, giá trị mặc định. Nạp firmware có thêm tham số thì app hiện được ngay,
 * không phải cập nhật theo.
 *
 * `count` lặp lại trong mọi gói để app dựng được thanh tiến độ ngay từ gói
 * đầu tiên, và biết chính xác còn thiếu bao nhiêu nếu có gói rơi.
 */
typedef struct __attribute__((packed)) {
    uint16_t index;
    uint16_t count;                    /**< tổng số tham số            */
    uint8_t  type;                     /**< param_type_t               */
    uint8_t  flags;                    /**< PARAM_FLAG_*               */
    char     name[TLM_PARAM_NAME_MAX]; /**< có '\0' nếu còn chỗ        */
    float    value;
    float    min;
    float    max;
    float    def;                      /**< để app tô đậm chỗ đã đổi   */
} tlm_param_value_t;

/** Mã kết quả trong TLM_MSG_ACK. */
typedef enum {
    TLM_ACK_OK = 0,
    TLM_ACK_ERR_UNKNOWN,    /**< không hiểu mã lệnh                  */
    TLM_ACK_ERR_LENGTH,     /**< payload ngắn hơn struct             */
    TLM_ACK_ERR_RANGE,      /**< index hoặc giá trị ngoài phạm vi    */
    TLM_ACK_ERR_ARMED,      /**< từ chối vì đang bay                 */
    TLM_ACK_ERR_READONLY,
    TLM_ACK_ERR_FAILED,     /**< thao tác thất bại (ví dụ ghi flash) */
    TLM_ACK_ERR_BUSY
} tlm_ack_result_t;

/**
 * 0x0E — trả lời cho một lệnh uplink. (4 byte)
 *
 * MỌI lệnh uplink đều được trả lời, kể cả khi thành công. Không có ACK thì
 * app phải đoán bằng cách chờ hết giờ, và "lệnh bị bỏ" trông y hệt "lệnh
 * chạy xong nhưng không đổi gì".
 */
typedef struct __attribute__((packed)) {
    uint8_t  cmd_id;        /**< mã lệnh đang được trả lời           */
    uint8_t  result;        /**< tlm_ack_result_t                    */
    uint16_t detail;        /**< tuỳ lệnh: chỉ số tham số, mã lỗi... */
} tlm_ack_t;

/** Bit trong tlm_fc_info_t.capabilities. */
#define TLM_CAP_PARAMS      (1u << 0)
#define TLM_CAP_CLI         (1u << 1)
#define TLM_CAP_MOTOR_TEST  (1u << 2)
#define TLM_CAP_BLACKBOX    (1u << 3)
#define TLM_CAP_IMU2        (1u << 4)
#define TLM_CAP_MAG         (1u << 5)

/**
 * 0x0F — nhận dạng mạch bay. (32 byte)
 *
 * Gói đầu tiên app xin sau khi mở cổng. `protocol_version` lệch thì app từ
 * chối kết nối; `param_table_crc` lệch so với lần trước thì app biết phải
 * đọc lại toàn bộ bảng chứ không dùng bản đã nhớ.
 */
typedef struct __attribute__((packed)) {
    char     board[16];
    uint8_t  fw_major;
    uint8_t  fw_minor;
    uint8_t  fw_patch;
    uint8_t  protocol_version;
    uint16_t param_count;
    uint16_t reserved;
    uint32_t param_table_crc;
    uint32_t capabilities;
} tlm_fc_info_t;

/** Bit trong tlm_cli_line_t.flags. */
#define TLM_CLI_FLAG_LAST   (1u << 0)
#define TLM_CLI_FLAG_TRUNC  (1u << 1)  /**< dòng dài hơn text[], đã bị cắt */

/**
 * 0x10 — một dòng chữ trả lời lệnh CLI. Độ dài thay đổi.
 *
 * Cùng bộ mã sinh ra những dòng này với console chữ trên USART1, nên khi app
 * cư xử lạ thì gõ tay đúng lệnh đó vào PuTTY là biết lỗi ở firmware hay ở app.
 *
 * VÌ SAO text[] TO ĐẾN THẾ — và vì sao không to hơn được nữa:
 *
 *   `flash dump` in mỗi bản ghi log thành một dòng CSV 34 cột. Dòng dài nhất
 *   log_record_to_csv() sinh được là 241 ký tự (mọi trường kịch biên int16).
 *   Với text[60] của bản giao thức 1 thì dòng bị cắt ÂM THẦM — dữ liệu ra sai
 *   mà không có dấu hiệu nào. Đó là lý do bản 2 tồn tại.
 *
 *   Trần cứng là 255: byte LEN trong khung chỉ có một byte (tlm_protocol.h).
 *   Nên 245 là gần hết cỡ, và LOG_RECORD_CSV_MAX (320) KHÔNG chứa vừa. Thêm
 *   cột vào bản ghi log tới mức vượt 245 thì phải đi đường khác, không nới
 *   thêm được. TLM_CLI_FLAG_TRUNC có mặt để lúc đó app biết mà kêu, thay vì
 *   lặp lại đúng lỗi âm thầm cũ.
 */
typedef struct __attribute__((packed)) {
    uint16_t seq;           /**< số thứ tự dòng trong một lần trả lời */
    uint8_t  flags;         /**< TLM_CLI_FLAG_*                       */
    char     text[245];     /**< không cần ký tự kết thúc chuỗi       */
} tlm_cli_line_t;

/**
 * 0x11 — một cụm mẫu IMU liên tiếp ở nhịp gốc 8 kHz, CHƯA QUA BỘ LỌC nào.
 * Dùng để phân tích phổ rung. Chỉ phát qua USB CDC, xem vib_stream.h.
 *
 * Hệ trục THÂN (đã xoay theo imu_axis_map/sign), gyro đã trừ bias.
 *
 * Mất mẫu: idx0 của khung sau phải bằng idx0 + n của khung trước. Hở mà
 * `ring_drops` không tăng thì mất ở phía máy tính/USB; `ring_drops` tăng thì
 * firmware tự bỏ vì USB không rút kịp.
 */
#define TLM_VIB_SAMPLES_PER_FRAME  16u
#define TLM_VIB_GYRO_LSB_PER_DPS   16.0f    /**< ±2048 °/s                */
#define TLM_VIB_ACCEL_LSB_PER_G    2000.0f  /**< ±16,4 g                  */

typedef struct __attribute__((packed)) {
    int16_t gyro[3];            /**< °/s × TLM_VIB_GYRO_LSB_PER_DPS   */
    int16_t accel[3];           /**< g × TLM_VIB_ACCEL_LSB_PER_G      */
} tlm_vib_sample_t;

typedef struct __attribute__((packed)) {
    uint32_t idx0;              /**< số thứ tự mẫu đầu tiên trong khung */
    uint32_t t0_us;             /**< micros() lúc đọc mẫu đầu tiên     */
    uint32_t ring_drops;        /**< tổng số mẫu firmware đã bỏ        */
    uint16_t motor[4];          /**< giá trị DShot lúc đóng khung      */
    uint8_t  n;                 /**< số mẫu hợp lệ trong samples[]     */
    uint8_t  flags;             /**< bit0 = đang ARM                   */
    tlm_vib_sample_t samples[TLM_VIB_SAMPLES_PER_FRAME];
} tlm_vib_t;

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

/** 0x44 — xin lại một tham số (dùng khi phát hiện gói rơi). */
typedef struct __attribute__((packed)) {
    uint16_t index;
} tlm_cmd_param_read_t;

/**
 * 0x45 — đặt một tham số.
 *
 * `type` KHÔNG thừa: nó được đối chiếu với bảng trước khi ghi. App dùng bản
 * bảng đã nhớ từ phiên trước mà firmware đã đổi thì chỉ số sẽ trỏ sang tham
 * số khác — kiểm kiểu bắt được phần lớn trường hợp đó trước khi ghi bừa.
 *
 * Firmware LUÔN trả về một TLM_MSG_PARAM_VALUE của chính tham số vừa ghi,
 * nên app hiển thị con số firmware THỰC SỰ nhận chứ không phải con số đã gõ.
 */
typedef struct __attribute__((packed)) {
    uint16_t index;
    uint8_t  type;              /**< param_type_t, để đối chiếu       */
    uint8_t  reserved;
    float    value;
} tlm_cmd_param_set_t;

/**
 * 0x46 — quay thử motor.
 *
 * MỘT motor mỗi lần, KHÔNG phải bitmask. Driver dshot bảo đảm "chỉ một motor
 * quay tại một thời điểm, ba cái còn lại nhận lệnh 0" và đó là một trong bốn
 * chốt an toàn của nó — cho phép chọn nhiều motor cùng lúc là gỡ mất chốt đó.
 *
 * `timeout_ms` là CANH CHỪNG: firmware tự dừng sau ngần ấy thời gian (trần
 * cứng DSHOT_TEST_MAX_MS). App gửi lại đều đặn — khoảng 200 ms một lần với
 * timeout 500 ms — suốt lúc người dùng còn giữ thanh trượt.
 *
 * Nhờ vậy rút cáp, treo app, hay mất ESP-NOW đều dẫn tới motor dừng, không
 * cần app còn sống để dọn dẹp. Đây là điều kiện AN TOÀN, không phải tối ưu.
 */
typedef struct __attribute__((packed)) {
    uint8_t  motor;             /**< 0..3; TLM_MOTOR_TEST_STOP = dừng ngay */
    uint8_t  throttle_pct;      /**< bị kẹp bởi DSHOT_TEST_MAX_PERCENT     */
    uint16_t timeout_ms;        /**< tự dừng sau ngần này                  */
} tlm_cmd_motor_test_t;

#define TLM_MOTOR_TEST_STOP  0xFFu

/** 0x47 — một dòng lệnh CLI. Độ dài thay đổi, không cần '\0'. */
typedef struct __attribute__((packed)) {
    char text[60];
} tlm_cmd_cli_t;

#endif /* TLM_MESSAGES_H */
