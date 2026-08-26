/**
 * @file    fc_state.h
 * @brief   Cấu trúc trạng thái trung tâm của hệ thống bay ("blackboard").
 *
 * MÔ HÌNH:
 *   Toàn bộ firmware chia sẻ đúng MỘT thực thể `fc_t g_fc`. Mỗi module chỉ
 *   GHI vào phần của mình và ĐỌC phần của module khác:
 *
 *     Driver cảm biến  --ghi-->  g_fc.imu / .baro / .flow / .rc / .power
 *     Bộ ước lượng     --đọc-->  .imu .baro .flow      --ghi-->  .est
 *     Bộ điều khiển    --đọc-->  .est .rc              --ghi-->  .ctrl
 *     Trộn động cơ     --đọc-->  .ctrl                 --ghi-->  .motor
 *     Telemetry / Log  --đọc-->  tất cả (chỉ đọc)
 *
 * CÁCH MỞ RỘNG (thêm cảm biến mới, ví dụ la bàn hoặc GPS):
 *   1. Định nghĩa struct dữ liệu mới bên dưới, đủ 3 trường bắt buộc:
 *      `timestamp_us`, `healthy`, `error_count`.
 *   2. Thêm một trường vào `fc_t` — đặt ở CUỐI, không chèn vào giữa.
 *   3. Thêm một bit vào `fc_sensor_mask_t`.
 *   4. (tuỳ chọn) Thêm bản tin telemetry trong App/Telemetry/tlm_messages.h.
 *   Các bước trên KHÔNG phá vỡ code đang có vì mọi truy cập đều qua tên trường.
 *
 * AN TOÀN ĐỒNG THỜI:
 *   ISR ưu tiên cao (gyro prio 0, DShot prio 1) ghi vào struct trong khi vòng
 *   lặp chính đọc. Với dữ liệu nhiều trường cần đọc nhất quán, dùng
 *   fc_state_snapshot() để chụp bản sao trong vùng khoá ngắt.
 */
#ifndef FC_STATE_H
#define FC_STATE_H

#include "fc_types.h"
#include "fc_config.h"

/* ========================================================================== */
/*  Trạng thái bay và cờ hệ thống                                             */
/* ========================================================================== */

typedef enum {
    FC_MODE_INIT = 0,       /**< đang khởi tạo phần cứng             */
    FC_MODE_CALIBRATING,    /**< đang lấy bias gyro / mức áp suất    */
    FC_MODE_DISARMED,       /**< sẵn sàng, motor đã khoá             */
    FC_MODE_ARMED,          /**< đang bay, motor hoạt động           */
    FC_MODE_FAILSAFE,       /**< mất RC hoặc lỗi nghiêm trọng        */
    FC_MODE_FAULT,          /**< lỗi không thể phục hồi              */
    FC_MODE_COUNT
} fc_mode_t;

/** Chế độ bay do người dùng chọn qua công tắc trên tay điều khiển. */
typedef enum {
    FLIGHT_MODE_ACRO = 0,   /**< điều khiển tốc độ góc trực tiếp     */
    FLIGHT_MODE_ANGLE,      /**< tự cân bằng theo góc nghiêng        */
    FLIGHT_MODE_ALTHOLD,    /**< giữ độ cao (baro + range)           */
    FLIGHT_MODE_POSHOLD,    /**< giữ vị trí (cần optical flow)       */
    FLIGHT_MODE_COUNT
} flight_mode_t;

/** Bitmask tình trạng cảm biến. Thêm bit mới ở CUỐI, không chèn vào giữa. */
typedef enum {
    SENSOR_GYRO   = (1u << 0),
    SENSOR_ACCEL  = (1u << 1),
    SENSOR_BARO   = (1u << 2),
    SENSOR_FLOW   = (1u << 3),
    SENSOR_RANGE  = (1u << 4),
    SENSOR_RC     = (1u << 5),
    SENSOR_POWER  = (1u << 6),
    SENSOR_SDCARD = (1u << 7),
    /* dành sẵn cho tương lai */
    SENSOR_MAG    = (1u << 8),
    SENSOR_GPS    = (1u << 9),
    SENSOR_IMU2   = (1u << 10),
} fc_sensor_mask_t;

/** Lý do KHÔNG cho phép arm. Bằng 0 nghĩa là được phép arm. */
typedef enum {
    ARM_BLOCK_NONE          = 0,
    ARM_BLOCK_NO_RC         = (1u << 0),
    ARM_BLOCK_THROTTLE_HIGH = (1u << 1),
    ARM_BLOCK_NOT_LEVEL     = (1u << 2),
    ARM_BLOCK_GYRO_CALIB    = (1u << 3),
    ARM_BLOCK_SENSOR_FAIL   = (1u << 4),
    ARM_BLOCK_LOW_BATTERY   = (1u << 5),
    ARM_BLOCK_FAILSAFE      = (1u << 6),
    ARM_BLOCK_SWITCH        = (1u << 7),

    /*
     * Dang o che do doc the qua USB. May tinh dang toan quyen ghi vao tung
     * sector cua the, ke ca bang FAT - cho bay luc nay thi firmware va may
     * tinh ghi de len nhau. Chan tuyet doi, khong co ngoai le.
     */
    ARM_BLOCK_USB_MSC       = (1u << 8),
} fc_arm_block_t;

/** Cờ lỗi hệ thống, tích luỹ dần, chỉ xoá khi khởi động lại. */
typedef enum {
    FC_ERR_IMU_TIMEOUT   = (1u << 0),
    FC_ERR_IMU_SPI       = (1u << 1),
    FC_ERR_BARO_I2C      = (1u << 2),
    FC_ERR_FLOW_TIMEOUT  = (1u << 3),
    FC_ERR_RC_TIMEOUT    = (1u << 4),
    FC_ERR_LOOP_OVERRUN  = (1u << 5),
    FC_ERR_SDCARD        = (1u << 6),
    FC_ERR_DSHOT_DMA     = (1u << 7),
    FC_ERR_LOW_VOLTAGE   = (1u << 8),
    FC_ERR_IMU2_SPI      = (1u << 9),  /* LSM6DSV tren SPI3 */
} fc_error_flag_t;

/* ========================================================================== */
/*  Dữ liệu cảm biến thô / đã hiệu chỉnh                                      */
/* ========================================================================== */

/** ICM20602 qua SPI1, đồng bộ theo chân DRDY trên PC4 (EXTI4). */
typedef struct {
    vec3i16_t gyro_raw;          /**< giá trị ADC thô từ cảm biến     */
    vec3i16_t accel_raw;
    vec3f_t   gyro_dps;          /**< đã trừ bias, đã đổi thang       */
    vec3f_t   accel_mps2;
    vec3f_t   gyro_filtered_dps; /**< sau bộ lọc, dùng cho PID        */
    vec3f_t   gyro_bias_dps;     /**< bias lấy lúc hiệu chuẩn         */
    float     temperature_c;

    uint32_t  timestamp_us;      /**< thời điểm mẫu gần nhất          */
    uint32_t  sample_count;
    uint32_t  dt_us;             /**< khoảng cách giữa 2 mẫu          */
    uint16_t  error_count;
    bool      calibrated;
    bool      healthy;
} imu_data_t;

/** BMP388 qua I2C1. */
typedef struct {
    float    pressure_pa;
    float    temperature_c;
    float    altitude_m;         /**< so với mực nước biển chuẩn      */
    float    altitude_rel_m;     /**< so với mặt đất lúc khởi động    */
    float    ground_pressure_pa; /**< mốc áp suất lúc hiệu chuẩn      */

    uint32_t timestamp_us;
    uint32_t sample_count;
    uint16_t error_count;
    bool     calibrated;
    bool     healthy;
} baro_data_t;

/** MTF01P qua UART4 — gồm optical flow và cảm biến khoảng cách. */
typedef struct {
    int16_t  flow_x_raw;         /**< dịch chuyển thô theo trục X     */
    int16_t  flow_y_raw;
    vec3f_t  velocity_mps;       /**< vận tốc hệ toạ độ thân          */
    uint16_t range_mm;           /**< khoảng cách tới mặt đất         */
    uint8_t  flow_quality;       /**< 0..255, càng cao càng tốt       */
    uint8_t  range_quality;
    bool     range_valid;

    uint32_t timestamp_us;
    uint32_t sample_count;
    uint16_t error_count;
    bool     healthy;
} flow_data_t;

/** Tín hiệu điều khiển từ xa — CRSF/ELRS qua USART2. */
typedef struct {
    uint16_t channel_raw[RC_CHANNEL_COUNT];  /**< 172..1811 kiểu CRSF */

    /* Đã chuẩn hoá: 3 trục -1..+1, ga 0..1 */
    float    roll;
    float    pitch;
    float    yaw;
    float    throttle;

    uint8_t  link_quality;       /**< 0..100 %                        */
    int8_t   rssi_dbm;
    uint8_t  frame_loss_pct;
    uint16_t frame_count;

    uint32_t last_frame_us;
    uint16_t error_count;
    bool     failsafe;
    bool     healthy;
} rc_data_t;

/**
 * QMC6309 — từ kế, đọc gián tiếp qua sensor hub của LSM6DSV trên SPI3.
 *
 * KHÔNG có dây I2C nào ra MCU: LSM6DSV làm I2C master trên chân phụ của nó,
 * tự đọc từ kế rồi để kết quả vào thanh ghi SENSOR_HUB_1..6.
 */
typedef struct {
    vec3i16_t raw;               /**< số thô 16 bit, hệ trục CẢM BIẾN */
    vec3f_t   raw_gauss;         /**< đã đổi thang, hệ CẢM BIẾN, CHƯA hiệu chuẩn
                                  *   — đây là thứ giai đoạn 4 dùng để hiệu chuẩn */
    vec3f_t   field_gauss;       /**< đã hiệu chuẩn và xoay trục, hệ THÂN, Gauss */
    float     magnitude_gauss;   /**< |B| — hằng số nếu hiệu chuẩn đúng */

    uint32_t  timestamp_us;
    uint32_t  sample_count;
    uint16_t  error_count;
    uint8_t   chip_id;           /**< đọc lúc init, phải là 0x90      */
    bool      overflow;          /**< có trục vượt dải đo             */
    bool      calibrated;        /**< đã nạp hiệu chuẩn sắt cứng/mềm  */
    bool      healthy;
} mag_data_t;

/** Đo pin qua ADC1 (PC1 = điện áp, PC0 = dòng điện). */
typedef struct {
    uint16_t vbat_adc_raw;
    uint16_t current_adc_raw;
    float    voltage_v;
    float    current_a;
    float    cell_voltage_v;
    float    consumed_mah;
    uint8_t  cell_count;         /**< tự nhận dạng số cell lúc cắm pin */
    bool     warning;            /**< dưới ngưỡng cảnh báo            */
    bool     critical;           /**< dưới ngưỡng nguy hiểm           */
    uint32_t timestamp_us;
    bool     healthy;
} power_data_t;

/* ========================================================================== */
/*  Kết quả ước lượng trạng thái                                              */
/* ========================================================================== */

typedef struct {
    quatf_t  attitude_q;         /**< quaternion thân so với hệ NED   */
    euler_t  attitude_rad;       /**< dạng Euler cho hiển thị / điều khiển */
    vec3f_t  rate_dps;           /**< tốc độ góc thân, đã lọc         */

    vec3f_t  velocity_mps;       /**< vận tốc hệ NED                  */
    vec3f_t  position_m;         /**< vị trí hệ NED (gốc lúc arm)     */
    float    altitude_m;         /**< độ cao hợp nhất baro + range    */
    float    climb_rate_mps;

    bool     attitude_valid;
    bool     altitude_valid;
    bool     position_valid;
    uint32_t timestamp_us;
} estimator_data_t;

/* ========================================================================== */
/*  Bộ điều khiển                                                             */
/* ========================================================================== */

/**
 * Một khâu PID. Tách riêng hệ số (gains) và trạng thái chạy (state) để có thể
 * nạp / lưu hệ số từ thẻ nhớ mà không đụng tới trạng thái đang chạy.
 */
typedef struct {
    float kp, ki, kd, kff;
    float i_limit;               /**< chặn tích phân (anti-windup)    */
    float out_limit;
} pid_gains_t;

typedef struct {
    pid_gains_t gains;
    float       integral;
    float       prev_measurement;
    float       derivative;
    float       output;
} pid_t;

typedef struct {
    flight_mode_t mode;

    vec3f_t  setpoint_rate_dps;      /**< mục tiêu tốc độ góc         */
    euler_t  setpoint_angle_rad;     /**< mục tiêu góc nghiêng        */
    float    setpoint_altitude_m;
    float    throttle_cmd;           /**< 0..1 trước khi trộn         */

    pid_t    rate_pid[AXIS_COUNT];
    pid_t    angle_pid[AXIS_COUNT];
    pid_t    altitude_pid;

    vec3f_t  pid_output;             /**< -1..+1 cho khâu trộn        */
    uint32_t timestamp_us;
} control_data_t;

/* ========================================================================== */
/*  Đầu ra động cơ                                                            */
/* ========================================================================== */

typedef struct {
    float    output_norm[FC_MOTOR_COUNT];  /**< 0..1 sau khâu trộn    */
    uint16_t throttle[FC_MOTOR_COUNT];     /**< 48..2047 gói DShot    */
    uint32_t erpm[FC_MOTOR_COUNT];         /**< dành cho DShot 2 chiều */
    bool     armed;
    bool     saturated;                    /**< có motor bị chạm trần */
    uint32_t frame_count;
    uint32_t timestamp_us;
} motor_data_t;

/* ========================================================================== */
/*  Sức khoẻ hệ thống                                                         */
/* ========================================================================== */

typedef struct {
    uint32_t uptime_ms;
    uint32_t loop_count;
    uint32_t loop_time_us;       /**< thời gian thực thi vòng lặp vừa rồi */
    uint32_t loop_time_max_us;   /**< đỉnh kể từ lúc khởi động            */
    uint32_t loop_overruns;      /**< số lần vượt quá chu kỳ              */
    uint8_t  cpu_load_pct;

    uint32_t arm_block_flags;    /**< fc_arm_block_t                      */
    uint32_t error_flags;        /**< fc_error_flag_t, tích luỹ           */
    uint32_t sensor_health;      /**< fc_sensor_mask_t, cập nhật liên tục */

    bool     sdcard_mounted;
    bool     logging_active;
    bool     usb_connected;
} system_data_t;

/* ========================================================================== */
/*  Struct tổng                                                               */
/* ========================================================================== */

typedef struct {
    /* --- Cảm biến (module driver ghi) --- */
    imu_data_t       imu;
    baro_data_t      baro;
    flow_data_t      flow;
    rc_data_t        rc;
    power_data_t     power;

    /* --- Xử lý (module thuật toán ghi) --- */
    estimator_data_t est;
    control_data_t   ctrl;
    motor_data_t     motor;

    /* --- Điều phối chung --- */
    system_data_t    sys;
    fc_mode_t        mode;

    /* Thêm module mới ở ĐÂY, không chèn vào giữa để giữ nguyên bố cục
     * bộ nhớ khi debug bằng bản đồ ký hiệu (symbol map). */
    /* IMU phu LSM6DSV tren SPI3. Dat o CUOI theo dung ghi chu tren.
     * KHONG tham gia vong dieu khien - xem App/Docs/KE_HOACH_LSM6DSV.md */
    imu_data_t       imu2;

    /* Tu ke QMC6309 sau sensor hub cua LSM6DSV. */
    mag_data_t       mag;

} fc_t;

/* ========================================================================== */
/*  API                                                                       */
/* ========================================================================== */

/** Thực thể duy nhất của toàn hệ thống. Nằm ở AXI SRAM (DMA truy cập được). */
extern fc_t g_fc;

/** Xoá toàn bộ và nạp giá trị mặc định an toàn. Gọi một lần trước mọi init. */
void fc_state_init(void);

/** Chụp bản sao nhất quán trong vùng khoá ngắt (dùng cho log / telemetry). */
void fc_state_snapshot(fc_t *dst);

/** Cập nhật bitmask sensor_health dựa trên timestamp và cờ healthy. */
void fc_state_update_health(uint32_t now_us);

/** Tính lại arm_block_flags. Trả về true nếu được phép arm. */
bool fc_state_check_arm(void);

/** Đặt / xoá cờ lỗi. */
void fc_state_set_error(fc_error_flag_t flag);
void fc_state_clear_error(fc_error_flag_t flag);

/** Chuyển trạng thái bay có kiểm soát. Trả về false nếu chuyển không hợp lệ. */
bool fc_state_set_mode(fc_mode_t mode);

/** Tên dạng chuỗi, phục vụ hiển thị và gỡ lỗi. */
const char *fc_mode_name(fc_mode_t mode);
const char *fc_flight_mode_name(flight_mode_t mode);

#endif /* FC_STATE_H */
