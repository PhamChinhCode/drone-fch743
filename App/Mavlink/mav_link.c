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
#include "main.h"

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
    const uint32_t health  = mav_sensor_bits(g_fc.sys.sensor_health);

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

static void send_vfr_hud(void)
{
    mavlink_message_t msg;

    const float vx = g_fc.est.velocity_mps.x;
    const float vy = g_fc.est.velocity_mps.y;
    const float ground_speed = sqrtf(vx * vx + vy * vy);

    /* heading: 0..359 độ theo quy ước la bàn. yaw của bộ ước lượng là -pi..+pi. */
    float heading_deg = g_fc.est.attitude_rad.yaw * FC_RAD_TO_DEG;
    if (heading_deg < 0.0f) {
        heading_deg += 360.0f;
    }

    mavlink_msg_vfr_hud_pack(MAV_SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, &msg,
                             0.0f,                     /* không có ống pitot */
                             ground_speed,
                             (int16_t)heading_deg,
                             (uint16_t)(g_fc.ctrl.throttle_cmd * 100.0f),
                             g_fc.est.altitude_m,
                             g_fc.est.climb_rate_mps);
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

static void tick_heartbeat (uint32_t now_ms) { (void)now_ms; send_heartbeat();  }
static void tick_sys_status(uint32_t now_ms) { (void)now_ms; send_sys_status(); }
static void tick_attitude  (uint32_t now_ms) { send_attitude(now_ms);           }
static void tick_vfr_hud   (uint32_t now_ms) { (void)now_ms; send_vfr_hud();    }

static mav_sched_t s_rates[] = {
    { 1000, 0, tick_heartbeat  },   /*  1 Hz */
    {  500, 0, tick_sys_status },   /*  2 Hz */
    {   20, 0, tick_attitude   },   /* 50 Hz */
    {  100, 0, tick_vfr_hud    },   /* 10 Hz */
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
        if (cmd.param1 == 0.0f) {
            /* --- DISARM: luôn cho phép --- */
            result = arming_request_disarm() ? MAV_RESULT_ACCEPTED
                                             : MAV_RESULT_FAILED;
        } else {
            /*
             * --- ARM: TỪ CHỐI, và đây là chủ ý ---
             *
             * Toàn bộ thiết kế của arming.c đặt trên một quy tắc: hành động
             * arm phải là chủ ý của người đang đứng cạnh máy bay, thể hiện
             * bằng cách gạt công tắc trên tay điều khiển. Trạng thái LOCKED
             * tồn tại chỉ để bảo vệ quy tắc đó — xem ba tình huống nguy hiểm
             * liệt kê trong arming.h.
             *
             * Cho arm qua đường này là mở đúng cái cửa ấy: một node ROS2 lỗi,
             * hoặc chỉ đơn giản là khởi động lại đúng lúc, sẽ làm cánh quạt
             * quay khi có người đang cầm máy bay.
             *
             * Mức 2 (OFFBOARD) sẽ cần arm từ xa thật. Khi đó nó phải đi kèm
             * một công tắc cho phép trên tay điều khiển, chứ không phải bằng
             * cách gỡ dòng này.
             */
            result = MAV_RESULT_TEMPORARILY_REJECTED;
        }
    }

    mavlink_message_t ack;
    mavlink_msg_command_ack_pack(MAV_SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, &ack,
                                 cmd.command, result, 0, 0,
                                 msg->sysid, msg->compid);
    send_msg(&ack);
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
     * Rải điểm phát đầu tiên của 4 bản tin ra 4 mili giây khác nhau. Để chung
     * một mốc thì vòng lặp đầu tiên phải đóng gói cả 4 khung cùng lúc — không
     * nguy hiểm, nhưng tạo một đỉnh thời gian vô cớ, mà ta có ngân sách 125 us
     * mỗi vòng để giữ.
     */
    const uint32_t t0 = millis();
    for (uint32_t i = 0; i < MAV_SCHED_COUNT; i++) {
        s_rates[i].next_ms = t0 + i;
    }
}

void mav_update(uint32_t now_ms)
{
    rx_update(now_ms);

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
