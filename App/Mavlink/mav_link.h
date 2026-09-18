/**
 * @file    mav_link.h
 * @brief   Lớp bản tin MAVLink v2 — đóng gói downlink, phân tích uplink.
 *
 * MỤC ĐÍCH: nối bộ điều khiển bay với máy tính nhúng chạy ROS2 (MAVROS hoặc
 * node tự viết) qua UART8, phục vụ bài toán bay gắp hàng.
 *
 * HỢP ĐỒNG: App/Docs/GIAO_UOC_FC_ROS2.md là nguồn sự thật DUY NHẤT cho mọi thứ
 * đi qua UART8 (bảng phát mục 4, bảng nhận mục 5, sổ đăng ký mục 9). File này
 * chỉ tóm tắt; khi lệch nhau, giao ước đúng. Phiên bản hiện thực:
 * MAV_CONTRACT_MAJOR.MAV_CONTRACT_MINOR bên dưới.
 *
 * PHÁT (s_rates trong mav_link.c):
 *   (hợp đồng 1.5) HEARTBEAT 1 Hz, SYS_STATUS 2 Hz, ATTITUDE 30 Hz,
 *   HIGHRES_IMU 30 Hz, ODOMETRY 30 Hz, GLOBAL_POSITION_INT 1 Hz,
 *   BATTERY_STATUS 1 Hz, EXTENDED_SYS_STATE 1 Hz, DISTANCE_SENSOR 20 Hz,
 *   RC_CHANNELS 5 Hz, NAMED_VALUE_INT 11 tên + NAMED_VALUE_FLOAT 4 tên 2 Hz.
 *   Theo sự kiện: COMMAND_ACK, AUTOPILOT_VERSION, STATUSTEXT (chuyển trạng
 *   thái OFFBOARD). LOCAL_POSITION_NED và VFR_HUD ngừng từ 1.5.
 *
 * NHẬN:
 *   HEARTBEAT                      — theo dõi đường truyền còn sống
 *   SET_POSITION_TARGET_LOCAL_NED  — setpoint vận tốc, xem ctrl_offboard.h
 *   COMMAND_LONG ARM_DISARM (400)  — chỉ ở chế độ Pi và khi Pi còn quyền, arming.h
 *   COMMAND_LONG 520, 512 (p1=148) — trả AUTOPILOT_VERSION
 *   Mọi bản tin khác bị bỏ qua. Mọi COMMAND_LONG đều được trả COMMAND_ACK, kể
 *   cả lệnh không hỗ trợ — thiếu ACK thì service bên MAVROS treo tới hết giờ.
 *
 * ĐỊNH DANH:
 *   system_id = 1, component_id = MAV_COMP_ID_AUTOPILOT1. Đặt fcu_url của
 *   MAVROS là /dev/ttyXXX:921600 và tgt_system = 1.
 */
#ifndef MAV_LINK_H
#define MAV_LINK_H

#include "fc_types.h"
#include "fc_config.h"

/** Định danh MAVLink của bộ điều khiển bay này. */
#define MAV_SYSTEM_ID     1

/**
 * Phiên bản hợp đồng FC <-> Pi mà firmware này hiện thực
 * (App/Docs/GIAO_UOC_FC_ROS2.md, mục 10.1). Phát lên dây qua NAMED_VALUE_INT
 * `FC_CTR_VER` = MAJOR*10000 + MINOR*100.
 *
 * Tăng theo ĐÚNG quy tắc mục 10.1 của giao ước, và CÙNG LÚC với bảng lịch sử
 * phiên bản trong tài liệu đó. Lệch hai chỗ này là Pi đọc sai hợp đồng.
 */
#define MAV_CONTRACT_MAJOR  1
#define MAV_CONTRACT_MINOR  7

/** Quá thời gian này không nhận được HEARTBEAT thì coi như mất máy tính nhúng. */
#define MAV_LINK_TIMEOUT_MS  3000u

/** Khởi tạo cổng và bảng lịch phát. Gọi sau MX_UART8_Init() và fc_time_init(). */
void mav_init(void);

/**
 * Rút uplink, phát downlink tới hạn, kích DMA.
 * Gọi mỗi vòng lặp chính, SAU khi g_fc đã cập nhật xong trong nhịp đó.
 * @param now_ms  mốc mili giây, lấy từ HAL_GetTick()
 */
void mav_update(uint32_t now_ms);

/* --- Theo dõi sức khoẻ đường truyền, phục vụ console và gỡ lỗi ----------- */

/** Có nhận được HEARTBEAT trong MAV_LINK_TIMEOUT_MS gần đây hay không. */
bool     mav_link_ok(void);

/** Số khung MAVLink hợp lệ đã nhận. */
uint32_t mav_rx_frames(void);

/** Số byte bị parser loại (sai CRC, sai độ dài, rác đường truyền). */
uint32_t mav_rx_errors(void);

/** Số khung downlink bị bỏ vì đệm gửi đầy. */
uint32_t mav_tx_dropped(void);

#endif /* MAV_LINK_H */
