/**
 * @file    mav_link.h
 * @brief   Lớp bản tin MAVLink v2 — đóng gói downlink, phân tích uplink.
 *
 * MỤC ĐÍCH: nối bộ điều khiển bay với máy tính nhúng chạy ROS2 (MAVROS hoặc
 * node tự viết) qua UART8, phục vụ bài toán bay gắp hàng.
 *
 * ĐÂY LÀ MỨC 1 — CHỈ GIÁM SÁT VÀ CẮT KHẨN CẤP.
 *   Máy tính nhúng ĐỌC được trạng thái máy bay và RA LỆNH DISARM. Nó KHÔNG
 *   điều khiển được chuyến bay: chưa có OFFBOARD, chưa nhận setpoint vị trí,
 *   chưa nhận pose từ VIO. Bay vẫn hoàn toàn do người lái trên tay điều khiển.
 *
 * BẢNG PHÁT (xem s_rates trong mav_link.c):
 *   HEARTBEAT    1 Hz   — bắt buộc, MAVROS dựa vào đây để nhận diện
 *   SYS_STATUS   2 Hz   — pin, tình trạng cảm biến
 *   ATTITUDE    50 Hz   — góc và tốc độ góc
 *   VFR_HUD     10 Hz   — độ cao, tốc độ, hướng, ga
 *
 *   Tổng khoảng 2,6 KB/s. Ở 921600 baud (92 KB/s) là 3% băng thông.
 *
 * NHẬN:
 *   HEARTBEAT     — theo dõi đường truyền còn sống
 *   COMMAND_LONG  — chỉ MAV_CMD_COMPONENT_ARM_DISARM, và CHỈ chiều DISARM.
 *                   Lệnh ARM luôn bị từ chối, xem lý do ở mav_link.c.
 *   Mọi bản tin khác bị bỏ qua trong im lặng.
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
