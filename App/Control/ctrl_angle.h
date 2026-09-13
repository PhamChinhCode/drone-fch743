/**
 * @file    ctrl_angle.h
 * @brief   Vòng ngoài: chọn chế độ bay và sinh setpoint cho vòng tốc độ góc.
 *
 * CẤU TRÚC TẦNG (cascade):
 *
 *   cần điều khiển ─┬─ ACRO  ─────────────────────────┐
 *                   │                                 ├─> setpoint_rate_dps
 *                   └─ ANGLE ─> [P góc] ─> tốc độ ────┘         │
 *                                  ↑                            v
 *                            est.attitude_rad            [PID tốc độ góc]
 *                              (từ EKF)                         │
 *                                                               v
 *                                                          khâu trộn
 *
 *   Vòng ngoài KHÔNG chạm tới motor. Nó chỉ nói với vòng trong "hãy quay với
 *   tốc độ này", còn việc đạt được tốc độ đó là của vòng trong. Nhờ vậy chỉnh
 *   PID vẫn tách bạch: vòng trong chỉnh trước cho chắc, rồi mới chồng vòng
 *   ngoài lên. Vòng trong sai thì vòng ngoài không cứu được.
 *
 * HAI CHẾ ĐỘ:
 *   ACRO  — cần điều khiển ra thẳng TỐC ĐỘ quay. Buông cần thì máy bay giữ
 *           nguyên góc đang nghiêng, không tự về ngang.
 *   ANGLE — cần điều khiển ra GÓC nghiêng. Buông cần thì máy bay tự về ngang.
 *           Dễ bay hơn nhiều, và là chế độ nên dùng cho chuyến đầu tiên.
 *
 *   Trục YAW luôn điều khiển theo TỐC ĐỘ ở cả hai chế độ. Không có la bàn thì
 *   yaw của bộ ước lượng trôi vài độ mỗi phút (xem ekf_attitude.h), giữ hướng
 *   theo con số trôi ấy chỉ tổ làm máy bay từ từ quay đi.
 *
 * DỰ PHÒNG KHI MẤT GÓC:
 *   Bộ ước lượng chưa dựng xong quaternion ban đầu thì không có góc để bám.
 *   Lúc đó vòng này tự lùi về ACRO thay vì bám một con số vô nghĩa, và báo ra
 *   console. Chuyện này chỉ xảy ra trong vài phần giây đầu sau khi bật nguồn.
 */
#ifndef CTRL_ANGLE_H
#define CTRL_ANGLE_H

#include "fc_types.h"
#include "fc_config.h"
#include "fc_state.h"   /* flight_mode_t */

void ctrl_angle_init(void);

/**
 * Đọc cần điều khiển và công tắc chế độ, ghi g_fc.ctrl.setpoint_rate_dps,
 * g_fc.ctrl.setpoint_angle_rad và g_fc.ctrl.throttle_cmd.
 *
 * Tự giữ nhịp FC_ATTITUDE_RATE_HZ. Gọi trong vòng lặp chính, NGAY TRƯỚC
 * ctrl_rate_update() để vòng trong luôn thấy setpoint mới nhất.
 *
 * @param now_us  mốc thời gian micro giây từ micros()
 * @return true nếu vừa chạy một bước
 */
bool ctrl_angle_update(uint32_t now_us);

/** Chế độ đang thực sự chạy (có thể khác cần gạt nếu phải lùi về ACRO). */
flight_mode_t ctrl_angle_active_mode(void);

/**
 * Chế độ mà CÔNG TẮC ch6 đang chọn — không phải chế độ đang chạy.
 *
 * Hai thứ khác nhau: đang OFFBOARD thì chế độ chạy là OFFBOARD còn công tắc
 * vẫn chỉ POSHOLD; mất flow thì chế độ chạy tụt về ANGLE còn công tắc vẫn chỉ
 * POSHOLD. OFFBOARD và arming.c cần câu hỏi thứ hai: "người lái đã đặt đường
 * lùi an toàn chưa" (GIAO_UOC_FC_ROS2.md mục 6.3).
 */
flight_mode_t ctrl_angle_switch_mode(void);

/** true khi công tắc đòi ANGLE nhưng phải lùi về ACRO vì chưa có góc. */
bool ctrl_angle_fallback(void);

#endif /* CTRL_ANGLE_H */
