/**
 * @file    ctrl_poshold.h
 * @brief   Giữ vận tốc bằng optical flow — vòng ngoài cùng của chế độ POSHOLD.
 *
 * VỊ TRÍ TRONG CHUỖI:
 *
 *   cần ─> [vận tốc mong muốn] ─> P+I ─> góc nghiêng ─> [vòng góc] ─> [vòng
 *                    ↑                                  tốc độ góc] ─> trộn
 *              est.velocity_mps
 *              (flow + gia tốc kế)
 *
 *   Vòng này KHÔNG chạm tới motor, cũng không chạm tới tốc độ góc. Nó chỉ
 *   thay thế phần "cần điều khiển ra góc nghiêng" của chế độ ANGLE bằng
 *   "sai số vận tốc ra góc nghiêng". Mọi tầng bên dưới giữ nguyên.
 *
 * NGUYÊN LÝ: máy bay đa cánh quạt chỉ đi ngang được bằng cách NGHIÊNG. Nghiêng
 * góc θ cho gia tốc ngang a = g·tan(θ). Nên muốn hãm vận tốc trôi, cứ nghiêng
 * ngược chiều trôi một góc tỉ lệ với sai số.
 *
 * LÀM VIỆC TRONG HỆ THÂN:
 *   Bộ ước lượng cho vận tốc hệ NED, nhưng cần điều khiển và góc nghiêng đều
 *   là khái niệm hệ THÂN. Vòng này xoay vận tốc về hệ thân bằng yaw rồi tính
 *   toàn bộ ở đó — đỡ phải xoay ngược lại ở đầu ra, và dễ đọc hơn nhiều.
 *
 * KHÂU I LÀ ĐỂ CHỐNG GIÓ:
 *   Gió thổi đều là nhiễu loạn không đổi. Chỉ có P thì máy bay đứng ở một độ
 *   nghiêng cân bằng nhưng VẪN TRÔI đều — đúng bài toán đã gặp ở trục yaw.
 *   Khâu I tích luỹ tới khi tự nó giữ đủ độ nghiêng, lúc đó vận tốc mới về 0.
 *
 * DỰ PHÒNG:
 *   Mất flow (bay quá cao, mặt sàn trơn không kết cấu, laser ngoài tầm) thì
 *   không còn gì đo vận tốc. Vòng này báo không dùng được, và ctrl_angle tự
 *   lùi về ANGLE. Bám một vận tốc không đo được là cách chắc chắn nhất để
 *   máy bay lao đi mất.
 *
 * CHƯA LÀM — GIỮ VỊ TRÍ THẬT:
 *   Đây là giữ VẬN TỐC, chưa phải giữ VỊ TRÍ. Máy bay sẽ ngừng trôi, nhưng
 *   nếu bị đẩy lệch một mét thì nó đứng yên ở chỗ mới chứ không quay lại.
 *   Giữ vị trí cần tích phân vận tốc thành vị trí rồi thêm một vòng P nữa —
 *   làm sau, khi tầng này đã chứng minh chạy tốt.
 */
#ifndef CTRL_POSHOLD_H
#define CTRL_POSHOLD_H

#include "fc_types.h"
#include "fc_config.h"

void ctrl_poshold_init(void);

/** Xoá khâu tích phân. Gọi khi disarm hoặc khi rời chế độ POSHOLD. */
void ctrl_poshold_reset(void);

/**
 * Tính góc nghiêng mục tiêu từ sai số vận tốc.
 *
 * @param dt_s        bước thời gian, giây
 * @param roll_rad    [ra] góc nghiêng roll mong muốn
 * @param pitch_rad   [ra] góc nghiêng pitch mong muốn
 * @return false nếu chưa có ước lượng vận tốc tin cậy — khi đó KHÔNG ghi gì
 *         vào hai tham số ra, và bên gọi phải lùi về chế độ ANGLE.
 */
bool ctrl_poshold_update(float dt_s, float *roll_rad, float *pitch_rad);

/* --- Số liệu phục vụ chỉnh và chẩn đoán -------------------------------- */

/** Vận tốc hệ THÂN đang đo được (x = tới, y = phải), m/s. */
vec3f_t ctrl_poshold_velocity_body(void);

/** Vận tốc mong muốn hệ thân, do cần điều khiển đặt ra. */
vec3f_t ctrl_poshold_target_body(void);

/** Phần đóng góp của khâu I, độ. */
/** Mốc vị trí đang giữ, hệ NED. Chỉ có nghĩa khi ctrl_poshold_position_locked(). */
vec3f_t ctrl_poshold_target_ned(void);

/** true khi đang GIỮ CHỖ; false khi người lái đang cầm lái hoặc mất flow. */
bool ctrl_poshold_position_locked(void);

vec3f_t ctrl_poshold_integral_deg(void);

#endif /* CTRL_POSHOLD_H */
