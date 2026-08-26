/**
 * @file    ekf_velocity.h
 * @brief   Ước lượng vận tốc ngang, hợp nhất optical flow với gia tốc kế.
 *
 * TRẠNG THÁI — hai bộ lọc 2 trạng thái độc lập, một cho mỗi trục ngang NED:
 *     x = [ v  b ]
 *   v = vận tốc theo trục đó, m/s
 *   b = độ lệch (bias) của gia tốc theo trục đó, m/s²
 *
 *   Tách hai trục là có chủ ý: ghép chéo giữa vận tốc Bắc và vận tốc Đông gần
 *   như bằng 0, gộp lại thành bộ 4 trạng thái chỉ tốn gấp đôi phép tính để mô
 *   tả những tương quan không tồn tại.
 *
 * BÙ QUAY — phần dễ sai nhất, và sai thì hỏng cả hệ:
 *
 *   Cảm biến flow đo dịch chuyển của ẢNH, mà ảnh dịch vì HAI nguyên nhân:
 *   máy bay tịnh tiến (thứ ta cần) và máy bay quay (thứ phải loại bỏ).
 *   Nghiêng tại chỗ cũng làm cả ảnh trượt đi y như đang bay ngang.
 *
 *   Suy từ hình học, với thân X trước / Y phải / Z xuống và camera nhìn theo +Z:
 *       flow_x = +vy/h - gyro_x
 *       flow_y = -vx/h - gyro_y
 *   nên
 *       vy =  h · (flow_x + gyro_x)
 *       vx = -h · (flow_y + gyro_y)
 *
 *   DẤU CỘNG, không phải trừ. Lăn phải làm điểm mặt đất dịch về +Y trong hệ
 *   thân, còn bay phải làm nó lùi về -Y — hai chiều ngược nhau, nên trong một
 *   tín hiệu flow duy nhất chúng mang dấu trái ngược.
 *
 *   Bỏ qua phép trừ gyro thì nghiêng 10° trong 0,2 giây ở độ cao 1 m sẽ bị
 *   hiểu nhầm thành 1,5 m/s vận tốc. Bộ giữ vị trí sẽ chống lại vận tốc ảo đó
 *   bằng cách nghiêng THÊM — tức tự kích, máy bay lắc mạnh dần rồi lật.
 *
 * ĐỘ CAO LẤY TỪ ĐÂU:
 *   Từ tia laser ToF, KHÔNG phải từ EKF độ cao. Flow nhìn mặt sàn ngay bên
 *   dưới, nên thứ cần là khoảng cách tới đúng mặt sàn đó. Baro đo so với mức
 *   áp suất lúc cất cánh — bay qua cái bàn thì hai số khác hẳn nhau, mà flow
 *   thì thấy mặt bàn.
 *
 * HỆ QUY CHIẾU:
 *   Vận tốc ước lượng trong hệ NED, xoay từ hệ thân bằng ma trận của bộ ước
 *   lượng góc. Yaw trôi (không có la bàn) nên hệ NED này quay chậm theo thời
 *   gian. Giữ vị trí vẫn hoạt động, nhưng điểm neo sẽ trôi dần — đó là giới
 *   hạn của bộ cảm biến, không phải của hiện thực.
 */
#ifndef EKF_VELOCITY_H
#define EKF_VELOCITY_H

#include "fc_types.h"
#include "fc_config.h"

void ekf_velocity_init(void);

/**
 * Bước dự báo, chạy mỗi nhịp bộ lọc.
 * @param accel_ned_xy  gia tốc ngang hệ NED, đã trừ trọng lực, m/s²
 * @param dt_s          bước thời gian, giây
 */
void ekf_velocity_predict(float accel_n, float accel_e, float dt_s);

/**
 * Cập nhật bằng một mẫu optical flow.
 *
 * Hàm tự làm toàn bộ phần bù quay và quy đổi, và tự từ chối khi điều kiện
 * ngoài cửa sổ dùng được (quá thấp, quá cao, nghiêng quá, chất lượng kém).
 *
 * @param flow_x_rad   góc dịch chuyển thô trục X kể từ mẫu trước, radian
 * @param flow_y_rad   góc dịch chuyển thô trục Y
 * @param gyro_dps     tốc độ góc thân cùng thời điểm, độ/giây
 * @param dt_s         khoảng thời gian giữa hai mẫu flow
 * @param range_m      khoảng cách laser đo được dọc trục thân
 * @param tilt_cos     cosin góc nghiêng tổng
 * @param quality      chất lượng flow 0..255
 * @return true nếu số đo được dùng
 */
bool ekf_velocity_update_flow(float flow_x_rad, float flow_y_rad,
                              vec3f_t gyro_dps, float dt_s,
                              float range_m, float tilt_cos,
                              uint8_t quality);

float ekf_velocity_north(void);
float ekf_velocity_east(void);

/** Vận tốc hệ THÂN suy từ mẫu flow gần nhất — dùng để kiểm chứng bằng mắt. */
vec3f_t ekf_velocity_body_measured(void);

float ekf_velocity_uncertainty_mps(void);

/**
 * Số liệu thô của lần cập nhật gần nhất, rad/s: tốc độ góc flow đo được và
 * tốc độ góc gyro cùng thời điểm. Dùng để kiểm phần bù quay có triệt tiêu
 * đúng không — khi nghiêng tại chỗ, wx phải xấp xỉ -gx và wy xấp xỉ -gy.
 */
void ekf_velocity_debug_rates(float *wx, float *wy, float *gx, float *gy);

/** true khi đã có ít nhất một mẫu flow hợp lệ gần đây. */
bool ekf_velocity_is_valid(void);

/** Số mẫu flow đã dùng và đã bị từ chối. */
uint32_t ekf_velocity_accepted(void);
uint32_t ekf_velocity_rejected(void);

#endif /* EKF_VELOCITY_H */
