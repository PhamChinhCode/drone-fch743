/**
 * @file    ekf_attitude.h
 * @brief   Ước lượng góc nghiêng bằng EKF dạng sai số (Error-State KF).
 *
 * VÌ SAO DÙNG DẠNG SAI SỐ:
 *   Quaternion có 4 số nhưng chỉ mô tả 3 bậc tự do, nên ràng buộc |q| = 1
 *   khiến ma trận hiệp phương sai của nó luôn suy biến. Nhồi thẳng quaternion
 *   vào vector trạng thái EKF sẽ làm P mất hạng và sớm muộn cũng vỡ số học.
 *
 *   Cách chuẩn là tách đôi:
 *     - Trạng thái danh nghĩa: quaternion q, tích phân thẳng từ gyro, KHÔNG
 *       nằm trong bộ lọc.
 *     - Trạng thái sai số: góc lệch nhỏ δθ (3 số, không ràng buộc) cộng với
 *       bias gyro (3 số). Đây mới là thứ EKF ước lượng.
 *   Sau mỗi lần cập nhật, δθ được "tiêm" vào q rồi xoá về 0. Nhờ vậy P luôn
 *   đủ hạng và góc lệch luôn nhỏ nên xấp xỉ tuyến tính mới hợp lệ.
 *
 * TRẠNG THÁI (6):
 *     x = [ δθx δθy δθz | bgx bgy bgz ]
 *   δθ tính bằng radian, bias gyro bằng radian/giây.
 *
 * DỰ BÁO:  gyro, chạy ở EST_RATE_HZ.
 * CẬP NHẬT: accel — khi máy bay không tăng tốc, accel chỉ đúng hướng trọng
 *   lực, và độ lệch giữa hướng đo được với hướng do q dự đoán chính là δθ.
 *
 * YAW KHÔNG QUAN SÁT ĐƯỢC:
 *   Ma trận H của phép cập nhật accel là [g]× — hạng 2, nhân không gian nằm
 *   dọc chính vector trọng lực. Nói cách khác accel không hề biết gì về hướng
 *   mũi máy bay. Đây là tính chất vật lý, không phải khuyết điểm hiện thực:
 *   muốn chặn yaw trôi thì phải có la bàn hoặc GPS.
 *
 *   Hệ quả kéo theo: bias gyro trục Z cũng không quan sát được, nên phương
 *   sai của nó lớn dần vô hạn nếu để tự nhiên. Driver chặn cả giá trị bias
 *   (EST_GYRO_BIAS_MAX_DPS) lẫn phương sai của nó.
 *
 *   Vì vậy attitude_rad.yaw ở đây là "hướng so với lúc khởi động", trôi
 *   khoảng vài độ mỗi phút. Dùng được cho chế độ ACRO và ANGLE; KHÔNG dùng
 *   được để giữ hướng lâu dài.
 *
 * QUY ƯỚC:
 *   q xoay từ hệ THÂN sang hệ NED. Thân: X mũi trước, Y cánh phải, Z xuống.
 *   Lúc nằm yên thăng bằng, accel đọc ra (0, 0, +9,81) — xem EST_ACCEL_Z_SIGN
 *   trong fc_config.h nếu mạch của bạn cho dấu ngược.
 */
#ifndef EKF_ATTITUDE_H
#define EKF_ATTITUDE_H

#include "fc_types.h"
#include "fc_config.h"

/** Số trạng thái sai số: 3 góc lệch + 3 bias gyro. */
#define EKF_ATT_STATES  6

/**
 * Đặt bộ lọc về trạng thái chưa khởi tạo. Lần gọi ekf_attitude_update() đầu
 * tiên có accel hợp lệ sẽ tự dựng quaternion ban đầu từ hướng trọng lực.
 */
void ekf_attitude_init(void);

/**
 * Chạy một bước: dự báo bằng gyro rồi cập nhật bằng accel.
 *
 * @param gyro_dps    tốc độ góc thân, độ/giây (đã trừ bias tĩnh của driver)
 * @param accel_mps2  gia tốc thân, m/s²
 * @param dt_s        khoảng thời gian kể từ lần gọi trước, giây
 */
void ekf_attitude_update(vec3f_t gyro_dps, vec3f_t accel_mps2, float dt_s);

/** Quaternion hiện tại (thân -> NED). */
quatf_t ekf_attitude_quaternion(void);

/** Góc Euler theo quy ước hàng không ZYX, radian. */
euler_t ekf_attitude_euler(void);

/** Bias gyro bộ lọc đang ước lượng, độ/giây. */
vec3f_t ekf_attitude_gyro_bias_dps(void);

/**
 * Thành phần thứ ba của hàng cuối ma trận xoay, tức cosin góc nghiêng tổng.
 * Bằng 1 khi nằm phẳng, bằng 0 khi dựng đứng 90°.
 * Dùng để chiếu số đo laser xuống phương thẳng đứng.
 */
float ekf_attitude_tilt_cos(void);

/** Xoay một vector từ hệ thân sang hệ NED. */
vec3f_t ekf_attitude_body_to_ned(vec3f_t v_body);

/** true khi bộ lọc đã dựng xong quaternion ban đầu. */
bool ekf_attitude_is_valid(void);

/**
 * Độ lệch góc trung bình bộ lọc còn đang sửa, độ. Lấy từ đường chéo của P.
 * Tụt dần về gần 0 sau vài giây là dấu hiệu bộ lọc đã hội tụ.
 */
float ekf_attitude_uncertainty_deg(void);

#endif /* EKF_ATTITUDE_H */
