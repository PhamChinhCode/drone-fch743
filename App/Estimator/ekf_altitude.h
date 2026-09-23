/**
 * @file    ekf_altitude.h
 * @brief   Ước lượng độ cao bằng EKF 3 trạng thái, hợp nhất accel + baro + laser.
 *
 * TRẠNG THÁI:
 *     x = [ h  v  b ]
 *   h = độ cao so với mặt đất lúc hiệu chuẩn, mét, DƯƠNG LÀ LÊN
 *   v = tốc độ lên, m/s
 *   b = độ lệch (bias) của gia tốc thẳng đứng, m/s²
 *
 * VÌ SAO CẦN TRẠNG THÁI BIAS:
 *   Gia tốc thẳng đứng suy ra từ accel sau khi trừ trọng lực. Chỉ cần accel
 *   lệch 0,1 m/s² — chuyện thường tình khi nhiệt độ thay đổi — thì tích phân
 *   hai lần trong 10 giây đã cho sai số 5 mét. Đưa bias vào làm trạng thái để
 *   baro và laser tự dò ra nó, thay vì để sai số tích luỹ vô hạn.
 *
 * VÌ SAO HỢP NHẤT CẢ BA:
 *   accel  nhanh, mượt, nhưng trôi — cho biết ĐỔI như thế nào
 *   baro   không trôi, đo được mọi độ cao, nhưng ồn và nhạy với gió lùa
 *   laser  chính xác cỡ centimet nhưng chỉ tới ~8 m và cần mặt đất phẳng
 *   Mỗi cái bù đúng nhược điểm của cái kia. Bộ lọc tự cân theo độ tin cậy
 *   khai báo trong fc_config.h chứ không chuyển cứng giữa các nguồn.
 *
 * GIỚI HẠN — ĐỊA HÌNH KHÔNG PHẲNG:
 *   baro đo độ cao so với MỨC ÁP SUẤT lúc cất cánh, còn laser đo khoảng cách
 *   tới MẶT ĐẤT NGAY BÊN DƯỚI. Bay qua cái bàn hay bờ tường thì hai số này
 *   mâu thuẫn nhau và bộ lọc sẽ giằng co. Cách sửa đúng là thêm trạng thái
 *   thứ tư mô tả cao độ địa hình; ở đây chưa làm, nên hãy chọn ngưỡng
 *   EST_RANGE_MAX_M thấp và bay trên nền phẳng khi thử.
 */
#ifndef EKF_ALTITUDE_H
#define EKF_ALTITUDE_H

#include "fc_types.h"
#include "fc_config.h"

void ekf_altitude_init(void);

/**
 * Bước dự báo, chạy mỗi nhịp bộ lọc.
 * @param accel_up_mps2  gia tốc thẳng đứng hệ NED, đã trừ trọng lực, dương là lên
 * @param dt_s           bước thời gian, giây
 */
void ekf_altitude_predict(float accel_up_mps2, float dt_s);

/**
 * Cập nhật bằng độ cao khí áp. Gọi khi có mẫu baro mới (khoảng 50 Hz).
 * @param altitude_m  g_fc.baro.altitude_rel_m
 */
void ekf_altitude_update_baro(float altitude_m);

/**
 * Cập nhật bằng cảm biến khoảng cách laser.
 * Hàm tự chiếu số đo nghiêng xuống phương thẳng đứng và tự bỏ qua khi máy bay
 * nghiêng quá EST_RANGE_MAX_TILT_DEG (có trễ) hoặc đo ngoài [EST_RANGE_MIN_M, EST_RANGE_MAX_M] (dưới là bị che).
 * Bỏ mẫu lệch dự đoán quá EST_RANGE_GATE_SIGMA; sau EST_RANGE_REANCHOR_MS không
 * dùng được laser thì mẫu kế tiếp neo lại độ cao, không kéo theo tốc độ lên.
 *
 * @param range_m    khoảng cách đo được dọc trục thân, mét
 * @param tilt_cos   cosin góc nghiêng, lấy từ ekf_attitude_tilt_cos()
 * @return true nếu số đo được dùng
 */
bool ekf_altitude_update_range(float range_m, float tilt_cos);

float ekf_altitude_m(void);
float ekf_altitude_climb_rate_mps(void);
float ekf_altitude_accel_bias(void);

/** Sai số chuẩn của độ cao, mét. Lấy từ đường chéo P. */
float ekf_altitude_uncertainty_m(void);

/** Độ không tin cậy 1-sigma của tốc độ lên/xuống, m/s — bộ lọc tự chấm, như trên. */
float ekf_altitude_climb_uncertainty_mps(void);

/** true khi đã có ít nhất một lần cập nhật từ cảm biến tuyệt đối. */
bool ekf_altitude_is_valid(void);

/**
 * Tổng lượng độ cao bị DỜI do neo lại theo laser kể từ khởi động, mét. Bên
 * giữ độ cao so với giá trị lần trước và dời mốc đúng phần chênh — xem
 * EST_RANGE_STEP_M trong fc_config.h.
 */
float ekf_altitude_reset_sum_m(void);

/**
 * true nếu laser được DÙNG (cập nhật hoặc neo lại) trong max_ms vừa qua. Không có laser thì độ
 * cao chỉ còn baro + gia tốc: trôi tới ~0,9 m/s khi nghiêng nhiều mà P vẫn nhỏ (đo 09-18).
 */
bool ekf_altitude_range_recent(uint32_t max_ms);

#endif /* EKF_ALTITUDE_H */
