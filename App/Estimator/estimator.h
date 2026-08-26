/**
 * @file    estimator.h
 * @brief   Điều phối các bộ lọc hợp nhất, ghi kết quả vào g_fc.est.
 *
 * VÌ SAO CHIA THÀNH NHIỀU BỘ LỌC NHỎ:
 *   Cách "sách vở" là một EKF 15 trạng thái gộp góc, vận tốc, vị trí và mọi
 *   bias vào chung. Thực tế firmware bay nhỏ hầu như đều tách ra, vì:
 *
 *     - Chi phí tính toán tăng theo BẬC BA của số trạng thái. Hai bộ lọc
 *       6 và 3 trạng thái tốn 6³+3³ = 243 đơn vị; một bộ 9 trạng thái tốn
 *       729 — gấp ba, cho cùng một lượng thông tin.
 *     - Các phần ghép chéo giữa góc và độ cao rất yếu. Gộp lại tốn rất nhiều
 *       phép tính để mô tả những tương quan gần như bằng không.
 *     - Gỡ lỗi được. Độ cao sai thì biết chắc lỗi nằm trong ekf_altitude,
 *       không phải mò trong ma trận 15×15.
 *
 *   Ghép nối giữa hai bộ lọc là một chiều và tường minh: bộ lọc góc cấp ma
 *   trận xoay cho bộ lọc độ cao dùng (để tách trọng lực khỏi accel và để
 *   chiếu tia laser xuống phương thẳng đứng). Không có chiều ngược lại.
 *
 * NHỊP CHẠY:
 *   EST_RATE_HZ (mặc định 1000). Driver IMU chạy 8 kHz trong ngắt và ghi
 *   thẳng vào g_fc.imu; bộ ước lượng chỉ lấy mẫu mới nhất. Baro (50 Hz) và
 *   laser (100 Hz) được nạp vào bộ lọc đúng lúc có mẫu mới, nhận biết qua
 *   sample_count chứ không phải qua mốc thời gian.
 *
 * CHƯA LÀM — VẬN TỐC NGANG TỪ OPTICAL FLOW:
 *   Cố ý bỏ. FLOW_RAD_PER_COUNT trong fc_config.h hiện là giá trị phỏng đoán,
 *   chính chú thích ở đó ghi rõ "CẦN HIỆU CHUẨN BẰNG THỰC NGHIỆM". Hợp nhất
 *   một cảm biến chưa hiệu chuẩn không cho ra vận tốc sai một cách lộ liễu —
 *   nó cho ra con số trông hợp lý nhưng sai tỉ lệ, và bộ lọc sẽ khẳng định
 *   con số đó với độ tin cậy cao. Hiệu chuẩn hệ số ấy trước đã, rồi mới thêm
 *   bộ lọc vận tốc ngang.
 *   Vì vậy g_fc.est.position_valid luôn bằng false ở phiên bản này.
 */
#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "fc_types.h"
#include "fc_config.h"

/** Đặt lại toàn bộ bộ lọc. Gọi sau khi các driver cảm biến đã init. */
void estimator_init(void);

/**
 * Chạy một nhịp bộ lọc nếu đã tới hạn. Tự giữ nhịp EST_RATE_HZ nên gọi bao
 * nhiêu lần cũng được. Gọi trong vòng lặp chính, SAU các driver cảm biến.
 *
 * @param now_us  mốc thời gian micro giây từ micros()
 * @return true nếu vừa chạy một bước
 */
bool estimator_update(uint32_t now_us);

/* --- Thống kê phục vụ chẩn đoán ---------------------------------------- */

uint32_t estimator_steps(void);
uint32_t estimator_baro_updates(void);
uint32_t estimator_range_updates(void);

/** Số lần số đo laser bị bỏ vì nghiêng quá hoặc ngoài tầm tin cậy. */
uint32_t estimator_range_rejected(void);

#endif /* ESTIMATOR_H */
