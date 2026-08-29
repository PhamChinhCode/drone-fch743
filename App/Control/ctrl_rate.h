/**
 * @file    ctrl_rate.h
 * @brief   Vòng PID tốc độ góc — chế độ ACRO.
 *
 * ĐÂY LÀ VÒNG TRONG CÙNG. Người lái yêu cầu một TỐC ĐỘ QUAY (độ/giây), vòng
 * này bắt máy bay bám đúng tốc độ đó. Mọi chế độ khác đều chồng lên nó:
 * ANGLE tính ra tốc độ quay cần thiết để về góc mong muốn rồi giao xuống đây;
 * ALTHOLD cũng vậy với trục đứng. Vòng này sai thì không chế độ nào cứu được.
 *
 * NHỊP CHẠY — bám theo mẫu gyro, không bám theo đồng hồ:
 *   Vòng chạy khi driver IMU báo có mẫu mới, chứ không phải khi đủ N micro
 *   giây. Lý do là khâu vi phân: nó chia cho dt, nên dt rung một chút là D
 *   nảy lung tung. Chân DRDY của con quay do thạch anh của chính nó phát,
 *   đều hơn nhiều so với thời điểm vòng lặp chính chạy tới đây.
 *   IMU chạy 8 kHz, chia đôi còn FC_LOOP_RATE_HZ = 4 kHz.
 *
 * KHÂU VI PHÂN LẤY TRÊN SỐ ĐO, KHÔNG LẤY TRÊN SAI SỐ:
 *   D = -kd · d(gyro)/dt  chứ không phải  kd · d(sai số)/dt
 *   Hai cách chỉ khác nhau khi setpoint đổi, nhưng khác rất mạnh: gạt cần đột
 *   ngột làm sai số nhảy bậc, đạo hàm của bậc là xung vô hạn — máy bay giật
 *   nảy mỗi lần chạm cần. Lấy trên số đo thì setpoint đổi bao nhiêu cũng không
 *   sinh xung, mà tác dụng dập dao động vẫn nguyên vẹn.
 *
 * CHỐNG DỒN TÍCH PHÂN, hai lớp:
 *   1. Chặn cứng theo RATE_PID_I_LIMIT.
 *   2. NGỪNG tích luỹ khi khâu trộn đang bão hoà. Motor đã chạm trần thì tích
 *      thêm cũng không ra thêm lực nào, chỉ dồn một khoản nợ mà lúc thoát bão
 *      hoà phải xả ra — và lúc xả thì máy bay lật.
 *
 * AN TOÀN:
 *   Chưa arm thì vòng này ghi 0 vào pid_output và xoá sạch tích phân. Đây
 *   không phải chốt an toàn — chốt thật nằm ở arming.c và dshot.c — mà để
 *   tích phân không dồn sẵn trong lúc máy bay nằm trên bàn rồi bung ra ngay
 *   giây đầu tiên sau khi arm.
 */
#ifndef CTRL_RATE_H
#define CTRL_RATE_H

#include "fc_types.h"
#include "fc_config.h"

/** Nạp hệ số từ fc_config.h và xoá trạng thái. Gọi một lần lúc khởi động. */
void ctrl_rate_init(void);

/**
 * Nap lai he so PID tu g_params vao cau truc dieu khien.
 *
 * PHAI goi sau moi lan he so doi luc chay. Module nay giu mot ban sao (vong
 * nong duyet ba truc bang chi so mang, khong doc thang truong phang cua
 * g_params duoc), nen khong goi thi tham so doi ma may bay khong doi.
 *
 * param_apply.c goi ham nay; binh thuong khong can goi tay.
 */
void ctrl_rate_apply_params(void);

/**
 * Chạy một bước nếu có mẫu gyro mới. Tự giữ nhịp nên gọi bao nhiêu lần cũng
 * được. Gọi trong vòng lặp chính, TRƯỚC mixer_update().
 *
 * Đọc g_fc.rc (cần điều khiển) và g_fc.imu.gyro_filtered_dps, ghi
 * g_fc.ctrl.setpoint_rate_dps và g_fc.ctrl.pid_output.
 *
 * @return true nếu vừa chạy một bước
 */
bool ctrl_rate_update(void);

/** Xoá tích phân và trạng thái vi phân. Gọi khi disarm hoặc đổi chế độ. */
void ctrl_rate_reset(void);

/* --- Thống kê phục vụ chỉnh PID ---------------------------------------- */

uint32_t ctrl_rate_loops(void);

/** Tần số thực tế của vòng, Hz. Phải bám sát FC_LOOP_RATE_HZ. */
uint32_t ctrl_rate_hz(void);

/** Số lần bỏ bước vì nhịp gyro bất thường. Phải đứng yên ở 0. */
uint32_t ctrl_rate_skips(void);

#endif /* CTRL_RATE_H */
