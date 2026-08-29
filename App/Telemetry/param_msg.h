/**
 * @file    param_msg.h
 * @brief   Lớp giao thức của hệ tham số — đường nhị phân cho phần mềm cấu hình.
 *
 * VAI TRÒ
 *
 *   CLI trên USART1 đã cho phép chỉnh và lưu tham số bằng tay. Module này làm
 *   cùng việc đó qua khung nhị phân, để phần mềm trên máy tính dùng được.
 *
 *   Khác biệt cốt lõi so với CLI: app KHÔNG cần biết trước tham số nào tồn
 *   tại. Nó xin cả bảng và firmware tự khai báo tên, kiểu, giới hạn, mặc
 *   định. Nạp firmware có thêm tham số thì app hiện được ngay mà không phải
 *   cập nhật theo — đây là điều làm nên một phần mềm cấu hình thật sự, thay
 *   vì một cái bảng điều khiển gắn cứng.
 *
 * ĐIỀU TIẾT LƯU LƯỢNG
 *
 *   130 tham số × 57 byte ≈ 7,4 KB. Qua USB 921600 baud là tức thời; qua
 *   ESP-NOW ở hồ sơ FLIGHT (~250 B/s) thì mất nửa phút.
 *
 *   Nên bảng được bơm dần: mỗi lần gọi param_msg_update() chỉ phát khi bộ đệm
 *   gửi còn chỗ. Đẩy hết một lượt sẽ làm tràn đệm và NUỐT MẤT heartbeat —
 *   tức là trong lúc app đang tải cấu hình thì nó lại tưởng mất kết nối.
 *
 * MẤT GÓI
 *
 *   Không có phát lại tự động. Mỗi PARAM_VALUE mang theo `count` nên app biết
 *   chính xác nó thiếu chỉ số nào, rồi xin lại từng cái bằng CMD_PARAM_READ.
 *   Cách này rẻ hơn hẳn một tầng ACK cho từng gói, và tự phục hồi được kể cả
 *   khi đường truyền rơi mất nửa bảng.
 *
 * AN TOÀN
 *
 *   Mọi lệnh ghi đều bị từ chối khi đang ARM. Việc chặn nằm ngay trong
 *   handler ở đây chứ không chỉ ở tlm_stream.c, vì đây là lớp duy nhất biết
 *   lệnh nào là lệnh ghi.
 */
#ifndef PARAM_MSG_H
#define PARAM_MSG_H

#include "fc_types.h"
#include "tlm_protocol.h"

/** Đưa máy trạng thái phát bảng về trạng thái nghỉ. Gọi sau tlm_stream_init(). */
void param_msg_init(void);

/**
 * Phát tiếp phần bảng tham số và phần đầu ra CLI còn dở.
 *
 * Gọi đều đặn trong vòng lặp chính, ngay cạnh tlm_stream_update().
 *
 * Việc canh chừng lệnh quay thử motor KHÔNG nằm ở đây: driver dshot đã tự
 * dừng sau `duration_ms` và tự từ chối khi đang arm. Nhân bản phép canh chừng
 * lên tầng này chỉ tạo thêm một chỗ để quên cập nhật, trong khi chốt thật sự
 * cần nằm sát phần cứng để MỌI đường gọi đều đi qua.
 */
void param_msg_update(uint32_t now_ms);

/**
 * Xử lý một khung lệnh thuộc nhóm tham số / CLI / motor test.
 *
 * @return true nếu đã nhận ra và xử lý mã lệnh này.
 */
bool param_msg_handle(const tlm_parser_t *p);

/** Gửi bản tin nhận dạng mạch bay. */
bool param_msg_send_fc_info(void);

/** true nếu đang có lệnh quay thử motor còn hiệu lực. */
bool param_msg_motor_test_active(void);

#endif /* PARAM_MSG_H */
