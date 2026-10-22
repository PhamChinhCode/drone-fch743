/**
 * @file    servo.h
 * @brief   Hai servo trên TIM4: kẹp hàng (PD14, CH3) và nghiêng camera (PD15, CH4).
 *
 * QUYỀN ĐIỀU KHIỂN — đi theo đúng quyền bay, không có công tắc riêng:
 *
 *   ctrl_offboard_pi_has_authority() đúng  -> Pi ra lệnh qua MAVLink
 *   ngược lại                               -> tay điều khiển RC
 *
 *   Nghĩa là: không có Pi, ch8 tắt, hay người lái đẩy cần giành lái (OFFBOARD
 *   về KHOA) thì servo về tay RC, cùng một lúc với vòng bay.
 *
 * CHUYỂN QUYỀN KHÔNG LÀM SERVO GIẬT:
 *
 *   Pi -> RC: servo GIỮ NGUYÊN vị trí Pi đặt, cho tới khi người lái thật sự
 *   động vào kênh của servo đó (lệch quá SERVO_RC_PICKUP_DELTA so với lúc nhận
 *   quyền). Không làm vậy thì đang chở hàng mà công tắc kẹp trên tay cầm nằm
 *   ở nấc "nhả", người lái vừa đẩy cần giành lái là hàng rơi.
 *
 *   RC -> Pi: servo giữ nguyên cho tới lệnh đầu tiên của Pi.
 *
 *   Mất sóng RC: giữ nguyên vị trí cuối.
 *
 * Servo không có phản hồi. ACCEPTED chỉ có nghĩa là xung đã đổi — Pi tự chờ
 * servo chạy hết hành trình.
 */
#ifndef SERVO_H
#define SERVO_H

#include "fc_types.h"

/** Bật PWM TIM4, đặt vị trí khởi động: kẹp ĐÓNG, camera ở giữa. */
void servo_init(void);

/** Đọc RC, áp quyền điều khiển. Gọi mỗi vòng lặp chính, SAU ctrl_angle_update(). */
void servo_update(void);

/* Cửa vào từ MAVLink. Trả false nếu Pi đang không có quyền. */
bool servo_link_grip(bool close);
bool servo_link_cam_us(uint16_t pulse_us);

/*
 * Đặt thẳng, BỎ QUA quyền — cho lệnh CLI `servo` khi thử trên bàn. RC đang có
 * sóng sẽ ghi đè ngay nhịp sau, nên chỉ có tác dụng khi không cắm thu RC.
 */
void servo_set_grip(bool close);
void servo_set_cam_us(uint16_t pulse_us);

/* Xung đang xuất, µs. */
uint16_t servo_grip_us(void);
uint16_t servo_cam_us(void);

#endif /* SERVO_H */
