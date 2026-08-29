/**
 * @file    param_apply.c
 * @brief   Xem param_apply.h.
 *
 * THÊM MODULE VÀO ĐÂY KHI NÀO
 *
 *   Chỉ khi module đó giữ bản sao hoặc giá trị tính sẵn từ g_params. Module
 *   đọc thẳng `g_params.x` trong hàm update() thì KHÔNG cần có mặt ở đây —
 *   thêm vào chỉ tốn công mà không đổi hành vi.
 *
 *   Cách nhận ra: grep tên tham số trong module. Nếu nó chỉ xuất hiện trong
 *   hàm *_init() hoặc được gán vào một biến static, đó là bản sao và phải
 *   thêm hàm *_apply_params() vào danh sách dưới đây.
 */
#include "param_apply.h"
#include "ctrl_rate.h"
#include "mixer.h"
#include "dshot.h"

void fc_params_apply(void)
{
    ctrl_rate_apply_params();
    mixer_apply_params();
    dshot_apply_params();

    /*
     * ctrl_angle.c và ctrl_poshold.c cố ý KHÔNG có mặt ở đây: chúng đọc
     * thẳng g_params trong hàm update() nên không có gì để đồng bộ.
     *
     * Các driver cảm biến (icm20602, lsm6dsv, bmp388, mtf01p) sẽ vào đây khi
     * được chuyển sang g_params — phần lớn tham số của chúng ghi vào thanh
     * ghi phần cứng lúc init nên mang cờ PARAM_FLAG_REBOOT và không đẩy lại
     * lúc chạy được; chỉ những thứ thuần tính toán (ma trận trục, hệ số lọc,
     * hiệu chuẩn từ kế) mới đẩy lại được.
     */
}
