/**
 * @file    fc_hooks.c
 * @brief   Hiện thực các hook mà tlm_stream.c khai báo `weak`.
 *
 * CÁCH NÓ HOẠT ĐỘNG
 *
 *   tlm_stream.c định nghĩa fc_hook_*() với thuộc tính `weak` và thân rỗng,
 *   để lớp giao thức biên dịch được ngay cả khi chưa có module hiệu chuẩn hay
 *   lưu cấu hình. Định nghĩa lại cùng tên ở đây thì trình liên kết tự thay
 *   thế — KHÔNG phải sửa một dòng nào trong tlm_stream.c.
 *
 * ĐIỀU KIỆN AN TOÀN
 *
 *   handle_action() trong tlm_stream.c đã từ chối mọi lệnh khi đang ARM trước
 *   khi gọi tới đây. Các hàm dưới đây vẫn tự kiểm lại lần nữa: chúng cũng gọi
 *   được từ CLI và từ code khác về sau, mà hậu quả của việc quên chặn là mất
 *   motor giữa không trung.
 */
#include "fc_state.h"
#include "param_store.h"
#include "tlm_stream.h"
#include "main.h"

/* ==========================================================================
 * Lưu cấu hình
 * ========================================================================== */

void fc_hook_save_config(void)
{
    const param_store_result_t res = param_store_save();

    /*
     * Báo kết quả bằng bản tin chữ. Lệnh SAVE_CONFIG hiện không có đường trả
     * mã lỗi riêng, nên đây là cách duy nhất để người dùng biết việc ghi có
     * thành công hay không. Im lặng khi ghi hỏng là kiểu hỏng tệ nhất: người
     * dùng tin rằng cấu hình đã an toàn rồi mới nạp firmware mới.
     *
     * (Bản tin ACK có mã lỗi sẽ thêm ở giai đoạn mở rộng giao thức.)
     */
    if (res == PARAM_STORE_OK) {
        tlm_stream_send_text(0, "cau hinh: da ghi flash");
    } else {
        tlm_stream_send_text(2, param_store_result_name(res));
    }
}

/* ==========================================================================
 * Khởi động lại
 * ========================================================================== */

void fc_hook_reboot(void)
{
    if (g_fc.mode == FC_MODE_ARMED) {
        return;
    }

    tlm_stream_send_text(0, "dang khoi dong lai...");

    /*
     * Cho bộ đệm gửi kịp xả trước khi reset. Không có nhịp chờ này thì dòng
     * thông báo nằm lại trong ring buffer và biến mất cùng lần reset — người
     * dùng thấy máy im bặt mà không biết vì sao.
     *
     * HAL_Delay() chặn, nhưng đã DISARM nên vòng lặp đứng lại không sao.
     */
    HAL_Delay(50);

    NVIC_SystemReset();
}
