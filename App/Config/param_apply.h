/**
 * @file    param_apply.h
 * @brief   Đẩy g_params vào những cấu trúc runtime dẫn xuất từ nó.
 *
 * VẤN ĐỀ
 *
 *   Hầu hết module đọc THẲNG `g_params.ten_tham_so`, nên sửa tham số là có
 *   tác dụng ngay ở vòng lặp kế tiếp — không cần làm gì thêm.
 *
 *   Nhưng một số chỗ không đọc thẳng được và phải giữ BẢN SAO hoặc GIÁ TRỊ
 *   TÍNH SẴN:
 *
 *     - ctrl_rate.c : vòng nóng duyệt ba trục bằng chỉ số mảng
 *       (`rate_pid[axis].gains`), mà g_params là các trường phẳng có tên
 *       riêng nên không lập chỉ mục được. Ngoài ra hệ số lọc D được tính sẵn
 *       từ tần số cắt để tránh phép chia trong vòng 4 kHz.
 *
 *   Với những chỗ đó, đổi tham số mà không đẩy lại thì hiện tượng là: CLI in
 *   ra giá trị mới, app hiển thị giá trị mới, mà máy bay vẫn bay y như cũ.
 *   Đây là kiểu hỏng tệ nhất vì mọi thứ trông đều đúng.
 *
 * CÁCH GIẢI
 *
 *   Mọi đường ghi tham số (CLI `set`, `defaults`, và sau này là lệnh nhị phân
 *   từ app PC) gọi fc_params_apply() sau khi ghi xong. Hàm này đẩy lại TẤT CẢ
 *   — không cố đoán tham số nào vừa đổi.
 *
 *   Đẩy thừa là chuyện vặt: việc ghi tham số do con người kích hoạt nên tần
 *   suất tính bằng lần mỗi phút, còn phí tổn chỉ là chép vài chục float. Đổi
 *   lại ta không bao giờ phải bảo trì một bảng "tham số nào ảnh hưởng module
 *   nào" — thứ chắc chắn sẽ lệch khỏi thực tế sau vài lần sửa.
 *
 * KHÔNG GỌI TRONG VÒNG NÓNG.
 */
#ifndef PARAM_APPLY_H
#define PARAM_APPLY_H

/**
 * Đẩy g_params vào mọi cấu trúc dẫn xuất.
 *
 * Gọi sau mỗi lần ghi tham số, và một lần lúc khởi động sau khi đã nạp cấu
 * hình từ flash.
 *
 * An toàn khi gọi lúc đang ARM: nó chỉ chép số, không đụng phần cứng và
 * không xoá trạng thái tích phân đang chạy. (Việc chặn sửa tham số lúc ARM
 * nằm ở tầng trên, không phải ở đây.)
 */
void fc_params_apply(void);

#endif /* PARAM_APPLY_H */
