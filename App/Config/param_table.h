/**
 * @file    param_table.h
 * @brief   Tham số chỉnh được lúc chạy — struct giá trị và bảng mô tả.
 *
 * VẤN ĐỀ NÓ GIẢI QUYẾT
 *
 *   fc_config.h chỉnh cấu hình ở compile-time. Đổi một hệ số PID là phải sửa
 *   mã nguồn -> build -> nạp lại -> rút pin cắm lại ESC. Vòng lặp đó quá chậm
 *   để tinh chỉnh một chiếc drone thật: chỉnh PID cần hàng chục lần thử, mỗi
 *   lần vài phút thì một buổi chiều chỉ đi được vài bước.
 *
 * CÁCH LÀM
 *
 *   Một struct duy nhất g_params chứa mọi giá trị chỉnh được, cạnh nó là một
 *   bảng g_param_table[] mô tả từng trường: tên, kiểu, min, max, mặc định.
 *   Cả hai sinh ra từ CÙNG một danh sách trong param_list.h nên không lệch
 *   nhau được.
 *
 *   Bảng mô tả là thứ làm nên phần mềm cấu hình: app PC không cần biết trước
 *   tham số nào tồn tại, nó hỏi firmware và firmware tự khai báo. Nạp firmware
 *   mới có thêm tham số thì app hiển thị được ngay, không phải cập nhật theo.
 *
 * QUAN HỆ VỚI fc_config.h
 *
 *   fc_config.h KHÔNG bị thay thế. Nó trở thành "cấu hình xuất xưởng": cột
 *   `def` của mỗi dòng trong param_list.h trỏ thẳng vào macro tương ứng. Mọi
 *   ghi chú đo đạc trong đó vẫn nguyên giá trị và vẫn là nơi đọc để hiểu VÌ
 *   SAO một con số lại là con số đó.
 *
 * VÌ SAO param_storage_t KHÔNG `packed`
 *
 *   Đây là quyết định có chủ ý, ngược với các struct bản tin trong
 *   tlm_messages.h (những struct đó BẮT BUỘC packed vì phải khớp byte với
 *   máy tính ở đầu kia).
 *
 *   Struct này không đi qua đường truyền — nó chỉ nằm trong RAM và được đọc
 *   trong VÒNG LẶP 4 kHz. Packed thì một float có thể rơi vào địa chỉ lệch
 *   4 byte, và VLDR của Cortex-M7 KHÔNG nạp được địa chỉ lệch: trình biên
 *   dịch buộc phải sinh chuỗi nạp từng byte rồi ghép lại. Vài chục lần như
 *   vậy mỗi vòng lặp là lãng phí không có lý do.
 *
 *   Đổi lại, không packed thì có thể sinh byte đệm giữa các trường. Byte đệm
 *   không được khởi tạo sẽ làm CRC của khối lưu flash đổi ngẫu nhiên giữa hai
 *   lần chạy. Chặn bằng cách memset toàn bộ struct về 0 trong
 *   param_load_defaults() TRƯỚC khi điền — từ đó byte đệm luôn là 0 xác định.
 *
 *   param_list.h cũng đã xếp trường theo kích thước giảm dần (4 -> 2 -> 1
 *   byte) nên thực tế gần như không sinh lỗ đệm nào.
 *
 * ĐỌC TRONG VÒNG NÓNG
 *
 *   Đọc THẲNG trường: `g_params.rate_pid_roll_kp`. Chi phí đúng bằng đọc một
 *   biến toàn cục, tức bằng chi phí của macro trước đây.
 *
 *   TUYỆT ĐỐI KHÔNG gọi param_get_f32() trong vòng 4 kHz — hàm đó tra bảng và
 *   rẽ nhánh theo kiểu, nó dành cho CLI và cho lớp giao thức.
 *
 * THÊM MỘT THAM SỐ
 *
 *   Thêm một dòng P(...) vào param_list.h, rồi tăng PARAM_SCHEMA_VERSION
 *   trong param_store.h. Không phải sửa file này.
 */
#ifndef PARAM_TABLE_H
#define PARAM_TABLE_H

#include "fc_types.h"
#include "fc_config.h"
#include <stddef.h>

/* ==========================================================================
 * Kiểu và cờ
 * ========================================================================== */

/**
 * Mã kiểu gửi kèm trong bản tin PARAM_VALUE. App PC dùng nó để biết hiển thị
 * ô nhập số nguyên hay số thực, và để đóng gói giá trị khi gửi ngược lại.
 *
 * KHÔNG ĐƯỢC đổi thứ tự hay dùng lại mã cũ — app PC đã biên dịch sẵn theo
 * bảng này.
 */
typedef enum {
    PT_U8  = 0,
    PT_I8  = 1,
    PT_U16 = 2,
    PT_I16 = 3,
    PT_U32 = 4,
    PT_I32 = 5,
    PT_F32 = 6
} param_type_t;

/** Đổi xong phải khởi động lại mới có tác dụng (giá trị tính sẵn lúc init). */
#define PARAM_FLAG_REBOOT    0x01u

/**
 * Đặt sai thì máy bay mất kiểm soát: đảo chiều cần, ma trận trục cảm biến,
 * thứ tự motor, dấu yaw. App PC PHẢI hỏi xác nhận trước khi gửi.
 */
#define PARAM_FLAG_DANGER    0x02u

/** Chỉ đọc — firmware từ chối mọi lệnh ghi. */
#define PARAM_FLAG_READONLY  0x04u

/**
 * Độ dài tối đa của tên, TÍNH CẢ ký tự kết thúc chuỗi. Khớp tlm_param_value_t.
 *
 * 28 chứ không phải một số tròn hơn: tên dài nhất hiện nay là
 * `rc_mode_poshold_threshold` (25 ký tự) và quy tắc đặt tên bắt nó phải giữ
 * nguyên dạng đó. Với 28 byte thì payload PARAM_VALUE là 46 byte, vẫn nằm
 * thoải mái dưới TLM_MAX_PAYLOAD = 64.
 *
 * param_table.c có _Static_assert kiểm từng tên lúc biên dịch, nên vượt hạn
 * là build hỏng ngay chứ không phải cắt cụt lặng lẽ rồi app PC nhận nhầm tên.
 */
#define PARAM_NAME_MAX  28

/* ==========================================================================
 * Bảng mô tả
 * ========================================================================== */

typedef struct {
    const char *name;    /**< tên CLI, ngắn hơn PARAM_NAME_MAX      */
    uint8_t     type;    /**< param_type_t                          */
    uint8_t     flags;   /**< PARAM_FLAG_*                          */
    uint16_t    offset;  /**< offsetof(param_storage_t, trường)     */
    float       min;
    float       max;
    float       def;
} param_meta_t;

/* ==========================================================================
 * Struct giá trị — sinh từ param_list.h
 * ========================================================================== */

typedef struct {
#define P(name, ctype, ptype, flags, min, max, def)  ctype name;
#include "param_list.h"
#undef P
} param_storage_t;

/** Giá trị đang chạy. Các module đọc thẳng trường của biến này. */
extern param_storage_t g_params;

/** Bảng mô tả, nằm trong flash (const). */
extern const param_meta_t g_param_table[];

/** Số tham số trong bảng. */
extern const uint16_t g_param_count;

/* ==========================================================================
 * API
 *
 * Toàn bộ nhóm này KHÔNG dành cho vòng nóng — chỉ CLI, lớp giao thức và
 * param_store gọi tới.
 * ========================================================================== */

/**
 * Đặt toàn bộ g_params về giá trị mặc định trong bảng.
 *
 * Memset về 0 trước khi điền, nên byte đệm của struct luôn xác định và CRC
 * của khối lưu flash lặp lại được. Gọi hàm này TRƯỚC param_store_load().
 */
void param_load_defaults(void);

/**
 * Đọc một tham số, quy về float bất kể kiểu thật.
 * @return 0.0f nếu index vượt bảng.
 */
float param_get_f32(uint16_t index);

/**
 * Ghi một tham số. Giá trị LUÔN bị kẹp về [min, max] trước khi ghi — firmware
 * không tin app PC gửi gì, và CLI cũng đi qua đúng đường này.
 *
 * @return false nếu index vượt bảng hoặc tham số có cờ READONLY. Bị kẹp giá
 *         trị KHÔNG phải là lỗi: hàm trả true, và người gọi nên đọc lại bằng
 *         param_get_f32() để biết firmware thực sự nhận con số nào.
 */
bool param_set_f32(uint16_t index, float value);

/**
 * Tìm theo tên, so khớp chính xác.
 * @return chỉ số, hoặc PARAM_INDEX_NONE nếu không có.
 */
uint16_t param_find(const char *name);

#define PARAM_INDEX_NONE  0xFFFFu

/** true nếu tham số đang khác giá trị mặc định. Dùng cho lệnh CLI `diff`. */
bool param_is_modified(uint16_t index);

/**
 * CRC32 của chính BẢNG MÔ TẢ (tên, kiểu, cờ, offset, min, max, def) — không
 * phải của giá trị.
 *
 * Đây là "chữ ký" của tập tham số mà firmware này hiểu. Dùng vào hai việc:
 *   - param_store phát hiện khối lưu trong flash thuộc về một firmware khác,
 *     tức các offset không còn nghĩa cũ, nên phải bỏ chứ không được nạp bừa.
 *   - app PC phát hiện bảng đã đổi và biết phải đọc lại từ đầu.
 *
 * Tính một lần rồi nhớ lại; bảng nằm trong flash nên không đổi lúc chạy.
 */
uint32_t param_table_crc32(void);

/** CRC32 chuẩn (đa thức đảo 0xEDB88320). Dùng chung với param_store. */
uint32_t param_crc32(const void *data, size_t len);

#endif /* PARAM_TABLE_H */
