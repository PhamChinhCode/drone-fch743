/**
 * @file    flashlog.h
 * @brief   Ghi log bay thẳng vào flash NOR trên QUADSPI, ngay trong lúc bay.
 *
 * KHÁC BLACKBOX Ở ĐIỂM NÀO
 *
 *   blackbox.c phải gom hết vào RAM rồi mới xả ra thẻ SD sau khi hạ cánh, vì
 *   f_write() có thể khựng hàng trăm mili giây. Hệ quả là chuyến bay bị giới
 *   hạn ở 55 giây — đúng bằng chỗ chứa của bộ đệm.
 *
 *   Flash NOR không có giới hạn đó. Ghi một trang 256 byte tách làm hai việc:
 *   đẩy byte xuống chip mất 34 µs, rồi chip TỰ lập trình thêm 0,4 ms trong
 *   khi CPU đi làm việc khác. Ở nhịp 100 Hz thì mỗi trang cách nhau 53 ms,
 *   thừa thãi. Nên ở đây ghi thẳng, không cần đệm chuyến bay trong RAM.
 *
 *   Đổi lại: 8 MB thay vì hàng GB, và phải xoá trước khi ghi.
 *
 * BỐ TRÍ TRÊN CHIP — KHÔNG CÓ HỆ TẬP TIN
 *
 *   Một dòng bản ghi 48 byte nối tiếp, mọc từ địa chỉ 0 lên. Mỗi chuyến bay
 *   mở đầu bằng một bản ghi mốc 48 byte mang magic riêng.
 *
 *   Không có bảng FAT, không có chỉ mục, không có siêu dữ liệu nào để hỏng.
 *   Mất điện giữa lúc ghi thì tệ nhất mất 256 byte cuối; bật lên lại, tìm
 *   nhị phân trang đã xoá đầu tiên là biết ghi tiếp từ đâu.
 *
 *   Cuối mỗi chuyến, trang dở dang được đệm 0xFF cho đủ rồi ghi xuống — dữ
 *   liệu an toàn ngay cả khi rút điện liền sau đó. Người đọc bỏ qua phần đệm
 *   bằng cách nhảy tới ranh giới trang kế tiếp.
 *
 * KHÔNG BAO GIỜ TỰ XOÁ
 *
 *   Đầy thì DỪNG ghi, không ghi đè vòng tròn. Xoá phải do người ra lệnh
 *   ('flash erase'). Không thể để log của một sự cố bị mất chỉ vì bay thêm
 *   một chuyến nữa.
 */
#ifndef FLASHLOG_H
#define FLASHLOG_H

#include "fc_types.h"
#include "fc_config.h"
#include "log_record.h"

/**
 * Bản ghi mốc mở đầu mỗi chuyến bay. Đúng 48 byte như bản ghi dữ liệu.
 *
 * Nhận ra nó bằng magic nằm ở 4 byte đầu — cùng chỗ với trường t_ms của bản
 * ghi dữ liệu. Giá trị magic tương ứng với 49 ngày bay liên tục nên không
 * thể nhầm với một mốc thời gian thật.
 */
#define FLASHLOG_MAGIC  0xFCBB0001u

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t boot_ms;     /**< HAL_GetTick lúc bắt đầu chuyến      */
    uint16_t rate_hz;
    uint8_t  rec_bytes;
    uint8_t  version;
    uint8_t  reserved[52];
} flashlog_hdr_t;

_Static_assert(sizeof(flashlog_hdr_t) == LOG_RECORD_BYTES,
               "moc dau chuyen phai dai bang mot ban ghi");

typedef enum {
    FL_STATE_OFF = 0,     /**< tắt bằng cấu hình, hoặc không thấy chip */
    FL_STATE_READY,       /**< sẵn sàng, đang chờ ARM                  */
    FL_STATE_RECORDING,   /**< đang ghi thẳng vào flash                */
    FL_STATE_FULL,        /**< hết chỗ, phải 'flash erase'             */
    FL_STATE_ERROR        /**< chip báo lỗi, đã ngừng hẳn              */
} flashlog_state_t;

/** Dò điểm cuối dữ liệu đã có. CHẶN khoảng 1,5 ms. Gọi lúc khởi động. */
bool flashlog_init(void);

/** Chạy máy trạng thái. Gọi mỗi vòng lặp chính. */
void flashlog_update(uint32_t now_us);

flashlog_state_t  flashlog_state(void);
const char       *flashlog_state_name(void);

/** Số byte đã dùng, tính cả những chuyến trước còn trên chip. */
uint32_t flashlog_used_bytes(void);

/** Chỗ chứa tổng cộng dành cho log. */
uint32_t flashlog_capacity_bytes(void);

/** Số bản ghi đã ghi trong chuyến hiện tại. */
uint32_t flashlog_records(void);

/** Số bản ghi mất vì cả hai đệm trang đều đầy. Khác 0 là chip quá chậm. */
uint32_t flashlog_dropped(void);

/**
 * Chẩn đoán trên bàn: ghi một chuyến bay giả gồm `records` bản ghi.
 *
 * Đi qua ĐÚNG đường ghi của lúc bay thật — mốc đầu chuyến, nối bản ghi, đảo
 * hai đệm trang, xả trang dở dang — nên nó kiểm được cả chuỗi mà không cần
 * ARM và không cần quay motor.
 *
 * Khác một điểm: ở đây có đợi chip ghi xong giữa các trang, vì bản ghi được
 * bơm liên tục chứ không cách nhau 10 ms như lúc bay.
 *
 * Hàm CHẶN. Chỉ chạy khi đang READY và đã DISARM.
 */
bool flashlog_selftest(uint32_t records);

#endif /* FLASHLOG_H */
