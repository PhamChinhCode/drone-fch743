/**
 * @file    blackbox.h
 * @brief   Ghi log chuyến bay ra thẻ SD, qua bộ đệm RAM.
 *
 * VÌ SAO PHẢI QUA BỘ ĐỆM RAM
 *
 *   f_write() là hàm CHẶN. Thẻ SD có thể khựng hàng chục tới hàng trăm mili
 *   giây khi nó tự dọn khối hoặc san đều hao mòn — không đoán trước được.
 *
 *   Vòng lặp chính phải xong dưới 250 µs để giữ nhịp PID 4 kHz và nhịp phát
 *   DShot 1 kHz. Một cú khựng 100 ms sẽ làm ESC thấy mất tín hiệu và cắt
 *   motor. Giữa không trung thì đó là rơi.
 *
 *   Nên đường đi là: BAY thì ghi vào RAM (vài chục nano giây mỗi bản ghi),
 *   HẠ XUỐNG rồi mới xả ra thẻ.
 *
 * MÁY TRẠNG THÁI
 *
 *      init  ──> IDLE ──(arm)──> RECORDING ──(disarm)──> FLUSHING ──┐
 *                 ▲                                                  │
 *                 └──────────────────────────────────────────────────┘
 *
 *   Mọi thao tác với thẻ (mở file, ghi, đóng) đều nằm ở IDLE và FLUSHING,
 *   tức lúc đã DISARM. RECORDING tuyệt đối không đụng vào thẻ.
 *
 * ĐỊNH DẠNG
 *
 *   Trong RAM là bản ghi nhị phân 48 byte cho gọn. Lúc xả ra thẻ mới đổi
 *   sang CSV — thẻ có hàng GB nên không tiếc chỗ, và CSV thì mở thẳng bằng
 *   Excel hay Python được ngay.
 */
#ifndef BLACKBOX_H
#define BLACKBOX_H

#include "fc_types.h"
#include "fc_config.h"
#include "log_record.h"

/* bb_record_t va cach doi ra CSV nam o log_record.h - dung chung voi flashlog. */

typedef enum {
    BB_STATE_OFF = 0,     /**< tắt bằng cấu hình, hoặc chưa mount được thẻ */
    BB_STATE_IDLE,        /**< file đã mở sẵn, đang chờ arm                */
    BB_STATE_RECORDING,   /**< đang ghi vào RAM, KHÔNG chạm thẻ            */
    BB_STATE_FLUSHING,    /**< đang xả bộ đệm ra thẻ                       */
    BB_STATE_ERROR        /**< lỗi thẻ, đã ngừng hẳn                       */
} bb_state_t;

/**
 * Mount thẻ, tìm số thứ tự file kế tiếp, mở file và ghi dòng tiêu đề.
 *
 * Hàm CHẶN, có thể mất vài trăm ms. Gọi lúc khởi động, khi chưa có gì
 * chạy theo thời gian thực.
 *
 * @return true nếu thẻ mount được và file đã mở. false thì mọi hàm khác
 *         của module này trở thành lệnh rỗng — máy bay vẫn bay bình thường.
 */
bool blackbox_init(void);

/**
 * Chạy máy trạng thái. Gọi mỗi vòng lặp chính.
 *
 * Lúc RECORDING chỉ chép 48 byte vào RAM theo nhịp BB_RATE_HZ — vài chục
 * nano giây, an toàn tuyệt đối cho vòng 4 kHz.
 */
void blackbox_update(uint32_t now_us);

bb_state_t  blackbox_state(void);
const char *blackbox_state_name(void);

/** Số bản ghi đang có trong bộ đệm. */
uint32_t blackbox_records(void);

/** Số bản ghi bị bỏ vì bộ đệm đầy. Khác 0 nghĩa là chuyến bay dài hơn
 *  sức chứa — xem BB_BUFFER_BYTES. */
uint32_t blackbox_dropped(void);

/** Tiến độ xả ra thẻ, 0..100. */
uint8_t blackbox_flush_percent(void);

/** Số thứ tự file đang mở, tức LOGnnnn.CSV. */
uint16_t blackbox_file_index(void);

/** Mã lỗi FRESULT của thao tác thẻ gần nhất. 0 là không lỗi. */
uint8_t blackbox_last_error(void);

#endif /* BLACKBOX_H */
