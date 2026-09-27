/**
 * @file    log_record.h
 * @brief   Bản ghi log bay: định dạng nhị phân, cách điền, cách đổi ra CSV.
 *
 * VÌ SAO TÁCH RA RIÊNG
 *
 *   Có HAI đường ghi log, cùng một dữ liệu:
 *     - blackbox.c  -> thẻ SD, đệm trong RAM rồi xả khi đã hạ cánh
 *     - flashlog.c  -> flash NOR trên QUADSPI, ghi thẳng trong lúc bay
 *
 *   Cả hai đều cần đúng cùng một bản ghi 48 byte và đúng cùng một cách đổi
 *   sang CSV. Chép đôi thì sớm muộn hai bên lệch nhau, và lúc đó hai file
 *   log của cùng một chuyến bay sẽ không so được với nhau nữa.
 */
#ifndef LOG_RECORD_H
#define LOG_RECORD_H

#include "fc_types.h"

/** Một bản ghi. Đúng 48 byte, xem chú thích từng trường. */
typedef struct __attribute__((packed)) {
    uint32_t t_ms;        /**< mili giây kể từ lúc bắt đầu ghi        */
    int16_t  gyro[3];     /**< tốc độ góc đã lọc, đơn vị 0,1 °/s      */
    int16_t  accel[3];    /**< gia tốc, đơn vị mg                     */
    int16_t  sp[3];       /**< mục tiêu tốc độ góc, đơn vị 0,1 °/s    */
    int16_t  pid[3];      /**< đầu ra PID -1..1, nhân 10000           */
    uint16_t motor[4];    /**< giá trị DShot thô 0..2047              */
    int16_t  att[3];      /**< roll/pitch/yaw, đơn vị 0,01 rad        */
    int16_t  alt_cm;      /**< độ cao ước lượng, cm                   */
    uint16_t thr;         /**< lệnh ga 0..1, nhân 10000               */
    uint8_t  mode;        /**< flight_mode_t                          */
    uint8_t  flags;       /**< bit0 armed, bit1 mixer bão hoà         */

    /* --- Chẩn đoán giữ vị trí ---
     *
     * Sáu nguyên nhân thường gặp nhất của "không giữ được chỗ" đều nằm ở
     * phía TRÊN vòng góc: flow chết, ước lượng vị trí không hợp lệ, nghiêng
     * quá ngưỡng làm bộ ước lượng từ chối flow, hệ số poshold sai, tích phân
     * bị cuốn, yaw lệch. Không có mấy trường dưới đây thì log chỉ cho thấy
     * hai vòng trong cùng, tức phần hiếm khi có lỗi.
     */
    int16_t  sp_angle[2]; /**< roll, pitch mục tiêu, đơn vị 0,01 rad  */
    int16_t  vel_body[2]; /**< vận tốc thân tới/phải, cm/s            */
    int16_t  vel_tgt[2];  /**< vận tốc thân mong muốn, cm/s           */
    uint16_t range_cm;    /**< khoảng cách tới mặt đất, cm            */
    uint8_t  flow_q;      /**< chất lượng optical flow, 0..255        */
    uint8_t  est_flags;   /**< các cờ LOG_EST_* dưới đây              */
} bb_record_t;

/*
 * 64 byte chứ không phải 48. Ngoài chỗ cho các trường trên, con số này còn
 * chia đúng 256 — mỗi trang flash chứa trọn 4 bản ghi, không bản ghi nào bị
 * cắt qua ranh giới trang.
 *
 * Giá phải trả: đệm RAM của blackbox còn 41 giây thay vì 55, và flash QSPI
 * chứa được 21,8 phút thay vì 29.
 */
#define LOG_RECORD_BYTES  64u

/** Các bit của bb_record_t.est_flags. */
#define LOG_EST_POS_VALID    0x01u  /**< ước lượng vị trí dùng được      */
#define LOG_EST_ALT_VALID    0x02u
#define LOG_EST_ATT_VALID    0x04u
#define LOG_EST_FLOW_OK      0x08u  /**< driver MTF-01P còn sống         */
#define LOG_EST_RANGE_VALID  0x10u
#define LOG_EST_POS_LOCKED   0x20u  /**< poshold đã chốt mốc giữ chỗ     */
#define LOG_EST_ALT_VIBE     0x40u  /**< EKF độ cao đang bỏ accel vì rung */
#define LOG_EST_SAT_GUARD    0x80u  /**< ALTHOLD đang chặn ga vì bão hoà  */

_Static_assert(sizeof(bb_record_t) == LOG_RECORD_BYTES,
               "ban ghi log phai dung 48 byte");

/** Độ dài tối đa một dòng CSV, kể cả CR LF. Dùng để định cỡ đệm. */
#define LOG_RECORD_CSV_MAX  320

/** Dòng tiêu đề CSV, kết thúc bằng CR LF. */
extern const char g_log_csv_header[];

/** Chụp trạng thái hiện tại của máy bay vào một bản ghi. */
void log_record_fill(bb_record_t *r, uint32_t t_ms);

/**
 * Đổi một bản ghi thành một dòng CSV kết thúc bằng CR LF.
 *
 * KHÔNG đặt dấu kết chuỗi. Trả về số ký tự đã ghi, luôn nhỏ hơn
 * LOG_RECORD_CSV_MAX.
 */
int log_record_to_csv(char *out, const bb_record_t *r);

#endif /* LOG_RECORD_H */
