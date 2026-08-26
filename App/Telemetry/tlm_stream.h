/**
 * @file    tlm_stream.h
 * @brief   Bộ lập lịch phát telemetry theo bảng đăng ký.
 *
 * Ý TƯỞNG:
 *   Mỗi loại bản tin là một "luồng" (stream) có chu kỳ phát riêng. Bảng
 *   `g_tlm_streams[]` trong tlm_stream.c liệt kê toàn bộ luồng. Hàm
 *   tlm_stream_update() được gọi đều đặn trong vòng lặp chính, quét bảng và
 *   phát những luồng đã tới hạn.
 *
 *   Nhờ vậy việc "gửi một phần dữ liệu" chỉ là chuyện đặt chu kỳ:
 *     - Bay bình thường : chỉ HEARTBEAT 1 Hz + ATTITUDE 10 Hz  (~200 B/s)
 *     - Đang chỉnh PID  : bật thêm PID 100 Hz                  (~2 kB/s)
 *     - Gỡ lỗi qua USB  : bật hết, USB CDC thừa băng thông
 *
 * THÊM LUỒNG MỚI: xem hướng dẫn ở đầu tlm_messages.h.
 */
#ifndef TLM_STREAM_H
#define TLM_STREAM_H

#include "fc_state.h"
#include "tlm_messages.h"
#include "tlm_protocol.h"

/** Hàm đóng gói: đọc trạng thái, ghi payload vào dst. */
typedef void (*tlm_pack_fn)(const fc_t *fc, void *dst);

typedef struct {
    uint8_t     id;             /**< tlm_msg_id_t                     */
    uint8_t     size;           /**< kích thước payload (byte)        */
    uint16_t    period_ms;      /**< chu kỳ phát; 0 = tắt             */
    tlm_pack_fn pack;
    uint32_t    next_due_ms;    /**< nội bộ: mốc phát kế tiếp         */
} tlm_stream_t;

/** Cấu hình sẵn cho từng tình huống sử dụng. */
typedef enum {
    TLM_PROFILE_SILENT = 0,  /**< tắt hết, chỉ còn heartbeat 1 Hz     */
    TLM_PROFILE_FLIGHT,      /**< nhẹ, hợp cho đường telemetry 2.4G   */
    TLM_PROFILE_TUNING,      /**< thêm luồng PID tốc độ cao           */
    TLM_PROFILE_DEBUG,       /**< bật tất cả, dùng khi cắm USB        */
    TLM_PROFILE_COUNT
} tlm_profile_t;

/** Khởi tạo bảng luồng và parser lệnh. Gọi sau fc_state_init(). */
void tlm_stream_init(void);

/**
 * Quét bảng và phát các luồng tới hạn.
 * @param now_ms  mốc thời gian hiện tại (HAL_GetTick())
 * @return        số gói đã đẩy vào bộ đệm gửi
 */
uint8_t tlm_stream_update(uint32_t now_ms);

/** Đặt chu kỳ cho một luồng. period_ms = 0 để tắt. */
bool tlm_stream_set_period(uint8_t msg_id, uint16_t period_ms);

/** Áp dụng một bộ cấu hình dựng sẵn. */
void tlm_stream_apply_profile(tlm_profile_t profile);

/** Gửi ngay một bản tin, bỏ qua lịch (dùng cho sự kiện đột xuất). */
bool tlm_stream_send_now(uint8_t msg_id);

/** Gửi thông báo dạng chữ. severity: 0 = info, 1 = warn, 2 = error. */
bool tlm_stream_send_text(uint8_t severity, const char *text);

/**
 * Rút byte uplink từ tlm_port, nạp vào parser và thực thi lệnh nhận được.
 * Gọi đều đặn trong vòng lặp chính, ngay cạnh tlm_stream_update().
 * @return số lệnh đã thực thi trong lần gọi này
 */
uint8_t tlm_stream_rx_update(void);

/**
 * Xử lý một khung lệnh vừa nhận hợp lệ từ máy tính.
 * @return true nếu lệnh được nhận và thực thi
 */
bool tlm_stream_handle_command(const tlm_parser_t *p);

/** Thống kê parser uplink (frames_ok / crc_errors / overruns). */
const tlm_parser_t *tlm_stream_rx_stats(void);

#endif /* TLM_STREAM_H */
