/**
 * @file    vib_stream.h
 * @brief   Phát gyro + accel CHƯA LỌC ở nhịp gốc 8 kHz qua USB CDC để phân
 *          tích phổ rung.
 *
 * VÌ SAO CẦN RIÊNG MỘT LUỒNG:
 *   Mọi đường log khác đều lấy mẫu THƯA từ số liệu 8 kHz (flashlog 500 Hz,
 *   TLM_MSG_IMU tối đa 1 kHz), và lấy thưa không có lọc chống chồng phổ thì
 *   rung ở tần số cao hiện ra thành một đỉnh giả ở tần số thấp. Luồng này gửi
 *   ĐỦ từng mẫu một, nên phổ đúng tới 4 kHz.
 *
 *   Cũng không lấy gyro_filtered_dps: LPF 100 Hz + notch 220 Hz đã xoá chính
 *   thứ cần đo.
 *
 * CHỈ CHẠY TRÊN USB CDC:
 *   ~110 kB/s — hơn trần 92 kB/s của UART 921600. Đổi port khác USB thì luồng
 *   tự tắt. Nghĩa là chỉ đo được khi cắm dây: trên bàn hoặc buộc cố định.
 *
 * LUỒNG DỮ LIỆU:
 *   ngắt SPI (8 kHz) -> vib_stream_push() -> vòng đệm trong DTCM
 *   vòng lặp chính   -> vib_stream_update() -> TLM_MSG_VIB 16 mẫu/khung
 *
 *   USB không rút kịp thì vòng đệm đầy và NGẮT BỎ MẪU MỚI (không chặn, không
 *   ghi đè), đếm vào ring_drops. Máy tính thấy lỗ hổng qua idx0.
 */
#ifndef VIB_STREAM_H
#define VIB_STREAM_H

#include "fc_types.h"

/**
 * Nạp một mẫu. Gọi từ ngắt IMU cho MỌI mẫu, kể cả khi luồng đang tắt — lúc
 * đó hàm chỉ kiểm một cờ rồi trả về.
 *
 * @param gyro_dps     hệ thân, đã trừ bias, CHƯA lọc
 * @param accel_mps2   hệ thân, CHƯA lọc
 * @param t_us         micros() lúc đọc mẫu
 */
void vib_stream_push(const vec3f_t *gyro_dps, const vec3f_t *accel_mps2,
                     uint32_t t_us);

/** Gọi mỗi vòng lặp chính. Đóng khung và đẩy xuống tlm_port. */
void vib_stream_update(void);

/** Bật luồng. false nếu telemetry đang không ở USB. */
bool vib_stream_start(void);
void vib_stream_stop(void);

bool     vib_stream_enabled(void);
uint32_t vib_stream_frames_sent(void);
uint32_t vib_stream_ring_drops(void);

#endif /* VIB_STREAM_H */
