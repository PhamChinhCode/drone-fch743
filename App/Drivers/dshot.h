/**
 * @file    dshot.h
 * @brief   Driver DShot cho 4 ESC, dùng TIM1 + DMA burst.
 *
 *   Tốc độ đặt bằng DSHOT_BITRATE_HZ trong fc_config.h (150/300/600).
 *   ĐANG DÙNG DShot300 — bo này đã thử thực tế: 600 và 150 đều hỏng,
 *   chỉ 300 chạy ổn. Xem bảng kết quả ở chú thích cạnh hằng số đó.
 *
 * ĐẤU NỐI (FCH743_V1.0):
 *   PE9   TIM1_CH1  -> tín hiệu ESC motor 1
 *   PE11  TIM1_CH2  -> motor 2
 *   PE13  TIM1_CH3  -> motor 3
 *   PE14  TIM1_CH4  -> motor 4
 *   GND chung với ESC. DShot là tín hiệu một chiều, không cần dây về.
 *
 * VÌ SAO DÙNG DMA BURST:
 *   Bốn motor cần bốn dòng xung độc lập, đồng bộ tới từng bit. Cách thẳng
 *   thừng là bốn kênh DMA riêng, nhưng CubeMX chỉ cấp đúng một kênh cho TIM1
 *   (DMA1_Stream2, TIM1_UP) — và đó chính là cách làm đúng.
 *
 *   Chế độ burst của timer cho phép MỘT lượt DMA ghi liên tiếp vào nhiều
 *   thanh ghi: đặt DCR.DBA trỏ vào CCR1 và DCR.DBL = 4, mỗi lần ghi vào
 *   thanh ghi DMAR sẽ lần lượt rơi vào CCR1, CCR2, CCR3, CCR4. Mỗi sự kiện
 *   UPDATE (tức mỗi bit) tiêu thụ đúng 4 word, nạp độ rộng xung cho cả bốn
 *   motor cùng lúc. Một kênh DMA, bốn đường ra, lệch pha bằng 0.
 *
 * BỐ CỤC BỘ ĐỆM (xen kẽ theo bit, KHÔNG phải theo motor):
 *
 *       chỉ số:  0    1    2    3    4    5    6    7   ...
 *              [b0c1 b0c2 b0c3 b0c4 b1c1 b1c2 b1c3 b1c4 ...]
 *                \________________/  \________________/
 *                    bit 0 của          bit 1 của
 *                   cả 4 motor         cả 4 motor
 *
 *   18 nhóm × 4 word = 72 word: 16 bit dữ liệu + 2 bit khoảng lặng cuối
 *   khung (CCR = 0, đường giữ mức thấp) để ESC nhận ra ranh giới khung.
 *
 * MÃ HOÁ BIT (tỉ lệ cố định của chuẩn DShot, không phụ thuộc tốc độ):
 *   bit 0 -> độ rộng xung 37,5 % chu kỳ  (DSHOT_T0H)
 *   bit 1 -> độ rộng xung 75,0 % chu kỳ  (DSHOT_T1H)
 *
 *   Với DShot300 đang dùng: TIM1 @ 240 MHz, ARR = 799 -> 300 kHz, chu kỳ bit
 *   3,333 µs, T0H = 300 (1,25 µs), T1H = 600 (2,50 µs). Cả ba hằng số tính
 *   sẵn từ DSHOT_BITRATE_HZ nên đổi tốc độ chỉ cần sửa một dòng; dshot_init()
 *   ghi ARR theo, và có #error chặn nếu T1H vượt quá ARR.
 *
 * KHUNG DSHOT (16 bit):
 *   [11 bit giá trị][1 bit xin telemetry][4 bit CRC]
 *   CRC = XOR của ba nibble đứng trước. Bit cao phát trước.
 *   Giá trị 0 = dừng motor, 1..47 = lệnh đặc biệt, 48..2047 = mức ga.
 *
 * AN TOÀN:
 *   dshot_update() chỉ phát mức ga khi CẢ HAI điều kiện đúng:
 *   `g_fc.motor.armed` và `g_fc.mode == FC_MODE_ARMED`. Mọi trường hợp khác
 *   đều phát lệnh 0 (dừng motor) — phát đều đặn chứ không phải ngừng phát,
 *   vì ESC cần dòng khung liên tục mới giữ trạng thái sẵn sàng.
 *
 *   Hai cờ này do arming.c điều khiển, và cả hai đều được fc_state_set_mode()
 *   đặt cùng lúc. Kiểm tra cả hai là phòng thủ nhiều lớp: một lần ghi nhầm
 *   vào riêng một cờ không đủ để làm motor quay.
 *
 * CHƯA LÀM:
 *   DShot hai chiều (đọc eRPM ngược từ ESC). Trường g_fc.motor.erpm[] đã có
 *   sẵn chỗ. Việc đó cần đảo chiều chân giữa hai khung và bắt xung bằng
 *   input capture — thay đổi lớn, không nằm trong driver này.
 */
#ifndef DSHOT_H
#define DSHOT_H

#include "fc_types.h"
#include "fc_config.h"

/* ==========================================================================
 * Lệnh đặc biệt (giá trị 0..47)
 *
 * ESC chỉ chấp nhận lệnh khi đang ở trạng thái dừng, và phải nhận được lệnh
 * lặp lại nhiều lần — xem DSHOT_CMD_REPEAT.
 * ========================================================================== */

#define DSHOT_CMD_MOTOR_STOP              0u
#define DSHOT_CMD_BEEP1                   1u   /**< tiếng bíp, dùng để tìm máy bay */
#define DSHOT_CMD_BEEP2                   2u
#define DSHOT_CMD_BEEP3                   3u
#define DSHOT_CMD_BEEP4                   4u
#define DSHOT_CMD_BEEP5                   5u
#define DSHOT_CMD_ESC_INFO                6u
#define DSHOT_CMD_SPIN_DIRECTION_1        7u
#define DSHOT_CMD_SPIN_DIRECTION_2        8u
#define DSHOT_CMD_3D_MODE_OFF             9u
#define DSHOT_CMD_3D_MODE_ON              10u
#define DSHOT_CMD_SETTINGS_REQUEST        11u
#define DSHOT_CMD_SAVE_SETTINGS           12u  /**< ghi vào EEPROM của ESC   */
#define DSHOT_CMD_SPIN_DIRECTION_NORMAL   20u
#define DSHOT_CMD_SPIN_DIRECTION_REVERSED 21u

/** Giá trị lớn nhất còn được coi là lệnh chứ không phải mức ga. */
#define DSHOT_CMD_MAX                     47u

/* ==========================================================================
 * API
 * ========================================================================== */

/**
 * Bật bốn kênh PWM của TIM1 và đưa cả bốn đường về mức thấp.
 * Gọi sau MX_TIM1_Init() và fc_time_init(). KHÔNG chặn.
 *
 * Sau khi gọi, timer chạy tự do ở 600 kHz với CCR = 0, tức bốn đường nằm im
 * ở mức thấp — đúng trạng thái nghỉ của DShot. Chưa có khung nào được phát
 * cho tới lần gọi dshot_update() đầu tiên.
 *
 * @return false nếu HAL không bật được kênh PWM nào đó.
 */
bool dshot_init(void);

/**
 * Dung lai map motor, muc ga day va nhip bit tu g_params.
 *
 * Map motor va muc ga day co tac dung NGAY. Nhip bit thi khong: no chi ghi
 * vao ARR cua TIM1 trong dshot_init(), va doi nhip bit con doi HAI dieu kien
 * nua ma phan mem khong lam duoc - phai RUT PIN CAM LAI cho ESC vi BLHeli_S
 * do giao thuc dung mot lan luc no khoi dong.
 */
void dshot_apply_params(void);

/**
 * Dựng và phát một khung cho cả bốn motor. KHÔNG chặn — nạp DMA rồi trả về.
 * Tự giữ nhịp DSHOT_UPDATE_RATE_HZ nên gọi bao nhiêu lần cũng được; gọi
 * trong vòng lặp chính, SAU arming_update() để thấy trạng thái arm mới nhất.
 *
 * Đọc g_fc.motor.output_norm[] (0..1) và ghi lại g_fc.motor.throttle[].
 *
 * @param now_us  mốc thời gian micro giây, lấy từ micros()
 * @return true nếu vừa phát một khung
 */
bool dshot_update(uint32_t now_us);

/**
 * Xếp hàng một lệnh đặc biệt, phát lặp lại DSHOT_CMD_REPEAT lần cho cả bốn
 * motor. Dùng để đổi chiều quay, bíp tìm máy bay, lưu cài đặt ESC.
 *
 * TỪ CHỐI khi đang arm — đổi chiều quay giữa không trung là tai nạn.
 * THÁO CÁNH QUẠT trước khi dùng hàm này.
 *
 * @param command  1..47, xem danh sách DSHOT_CMD_* ở trên
 * @return false nếu lệnh không hợp lệ hoặc máy bay đang arm
 */
bool dshot_send_command(uint16_t command);

/**
 * Như trên nhưng chỉ gửi cho MỘT motor; ba cái còn lại nhận lệnh dừng.
 * Dùng để đảo chiều riêng một motor mà không đụng ba cái kia.
 * @return false nếu chỉ số sai, lệnh sai, hoặc đang arm
 */
bool dshot_send_command_motor(uint8_t motor, uint16_t command);

/** true khi còn lệnh đang được phát lặp. */
bool dshot_command_busy(void);

/**
 * Chạy chuỗi đảo chiều cho các motor trong bitmask (bit0 = motor 1).
 * Với mỗi motor: gửi SPIN_DIRECTION_REVERSED rồi SAVE_SETTINGS, lần lượt
 * từng cái một. Từ chối khi đang arm.
 * @return false nếu mask rỗng, đang arm, hoặc chuỗi trước chưa xong
 */
bool dshot_reverse_motors(uint8_t mask);

/* --- Quay thử từng motor ------------------------------------------------
 *
 * Dùng để xác nhận thứ tự motor và chiều quay trước khi có khâu trộn.
 * THÁO CÁNH QUẠT. Đây là hàm nguy hiểm nhất trong firmware nên nó có bốn
 * chốt an toàn, và không chốt nào bỏ qua được từ bên ngoài:
 *
 *   1. Từ chối khi đang arm.
 *   2. Chỉ MỘT motor quay tại một thời điểm; ba motor còn lại nhận lệnh 0.
 *   3. Ga bị chặn trần ở DSHOT_TEST_MAX_PERCENT.
 *   4. Tự dừng sau duration_ms, và trần là DSHOT_TEST_MAX_MS. Nếu vòng lặp
 *      chính treo thì dshot_update() ngừng chạy, khung ngừng phát và ESC tự
 *      cắt — không có đường nào để motor quay mãi.
 */

/**
 * Bắt đầu quay thử một motor.
 * @param motor          chỉ số 0..FC_MOTOR_COUNT-1 (0 = motor 1)
 * @param throttle_norm  0..1, bị chặn trần theo DSHOT_TEST_MAX_PERCENT
 * @param duration_ms    thời gian quay, bị chặn trần theo DSHOT_TEST_MAX_MS
 * @return false nếu đang arm hoặc chỉ số motor sai
 */
bool dshot_motor_test_start(uint8_t motor, float throttle_norm,
                            uint16_t duration_ms);

/** Dừng quay thử ngay lập tức. */
void dshot_motor_test_stop(void);

/** Chỉ số motor đang quay thử, hoặc -1 nếu không có. */
int8_t dshot_motor_test_active(void);

/** true khi một lượt DMA đang chạy dở. */
bool dshot_is_busy(void);

/* --- Thống kê phục vụ chẩn đoán ---------------------------------------- */

uint32_t dshot_frames_sent(void);
uint32_t dshot_dma_errors(void);

/** Mã lỗi HAL_DMA_ERROR_* của lần lỗi gần nhất: 1=TE, 2=FE, 4=DME. */
uint32_t dshot_dma_errcode(void);

/** Số lần bỏ nhịp vì khung trước chưa phát xong. Tăng đều nghĩa là DMA kẹt. */
uint32_t dshot_skipped(void);

/* --- Hàm gọi từ ngắt, xem App/Drivers/drv_hal_callbacks.c --------------- */

/** Gọi khi DMA1_Stream2 phát xong toàn bộ khung. */
void dshot_dma_complete_isr(void);

/** Gọi khi DMA của TIM1 báo lỗi. */
void dshot_dma_error_isr(void);

#endif /* DSHOT_H */
