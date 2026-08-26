/**
 * @file    dbg_console.h
 * @brief   Console dạng chữ để xem số liệu cảm biến bằng mắt.
 *
 * KHÁC GÌ tlm_stream?
 *   tlm_* phát gói nhị phân cho phần mềm trên máy tính đọc. Module này in ra
 *   chữ thường, mở PuTTY / Tera Term / terminal của STM32CubeIDE là đọc được
 *   ngay, không cần công cụ riêng. Dùng lúc bring-up phần cứng.
 *
 * KHÔNG DÙNG printf:
 *   Toàn bộ định dạng số viết tay bằng số nguyên. Lý do:
 *     - newlib-nano mặc định KHÔNG hỗ trợ %f (phải thêm cờ -u _printf_float),
 *       in ra sẽ thành chuỗi rỗng mà không báo lỗi gì.
 *     - printf của newlib cấp phát động và ngốn vài KB stack — không hợp với
 *       firmware bay.
 *   Cách viết tay ở đây cho ra kết quả xác định, tốn khoảng 60 byte stack.
 *
 * KHÔNG CHẶN:
 *   Chuỗi được chép vào ring buffer rồi đẩy đi bằng DMA (nếu UART có cấu hình
 *   DMA) hoặc bằng ngắt. Nếu đệm đầy thì dòng mới bị bỏ, chương trình không
 *   bao giờ dừng chờ. Gọi được từ vòng lặp chính; KHÔNG gọi trong ISR ưu tiên
 *   cao vì hàm định dạng mất vài micro giây.
 */
#ifndef DBG_CONSOLE_H
#define DBG_CONSOLE_H

#include "fc_types.h"
#include "fc_config.h"
#include "main.h"

/** Nội dung in ra định kỳ. */
typedef enum {
    DBG_MODE_OFF = 0,
    DBG_MODE_IMU,       /**< gyro/accel theo đơn vị vật lý, có tiêu đề cột  */
    DBG_MODE_IMU_RAW,   /**< số thô 16-bit — dùng để kiểm chứng chiều trục  */
    DBG_MODE_IMU_CSV,   /**< CSV cho phần mềm vẽ đồ thị                     */
    DBG_MODE_FLOW,      /**< MTF-01P: optical flow + khoảng cách          */
    DBG_MODE_FLOW_RAW,  /**< hexdump byte thô từ UART4 — dùng để dò giao thức */
    DBG_MODE_BARO,      /**< BMP388: áp suất, nhiệt độ, độ cao             */
    DBG_MODE_RC,        /**< ELRS/CRSF: kênh thô, cần đã chuẩn hoá, chất lượng sóng */
    DBG_MODE_RC_RAW,    /**< hexdump byte thô từ USART2 — dùng để dò baud   */
    DBG_MODE_ARM,       /**< máy trạng thái arm, công tắc, lý do chặn arm   */
    DBG_MODE_MOTOR,     /**< giá trị DShot đang phát ra 4 ESC               */
    DBG_MODE_FLOWCAL,   /**< hiệu chuẩn FLOW_RAD_PER_COUNT: đếm dồn flow  */
    DBG_MODE_VEL,       /**< vận tốc ngang từ flow, đã bù quay            */
    DBG_MODE_POSHOLD,   /**< giữ vận tốc: vận tốc thật vs mong muốn, góc ra */
    DBG_MODE_ANGLE,     /**< vòng góc: góc mục tiêu vs góc thật, chế độ bay */
    DBG_MODE_PID,       /**< chỉnh PID: setpoint vs gyro, P/I/D, bão hoà   */
    DBG_MODE_EST,       /**< kết quả EKF: góc, độ cao, bias, độ tin cậy     */
    DBG_MODE_STATUS,    /**< trạng thái hệ thống, cờ lỗi, thời gian vòng lặp */
    DBG_MODE_IMU2,      /**< LSM6DSV tren SPI3: gyro/accel, hz, |a|/g       */
    DBG_MODE_IMU2_RAW,  /**< so tho LSM6DSV - dung de xac dinh chieu truc   */
    DBG_MODE_LOG,       /**< Blackbox: trang thai the SD va tien do xa   */
    DBG_MODE_MAG,
    DBG_MODE_MAGCAL,    /**< HIEU CHUAN tu ke - cong cu GIAI DOAN 4       */       /**< QMC6309 qua sensor hub - GIAI DOAN 3        */
    DBG_MODE_AXISCAL,   /**< TU NHAN chieu truc IMU2 - cong cu GIAI DOAN 2A  */
    DBG_MODE_IMU_CMP,   /**< SO SANH hai IMU + nen nhieu - cong cu GIAI DOAN 2 */
    DBG_MODE_COUNT
} dbg_mode_t;

/**
 * Gắn console vào một UART.
 * @param huart  handle do CubeMX sinh, ví dụ &huart1 hoặc &huart3.
 *               Tự dùng DMA nếu UART đó có cấu hình DMA TX, không thì dùng ngắt.
 */
void dbg_console_init(UART_HandleTypeDef *huart);

void      dbg_console_set_mode(dbg_mode_t mode);
dbg_mode_t dbg_console_get_mode(void);

/** Đặt số dòng in ra mỗi giây (1..1000). */
void dbg_console_set_rate(uint16_t hz);

/** Gọi đều đặn trong vòng lặp chính. Tự in khi tới hạn và đẩy đệm đi. */
void dbg_console_update(uint32_t now_ms);

/* --- In thủ công ------------------------------------------------------- */

/** In một chuỗi (không tự thêm xuống dòng). */
void dbg_puts(const char *s);

/** In một chuỗi kèm CR LF. */
void dbg_println(const char *s);

/** In "nhãn = giá trị" cho số nguyên. */
void dbg_print_int(const char *label, int32_t value);

/** In mot gia tri dang hex: "label = 0xNN". */
void dbg_print_hex(const char *label, uint32_t value, uint8_t digits);

/** In "nhãn = giá trị" cho số thực, với số chữ số thập phân chỉ định. */
void dbg_print_float(const char *label, float value, uint8_t decimals);

/** true nếu console đang chiếm UART này (dùng để phân luồng callback). */
bool dbg_console_owns(const UART_HandleTypeDef *huart);

/** Số byte đã bị bỏ vì đệm đầy — nếu tăng thì đang in quá nhanh. */
uint32_t dbg_console_dropped(void);

/* --- Gọi từ ngắt, xem drv_hal_callbacks.c ------------------------------ */
void dbg_console_tx_complete_isr(void);

#endif /* DBG_CONSOLE_H */
