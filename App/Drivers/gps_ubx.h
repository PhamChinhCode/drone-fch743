/**
 * @file    gps_ubx.h
 * @brief   Driver GPS MicoAir MG-F10-A (u-blox NEO-F10N) — UBX trên UART7.
 *
 * ĐẤU NỐI (FCH743_V1.0):
 *   PE7  UART7_RX  <- TX của module
 *   PE8  UART7_TX  -> RX của module (cần để gửi cấu hình)
 *   5 V và GND chung. Dây SDA/SCL của la bàn IST8310 đi I2C1, không qua đây.
 *
 * GIAO THỨC:
 *   Khung UBX:  0xB5 0x62 class id len_lo len_hi payload... ck_a ck_b
 *   Checksum Fletcher 8 bit tính từ `class` tới hết payload.
 *
 *   Driver chỉ dùng MỘT bản tin: UBX-NAV-PVT (0x01 0x07, 92 byte) — một gói
 *   đủ vị trí, vận tốc NED, sai số ước lượng, số vệ tinh và kiểu fix.
 *
 * CẤU HÌNH — UBX-CFG-VALSET, chỉ ghi lớp RAM:
 *   UART1 baud = GPS_BAUD, UART1 vào/ra UBX, TẮT NMEA, NAV-PVT mỗi chu kỳ,
 *   chu kỳ đo GPS_MEAS_PERIOD_MS, mô hình động học GPS_DYNMODEL.
 *   Không ghi flash module: cắm sang FC khác vẫn là module nguyên bản.
 *
 * DÒ BAUD:
 *   Module mới xuất xưởng ở 115200, nhưng nếu từng cắm qua ArduPilot/PX4 thì
 *   có thể đã bị lưu sang baud khác. Driver thử lần lượt từng baud trong
 *   danh sách: ở mỗi baud gửi cấu hình (trong đó có lệnh đổi baud về
 *   GPS_BAUD), rồi chuyển UART về GPS_BAUD, gửi lại cấu hình và chờ một khung
 *   UBX hợp lệ. Có khung là xong; không có thì sang baud kế tiếp.
 *
 *   Đang chạy mà mất NAV-PVT quá GPS_RECONFIG_MS (module mất điện rồi khởi
 *   động lại về cấu hình gốc chẳng hạn) thì dò lại từ đầu.
 *
 * KHÔNG CHẶN:
 *   Gửi cấu hình bằng HAL_UART_Transmit_IT, nhận bằng DMA vòng tròn. Mọi việc
 *   khác chạy trong gps_ubx_update() ở vòng lặp chính, kể cả khởi động lại
 *   DMA sau lỗi UART — ISR chỉ bật một cờ.
 */
#ifndef GPS_UBX_H
#define GPS_UBX_H

#include "fc_types.h"
#include "fc_config.h"

typedef enum {
    GPS_STATE_OFF = 0,     /**< chưa init hoặc GPS_ENABLE = 0            */
    GPS_STATE_PROBE,       /**< đang dò baud / gửi cấu hình              */
    GPS_STATE_RUNNING      /**< đang nhận NAV-PVT đều                    */
} gps_state_t;

/**
 * Khởi động DMA UART7 và bắt đầu dò. Gọi sau MX_UART7_Init() và
 * fc_time_init(). KHÔNG chặn.
 * @return false nếu HAL từ chối khởi động DMA.
 */
bool gps_ubx_init(void);

/**
 * Rút byte mới, phân tích, chạy máy trạng thái dò/cấu hình. Gọi mỗi vòng
 * lặp chính. Cập nhật thẳng vào g_fc.gps.
 * @return true nếu vừa nhận xong một gói NAV-PVT.
 */
bool gps_ubx_update(uint32_t now_ms);

gps_state_t  gps_ubx_state(void);
const char  *gps_ubx_state_name(void);

/** Baud UART7 đang dùng. */
uint32_t gps_ubx_baud(void);

/* --- Thống kê phục vụ chẩn đoán ---------------------------------------- */

uint32_t gps_ubx_bytes_received(void);
uint32_t gps_ubx_frames_ok(void);        /**< mọi khung UBX đúng checksum */
uint32_t gps_ubx_checksum_errors(void);
uint32_t gps_ubx_ack_count(void);        /**< ACK-ACK cho CFG-VALSET      */
uint32_t gps_ubx_nak_count(void);        /**< ACK-NAK cho CFG-VALSET      */
uint32_t gps_ubx_uart_errors(void);
uint32_t gps_ubx_config_sent(void);      /**< số lần đã gửi CFG-VALSET    */

/** Bao lâu kể từ gói NAV-PVT gần nhất, ms. UINT32_MAX = chưa có gói nào. */
uint32_t gps_ubx_age_ms(uint32_t now_ms);

/** Gọi từ HAL_UART_ErrorCallback khi UART7 báo lỗi. Chỉ bật cờ. */
void gps_ubx_uart_error_isr(void);

#endif /* GPS_UBX_H */
