/**
 * @file    mag_i2c.h
 * @brief   Driver từ kế rời trên I2C1 — dò và hỗ trợ BỐN chip: QMC5883P
 *          (0x2C, chip trên bo), QMC5883L (0x0D), HMC5883L (0x1E, Honeywell
 *          thật) và IST8310 (0x0E, la bàn trên module GPS MG-F10-A).
 *
 * CHỌN CHIP (fc_config.h, MAG_I2C_USE_GPS_MAG):
 *   1 -> CHỈ dò IST8310 trên GPS. Không thấy thì báo lỗi, KHÔNG lùi về chip
 *        trên bo — bộ hiệu chuẩn và trục thuộc về đúng một chip.
 *   0 -> dò ba chip trên bo như trước, bỏ qua IST8310.
 *
 * ĐẤU NỐI (FCH743_V1.0):
 *   Dùng CHUNG bus I2C1 với BMP388: PB8 I2C1_SCL, PB7 I2C1_SDA, 400 kHz.
 *   Không cần dây riêng — chỉ cần điện trở kéo lên đã có sẵn cho BMP388.
 *
 * VÌ SAO DÒ BA CHIP:
 *   Module bán ngoài ghi nhãn "HMC5883L" gần như không bao giờ là HMC5883L
 *   thật. Bo này đã quét bus và thấy chip nằm ở 0x2C, tức QMC5883P — không
 *   phải HMC5883L (0x1E) mà cũng không phải QMC5883L (0x0D) như dự đoán
 *   ban đầu. Ba chip có bản đồ thanh ghi khác nhau hoàn toàn, và đọc sai
 *   bản đồ cho ra SỐ RÁC chứ không báo lỗi, nên driver dò cả ba địa chỉ
 *   lúc khởi động thay vì tin vào cái nhãn.
 *
 *   QMC5883P là chip đang dùng thật; hai chip kia giữ lại để đổi module
 *   sau này không phải viết lại driver.
 *
 * CHIA SẺ BUS VỚI BMP388:
 *   HAL lưu hi2c->Devaddress ngay khi HAL_I2C_Mem_Read_IT() bắt đầu và không
 *   xoá nó cho tới lượt kế tiếp, nên callback trong drv_hal_callbacks.c phân
 *   phối đúng theo địa chỉ mà không cần module trọng tài nào. Cả hai driver
 *   đều khởi phát lượt đọc từ VÒNG LẶP CHÍNH (không phải ISR) nên không có
 *   tranh chấp preemption — ai gọi HAL_I2C_Mem_Read_IT() trước thì HAL nhận,
 *   người sau nhận HAL_BUSY và tự bỏ lượt, thử lại lần gọi sau.
 *   Xem App/Docs/KE_HOACH_LA_BAN_I2C.md mục "Việc thật sự khó".
 *
 * CÁCH HOẠT ĐỘNG — khuôn y hệt bmp388.c:
 *   1. mag_i2c_init() dò 0x1E, 0x0D rồi 0x2C, xác nhận chip ID, ghi cấu
 *      hình đo. Hàm CHẶN vài ms, chạy một lần lúc khởi động, SAU bmp388_init().
 *   2. mag_i2c_update() gọi từ vòng lặp chính, phát lệnh đọc I2C ở chế độ
 *      NGẮT rồi trả về ngay.
 *   3. Ngắt I2C1 báo xong -> driver chỉ bật một cờ, không tính gì trong ISR.
 *   4. Lần gọi mag_i2c_update() kế tiếp đổi thang, hiệu chuẩn sắt cứng/mềm
 *      trong hệ cảm biến (dùng chung tham số mag_offset_x_g..z_g và
 *      mag_scale_x..z với nguồn SHUB), rồi xoay sang hệ thân và ghi vào
 *      g_fc.mag — ĐÚNG công thức lsm6dsv_mag_update() đang dùng, để phần
 *      phía sau (DBG_MODE_MAG,
 *      DBG_MODE_MAGCAL, EKF sau này) không cần biết nguồn nào đang chạy.
 */
#ifndef MAG_I2C_H
#define MAG_I2C_H

#include "fc_types.h"
#include "fc_config.h"

/* ==========================================================================
 * Trạng thái driver
 * ========================================================================== */

typedef enum {
    MAG_I2C_STATE_UNINIT = 0,  /**< chưa gọi init                           */
    MAG_I2C_STATE_RUNNING,     /**< đang đo liên tục                        */
    MAG_I2C_STATE_ERROR        /**< không dò được chip nào, không dùng được */
} mag_i2c_state_t;

typedef enum {
    MAG_I2C_VARIANT_NONE = 0,  /**< chưa dò ra, hoặc dò thất bại             */
    MAG_I2C_VARIANT_HMC5883L,  /**< @ 0x1E, xác nhận qua ID 'H','4','3'      */
    MAG_I2C_VARIANT_QMC5883L,  /**< @ 0x0D, xác nhận qua chip id 0xFF        */
    MAG_I2C_VARIANT_QMC5883P,  /**< @ 0x2C, chip id 0x80 — la bàn trên bo    */
    MAG_I2C_VARIANT_IST8310    /**< @ 0x0E/0x0C, WAI 0x10 — la bàn trên GPS  */
} mag_i2c_variant_t;

/* ==========================================================================
 * API
 * ========================================================================== */

/**
 * Dò địa chỉ 0x1E, 0x0D rồi 0x2C, xác nhận chip và ghi cấu hình đo. Hàm
 * CHẶN khoảng vài ms. Gọi sau bmp388_init() để bus I2C1 đã rảnh, và sau
 * MX_I2C1_Init() + fc_time_init().
 * @return true nếu dò được một trong ba chip và ghi cấu hình thành công.
 */
bool mag_i2c_init(void);

/**
 * Xử lý mẫu vừa nhận rồi phát lệnh đọc kế tiếp. KHÔNG chặn.
 * Gọi đều đặn trong vòng lặp chính, tối thiểu MAG_I2C_UPDATE_RATE_HZ lần/giây.
 * Cập nhật thẳng vào g_fc.mag.
 * @return true nếu vừa xử lý xong một mẫu mới
 */
bool mag_i2c_update(uint32_t now_us);

/** Trạng thái hiện tại của driver. */
mag_i2c_state_t mag_i2c_get_state(void);

/** Chip nào vừa dò được lúc init. MAG_I2C_VARIANT_NONE nếu init thất bại. */
mag_i2c_variant_t mag_i2c_variant(void);

/** Tên chip dạng chữ, dùng cho console lúc khởi động. */
const char *mag_i2c_variant_name(void);

/* --- Thống kê phục vụ chẩn đoán ---------------------------------------- */

uint32_t mag_i2c_errors(void);
uint32_t mag_i2c_stale_reads(void);

/**
 * Số lần GỌI đọc gặp bus I2C1 đang bận (BMP388 đang truyền).
 *
 * ⚠️ KHÔNG phải số mẫu bị mất. Khi gặp bus bận, driver giữ nguyên mốc thời
 * gian và thử lại ở vòng lặp chính kế tiếp (dưới 1 ms sau) thay vì đợi hết
 * chu kỳ 20 ms — nên MỘT mẫu có thể tốn nhiều lần thử. Đo trên bo này:
 * khoảng 9 lần thử cho mỗi mẫu, tức ~450/giây ở nhịp 50 Hz.
 *
 * Cách đọc đúng: chia cho nhịp mẫu ra "số lần thử trung bình mỗi mẫu". Con
 * số đó tăng dần theo thời gian nghĩa là bus ngày càng đông; muốn biết có
 * MẤT mẫu thật hay không thì xem cột count có tăng đủ nhịp không.
 *
 * (Khác với bmp388_bus_lost(): BMP388 đóng dấu mốc vô điều kiện nên bộ đếm
 * của nó đúng là "số chu kỳ đã bỏ". Nó hỏi vòng 100 Hz trên ODR 50 Hz nên
 * bỏ một lượt vẫn kịp bắt mẫu ở lượt sau, không cần thử lại nhanh.)
 */
uint32_t mag_i2c_bus_lost(void);

/**
 * Quét toàn bộ dải địa chỉ I2C1 (0x08..0x77) và in ra mọi địa chỉ có trả
 * lời. Gọi khi mag_i2c_init() thất bại — cùng vai trò với lsm6dsv_mag_dump()
 * ở nhánh SHUB: trả lời câu hỏi "không thấy chip" là do dây, do nguồn, hay
 * do chip nằm ở địa chỉ khác dự đoán.
 *
 * BMP388 ở 0x77 chính là PHÉP THỬ ĐỐI CHỨNG: nếu bản quét không thấy cả nó
 * thì lỗi nằm ở bus/điện trở kéo lên chứ không phải ở module từ kế.
 *
 * Hàm CHẶN (tối đa ~0,5 giây) và in trực tiếp qua dbg_println. Chỉ dùng lúc
 * khởi động, không bao giờ gọi trong vòng lặp chính.
 */
void mag_i2c_scan_dump(void);

/* --- Hàm gọi từ ngắt, xem App/Drivers/drv_hal_callbacks.c --------------- */

/** Gọi khi I2C1 đọc xong khối thanh ghi CỦA TỪ KẾ (đã lọc theo Devaddress). */
void mag_i2c_complete_isr(void);

/** Gọi khi I2C1 ghi xong lệnh đo của IST8310 (lọc theo Devaddress). */
void mag_i2c_write_complete_isr(void);

/** Gọi khi I2C1 báo lỗi trong lúc đang đọc từ kế. */
void mag_i2c_error_isr(void);

#endif /* MAG_I2C_H */
