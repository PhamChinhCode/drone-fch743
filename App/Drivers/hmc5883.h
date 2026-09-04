/**
 * @file    hmc5883.h
 * @brief   Bản đồ thanh ghi từ kế Honeywell HMC5883L (chip THẬT, không phải
 *          clone QMC5883L hay gắn trên module GY-271 đời mới).
 *
 * NGUỒN:
 *   Bản đồ thanh ghi phổ biến, dùng chung bởi hầu hết driver mã nguồn mở cho
 *   HMC5883L (Adafruit, SparkFun, v.v.) — đây là chip đã ngừng sản xuất và
 *   tài liệu chính thức của Honeywell không có số hiệu tài liệu cố định như
 *   datasheet QST dùng cho qmc6309.h. CHƯA đối chiếu với bản PDF gốc trong
 *   dự án này. Nếu có datasheet chính thức, nên so lại trước khi tin tuyệt
 *   đối các giá trị dưới đây — nhất là bảng chuyển gain sang LSB/Gauss.
 *
 *   Cách tự kiểm khi chạy: từ trường Trái Đất là 0,25 - 0,65 G. Nếu
 *   |B| tính ra ngoài dải đó nhiều thì rất có thể bản đồ ở đây sai, không
 *   phải cảm biến hỏng — xem lại thứ tự byte trước khi nghi phần cứng.
 */
#ifndef HMC5883_H
#define HMC5883_H

#include "fc_types.h"

/* ==========================================================================
 * Địa chỉ I2C — CỐ ĐỊNH, không có chân chọn địa chỉ nào.
 * ========================================================================== */

#define HMC_I2C_ADDR_7BIT      0x1Eu

/* ==========================================================================
 * Bản đồ thanh ghi
 * ========================================================================== */

#define HMC_REG_CONFIG_A       0x00u   /**< so mau trung binh + data rate    */
#define HMC_REG_CONFIG_B       0x01u   /**< gain                             */
#define HMC_REG_MODE            0x02u
#define HMC_REG_DATA_X_MSB     0x03u   /**< dau khoi 6 byte du lieu           */
#define HMC_REG_DATA_X_LSB     0x04u
#define HMC_REG_DATA_Z_MSB     0x05u   /**< thu tu la X, Z, Y — KHONG phai XYZ */
#define HMC_REG_DATA_Z_LSB     0x06u
#define HMC_REG_DATA_Y_MSB     0x07u
#define HMC_REG_DATA_Y_LSB     0x08u
#define HMC_REG_STATUS         0x09u
#define HMC_REG_ID_A           0x0Au   /**< 'H' = 0x48                       */
#define HMC_REG_ID_B           0x0Bu   /**< '4' = 0x34                       */
#define HMC_REG_ID_C           0x0Cu   /**< '3' = 0x33                       */

#define HMC_ID_A_VALUE          0x48u
#define HMC_ID_B_VALUE          0x34u
#define HMC_ID_C_VALUE          0x33u

/**
 * Số byte đọc mỗi chu kỳ: 0x03..0x09 liên tiếp (6 byte dữ liệu + STATUS).
 * Khác thứ tự với BMP388/QMC5883L — ở đây STATUS nằm SAU dữ liệu.
 */
#define HMC_BURST_LEN           7u
#define HMC_BURST_STATUS_IDX    6u     /**< vị trí byte STATUS trong burst   */

/* ==========================================================================
 * STATUS (0x09)
 * ========================================================================== */

#define HMC_STATUS_RDY          0x01u  /**< dữ liệu mới đã ghi đủ cả 6 byte  */
#define HMC_STATUS_LOCK         0x02u

/* ==========================================================================
 * CONFIG_A (0x00)
 *
 *   bit 6..5  MA   so mau trung binh (00=1, 01=2, 10=4, 11=8)
 *   bit 4..2  DO   toc do ra (100 = 15 Hz, mac dinh)
 *   bit 1..0  MS   che do do (00 = binh thuong)
 * ========================================================================== */

#define HMC_CONFIG_A_MA_SHIFT   5
#define HMC_MA_1                0x00u
#define HMC_MA_2                0x01u
#define HMC_MA_4                0x02u
#define HMC_MA_8                0x03u

#define HMC_CONFIG_A_DO_SHIFT   2
#define HMC_DO_0_75HZ           0x00u
#define HMC_DO_1_5HZ            0x01u
#define HMC_DO_3HZ               0x02u
#define HMC_DO_7_5HZ            0x03u
#define HMC_DO_15HZ             0x04u
#define HMC_DO_30HZ             0x05u
#define HMC_DO_75HZ             0x06u

#define HMC_MS_NORMAL           0x00u

/* ==========================================================================
 * CONFIG_B (0x01) — gain
 *
 *   bit 7..5  GN
 * ========================================================================== */

#define HMC_CONFIG_B_GN_SHIFT   5
#define HMC_GN_0_88GA           0x00u  /**< 1370 LSB/Gauss                   */
#define HMC_GN_1_3GA            0x01u  /**< 1090 LSB/Gauss, mac dinh chip     */
#define HMC_GN_1_9GA            0x02u  /**<  820 LSB/Gauss                   */
#define HMC_GN_2_5GA            0x03u  /**<  660 LSB/Gauss                   */
#define HMC_GN_4_0GA            0x04u  /**<  440 LSB/Gauss                   */
#define HMC_GN_4_7GA            0x05u  /**<  390 LSB/Gauss                   */
#define HMC_GN_5_6GA            0x06u  /**<  330 LSB/Gauss                   */
#define HMC_GN_8_1GA            0x07u  /**<  230 LSB/Gauss                   */

#define HMC_LSB_PER_GAUSS_0_88GA 1370.0f
#define HMC_LSB_PER_GAUSS_1_3GA  1090.0f
#define HMC_LSB_PER_GAUSS_1_9GA   820.0f
#define HMC_LSB_PER_GAUSS_2_5GA   660.0f
#define HMC_LSB_PER_GAUSS_4_0GA   440.0f
#define HMC_LSB_PER_GAUSS_4_7GA   390.0f
#define HMC_LSB_PER_GAUSS_5_6GA   330.0f
#define HMC_LSB_PER_GAUSS_8_1GA   230.0f

/* ==========================================================================
 * MODE (0x02)
 * ========================================================================== */

#define HMC_MODE_CONTINUOUS     0x00u
#define HMC_MODE_SINGLE         0x01u
#define HMC_MODE_IDLE           0x02u

/**
 * Mã báo tràn dải đo trên một trục — datasheet gọi là "-4096" nhưng đó là
 * số 16 bit bù hai 0xF000 chứ không phải giá trị thật.
 */
#define HMC_OVERFLOW_CODE       ((int16_t)0xF000)

#endif /* HMC5883_H */
