/**
 * @file    qmc5883.h
 * @brief   Bản đồ thanh ghi từ kế QST QMC5883L — chip THẬT bên trong đa số
 *          module bán ngoài chợ ghi nhãn "HMC5883L"/"GY-271" đời mới.
 *
 * NGUỒN:
 *   Bản đồ thanh ghi phổ biến, khớp giữa nhiều driver mã nguồn mở (DFRobot
 *   QMC5883LCompass, MechaSolution, v.v.). CHƯA đối chiếu với bản PDF gốc
 *   của QST trong dự án này — khác với qmc6309.h, chip đó có datasheet chính
 *   thức mục lục rõ ràng mà QMC5883L (đời cũ hơn, phổ biến trong module rẻ)
 *   thì khó tìm bản chính thức trên mạng. Nếu có datasheet thật, đối chiếu
 *   lại trước khi tin tuyệt đối, nhất là hai thanh ghi CTRL1/CTRL2.
 *
 *   KHÔNG nhầm với QMC6309 (App/Drivers/qmc6309.h) — hai chip khác hãng,
 *   khác bản đồ thanh ghi hoàn toàn dù tên gần giống nhau.
 *
 *   Cách tự kiểm khi chạy: từ trường Trái Đất là 0,25 - 0,65 G. |B| ra
 *   ngoài dải đó nhiều thì khả năng cao bản đồ ở đây sai, không phải cảm
 *   biến hỏng.
 */
#ifndef QMC5883_H
#define QMC5883_H

#include "fc_types.h"

/* ==========================================================================
 * Địa chỉ I2C — CỐ ĐỊNH.
 * ========================================================================== */

#define QMC5883_I2C_ADDR_7BIT   0x0Du

/* ==========================================================================
 * Bản đồ thanh ghi
 * ========================================================================== */

#define QMC5883_REG_DATA_X_LSB  0x00u  /**< dau khoi 6 byte, LSB TRUOC, X-Y-Z */
#define QMC5883_REG_DATA_X_MSB  0x01u
#define QMC5883_REG_DATA_Y_LSB  0x02u
#define QMC5883_REG_DATA_Y_MSB  0x03u
#define QMC5883_REG_DATA_Z_LSB  0x04u
#define QMC5883_REG_DATA_Z_MSB  0x05u
#define QMC5883_REG_STATUS      0x06u
#define QMC5883_REG_TEMP_LSB    0x07u
#define QMC5883_REG_TEMP_MSB    0x08u
#define QMC5883_REG_CTRL1       0x09u
#define QMC5883_REG_CTRL2       0x0Au
#define QMC5883_REG_SET_RESET   0x0Bu  /**< PHAI ghi 0x01, khong tu dat      */
#define QMC5883_REG_CHIP_ID     0x0Du

#define QMC5883_CHIP_ID_VALUE   0xFFu
#define QMC5883_SET_RESET_VALUE 0x01u

/**
 * Số byte đọc mỗi chu kỳ: 0x00..0x06 liên tiếp (6 byte dữ liệu + STATUS).
 */
#define QMC5883_BURST_LEN         7u
#define QMC5883_BURST_STATUS_IDX  6u   /**< vị trí byte STATUS trong burst   */

/* ==========================================================================
 * STATUS (0x06)
 * ========================================================================== */

#define QMC5883_STATUS_DRDY      0x01u
#define QMC5883_STATUS_OVL       0x02u
#define QMC5883_STATUS_DOR       0x04u  /**< bo mau vi doc khong kip          */

/* ==========================================================================
 * CTRL1 (0x09)
 *
 *   bit 7..6  OSR    oversampling (00=512 nhieu thap nhat .. 11=64)
 *   bit 5..4  RNG    dai do (00 = 2G, 01 = 8G)
 *   bit 3..2  ODR    toc do ra
 *   bit 1..0  MODE
 * ========================================================================== */

#define QMC5883_CTRL1_MODE_MASK  0x03u
#define QMC5883_MODE_STANDBY     0x00u
#define QMC5883_MODE_CONTINUOUS  0x01u

#define QMC5883_CTRL1_ODR_SHIFT  2
#define QMC5883_ODR_10HZ         0x00u
#define QMC5883_ODR_50HZ         0x01u
#define QMC5883_ODR_100HZ        0x02u
#define QMC5883_ODR_200HZ        0x03u

#define QMC5883_CTRL1_RNG_SHIFT  4
#define QMC5883_RNG_2G           0x00u  /**< 12000 LSB/Gauss                  */
#define QMC5883_RNG_8G           0x01u  /**<  3000 LSB/Gauss                  */

#define QMC5883_CTRL1_OSR_SHIFT  6
#define QMC5883_OSR_512          0x00u  /**< nhieu thap nhat, bang thong hep nhat */
#define QMC5883_OSR_256          0x01u
#define QMC5883_OSR_128          0x02u
#define QMC5883_OSR_64           0x03u

#define QMC5883_LSB_PER_GAUSS_2G 12000.0f
#define QMC5883_LSB_PER_GAUSS_8G  3000.0f

/* ==========================================================================
 * CTRL2 (0x0A)
 * ========================================================================== */

#define QMC5883_CTRL2_SOFT_RST   0x80u
#define QMC5883_CTRL2_ROL_PNT    0x40u
#define QMC5883_CTRL2_INT_ENB    0x01u  /**< ghi 1 = TAT chan ngat, khong dung */

#endif /* QMC5883_H */
