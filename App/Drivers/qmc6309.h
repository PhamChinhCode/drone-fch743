/**
 * @file    qmc6309.h
 * @brief   Bản đồ thanh ghi từ kế QST QMC6309.
 *
 * ĐẤU NỐI:
 *   Chip này KHÔNG nối thẳng vào MCU. Nó nằm trên bus I2C phụ của LSM6DSV
 *   (chân SDX/SCX), và host đọc nó gián tiếp qua khối "sensor hub" của
 *   LSM6DSV. Toàn bộ phần giao tiếp nằm trong lsm6dsv.c; file này chỉ chứa
 *   hằng số.
 *
 * NGUỒN:
 *   Datasheet chính thức QST — "QMC6309 Preliminary Datasheet Rev: A",
 *   Document #13-52-22, mục 9 REGISTERS.
 *
 *   ⚠️ Thư viện QMC6309 phổ biến trên GitHub (SensorLib) đặt SAI vị trí bit:
 *   nó để ODR trong thanh ghi 0x0A và dùng ba mặt nạ chồng lấn nhau
 *   (0x18, 0x70, 0xE0) trong cùng một thanh ghi 8 bit — không thể cùng đúng.
 *   Các giá trị dưới đây lấy từ datasheet và đã đối chiếu khớp với chính hai
 *   ví dụ trong datasheet (mục 7.1 và 7.2).
 */
#ifndef QMC6309_H
#define QMC6309_H

#include "fc_types.h"

/* ==========================================================================
 * Địa chỉ I2C
 *
 * Datasheet mục 5.4: "This device has a 7-bit serial address... There are
 * only one I2C address available. The default value is 7CH."
 * ========================================================================== */

#define QMC_I2C_ADDR_7BIT     0x7Cu

/*
 * Địa chỉ dự phòng.
 *
 * Datasheet ghi "7-bit serial address ... default value is 7CH", nhưng rất
 * nhiều datasheet của hãng khác lại ghi dạng 8 BIT ở chính chỗ này. Nếu 0x7C
 * thật ra là dạng 8 bit thì địa chỉ 7 bit là 0x3E.
 *
 * Driver dò CẢ HAI và báo địa chỉ nào trả lời, thay vì để một chữ số mơ hồ
 * làm mất một vòng gỡ lỗi.
 */
#define QMC_I2C_ADDR_7BIT_ALT 0x3Eu

/* ==========================================================================
 * Bản đồ thanh ghi (Table 12)
 * ========================================================================== */

#define QMC_REG_CHIP_ID       0x00u   /**< chỉ đọc, mặc định 0x90          */
#define QMC_REG_XOUT_L        0x01u   /**< đầu khối 6 byte dữ liệu         */
#define QMC_REG_XOUT_H        0x02u
#define QMC_REG_YOUT_L        0x03u
#define QMC_REG_YOUT_H        0x04u
#define QMC_REG_ZOUT_L        0x05u
#define QMC_REG_ZOUT_H        0x06u
#define QMC_REG_STATUS        0x09u
#define QMC_REG_CTRL1         0x0Au
#define QMC_REG_CTRL2         0x0Bu
#define QMC_REG_CTRL3         0x0Eu   /**< chỉ có bit tự kiểm tra          */
#define QMC_REG_ST_X          0x13u
#define QMC_REG_ST_Y          0x14u
#define QMC_REG_ST_Z          0x15u

#define QMC_CHIP_ID_VALUE     0x90u
#define QMC_DATA_LEN          6u      /**< 0x01..0x06                      */

/* ==========================================================================
 * STATUS (0x09) — Table 14
 *
 * DRDY và OVFL đều TỰ XOÁ khi đọc thanh ghi này.
 * ========================================================================== */

#define QMC_STATUS_DRDY       0x01u   /**< đủ ba trục, dữ liệu mới sẵn sàng */
#define QMC_STATUS_OVFL       0x02u   /**< có trục vượt ±32000 LSB          */
#define QMC_STATUS_ST_RDY     0x04u
#define QMC_STATUS_NVM_RDY    0x08u
#define QMC_STATUS_NVM_LOAD   0x10u

/* ==========================================================================
 * CTRL1 (0x0A) — Table 15
 *
 *   bit 7..5  OSR2  độ sâu bộ lọc thông thấp
 *   bit 4..3  OSR1  tỉ lệ lấy mẫu quá mức (băng thông bộ lọc số)
 *   bit 2     -
 *   bit 1..0  MODE
 * ========================================================================== */

#define QMC_CTRL1_MODE_MASK   0x03u
#define QMC_MODE_SUSPEND      0x00u
#define QMC_MODE_NORMAL       0x01u
#define QMC_MODE_SINGLE       0x02u
#define QMC_MODE_CONTINUOUS   0x03u

#define QMC_CTRL1_OSR1_SHIFT  3
#define QMC_OSR1_8            0x00u   /**< băng thông hẹp nhất, ít nhiễu nhất */
#define QMC_OSR1_4            0x01u
#define QMC_OSR1_2            0x02u
#define QMC_OSR1_1            0x03u

#define QMC_CTRL1_OSR2_SHIFT  5
#define QMC_OSR2_1            0x00u
#define QMC_OSR2_2            0x01u
#define QMC_OSR2_4            0x02u
#define QMC_OSR2_8            0x03u
#define QMC_OSR2_16           0x04u

/* ==========================================================================
 * CTRL2 (0x0B) — Table 16
 *
 *   bit 7     SOFT_RST   ghi 1 rồi PHẢI ghi 0, không tự xoá
 *   bit 6..4  ODR
 *   bit 3..2  RNG        dải đo
 *   bit 1..0  SET/RESET MODE
 * ========================================================================== */

#define QMC_CTRL2_SOFT_RST    0x80u

#define QMC_CTRL2_ODR_SHIFT   4
#define QMC_ODR_1HZ           0x00u
#define QMC_ODR_10HZ          0x01u
#define QMC_ODR_50HZ          0x02u
#define QMC_ODR_100HZ         0x03u
#define QMC_ODR_200HZ         0x04u

#define QMC_CTRL2_RNG_SHIFT   2
#define QMC_RNG_32G           0x00u   /**< 1000 LSB/Gauss                  */
#define QMC_RNG_16G           0x01u   /**< 2000 LSB/Gauss                  */
#define QMC_RNG_8G            0x02u   /**< 4000 LSB/Gauss, phân giải tốt nhất */

/*
 * Chế độ SET/RESET điều khiển chu trình khử từ dư. Bật cả hai (giá trị 00)
 * thì offset được làm mới mỗi lần đo — chính xác nhất, và là thứ ta cần cho
 * la bàn. Hai chế độ kia bỏ qua bước đó nên offset trôi theo thời gian.
 */
#define QMC_SETRESET_ON       0x00u   /**< set và reset, offset luôn mới    */
#define QMC_SETRESET_SET_ONLY 0x01u
#define QMC_SETRESET_OFF      0x03u

/* ==========================================================================
 * Đổi thang
 *
 * Datasheet Table 2: 32G -> 1000 LSB/G, 16G -> 2000 LSB/G, 8G -> 4000 LSB/G.
 * Dữ liệu là 16 bit bù hai, BYTE THẤP Ở ĐỊA CHỈ THẤP (little-endian).
 * ========================================================================== */

#define QMC_LSB_PER_GAUSS_32G 1000.0f
#define QMC_LSB_PER_GAUSS_16G 2000.0f
#define QMC_LSB_PER_GAUSS_8G  4000.0f

#endif /* QMC6309_H */
