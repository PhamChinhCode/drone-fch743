/**
 * @file    qmc5883p.h
 * @brief   Bản đồ thanh ghi từ kế QST QMC5883P — chip THẬT trên module đã
 *          lắp vào bo này (quét bus I2C1 thấy trả lời ở 0x2C).
 *
 * ĐÂY LÀ CHIP ĐANG DÙNG. Hai file kia (hmc5883.h, qmc5883.h) giữ lại để
 * driver vẫn dò được nếu sau này đổi module.
 *
 * ĐỪNG NHẦM VỚI BA CHIP TÊN GẦN GIỐNG:
 *   QMC5883P  0x2C  chip id 0x80  <- file này, chip đang lắp
 *   QMC5883L  0x0D  chip id 0xFF  <- App/Drivers/qmc5883.h
 *   QMC6309   0x7C  chip id 0x90  <- App/Drivers/qmc6309.h (nguồn SHUB cũ)
 * Ba bản đồ thanh ghi khác nhau hoàn toàn, không thanh ghi nào trùng nghĩa.
 *
 * NGUỒN:
 *   Đối chiếu từ driver sản xuất của ArduPilot
 *   (libraries/AP_Compass/AP_Compass_QMC5883P.cpp), là code đã chạy thực tế
 *   trên phần cứng. Datasheet chính thức QST có tồn tại nhưng bản tải được
 *   là ảnh scan tiếng Trung, không trích được text để đối chiếu từng bit.
 *
 *   ⚠️ ArduPilot có LỖI GÕ trong chính bảng dải đo của họ:
 *
 *       #define QMC5883P_RNG_8G  (0x10 << 2)
 *       #define QMC5883P_RNG_2G  (0x11 << 2)
 *
 *   Dãy giá trị 0x00, 0x01, 0x10, 0x11 rõ ràng có ý là NHỊ PHÂN 00/01/10/11
 *   nhưng lại viết dưới dạng HEX, nên 0x10 << 2 = 0x40 tràn ra khỏi trường
 *   2 bit. Họ không lộ lỗi này vì không dùng tới hai define đó — lúc init
 *   họ ghi CONF2 = 0x08, mà 0x08 đúng bằng (0b10 << 2), tức RNG = 8G.
 *   Bảng dưới đây dùng NHỊ PHÂN cho đúng, và con số 3000 LSB/Gauss của dải
 *   8 G khớp với `range_scale = 1000.0f / 3000.0f` trong chính code đó.
 *
 *   Cách tự kiểm khi chạy: từ trường Trái Đất là 0,25 - 0,65 G. |B| ra
 *   ngoài dải đó nhiều thì bản đồ ở đây sai, không phải cảm biến hỏng.
 */
#ifndef QMC5883P_H
#define QMC5883P_H

#include "fc_types.h"

/* ==========================================================================
 * Địa chỉ I2C — CỐ ĐỊNH.
 * ========================================================================== */

#define QMC5883P_I2C_ADDR_7BIT  0x2Cu

/* ==========================================================================
 * Bản đồ thanh ghi
 * ========================================================================== */

#define QMC5883P_REG_CHIP_ID    0x00u  /**< chi doc, phai la 0x80            */
#define QMC5883P_REG_DATA_X_LSB 0x01u  /**< dau khoi 6 byte, LSB TRUOC, X-Y-Z */
#define QMC5883P_REG_DATA_X_MSB 0x02u
#define QMC5883P_REG_DATA_Y_LSB 0x03u
#define QMC5883P_REG_DATA_Y_MSB 0x04u
#define QMC5883P_REG_DATA_Z_LSB 0x05u
#define QMC5883P_REG_DATA_Z_MSB 0x06u  /**< cung la cho ghi SET_XYZ_SIGN      */
#define QMC5883P_REG_STATUS     0x09u
#define QMC5883P_REG_CTRL1      0x0Au
#define QMC5883P_REG_CTRL2      0x0Bu

#define QMC5883P_CHIP_ID_VALUE  0x80u

/*
 * Trước khi bật chế độ đọc liên tục PHẢI ghi giá trị 0x29 vào thanh ghi
 * 0x06 để chốt dấu ba trục (datasheet mục 7.2).
 *
 * ⚠️ Rất dễ nhớ ngược: 0x29 là GIÁ TRỊ, 0x06 là THANH GHI — chứ không phải
 * ghi 0x06 vào thanh ghi 0x29. Thanh ghi 0x06 vốn là byte cao trục Z, nên
 * nhìn vào code dễ tưởng là ghi nhầm chỗ.
 */
#define QMC5883P_SET_XYZ_SIGN   0x29u

/**
 * Số byte đọc mỗi chu kỳ: 0x01..0x09 liên tiếp — 6 byte dữ liệu, 2 byte
 * ở giữa không dùng (0x07/0x08), rồi STATUS.
 *
 * Đọc liền một khối thay vì hai lượt như ArduPilot: trên bus dùng chung với
 * BMP388 thì một lượt truyền dài vẫn rẻ hơn hai lượt tranh chấp riêng.
 */
#define QMC5883P_BURST_LEN        9u
#define QMC5883P_BURST_STATUS_IDX 8u

/* ==========================================================================
 * STATUS (0x09)
 * ========================================================================== */

#define QMC5883P_STATUS_DRDY    0x01u
#define QMC5883P_STATUS_OVL     0x02u

/* ==========================================================================
 * CTRL1 (0x0A)
 *
 *   bit 5..4  OSR1   (00 = 8, nhieu thap nhat)
 *   bit 3..2  ODR
 *   bit 1..0  MODE
 * ========================================================================== */

#define QMC5883P_CTRL1_MODE_MASK 0x03u
#define QMC5883P_MODE_SUSPEND    0x00u
#define QMC5883P_MODE_NORMAL     0x01u
#define QMC5883P_MODE_SINGLE     0x02u
#define QMC5883P_MODE_CONTINUOUS 0x03u

#define QMC5883P_CTRL1_ODR_SHIFT 2
#define QMC5883P_ODR_10HZ        0x00u
#define QMC5883P_ODR_50HZ        0x01u
#define QMC5883P_ODR_100HZ       0x02u
#define QMC5883P_ODR_200HZ       0x03u

#define QMC5883P_CTRL1_OSR1_SHIFT 4
#define QMC5883P_OSR1_8          0x00u  /**< bang thong hep nhat, it nhieu nhat */
#define QMC5883P_OSR1_4          0x01u
#define QMC5883P_OSR1_2          0x02u
#define QMC5883P_OSR1_1          0x03u

/* ==========================================================================
 * CTRL2 (0x0B)
 *
 *   bit 7     SOFT_RST
 *   bit 3..2  RNG    dai do
 * ========================================================================== */

#define QMC5883P_CTRL2_SOFT_RST  0x80u

#define QMC5883P_CTRL2_RNG_SHIFT 2
#define QMC5883P_RNG_30G         0x00u
#define QMC5883P_RNG_12G         0x01u
#define QMC5883P_RNG_8G          0x02u  /**< (0b10 << 2) = 0x08 */
#define QMC5883P_RNG_2G          0x03u

/*
 * CHỈ dải 8 G có hệ số đổi thang ĐÃ ĐƯỢC KIỂM CHỨNG (3000 LSB/Gauss, khớp
 * với ArduPilot). Ba dải kia suy ra theo tỉ lệ nghịch thì được 800 / 2000 /
 * 12000 LSB/Gauss, nhưng KHÔNG có nguồn nào xác nhận, nên driver cố tình
 * chỉ dùng 8 G và bỏ qua tham số mag_range_g cho riêng chip này.
 *
 * 8 G là lựa chọn đúng dù sao: từ trường Trái Đất ~0,5 G cho ~1500 count
 * (đủ phân giải), và còn thừa rất nhiều chỗ cho nhiễu từ động cơ.
 * Đừng thêm dải khác vào đây bằng cách suy đoán — số đổi thang sai làm |B|
 * sai mà vẫn trông "hợp lý", đúng kiểu lỗi khó tìm nhất.
 */
#define QMC5883P_LSB_PER_GAUSS_8G 3000.0f

#endif /* QMC5883P_H */
