/**
 * @file    ist8310.h
 * @brief   Bản đồ thanh ghi từ kế iSentek IST8310 — la bàn trên module GPS
 *          MicoAir MG-F10-A. Chỉ có hằng số; driver nằm trong mag_i2c.c.
 *
 * KHÁC BIỆT SO VỚI CÁC CHIP QMC/HMC — những chỗ dễ sai:
 *
 *   1. KHÔNG CÓ CHẾ ĐỘ ĐO LIÊN TỤC. Mỗi mẫu phải ra lệnh đo đơn (ghi 0x01 vào
 *      CNTL1), chờ vài ms, rồi mới đọc. Đọc xong phải ra lệnh tiếp — quên là
 *      chip đứng im ở mẫu cũ, bit DRDY không bao giờ lên lại.
 *
 *   2. HỆ TRỤC DỮ LIỆU LÀ TAY TRÁI: theo ký hiệu trên chip X tới, Y phải,
 *      Z LÊN. PX4 (drivers/magnetometer/isentek/ist8310) đảo Z ngay khi đọc
 *      để có hệ tay phải. mag_i2c.c làm y hệt, nên "hệ cảm biến" đưa vào
 *      mag_axis_* luôn là tay phải và quy tắc định thức +1 vẫn đúng.
 *
 *   3. Khối đọc bắt đầu từ STAT1 (0x02) rồi mới tới dữ liệu 0x03..0x08, tức
 *      byte trạng thái đứng ĐẦU — ngược với QMC (trạng thái ở cuối).
 *
 * ĐỘ NHẠY 0,3 µT/LSB = 333,3 LSB/Gauss (datasheet). ĐÃ ĐO 2026-09-22: xoay cả
 * máy bay 60 s, bán kính ellipsoid 0,41-0,49 G với 333 — khớp từ trường WMM
 * tại chỗ 0,46 G. Bản 09-21 đổi sang 1320 vì thấy |B| thô ~1,5 G: SAI — phần
 * lớn con số đó là độ lệch sắt cứng ~1,7 G CỐ ĐỊNH theo module (đo cả khi
 * module nằm rời trên bàn), hiệu chuẩn khử được. Kiểm thang bằng BÁN KÍNH
 * mặt cầu khi xoay, không bằng |B| thô.
 *
 * Dải đo rộng hơn nhiều so với từ trường Trái Đất — không thể tràn, nên
 * chip không có bit tràn nào để đọc.
 */
#ifndef IST8310_H
#define IST8310_H

/* Địa chỉ 7 bit. Chân CAD0/CAD1 chọn 0x0C..0x0F; module GPS thường để 0x0E. */
#define IST8310_I2C_ADDR_7BIT      0x0Eu
#define IST8310_I2C_ADDR_ALT_7BIT  0x0Cu

#define IST8310_REG_WAI      0x00u   /**< chỉ đọc, phải là 0x10            */
#define IST8310_REG_STAT1    0x02u   /**< bit0 DRDY, bit1 DOR              */
#define IST8310_REG_DATAXL   0x03u   /**< 6 byte, LSB TRƯỚC, X-Y-Z         */
#define IST8310_REG_CNTL1    0x0Au   /**< bit3..0 chế độ: 0 nghỉ, 1 đo đơn */
#define IST8310_REG_CNTL2    0x0Bu   /**< bit0 SRST — đặt lại mềm          */
#define IST8310_REG_AVGCNTL  0x41u   /**< số lần lấy trung bình            */
#define IST8310_REG_PDCNTL   0x42u   /**< độ rộng xung đo                  */

#define IST8310_WAI_VALUE    0x10u

#define IST8310_CNTL1_SINGLE 0x01u
#define IST8310_CNTL2_SRST   0x01u

/* Trung bình 16 lần cho cả Y (bit 5..3) lẫn X/Z (bit 2..0) — như PX4. */
#define IST8310_AVGCNTL_16X  0x24u

/* Độ rộng xung "normal" — datasheet yêu cầu ghi giá trị này. */
#define IST8310_PDCNTL_NORMAL 0xC0u

/* STAT1 + 6 byte dữ liệu. */
#define IST8310_BURST_LEN         7u
#define IST8310_BURST_STATUS_IDX  0u

#define IST8310_LSB_PER_GAUSS     333.333f

#endif /* IST8310_H */
