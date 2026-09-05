/**
 * @file    qspi_flash.h
 * @brief   Flash NOR W25Q64 treo tren QUADSPI (U3).
 *
 * VÌ SAO CÓ MODULE NÀY
 *
 *   Thẻ SD mất điện giữa lúc ghi thì hỏng bảng FAT, nặng thì kẹt luôn bộ
 *   điều khiển bên trong thẻ và không init lại được. Flash NOR không có FAT
 *   cũng không có bảng ánh xạ nội bộ nào để mà hỏng — mất điện tệ nhất là
 *   rác 256 byte cuối.
 *
 *   Đổi lại chỉ có 8 MB, và phải xoá trước khi ghi.
 *
 * PHẦN CỨNG
 *
 *   W25Q64, 8 MB. QUADSPI bank 1, nối đủ bốn đường dữ liệu:
 *     PB6  NCS      PB2  CLK
 *     PD11 IO0      PD12 IO1      PE2 IO2      PD13 IO3
 *
 *   CubeMX đã cấu hình sẵn chân, AF và clock trong HAL_QSPI_MspInit(). Module
 *   này KHÔNG đụng vào đó, chỉ chỉnh lại hai tham số tốc độ rồi gọi
 *   HAL_QSPI_Init() lần nữa — đúng cách lsm6dsv.c làm với hspi3.
 *
 *   MX_QUADSPI_Init() để ClockPrescaler = 255, tức 240 MHz / 256 ≈ 0,94 MHz.
 *   Đó chỉ là mặc định CubeMX điền vào, không phải lựa chọn có chủ ý.
 */
#ifndef QSPI_FLASH_H
#define QSPI_FLASH_H

#include "fc_types.h"
#include "fc_config.h"

/**
 * Đặt tốc độ bus rồi đọc JEDEC ID để xác nhận có chip thật.
 *
 * Thăm dò HAI LẦN: lần đầu ở tốc độ chậm an toàn, lần sau ở tốc độ chạy
 * thật. Hai lần cùng ra một ID thì mới coi là bus chịu được tốc độ cao —
 * rẻ hơn nhiều so với việc phát hiện ra điều đó lúc đang ghi log.
 *
 * Hàm CHẶN, mất chưa tới 1 ms. Gọi lúc khởi động.
 *
 * @return true nếu ID hợp lệ ở CẢ HAI tốc độ.
 */
bool qspi_flash_init(void);

/** JEDEC ID gói trong 24 bit: 0x00EF4017 với W25Q64. 0 nếu chưa đọc được. */
uint32_t qspi_flash_jedec(void);

/** Dung lượng suy ra từ byte thứ ba của JEDEC ID. 0 nếu chưa nhận ra chip. */
uint32_t qspi_flash_bytes(void);

/* ==========================================================================
 * Hình học
 *
 * Ba con số này là của mọi dòng NOR nối tiếp, không riêng W25Q64:
 *   - Ghi theo TRANG 256 byte, và một lệnh ghi KHÔNG được vắt qua ranh giới
 *     trang. Vắt qua thì chip quay vòng về đầu trang và đè lên chính nó.
 *   - Xoá theo SECTOR 4 KB. Không có cách xoá nhỏ hơn.
 *   - Ghi chỉ đổi bit 1 thành 0. Muốn đổi ngược lại thì phải xoá cả sector.
 * ========================================================================== */
#define QSPI_FLASH_PAGE_BYTES    256u
#define QSPI_FLASH_SECTOR_BYTES  4096u

/**
 * Đọc. Không giới hạn độ dài, không cần căn theo trang.
 *
 * CHẶN. 60 MHz một đường cho ra ~7,5 MB/s, tức một trang 256 byte mất 34 µs.
 */
bool qspi_flash_read(uint32_t addr, void *dst, uint32_t len);

/**
 * Ghi tối đa một trang.
 *
 * @param addr  phải nằm sao cho addr + len không vượt ranh giới trang 256 B.
 * @return false nếu tham số sai, hoặc chip không sẵn sàng trong 50 ms.
 *
 * Hàm CHẶN ~34 µs để đẩy byte xuống, rồi ĐỢI chip lập trình xong (0,4 ms
 * điển hình). Đường log lúc bay KHÔNG dùng hàm này mà tách hai việc đó ra —
 * xem qspi_flash_program_start().
 */
bool qspi_flash_write_page(uint32_t addr, const void *src, uint32_t len);

/**
 * Đẩy một trang xuống chip rồi TRẢ VỀ NGAY, không đợi chip lập trình xong.
 *
 * Đây là hàm dành cho lúc đang bay: phần chặn chỉ là 34 µs truyền byte, còn
 * 0,4 ms chip tự nướng thì vòng lặp đi làm việc khác. Lần sau hỏi
 * qspi_flash_is_busy() trước khi ghi trang kế tiếp.
 */
bool qspi_flash_program_start(uint32_t addr, const void *src, uint32_t len);

/** Xoá một sector 4 KB. CHẶN 45 ms, xấu nhất 400 ms. KHÔNG gọi khi đang ARM. */
bool qspi_flash_erase_sector(uint32_t addr);

/**
 * Phát lệnh xoá TOÀN BỘ chip rồi trả về ngay. Chip bận 20–100 giây sau đó.
 *
 * Không đợi, vì 100 giây trong vòng lặp chính là không thể chấp nhận. Hỏi
 * qspi_flash_is_busy() để biết xong chưa.
 */
bool qspi_flash_erase_chip_start(void);

/** Bit BUSY của thanh ghi trạng thái 1. true khi chip đang xoá hoặc ghi. */
bool qspi_flash_is_busy(void);

/** Đợi chip rảnh. false nếu quá hạn. */
bool qspi_flash_wait_ready(uint32_t timeout_ms);

/** Thanh ghi trạng thái 1 và 2 — để chẩn đoán. SR2 bit 1 là cờ QE. */
uint8_t qspi_flash_sr1(void);
uint8_t qspi_flash_sr2(void);

#endif /* QSPI_FLASH_H */
