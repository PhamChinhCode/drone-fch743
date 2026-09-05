/**
 * @file    qspi_flash.c
 * @brief   Hiện thực driver W25Q64 trên QUADSPI. Xem qspi_flash.h.
 */

#include "qspi_flash.h"
#include "dbg_console.h"
#include "main.h"

extern QSPI_HandleTypeDef hqspi;

#if QSPI_FLASH_ENABLE

/* ==========================================================================
 * Tốc độ bus
 *
 * Clock nhân của QUADSPI là D1HCLK = 240 MHz (xem HAL_QSPI_MspInit). Tần số
 * ra = 240 / (ClockPrescaler + 1).
 *
 * Thăm dò ở 20 MHz trước: dưới ngưỡng 50 MHz mà W25Q64 áp cho lệnh Read Data
 * thường, nên nếu ở đây đã không đọc được ID thì lỗi nằm ở dây nối hoặc
 * nguồn, không phải ở tốc độ. Tách được hai nguyên nhân đó ngay từ đầu.
 * ========================================================================== */

#define QSPI_PRESCALER_PROBE  11u   /* 240 / 12 = 20 MHz */

#define CMD_READ_JEDEC_ID     0x9Fu
#define CMD_WRITE_ENABLE      0x06u
#define CMD_READ_SR1          0x05u
#define CMD_READ_SR2          0x35u
#define CMD_FAST_READ         0x0Bu
#define CMD_PAGE_PROGRAM      0x02u
#define CMD_SECTOR_ERASE_4K   0x20u
#define CMD_CHIP_ERASE        0xC7u

#define SR1_BUSY              0x01u

/*
 * VÌ SAO CHỈ DÙNG LỆNH MỘT ĐƯỜNG, DÙ MẠCH NỐI ĐỦ BỐN
 *
 * Bốn đường cho tốc độ gấp bốn, nhưng phải bật cờ QE trong thanh ghi trạng
 * thái — một lần ghi VĨNH VIỄN vào chip — và thêm một lớp nữa để sai.
 *
 * Mà tốc độ thì đã thừa xa: một đường ở 60 MHz cho 7,5 MB/s, trong khi log
 * chỉ cần 4,8 KB/s. Đọc cả chip 8 MB mất 1,1 giây, còn trút ra ngoài thì
 * nghẽn ở USB 1 MB/s chứ không phải ở đây.
 *
 * Cần nhanh hơn thì đổi sau, và lúc đó nó là thay đổi có lý do đo được.
 *
 * Lệnh 0x0B Fast Read chứ không phải 0x03 Read Data: 0x03 chỉ chạy được tới
 * 50 MHz, mà ta đang ở 60 MHz. 0x0B đổi lấy 8 chu kỳ rỗng để chạy tới 133 MHz.
 */

static uint32_t s_jedec;

/** Nạp lại cấu hình QUADSPI với một prescaler khác. */
static bool set_clock(uint32_t prescaler)
{
    hqspi.Init.ClockPrescaler     = prescaler;
    hqspi.Init.ChipSelectHighTime = QSPI_FLASH_CS_HIGH_TIME;

    return (HAL_QSPI_Init(&hqspi) == HAL_OK);
}

/**
 * Đọc ba byte JEDEC ID: nhà sản xuất, loại bộ nhớ, dung lượng.
 * W25Q64 trả về EF 40 17.
 */
static bool read_jedec(uint32_t *out)
{
    QSPI_CommandTypeDef cmd = {0};
    uint8_t             id[3] = {0};

    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction       = CMD_READ_JEDEC_ID;
    cmd.AddressMode       = QSPI_ADDRESS_NONE;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode          = QSPI_DATA_1_LINE;
    cmd.DummyCycles       = 0;
    cmd.NbData            = sizeof(id);
    cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(&hqspi, &cmd, 100u) != HAL_OK) {
        return false;
    }
    /*
     * Chế độ polling: HAL chép từng byte qua thanh ghi dữ liệu bằng CPU, KHÔNG
     * dùng DMA. Nên đệm nằm trên ngăn xếp (DTCMRAM) là hợp lệ ở đây — khác hẳn
     * đường SDMMC, chỗ mà con trỏ đi thẳng xuống IDMA.
     */
    if (HAL_QSPI_Receive(&hqspi, id, 100u) != HAL_OK) {
        return false;
    }

    *out = ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | (uint32_t)id[2];
    return true;
}

/** ID hợp lệ khi không phải toàn 0 hay toàn 1 — hai dạng của bus chết. */
static bool jedec_plausible(uint32_t id)
{
    return (id != 0x000000u) && (id != 0xFFFFFFu);
}

bool qspi_flash_init(void)
{
    uint32_t id_slow = 0;
    uint32_t id_fast = 0;

    s_jedec = 0;

    if (!set_clock(QSPI_PRESCALER_PROBE) || !read_jedec(&id_slow)) {
        dbg_println("QSPI: khong doc duoc JEDEC ID o 20 MHz");
        return false;
    }
    if (!jedec_plausible(id_slow)) {
        dbg_print_hex("QSPI: JEDEC ID vo nghia o 20 MHz", id_slow, 6);
        return false;
    }

    if (!set_clock(QSPI_FLASH_PRESCALER) || !read_jedec(&id_fast)) {
        dbg_println("QSPI: khong doc duoc JEDEC ID o toc do chay");
        return false;
    }
    if (id_fast != id_slow) {
        /* Chậm thì đúng, nhanh thì sai: bus không chịu nổi tốc độ này. */
        dbg_print_hex("QSPI: ID lech o toc do cao, cham duoc", id_slow, 6);
        dbg_print_hex("                          nhanh ra   ", id_fast, 6);
        return false;
    }

    s_jedec = id_fast;
    return true;
}

/* ==========================================================================
 * Thao tác đọc / ghi / xoá
 * ========================================================================== */

/** Khung lệnh dùng chung. Mặc định: lệnh một đường, không địa chỉ, không dữ liệu. */
static void cmd_init(QSPI_CommandTypeDef *c, uint8_t instruction)
{
    c->InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    c->Instruction       = instruction;
    c->AddressMode       = QSPI_ADDRESS_NONE;
    c->AddressSize       = QSPI_ADDRESS_24_BITS;
    c->Address           = 0;
    c->AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    c->DataMode          = QSPI_DATA_NONE;
    c->DummyCycles       = 0;
    c->NbData            = 0;
    c->DdrMode           = QSPI_DDR_MODE_DISABLE;
    c->DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    c->SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
}

/** Lệnh trống, không dữ liệu: Write Enable, Chip Erase... */
static bool cmd_only(uint8_t instruction)
{
    QSPI_CommandTypeDef c;

    cmd_init(&c, instruction);
    return (HAL_QSPI_Command(&hqspi, &c, 100u) == HAL_OK);
}

/** Đọc một thanh ghi trạng thái một byte. */
static uint8_t read_sr(uint8_t instruction)
{
    QSPI_CommandTypeDef c;
    uint8_t             v = 0;

    cmd_init(&c, instruction);
    c.DataMode = QSPI_DATA_1_LINE;
    c.NbData   = 1;

    if (HAL_QSPI_Command(&hqspi, &c, 100u) != HAL_OK) {
        return 0;
    }
    if (HAL_QSPI_Receive(&hqspi, &v, 100u) != HAL_OK) {
        return 0;
    }
    return v;
}

uint8_t qspi_flash_sr1(void) { return read_sr(CMD_READ_SR1); }
uint8_t qspi_flash_sr2(void) { return read_sr(CMD_READ_SR2); }

bool qspi_flash_is_busy(void)
{
    return (read_sr(CMD_READ_SR1) & SR1_BUSY) != 0u;
}

bool qspi_flash_wait_ready(uint32_t timeout_ms)
{
    const uint32_t t0 = HAL_GetTick();

    while (qspi_flash_is_busy()) {
        if ((HAL_GetTick() - t0) > timeout_ms) {
            return false;
        }
    }
    return true;
}

bool qspi_flash_read(uint32_t addr, void *dst, uint32_t len)
{
    QSPI_CommandTypeDef c;

    if (dst == NULL || len == 0u || (addr + len) > qspi_flash_bytes()) {
        return false;
    }

    cmd_init(&c, CMD_FAST_READ);
    c.AddressMode = QSPI_ADDRESS_1_LINE;
    c.Address     = addr;
    c.DataMode    = QSPI_DATA_1_LINE;
    c.DummyCycles = 8;
    c.NbData      = len;

    if (HAL_QSPI_Command(&hqspi, &c, 100u) != HAL_OK) {
        return false;
    }
    /*
     * Hạn thời gian nới theo độ dài: 8 MB ở 7,5 MB/s mất hơn một giây, mà
     * một con số cứng 100 ms thì lệnh đọc dài nào cũng trượt.
     */
    return (HAL_QSPI_Receive(&hqspi, (uint8_t *)dst,
                             100u + (len / 1024u)) == HAL_OK);
}

bool qspi_flash_program_start(uint32_t addr, const void *src, uint32_t len)
{
    QSPI_CommandTypeDef c;

    if (src == NULL || len == 0u || len > QSPI_FLASH_PAGE_BYTES ||
        (addr + len) > qspi_flash_bytes()) {
        return false;
    }
    /*
     * Một lệnh ghi không được vắt qua ranh giới trang: chip sẽ quay vòng về
     * đầu trang và đè lên chính dữ liệu vừa ghi, im lặng, không báo lỗi.
     */
    if (((addr % QSPI_FLASH_PAGE_BYTES) + len) > QSPI_FLASH_PAGE_BYTES) {
        return false;
    }

    if (!cmd_only(CMD_WRITE_ENABLE)) {
        return false;
    }

    cmd_init(&c, CMD_PAGE_PROGRAM);
    c.AddressMode = QSPI_ADDRESS_1_LINE;
    c.Address     = addr;
    c.DataMode    = QSPI_DATA_1_LINE;
    c.NbData      = len;

    if (HAL_QSPI_Command(&hqspi, &c, 100u) != HAL_OK) {
        return false;
    }
    /*
     * Transmit xong là byte đã nằm trong chip, nhưng chip còn tự lập trình
     * thêm ~0,4 ms nữa. Hàm này TRẢ VỀ NGAY tại đây — người gọi tự hỏi
     * qspi_flash_is_busy() lúc cần.
     */
    return (HAL_QSPI_Transmit(&hqspi, (uint8_t *)src, 100u) == HAL_OK);
}

bool qspi_flash_write_page(uint32_t addr, const void *src, uint32_t len)
{
    if (!qspi_flash_wait_ready(50u)) {
        return false;
    }
    if (!qspi_flash_program_start(addr, src, len)) {
        return false;
    }
    return qspi_flash_wait_ready(50u);   /* tPP xấu nhất 3 ms */
}

bool qspi_flash_erase_sector(uint32_t addr)
{
    QSPI_CommandTypeDef c;

    if (addr >= qspi_flash_bytes()) {
        return false;
    }
    if (!qspi_flash_wait_ready(1000u)) {
        return false;
    }
    if (!cmd_only(CMD_WRITE_ENABLE)) {
        return false;
    }

    cmd_init(&c, CMD_SECTOR_ERASE_4K);
    c.AddressMode = QSPI_ADDRESS_1_LINE;
    c.Address     = addr & ~(QSPI_FLASH_SECTOR_BYTES - 1u);

    if (HAL_QSPI_Command(&hqspi, &c, 100u) != HAL_OK) {
        return false;
    }
    return qspi_flash_wait_ready(1000u); /* tSE xấu nhất 400 ms */
}

bool qspi_flash_erase_chip_start(void)
{
    if (qspi_flash_is_busy()) {
        return false;
    }
    if (!cmd_only(CMD_WRITE_ENABLE)) {
        return false;
    }
    return cmd_only(CMD_CHIP_ERASE);
}

uint32_t qspi_flash_jedec(void)
{
    return s_jedec;
}

uint32_t qspi_flash_bytes(void)
{
    /*
     * Byte thứ ba của JEDEC ID là log2 của dung lượng tính theo byte. W25Q64
     * trả 0x17 tức 2^23 = 8 MB. Chặn khoảng hợp lý để một byte rác không
     * biến thành con số vô lý.
     */
    const uint32_t cap = s_jedec & 0xFFu;

    if (s_jedec == 0u || cap < 16u || cap > 26u) {
        return 0u;
    }
    return (uint32_t)1u << cap;
}

#else /* QSPI_FLASH_ENABLE == 0 */

bool     qspi_flash_init(void)  { return false; }
uint32_t qspi_flash_jedec(void) { return 0u; }
uint32_t qspi_flash_bytes(void) { return 0u; }
uint8_t  qspi_flash_sr1(void)   { return 0u; }
uint8_t  qspi_flash_sr2(void)   { return 0u; }
bool     qspi_flash_is_busy(void) { return false; }
bool     qspi_flash_wait_ready(uint32_t t) { (void)t; return false; }
bool     qspi_flash_erase_chip_start(void) { return false; }
bool     qspi_flash_erase_sector(uint32_t a) { (void)a; return false; }
bool     qspi_flash_read(uint32_t a, void *d, uint32_t l)
{ (void)a; (void)d; (void)l; return false; }
bool     qspi_flash_write_page(uint32_t a, const void *s, uint32_t l)
{ (void)a; (void)s; (void)l; return false; }
bool     qspi_flash_program_start(uint32_t a, const void *s, uint32_t l)
{ (void)a; (void)s; (void)l; return false; }

#endif /* QSPI_FLASH_ENABLE */
