/**
 * @file    blackbox.c
 * @brief   Hiện thực ghi log chuyến bay ra thẻ SD.
 */

#include "blackbox.h"
#include "fc_state.h"
#include "fc_time.h"
#include "fatfs.h"
#include "bsp_driver_sd.h"
#include "dbg_console.h"

extern SD_HandleTypeDef hsd1;

#if BB_ENABLE

/* ==========================================================================
 * Ghi đè BSP_SD_Init() — sửa lỗi khởi tạo thẻ HAI LẦN
 *
 * Bản __weak trong bsp_driver_sd.c gọi HAL_SD_Init() thêm một lần nữa, trong
 * khi MX_SDMMC1_SD_Init() ở main.c đã khởi tạo thẻ xong từ trước (nếu nó
 * thất bại thì chương trình đã dừng ở Error_Handler rồi).
 *
 * Khởi tạo lại một thẻ đang ở trạng thái transfer thì thất bại: thẻ không
 * đáp lại chuỗi CMD0/ACMD41 như lúc mới cắm. Hậu quả là SD_initialize để
 * nguyên STA_NOINIT và f_mount trả về FR_DISK_ERR — đúng triệu chứng đã gặp.
 *
 * Ở đây chỉ hỏi xem thẻ có đang sẵn sàng không, không đụng vào cấu hình.
 *
 * HỆ QUẢ CẦN BIẾT: rút thẻ ra cắm lại giữa chừng sẽ KHÔNG tự phục hồi, phải
 * khởi động lại mạch. Chấp nhận được vì thẻ nằm trong thân máy bay.
 * ========================================================================== */

uint8_t BSP_SD_Init(void)
{
    return (BSP_SD_GetCardState() == MSD_OK) ? MSD_OK : MSD_ERROR;
}

#define BB_PERIOD_US  (1000000UL / BB_RATE_HZ)
#define BB_CAPACITY   (BB_BUFFER_BYTES / sizeof(bb_record_t))

/*
 * Bộ đệm nằm ở AXI SRAM qua FC_DMA_BUFFER. Hai lý do:
 *   - .bss mặc định rơi vào DTCMRAM chỉ có 128 KB, không đủ chỗ.
 *   - FATFS có thể chuyển thẳng con trỏ người dùng xuống DMA của SDMMC khi
 *     ghi nhiều sector liền, mà DMA thì không truy cập được DTCM.
 */
FC_DMA_BUFFER static bb_record_t s_buf[BB_CAPACITY];

/* Vùng dựng văn bản CSV trước khi đẩy xuống thẻ. Cũng phải DMA đọc được. */
FC_DMA_BUFFER static char s_txt[BB_FLUSH_CHUNK_BYTES + 256u];

/* ==========================================================================
 * Đối tượng FATFS của RIÊNG module này, KHÔNG dùng SDFatFS/SDFile của CubeMX
 *
 * Đây là nguyên nhân gốc làm f_mount trả về FR_DISK_ERR dù thẻ hoàn toàn khoẻ
 * (card state = 4, HAL error = 0).
 *
 * SD_read() truyền THẲNG con trỏ bộ đệm của FATFS xuống BSP_SD_ReadBlocks_DMA.
 * Mà SDFatFS và SDFile do CubeMX sinh ra là biến toàn cục thường, nên rơi vào
 * .bss tức DTCMRAM ở 0x20000000. DTCM chỉ nối trực tiếp với lõi Cortex-M7;
 * IDMA của SDMMC là một bus master trên AHB và KHÔNG với tới được vùng đó.
 * Kết quả: lệnh đọc phát ra rồi không bao giờ hoàn tất.
 *
 * Đúng cùng một bài học đã ghi sẵn trong linker script cho .dma_buffer, chỉ
 * khác là lần này nạn nhân nằm trong code CubeMX sinh chứ không phải code
 * mình viết.
 *
 * Cách chữa gọn nhất: tự khai báo FATFS và FIL trong .dma_buffer (AXI SRAM)
 * rồi dùng cặp đó. Không phải sửa file CubeMX sinh ra, nên Generate Code lại
 * cũng không mất.
 *
 * Với _FS_TINY = 0 thì FATFS giữ đệm win[] cho bảng FAT và thư mục, còn FIL
 * giữ đệm buf[] cho dữ liệu file — cả hai đều đi xuống DMA nên cả hai đều
 * phải nằm ở đây.
 * ========================================================================== */

FC_DMA_BUFFER static FATFS s_fs;
FC_DMA_BUFFER static FIL   s_fil;

/*
 * FIL dung de DO xem file da ton tai chua. Phai la static trong .dma_buffer,
 * KHONG duoc khai bao trong ham: bien cuc bo nam tren ngan xep, ma ngan xep
 * thi o DTCMRAM — dung y het loi vua sua o tren.
 */
FC_DMA_BUFFER static FIL   s_probe;

static bb_state_t s_state = BB_STATE_OFF;
static uint32_t   s_count;        /* số bản ghi đang có trong bộ đệm  */
static uint32_t   s_dropped;
static uint32_t   s_flushed;      /* đã xả được bao nhiêu bản ghi     */
static uint32_t   s_last_us;
static uint32_t   s_t0_us;        /* mốc thời gian lúc bắt đầu ghi    */
static uint16_t   s_index;        /* LOGnnnn.CSV                      */
static uint8_t    s_err;          /* FRESULT gần nhất                 */
static bool       s_file_open;
static uint32_t   s_retry_us;     /* mốc lần thử phục hồi gần nhất     */
static uint8_t    s_retries;      /* đã thử phục hồi bao nhiêu lần     */

/* ==========================================================================
 * Phục hồi thẻ sau sự cố — mất điện giữa lúc đang ghi là ca điển hình
 *
 * Mất điện khi f_write hay f_close đang chạy để lại hai kiểu hỏng, và hai
 * kiểu này chữa hoàn toàn khác nhau:
 *
 *   1. Hỏng FILESYSTEM — chiếm phần lớn. Sector của bảng FAT hoặc entry thư
 *      mục ghi dở nên chuỗi cluster đứt. Triệu chứng: f_mount trả
 *      FR_NO_FILESYSTEM, hoặc thao tác file trả FR_DISK_ERR / FR_INT_ERR.
 *      Ghi lại boot sector và bảng FAT là xong, tức f_mkfs — nhưng mất sạch
 *      dữ liệu, nên phải xin phép qua BB_AUTOFORMAT.
 *
 *   2. Hỏng ở MỨC THẺ. Bộ điều khiển bên trong thẻ kẹt ở trạng thái
 *      programming và không đáp CMD0/ACMD41 nữa. Reset MCU KHÔNG cứu được,
 *      vì reset không cắt điện thẻ. Việc duy nhất phần mềm làm được là
 *      DeInit/Init lại khối SDMMC rồi đợi thẻ tự thoát busy. Hết đường thì
 *      phải rút hẳn nguồn — muốn tự động thì cần công tắc nguồn cho thẻ trên
 *      phần cứng, mạch hiện tại chưa có.
 *
 * BSP_SD_Init() ở đầu file này cố tình KHÔNG init lại thẻ, nên FATFS tự nó
 * không bao giờ khởi tạo lại được. Đường phục hồi duy nhất là gọi tay
 * sd_hard_reinit() ở đây.
 * ========================================================================== */

#if BB_AUTOFORMAT
/* f_mkfs đọc ghi thẻ qua đệm này nên nó cũng phải DMA với tới được. */
FC_DMA_BUFFER static BYTE s_mkfs_work[_MAX_SS];
#endif

/**
 * Reset cứng khối SDMMC rồi đợi thẻ về trạng thái transfer.
 *
 * CHẶN khoảng 20 ms, tệ nhất là 520 ms. CHỈ gọi khi đã DISARM.
 */
static bool sd_hard_reinit(void)
{
    (void)HAL_SD_DeInit(&hsd1);
    HAL_Delay(20);                    /* để thẻ xả hết lệnh còn treo */

    if (HAL_SD_Init(&hsd1) != HAL_OK) {
        return false;
    }

    /*
     * Thẻ vừa bị cắt điện giữa lúc ghi có thể còn bận dọn khối bên trong
     * hàng trăm ms. Đợi có hạn, chứ không quay vòng vô tận.
     */
    {
        const uint32_t t0 = HAL_GetTick();

        while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
            if ((HAL_GetTick() - t0) > 500u) {
                return false;
            }
        }
    }
    return true;
}

/** Mount thẻ, có leo thang: mount thẳng → reset SDMMC → format (nếu cho phép). */
static bool mount_with_recovery(void)
{
    FRESULT fr = f_mount(&s_fs, SDPath, 1);

    if (fr == FR_OK) {
        return true;
    }

    /* Thẻ không đáp hoặc đáp sai — thử dựng lại từ mức phần cứng. */
    if (fr == FR_DISK_ERR || fr == FR_NOT_READY) {
        dbg_print_int("Blackbox: mount hong, reset SDMMC. FRESULT", (int32_t)fr);
        (void)f_mount(NULL, SDPath, 0);
        if (sd_hard_reinit()) {
            fr = f_mount(&s_fs, SDPath, 1);
        }
    }

#if BB_AUTOFORMAT
    /*
     * Thẻ khoẻ nhưng không còn nhận ra hệ tập tin nào. Format là cách duy
     * nhất, và nó xoá sạch — xem chú thích của BB_AUTOFORMAT.
     */
    if (fr == FR_NO_FILESYSTEM) {
        dbg_println("Blackbox: FAT hong han, dang format lai the");
        if (f_mkfs(SDPath, FM_FAT32, 0, s_mkfs_work, sizeof(s_mkfs_work)) == FR_OK) {
            fr = f_mount(&s_fs, SDPath, 1);
        }
    }
#endif

    if (fr != FR_OK) {
        s_err = (uint8_t)fr;
        return false;
    }
    return true;
}

/* ==========================================================================
 * Thao tác với thẻ — CHỈ gọi khi đã DISARM
 * ========================================================================== */

static void make_name(char *name, uint16_t idx)
{
    /* LOGnnnn.CSV — bốn chữ số, có số 0 đứng đầu để sắp xếp đúng thứ tự. */
    name[0] = 'L'; name[1] = 'O'; name[2] = 'G';
    name[3] = (char)('0' + (idx / 1000u) % 10u);
    name[4] = (char)('0' + (idx / 100u)  % 10u);
    name[5] = (char)('0' + (idx / 10u)   % 10u);
    name[6] = (char)('0' + (idx)         % 10u);
    name[7] = '.'; name[8] = 'C'; name[9] = 'S'; name[10] = 'V';
    name[11] = '\0';
}

/** Mở file kế tiếp còn trống và ghi dòng tiêu đề. */
static bool open_next_file(void)
{
    char    name[16];
    FRESULT fr;

    /*
     * Tìm số thứ tự còn trống bằng cách thử mở để ĐỌC. Cách này O(n) nhưng
     * chỉ chạy một lần lúc khởi động và sau mỗi lần xả, nên không đáng ngại.
     * Trần 1000 file là để không quét vô hạn nếu thẻ đầy file cũ.
     */
    while (s_index < 1000u) {
        make_name(name, s_index);
        fr = f_open(&s_probe, name, FA_READ);
        if (fr == FR_NO_FILE) {
            break;                    /* chỗ trống */
        }
        if (fr == FR_OK) {
            (void)f_close(&s_probe);
        }
        s_index++;
    }
    if (s_index >= 1000u) {
        s_err = (uint8_t)FR_DENIED;
        return false;
    }

    make_name(name, s_index);
    fr = f_open(&s_fil, name, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        s_err = (uint8_t)fr;
        return false;
    }

    {
        UINT wrote = 0;
        UINT len   = 0;

        while (g_log_csv_header[len] != '\0') { len++; }
        fr = f_write(&s_fil, g_log_csv_header, len, &wrote);
        if (fr != FR_OK || wrote != len) {
            s_err = (uint8_t)fr;
            (void)f_close(&s_fil);
            return false;
        }
    }

    /* Đẩy tiêu đề xuống thẻ ngay, để mất điện đột ngột vẫn còn file hợp lệ. */
    (void)f_sync(&s_fil);
    s_file_open = true;
    return true;
}

bool blackbox_init(void)
{
    s_state     = BB_STATE_OFF;
    s_count     = 0;
    s_dropped   = 0;
    s_flushed   = 0;
    s_index     = 0;
    s_err       = 0;
    s_file_open = false;
    s_retries   = 0;
    s_retry_us  = 0;

    g_fc.sys.sdcard_mounted = false;
    g_fc.sys.logging_active = false;

    /*
     * Mount phai chay TRUOC khi bat bat ky luong ngat toc do cao nao.
     * Do la ly do blackbox_init() duoc goi ngay sau dbg_console_set_rate()
     * trong main.c, truoc icm20602_init(). Da do: mount xen giua luong DRDY
     * 8 kHz thi the roi ve card state 0 va khong mount duoc.
     */
    if (!mount_with_recovery()) {
        dbg_print_int("Blackbox: khong mount duoc the, FRESULT", (int32_t)s_err);
        /*
         * ERROR chứ không phải OFF: máy trạng thái còn thử lại lúc chạy, xem
         * nhánh BB_STATE_ERROR trong blackbox_update().
         */
        s_state    = BB_STATE_ERROR;
        s_retry_us = micros();
        return false;
    }
    g_fc.sys.sdcard_mounted = true;

    /* Don file thu do khoi chan doan cu de lai, neu con. */
    (void)f_unlink("BBTEST.TXT");

    if (!open_next_file()) {
        s_state    = BB_STATE_ERROR;
        s_retry_us = micros();
        return false;
    }

    s_state   = BB_STATE_IDLE;
    s_last_us = micros();
    return true;
}

/* ==========================================================================
 * Xả bộ đệm ra thẻ — chạy từng khối nhỏ, chỉ khi đã DISARM
 * ========================================================================== */

static void flush_chunk(void)
{
    UINT len = 0;

    /* Dựng văn bản cho tới khi gần đầy khối, rồi ghi một phát. */
    while (s_flushed < s_count && len < BB_FLUSH_CHUNK_BYTES) {
        const int n = log_record_to_csv(&s_txt[len], &s_buf[s_flushed]);

        len += (UINT)n;
        s_flushed++;
    }

    if (len > 0u) {
        UINT          wrote = 0;
        const FRESULT fr    = f_write(&s_fil, s_txt, len, &wrote);

        if (fr != FR_OK || wrote != len) {
            s_err   = (uint8_t)fr;
            s_state = BB_STATE_ERROR;
            (void)f_close(&s_fil);
            s_file_open = false;
            return;
        }
    }

    if (s_flushed >= s_count) {
        (void)f_close(&s_fil);
        s_file_open = false;
        s_index++;                       /* chuyến sau ghi sang file mới */

        s_count   = 0;
        s_flushed = 0;
        s_dropped = 0;

        s_state = open_next_file() ? BB_STATE_IDLE : BB_STATE_ERROR;
    }
}

/* ==========================================================================
 * Máy trạng thái
 * ========================================================================== */

static void append_record(uint32_t now_us)
{
    if (s_count >= BB_CAPACITY) {
        s_dropped++;
        return;
    }
    log_record_fill(&s_buf[s_count], fc_elapsed_us(now_us, s_t0_us) / 1000u);
    s_count++;
}

void blackbox_update(uint32_t now_us)
{
    const bool armed = g_fc.motor.armed && (g_fc.mode == FC_MODE_ARMED);

    switch (s_state) {

    case BB_STATE_IDLE:
        if (armed) {
            s_count   = 0;
            s_flushed = 0;
            s_dropped = 0;
            s_t0_us   = now_us;
            s_last_us = now_us;
            s_state   = BB_STATE_RECORDING;
            g_fc.sys.logging_active = true;
        }
        break;

    case BB_STATE_RECORDING:
        /*
         * KHÔNG chạm vào thẻ ở nhánh này. Chỉ chép 48 byte vào RAM.
         */
        if (!armed) {
            s_state = BB_STATE_FLUSHING;
            g_fc.sys.logging_active = false;
            break;
        }
        if (fc_elapsed_us(now_us, s_last_us) >= BB_PERIOD_US) {
            s_last_us = now_us;
            append_record(now_us);
        }
        break;

    case BB_STATE_FLUSHING:
        /*
         * Đã DISARM nên thẻ có khựng cũng vô hại: DShot đang phát lệnh dừng,
         * motor không quay. Vẫn chia nhỏ từng khối để console cập nhật được
         * tiến độ thay vì treo im lìm vài giây.
         *
         * Arm lại giữa chừng thì bỏ dở phần còn lại và ghi chuyến mới — thà
         * mất đuôi log còn hơn chặn vòng lặp lúc motor đang quay.
         */
        if (armed) {
            (void)f_close(&s_fil);
            s_file_open = false;
            s_index++;
            s_count = 0; s_flushed = 0; s_dropped = 0;
            s_state = open_next_file() ? BB_STATE_IDLE : BB_STATE_ERROR;
            break;
        }
        if (s_count == 0u) {
            s_state = BB_STATE_IDLE;   /* không có gì để xả */
            break;
        }
        flush_chunk();
        break;

    case BB_STATE_ERROR:
        /*
         * Thử dựng lại thẻ. Ba ràng buộc, cái nào cũng có lý do:
         *
         *   - CHỈ khi đã DISARM: sd_hard_reinit() chặn tới nửa giây, và
         *     HAL_SD_Init với thẻ hỏng nặng còn có thể không trả về.
         *   - Giãn cách BB_RECOVER_PERIOD_US: đừng nện vào cái thẻ đang hấp
         *     hối mỗi vòng lặp.
         *   - Có trần BB_RECOVER_RETRIES: hết lượt thì nghỉ hẳn tới lần khởi
         *     động sau. Thà mất log còn hơn cứ vài giây lại đứng hình.
         */
        if (!armed && s_retries < BB_RECOVER_RETRIES &&
            fc_elapsed_us(now_us, s_retry_us) >= BB_RECOVER_PERIOD_US) {

            s_retry_us = now_us;
            s_retries++;

            if (s_file_open) {
                (void)f_close(&s_fil);
                s_file_open = false;
            }
            (void)f_mount(NULL, SDPath, 0);
            g_fc.sys.sdcard_mounted = false;

            if (mount_with_recovery() && open_next_file()) {
                g_fc.sys.sdcard_mounted = true;
                s_err = 0;
                /*
                 * Còn dữ liệu trong RAM thì xả lại TỪ ĐẦU vào file mới. Bộ
                 * đệm vẫn nguyên vẹn — cái hỏng là cái thẻ — nên chuyến bay
                 * vừa rồi cứu được trọn vẹn.
                 */
                s_flushed = 0;
                s_state   = (s_count > 0u) ? BB_STATE_FLUSHING : BB_STATE_IDLE;
                dbg_print_int("Blackbox: phuc hoi the OK, lan thu", (int32_t)s_retries);
            } else {
                dbg_print_int("Blackbox: phuc hoi that bai, FRESULT", (int32_t)s_err);
            }
        }
        break;

    case BB_STATE_OFF:
    default:
        break;
    }
}

/* ==========================================================================
 * API đọc
 * ========================================================================== */

bb_state_t blackbox_state(void)      { return s_state; }
uint32_t   blackbox_records(void)    { return s_count; }
uint32_t   blackbox_dropped(void)    { return s_dropped; }
uint16_t   blackbox_file_index(void) { return s_index; }
uint8_t    blackbox_last_error(void) { return s_err; }

uint8_t blackbox_flush_percent(void)
{
    if (s_state != BB_STATE_FLUSHING || s_count == 0u) {
        return (s_state == BB_STATE_IDLE) ? 100u : 0u;
    }
    return (uint8_t)((s_flushed * 100u) / s_count);
}

const char *blackbox_state_name(void)
{
    switch (s_state) {
    case BB_STATE_OFF:       return "TAT";
    case BB_STATE_IDLE:      return "san sang, cho arm";
    case BB_STATE_RECORDING: return "DANG GHI vao RAM";
    case BB_STATE_FLUSHING:  return "dang xa ra the";
    case BB_STATE_ERROR:     return "LOI THE";
    default:                 return "?";
    }
}

#else  /* BB_ENABLE == 0 */

bool        blackbox_init(void)             { return false; }
void        blackbox_update(uint32_t u)     { (void)u; }
bb_state_t  blackbox_state(void)            { return BB_STATE_OFF; }
uint32_t    blackbox_records(void)          { return 0; }
uint32_t    blackbox_dropped(void)          { return 0; }
uint8_t     blackbox_flush_percent(void)    { return 0; }
uint16_t    blackbox_file_index(void)       { return 0; }
uint8_t     blackbox_last_error(void)       { return 0; }
const char *blackbox_state_name(void)       { return "TAT bang BB_ENABLE = 0"; }

#endif /* BB_ENABLE */
