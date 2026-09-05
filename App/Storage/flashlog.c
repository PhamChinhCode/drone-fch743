/**
 * @file    flashlog.c
 * @brief   Hiện thực ghi log vào flash NOR. Xem flashlog.h.
 */

#include "flashlog.h"
#include "qspi_flash.h"
#include "fc_state.h"
#include "fc_time.h"
#include "main.h"

#include <string.h>

#if FLASHLOG_ENABLE

#define FL_PERIOD_US  (1000000UL / FLASHLOG_RATE_HZ)
#define PAGE          QSPI_FLASH_PAGE_BYTES

/*
 * Sector CUỐI chip để dành cho lệnh 'flash test' — lệnh đó xoá rồi ghi đè
 * lên nó. Bớt 4 KB trong 8 MB đổi lấy việc tự kiểm tra được phần cứng bất cứ
 * lúc nào mà không sợ đụng vào log.
 */
#define FL_RESERVED_TAIL  QSPI_FLASH_SECTOR_BYTES

/* ==========================================================================
 * Hai đệm trang, thay phiên nhau
 *
 * Một đệm đang được đổ dữ liệu vào, đệm kia chờ tới lượt ghi. Cần hai vì
 * chip có thể còn bận lập trình trang trước (0,4 ms điển hình, 3 ms xấu
 * nhất) đúng lúc đệm này vừa đầy — mà đợi ở đó thì chặn vòng lặp chính.
 *
 * AN TOÀN KHI DÙNG LẠI ĐỆM: qspi_flash_program_start() truyền byte theo kiểu
 * CHẶN, nên lúc nó trả về thì dữ liệu đã nằm trong chip. Chip còn bận nướng
 * tiếp nhưng không đọc đệm của ta nữa.
 * ========================================================================== */

static uint8_t  s_page[2][PAGE];
static uint8_t  s_fill;         /* đệm đang được đổ vào       */
static uint16_t s_fill_len;
static bool     s_pending;      /* đệm kia đang chờ ghi       */
static uint8_t  s_pend_idx;

static uint32_t s_write_addr;   /* địa chỉ của trang kế tiếp sẽ ghi */
static uint32_t s_limit;        /* hết chỗ khi write_addr chạm đây  */

static flashlog_state_t s_state = FL_STATE_OFF;
static uint32_t s_records;
static uint32_t s_dropped;
static uint32_t s_last_us;
static uint32_t s_t0_us;

/* ==========================================================================
 * Dò điểm cuối của dữ liệu đã có
 * ========================================================================== */

/**
 * 1 = trang đã xoá, 0 = trang đã ghi, -1 = ĐỌC HỎNG.
 *
 * Ba trạng thái chứ không phải hai, và đó là điểm mấu chốt: coi một lần đọc
 * hỏng là "đã ghi" thì tìm nhị phân vẫn chạy trơn tru và vẫn trả về một con
 * số trông hợp lý — chỉ có điều nó sai. Con trỏ ghi nhảy vọt lên phía trước,
 * âm thầm ăn mất mấy chục KB mà không ai biết.
 */
static int page_is_erased(uint32_t addr)
{
    uint8_t buf[64];

    /*
     * Chỉ cần đọc 64 byte đầu trang. Ta luôn ghi trọn một trang mỗi lần, nên
     * trang nào đã ghi thì 64 byte đầu chắc chắn có dữ liệu — mà một bản ghi
     * toàn 0xFF là bất khả: t_ms sẽ thành 49 ngày bay liên tục.
     */
    if (!qspi_flash_read(addr, buf, sizeof(buf))) {
        return -1;
    }
    for (uint32_t i = 0; i < sizeof(buf); i++) {
        if (buf[i] != 0xFFu) {
            return 0;
        }
    }
    return 1;
}

/**
 * Tìm trang đã xoá đầu tiên bằng tìm nhị phân.
 *
 * Dữ liệu luôn mọc liên tục từ địa chỉ 0 nên các trang chia làm đúng hai
 * vùng: đã ghi rồi tới đã xoá, không xen kẽ. 15 lần đọc thay vì quét hết
 * 32768 trang.
 *
 * @return false nếu có lần đọc nào hỏng. Lúc đó *out KHÔNG dùng được — thà
 *         không ghi log còn hơn ghi đè lên chuyến bay trước.
 */
static bool find_end(uint32_t *out)
{
    uint32_t lo = 0;                    /* biết là ĐÃ GHI  */
    uint32_t hi = s_limit / PAGE;       /* coi như ĐÃ XOÁ  */
    int      r  = page_is_erased(0);

    if (r < 0) {
        return false;
    }
    if (r == 1) {
        *out = 0;
        return true;
    }

    /* Bất biến: trang lo đã ghi, trang hi đã xoá (hoặc là hết chỗ). */
    while ((hi - lo) > 1u) {
        const uint32_t mid = lo + ((hi - lo) / 2u);

        r = page_is_erased(mid * PAGE);
        if (r < 0) {
            return false;
        }
        if (r == 1) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    *out = hi * PAGE;
    return true;
}

/* ==========================================================================
 * Ghi
 * ========================================================================== */

/** Đẩy đệm đang chờ xuống chip, nếu chip đã rảnh. Không bao giờ đợi. */
static void pump(void)
{
    if (!s_pending) {
        return;
    }
    if (qspi_flash_is_busy()) {
        return;
    }
    if (!qspi_flash_program_start(s_write_addr, s_page[s_pend_idx], PAGE)) {
        s_state = FL_STATE_ERROR;
        return;
    }
    s_write_addr += PAGE;
    s_pending = false;
}

/** Đệm đầy: chuyển nó sang hàng chờ rồi quay sang đệm kia. */
static bool rotate(void)
{
    if (s_pending) {
        pump();                 /* thử dọn chỗ trước đã */
        if (s_pending) {
            return false;       /* chip vẫn bận, cả hai đệm đều kẹt */
        }
    }
    if ((s_write_addr + PAGE) > s_limit) {
        s_state = FL_STATE_FULL;
        return false;
    }
    s_pend_idx = s_fill;
    s_pending  = true;
    s_fill     = (uint8_t)(s_fill ^ 1u);
    s_fill_len = 0;
    pump();
    return true;
}

/** Nối một mẩu byte vào dòng, tự cắt theo ranh giới trang. */
static void append_bytes(const void *src, uint16_t len)
{
    const uint8_t *p = (const uint8_t *)src;

    while (len > 0u) {
        if (s_fill_len >= PAGE) {
            if (!rotate()) {
                s_dropped++;
                return;
            }
        }

        uint16_t n = (uint16_t)(PAGE - s_fill_len);

        if (n > len) {
            n = len;
        }
        memcpy(&s_page[s_fill][s_fill_len], p, n);
        s_fill_len = (uint16_t)(s_fill_len + n);
        p   += n;
        len  = (uint16_t)(len - n);
    }
}

/**
 * Ghi nốt trang dở dang, đệm 0xFF cho đủ. CHẶN tới vài ms.
 *
 * Chỉ gọi khi đã DISARM. Phần đệm 0xFF làm chuyến sau bắt đầu ở ranh giới
 * trang; người đọc bỏ qua nó bằng cách nhảy tới ranh giới kế tiếp khi gặp
 * một bản ghi toàn 0xFF.
 */
static void flush_tail(void)
{
    if (s_pending) {
        (void)qspi_flash_wait_ready(50u);
        pump();
    }
    if (s_fill_len == 0u) {
        return;
    }
    memset(&s_page[s_fill][s_fill_len], 0xFF, PAGE - s_fill_len);

    if (!qspi_flash_write_page(s_write_addr, s_page[s_fill], PAGE)) {
        s_state = FL_STATE_ERROR;
        return;
    }
    s_write_addr += PAGE;
    s_fill_len = 0;
}

/* ==========================================================================
 * Vòng đời
 * ========================================================================== */

bool flashlog_init(void)
{
    s_state    = FL_STATE_OFF;
    s_fill     = 0;
    s_fill_len = 0;
    s_pending  = false;
    s_records  = 0;
    s_dropped  = 0;

    const uint32_t total = qspi_flash_bytes();

    if (total <= FL_RESERVED_TAIL) {
        return false;                       /* không thấy chip */
    }
    s_limit = total - FL_RESERVED_TAIL;

    if (!find_end(&s_write_addr)) {
        s_state = FL_STATE_ERROR;
        return false;
    }

    s_state   = (s_write_addr < s_limit) ? FL_STATE_READY : FL_STATE_FULL;
    s_last_us = micros();
    return true;
}

void flashlog_update(uint32_t now_us)
{
    const bool armed = g_fc.motor.armed && (g_fc.mode == FC_MODE_ARMED);

    switch (s_state) {

    case FL_STATE_READY:
        if (armed) {
            flashlog_hdr_t hdr;

            memset(&hdr, 0, sizeof(hdr));
            hdr.magic     = FLASHLOG_MAGIC;
            hdr.boot_ms   = HAL_GetTick();
            hdr.rate_hz   = FLASHLOG_RATE_HZ;
            hdr.rec_bytes = (uint8_t)LOG_RECORD_BYTES;
            hdr.version   = 2u;   /* 2 = ban ghi 64 byte, them truong chan doan poshold */

            s_records = 0;
            s_dropped = 0;
            s_t0_us   = now_us;
            s_last_us = now_us;
            s_state   = FL_STATE_RECORDING;

            append_bytes(&hdr, sizeof(hdr));
        }
        break;

    case FL_STATE_RECORDING:
        pump();

        if (!armed) {
            flush_tail();
            if (s_state == FL_STATE_RECORDING) {
                s_state = (s_write_addr < s_limit) ? FL_STATE_READY
                                                   : FL_STATE_FULL;
            }
            break;
        }
        if (fc_elapsed_us(now_us, s_last_us) >= FL_PERIOD_US) {
            bb_record_t r;

            s_last_us = now_us;
            log_record_fill(&r, fc_elapsed_us(now_us, s_t0_us) / 1000u);
            append_bytes(&r, sizeof(r));
            s_records++;
        }
        break;

    case FL_STATE_OFF:
    case FL_STATE_FULL:
    case FL_STATE_ERROR:
    default:
        break;
    }
}

bool flashlog_selftest(uint32_t records)
{
    flashlog_hdr_t hdr;

    if (s_state != FL_STATE_READY) {
        return false;
    }

    memset(&hdr, 0, sizeof(hdr));
    hdr.magic     = FLASHLOG_MAGIC;
    hdr.boot_ms   = HAL_GetTick();
    hdr.rate_hz   = FLASHLOG_RATE_HZ;
    hdr.rec_bytes = (uint8_t)LOG_RECORD_BYTES;
    hdr.version   = 2u;   /* 2 = ban ghi 64 byte, them truong chan doan poshold */

    s_records = 0;
    s_dropped = 0;
    append_bytes(&hdr, sizeof(hdr));

    for (uint32_t i = 0; i < records && s_state == FL_STATE_READY; i++) {
        bb_record_t r;

        /*
         * Mốc thời gian giả theo đúng nhịp thật, để dòng CSV trút ra nhìn
         * y như một chuyến bay. Các trường còn lại là trạng thái thật của
         * máy bay lúc này — đứng yên trên bàn.
         */
        log_record_fill(&r, (i * 1000u) / FLASHLOG_RATE_HZ);
        append_bytes(&r, sizeof(r));
        s_records++;

        /*
         * Lúc bay, hai lần ghi trang cách nhau 53 ms nên chip luôn kịp rảnh.
         * Ở đây bản ghi được bơm liên tục nên phải tự đợi, nếu không rotate()
         * sẽ thấy cả hai đệm kẹt và bắt đầu đếm bản ghi bị bỏ.
         */
        while (s_pending) {
            if (!qspi_flash_wait_ready(50u)) {
                s_state = FL_STATE_ERROR;
                return false;
            }
            pump();
        }
    }

    flush_tail();
    return (s_state == FL_STATE_READY) && (s_dropped == 0u);
}

flashlog_state_t flashlog_state(void)          { return s_state; }
uint32_t         flashlog_records(void)        { return s_records; }
uint32_t         flashlog_dropped(void)        { return s_dropped; }
uint32_t         flashlog_used_bytes(void)     { return s_write_addr; }
uint32_t         flashlog_capacity_bytes(void) { return s_limit; }

const char *flashlog_state_name(void)
{
    switch (s_state) {
    case FL_STATE_OFF:       return "TAT / khong thay chip";
    case FL_STATE_READY:     return "san sang, cho arm";
    case FL_STATE_RECORDING: return "DANG GHI vao flash";
    case FL_STATE_FULL:      return "DAY - can 'flash erase'";
    case FL_STATE_ERROR:     return "LOI CHIP";
    default:                 return "?";
    }
}

#else  /* FLASHLOG_ENABLE == 0 */

bool             flashlog_init(void)           { return false; }
void             flashlog_update(uint32_t u)   { (void)u; }
flashlog_state_t flashlog_state(void)          { return FL_STATE_OFF; }
uint32_t         flashlog_records(void)        { return 0; }
uint32_t         flashlog_dropped(void)        { return 0; }
uint32_t         flashlog_used_bytes(void)     { return 0; }
uint32_t         flashlog_capacity_bytes(void) { return 0; }
bool             flashlog_selftest(uint32_t n) { (void)n; return false; }
const char      *flashlog_state_name(void)     { return "TAT bang FLASHLOG_ENABLE = 0"; }

#endif /* FLASHLOG_ENABLE */
