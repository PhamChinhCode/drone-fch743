/**
 * @file    param_store.c
 * @brief   Ghi / đọc khối cấu hình trên flash nội. Xem param_store.h.
 */
#include "param_store.h"
#include "fc_state.h"
#include "main.h"

#include <string.h>

/* ==========================================================================
 * Bố trí
 * ========================================================================== */

#define PARAM_STORE_MAGIC   0x4D504346u  /* 'FCPM' đọc theo little-endian */

#define SLOT_A_ADDR         0x081C0000u
#define SLOT_B_ADDR         0x081E0000u
#define SLOT_A_SECTOR       FLASH_SECTOR_6
#define SLOT_B_SECTOR       FLASH_SECTOR_7
#define SLOT_BANK           FLASH_BANK_2

/** Đơn vị ghi của H743: 256 bit. Không ghi lẻ hơn được. */
#define FLASH_WORD_BYTES    32u

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;        /**< sizeof(param_storage_t) lúc ghi */
    uint32_t seq;
    uint32_t table_crc;
    uint32_t data_crc;    /**< CRC32 của riêng phần param_storage_t */
} param_blob_header_t;

/** Tổng kích thước khối, làm tròn LÊN bội số của một flash-word. */
#define BLOB_BYTES  ((((sizeof(param_blob_header_t) + sizeof(param_storage_t)) \
                       + (FLASH_WORD_BYTES - 1u)) / FLASH_WORD_BYTES) * FLASH_WORD_BYTES)

_Static_assert(BLOB_BYTES <= FLASH_SECTOR_SIZE,
               "khoi cau hinh vuot mot sector flash");

/*
 * Đệm dựng khối trước khi ghi.
 *
 * `aligned(32)` không phải để làm đẹp: HAL_FLASH_Program() nhận ĐỊA CHỈ nguồn
 * và nạp 8 word 32 bit liên tiếp từ đó. Căn theo đúng flash-word tránh mọi
 * bất ngờ về truy cập lệch, và cũng là điều kiện cần nếu sau này bật D-cache
 * (thao tác dọn cache làm việc theo dòng 32 byte).
 *
 * Để static thay vì trên stack: khối cỡ vài trăm byte, mà stack chính chỉ
 * nằm trong DTCMRAM 128 KB dùng chung với mọi thứ khác.
 */
static uint8_t s_blob[BLOB_BYTES] __attribute__((aligned(32)));

static uint32_t s_current_seq;

/* ==========================================================================
 * Đọc
 * ========================================================================== */

/**
 * Kiểm một ô và cho biết nó chứa gì.
 *
 * Flash chưa ghi đọc ra toàn 0xFF, nên magic sai là dấu hiệu "trống" chứ
 * không phải "hỏng" — phân biệt hai trường hợp này để thông báo cho người
 * dùng đúng nguyên nhân.
 */
static param_store_result_t slot_inspect(uint32_t addr, uint32_t *seq_out)
{
    const param_blob_header_t *h = (const param_blob_header_t *)addr;

    if (h->magic != PARAM_STORE_MAGIC) {
        return PARAM_STORE_EMPTY;
    }

    /*
     * Kiểm size TRƯỚC khi tính CRC. Khối của một firmware khác có thể khai
     * báo size lớn hơn struct hiện tại, và đọc theo con số đó là đọc tràn ra
     * ngoài vùng dữ liệu.
     */
    if (h->size != (uint16_t)sizeof(param_storage_t)) {
        return PARAM_STORE_MISMATCH;
    }

    const void *data = (const uint8_t *)addr + sizeof(param_blob_header_t);

    if (param_crc32(data, sizeof(param_storage_t)) != h->data_crc) {
        return PARAM_STORE_CORRUPT;
    }

    /*
     * CRC đúng rồi mới xét firmware. Thứ tự này có chủ ý: một khối hỏng CRC
     * thì mọi trường trong đó đều không đáng tin, kể cả version và table_crc.
     */
    if (h->version != PARAM_SCHEMA_VERSION ||
        h->table_crc != param_table_crc32()) {
        return PARAM_STORE_MISMATCH;
    }

    if (seq_out != NULL) {
        *seq_out = h->seq;
    }
    return PARAM_STORE_OK;
}

param_store_result_t param_store_load(void)
{
    uint32_t seq_a = 0, seq_b = 0;

    const param_store_result_t res_a = slot_inspect(SLOT_A_ADDR, &seq_a);
    const param_store_result_t res_b = slot_inspect(SLOT_B_ADDR, &seq_b);

    uint32_t src = 0;

    if (res_a == PARAM_STORE_OK && res_b == PARAM_STORE_OK) {
        /*
         * Cả hai hợp lệ là trạng thái BÌNH THƯỜNG, không phải bất thường: ô
         * cũ vẫn nằm đó cho tới lần lưu kế tiếp ghi đè lên nó. Chọn seq lớn
         * hơn. Phép trừ unsigned cho kết quả đúng cả khi seq tràn.
         */
        src = ((seq_a - seq_b) < 0x80000000u) ? SLOT_A_ADDR : SLOT_B_ADDR;
        s_current_seq = (src == SLOT_A_ADDR) ? seq_a : seq_b;
    } else if (res_a == PARAM_STORE_OK) {
        src = SLOT_A_ADDR;
        s_current_seq = seq_a;
    } else if (res_b == PARAM_STORE_OK) {
        src = SLOT_B_ADDR;
        s_current_seq = seq_b;
    } else {
        /*
         * Không ô nào dùng được. g_params giữ nguyên giá trị mặc định mà
         * param_load_defaults() vừa điền — máy bay vẫn bay được, chỉ là mất
         * phần tinh chỉnh.
         *
         * Báo về nguyên nhân CỤ THỂ hơn thay vì gộp chung: "trống" là bình
         * thường ở lần chạy đầu, còn "hỏng" hay "firmware khác" thì người
         * dùng cần biết để đi phục hồi cấu hình.
         */
        s_current_seq = 0;
        if (res_a == PARAM_STORE_CORRUPT || res_b == PARAM_STORE_CORRUPT) {
            return PARAM_STORE_CORRUPT;
        }
        if (res_a == PARAM_STORE_MISMATCH || res_b == PARAM_STORE_MISMATCH) {
            return PARAM_STORE_MISMATCH;
        }
        return PARAM_STORE_EMPTY;
    }

    memcpy(&g_params,
           (const uint8_t *)src + sizeof(param_blob_header_t),
           sizeof(param_storage_t));

    return PARAM_STORE_OK;
}

/* ==========================================================================
 * Ghi
 * ========================================================================== */

/**
 * Dọn D-cache trên vùng vừa ghi.
 *
 * Hiện main.c chỉ bật I-cache (SCB_EnableICache), nên hàm này không làm gì.
 * Vẫn giữ lại và kiểm cờ lúc chạy: bật D-cache về sau là một dòng trong
 * main.c, và nếu chỗ này không có thì lỗi biểu hiện thành "ghi xong đọc lại
 * vẫn ra dữ liệu cũ" — đúng loại lỗi ngốn cả buổi để tìm.
 */
static void cache_invalidate(uint32_t addr, uint32_t len)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if (SCB->CCR & SCB_CCR_DC_Msk) {
        SCB_InvalidateDCache_by_Addr((uint32_t *)addr, (int32_t)len);
    }
#else
    (void)addr;
    (void)len;
#endif
}

static bool erase_slot(uint32_t sector)
{
    FLASH_EraseInitTypeDef erase = {
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .Banks        = SLOT_BANK,
        .Sector       = sector,
        .NbSectors    = 1u,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3,
    };
    uint32_t sector_error = 0xFFFFFFFFu;

    return HAL_FLASHEx_Erase(&erase, &sector_error) == HAL_OK;
}

static bool program_blob(uint32_t addr)
{
    for (uint32_t off = 0; off < BLOB_BYTES; off += FLASH_WORD_BYTES) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                              addr + off,
                              (uint32_t)&s_blob[off]) != HAL_OK) {
            return false;
        }
    }
    return true;
}

param_store_result_t param_store_save(void)
{
    /*
     * Lớp chặn thứ hai. tlm_stream.c đã từ chối mọi lệnh ACTION khi đang ARM,
     * nhưng hàm này cũng gọi được từ CLI và từ code khác về sau. Hậu quả của
     * việc quên chặn là ngừng phát DShot giữa không trung, nên chặn hai lần.
     */
    if (g_fc.mode == FC_MODE_ARMED) {
        return PARAM_STORE_ERR_ARMED;
    }

    /* Ô nào KHÔNG chứa bản mới nhất thì ghi vào đó. */
    uint32_t seq_a = 0;
    const bool a_is_current = (slot_inspect(SLOT_A_ADDR, &seq_a) == PARAM_STORE_OK) &&
                              (seq_a == s_current_seq) && (s_current_seq != 0);

    const uint32_t dst_addr   = a_is_current ? SLOT_B_ADDR   : SLOT_A_ADDR;
    const uint32_t dst_sector = a_is_current ? SLOT_B_SECTOR : SLOT_A_SECTOR;

    /* Dựng khối. Phần đệm cuối để 0 cho CRC lặp lại được giữa các lần chạy. */
    memset(s_blob, 0, sizeof(s_blob));

    param_blob_header_t *h = (param_blob_header_t *)s_blob;
    h->magic     = PARAM_STORE_MAGIC;
    h->version   = PARAM_SCHEMA_VERSION;
    h->size      = (uint16_t)sizeof(param_storage_t);
    h->seq       = s_current_seq + 1u;
    h->table_crc = param_table_crc32();
    h->data_crc  = param_crc32(&g_params, sizeof(param_storage_t));

    memcpy(s_blob + sizeof(param_blob_header_t), &g_params, sizeof(param_storage_t));

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return PARAM_STORE_ERR_FLASH;
    }

    const bool ok = erase_slot(dst_sector) && program_blob(dst_addr);

    HAL_FLASH_Lock();

    if (!ok) {
        return PARAM_STORE_ERR_FLASH;
    }

    cache_invalidate(dst_addr, BLOB_BYTES);

    /*
     * Đọc lại và so từng byte. Không tin HAL_OK là đủ: nó chỉ báo bộ máy flash
     * không kêu lỗi, không bảo đảm nội dung đúng. Với ECC của H743 thì một ô
     * đã ghi mà bị ghi lại lần nữa sẽ không báo lỗi ngay lúc ghi nhưng đọc ra
     * sai — chỉ so sánh mới thấy.
     */
    if (memcmp((const void *)dst_addr, s_blob, BLOB_BYTES) != 0) {
        return PARAM_STORE_ERR_VERIFY;
    }

    s_current_seq = h->seq;
    return PARAM_STORE_OK;
}

uint32_t param_store_seq(void)
{
    return s_current_seq;
}

const char *param_store_result_name(param_store_result_t result)
{
    switch (result) {
    case PARAM_STORE_OK:         return "ok";
    case PARAM_STORE_EMPTY:      return "trong";
    case PARAM_STORE_CORRUPT:    return "hong-crc";
    case PARAM_STORE_MISMATCH:   return "khac-firmware";
    case PARAM_STORE_ERR_ARMED:  return "dang-arm";
    case PARAM_STORE_ERR_FLASH:  return "loi-flash";
    case PARAM_STORE_ERR_VERIFY: return "kiem-lai-sai";
    default:                     return "?";
    }
}
