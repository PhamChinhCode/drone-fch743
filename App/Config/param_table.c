/**
 * @file    param_table.c
 * @brief   Dựng bảng mô tả tham số và các phép truy cập theo chỉ số.
 *
 * Toàn bộ bảng sinh ra từ param_list.h bằng X-macro — xem giải thích ở đầu
 * hai file đó. File này chỉ chứa phần LOGIC: đổi kiểu, kẹp giá trị, tìm theo
 * tên, tính CRC bảng.
 */
#include "param_table.h"

#include <math.h>
#include <string.h>

/* ==========================================================================
 * Kiểm tra lúc biên dịch
 *
 * Tên tham số đi vào bản tin PARAM_VALUE với đúng PARAM_NAME_MAX byte. Tên
 * quá dài sẽ bị cắt cụt lúc chạy và app PC nhận về một cái tên khác — lỗi
 * rất khó nhận ra. Bắt ngay ở đây.
 * ========================================================================== */
#define P(name, ctype, ptype, flags, min, max, def) \
    _Static_assert(sizeof(#name) <= PARAM_NAME_MAX, \
                   "ten tham so vuot PARAM_NAME_MAX: " #name);
#include "param_list.h"
#undef P

/* ==========================================================================
 * Dữ liệu
 * ========================================================================== */

param_storage_t g_params;

const param_meta_t g_param_table[] = {
#define P(name, ctype, ptype, flags, min, max, def)                       \
    { #name, (uint8_t)(ptype), (uint8_t)(flags),                          \
      (uint16_t)offsetof(param_storage_t, name),                          \
      (float)(min), (float)(max), (float)(def) },
#include "param_list.h"
#undef P
};

const uint16_t g_param_count =
    (uint16_t)(sizeof(g_param_table) / sizeof(g_param_table[0]));

/* ==========================================================================
 * Truy cập trường
 * ========================================================================== */

static void *field_ptr(uint16_t index)
{
    return (uint8_t *)&g_params + g_param_table[index].offset;
}

/**
 * Giá trị THỰC SỰ sẽ được cất giữ khi ghi `value` vào tham số `index`.
 *
 * Gộp hai phép biến đổi vào một chỗ để mọi đường ghi đều đi qua đúng một logic:
 *   - kẹp về [min, max]
 *   - làm tròn nếu trường là số nguyên
 *
 * Nhờ vậy param_is_modified() so sánh được chính xác: nó chỉ cần hỏi "nếu
 * ghi giá trị mặc định vào thì ra số nào" rồi so với số đang có, thay vì so
 * với cột `def` thô (vốn có thể nằm ngoài min/max hoặc là số lẻ ở trường int).
 */
static float quantize(uint16_t index, float value)
{
    const param_meta_t *m = &g_param_table[index];

    /*
     * Bảo vệ khỏi min/max gõ ngược nhau. fc_constrainf() với lo > hi trả về
     * hi cho MỌI đầu vào — tức tham số kẹt cứng ở một giá trị mà không có gì
     * báo. Thà bỏ qua phép kẹp còn hơn kẹp sai lặng lẽ.
     */
    if (m->min <= m->max) {
        value = fc_constrainf(value, m->min, m->max);
    }

    if (m->type != PT_F32) {
        value = roundf(value);
    }
    return value;
}

static void store(uint16_t index, float value)
{
    void *p = field_ptr(index);

    switch (g_param_table[index].type) {
    case PT_U8:  *(uint8_t  *)p = (uint8_t)value;  break;
    case PT_I8:  *(int8_t   *)p = (int8_t)value;   break;
    case PT_U16: *(uint16_t *)p = (uint16_t)value; break;
    case PT_I16: *(int16_t  *)p = (int16_t)value;  break;
    case PT_U32: *(uint32_t *)p = (uint32_t)value; break;
    case PT_I32: *(int32_t  *)p = (int32_t)value;  break;
    case PT_F32: *(float    *)p = value;           break;
    default:                                       break;
    }
}

/* ==========================================================================
 * API
 * ========================================================================== */

void param_load_defaults(void)
{
    /*
     * Memset TRƯỚC khi điền. Struct không packed nên có thể có byte đệm giữa
     * các trường; byte đệm không khởi tạo sẽ làm CRC của khối lưu flash đổi
     * ngẫu nhiên giữa hai lần chạy, và param_store sẽ tưởng khối bị hỏng.
     * Xem giải thích dài trong param_table.h.
     */
    memset(&g_params, 0, sizeof(g_params));

    for (uint16_t i = 0; i < g_param_count; i++) {
        store(i, quantize(i, g_param_table[i].def));
    }
}

float param_get_f32(uint16_t index)
{
    if (index >= g_param_count) {
        return 0.0f;
    }

    const void *p = (const uint8_t *)&g_params + g_param_table[index].offset;

    switch (g_param_table[index].type) {
    case PT_U8:  return (float)*(const uint8_t  *)p;
    case PT_I8:  return (float)*(const int8_t   *)p;
    case PT_U16: return (float)*(const uint16_t *)p;
    case PT_I16: return (float)*(const int16_t  *)p;
    case PT_U32: return (float)*(const uint32_t *)p;
    case PT_I32: return (float)*(const int32_t  *)p;
    case PT_F32: return        *(const float    *)p;
    default:     return 0.0f;
    }
}

bool param_set_f32(uint16_t index, float value)
{
    if (index >= g_param_count) {
        return false;
    }
    if (g_param_table[index].flags & PARAM_FLAG_READONLY) {
        return false;
    }

    /*
     * NaN không so sánh được với bất cứ giá trị nào nên fc_constrainf() để nó
     * lọt qua nguyên vẹn, và một NaN trong hệ số PID sẽ lan ra toàn bộ vòng
     * điều khiển. Chặn ngay tại cửa.
     */
    if (isnan(value) || isinf(value)) {
        return false;
    }

    store(index, quantize(index, value));
    return true;
}

uint16_t param_find(const char *name)
{
    if (name == NULL) {
        return PARAM_INDEX_NONE;
    }
    for (uint16_t i = 0; i < g_param_count; i++) {
        if (strcmp(g_param_table[i].name, name) == 0) {
            return i;
        }
    }
    return PARAM_INDEX_NONE;
}

bool param_is_modified(uint16_t index)
{
    if (index >= g_param_count) {
        return false;
    }
    return param_get_f32(index) != quantize(index, g_param_table[index].def);
}

/* ==========================================================================
 * CRC32
 *
 * Dùng bản tính từng bit thay vì bảng tra 1 KB: hàm này chỉ chạy lúc khởi
 * động và lúc lưu cấu hình, còn 1 KB flash thì luôn đáng tiếc hơn vài trăm
 * micro giây chạy một lần.
 *
 * Đa thức đảo 0xEDB88320 — CRC-32/ISO-HDLC, cùng loại zip và Ethernet dùng,
 * nên bên máy tính có sẵn (Python: zlib.crc32, C#: System.IO.Hashing.Crc32).
 * ========================================================================== */

uint32_t param_crc32(const void *data, size_t len)
{
    const uint8_t *p   = (const uint8_t *)data;
    uint32_t       crc = 0xFFFFFFFFu;

    while (len--) {
        crc ^= *p++;
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

/**
 * Băm nội dung bảng mô tả.
 *
 * Băm CHUỖI TÊN chứ không phải con trỏ tên — con trỏ đổi mỗi lần liên kết lại
 * dù nội dung y hệt, và khi đó mọi cấu hình đã lưu sẽ bị vứt sau mỗi lần build
 * dù chẳng có gì thay đổi.
 *
 * Băm luôn cả min/max/def: đổi giới hạn của một tham số cũng là đổi ý nghĩa
 * của nó, app PC cần biết để đọc lại.
 */
uint32_t param_table_crc32(void)
{
    static uint32_t cached = 0;
    static bool     valid  = false;

    if (valid) {
        return cached;
    }

    uint32_t crc = 0xFFFFFFFFu;

    for (uint16_t i = 0; i < g_param_count; i++) {
        const param_meta_t *m = &g_param_table[i];

        /* Gom phần nhị phân của một mục vào đệm tạm rồi băm một lượt. */
        uint8_t buf[sizeof(m->type) + sizeof(m->flags) + sizeof(m->offset) +
                    sizeof(m->min) + sizeof(m->max) + sizeof(m->def)];
        size_t  n = 0;

        memcpy(buf + n, &m->type,   sizeof(m->type));   n += sizeof(m->type);
        memcpy(buf + n, &m->flags,  sizeof(m->flags));  n += sizeof(m->flags);
        memcpy(buf + n, &m->offset, sizeof(m->offset)); n += sizeof(m->offset);
        memcpy(buf + n, &m->min,    sizeof(m->min));    n += sizeof(m->min);
        memcpy(buf + n, &m->max,    sizeof(m->max));    n += sizeof(m->max);
        memcpy(buf + n, &m->def,    sizeof(m->def));    n += sizeof(m->def);

        /*
         * param_crc32() tự khởi tạo và tự đảo cuối, nên không nối chuỗi được.
         * Ở đây cộng dồn thủ công: băm từng khối rồi trộn vào crc đang chạy.
         */
        crc ^= param_crc32(m->name, strlen(m->name));
        crc ^= param_crc32(buf, n);
    }

    cached = crc;
    valid  = true;
    return cached;
}
