/**
 * @file    dbg_console.c
 * @brief   Hiện thực console gỡ lỗi dạng chữ.
 */

#include "dbg_console.h"
#include "fc_state.h"
#include "fc_time.h"
#include "icm20602.h"
#include "lsm6dsv.h"
#include "bmp388.h"
#if MAG_SOURCE == MAG_SOURCE_I2C
#include "mag_i2c.h"
#endif
#include "mtf01p.h"
#include "crsf.h"
#include "dshot.h"
#include "arming.h"
#include "ctrl_angle.h"
#include "ctrl_poshold.h"
#include "ctrl_althold.h"
#include "ctrl_rate.h"
#include "mixer.h"
#include "estimator.h"
#include "blackbox.h"
#include "ekf_attitude.h"
#include "ekf_altitude.h"
#include "ekf_velocity.h"

/* ==========================================================================
 * Bộ đệm gửi
 *
 * Là nguồn trực tiếp của DMA khi UART có cấu hình DMA TX, nên bắt buộc dùng
 * FC_DMA_BUFFER để nằm ở AXI SRAM. Xem chú thích trong fc_types.h.
 * ========================================================================== */

FC_DMA_BUFFER static uint8_t s_tx[DBG_TX_BUFFER_SIZE];

static UART_HandleTypeDef *s_uart;
static volatile uint16_t   s_head;
static volatile uint16_t   s_tail;
static volatile uint16_t   s_inflight;
static volatile bool       s_busy;
static uint32_t            s_dropped;

static dbg_mode_t s_mode      = DBG_MODE_OFF;
static uint16_t   s_period_ms = 1000u / DBG_DEFAULT_RATE_HZ;
static uint32_t   s_next_ms;
static uint16_t   s_line_count;

/* ==========================================================================
 * Ring buffer
 * ========================================================================== */

static inline uint32_t critical_enter(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void critical_exit(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static inline uint16_t tx_used(void)
{
    const uint16_t h = s_head;
    const uint16_t t = s_tail;
    return (uint16_t)((h >= t) ? (h - t) : (DBG_TX_BUFFER_SIZE - t + h));
}

static inline uint16_t tx_free(void)
{
    return (uint16_t)(DBG_TX_BUFFER_SIZE - 1u - tx_used());
}

/** Nạp khối liên tiếp kế tiếp cho phần cứng. Gọi khi đang rảnh. */
static void tx_kick(void)
{
    if (s_busy || s_uart == NULL) {
        return;
    }

    const uint16_t h = s_head;
    const uint16_t t = s_tail;
    if (h == t) {
        return;
    }

    /* Phần cứng chỉ gửi được vùng liên tiếp; vòng qua cuối đệm thì cắt hai lần. */
    const uint16_t chunk = (h > t) ? (uint16_t)(h - t)
                                   : (uint16_t)(DBG_TX_BUFFER_SIZE - t);

    s_busy     = true;
    s_inflight = chunk;

    /*
     * Dùng DMA nếu UART này có cấu hình DMA TX, không thì lùi về chế độ ngắt.
     * Nhờ vậy module chạy được trên USART1 (có DMA) lẫn USART3 (chỉ có ngắt)
     * mà không phải sửa dòng nào.
     */
    const HAL_StatusTypeDef st =
        (s_uart->hdmatx != NULL)
            ? HAL_UART_Transmit_DMA(s_uart, &s_tx[t], chunk)
            : HAL_UART_Transmit_IT(s_uart, &s_tx[t], chunk);

    if (st != HAL_OK) {
        s_busy     = false;
        s_inflight = 0;
    }
}

/** Chép len byte vào ring buffer. Bỏ trọn cả khối nếu không đủ chỗ. */
static bool tx_push(const char *data, uint16_t len)
{
    if (s_uart == NULL || len == 0) {
        return false;
    }
    if (len > tx_free()) {
        s_dropped += len;
        return false;
    }

    const uint32_t primask = critical_enter();

    uint16_t h = s_head;
    for (uint16_t i = 0; i < len; i++) {
        s_tx[h] = (uint8_t)data[i];
        h = (uint16_t)((h + 1u) % DBG_TX_BUFFER_SIZE);
    }
    s_head = h;

    tx_kick();
    critical_exit(primask);
    return true;
}

void dbg_console_tx_complete_isr(void)
{
    s_tail     = (uint16_t)((s_tail + s_inflight) % DBG_TX_BUFFER_SIZE);
    s_inflight = 0;
    s_busy     = false;
    tx_kick();
}

uint16_t dbg_console_tx_free(void)
{
    return tx_free();
}

uint32_t dbg_console_dropped(void)
{
    return s_dropped;
}

/* ==========================================================================
 * Bộ ghi chuỗi — thay thế printf
 * ========================================================================== */

typedef struct {
    char    *buf;
    uint16_t len;
    uint16_t cap;
} wr_t;

static void wr_ch(wr_t *w, char c)
{
    if (w->len < w->cap) {
        w->buf[w->len++] = c;
    }
}

static void wr_str(wr_t *w, const char *s)
{
    while (*s != '\0') {
        wr_ch(w, *s++);
    }
}

static void wr_spaces(wr_t *w, int n)
{
    for (int i = 0; i < n; i++) {
        wr_ch(w, ' ');
    }
}

/** Số nguyên có dấu, căn phải trong trường rộng `width` (0 = sát trái). */
/** Chuoi can TRAI trong truong rong `width`, dem khoang trang phia sau. */
static void wr_str_pad(wr_t *w, const char *s, int width)
{
    int n = 0;

    while (s[n] != 0) { n++; }
    wr_str(w, s);
    wr_spaces(w, width - n);
}


static void wr_i32(wr_t *w, int32_t v, int width)
{
    char    tmp[12];
    int     n   = 0;
    bool    neg = (v < 0);
    /* 0u - (uint32_t)v xu ly dung ca truong hop INT32_MIN, khong can 64-bit. */
    uint32_t u  = neg ? (0u - (uint32_t)v) : (uint32_t)v;

    do {
        tmp[n++] = (char)('0' + (u % 10u));
        u /= 10u;
    } while (u != 0u);

    if (neg) {
        tmp[n++] = '-';
    }

    wr_spaces(w, width - n);
    while (n > 0) {
        wr_ch(w, tmp[--n]);
    }
}

/** Số nguyên không dấu dạng hex, luôn đủ `digits` ký tự. */
static void wr_hex(wr_t *w, uint32_t v, int digits)
{
    static const char hex[] = "0123456789ABCDEF";
    wr_str(w, "0x");
    for (int i = digits - 1; i >= 0; i--) {
        wr_ch(w, hex[(v >> (i * 4)) & 0xFu]);
    }
}

/** Gia tri hex dang 0xNNNN, can PHAI trong truong rong `width`. */
static void wr_hex_col(wr_t *w, uint32_t v, int digits, int width)
{
    /* wr_hex TU THEM tien to "0x", nen be rong that la digits + 2. */
    wr_spaces(w, width - (digits + 2));
    wr_hex(w, v, digits);
}

/**
 * Số thực dạng dấu phẩy cố định, căn phải.
 * Không dùng printf: newlib-nano bỏ %f trừ khi thêm cờ -u _printf_float.
 */
static void wr_fix(wr_t *w, float v, uint8_t decimals, int width)
{
    /* Giá trị bất thường vẫn phải in ra được, không làm hỏng cả dòng. */
    if (isnan(v)) { wr_spaces(w, width - 3); wr_str(w, "nan"); return; }
    if (isinf(v)) { wr_spaces(w, width - 4); wr_str(w, v > 0 ? " inf" : "-inf"); return; }

    int32_t mul = 1;
    for (uint8_t i = 0; i < decimals; i++) {
        mul *= 10;
    }

    const bool neg = (v < 0.0f);
    if (neg) {
        v = -v;
    }

    /* Chặn tràn khi giá trị vô lý (cảm biến lỗi). */
    if (v > 2.0e6f) {
        v = 2.0e6f;
    }

    const int32_t scaled = (int32_t)(v * (float)mul + 0.5f);
    int32_t ip = scaled / mul;
    int32_t fp = scaled % mul;

    char tmp[20];
    int  n = 0;

    for (uint8_t i = 0; i < decimals; i++) {
        tmp[n++] = (char)('0' + (fp % 10));
        fp /= 10;
    }
    if (decimals > 0) {
        tmp[n++] = '.';
    }
    if (ip == 0) {
        tmp[n++] = '0';
    } else {
        while (ip != 0) {
            tmp[n++] = (char)('0' + (ip % 10));
            ip /= 10;
        }
    }
    if (neg) {
        tmp[n++] = '-';
    }

    wr_spaces(w, width - n);
    while (n > 0) {
        wr_ch(w, tmp[--n]);
    }
}

/** Mot byte dang hex hai ky tu, khong co tien to "0x". */
static void wr_hex_byte(wr_t *w, uint8_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    wr_ch(w, hex[(v >> 4) & 0xFu]);
    wr_ch(w, hex[v & 0xFu]);
}

static void wr_eol(wr_t *w)
{
    wr_ch(w, '\r');
    wr_ch(w, '\n');
}

/* ==========================================================================
 * API in thủ công
 * ========================================================================== */

void dbg_puts(const char *s)
{
    if (s == NULL) {
        return;
    }
    (void)tx_push(s, (uint16_t)strlen(s));
}

void dbg_println(const char *s)
{
    dbg_puts(s);
    (void)tx_push("\r\n", 2);
}

void dbg_print_int(const char *label, int32_t value)
{
    char  line[DBG_LINE_MAX];
    wr_t  w = { line, 0, sizeof(line) };

    wr_str(&w, label);
    wr_str(&w, " = ");
    wr_i32(&w, value, 0);
    wr_eol(&w);
    (void)tx_push(line, w.len);
}

void dbg_print_hex(const char *label, uint32_t value, uint8_t digits)
{
    char  line[DBG_LINE_MAX];
    wr_t  w = { line, 0, sizeof(line) };

    wr_str(&w, label);
    wr_str(&w, " = ");
    wr_hex(&w, value, (int)digits);   /* wr_hex tu them tien to "0x" */
    wr_eol(&w);
    (void)tx_push(line, w.len);
}

void dbg_print_float(const char *label, float value, uint8_t decimals)
{
    char  line[DBG_LINE_MAX];
    wr_t  w = { line, 0, sizeof(line) };

    wr_str(&w, label);
    wr_str(&w, " = ");
    wr_fix(&w, value, decimals, 0);
    wr_eol(&w);
    (void)tx_push(line, w.len);
}

/* ==========================================================================
 * Các định dạng in định kỳ
 * ========================================================================== */


/* ==========================================================================
 * Tu nhan chieu truc cho IMU phu - cong cu giai doan 2A
 *
 * Nguoi dung lan luot dat may bay o BA tu the. Voi moi tu the ta BIET TRUOC
 * vector gia toc phai ra gi trong he THAN, con cam bien thi bao cho ta biet
 * truc nao cua NO dang chiu luc va dau ra sao. Ghep hai dieu do lai la ra
 * duoc IMU2_AXIS_MAP_* va IMU2_AXIS_SIGN_*.
 *
 *   buoc 1  nam phang, mat tren huong len  -> than a = (0, 0, -1) g
 *   buoc 2  dung tren duoi, mui huong len  -> than a = (+1, 0, 0) g
 *   buoc 3  nam nghieng, canh PHAI cham dat-> than a = (0, -1, 0) g
 *
 * Vi sao dau AM o buoc 1: he than la NED, truc Z huong XUONG, ma gia toc ke
 * do LUC RIENG (phan luc mat ban) huong LEN. Nam yen thi so doc phai am.
 *
 * Cong thuc suy ra: goi k la truc CAM BIEN chiem uu the va s = +-1 la dau
 * cua no. Ta can  SIGN * sensor[k] = gia tri than mong doi, nen:
 *     MAP  = k
 *     SIGN = (gia tri than mong doi) * s      (vi s*s = 1)
 * ========================================================================== */

#define AXCAL_HOLD_TICKS   40u    /* ~0,8 s o nhip console 50 Hz */
#define AXCAL_DOMINANCE    0.90f  /* mot truc phai chiem >90% do lon vector */

/** Gia tri than mong doi cua tung buoc, theo thu tu buoc 1, 2, 3. */
static const int8_t s_axcal_want[3]     = { -1, +1, -1 };
/** Buoc thu i xac dinh truc THAN nao: 2 = Z, 0 = X, 1 = Y. */
static const uint8_t s_axcal_body[3]    = {  2,  0,  1 };
static const char   *s_axcal_name[3]    = { "Z", "X", "Y" };
static const char   *s_axcal_pose[3]    = {
    "NAM PHANG, mat tren huong LEN",
    "DUNG TREN DUOI, mui huong LEN",
    "NAM NGHIENG, canh PHAI cham dat"
};

static struct {
    int8_t map[3];      /* map[truc than] = truc cam bien, -1 = chua bat */
    int8_t sign[3];
} s_axcal;

static uint8_t s_axcal_step;
static uint8_t s_axcal_hold;
static int8_t  s_axcal_cand;      /* truc cam bien dang ung cu, -1 = khong */
static int8_t  s_axcal_cand_sign;
static bool    s_axcal_wait;      /* vua chot xong, cho nguoi dung xoay may  */
static bool    s_axcal_reported;

static void axcal_reset(void)
{
    for (int i = 0; i < 3; i++) {
        s_axcal.map[i]  = -1;
        s_axcal.sign[i] = 0;
    }
    s_axcal_step      = 0;
    s_axcal_hold      = 0;
    s_axcal_cand      = -1;
    s_axcal_cand_sign = 0;
    s_axcal_wait      = false;
    s_axcal_reported  = false;
}

/**
 * Truc cam bien nay da bi mot buoc TRUOC chiem chua?
 *
 * Ba tu the phai roi vao ba truc cam bien KHAC NHAU - do la he qua truc tiep
 * cua viec phep xoay la mot hoan vi. Neu hai buoc cung bat mot truc thi chac
 * chan may bay chua duoc xoay, va ket qua se vo nghia.
 */
static bool axcal_axis_taken(int k)
{
    for (int i = 0; i < 3; i++) {
        if (s_axcal.map[i] == (int8_t)k) { return true; }
    }
    return false;
}


/* ==========================================================================
 * Hiệu chuẩn từ kế — công cụ giai đoạn 4
 *
 * NGUYÊN LÝ: từ trường Trái Đất có ĐỘ LỚN không đổi. Xoay máy bay theo mọi
 * hướng thì đầu mút vector từ trường phải vẽ ra một MẶT CẦU tâm gốc toạ độ.
 *
 * Thực tế nó vẽ ra một ellipsoid bị dời tâm:
 *   - Tâm bị dời  = sắt CỨNG: nam châm trong động cơ tạo từ trường cộng thêm
 *                   một hằng số vào mọi số đo.
 *   - Méo thành ellipsoid = sắt MỀM: kim loại quanh chip bẻ cong đường sức,
 *                   làm độ nhạy mỗi trục khác nhau.
 *
 * Cách chữa: tìm tâm rồi dời về gốc (bù sắt cứng), tìm ba bán trục rồi kéo
 * về bằng nhau (bù sắt mềm).
 *
 * Ở đây dùng min/max từng trục thay vì khớp ellipsoid đầy đủ. Đơn giản, bền,
 * và đủ tốt khi ellipsoid không bị XOAY — điều đúng khi các trục cảm biến
 * trùng với hướng méo, tức phần lớn trường hợp thực tế.
 *
 * ĐIỀU KIỆN TIÊN QUYẾT: phải xoay đủ MỌI HƯỚNG. Chỉ xoay quanh một trục thì
 * min/max của hai trục kia sai hoàn toàn. Cột "phu" đếm số góc phần tám đã
 * chạm tới để biết đã xoay đủ chưa.
 * ========================================================================== */

#define MAGCAL_MIN_SAMPLES  300u   /* dưới mức này thì chưa tin được */

/*
 * KHỚP ELLIPSOID BẰNG BÌNH PHƯƠNG TỐI THIỂU — không dùng min/max.
 *
 * Bản đầu dùng min/max từng trục. ĐÃ ĐO và thấy nó tệ: dao động |B| sau hiệu
 * chuẩn còn 9,8%, trong khi khớp bình phương tối thiểu trên CÙNG bộ dữ liệu
 * cho 1,7%.
 *
 * Lý do: min/max chỉ dùng đúng SÁU điểm cực trị trong hàng nghìn mẫu. Không
 * xoay tới đúng cực trị của một trục thì tâm và bán trục của trục đó sai
 * ngay — và tệ hơn, nó hiểu nhầm việc xoay không đều thành méo sắt mềm, sinh
 * ra hệ số tỉ lệ giả. Trên bo này min/max cho ra (0,99 · 0,91 · 1,11) trong
 * khi giá trị đúng là (1,00 · 1,00 · 1,01) — tức từ kế gần như KHÔNG có méo
 * sắt mềm, toàn bộ sai lệch là do TÂM.
 *
 * Khớp bình phương tối thiểu dùng MỌI mẫu nên không phụ thuộc việc có chạm
 * đúng cực trị hay không.
 *
 * Mô hình: điểm nằm trên mặt ellipsoid TRỤC-THẲNG  dᵀv = 1  với
 *     d = [x², y², z², 2x, 2y, 2z]
 * Tích luỹ trực tiếp phương trình chuẩn tắc AᵀA và Aᵀ1 nên KHÔNG cần lưu mẫu
 * — 27 phép nhân cộng mỗi mẫu, ở 20 Hz là không đáng kể.
 *
 * ⚠️ VÌ SAO KHÔNG DÙNG 9 THAM SỐ (có số hạng chéo 2xy, 2xz, 2yz):
 *
 * Bản trước khớp ellipsoid ĐẦY ĐỦ có xoay rồi rút hệ số tỉ lệ bằng cách lấy
 * ĐƯỜNG CHÉO của ma trận A. Sai: đường chéo chỉ bằng trị riêng khi ellipsoid
 * KHÔNG xoay. Có xoay là hệ số tỉ lệ sai, mà rút đúng bằng trị riêng cũng
 * không cứu được — vì driver chỉ áp được
 *     cal[i] = (raw[i] - offset[i]) * scale[i]
 * tức mô hình TRỤC-THẲNG, không có chỗ đặt phép xoay.
 *
 * Khớp một mô hình rồi áp một mô hình khác thì kết quả không tối ưu theo bất
 * kỳ nghĩa nào. Nên khớp ĐÚNG thứ sẽ được áp dụng.
 *
 * ĐO TRÊN DỮ LIỆU THẬT (logimu.txt, 800 mẫu, QMC5883P):
 *     bản 9 tham số + lấy đường chéo : dao động |B| 9,3%
 *     bản 6 tham số trục-thẳng       : dao động |B| 3,2%
 * Bỏ 3 ẩn còn làm hệ bớt suy biến khi độ phủ góc không đều — bộ dữ liệu trên
 * làm hệ 9x9 SUY BIẾN hoàn toàn, còn hệ 6x6 vẫn giải được.
 *
 * Cái giá: không bù được sắt mềm có XOAY. Chấp nhận được — sắt mềm trên bo
 * này đo ra gần như bằng không (ba hệ số tỉ lệ đều ~1,00), và dù có thì
 * driver cũng không áp được.
 *
 * Dùng double cho bộ tích luỹ: giá trị lên tới x⁴ ~ 1e-2 cộng dồn hàng nghìn
 * lần, float 32 bit sẽ mất chính xác. M7 phải giả lập double bằng phần mềm
 * nhưng ở 20 Hz thì vài micro giây mỗi mẫu là quá rẻ so với độ chính xác.
 */
#define MAGCAL_N 6

static struct {
    double   ata[MAGCAL_N][MAGCAL_N];
    double   atb[MAGCAL_N];
    uint32_t n;
    uint8_t  octant;
    bool     started;
} s_magcal;

static void magcal_reset(void)
{
    memset(&s_magcal, 0, sizeof(s_magcal));
}

static int magcal_coverage(void)
{
    int n = 0;

    for (int i = 0; i < 8; i++) {
        if ((s_magcal.octant & (1u << i)) != 0u) { n++; }
    }
    return n;
}

/** Khử Gauss có chọn trụ. Trả về false nếu ma trận suy biến. */
static bool magcal_solve(double m[MAGCAL_N][MAGCAL_N + 1], double out[MAGCAL_N])
{
    for (int i = 0; i < MAGCAL_N; i++) {
        int piv = i;

        for (int k = i + 1; k < MAGCAL_N; k++) {
            if (fabs(m[k][i]) > fabs(m[piv][i])) { piv = k; }
        }
        if (fabs(m[piv][i]) < 1e-18) { return false; }
        if (piv != i) {
            for (int j = 0; j <= MAGCAL_N; j++) {
                const double t = m[i][j]; m[i][j] = m[piv][j]; m[piv][j] = t;
            }
        }
        for (int k = i + 1; k < MAGCAL_N; k++) {
            const double f = m[k][i] / m[i][i];

            for (int j = i; j <= MAGCAL_N; j++) { m[k][j] -= f * m[i][j]; }
        }
    }
    for (int i = MAGCAL_N - 1; i >= 0; i--) {
        double acc = m[i][MAGCAL_N];

        for (int j = i + 1; j < MAGCAL_N; j++) { acc -= m[i][j] * out[j]; }
        out[i] = acc / m[i][i];
    }
    return true;
}

/**
 * Giải ra tâm và ba hệ số tỉ lệ từ bộ tích luỹ.
 *
 * @param off  ra: tâm ellipsoid (bù sắt cứng), Gauss
 * @param sc   ra: ba hệ số tỉ lệ, đã chuẩn hoá quanh 1
 * @param rad  ra: bán kính sau hiệu chỉnh, tức |B| kỳ vọng
 * @return false nếu chưa đủ dữ liệu hoặc hệ suy biến
 */
static bool magcal_fit(float off[3], float sc[3], float *rad)
{
    double m[MAGCAL_N][MAGCAL_N + 1];
    double v[MAGCAL_N];

    if (s_magcal.n < 20u) { return false; }

    /* ata chi tich luy nua tren vi doi xung — soi guong lai khi dung. */
    for (int i = 0; i < MAGCAL_N; i++) {
        for (int j = 0; j < MAGCAL_N; j++) {
            m[i][j] = (j >= i) ? s_magcal.ata[i][j] : s_magcal.ata[j][i];
        }
        m[i][MAGCAL_N] = s_magcal.atb[i];
    }
    if (!magcal_solve(m, v)) { return false; }

    /*
     * A là ma trận ĐƯỜNG CHÉO — mô hình trục-thẳng không có số hạng chéo.
     * Nhờ vậy đường chéo CHÍNH LÀ trị riêng, và phép rút hệ số tỉ lệ ở dưới
     * là đúng chứ không còn là phép xấp xỉ như bản 9 tham số.
     */
    const double A[3]  = { v[0], v[1], v[2] };
    const double bb[3] = { v[3], v[4], v[5] };

    for (int i = 0; i < 3; i++) {
        if (fabs(A[i]) < 1e-18) { return false; }
    }

    /* Tâm: c[i] = -b[i]/A[i]. A đường chéo nên không cần Cramer. */
    double c[3];
    for (int i = 0; i < 3; i++) { c[i] = -bb[i] / A[i]; }

    /* k = 1 + cᵀAc, rồi A/k cho bình phương hệ số tỉ lệ. */
    double k = 1.0;
    for (int i = 0; i < 3; i++) { k += c[i] * A[i] * c[i]; }
    if (k <= 0.0) { return false; }

    double d[3];
    for (int i = 0; i < 3; i++) {
        const double a = A[i] / k;

        if (a <= 0.0) { return false; }
        d[i] = sqrt(a);
    }

    /*
     * Chuẩn hoá quanh 1 để độ lớn sau hiệu chỉnh vẫn tính bằng Gauss chứ
     * không thành số không thứ nguyên. Trung bình nhân giữ nguyên thể tích.
     */
    const double gm = pow(d[0] * d[1] * d[2], 1.0 / 3.0);

    if (gm <= 0.0) { return false; }

    for (int i = 0; i < 3; i++) {
        off[i] = (float)c[i];
        sc[i]  = (float)(d[i] / gm);
    }
    *rad = (float)(1.0 / gm);
    return true;
}

static void emit_header(void)
{
    char line[DBG_LINE_MAX];
    wr_t w = { line, 0, sizeof(line) };

    switch (s_mode) {
    case DBG_MODE_IMU:
        wr_str(&w, "     gx      gy      gz |     ax      ay      az |  temp |   dt |    count | err | cal");
        break;
    case DBG_MODE_IMU_RAW:
        wr_str(&w, "  raw_ax  raw_ay  raw_az |  raw_gx  raw_gy  raw_gz |  az[g]");
        break;
    case DBG_MODE_IMU_CSV:
        wr_str(&w, "t_us,gx_dps,gy_dps,gz_dps,ax_mps2,ay_mps2,az_mps2,temp_c");
        break;
    case DBG_MODE_FLOW:
        wr_str(&w, "     fx      fy | qual | range_mm | rq |   v_fwd   v_rgt |   dt |    count |  frames | crc | err");
        break;
    case DBG_MODE_BARO:
        wr_str(&w, "   press_pa |   temp |     alt_m |  alt_rel |  ground_pa |   dt |    count |  stale |  lost | err | cal");
        break;
    case DBG_MODE_FLOWCAL:
        wr_str(&w, "     sum_x      sum_y |   fx   fy | qual | range_m | rad_per_count hien tai");
        break;
    case DBG_MODE_VEL:
        wr_str(&w, "     wx      gx   tong |     wy      gy   tong |  vb_x   vb_y | range_m | qual |  bo | tuoi | tin |    gz |  ek_f  ek_r |   b_f    b_r");
        break;
    case DBG_MODE_POSHOLD:
        wr_str(&w, "  v_fwd  v_rgt |  t_fwd  t_rgt |   I_x   I_y |     dN     dE | giu |  tgt_r  tgt_p | che do");
        break;
    case DBG_MODE_ALTHOLD:
        wr_str(&w, "    alt    moc |  climb  c_tgt |     thr   I_thr |  valid | che do");
        break;
    case DBG_MODE_ANGLE:
        wr_str(&w, "  tgt_r  ang_r |  tgt_p  ang_p |   sp_r   gy_r |   sp_p   gy_p |   ch6 | che do");
        break;
    case DBG_MODE_PID:
        wr_str(&w, "   sp_r   gy_r |   sp_p   gy_p |   sp_y   gy_y |    o_r    o_p    o_y |    I_r    I_p    I_y | sat |     hz");
        break;
    case DBG_MODE_EST:
        wr_str(&w, "    roll   pitch     yaw |     d_yaw    do/ph |    alt   climb |  bias_z |   bgx   bgy   bgz | sig_deg | sig_m |  baro | range |  rej");
        break;
    case DBG_MODE_MOTOR:
        wr_str(&w, "armed |    m1    m2    m3    m4 |   out1   out2   out3   out4 |    frames | skip |  err | code");
        break;
    case DBG_MODE_RC:
        wr_str(&w, "   ch1   ch2   ch3   ch4   ch5   ch6   ch7   ch8 |    roll   pitch     yaw     thr |  LQ | rssi |  Hz |   frames | crc | err");
        break;
    case DBG_MODE_IMU2:
        wr_str(&w, "     gx      gy      gz |     ax      ay      az |   |a|/g |  temp |   dt |    hz |    count | err |  ovr | cal");
        break;
    case DBG_MODE_IMU2_RAW:
        wr_str(&w, "  raw_ax  raw_ay  raw_az |  raw_gx  raw_gy  raw_gz |   ax/g   ay/g   az/g");
        break;
    case DBG_MODE_STATUS:
        wr_str(&w, "che do       |  health     err  armblk |     imu_n  dt_us   ierr |   loop   lmax   slow |    rhz   rsk |      imu2 |     up   drop");
        break;
    case DBG_MODE_MAG:
#if MAG_SOURCE == MAG_SOURCE_SHUB
        wr_str(&w, "     mx     my     mz |    Btot |  raw_x  raw_y  raw_z |    hz |     count |  err  nack  busy | tt");
#elif MAG_SOURCE == MAG_SOURCE_I2C
        wr_str(&w, "     mx     my     mz |    Btot |  raw_x  raw_y  raw_z | dt_ms |     count |  err  lost stale |   yerr    mrej | tt");
#else
        wr_str(&w, "     mx     my     mz |    Btot |  raw_x  raw_y  raw_z | (MAG_SOURCE = NONE, khong co du lieu) | tt");
#endif
        break;
    case DBG_MODE_LOG:
        wr_str(&w, " ban ghi |  suc chua |  bo | file |  xa | loi | trang thai");
        break;
    case DBG_MODE_MAGCAL:
        wr_str(&w, "     mx     my     mz |    Btot |     mau | phu |  off_x  off_y  off_z |   sc_x   sc_y   sc_z | huong dan");
        break;
    case DBG_MODE_AXISCAL:
        wr_str(&w, "  raw_x  raw_y  raw_z |  truc  dau |  giu | buoc | huong dan");
        break;
    case DBG_MODE_IMU_CMP:
        wr_str(&w, "   gx_1   gx_2 |   gy_1   gy_2 |   gz_1   gz_2 |   sd_1   sd_2 |  |a|_1  |a|_2 |  hz_1  hz_2");
        break;
    default:
        return;
    }

    wr_eol(&w);
    (void)tx_push(line, w.len);
}

static void emit_imu(void)
{
    char line[DBG_LINE_MAX];
    wr_t w = { line, 0, sizeof(line) };

    const imu_data_t *imu = &g_fc.imu;

    wr_fix(&w, imu->gyro_dps.x,   2, 7);
    wr_fix(&w, imu->gyro_dps.y,   2, 8);
    wr_fix(&w, imu->gyro_dps.z,   2, 8);
    wr_str(&w, " |");
    wr_fix(&w, imu->accel_mps2.x, 2, 7);
    wr_fix(&w, imu->accel_mps2.y, 2, 8);
    wr_fix(&w, imu->accel_mps2.z, 2, 8);
    wr_str(&w, " |");
    wr_fix(&w, imu->temperature_c, 1, 6);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)imu->dt_us, 5);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)imu->sample_count, 9);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)imu->error_count, 4);
    wr_str(&w, " | ");

    if (imu->calibrated) {
        wr_str(&w, "OK");
    } else if (icm20602_get_state() == ICM_STATE_CALIBRATING) {
        wr_i32(&w, (int32_t)icm20602_calibration_progress(), 0);
        wr_ch(&w, '%');
    } else {
        wr_str(&w, "--");
    }

    wr_eol(&w);
    (void)tx_push(line, w.len);
}

static void emit_imu_raw(void)
{
    char line[DBG_LINE_MAX];
    wr_t w = { line, 0, sizeof(line) };

    const imu_data_t *imu = &g_fc.imu;

    wr_i32(&w, imu->accel_raw.x, 8);
    wr_i32(&w, imu->accel_raw.y, 8);
    wr_i32(&w, imu->accel_raw.z, 8);
    wr_str(&w, " |");
    wr_i32(&w, imu->gyro_raw.x, 8);
    wr_i32(&w, imu->gyro_raw.y, 8);
    wr_i32(&w, imu->gyro_raw.z, 8);
    wr_str(&w, " |");

    /*
     * Để yên trên bàn, mặt trên hướng lên: cột này phải ≈ -1.00.
     * Dấu âm là đúng vì hệ toạ độ thân là NED, trục Z hướng XUỐNG.
     * Nếu ra +1.00 thì đảo IMU_AXIS_SIGN_Z trong fc_config.h.
     */
    wr_fix(&w, imu->accel_mps2.z / FC_GRAVITY_MPS2, 2, 7);

    wr_eol(&w);
    (void)tx_push(line, w.len);
}

static void emit_imu_csv(void)
{
    char line[DBG_LINE_MAX];
    wr_t w = { line, 0, sizeof(line) };

    const imu_data_t *imu = &g_fc.imu;

    wr_i32(&w, (int32_t)imu->timestamp_us, 0);  wr_ch(&w, ',');
    wr_fix(&w, imu->gyro_dps.x,   3, 0);        wr_ch(&w, ',');
    wr_fix(&w, imu->gyro_dps.y,   3, 0);        wr_ch(&w, ',');
    wr_fix(&w, imu->gyro_dps.z,   3, 0);        wr_ch(&w, ',');
    wr_fix(&w, imu->accel_mps2.x, 3, 0);        wr_ch(&w, ',');
    wr_fix(&w, imu->accel_mps2.y, 3, 0);        wr_ch(&w, ',');
    wr_fix(&w, imu->accel_mps2.z, 3, 0);        wr_ch(&w, ',');
    wr_fix(&w, imu->temperature_c, 2, 0);

    wr_eol(&w);
    (void)tx_push(line, w.len);
}

static void emit_flow(void)
{
    char line[DBG_LINE_MAX];
    wr_t w = { line, 0, sizeof(line) };

    const flow_data_t *f = &g_fc.flow;

    wr_i32(&w, f->flow_x_raw, 7);
    wr_i32(&w, f->flow_y_raw, 8);
    wr_str(&w, " |");
    wr_i32(&w, f->flow_quality, 5);
    wr_str(&w, " |");
    wr_i32(&w, f->range_mm, 9);
    wr_str(&w, " |");
    wr_i32(&w, f->range_quality, 4);
    wr_str(&w, " |");
    wr_fix(&w, f->velocity_mps.x, 3, 8);
    wr_fix(&w, f->velocity_mps.y, 3, 8);
    wr_str(&w, " |");

    /* Khoang cach giua hai goi flow, tinh bang mili giay. */
    wr_i32(&w, (int32_t)(fc_elapsed_us(micros(), f->timestamp_us) / 1000u), 5);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)f->sample_count, 9);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)mtf01p_frames_ok(), 8);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)mtf01p_crc_errors(), 4);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)f->error_count, 4);

    if (!f->healthy) {
        wr_str(&w, "  [MAT TIN HIEU]");
    } else if (!f->range_valid) {
        wr_str(&w, "  [NGOAI TAM DO]");
    }

    wr_eol(&w);
    (void)tx_push(line, w.len);
}

/**
 * Do byte tho cua UART4 ra dang hex.
 *
 * Dung khi bo phan tich MSP khong ra du lieu: nhin byte that de biet cam bien
 * dang phat giao thuc gi. Khung MSP V2 bat dau bang 24 58 ('$' 'X').
 */
static void emit_flow_raw(void)
{
    char    line[DBG_LINE_MAX];
    wr_t    w = { line, 0, sizeof(line) };
    uint8_t raw[MTF01P_RAW_SNAPSHOT_LEN];

    const uint8_t n = mtf01p_peek_raw(raw, (uint8_t)sizeof(raw));

    wr_str(&w, "rx=");
    wr_i32(&w, (int32_t)mtf01p_bytes_received(), 8);
    wr_str(&w, " frm=");
    wr_i32(&w, (int32_t)mtf01p_frames_ok(), 6);
    wr_str(&w, " crc=");
    wr_i32(&w, (int32_t)mtf01p_crc_errors(), 4);
    wr_str(&w, " |");

    for (uint8_t i = 0; i < n; i++) {
        wr_ch(&w, ' ');
        wr_hex_byte(&w, raw[i]);
    }

    if (n == 0u || mtf01p_bytes_received() == 0u) {
        wr_str(&w, "  (khong nhan duoc byte nao)");
    }

    wr_eol(&w);
    (void)tx_push(line, w.len);
}

/**
 * BMP388: ap suat, nhiet do va hai kieu do cao.
 *
 * CACH DOC:
 *   press_pa   ap suat da bu, ngang muc nuoc bien la ~101325 Pa. Trong nha
 *              thuong thay 99000..102000 Pa tuy thoi tiet va do cao dia phuong.
 *   alt_m      do cao so voi muc nuoc bien chuan — dung de doi chieu voi ban
 *              do, KHONG dung cho vong giu do cao.
 *   alt_rel    do cao so voi moc lay luc hieu chuan. Day moi la so vong giu
 *              do cao dung. De yen tren ban thi phai dao quanh 0.
 *   stale      so lan hoi vong trung mau cu. Hoi vong 100 Hz tren ODR 50 Hz
 *              nen so nay tang ~50/giay la BINH THUONG, khong phai loi.
 *   cal        tien do hieu chuan moc mat dat, 0..100 %.
 *
 * KIEM CHUNG NHANH: nhac cam bien len 30 cm thi alt_rel phai giam khoang
 * 0,30 m (ap suat giam khi len cao nen do cao tang — dau duong la len).
 * Thoi ne vao cam bien se thay press_pa nhay manh: do la dau hieu chip song.
 */
static void emit_baro(void)
{
    char line[DBG_LINE_MAX];
    wr_t w = { line, 0, sizeof(line) };

    const baro_data_t *b = &g_fc.baro;

    wr_fix(&w, b->pressure_pa, 1, 11);
    wr_str(&w, " |");
    wr_fix(&w, b->temperature_c, 2, 7);
    wr_str(&w, " |");
    wr_fix(&w, b->altitude_m, 2, 10);
    wr_str(&w, " |");
    wr_fix(&w, b->altitude_rel_m, 2, 9);
    wr_str(&w, " |");
    wr_fix(&w, b->ground_pressure_pa, 1, 11);
    wr_str(&w, " |");

    /* Khoang cach giua hai mau baro, tinh bang mili giay. */
    wr_i32(&w, (int32_t)(fc_elapsed_us(micros(), b->timestamp_us) / 1000u), 5);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)b->sample_count, 9);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)bmp388_stale_reads(), 7);
    wr_str(&w, " |");
    /* Nhuong bus cho tu ke - KHONG phai loi, xem bmp388.c start_read(). */
    wr_i32(&w, (int32_t)bmp388_bus_lost(), 6);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)b->error_count, 4);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)bmp388_calibration_progress(), 4);

    switch (bmp388_get_state()) {
    case BMP_STATE_UNINIT:
        wr_str(&w, "  [CHUA KHOI TAO]");
        break;
    case BMP_STATE_ERROR:
        wr_str(&w, "  [LOI I2C - kiem tra dia chi 0x77 va dien tro keo len]");
        break;
    case BMP_STATE_CALIBRATING:
        wr_str(&w, "  [DANG LAY MOC MAT DAT - de yen]");
        break;
    default:
        if (!b->healthy) {
            wr_str(&w, "  [MAT TIN HIEU]");
        }
        break;
    }

    wr_eol(&w);
    (void)tx_push(line, w.len);
}

/**
 * ELRS / CRSF: tam kenh dau, bon can da chuan hoa, chat luong duong truyen.
 *
 * CACH DOC:
 *   ch1..ch8   gia tri tho CRSF, 172 = het mot dau, 992 = giua, 1811 = het
 *              dau kia. Gat tung can mot de biet kenh nao ung voi can nao.
 *   roll/pitch/yaw  -1..+1 sau vung chet; thr 0..1 khong co vung chet.
 *   LQ         chat luong duong len, 100 la hoan hao. Duoi 70 la dang xa.
 *   rssi       dBm, cang gan 0 cang manh (-40 rat tot, -110 sap mat song).
 *   Hz         tan so khung RC thuc te — dung de biet ELRS dang chay che do
 *              nao (50/150/250/500/1000 Hz).
 *   crc        khung hong. Tang deu nghia la sai baud hoac nhieu day.
 */
static void emit_rc(void)
{
    char line[DBG_LINE_MAX];
    wr_t w = { line, 0, sizeof(line) };

    const rc_data_t *rc = &g_fc.rc;

    for (int i = 0; i < 8; i++) {
        wr_i32(&w, rc->channel_raw[i], 6);
    }
    wr_str(&w, " |");

    wr_fix(&w, rc->roll, 3, 8);
    wr_fix(&w, rc->pitch, 3, 8);
    wr_fix(&w, rc->yaw, 3, 8);
    wr_fix(&w, rc->throttle, 3, 8);
    wr_str(&w, " |");

    wr_i32(&w, rc->link_quality, 4);
    wr_str(&w, " |");
    wr_i32(&w, rc->rssi_dbm, 5);
    wr_str(&w, " |");

    /* Tan so khung suy ra tu chu ky giua hai khung RC gan nhat. */
    const uint32_t interval_us = crsf_frame_interval_us();
    wr_i32(&w, (interval_us > 0u) ? (int32_t)(1000000u / interval_us) : 0, 4);
    wr_str(&w, " |");

    wr_i32(&w, (int32_t)crsf_rc_frames(), 9);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)crsf_crc_errors(), 4);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)rc->error_count, 4);

    if (rc->failsafe) {
        wr_str(&w, (crsf_bytes_received() == 0u) ? "  [CHUA CO BYTE NAO]"
                                                 : "  [FAILSAFE]");
    }

    wr_eol(&w);
    (void)tx_push(line, w.len);
}

/**
 * Do byte tho cua USART2 ra dang hex.
 *
 * Dung khi khong ra kenh nao: khung CRSF luon bat dau bang C8 roi toi byte do
 * dai (18 voi khung RC 22 byte). Neu thay byte chay nhung khong bao gio co
 * C8, kha nang cao la sai baud — vai ban ELRS doi cu chay 400000 thay vi
 * 420000. Neu khong thay byte nao thi kiem tra day: TX cua may thu phai vao
 * PA3, khong phai PA2.
 */
static void emit_rc_raw(void)
{
    char    line[DBG_LINE_MAX];
    wr_t    w = { line, 0, sizeof(line) };
    uint8_t raw[CRSF_RAW_SNAPSHOT_LEN];

    const uint8_t n = crsf_peek_raw(raw, (uint8_t)sizeof(raw));

    wr_str(&w, "rx=");
    wr_i32(&w, (int32_t)crsf_bytes_received(), 8);
    wr_str(&w, " frm=");
    wr_i32(&w, (int32_t)crsf_frames_ok(), 6);
    wr_str(&w, " crc=");
    wr_i32(&w, (int32_t)crsf_crc_errors(), 4);
    wr_str(&w, " |");

    for (uint8_t i = 0; i < n; i++) {
        wr_ch(&w, ' ');
        wr_hex_byte(&w, raw[i]);
    }

    if (n == 0u || crsf_bytes_received() == 0u) {
        wr_str(&w, "  (khong nhan duoc byte nao)");
    }

    wr_eol(&w);
    (void)tx_push(line, w.len);
}

/**
 * May trang thai arm: cong tac, dieu kien chan, so lan arm/disarm.
 *
 * CACH DOC:
 *   mode   trang thai bay tong the (INIT/CALIB/DISARMED/ARMED/FAILSAFE/FAULT)
 *   arm    may trang thai cua bo giam sat: LOCKED / SAFE / ARMED
 *   sw     cong tac arm sau khi loc nhieu
 *   ch5    gia tri tho cua kenh arm — doi chieu voi ARM_SWITCH_*_THRESHOLD
 *   blk    bitmask fc_arm_block_t, 0x0000 la khong con gi chan
 *   last   nguyen nhan lan disarm gan nhat
 *   chan   ly do cap thiet nhat, dang chu — day la thu can doc dau tien
 *
 * LOCKED khong phai loi. Do la trang thai khoi dong, va la noi moi tinh
 * huong bat thuong roi ve. Gat cong tac arm ve OFF mot lan de nha khoa.
 */
static void emit_arm(void)
{
    char line[DBG_LINE_MAX];
    wr_t w = { line, 0, sizeof(line) };

    wr_str(&w, "mode=");
    wr_str(&w, fc_mode_name(g_fc.mode));
    wr_str(&w, " arm=");
    wr_str(&w, arming_state_name(arming_get_state()));
    wr_str(&w, " sw=");
    wr_str(&w, arming_switch_on() ? "ON " : "OFF");
    wr_str(&w, " ch5=");
    wr_i32(&w, g_fc.rc.channel_raw[ARM_SWITCH_CHANNEL], 5);
    wr_str(&w, " thr=");
    wr_fix(&w, g_fc.rc.throttle, 3, 6);
    wr_str(&w, " blk=");
    wr_hex(&w, g_fc.sys.arm_block_flags, 4);
    wr_str(&w, " arms=");
    wr_i32(&w, (int32_t)arming_arm_count(), 3);
    wr_str(&w, " disarms=");
    wr_i32(&w, (int32_t)arming_disarm_count(), 3);
    wr_str(&w, " last=");
    wr_str(&w, arming_cause_name(arming_last_disarm_cause()));

    /* Ly do dat cuoi cung vi do dai thay doi theo tung truong hop. */
    wr_str(&w, " | chan: ");
    wr_str(&w, arming_block_reason());

    wr_eol(&w);
    (void)tx_push(line, w.len);
}

/**
 * Gia tri DShot dang thuc su phat ra 4 ESC.
 *
 * CACH DOC:
 *   armed  1 khi ca g_fc.motor.armed lan mode deu la ARMED
 *   m1..m4 gia tri DShot tho: 0 = dung motor, 48 = ga thap nhat (motor QUAY),
 *          2047 = ga toi da. Nhay tu 0 len 48 ngay khi arm la DUNG.
 *   out    g_fc.motor.output_norm[] 0..1 do khau tron ghi vao. Chua co khau
 *          tron nen hien tai luon bang 0.
 *   skip   so lan bo nhip vi khung truoc chua phat xong. Phai dung yen o 0.
 *   err    loi DMA. Bat ky so nao khac 0 deu la van de phan cung.
 *
 * frames phai tang deu ~1000/giay. Dung yen nghia la dshot_update() khong
 * duoc goi hoac DMA da ket.
 */
static void emit_motor(void)
{
    char line[DBG_LINE_MAX];
    wr_t w = { line, 0, sizeof(line) };

    const motor_data_t *m = &g_fc.motor;
    const bool armed = m->armed && (g_fc.mode == FC_MODE_ARMED);

    wr_i32(&w, armed ? 1 : 0, 5);
    wr_str(&w, " |");
    for (int i = 0; i < FC_MOTOR_COUNT; i++) {
        wr_i32(&w, m->throttle[i], 6);
    }
    wr_str(&w, " |");
    for (int i = 0; i < FC_MOTOR_COUNT; i++) {
        wr_fix(&w, m->output_norm[i], 3, 7);
    }
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)dshot_frames_sent(), 10);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)dshot_skipped(), 5);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)dshot_dma_errors(), 5);
    wr_str(&w, " | ");
    wr_hex(&w, dshot_dma_errcode(), 2);

    if (armed) {
        wr_str(&w, "  [MOTOR DANG QUAY]");
    } else if (dshot_motor_test_active() >= 0) {
        wr_str(&w, "  [QUAY THU MOTOR ");
        wr_i32(&w, dshot_motor_test_active() + 1, 0);
        wr_str(&w, "]");
    }

    wr_eol(&w);
    (void)tx_push(line, w.len);
}

/**
 * Ket qua bo loc hop nhat EKF.
 *
 * CACH DOC:
 *   roll/pitch/yaw  goc uoc luong, DO. yaw la huong so voi luc khoi dong va
 *                   TROI dan vai do moi phut — khong co la ban thi khong the
 *                   khac duoc, day khong phai loi.
 *   alt / climb     do cao so voi mat dat luc hieu chuan (m) va toc do len (m/s)
 *   bias_z          bias gia toc thang dung bo loc do ra (m/s2). Hoi tu ve
 *                   mot gia tri nho va DUNG YEN la dau hieu tot.
 *   bgx/bgy/bgz     bias gyro bo loc do ra (do/giay). Phai rat nho va on dinh.
 *   sig_deg         sai so goc con lai. Tut ve duoi 1 do trong vai giay dau.
 *   sig_m           sai so do cao con lai (m).
 *   baro/range      so lan da nap mau tuyet doi vao bo loc.
 *   rej             so lan bo qua laser vi nghieng qua hoac ngoai tam.
 *
 * PHEP THU NHANH: de yen may bay -> roll/pitch quanh 0, climb quanh 0,
 * sig_deg tut xuong duoi 1. Nghieng 30 do sang phai -> roll ~ +30.
 */
/* ==========================================================================
 * Đo trôi yaw so với một mốc
 *
 * VÌ SAO PHẢI TÍCH LUỸ CHỨ KHÔNG TRỪ HAI GÓC
 *
 *   Yaw do atan2 sinh ra nên luôn nằm trong ±180°. Trừ thẳng góc hiện tại cho
 *   góc mốc thì trôi quá nửa vòng sẽ đọc ra số ÂM nhỏ dần thay vì số dương
 *   lớn dần — đúng lúc con số bắt đầu có ý nghĩa thì nó lại nói dối.
 *
 *   Nên ở đây cộng dồn từng bước nhỏ, mỗi bước tự gỡ gói qua mốc ±180°. Trôi
 *   ba vòng thì đọc ra 1080°, không phải 0°.
 *
 * NẰM Ở TẦNG CONSOLE, KHÔNG PHẢI TẦNG ĐIỀU KHIỂN
 *
 *   Đây là dụng cụ đo. Nó không được thêm một dòng nào vào vòng 1 kHz hay
 *   4 kHz — chạy ở nhịp console 50 Hz là quá đủ, vì trôi là hiện tượng chậm.
 *
 *   50 Hz vẫn gỡ gói đúng cho tới 9000 °/s, xa hơn mọi thứ máy bay này làm được.
 * ========================================================================== */

static float    s_yaw_prev_rad;
static float    s_yaw_drift_rad;   /* cộng dồn, KHÔNG gói về ±180 */
static uint32_t s_yaw_ref_ms;
static bool     s_yaw_ref_set;

/*
 * Mốc phụ cho phép đo TỐC ĐỘ, tách khỏi mốc tổng.
 *
 * VÌ SAO KHÔNG LẤY TỔNG CHIA THỜI GIAN: một cú nhảy một lần — bộ lọc đang hội
 * tụ, va chạm vào giá, hay đơn giản là mấy dòng cũ còn trong đệm phát — sẽ
 * nằm mãi trong tử số và bóp méo con số cho tới hết phiên đo. Đã gặp thật:
 * tổng phẳng lì suốt hai phút mà cột tốc độ vẫn đọc −8,8 độ/phút chỉ vì 18 độ
 * nhảy trong bốn giây đầu.
 *
 * Cửa sổ trượt chỉ nhìn quãng gần đây nên nó QUÊN được, và đó chính là điều
 * cần: câu hỏi là "bây giờ đang trôi bao nhanh", không phải "từ đầu tới giờ
 * đã đi bao xa".
 */
#define YAW_RATE_WINDOW_MS 20000u

static float    s_yaw_win_drift_rad;   /* giá trị cộng dồn tại đầu cửa sổ */
static uint32_t s_yaw_win_ms;
static float    s_yaw_rate_dpm;        /* kết quả của cửa sổ vừa đóng     */

static void yaw_drift_track(uint32_t now_ms)
{
    const float yaw = g_fc.est.attitude_rad.yaw;

    if (!s_yaw_ref_set) {
        s_yaw_prev_rad      = yaw;
        s_yaw_drift_rad     = 0.0f;
        s_yaw_ref_ms        = now_ms;
        s_yaw_win_drift_rad = 0.0f;
        s_yaw_win_ms        = now_ms;
        s_yaw_rate_dpm      = 0.0f;
        s_yaw_ref_set       = true;
        return;
    }

    float d = yaw - s_yaw_prev_rad;

    while (d >  FC_PI) { d -= 2.0f * FC_PI; }
    while (d < -FC_PI) { d += 2.0f * FC_PI; }

    s_yaw_prev_rad   = yaw;
    s_yaw_drift_rad += d;

    /* Đóng cửa sổ: chốt tốc độ rồi mở cửa sổ mới từ đây. */
    const uint32_t win_ms = now_ms - s_yaw_win_ms;

    if (win_ms >= YAW_RATE_WINDOW_MS) {
        s_yaw_rate_dpm = (s_yaw_drift_rad - s_yaw_win_drift_rad)
                         * FC_RAD_TO_DEG * 60000.0f / (float)win_ms;

        s_yaw_win_drift_rad = s_yaw_drift_rad;
        s_yaw_win_ms        = now_ms;
    }
}

void dbg_console_yaw_zero(void)
{
    s_yaw_ref_set = false;   /* nhịp kế tiếp sẽ chốt lại mốc */
}

float dbg_console_yaw_drift_deg(void)
{
    return s_yaw_drift_rad * FC_RAD_TO_DEG;
}

uint32_t dbg_console_yaw_elapsed_ms(void)
{
    return s_yaw_ref_set ? (uint32_t)(HAL_GetTick() - s_yaw_ref_ms) : 0u;
}

/**
 * Tốc độ trôi trên cửa sổ 20 giây gần nhất, độ/phút.
 *
 * Trả 0 cho tới khi cửa sổ đầu tiên đóng — thà không có số còn hơn có một số
 * sai. Sau đó nó cập nhật mỗi 20 giây.
 */
float dbg_console_yaw_drift_dpm(void)
{
    return s_yaw_rate_dpm;
}

static void emit_est(void)
{
    char line[DBG_LINE_MAX];
    wr_t w = { line, 0, sizeof(line) };

    const estimator_data_t *e = &g_fc.est;
    const vec3f_t bg = ekf_attitude_gyro_bias_dps();

    /*
     * Bề rộng 8 chứ không phải 7.
     *
     * Roll và yaw chạy tới ±180, nên "-179.99" chiếm đúng 7 ký tự và KHÔNG
     * còn chỗ cho khoảng trắng phân cách — hai cột dính liền thành
     * "-0.31-141.85". Đã gặp thật khi phân tích log: mọi công cụ tách cột
     * theo khoảng trắng đều đọc sai từ chỗ đó trở đi.
     *
     * d_yaw còn rộng hơn vì nó cộng dồn không giới hạn: trôi ba vòng là
     * "-1080.00", chín ký tự.
     */
    wr_fix(&w, e->attitude_rad.roll  * FC_RAD_TO_DEG, 2, 8);
    wr_fix(&w, e->attitude_rad.pitch * FC_RAD_TO_DEG, 2, 8);
    wr_fix(&w, e->attitude_rad.yaw   * FC_RAD_TO_DEG, 2, 8);
    wr_str(&w, " |");

    /* Trôi yaw so với mốc, và quy ra độ/phút. Xoá mốc bằng lệnh CLI `yawzero`. */
    wr_fix(&w, dbg_console_yaw_drift_deg(), 2, 10);
    wr_fix(&w, dbg_console_yaw_drift_dpm(), 2, 9);
    wr_str(&w, " |");
    wr_fix(&w, e->altitude_m, 2, 7);
    wr_fix(&w, e->climb_rate_mps, 2, 8);
    wr_str(&w, " |");
    wr_fix(&w, ekf_altitude_accel_bias(), 3, 8);
    wr_str(&w, " |");
    wr_fix(&w, bg.x, 2, 6);
    wr_fix(&w, bg.y, 2, 6);
    wr_fix(&w, bg.z, 2, 6);
    wr_str(&w, " |");
    wr_fix(&w, ekf_attitude_uncertainty_deg(), 3, 8);
    wr_str(&w, " |");
    wr_fix(&w, ekf_altitude_uncertainty_m(), 2, 6);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)estimator_baro_updates(), 6);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)estimator_range_updates(), 6);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)estimator_range_rejected(), 5);

    if (!e->attitude_valid) {
        wr_str(&w, "  [CHUA DUNG DUOC GOC BAN DAU]");
    } else if (!e->altitude_valid) {
        wr_str(&w, "  [CHO BARO]");
    }

    wr_eol(&w);
    (void)tx_push(line, w.len);
}

/**
 * Chinh PID vong toc do goc.
 *
 * CACH DOC — cot quan trong nhat la cap sp/gy cua tung truc:
 *   sp_x   toc do goc NGUOI LAI YEU CAU (do/giay)
 *   gy_x   toc do goc THUC TE do duoc
 *   Chinh PID chinh la lam cho gy bam sat sp. gy tre so voi sp -> tang P.
 *   gy vot qua roi dao quanh sp -> P qua cao hoac thieu D.
 *
 *   o_x    dau ra PID (-1..1) di vao khau tron
 *   I_x    tich phan. Phai nho va on dinh. Bo len sat I_LIMIT roi dung im
 *          nghia la co lech he thong (khung cong, motor lech) hoac ki qua cao.
 *   sat    khau tron dang bao hoa. Bang 1 lien tuc nghia la lenh vuot qua
 *          kha nang cua may bay — tich phan bi dung lai trong luc do.
 *   hz     tan so thuc te cua vong. PHAI bam sat FC_LOOP_RATE_HZ (4000).
 *          Tut xuong nghia la vong lap chinh khong theo kip.
 */
/**
 * Vong goc (che do ANGLE).
 *
 * CACH DOC:
 *   tgt_r/tgt_p  goc MUC TIEU do can dieu khien yeu cau (do)
 *   ang_r/ang_p  goc THUC TE do EKF uoc luong (do)
 *   Chinh vong ngoai la lam ang bam sat tgt. Buong can thi ca hai ve 0.
 *
 *   sp_r/sp_p    toc do quay ma vong ngoai GIAO XUONG cho vong trong (do/giay)
 *   gy_r/gy_p    toc do quay thuc te. Day la cho noi giua hai vong: sp la dau
 *                ra cua vong ngoai va dong thoi la dau vao cua vong trong.
 *
 *   ch6          gia tri tho kenh chon che do
 *   che do       ACRO hoac ANGLE. Hien [DU PHONG] khi cong tac doi ANGLE
 *                nhung EKF chua co goc tin cay nen phai lui ve ACRO.
 *
 * CHINH: tang ANGLE_PID_KP toi khi may bay ve ngang dut khoat ma khong vot
 * qua roi lac. Vot lo va lac nghia la qua cao — vong ngoai doi vong trong lam
 * nhieu hon kha nang cua no.
 */
/**
 * Hieu chuan FLOW_RAD_PER_COUNT.
 *
 * CACH DUNG:
 *   1. Giu may bay o do cao CO DINH DA BIET (vd 1.00 m), doc cot range_m de
 *      xac nhan. Doc sum_x va sum_y, ghi lai.
 *   2. Re NGANG dung mot quang duong da biet (vd 1.00 m sang phai), giu nguyen
 *      do cao va KHONG XOAY may bay.
 *   3. Doc lai sum_y. Lay hieu so.
 *   4. FLOW_RAD_PER_COUNT = (quang_duong / do_cao) / hieu_so_dem
 *
 * Vi du: cao 1.00 m, re 1.00 m, sum_y tang 9800 dem
 *        -> (1.00 / 1.00) / 9800 = 0.000102
 *
 * Lam lai vai lan roi lay trung binh. Re cang cham cang chinh xac, vi cham
 * thi phan dich anh do QUAY cang nho so voi phan do TINH TIEN.
 */
static void emit_flowcal(void)
{
    static int32_t sum_x = 0, sum_y = 0;
    static uint32_t seen = 0;

    char line[DBG_LINE_MAX];
    wr_t w = { line, 0, sizeof(line) };

    /* Chi cong don moi mau dung mot lan. */
    if (g_fc.flow.sample_count != seen) {
        seen   = g_fc.flow.sample_count;
        sum_x += g_fc.flow.flow_x_raw;
        sum_y += g_fc.flow.flow_y_raw;
    }

    wr_i32(&w, sum_x, 10);
    wr_i32(&w, sum_y, 11);
    wr_str(&w, " |");
    wr_i32(&w, g_fc.flow.flow_x_raw, 5);
    wr_i32(&w, g_fc.flow.flow_y_raw, 5);
    wr_str(&w, " |");
    wr_i32(&w, g_fc.flow.flow_quality, 5);
    wr_str(&w, " |");
    wr_fix(&w, (float)g_fc.flow.range_mm * 0.001f, 3, 8);
    wr_str(&w, " | ");
    wr_fix(&w, FLOW_RAD_PER_COUNT, 6, 0);

    if (!g_fc.flow.range_valid) {
        wr_str(&w, "  [LASER NGOAI TAM]");
    }

    wr_eol(&w);
    (void)tx_push(line, w.len);
}

/**
 * Van toc ngang tu optical flow, DA BU QUAY.
 *
 * CACH DOC:
 *   vb_x/vb_y  van toc he THAN do duoc tu mau flow gan nhat (m/s).
 *              vb_x duong = bay TOI, vb_y duong = bay SANG PHAI.
 *   v_n/v_e    van toc he NED sau khi hop nhat voi gia toc ke.
 *   sig        sai so con lai cua uoc luong (m/s).
 *   nhan/bo    so mau flow da dung / da tu choi.
 *
 * PHEP KIEM QUAN TRONG NHAT: giu may bay o do cao co dinh roi NGHIENG TAI CHO
 * (khong di chuyen). vb_x va vb_y phai o gan 0. Neu chung nhay theo moi cu
 * nghieng thi phan bu quay dang sai — va bo giu vi tri se tu kich.
 */
static void emit_vel(void)
{
    char line[DBG_LINE_MAX];
    wr_t w = { line, 0, sizeof(line) };

    const vec3f_t vb = ekf_velocity_body_measured();

    float wx, wy, gx, gy;
    ekf_velocity_debug_rates(&wx, &wy, &gx, &gy);

    /* Cot 'tong' la wx+gx. Nghieng tai cho thi no phai gan 0. */
    wr_fix(&w, wx, 2, 7);
    wr_fix(&w, gx, 2, 8);
    wr_fix(&w, wx + gx, 2, 7);
    wr_str(&w, " |");
    wr_fix(&w, wy, 2, 7);
    wr_fix(&w, gy, 2, 8);
    wr_fix(&w, wy + gy, 2, 7);
    wr_str(&w, " |");
    wr_fix(&w, vb.x, 2, 6);
    wr_fix(&w, vb.y, 2, 7);
    wr_str(&w, " |");
    wr_fix(&w, (float)g_fc.flow.range_mm * 0.001f, 3, 8);
    wr_str(&w, " |");
    wr_i32(&w, g_fc.flow.flow_quality, 5);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)ekf_velocity_rejected(), 4);

    /* Tuoi cua mau flow gan nhat, va co tin cay da het han chua. */
    {
        const uint32_t age = ekf_velocity_age_ms();

        wr_str(&w, " |");
        if (age == 0xFFFFFFFFu) {
            wr_str_pad(&w, "  --", 6);
        } else {
            wr_i32(&w, (int32_t)age, 6);
        }
        wr_str(&w, " |");
        wr_str_pad(&w, ekf_velocity_is_valid() ? " co" : " HET", 5);
    }

    /* Toc do yaw (rad/s), de khop vb_y voi gz khi kiem bu canh tay don. */
    wr_str(&w, " |");
    wr_fix(&w, g_fc.imu.gyro_dps.z * FC_DEG_TO_RAD, 2, 6);

    /*
     * Van toc SAU EKF va bias gia toc, cung quy ve he THAN theo yaw hien tai.
     * So voi vb_x/vb_y (flow tho) de biet sai so sinh ra o phia gia toc ke:
     * quay yaw tai cho ma ek_* lech con vb_* khong -> loi o EKF.
     */
    {
        const float yaw = g_fc.est.attitude_rad.yaw;
        const float cy  = cosf(yaw);
        const float sy  = sinf(yaw);
        const float vn  = g_fc.est.velocity_mps.x;
        const float ve  = g_fc.est.velocity_mps.y;
        float bn, be;

        ekf_velocity_bias_ne(&bn, &be);

        wr_str(&w, " |");
        wr_fix(&w,  vn * cy + ve * sy, 2, 6);
        wr_fix(&w, -vn * sy + ve * cy, 2, 6);
        wr_str(&w, " |");
        wr_fix(&w,  bn * cy + be * sy, 3, 7);
        wr_fix(&w, -bn * sy + be * cy, 3, 7);
    }
    wr_eol(&w);
    (void)tx_push(line, w.len);
}

/**
 * Vong giu van toc (che do POSHOLD).
 *
 * CACH DOC:
 *   v_fwd/v_rgt  van toc THUC te he than (m/s). Duong = dang bay toi / sang phai.
 *   t_fwd/t_rgt  van toc MONG MUON do can dieu khien dat. Can o giua thi ca hai
 *                bang 0, tuc dang ghi may bay dung yen.
 *   I_x/I_y      khau tich phan (do). Co gio deu thi no bo len mot gia tri roi
 *                dung yen — do la no dang giu do nghieng chong gio.
 *   tgt_r/tgt_p  goc nghieng ma vong nay giao xuong vong goc (do).
 *   che do       phai la POSHOLD. Hien [DU PHONG] khi mat flow va da lui ve ANGLE.
 *
 * CHINH: v bam duoc t la dat. v vot qua roi lac -> giam POSHOLD_VEL_KP.
 * v ve gan 0 nhung van troi cham deu -> tang POSHOLD_VEL_KI.
 */
static void emit_althold(void)
{
    char line[DBG_LINE_MAX];
    wr_t w = { line, 0, sizeof(line) };

    wr_fix(&w, g_fc.est.altitude_m,        2, 7);
    wr_fix(&w, ctrl_althold_target_m(),    2, 7);
    wr_str(&w, " |");
    wr_fix(&w, g_fc.est.climb_rate_mps,    2, 7);
    wr_fix(&w, ctrl_althold_climb_target(),2, 7);
    wr_str(&w, " |");
    wr_fix(&w, g_fc.ctrl.throttle_cmd,     3, 8);
    wr_fix(&w, ctrl_althold_integral(),    3, 8);
    wr_str(&w, " |");
    wr_str_pad(&w, g_fc.est.altitude_valid ? "co" : "KHONG", 7);
    wr_str(&w, " | ");
    wr_str(&w, fc_flight_mode_name(ctrl_angle_active_mode()));

    if (ctrl_angle_fallback()) {
        wr_str(&w, " [DU PHONG]");
    }

    wr_eol(&w);
    (void)tx_push(line, w.len);
}

static void emit_poshold(void)
{
    char line[DBG_LINE_MAX];
    wr_t w = { line, 0, sizeof(line) };

    const vec3f_t v = ctrl_poshold_velocity_body();
    const vec3f_t t = ctrl_poshold_target_body();
    const vec3f_t i = ctrl_poshold_integral_deg();

    wr_fix(&w, v.x, 2, 7);
    wr_fix(&w, v.y, 2, 7);
    wr_str(&w, " |");
    wr_fix(&w, t.x, 2, 7);
    wr_fix(&w, t.y, 2, 7);
    wr_str(&w, " |");
    wr_fix(&w, i.x, 2, 6);
    wr_fix(&w, i.y, 2, 6);
    wr_str(&w, " |");

    /* Sai số vị trí: còn cách mốc đang giữ bao xa, theo hệ NED. */
    {
        const vec3f_t tgt = ctrl_poshold_target_ned();

        wr_fix(&w, tgt.x - g_fc.est.position_m.x, 2, 7);
        wr_fix(&w, tgt.y - g_fc.est.position_m.y, 2, 7);
        wr_str(&w, " |");
        wr_str_pad(&w, ctrl_poshold_position_locked() ? " co" : " --", 4);
    }
    wr_str(&w, " |");
    wr_fix(&w, g_fc.ctrl.setpoint_angle_rad.roll * FC_RAD_TO_DEG, 2, 7);
    wr_fix(&w, g_fc.ctrl.setpoint_angle_rad.pitch * FC_RAD_TO_DEG, 2, 7);
    wr_str(&w, " | ");
    wr_str(&w, fc_flight_mode_name(ctrl_angle_active_mode()));

    if (ctrl_angle_fallback()) {
        wr_str(&w, " [DU PHONG - mat flow]");
    }

    wr_eol(&w);
    (void)tx_push(line, w.len);
}

static void emit_angle(void)
{
    char line[DBG_LINE_MAX];
    wr_t w = { line, 0, sizeof(line) };

    const control_data_t *c = &g_fc.ctrl;
    const euler_t *a = &g_fc.est.attitude_rad;

    wr_fix(&w, c->setpoint_angle_rad.roll * FC_RAD_TO_DEG, 1, 7);
    wr_fix(&w, a->roll * FC_RAD_TO_DEG, 1, 7);
    wr_str(&w, " |");
    wr_fix(&w, c->setpoint_angle_rad.pitch * FC_RAD_TO_DEG, 1, 7);
    wr_fix(&w, a->pitch * FC_RAD_TO_DEG, 1, 7);
    wr_str(&w, " |");

    wr_fix(&w, c->setpoint_rate_dps.x, 1, 7);
    wr_fix(&w, g_fc.imu.gyro_filtered_dps.x, 1, 7);
    wr_str(&w, " |");
    wr_fix(&w, c->setpoint_rate_dps.y, 1, 7);
    wr_fix(&w, g_fc.imu.gyro_filtered_dps.y, 1, 7);
    wr_str(&w, " |");

#if RC_MODE_CHANNEL >= 0
    wr_i32(&w, g_fc.rc.channel_raw[RC_MODE_CHANNEL], 6);
#else
    wr_i32(&w, -1, 6);
#endif
    wr_str(&w, " | ");
    wr_str(&w, fc_flight_mode_name(ctrl_angle_active_mode()));

    if (ctrl_angle_fallback()) {
        wr_str(&w, " [DU PHONG - chua co goc]");
    }

    wr_eol(&w);
    (void)tx_push(line, w.len);
}

static void emit_pid(void)
{
    char line[DBG_LINE_MAX];
    wr_t w = { line, 0, sizeof(line) };

    const control_data_t *c = &g_fc.ctrl;
    const vec3f_t g = g_fc.imu.gyro_filtered_dps;

    wr_fix(&w, c->setpoint_rate_dps.x, 1, 7);
    wr_fix(&w, g.x, 1, 7);
    wr_str(&w, " |");
    wr_fix(&w, c->setpoint_rate_dps.y, 1, 7);
    wr_fix(&w, g.y, 1, 7);
    wr_str(&w, " |");
    wr_fix(&w, c->setpoint_rate_dps.z, 1, 7);
    wr_fix(&w, g.z, 1, 7);
    wr_str(&w, " |");

    wr_fix(&w, c->pid_output.x, 3, 7);
    wr_fix(&w, c->pid_output.y, 3, 7);
    wr_fix(&w, c->pid_output.z, 3, 7);
    wr_str(&w, " |");

    wr_fix(&w, c->rate_pid[AXIS_ROLL].integral, 3, 7);
    wr_fix(&w, c->rate_pid[AXIS_PITCH].integral, 3, 7);
    wr_fix(&w, c->rate_pid[AXIS_YAW].integral, 3, 7);
    wr_str(&w, " |");

    wr_i32(&w, mixer_saturated() ? 1 : 0, 4);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)ctrl_rate_hz(), 7);

    if (ctrl_rate_skips() > 0u) {
        wr_str(&w, "  [BO NHIP: ");
        wr_i32(&w, (int32_t)ctrl_rate_skips(), 0);
        wr_str(&w, "]");
    }

    wr_eol(&w);
    (void)tx_push(line, w.len);
}

/**
 * LSM6DSV tren SPI3 - IMU phu. Cong cu kiem tra GIAI DOAN 1.
 *
 * CACH DOC:
 *   gx..az  don vi vat ly, da tru bias va xoay truc theo IMU2_AXIS_*
 *   |a|/g   DO LON gia toc chia cho g. De yen tren ban PHAI ~ 1.00.
 *           Lech qua 2% nghia la he so LSM_ACCEL_SCALE sai - xem ghi chu
 *           trong lsm6dsv.c ve hai hang so chua duoc kiem chung.
 *   hz      tan so lay mau THUC TE. Phai bam sat IMU2_ODR_HZ (1920).
 *           Ra ~960 nghia la ma ODR sai.
 *   ovr     so mau bi bo vi DMA truoc chua xong. PHAI dung yen o 0.
 *   err     loi giao tiep. Khac 0 la van de phan cung hoac day noi.
 *
 * Sau khi mode nay dat, nho kiem lai DBG_MODE_PID: cot hz VAN phai ~4000.
 */
static void emit_imu2(void)
{
    char line[DBG_LINE_MAX];
    wr_t w = { line, 0, sizeof(line) };

    const imu_data_t *imu = &g_fc.imu2;

    wr_fix(&w, imu->gyro_dps.x,   2, 7);
    wr_fix(&w, imu->gyro_dps.y,   2, 8);
    wr_fix(&w, imu->gyro_dps.z,   2, 8);
    wr_str(&w, " |");
    wr_fix(&w, imu->accel_mps2.x, 2, 7);
    wr_fix(&w, imu->accel_mps2.y, 2, 8);
    wr_fix(&w, imu->accel_mps2.z, 2, 8);
    wr_str(&w, " |");
    wr_fix(&w, vec3f_norm(imu->accel_mps2) / FC_GRAVITY_MPS2, 3, 8);
    wr_str(&w, " |");
    wr_fix(&w, imu->temperature_c, 1, 6);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)imu->dt_us, 5);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)lsm6dsv_sample_rate_hz(), 6);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)imu->sample_count, 9);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)imu->error_count, 4);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)lsm6dsv_overruns(), 4);
    wr_str(&w, " | ");

    if (imu->calibrated) {
        wr_str(&w, "OK");
    } else if (lsm6dsv_get_state() == LSM_STATE_CALIBRATING) {
        wr_i32(&w, (int32_t)lsm6dsv_calibration_progress(), 0);
        wr_ch(&w, '%');
    } else {
        wr_str(&w, "--");
    }

    /*
     * Chua co mau nao ve: in ba so tach bach ba tang co the hong.
     *   edges = 0            -> EXTI tren PD7 khong he kich.
     *   edges > 0, count = 0 -> EXTI kich nhung driver chan hoac DMA hong.
     *   st bit GDA/XLDA = 0  -> CHIP KHONG LAY MAU (loi ODR/che do).
     *   pd7 = 1 dai dang     -> INT1 bi CHOT cao, thieu DRDY_PULSED.
     */
    if (g_fc.imu2.sample_count == 0u) {
        wr_str(&w, "  [CHUA CO MAU edges=");
        wr_i32(&w, (int32_t)lsm6dsv_drdy_edges(), 0);
        wr_str(&w, " st=0x");
        wr_hex(&w, lsm6dsv_diag_status(), 2);
        wr_str(&w, " pd7=");
        wr_i32(&w, lsm6dsv_diag_int_level() ? 1 : 0, 0);
        wr_ch(&w, 93);
    }

    if (lsm6dsv_get_state() == LSM_STATE_ERROR) {
        wr_str(&w, "  [LOI - WHO_AM_I 0x");
        wr_hex(&w, lsm6dsv_who_am_i(), 2);
        wr_ch(&w, ']');
    }

    wr_eol(&w);
    (void)tx_push(line, w.len);
}

/**
 * So tho LSM6DSV. Dung de XAC DINH CHIEU TRUC o giai doan 2.
 *
 * CACH DUNG (xem App/Docs/KE_HOACH_LSM6DSV.md muc 2A):
 *   1. Bo nam phang, mat tren huong len -> cot az/g phai ~ -1.00.
 *      Ra +1.00 thi dao IMU2_AXIS_SIGN_Z trong fc_config.h.
 *   2. Dung bo len canh phai -> ay/g ~ +1.00 hoac -1.00, chinh SIGN_Y.
 *   3. Dung bo len mui truoc -> ax/g theo do, chinh SIGN_X.
 *
 * Dau AM cua az la DUNG: he truc than la NED, truc Z huong XUONG, ma gia
 * toc ke do luc rieng (phan luc mat ban) huong LEN.
 *
 * Bo ba dau phai la mot phep QUAY - dinh thuc bang +1. Dao 0 hoac 2 dau
 * thi hop le; dao 1 hoac 3 dau se lam sai chieu quay cua gyro.
 */
static void emit_imu2_raw(void)
{
    char line[DBG_LINE_MAX];
    wr_t w = { line, 0, sizeof(line) };

    const imu_data_t *imu = &g_fc.imu2;

    wr_i32(&w, imu->accel_raw.x, 8);
    wr_i32(&w, imu->accel_raw.y, 8);
    wr_i32(&w, imu->accel_raw.z, 8);
    wr_str(&w, " |");
    wr_i32(&w, imu->gyro_raw.x, 8);
    wr_i32(&w, imu->gyro_raw.y, 8);
    wr_i32(&w, imu->gyro_raw.z, 8);
    wr_str(&w, " |");
    wr_fix(&w, imu->accel_mps2.x / FC_GRAVITY_MPS2, 2, 7);
    wr_fix(&w, imu->accel_mps2.y / FC_GRAVITY_MPS2, 2, 7);
    wr_fix(&w, imu->accel_mps2.z / FC_GRAVITY_MPS2, 2, 7);

    wr_eol(&w);
    (void)tx_push(line, w.len);
}


static void emit_magcal(void)
{
    char line[DBG_LINE_MAX];
    wr_t w = { line, 0, sizeof(line) };

    const vec3f_t g = g_fc.mag.raw_gauss;   /* he CAM BIEN, CHUA hieu chuan */
    const double  v[3] = { (double)g.x, (double)g.y, (double)g.z };

    /* --- Nap mau vao phuong trinh chuan tac --- */
    if (g_fc.mag.healthy && g_fc.mag.sample_count > 0u) {
        if (!s_magcal.started) {
            magcal_reset();
            s_magcal.started = true;
        }
        {
            const double d[MAGCAL_N] = {
                v[0]*v[0], v[1]*v[1], v[2]*v[2],
                2.0*v[0], 2.0*v[1], 2.0*v[2]
            };

            for (int i = 0; i < MAGCAL_N; i++) {
                for (int j = i; j < MAGCAL_N; j++) {
                    s_magcal.ata[i][j] += d[i] * d[j];
                }
                s_magcal.atb[i] += d[i];
            }
        }
        s_magcal.n++;
    }

    /*
     * --- Khop MOT lan moi nhip ---
     *
     * magcal_fit() giai he 9x9 bang khu Gauss voi so thuc DOUBLE. M7 khong co
     * FPU double nen moi phep tinh deu la giai lap phan mem - mot lan giai
     * ton co tram micro giay. Truoc day ham nay bi goi HAI lan moi nhip (mot
     * cho phep do phu, mot cho hien thi), tuc gap doi chi phi ma khong duoc
     * gi. Giai mot lan roi dung chung.
     */
    float off[3] = { 0.0f, 0.0f, 0.0f };
    float sc[3]  = { 1.0f, 1.0f, 1.0f };
    float rad    = 0.0f;
    const bool ok = magcal_fit(off, sc, &rad);

    /*
     * Do phu chi de BAO nguoi dung da xoay du chua, khong tham gia tinh toan.
     */
    if (ok && g_fc.mag.healthy && g_fc.mag.sample_count > 0u) {
        const double bmag = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
        int    oct = 0;
        double d2 = 0.0;

        for (int i = 0; i < 3; i++) {
            const double dd = v[i] - (double)off[i];

            d2 += dd * dd;
            if (dd > 0.0) { oct |= (1 << i); }
        }
        if (sqrt(d2) > 0.3 * bmag) {
            s_magcal.octant |= (uint8_t)(1u << oct);
        }
    }

    wr_fix(&w, (float)v[0], 3, 7);
    wr_fix(&w, (float)v[1], 3, 7);
    wr_fix(&w, (float)v[2], 3, 7);
    wr_str(&w, " |");
    wr_fix(&w, g_fc.mag.magnitude_gauss, 3, 8);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)s_magcal.n, 8);
    wr_str(&w, " |");
    wr_i32(&w, magcal_coverage(), 4);
    wr_str(&w, " |");
    wr_fix(&w, off[0], 3, 7);
    wr_fix(&w, off[1], 3, 7);
    wr_fix(&w, off[2], 3, 7);
    wr_str(&w, " |");
    wr_fix(&w, sc[0], 3, 7);
    wr_fix(&w, sc[1], 3, 7);
    wr_fix(&w, sc[2], 3, 7);
    wr_str(&w, " | ");

    if (!g_fc.mag.healthy) {
        wr_str(&w, "TU KE CHUA CHAY");
    } else if (!ok) {
        wr_str(&w, "chua du du lieu de khop");
    } else if (magcal_coverage() < 8) {
        wr_str(&w, "XOAY TIEP - con thieu ");
        wr_i32(&w, 8 - magcal_coverage(), 0);
        wr_str(&w, "/8 huong");
    } else if (s_magcal.n < MAGCAL_MIN_SAMPLES) {
        wr_str(&w, "du huong roi, xoay them cho du mau");
    } else {
        wr_str(&w, "DU DIEU KIEN - bam K1 de chot va in ket qua");
    }

    wr_eol(&w);
    (void)tx_push(line, w.len);

    /* --- Chot ket qua khi bam nut K1 --- */
    {
        static bool k1_prev = false;
        const bool  k1 =
            (HAL_GPIO_ReadPin(BUTTON_K1_GPIO_Port, BUTTON_K1_Pin) == GPIO_PIN_RESET);

        if (k1 && !k1_prev && ok) {
            static const char *nm[6] = {
                "MAG_OFFSET_X_G", "MAG_OFFSET_Y_G", "MAG_OFFSET_Z_G",
                "MAG_SCALE_X   ", "MAG_SCALE_Y   ", "MAG_SCALE_Z   "
            };
            const float val[6] = { off[0], off[1], off[2], sc[0], sc[1], sc[2] };

            dbg_println("");
            dbg_println("=== KET QUA HIEU CHUAN TU KE - chep vao fc_config.h ===");
            for (int i = 0; i < 6; i++) {
                char  b[DBG_LINE_MAX];
                wr_t  bw = { b, 0, sizeof(b) };

                wr_str(&bw, "#define ");
                wr_str(&bw, nm[i]);
                wr_ch(&bw, ' ');
                wr_fix(&bw, val[i], 4, 0);
                wr_ch(&bw, 'f');
                wr_eol(&bw);
                (void)tx_push(b, bw.len);
            }
            dbg_println("");
            dbg_print_float("  |B| ky vong sau hieu chuan (G)", rad, 4);
            dbg_print_int("  so mau", (int32_t)s_magcal.n);
            dbg_print_int("  goc phan tam da phu (can 8)", magcal_coverage());

            if (magcal_coverage() < 8) {
                dbg_println("  !!! CHUA PHU DU 8 HUONG - ket qua KHONG dang tin.");
            }
            if (rad < 0.15f || rad > 0.80f) {
                dbg_println("  !!! |B| nam ngoai dai tu truong Trai Dat 0,25-0,65 G.");
                dbg_println("  !!! Nhieu kha nang co khoi kim loai gan do.");
            }
            dbg_println("");
            dbg_println("  Nap xong thi doi sang DBG_MODE_MAG va xoay kiem:");
            dbg_println("  Btot phai bam quanh gia tri tren, dao dong duoi 5%.");
            dbg_println("");
            magcal_reset();
        }
        k1_prev = k1;
    }
}

/**
 * Blackbox — trang thai ghi log ra the SD.
 *
 * CACH DOC:
 *   ban ghi   so ban ghi dang co trong bo dem RAM
 *   suc chua  toi da chua duoc bao nhieu, va bao nhieu giay o BB_RATE_HZ
 *   bo        so ban ghi bi BO vi bo dem day. Khac 0 nghia la chuyen bay dai
 *             hon suc chua - noi BB_BUFFER_BYTES hoac ha BB_RATE_HZ.
 *   file      dang ghi vao LOGnnnn.CSV
 *   xa        tien do xa ra the, %
 *   loi       ma FRESULT cua thao tac the gan nhat. Phai la 0.
 *
 * LUONG HOAT DONG: arm -> DANG GHI vao RAM (khong cham the) -> disarm ->
 * dang xa ra the -> san sang cho chuyen sau.
 */
static void emit_log(void)
{
    char line[DBG_LINE_MAX];
    wr_t w = { line, 0, sizeof(line) };

    const uint32_t cap = BB_BUFFER_BYTES / sizeof(bb_record_t);

    wr_i32(&w, (int32_t)blackbox_records(), 9);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)cap, 7);
    wr_str(&w, "/");
    wr_i32(&w, (int32_t)(cap / BB_RATE_HZ), 0);
    wr_ch(&w, 's');
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)blackbox_dropped(), 4);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)blackbox_file_index(), 5);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)blackbox_flush_percent(), 4);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)blackbox_last_error(), 4);
    wr_str(&w, " | ");
    wr_str(&w, blackbox_state_name());

    wr_eol(&w);
    (void)tx_push(line, w.len);
}

/**
 * Từ kế — nguồn SHUB (QMC6309 qua sensor hub LSM6DSV) hoặc I2C (module rời
 * trên I2C1, xem mag_i2c.c). Cột chung cho cả hai nguồn:
 *
 *   mx/my/mz  tu truong theo he truc THAN, don vi Gauss, da tru hieu chuan
 *   Btot      DO LON vector tu truong. Day la cot quan trong nhat:
 *             XOAY may bay theo MOI HUONG ma Btot gan nhu khong doi -> tot.
 *             Btot doi nhieu -> chua hieu chuan hoac doc sai byte.
 *             Tu truong Trai Dat khoang 0,25 - 0,65 G tuy vi tri dia ly.
 *   raw       so tho 16 bit, he truc CAM BIEN, dung de xac dinh chieu truc
 *
 * Cột riêng nguồn SHUB:
 *   hz        nhip cap nhat thuc te, phai bam MAG_UPDATE_RATE_HZ
 *   nack      QMC6309 khong tra loi tren bus I2C phu. PHAI dung yen o 0.
 *   busy      so lan bo mot luot doc vi khong gianh duoc bus SPI3 voi DMA
 *             cua IMU. Vai lan la binh thuong, tang lien tuc thi co van de.
 *
 * Cột riêng nguồn I2C:
 *   dt_ms     TUOI cua mau moi nhat luc in dong nay, khong phai chu ky giua
 *             hai mau. Nen no chay trong khoang 0..1000/MAG_I2C_UPDATE_RATE_HZ
 *             tuy pha giua nhip in va nhip doc. Dung de phat hien cam bien
 *             CHET (so leo len roi dung o tren MAG_TIMEOUT_MS), chu khong
 *             dung do tan so - muon do tan so thi xem cot count tang bao
 *             nhieu trong mot khoang thoi gian.
 *   lost      so lan GOI doc gap bus I2C1 dang ban vi BMP388 truyen. KHONG
 *             phai so mau bi mat: gap bus ban thi driver thu lai ngay o vong
 *             lap ke tiep, nen mot mau co the ton nhieu lan thu (do duoc
 *             ~9 lan/mau, tuc ~450/giay o nhip 50 Hz). Muon biet co mat mau
 *             that khong thi xem cot count co tang du nhip khong.
 *   stale     so lan doc lai dung mau cu (bit RDY/DRDY chua len) vi hoi vong
 *             nhanh hon ODR cua chip.
 *   yerr      SAI SO HUONG MUI lan do gan nhat, do. Day la thu EKF dang
 *             dung de sua yaw. Phai dao dong quanh 0; lech mot chieu keo
 *             dai nghia la do lech tu thien dat sai hoac tu ke bi nhieu.
 *   mrej      so mau bi EKF LOAI (nghieng qua, |B| lech qua, hoac doi moi
 *             vuot cong). Tang deu khi len ga = nhieu dong dong co - do la
 *             cong |B| dang lam dung viec cua no.
 */
static void emit_mag(void)
{
    char line[DBG_LINE_MAX];
    wr_t w = { line, 0, sizeof(line) };

    const mag_data_t *m = &g_fc.mag;

    wr_fix(&w, m->field_gauss.x, 3, 7);
    wr_fix(&w, m->field_gauss.y, 3, 7);
    wr_fix(&w, m->field_gauss.z, 3, 7);
    wr_str(&w, " |");
    wr_fix(&w, m->magnitude_gauss, 3, 8);
    wr_str(&w, " |");
    wr_i32(&w, m->raw.x, 7);
    wr_i32(&w, m->raw.y, 7);
    wr_i32(&w, m->raw.z, 7);
    wr_str(&w, " |");

#if MAG_SOURCE == MAG_SOURCE_SHUB
    wr_i32(&w, (int32_t)lsm6dsv_mag_rate_hz(), 6);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)m->sample_count, 10);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)m->error_count, 5);
    wr_i32(&w, (int32_t)lsm6dsv_mag_nacks(), 6);
    wr_i32(&w, (int32_t)lsm6dsv_mag_bus_busy(), 6);
    wr_str(&w, " | ");

    if (m->chip_id != 0x90u) {
        wr_str(&w, "KHONG THAY - ");
        wr_str(&w, lsm6dsv_mag_init_result_name());
        wr_str(&w, " | id=");
        wr_hex(&w, m->chip_id, 2);
        wr_str(&w, " STATUS_MASTER=");
        wr_hex(&w, lsm6dsv_mag_last_status(), 2);
    } else if (m->overflow) {
        wr_str(&w, "TRAN DAI DO - noi MAG_RANGE_G len 16 hoac 32");
    } else if (!m->healthy) {
        wr_str(&w, "chua co du lieu");
    } else if (!m->calibrated) {
        wr_str(&w, "chay @ ");
        wr_hex(&w, lsm6dsv_mag_addr(), 2);
        wr_str(&w, " - CHUA HIEU CHUAN (giai doan 4)");
    } else {
        wr_str(&w, "OK");
    }
#elif MAG_SOURCE == MAG_SOURCE_I2C
    wr_i32(&w, (int32_t)(fc_elapsed_us(micros(), m->timestamp_us) / 1000u), 6);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)m->sample_count, 10);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)m->error_count, 5);
    wr_i32(&w, (int32_t)mag_i2c_bus_lost(), 6);
    wr_i32(&w, (int32_t)mag_i2c_stale_reads(), 6);
    wr_str(&w, " |");
    /* Hai cot cua khau HOP NHAT YAW, khong phai cua driver. */
    wr_fix(&w, ekf_attitude_mag_yaw_err_deg(), 1, 7);
    wr_i32(&w, (int32_t)ekf_attitude_mag_rejects(), 8);
    wr_str(&w, " | ");

    if (m->chip_id == 0u) {
        wr_str(&w, mag_i2c_variant_name());
    } else if (m->overflow) {
        wr_str(&w, "TRAN DAI DO - dua mag_range_g len 8");
    } else if (!m->healthy) {
        wr_str(&w, "chua co du lieu");
    } else if (!m->calibrated) {
        wr_str(&w, mag_i2c_variant_name());
        wr_str(&w, " - CHUA HIEU CHUAN (xem GD4 trong KE_HOACH_LA_BAN_I2C.md)");
    } else if (!ekf_attitude_mag_enabled()) {
        wr_str(&w, "OK - hop nhat yaw DANG TAT (est_mag_yaw_enable=0)");
    } else if (!ekf_attitude_mag_aligned()) {
        wr_str(&w, "OK - dang cho chot huong lan dau");
    } else {
        wr_str(&w, "OK - dang sua yaw cho EKF");
    }
#else
    wr_i32(&w, (int32_t)m->sample_count, 10);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)m->error_count, 5);
    wr_str(&w, " | MAG_SOURCE = NONE, khong co du lieu");
#endif

    wr_eol(&w);
    (void)tx_push(line, w.len);
}

/**
 * TU NHAN chieu truc IMU phu. Cong cu chinh cua GIAI DOAN 2A.
 *
 * CACH DUNG: dat may bay lan luot vao ba tu the ma cot "huong dan" yeu cau,
 * giu yen moi tu the khoang mot giay. Bat duoc thi no tu chuyen buoc. Xong
 * ba buoc, console in thang sau dong can dan vao fc_config.h.
 *
 * Doc raw_x/raw_y/raw_z la so THO cua CAM BIEN, chua qua phep xoay truc -
 * dung vay moi suy ra duoc phep xoay.
 */
static void emit_axiscal(void)
{
    char line[DBG_LINE_MAX];
    wr_t w = { line, 0, sizeof(line) };

    const int32_t r[3] = { g_fc.imu2.accel_raw.x,
                           g_fc.imu2.accel_raw.y,
                           g_fc.imu2.accel_raw.z };

    wr_i32(&w, r[0], 7);
    wr_i32(&w, r[1], 7);
    wr_i32(&w, r[2], 7);
    wr_str(&w, " |");

    /* --- Tim truc chiem uu the --- */
    const float mag = sqrtf((float)(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]));

    int   dom  = -1;
    int   sign = 0;
    float best = 0.0f;

    for (int i = 0; i < 3; i++) {
        const float a = fabsf((float)r[i]);

        if (a > best) { best = a; dom = i; sign = (r[i] >= 0) ? +1 : -1; }
    }

    const bool level    = (fabsf(vec3f_norm(g_fc.imu2.accel_mps2) - FC_GRAVITY_MPS2)
                           < 0.15f * FC_GRAVITY_MPS2);
    const bool still    = (vec3f_norm(g_fc.imu2.gyro_dps) < 5.0f);
    const bool dominant = (mag > 1.0f) && ((best / mag) > AXCAL_DOMINANCE);
    const bool good     = level && still && dominant && (dom >= 0);

    if (good) {
        wr_spaces(&w, 5);
        wr_ch(&w, (char)('X' + dom));
        wr_spaces(&w, 4);
        wr_ch(&w, (sign > 0) ? '+' : '-');
    } else {
        wr_str(&w, "     -     -");
    }
    wr_str(&w, " |");

    /*
     * --- Giu on dinh thi chot ---
     *
     * HAI CONG chan viec tu chot lien tiep tren cung mot tu the:
     *
     *   1. Sau moi lan chot, phai thay may bay DA BI NHAC LEN (so doc khong
     *      con on dinh) thi moi bat dau do buoc ke tiep. Khong co cong nay
     *      thi ba buoc tu chot cach nhau dung AXCAL_HOLD_TICKS trong khi may
     *      van nam yen - da tung xay ra that.
     *   2. Truc cam bien nao da bi mot buoc truoc chiem thi khong nhan nua.
     */
    const bool taken = (dom >= 0) && axcal_axis_taken(dom);

    if (s_axcal_step < 3u) {
        if (s_axcal_wait) {
            /* Cho tin hieu nguoi dung da nhac may bay len. */
            if (!good) { s_axcal_wait = false; }
            s_axcal_hold = 0;
            s_axcal_cand = -1;
        } else if (good && !taken && dom == s_axcal_cand && sign == s_axcal_cand_sign) {
            if (s_axcal_hold < AXCAL_HOLD_TICKS) { s_axcal_hold++; }
        } else {
            s_axcal_cand      = (int8_t)((good && !taken) ? dom : -1);
            s_axcal_cand_sign = (int8_t)sign;
            s_axcal_hold      = 0;
        }

        if (s_axcal_hold >= AXCAL_HOLD_TICKS) {
            const uint8_t b = s_axcal_body[s_axcal_step];

            s_axcal.map[b]  = (int8_t)dom;
            s_axcal.sign[b] = (int8_t)(s_axcal_want[s_axcal_step] * sign);
            s_axcal_step++;
            s_axcal_hold = 0;
            s_axcal_cand = -1;
            s_axcal_wait = true;
        }
    }

    wr_i32(&w, (int32_t)((s_axcal_hold * 100u) / AXCAL_HOLD_TICKS), 4);
    wr_ch(&w, '%');
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)s_axcal_step, 4);
    wr_str(&w, " | ");

    if (s_axcal_step < 3u) {
        wr_str(&w, "dat ");
        wr_str(&w, s_axcal_pose[s_axcal_step]);
        wr_str(&w, " (than ");
        wr_str(&w, s_axcal_name[s_axcal_step]);
        wr_str(&w, (s_axcal_want[s_axcal_step] > 0) ? " = +1g)" : " = -1g)");

        if (s_axcal_wait) {
            wr_str(&w, "  [DA BAT BUOC TRUOC - HAY XOAY MAY BAY]");
        } else if (!still) {
            wr_str(&w, "  [DANG RUNG]");
        } else if (!level) {
            wr_str(&w, "  [|a| KHONG BANG 1g]");
        } else if (!dominant) {
            wr_str(&w, "  [CHUA DUNG HAN MOT TRUC]");
        } else if (taken) {
            wr_str(&w, "  [TRUC NAY DA DUNG O BUOC TRUOC - XOAY TIEP]");
        }
    } else {
        wr_str(&w, "XONG - xem ba dong duoi");
    }

    wr_eol(&w);
    (void)tx_push(line, w.len);

    /* --- Xong ba buoc: in ket qua mot lan roi khoa lai --- */
    if (s_axcal_step == 3u) {
        if (!s_axcal_reported) {
            s_axcal_reported = true;

            /*
             * Dinh thuc cua ma tran hoan vi CO DAU. Phai bang +1: dao 0 hoac
             * 2 dau la phep QUAY hop le, dao 1 hoac 3 dau bien he truc thanh
             * TAY TRAI va se lam sai chieu quay cua gyro.
             */
            int det = 0;
            {
                const int8_t *m = s_axcal.map;
                const int8_t *g = s_axcal.sign;

                if (m[0] != m[1] && m[1] != m[2] && m[0] != m[2]) {
                    /* dau cua phep hoan vi */
                    int perm = ((m[1] - m[0] + 3) % 3 == 1) ? +1 : -1;

                    det = perm * g[0] * g[1] * g[2];
                }
            }

            dbg_println("");
            dbg_println("=== KET QUA CAN TRUC IMU2 - chep vao fc_config.h ===");
            for (int i = 0; i < 3; i++) {
                char  buf[48];
                wr_t  b = { buf, 0, sizeof(buf) };

                wr_str(&b, "#define IMU2_AXIS_MAP_");
                wr_ch(&b, (char)('X' + i));
                wr_ch(&b, ' ');
                wr_i32(&b, s_axcal.map[i], 0);
                wr_eol(&b);
                (void)tx_push(buf, b.len);
            }
            for (int i = 0; i < 3; i++) {
                char  buf[48];
                wr_t  b = { buf, 0, sizeof(buf) };

                wr_str(&b, "#define IMU2_AXIS_SIGN_");
                wr_ch(&b, (char)('X' + i));
                wr_str(&b, (s_axcal.sign[i] > 0) ? " (+1)" : " (-1)");
                wr_eol(&b);
                (void)tx_push(buf, b.len);
            }

            if (det == 1) {
                dbg_println("dinh thuc = +1  -> phep QUAY hop le, dung duoc.");
            } else {
                dbg_println("!!! dinh thuc KHONG bang +1 - KHONG duoc dung.");
                dbg_println("!!! Ba tu the chac chan bi dat sai. Doi mode roi lam lai.");
            }
        }
    }
}

/**
 * SO SANH hai IMU - cong cu chinh cua GIAI DOAN 2.
 *
 * Cot _1 la ICM20602 (IMU chinh), cot _2 la LSM6DSV (IMU phu).
 *
 * 2B. KIEM THANG GYRO:
 *   Xoay bo bang tay quanh tung truc. Hai cot cua cung mot truc phai BAM
 *   NHAU. Lech theo TI LE CO DINH -> sai he so LSM_GYRO_SCALE. Lech DAU ->
 *   sai IMU2_AXIS_SIGN_*. HOAN VI cot -> sai IMU2_AXIS_MAP_*.
 *
 * 2C. DO NEN NHIEU - so lieu quyet dinh:
 *   sd_1, sd_2 la do lech chuan gyro cua cua so 1 giay vua xong, tinh ngay
 *   trong ISR nen phan anh dung nhieu o toc do day du.
 *
 *   CHI DOC KHI MAY DUNG YEN hoac giu ga on dinh. Dang xoay thi vo nghia.
 *
 *   Ghi lai HAI bo so:
 *     (a) Dung yen tren ban, khong cham.
 *     (b) THAO CANH QUAT, arm, ga ~30%.
 *
 *   CONG QUYET DINH:
 *     (b) >> (a) o CA HAI, va sd_1 ~ sd_2  -> nhieu la RUNG KHUNG.
 *         Bo giai doan 5. Hop nhat IMU khong giup gi. Lam thay: dem mem
 *         cho bo, can canh quat, bo loc chan dai.
 *     sd_1 va sd_2 KHAC NHAU ro  -> nhieu co phan doc lap, hop nhat co ich.
 *     sd_2 << sd_1 o ca hai dieu kien -> can nhac doi LSM6DSV lam IMU chinh.
 */
static void emit_imu_cmp(void)
{
    char line[DBG_LINE_MAX];
    wr_t w = { line, 0, sizeof(line) };

    const imu_data_t *a = &g_fc.imu;    /* ICM20602 */
    const imu_data_t *b = &g_fc.imu2;   /* LSM6DSV  */

    wr_fix(&w, a->gyro_dps.x, 2, 7);
    wr_fix(&w, b->gyro_dps.x, 2, 7);
    wr_str(&w, " |");
    wr_fix(&w, a->gyro_dps.y, 2, 7);
    wr_fix(&w, b->gyro_dps.y, 2, 7);
    wr_str(&w, " |");
    wr_fix(&w, a->gyro_dps.z, 2, 7);
    wr_fix(&w, b->gyro_dps.z, 2, 7);
    wr_str(&w, " |");
    wr_fix(&w, icm20602_gyro_sigma_dps(), 3, 7);
    wr_fix(&w, lsm6dsv_gyro_sigma_dps(),  3, 7);
    wr_str(&w, " |");
    wr_fix(&w, vec3f_norm(a->accel_mps2) / FC_GRAVITY_MPS2, 2, 7);
    wr_fix(&w, vec3f_norm(b->accel_mps2) / FC_GRAVITY_MPS2, 2, 7);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)ctrl_rate_hz(), 6);
    wr_i32(&w, (int32_t)lsm6dsv_sample_rate_hz(), 6);

    if (!b->healthy) {
        wr_str(&w, "  [IMU2 CHUA CO DU LIEU]");
    } else if (!b->calibrated) {
        wr_str(&w, "  [IMU2 CHUA HIEU CHUAN - huy ");
        wr_i32(&w, (int32_t)lsm6dsv_calib_restarts(), 0);
        wr_str(&w, " lan, bien do gyro vuot IMU2_CALIB_MOVE_LIMIT_DPS]");
    }

    wr_eol(&w);
    (void)tx_push(line, w.len);
}

/**
 * Trang thai he thong. Mot dong = mot anh chup toan bo suc khoe may bay.
 *
 * CACH DOC:
 *   che do  INIT / CALIBRATING / DISARMED / ARMED / FAILSAFE / FAULT
 *
 *   health  bitmask cam bien DANG SONG (fc_sensor_mask_t):
 *             bit0 gyro   bit1 accel  bit2 baro   bit3 flow   bit4 range
 *             bit5 RC     bit6 power  bit7 sdcard bit8 mag    bit10 imu2
 *           0x001F = 5 cam bien dau du, KHONG co RC (tat tay dieu khien).
 *           0x041F = nhu tren, co them IMU phu LSM6DSV.
 *
 *   err     co loi TICH LUY (fc_error_flag_t). KHONG tu xoa, nen mot loi
 *           thoang qua luc khoi dong se con treo o day mai. Doi chieu voi
 *           cot health de biet hien tai con hong that khong.
 *             bit0 imu timeout  bit1 imu spi    bit2 baro i2c
 *             bit3 flow timeout bit4 rc timeout bit5 loop overrun
 *             bit6 sdcard       bit7 dshot dma  bit8 low voltage
 *             bit9 imu2 spi
 *           0x0018 tren ban thu = flow timeout + rc timeout, binh thuong.
 *
 *   armblk  ly do dang chan arm. 0 = san sang arm.
 *   imu_n   so mau ICM20602. Phai tang 8000 moi giay.
 *   dt_us   khoang cach hai mau IMU. Phai la 125 us.
 *   ierr    loi giao tiep IMU chinh. PHAI dung yen o 0.
 *
 *   loop    thoi gian vong lap chinh vua roi, us
 *   lmax    dinh trong CUA SO 1 GIAY vua roi
 *   slow    so vong CHAM (>125 us = mot chu ky mau IMU) trong 1 giay vua roi.
 *           Nguong la 125 us chu KHONG phai 250 us: mot gia tri count chan
 *           chi ton tai 125 us truoc khi bi count+1 thay the, nen vong lap
 *           ban lau hon the la du de lo mot nhip PID.
 *
 *   rhz     tan so THUC TE cua vong PID toc do goc. Danh dinh 4000.
 *   rsk     ctrl_rate bo nhip vi dt bat thuong. Phai dung yen.
 *   imu2    so mau LSM6DSV. Dung yen = IMU phu chet.
 *   up      thoi gian chay, giay
 *   drop    so dong console bi mat vi day gui day. Phai la 0.
 */
static void emit_status(void)
{
    char line[DBG_LINE_MAX];
    wr_t w = { line, 0, sizeof(line) };

    wr_str_pad(&w, fc_mode_name(g_fc.mode), 12);
    wr_str(&w, " |");
    wr_hex_col(&w, g_fc.sys.sensor_health,   4, 8);
    wr_hex_col(&w, g_fc.sys.error_flags,     4, 8);
    wr_hex_col(&w, g_fc.sys.arm_block_flags, 4, 8);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)g_fc.imu.sample_count, 10);
    wr_i32(&w, (int32_t)g_fc.imu.dt_us,         7);
    wr_i32(&w, (int32_t)g_fc.imu.error_count,   7);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)g_fc.sys.loop_time_us,     7);
    wr_i32(&w, (int32_t)g_fc.sys.loop_time_max_us, 7);
    wr_i32(&w, (int32_t)g_fc.sys.loop_overruns,    7);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)ctrl_rate_hz(),    7);
    wr_i32(&w, (int32_t)ctrl_rate_skips(), 6);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)g_fc.imu2.sample_count, 10);
    wr_str(&w, " |");
    wr_i32(&w, (int32_t)(g_fc.sys.uptime_ms / 1000u), 7);
    wr_i32(&w, (int32_t)s_dropped, 7);

    wr_eol(&w);
    (void)tx_push(line, w.len);
}

/* ==========================================================================
 * API chính
 * ========================================================================== */

void dbg_console_init(UART_HandleTypeDef *huart)
{
    /* Section .dma_buffer là NOLOAD nên không được startup code xoá. */
    memset(s_tx, 0, sizeof(s_tx));

    s_uart       = huart;
    s_head       = 0;
    s_tail       = 0;
    s_inflight   = 0;
    s_busy       = false;
    s_dropped    = 0;
    s_mode       = DBG_MODE_OFF;
    s_period_ms  = 1000u / DBG_DEFAULT_RATE_HZ;
    s_next_ms    = 0;
    s_line_count = 0;
}

void dbg_console_set_mode(dbg_mode_t mode)
{
    if (mode >= DBG_MODE_COUNT) {
        return;
    }
    if (mode == DBG_MODE_MAGCAL) {
        magcal_reset();    /* moi lan vao mode nay la do lai tu dau */
    }
    if (mode == DBG_MODE_AXISCAL) {
        axcal_reset();     /* moi lan vao mode nay la lam lai tu buoc 1 */
    }
    s_mode       = mode;
    s_line_count = 0;      /* in lại tiêu đề ngay dòng đầu */
    s_next_ms    = 0;
}

dbg_mode_t dbg_console_get_mode(void)
{
    return s_mode;
}

void dbg_console_set_rate(uint16_t hz)
{
    if (hz == 0u)   { hz = 1u; }
    if (hz > 1000u) { hz = 1000u; }
    s_period_ms = (uint16_t)(1000u / hz);
    if (s_period_ms == 0u) {
        s_period_ms = 1u;
    }
}

void dbg_console_update(uint32_t now_ms)
{
    if (s_uart == NULL) {
        return;
    }

    /* Luôn cố đẩy phần còn lại trong đệm, kể cả khi console đang tắt. */
    const uint32_t primask = critical_enter();
    tx_kick();
    critical_exit(primask);

    if (s_mode == DBG_MODE_OFF) {
        return;
    }

    /* So sánh có dấu để an toàn khi bộ đếm mili giây tràn. */
    if ((int32_t)(now_ms - s_next_ms) < 0) {
        return;
    }
    s_next_ms = now_ms + s_period_ms;

    yaw_drift_track(now_ms);

    /* In lại tiêu đề cột theo chu kỳ cho dễ đọc khi màn hình đã cuộn. */
    if (s_line_count == 0u) {
        emit_header();
    }
    s_line_count = (uint16_t)((s_line_count + 1u) % DBG_HEADER_EVERY_LINES);

    switch (s_mode) {
    case DBG_MODE_IMU:      emit_imu();     break;
    case DBG_MODE_IMU_RAW:  emit_imu_raw(); break;
    case DBG_MODE_IMU_CSV:  emit_imu_csv(); break;
    case DBG_MODE_FLOW:     emit_flow();     break;
    case DBG_MODE_FLOW_RAW: emit_flow_raw(); break;
    case DBG_MODE_BARO:     emit_baro();    break;
    case DBG_MODE_RC:       emit_rc();      break;
    case DBG_MODE_RC_RAW:   emit_rc_raw();  break;
    case DBG_MODE_ARM:      emit_arm();     break;
    case DBG_MODE_MOTOR:    emit_motor();   break;
    case DBG_MODE_FLOWCAL:  emit_flowcal(); break;
    case DBG_MODE_VEL:      emit_vel();     break;
    case DBG_MODE_POSHOLD:  emit_poshold(); break;
    case DBG_MODE_ALTHOLD:  emit_althold(); break;
    case DBG_MODE_ANGLE:    emit_angle();   break;
    case DBG_MODE_PID:      emit_pid();     break;
    case DBG_MODE_EST:      emit_est();     break;
    case DBG_MODE_STATUS:   emit_status();  break;
    case DBG_MODE_IMU2:     emit_imu2();     break;
    case DBG_MODE_IMU2_RAW: emit_imu2_raw(); break;
    case DBG_MODE_LOG:      emit_log();      break;
    case DBG_MODE_MAG:      emit_mag();      break;
    case DBG_MODE_MAGCAL:   emit_magcal();   break;
    case DBG_MODE_AXISCAL:  emit_axiscal();  break;
    case DBG_MODE_IMU_CMP:  emit_imu_cmp();  break;
    default:                                break;
    }
}

bool dbg_console_owns(const UART_HandleTypeDef *huart)
{
    return (s_uart != NULL) && (huart == s_uart);
}
