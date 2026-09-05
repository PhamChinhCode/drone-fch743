/**
 * @file    cli.c
 * @brief   Dòng lệnh chỉnh tham số. Xem cli.h.
 */
#include "cli.h"
#include "dbg_console.h"
#include "param_table.h"
#include "param_store.h"
#include "param_apply.h"
#include "tlm_port.h"
#include "fc_state.h"
#include "arming.h"
#include "fc_time.h"
#include "qspi_flash.h"
#include "flashlog.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ==========================================================================
 * Nhận
 * ========================================================================== */

#define CLI_RX_BUFFER_SIZE  256u
#define CLI_LINE_MAX        96u

/* DMA2 không truy cập được DTCMRAM — bắt buộc nằm ở AXI SRAM. */
FC_DMA_BUFFER static uint8_t s_rx_buf[CLI_RX_BUFFER_SIZE];

static UART_HandleTypeDef *s_uart;
static uint16_t            s_rx_tail;

static char     s_line[CLI_LINE_MAX];
static uint16_t s_line_len;
static bool     s_overflow;   /**< dòng quá dài -> bỏ tới hết dòng */

/* ==========================================================================
 * Trạng thái in nhiều dòng
 *
 * `dump` / `diff` / `get` in nhiều hơn sức chứa bộ đệm console, nên chúng chỉ
 * ghi lại "đang in tới đâu" rồi trả quyền cho vòng lặp chính. cli_update()
 * in tiếp mỗi lần được gọi, chừng nào còn chỗ trống.
 * ========================================================================== */

typedef enum {
    LIST_NONE = 0,
    LIST_GET,      /**< dạng đầy đủ: giá trị + min/max/mặc định */
    LIST_DUMP,     /**< dạng `set tên=giá_trị`, dán lại được    */
    LIST_DIFF      /**< như DUMP nhưng bỏ qua tham số chưa đổi  */
} cli_list_mode_t;

static cli_list_mode_t s_list_mode;
static uint16_t        s_list_index;
static char            s_list_filter[PARAM_NAME_MAX];

/** Chừa dư trước khi in một dòng. Dòng dài nhất khoảng 90 ký tự. */
#define CLI_LINE_RESERVE  128u

/* ==========================================================================
 * Định dạng số thực
 * ========================================================================== */

static int put_uint(char *out, int cap, uint32_t v, int min_digits)
{
    char tmp[12];
    int  n = 0;

    do {
        tmp[n++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v != 0u && n < (int)sizeof(tmp));

    while (n < min_digits && n < (int)sizeof(tmp)) {
        tmp[n++] = '0';
    }

    int w = 0;
    while (n > 0 && w < cap) {
        out[w++] = tmp[--n];
    }
    return w;
}

/**
 * In `value` voi dung `sig` chu so co nghia.
 *
 * Tach rieng khoi cli_format_float() de ham do thu duoc nhieu do chinh xac
 * khac nhau - xem giai thich o do.
 */
static int format_sig(char *out, int cap, float value, int sig)
{
    int w = 0;

    if (value < 0.0f) {
        out[w++] = '-';
        value = -value;
    }

    /*
     * Chuan hoa ve a x 10^e voi 1 <= a < 10.
     *
     * Dung double cho phan nay. FPU cua H743 la fpv5-d16, tuc co phan cung
     * cho double - phep chia lap duoi day khong dat, ma lam bang float thi
     * sai so tich luy du de chu so thu 7 sai.
     */
    double a = (double)value;
    int    e = 0;

    while (a >= 10.0) { a /= 10.0;  e++; }
    while (a <  1.0)  { a *= 10.0;  e--; }

    uint32_t scale = 1u;
    for (int i = 1; i < sig; i++) scale *= 10u;

    uint32_t digits = (uint32_t)(a * (double)scale + 0.5);
    if (digits >= scale * 10u) {   /* lam tron len dung 10.000... */
        digits /= 10u;
        e++;
    }

    int ndig = sig;
    while (ndig > 1 && (digits % 10u) == 0u) {
        digits /= 10u;
        ndig--;
    }

    /*
     * Chon dang in. Trong khoang [1e-4, 1e9) thi thap phan thuong de doc hon
     * han; ngoai khoang do thi no dai le the nen chuyen sang dang mu.
     */
    if (e >= -4 && e < 9) {
        if (e >= ndig - 1) {
            /* So nguyen, co the phai them so 0 phia sau. */
            w += put_uint(out + w, cap - w, digits, 0);
            for (int i = 0; i < e - (ndig - 1) && w < cap - 1; i++) {
                out[w++] = '0';
            }
        } else if (e >= 0) {
            /* Dau phay nam giua: tach digits thanh hai phan. */
            uint32_t frac_scale = 1u;
            for (int i = 0; i < ndig - 1 - e; i++) frac_scale *= 10u;

            w += put_uint(out + w, cap - w, digits / frac_scale, 0);
            if (w < cap - 1) out[w++] = '.';
            w += put_uint(out + w, cap - w, digits % frac_scale, ndig - 1 - e);
        } else {
            /* 0.000ddd - chen (-e-1) so 0 giua dau phay va chu so dau. */
            if (w < cap - 1) out[w++] = '0';
            if (w < cap - 1) out[w++] = '.';
            for (int i = 0; i < -e - 1 && w < cap - 1; i++) out[w++] = '0';
            w += put_uint(out + w, cap - w, digits, 0);
        }
    } else {
        uint32_t head_scale = 1u;
        for (int i = 1; i < ndig; i++) head_scale *= 10u;

        w += put_uint(out + w, cap - w, digits / head_scale, 0);
        if (ndig > 1) {
            if (w < cap - 1) out[w++] = '.';
            w += put_uint(out + w, cap - w, digits % head_scale, ndig - 1);
        }
        if (w < cap - 1) out[w++] = 'e';
        if (e < 0) {
            if (w < cap - 1) out[w++] = '-';
            w += put_uint(out + w, cap - w, (uint32_t)(-e), 0);
        } else {
            w += put_uint(out + w, cap - w, (uint32_t)e, 0);
        }
    }

    if (w >= cap) w = cap - 1;
    out[w] = '\0';
    return w;
}

int cli_format_float(char *out, int cap, float value)
{
    if (cap < 2) {
        if (cap > 0) out[0] = '\0';
        return 0;
    }

    if (isnan(value)) { strcpy(out, "nan"); return 3; }
    if (isinf(value)) { strcpy(out, value < 0 ? "-inf" : "inf"); return value < 0 ? 4 : 3; }
    if (value == 0.0f) { strcpy(out, "0"); return 1; }

    /*
     * "Ngan nhat ma van doc lai dung": thu tang dan do chinh xac, doc nguoc
     * bang strtof va dung lai ngay khi ra dung bit cu.
     *
     * VI SAO KHONG IN THANG 9 CHU SO:
     *   9 la so it nhat BAO DAM moi float round-trip duoc, nhung dem lai
     *   nhung dong nhu "set rate_pid_roll_kp=0.00120000006" cho mot gia tri
     *   ma nguoi dung go vao la 0.0012. Ban dump la thu con nguoi doc, sua
     *   tay va dan lai - rac so hoc lam no kho doc va kho tin.
     *
     *   Vong lap nay cho ra "0.0012" o do chinh xac 2 chu so, va van tu dong
     *   len 9 chu so voi nhung gia tri thuc su can (vi du he so hieu chuan tu
     *   ke do bang khop ellipsoid).
     *
     *   Chi phi la vai lan strtof - khong dang ke o CLI, va ham nay KHONG
     *   BAO GIO duoc goi trong vong nong.
     */
    for (int sig = 1; sig < 9; sig++) {
        const int w = format_sig(out, cap, value, sig);
        if (strtof(out, NULL) == value) {
            return w;
        }
    }
    return format_sig(out, cap, value, 9);
}

/* ==========================================================================
 * In một tham số
 * ========================================================================== */

static void append(char *buf, int cap, int *w, const char *s)
{
    while (*s && *w < cap - 1) {
        buf[(*w)++] = *s++;
    }
    buf[*w] = '\0';
}

/* ==========================================================================
 * Lop dau ra
 *
 * Moi thu CLI in ra deu di qua ba ham nay, khong goi thang dbg_* nua. Nho vay
 * chuyen huong sang khung nhi phan chi la doi mot con tro.
 * ========================================================================== */

static void console_write(const char *line)
{
    dbg_println(line);
}

static uint16_t console_free(void)
{
    return dbg_console_tx_free();
}

static const cli_sink_t CLI_SINK_CONSOLE = { console_write, console_free };

static const cli_sink_t *s_sink = &CLI_SINK_CONSOLE;

static void cli_out(const char *line)
{
    s_sink->write(line);
}

/*
 * Dinh dang so nguyen va so hex tu viet, KHONG dung printf - xem ly do o dau
 * dbg_console.h (newlib-nano bo %f, va printf ngon vai KB stack).
 */
static void cli_out_int(const char *label, int32_t value)
{
    char line[80];
    int  w = 0;

    append(line, sizeof(line), &w, label);
    append(line, sizeof(line), &w, " = ");

    uint32_t mag = (uint32_t)value;
    if (value < 0) {
        if (w < (int)sizeof(line) - 1) line[w++] = '-';
        mag = (uint32_t)(-(int64_t)value);
    }
    w += put_uint(line + w, (int)sizeof(line) - w, mag, 0);
    line[w] = 0;

    cli_out(line);
}

static void cli_out_hex(const char *label, uint32_t value, uint8_t digits)
{
    static const char HEX[] = "0123456789ABCDEF";
    char line[80];
    int  w = 0;

    append(line, sizeof(line), &w, label);
    append(line, sizeof(line), &w, " = 0x");

    for (int i = (int)digits - 1; i >= 0 && w < (int)sizeof(line) - 1; i--) {
        line[w++] = HEX[(value >> (i * 4)) & 0xFu];
    }
    line[w] = 0;

    cli_out(line);
}

static void print_param_set_form(uint16_t i)
{
    char line[96];
    int  w = 0;

    append(line, sizeof(line), &w, "set ");
    append(line, sizeof(line), &w, g_param_table[i].name);
    append(line, sizeof(line), &w, "=");
    w += cli_format_float(line + w, (int)sizeof(line) - w, param_get_f32(i));

    cli_out(line);
}

static void print_param_full(uint16_t i)
{
    const param_meta_t *m = &g_param_table[i];
    char line[96];
    int  w = 0;

    append(line, sizeof(line), &w, m->name);
    append(line, sizeof(line), &w, " = ");
    w += cli_format_float(line + w, (int)sizeof(line) - w, param_get_f32(i));

    append(line, sizeof(line), &w, "  [");
    w += cli_format_float(line + w, (int)sizeof(line) - w, m->min);
    append(line, sizeof(line), &w, " .. ");
    w += cli_format_float(line + w, (int)sizeof(line) - w, m->max);
    append(line, sizeof(line), &w, "] mac dinh ");
    w += cli_format_float(line + w, (int)sizeof(line) - w, m->def);

    if (m->flags & PARAM_FLAG_REBOOT) append(line, sizeof(line), &w, " (reboot)");
    if (m->flags & PARAM_FLAG_DANGER) append(line, sizeof(line), &w, " (!)");
    if (param_is_modified(i))         append(line, sizeof(line), &w, " *");

    cli_out(line);
}

/** Tiếp tục lệnh liệt kê đang dở. Chỉ in khi bộ đệm console còn chỗ. */
static void list_pump(void)
{
    while (s_list_mode != LIST_NONE && s_list_index < g_param_count) {

        if (s_sink->free_space() < CLI_LINE_RESERVE) {
            return;   /* hết chỗ, in tiếp ở lần gọi sau */
        }

        const uint16_t i = s_list_index++;

        switch (s_list_mode) {
        case LIST_GET:
            if (s_list_filter[0] == '\0' ||
                strncmp(g_param_table[i].name, s_list_filter,
                        strlen(s_list_filter)) == 0) {
                print_param_full(i);
            }
            break;

        case LIST_DUMP:
            print_param_set_form(i);
            break;

        case LIST_DIFF:
            if (param_is_modified(i)) {
                print_param_set_form(i);
            }
            break;

        default:
            break;
        }
    }

    if (s_list_mode != LIST_NONE && s_list_index >= g_param_count) {
        s_list_mode = LIST_NONE;
        if (s_sink->free_space() >= CLI_LINE_RESERVE) {
            cli_out("# het");
        }
    }
}

static void list_start(cli_list_mode_t mode, const char *filter)
{
    s_list_mode  = mode;
    s_list_index = 0;
    s_list_filter[0] = '\0';

    if (filter != NULL) {
        strncpy(s_list_filter, filter, sizeof(s_list_filter) - 1u);
        s_list_filter[sizeof(s_list_filter) - 1u] = '\0';
    }
    list_pump();
}

/* ==========================================================================
 * Các lệnh
 * ========================================================================== */

static void cmd_help(void)
{
    cli_out("Lenh:");
    cli_out("  help                 danh sach nay");
    cli_out("  version              board, firmware, table_crc");
    cli_out("  status               arm, cam bien, vong lap");
    cli_out("  get [tien_to]        in tham so khop tien to");
    cli_out("  set <ten>=<gia_tri>  dat gia tri, in lai so THUC SU nhan");
    cli_out("  dump                 in tat ca dang 'set ten=gia_tri'");
    cli_out("  diff                 chi in tham so khac mac dinh");
    cli_out("  defaults             nap lai mac dinh (CHUA ghi flash)");
    cli_out("  save                 ghi flash");
    cli_out("  mode [so]            doi che do in cua console, 0 = tat");
    cli_out("  yawzero              chot moc do troi yaw (xem o mode 16)");
    cli_out("  port [uart|usb|here]  doi duong telemetry; here = duong vua gui lenh");
    cli_out("  flash [info|test|erase|dump|sim <n>]  log tren QSPI W25Q64");
}

static void cmd_version(void)
{
    cli_out("board " FC_BOARD_NAME);
    cli_out_int("fw major", FC_FIRMWARE_VERSION_MAJOR);
    cli_out_int("fw minor", FC_FIRMWARE_VERSION_MINOR);
    cli_out_int("fw patch", FC_FIRMWARE_VERSION_PATCH);
    cli_out_int("so tham so", (int32_t)g_param_count);
    cli_out_hex("table_crc", param_table_crc32(), 8);
    cli_out_int("seq da luu", (int32_t)param_store_seq());
}

static void cmd_status(void)
{
    cli_out(g_fc.mode == FC_MODE_ARMED ? "mode ARMED" : "mode DISARMED");
    cli_out(arming_block_reason());
    cli_out_hex("error_flags", g_fc.sys.error_flags, 8);
    cli_out_hex("sensor_health", g_fc.sys.sensor_health, 4);
    cli_out_int("loop_time_us", (int32_t)g_fc.sys.loop_time_us);
    cli_out_int("loop_max_us", (int32_t)g_fc.sys.loop_time_max_us);
    cli_out_int("loop_overruns", (int32_t)g_fc.sys.loop_overruns);

    /*
     * He so PID DANG CHAY, doc tu g_fc chu khong phai tu bang tham so.
     *
     * Vi sao in ca hai nguon: ctrl_rate.c giu mot BAN SAO cua he so (vong
     * nong duyet ba truc bang chi so mang). Neu duong dong bo hong thi `get
     * rate_pid_roll` van in ra so moi trong khi may bay bay theo so cu - va
     * khong co cach nao nhin thay dieu do tu ben ngoai.
     *
     * Hai cot nay PHAI trung nhau. Lech la dau hieu fc_params_apply() khong
     * duoc goi o mot duong ghi nao do.
     */
    {
        const pid_gains_t *g = &g_fc.ctrl.rate_pid[AXIS_ROLL].gains;
        char line[96];
        int  w = 0;

        append(line, sizeof(line), &w, "PID roll dang chay: kp=");
        w += cli_format_float(line + w, (int)sizeof(line) - w, g->kp);
        append(line, sizeof(line), &w, " ki=");
        w += cli_format_float(line + w, (int)sizeof(line) - w, g->ki);
        append(line, sizeof(line), &w, " kd=");
        w += cli_format_float(line + w, (int)sizeof(line) - w, g->kd);
        cli_out(line);
    }
}

static bool cmd_set(char *args)
{
    char *eq = strchr(args, '=');
    if (eq == NULL) {
        cli_out("ERR: dang dung la  set ten=gia_tri");
        return true;
    }
    *eq = '\0';

    /* Bỏ khoảng trắng bám quanh tên. */
    char *name = args;
    while (*name == ' ') name++;
    char *end = name + strlen(name);
    while (end > name && end[-1] == ' ') *--end = '\0';

    const uint16_t i = param_find(name);
    if (i == PARAM_INDEX_NONE) {
        cli_out("ERR: khong co tham so ten do");
        return true;
    }

    /*
     * strtof xử lý được cả "1.2e-8" lẫn "0.001". Đây KHÔNG phải printf nên
     * không dính hạn chế %f của newlib-nano.
     */
    char       *tail  = NULL;
    const float value = strtof(eq + 1, &tail);

    if (tail == eq + 1) {
        cli_out("ERR: khong doc duoc so");
        return true;
    }

    if (!param_set_f32(i, value)) {
        cli_out("ERR: tu choi (chi doc, hoac gia tri khong hop le)");
        return true;
    }

    /*
     * Day gia tri moi vao cac cau truc dan xuat. Khong co dong nay thi he so
     * PID doi trong bang ma vong dieu khien van chay he so cu - CLI in ra so
     * moi, may bay bay theo so cu.
     */
    fc_params_apply();

    /*
     * In lại giá trị firmware THỰC SỰ đang giữ, không phải cái vừa gõ. Bị
     * kẹp về min/max hay bị làm tròn ở trường số nguyên đều lộ ra ngay đây.
     */
    print_param_full(i);

    if (g_param_table[i].flags & PARAM_FLAG_REBOOT) {
        cli_out("# can khoi dong lai moi co tac dung");
    }
    return true;
}

/*
 * Doi che do in cua console.
 *
 * Truoc day doi DBG_MODE_* phai sua main.c roi build lai va nap lai. Voi CLI
 * thi no thanh mot dong lenh - va quan trong hon: 'mode 0' tat han luong in
 * dinh ky, nen ket qua cua cac lenh khac khong bi cac dong log chen vao.
 */
/* ==========================================================================
 * Flash NOR trên QUADSPI
 *
 * Mọi thao tác ở đây đều CHẶN vòng lặp chính: xoá một sector tới 400 ms, xoá
 * cả chip tới 100 giây. Nên tất cả đều chặn khi đang ARM — mất khung DShot
 * lâu như vậy thì ESC coi như mất tín hiệu và cắt motor.
 * ========================================================================== */

/* ==========================================================================
 * Trút log ra ngoài
 *
 * Đi theo đúng khuôn list_pump(): mỗi lần cli_update() gọi thì in tiếp vài
 * dòng, và dừng lại ngay khi đầu ra hết chỗ. Nhờ vậy 8 MB log không chặn
 * vòng lặp chính lấy một mili giây nào.
 *
 * Vì nó đi qua sink chứ không đi thẳng UART, lệnh "port usb" cho ta đường
 * USB CDC nhanh gấp mười mà không phải viết thêm dòng nào.
 * ========================================================================== */

static bool     s_dump_on;
static uint32_t s_dump_addr;
static uint32_t s_dump_end;

/** Chỗ phải chừa cho một dòng CSV. CLI_LINE_RESERVE 128 là không đủ. */
#define DUMP_RESERVE  (LOG_RECORD_CSV_MAX + 32u)

static void dump_stop(const char *why)
{
    s_dump_on = false;
    cli_out(why);
}

static void dump_pump(void)
{
    char    line[LOG_RECORD_CSV_MAX];
    uint8_t raw[LOG_RECORD_BYTES];

    while (s_dump_on) {

        if (s_sink->free_space() < DUMP_RESERVE) {
            return;                     /* hết chỗ, in tiếp ở lần gọi sau */
        }
        if (s_dump_addr >= s_dump_end) {
            dump_stop("# het");
            return;
        }
        if (!qspi_flash_read(s_dump_addr, raw, sizeof(raw))) {
            dump_stop("ERR: doc flash that bai");
            return;
        }

        bool erased = true;

        for (uint32_t i = 0; i < sizeof(raw); i++) {
            if (raw[i] != 0xFFu) {
                erased = false;
                break;
            }
        }

        if (erased) {
            /*
             * 0xFF là phần đệm cuối một chuyến bay. Nhảy tới ranh giới trang
             * kế tiếp, chỗ chuyến sau bắt đầu. Nhưng nếu đang ĐỨNG ở đầu
             * trang mà vẫn toàn 0xFF thì đây là vùng chưa ghi — hết dữ liệu.
             */
            if ((s_dump_addr % QSPI_FLASH_PAGE_BYTES) == 0u) {
                dump_stop("# het");
                return;
            }
            s_dump_addr = (s_dump_addr + QSPI_FLASH_PAGE_BYTES) &
                          ~(QSPI_FLASH_PAGE_BYTES - 1u);
            continue;
        }

        {
            const flashlog_hdr_t *h = (const flashlog_hdr_t *)(const void *)raw;

            if (h->magic == FLASHLOG_MAGIC) {
                cli_out("#");
                cli_out_int("# chuyen bay moi, moc boot_ms", (int32_t)h->boot_ms);
                cli_out_int("#   nhip Hz", (int32_t)h->rate_hz);
                s_dump_addr += LOG_RECORD_BYTES;
                continue;
            }
        }

        {
            const int n = log_record_to_csv(line,
                                            (const bb_record_t *)(const void *)raw);

            /* Bỏ CR LF ở cuối: sink tự xuống dòng. */
            line[n - 2] = 0;
            cli_out(line);
        }
        s_dump_addr += LOG_RECORD_BYTES;
    }
}

/** In dòng tiêu đề CSV, bỏ CR LF vì sink tự xuống dòng. */
static void dump_header(void)
{
    /* Kich co theo LOG_RECORD_CSV_MAX chu khong phai mot so cung: them cot
       vao ban ghi la tieu de dai ra, ma cat cut thi khong ai nhan ra ngay. */
    char     h[LOG_RECORD_CSV_MAX];
    uint32_t i = 0;

    while (g_log_csv_header[i] != 0 && i < (sizeof(h) - 1u)) {
        h[i] = g_log_csv_header[i];
        i++;
    }
    if (i >= 2u) {
        i -= 2u;
    }
    h[i] = 0;
    cli_out(h);
}

static void flash_selftest(void)
{
    /*
     * Đệm để static chứ không trên ngăn xếp: 512 byte là nhiều so với khung
     * ngăn xếp thông thường, mà ngăn xếp chính nằm ở DTCMRAM dùng chung.
     */
    static uint8_t wr[QSPI_FLASH_PAGE_BYTES];
    static uint8_t rd[QSPI_FLASH_PAGE_BYTES];

    /* Sector CUỐI chip: vùng log sau này mọc từ địa chỉ 0 lên, không đụng nhau. */
    const uint32_t addr = qspi_flash_bytes() - QSPI_FLASH_SECTOR_BYTES;
    uint32_t       t0;
    uint32_t       t_erase, t_prog, t_read;

    t0 = micros();
    if (!qspi_flash_erase_sector(addr)) {
        cli_out("ERR: xoa sector that bai");
        return;
    }
    t_erase = fc_elapsed_us(micros(), t0);

    if (!qspi_flash_read(addr, rd, sizeof(rd))) {
        cli_out("ERR: doc sau khi xoa that bai");
        return;
    }
    for (uint32_t i = 0; i < sizeof(rd); i++) {
        if (rd[i] != 0xFFu) {
            cli_out_int("ERR: sau khi xoa khong phai 0xFF tai byte", (int32_t)i);
            return;
        }
    }

    for (uint32_t i = 0; i < sizeof(wr); i++) {
        wr[i] = (uint8_t)(i ^ 0x5Au);   /* mẫu không đối xứng, lệch một byte là lộ */
    }

    t0 = micros();
    if (!qspi_flash_write_page(addr, wr, sizeof(wr))) {
        cli_out("ERR: ghi trang that bai");
        return;
    }
    t_prog = fc_elapsed_us(micros(), t0);

    t0 = micros();
    if (!qspi_flash_read(addr, rd, sizeof(rd))) {
        cli_out("ERR: doc lai that bai");
        return;
    }
    t_read = fc_elapsed_us(micros(), t0);

    for (uint32_t i = 0; i < sizeof(rd); i++) {
        if (rd[i] != wr[i]) {
            cli_out_int("ERR: doc lai lech tai byte", (int32_t)i);
            return;
        }
    }

    cli_out("flash test: OK - xoa, ghi, doc lai deu khop");
    cli_out_int("  xoa sector 4KB, us", (int32_t)t_erase);
    cli_out_int("  ghi 256 byte,   us", (int32_t)t_prog);
    cli_out_int("  doc 256 byte,   us", (int32_t)t_read);
}

static void cmd_flash(const char *args)
{
    if (qspi_flash_bytes() == 0u) {
        cli_out("ERR: chua nhan ra chip flash QSPI");
        return;
    }

    if (args == NULL || *args == '\0' || strcmp(args, "info") == 0) {
        cli_out_hex("flash jedec", qspi_flash_jedec(), 6);
        cli_out_int("flash dung luong MB",
                    (int32_t)(qspi_flash_bytes() / (1024u * 1024u)));
        cli_out_hex("flash sr1", qspi_flash_sr1(), 2);
        cli_out_hex("flash sr2", qspi_flash_sr2(), 2);
        cli_out(qspi_flash_is_busy() ? "flash: DANG BAN (xoa hoac ghi)"
                                     : "flash: ranh");
        cli_out_int("log da dung KB",
                    (int32_t)(flashlog_used_bytes() / 1024u));
        cli_out_int("log suc chua KB",
                    (int32_t)(flashlog_capacity_bytes() / 1024u));
        cli_out_int("log ban ghi chuyen nay", (int32_t)flashlog_records());
        cli_out_int("log ban ghi bi bo", (int32_t)flashlog_dropped());
        cli_out(flashlog_state_name());
        return;
    }

    if (strcmp(args, "dump") == 0) {
        s_dump_addr = 0;
        s_dump_end  = flashlog_used_bytes();
        if (s_dump_end == 0u) {
            cli_out("# chua co log nao");
            return;
        }
        s_dump_on = true;
        dump_header();
        dump_pump();
        return;
    }

    if (g_fc.motor.armed) {
        cli_out("ERR: dang ARM - thao tac flash chan vong lap toi 400 ms");
        return;
    }

    if (strcmp(args, "test") == 0) {
        flash_selftest();
        return;
    }

    if (strncmp(args, "sim", 3) == 0) {
        const long n = strtol(args + 3, NULL, 10);

        if (n <= 0) {
            cli_out("Dung: flash sim <so ban ghi>");
            return;
        }
        cli_out_int("Dang ghi chuyen bay gia, so ban ghi", (int32_t)n);
        if (flashlog_selftest((uint32_t)n)) {
            cli_out("flash sim: OK");
            cli_out_int("  log da dung KB",
                        (int32_t)(flashlog_used_bytes() / 1024u));
        } else {
            cli_out("ERR: ghi that bai");
            cli_out(flashlog_state_name());
            cli_out_int("  ban ghi bi bo", (int32_t)flashlog_dropped());
        }
        return;
    }

    if (strcmp(args, "erase") == 0) {
        if (!qspi_flash_erase_chip_start()) {
            cli_out("ERR: khong phat duoc lenh xoa");
            return;
        }
        cli_out("Da phat lenh xoa TOAN BO chip. Mat 20-100 giay.");
        cli_out("Go 'flash info' de xem xong chua - khong can cho o day.");
        cli_out("Xoa xong nho go 'flash rescan' de do lai diem cuoi.");
        return;
    }

    if (strcmp(args, "rescan") == 0) {
        /*
         * Do lai diem cuoi cua du lieu tren chip. Can sau khi xoa, vi con tro
         * ghi cua flashlog chi duoc do MOT LAN luc khoi dong.
         */
        if (qspi_flash_is_busy()) {
            cli_out("ERR: chip con dang ban, doi xoa xong da");
            return;
        }
        (void)flashlog_init();
        cli_out_int("log da dung KB", (int32_t)(flashlog_used_bytes() / 1024u));
        cli_out(flashlog_state_name());
        return;
    }

    cli_out("Dung: flash [info|test|erase|rescan|dump|sim <n>]");
}

static void cmd_mode(const char *args)
{
    if (args == NULL || *args == '\0') {
        cli_out_int("mode hien tai", (int32_t)dbg_console_get_mode());
        cli_out("Dat bang 'mode <so>'. 0 = tat han.");
        return;
    }

    char      *tail = NULL;
    const long v    = strtol(args, &tail, 10);

    if (tail == args || v < 0 || v >= (long)DBG_MODE_COUNT) {
        cli_out("ERR: so che do khong hop le");
        return;
    }

    dbg_console_set_mode((dbg_mode_t)v);
    cli_out_int("mode", (int32_t)v);
}

/*
 * Doi duong telemetry giua USART3 (-> ESP32 -> ESP-NOW) va USB CDC.
 *
 * CO Y KHONG tu doi khi cam USB. Tu doi nghe tien nhung no LANG LE CAT duong
 * telemetry xuong tram mat dat ngay giua chuyen bay, chi vi ai do cam cap USB
 * de sac hay de xem log. Doi duong truyen la quyet dinh cua nguoi dung.
 */
static void cmd_port(const char *args)
{
    if (args == NULL || *args == '\0') {
        const tlm_port_type_t p = tlm_port_get();
        cli_out(p == TLM_PORT_USB  ? "port = usb"
              : p == TLM_PORT_UART ? "port = uart"
                                   : "port = none");
        cli_out("Dat bang 'port uart', 'port usb', hoac 'port here'.");
        cli_out_int("usb rx bo (byte)", (int32_t)tlm_port_usb_overruns());
        return;
    }

    if (strcmp(args, "here") == 0) {
        /*
         * Chuyen huong PHAT ve dung duong ma lenh nay vua den.
         *
         * Nho vay phan mem PC chi can gui mot lenh la tu noi duoc, khong phai
         * doan minh dang o cong nao — va khong phai bat nguoi dung go tay
         * tren mot cong khac truoc.
         */
        const tlm_port_type_t p = tlm_port_last_rx();

        if (p == TLM_PORT_NONE) {
            cli_out("ERR: chua biet lenh den tu duong nao");
            return;
        }
        tlm_port_set(p);
        cli_out(p == TLM_PORT_USB ? "port = usb (theo duong lenh vua den)"
                                  : "port = uart (theo duong lenh vua den)");
        return;
    }

    if (strcmp(args, "uart") == 0) {
        tlm_port_set(TLM_PORT_UART);
        cli_out("port = uart (USART3 -> ESP32)");
    } else if (strcmp(args, "usb") == 0) {
        tlm_port_set(TLM_PORT_USB);
        cli_out("port = usb (CDC). Telemetry xuong ESP32 DUNG cho toi khi doi lai.");
    } else {
        cli_out("ERR: chi nhan 'uart', 'usb' hoac 'here'");
    }
}

static void cmd_save(void)
{
    const param_store_result_t res = param_store_save();

    if (res == PARAM_STORE_OK) {
        cli_out_int("Da ghi flash, seq", (int32_t)param_store_seq());
    } else {
        char line[64];
        int  w = 0;
        append(line, sizeof(line), &w, "ERR: khong ghi duoc - ");
        append(line, sizeof(line), &w, param_store_result_name(res));
        cli_out(line);
    }
}

bool cli_execute(const char *line)
{
    return cli_execute_ex(line, NULL);
}

bool cli_output_pending(void)
{
    return s_list_mode != LIST_NONE;
}

bool cli_execute_ex(const char *line, const cli_sink_t *sink)
{
    /*
     * Dat sink TRUOC khi chay lenh: lenh liet ke se ghi nho trang thai in do
     * dang, va list_pump() o nhung lan goi sau phai tra ket qua ve DUNG noi
     * da phat lenh. Doi sink giua chung nghia la nua ban dump di mot duong,
     * nua kia di duong khac.
     */
    s_sink = (sink != NULL) ? sink : &CLI_SINK_CONSOLE;

    char buf[CLI_LINE_MAX];

    strncpy(buf, line, sizeof(buf) - 1u);
    buf[sizeof(buf) - 1u] = '\0';

    /* Bỏ khoảng trắng đầu dòng. */
    char *p = buf;
    while (*p == ' ' || *p == '\t') p++;
    if (*p == '\0' || *p == '#') {
        return true;   /* dòng trống hoặc chú thích: bỏ qua, không báo lỗi */
    }

    /* Tách từ đầu tiên. */
    char *args = p;
    while (*args && *args != ' ' && *args != '\t') args++;
    if (*args) {
        *args++ = '\0';
        while (*args == ' ' || *args == '\t') args++;
    }

    if (strcmp(p, "help") == 0)     { cmd_help();    return true; }
    if (strcmp(p, "version") == 0)  { cmd_version(); return true; }
    if (strcmp(p, "status") == 0)   { cmd_status();  return true; }
    if (strcmp(p, "set") == 0)      { return cmd_set(args); }
    if (strcmp(p, "save") == 0)     { cmd_save();    return true; }
    if (strcmp(p, "mode") == 0)     { cmd_mode(args); return true; }
    if (strcmp(p, "port") == 0)     { cmd_port(args); return true; }
    if (strcmp(p, "flash") == 0)    { cmd_flash(args); return true; }

    if (strcmp(p, "yawzero") == 0) {
        dbg_console_yaw_zero();
        cli_out("Da chot moc yaw. Xem cot d_yaw va do/ph o 'mode 16'.");
        return true;
    }

    if (strcmp(p, "get") == 0)      { list_start(LIST_GET, args);  return true; }
    if (strcmp(p, "dump") == 0)     { list_start(LIST_DUMP, NULL); return true; }
    if (strcmp(p, "diff") == 0)     { list_start(LIST_DIFF, NULL); return true; }

    if (strcmp(p, "defaults") == 0) {
        param_load_defaults();
        fc_params_apply();
        cli_out("Da nap mac dinh. CHUA ghi flash - go 'save' neu muon giu.");
        return true;
    }

    cli_out("ERR: khong hieu lenh, go 'help'");
    return false;
}

/* ==========================================================================
 * Vòng đời
 * ========================================================================== */

void cli_init(UART_HandleTypeDef *huart)
{
    s_uart     = huart;
    s_rx_tail  = 0;
    s_line_len = 0;
    s_overflow = false;
    s_list_mode = LIST_NONE;

    /* Section .dma_buffer là NOLOAD -> startup code không xoá giúp. */
    memset(s_rx_buf, 0, sizeof(s_rx_buf));

    if (s_uart != NULL && s_uart->hdmarx != NULL) {
        HAL_UART_Receive_DMA(s_uart, s_rx_buf, CLI_RX_BUFFER_SIZE);
    }
}

static void feed(uint8_t c)
{
    if (c == '\r' || c == '\n') {
        if (s_overflow) {
            cli_out("ERR: dong qua dai");
            s_overflow = false;
        } else if (s_line_len > 0u) {
            s_line[s_line_len] = '\0';
            (void)cli_execute(s_line);
        }
        s_line_len = 0;
        return;
    }

    /* Backspace, để gõ tay trong terminal dễ chịu hơn. */
    if (c == 0x08u || c == 0x7Fu) {
        if (s_line_len > 0u) s_line_len--;
        return;
    }

    if (c < 0x20u) {
        return;   /* bỏ ký tự điều khiển khác */
    }

    if (s_line_len >= CLI_LINE_MAX - 1u) {
        s_overflow = true;   /* nuốt phần còn lại tới hết dòng */
        return;
    }
    s_line[s_line_len++] = (char)c;
}

void cli_update(void)
{
    /* In tiếp phần dở TRƯỚC khi nhận lệnh mới: lệnh mới sẽ ghi đè trạng thái. */
    list_pump();
    dump_pump();

    if (s_uart == NULL || s_uart->hdmarx == NULL) {
        return;
    }

    /*
     * DMA vòng tròn đếm lùi: vị trí ghi hiện tại suy ra từ bộ đếm còn lại.
     * Cùng cách làm với tlm_port_read().
     */
    const uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(s_uart->hdmarx);
    const uint16_t head      = (uint16_t)(CLI_RX_BUFFER_SIZE - remaining);

    while (s_rx_tail != head) {
        feed(s_rx_buf[s_rx_tail]);
        s_rx_tail = (uint16_t)((s_rx_tail + 1u) % CLI_RX_BUFFER_SIZE);

        /*
         * Một lệnh liệt kê vừa khởi động thì dừng rút byte ở đây. Phần chưa
         * lấy vẫn nằm nguyên trong đệm DMA. Không có chốt này thì dán một
         * lúc mười dòng `dump` sẽ khiến mỗi dòng huỷ bản in của dòng trước.
         */
        if (s_list_mode != LIST_NONE || s_dump_on) {
            break;
        }
    }
}
