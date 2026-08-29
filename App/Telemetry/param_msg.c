/**
 * @file    param_msg.c
 * @brief   Lớp giao thức của hệ tham số. Xem param_msg.h.
 */
#include "param_msg.h"
#include "tlm_stream.h"
#include "tlm_port.h"
#include "tlm_messages.h"
#include "param_table.h"
#include "param_apply.h"
#include "cli.h"
#include "dshot.h"
#include "fc_state.h"
#include "blackbox.h"

#include <string.h>

/*
 * Tên tham số đi nguyên vẹn qua đường truyền. Hai hằng số này BẮT BUỘC bằng
 * nhau; chúng nằm ở hai file khác nhau vì tlm_messages.h được chép sang phía
 * máy tính, nơi không có param_table.h. Lệch nhau thì tên bị cắt cụt và app
 * nhận về một cái tên khác — bắt ngay ở đây.
 */
_Static_assert(PARAM_NAME_MAX == TLM_PARAM_NAME_MAX,
               "PARAM_NAME_MAX va TLM_PARAM_NAME_MAX phai bang nhau");

/*
 * KHOA BO CUC DAY DAY.
 *
 * Phia may tinh khai bao lai cac struct nay bang tay (Python hom nay, C# sau
 * nay). Trinh bien dich khong the kiem tra ben kia, nhung no kiem duoc ben
 * NAY - va mot khi con so o day bi khoa thi bat ky thay doi vo tinh nao
 * trong tlm_messages.h cung lam build hong ngay, thay vi im lang doi bo cuc
 * roi de app doc ra so vo nghia mot cach tu tin.
 *
 * Doi mot trong nhung con so nay = doi giao thuc = PHAI tang
 * TLM_PROTOCOL_VERSION.
 */
_Static_assert(sizeof(tlm_param_value_t)   == 50u, "bo cuc PARAM_VALUE doi");
_Static_assert(sizeof(tlm_ack_t)           ==  4u, "bo cuc ACK doi");
_Static_assert(sizeof(tlm_fc_info_t)       == 32u, "bo cuc FC_INFO doi");
_Static_assert(sizeof(tlm_cli_line_t)      == 63u, "bo cuc CLI_LINE doi");
_Static_assert(sizeof(tlm_cmd_param_set_t) ==  8u, "bo cuc CMD_PARAM_SET doi");
_Static_assert(sizeof(tlm_cmd_param_read_t) == 2u, "bo cuc CMD_PARAM_READ doi");
_Static_assert(sizeof(tlm_cmd_motor_test_t) == 4u, "bo cuc CMD_MOTOR_TEST doi");

/* Moi ban tin phai lot trong mot khung. */
_Static_assert(sizeof(tlm_param_value_t) <= TLM_MAX_PAYLOAD, "PARAM_VALUE qua dai");
_Static_assert(sizeof(tlm_fc_info_t)     <= TLM_MAX_PAYLOAD, "FC_INFO qua dai");
_Static_assert(sizeof(tlm_cli_line_t)    <= TLM_MAX_PAYLOAD, "CLI_LINE qua dai");

/* ==========================================================================
 * Trả lời
 * ========================================================================== */

static void ack(uint8_t cmd_id, uint8_t result, uint16_t detail)
{
    const tlm_ack_t a = {
        .cmd_id = cmd_id,
        .result = result,
        .detail = detail,
    };
    (void)tlm_stream_send_payload(TLM_MSG_ACK, &a, sizeof(a));
}

/* ==========================================================================
 * Phát mô tả tham số
 * ========================================================================== */

/** Chừa dư trước khi phát một gói. Gói PARAM_VALUE là 50 + 7 byte khung. */
#define PARAM_MSG_RESERVE  128u

static bool send_param(uint16_t index)
{
    if (index >= g_param_count) {
        return false;
    }

    const param_meta_t *m = &g_param_table[index];
    tlm_param_value_t   v;

    memset(&v, 0, sizeof(v));
    v.index = index;
    v.count = g_param_count;
    v.type  = m->type;
    v.flags = m->flags;
    v.value = param_get_f32(index);
    v.min   = m->min;
    v.max   = m->max;
    v.def   = m->def;

    /*
     * strncpy chứ không strcpy: param_table.c đã có _Static_assert bảo đảm
     * mọi tên vừa trong PARAM_NAME_MAX, nhưng dựa vào một bất biến ở file
     * khác cho một phép chép vào bộ đệm cố định là thói quen xấu.
     */
    strncpy(v.name, m->name, sizeof(v.name) - 1u);

    return tlm_stream_send_payload(TLM_MSG_PARAM_VALUE, &v, sizeof(v));
}

/*
 * Máy trạng thái bơm bảng.
 *
 * s_send_next == g_param_count nghĩa là đang nghỉ. Không dùng cờ riêng để
 * khỏi có hai nguồn sự thật cho cùng một trạng thái.
 */
static uint16_t s_send_next;

void param_msg_init(void)
{
    s_send_next = g_param_count;
}

/* ==========================================================================
 * Quay thử motor
 * ========================================================================== */

static bool handle_motor_test(const tlm_parser_t *p)
{
    if (p->len < sizeof(tlm_cmd_motor_test_t)) {
        ack(TLM_MSG_CMD_MOTOR_TEST, TLM_ACK_ERR_LENGTH, 0);
        return true;
    }

    tlm_cmd_motor_test_t cmd;
    memcpy(&cmd, p->payload, sizeof(cmd));

    if (cmd.motor == TLM_MOTOR_TEST_STOP) {
        dshot_motor_test_stop();
        ack(TLM_MSG_CMD_MOTOR_TEST, TLM_ACK_OK, 0);
        return true;
    }

    /*
     * Không kiểm ARM ở đây. dshot_motor_test_start() tự từ chối khi đang arm,
     * và đó là chốt đúng chỗ — nó nằm cạnh phần cứng nên MỌI đường gọi đều đi
     * qua, kể cả CLI và code viết sau này. Nhân bản phép kiểm lên đây chỉ tạo
     * thêm một chỗ để quên cập nhật.
     *
     * Nhưng vẫn phải phân biệt được hai lý do thất bại để app báo cho đúng.
     */
    const float throttle = (float)cmd.throttle_pct * 0.01f;

    if (!dshot_motor_test_start(cmd.motor, throttle, cmd.timeout_ms)) {
        const bool armed = (g_fc.mode == FC_MODE_ARMED) || g_fc.motor.armed;
        ack(TLM_MSG_CMD_MOTOR_TEST,
            armed ? TLM_ACK_ERR_ARMED : TLM_ACK_ERR_RANGE, cmd.motor);
        return true;
    }

    ack(TLM_MSG_CMD_MOTOR_TEST, TLM_ACK_OK, cmd.motor);
    return true;
}

bool param_msg_motor_test_active(void)
{
    return dshot_motor_test_active() >= 0;
}

/* ==========================================================================
 * CLI qua đường nhị phân
 *
 * Dùng lại NGUYÊN VẸN bộ mã lệnh của console chữ, chỉ đổi nơi nhận đầu ra.
 * Nhờ vậy app và PuTTY luôn cho cùng một kết quả — và khi app cư xử lạ thì
 * gõ tay đúng lệnh đó vào PuTTY là biết lỗi nằm ở firmware hay ở app.
 * ========================================================================== */

static uint16_t s_cli_line_seq;

static void cli_binary_write(const char *line)
{
    tlm_cli_line_t msg;
    memset(&msg, 0, sizeof(msg));

    msg.seq   = s_cli_line_seq++;
    msg.flags = 0;   /* cờ "dòng cuối" do param_msg_update() đặt, xem dưới */

    size_t n = strlen(line);
    if (n > sizeof(msg.text)) {
        n = sizeof(msg.text);
    }
    memcpy(msg.text, line, n);

    /* Chỉ gửi đúng phần chữ có thật, không gửi thừa byte 0. */
    (void)tlm_stream_send_payload(TLM_MSG_CLI_LINE, &msg,
                                  (uint8_t)(3u + n));
}

static uint16_t cli_binary_free(void)
{
    return tlm_port_tx_free();
}

static const cli_sink_t CLI_SINK_BINARY = { cli_binary_write, cli_binary_free };

/** true khi lần trả lời gần nhất đã phát ít nhất một dòng và còn đang dở. */
static bool s_cli_active;

static bool handle_cli(const tlm_parser_t *p)
{
    char line[sizeof(((tlm_cmd_cli_t *)0)->text) + 1u];

    if (p->len == 0u) {
        ack(TLM_MSG_CMD_CLI, TLM_ACK_ERR_LENGTH, 0);
        return true;
    }

    size_t n = p->len;
    if (n > sizeof(line) - 1u) {
        n = sizeof(line) - 1u;
    }
    memcpy(line, p->payload, n);
    line[n] = '\0';

    s_cli_line_seq = 0;
    s_cli_active   = true;

    const bool ok = cli_execute_ex(line, &CLI_SINK_BINARY);

    /*
     * Lệnh liệt kê (dump/diff/get) chưa xong ở đây — nó còn được bơm dần
     * trong param_msg_update(). Chỉ đóng phiên khi thật sự hết dòng.
     */
    if (!cli_output_pending()) {
        cli_binary_write("");            /* dòng rỗng đánh dấu kết thúc */
        s_cli_active = false;
    }

    ack(TLM_MSG_CMD_CLI, ok ? TLM_ACK_OK : TLM_ACK_ERR_UNKNOWN, s_cli_line_seq);
    return true;
}

/* ==========================================================================
 * Nhận dạng mạch bay
 * ========================================================================== */

bool param_msg_send_fc_info(void)
{
    tlm_fc_info_t info;
    memset(&info, 0, sizeof(info));

    strncpy(info.board, FC_BOARD_NAME, sizeof(info.board) - 1u);
    info.fw_major         = FC_FIRMWARE_VERSION_MAJOR;
    info.fw_minor         = FC_FIRMWARE_VERSION_MINOR;
    info.fw_patch         = FC_FIRMWARE_VERSION_PATCH;
    info.protocol_version = TLM_PROTOCOL_VERSION;
    info.param_count      = g_param_count;
    info.param_table_crc  = param_table_crc32();

    info.capabilities = TLM_CAP_PARAMS | TLM_CAP_CLI | TLM_CAP_MOTOR_TEST;

#if BB_ENABLE
    info.capabilities |= TLM_CAP_BLACKBOX;
#endif
    if (g_params.imu2_enable) {
        info.capabilities |= TLM_CAP_IMU2;
    }
    if (g_params.mag_source != 0u) {
        info.capabilities |= TLM_CAP_MAG;
    }

    return tlm_stream_send_payload(TLM_MSG_FC_INFO, &info, sizeof(info));
}

/* ==========================================================================
 * Đọc / ghi tham số
 * ========================================================================== */

static bool handle_param_set(const tlm_parser_t *p)
{
    if (p->len < sizeof(tlm_cmd_param_set_t)) {
        ack(TLM_MSG_CMD_PARAM_SET, TLM_ACK_ERR_LENGTH, 0);
        return true;
    }

    tlm_cmd_param_set_t cmd;
    memcpy(&cmd, p->payload, sizeof(cmd));

    if (cmd.index >= g_param_count) {
        ack(TLM_MSG_CMD_PARAM_SET, TLM_ACK_ERR_RANGE, cmd.index);
        return true;
    }

    /*
     * Đối chiếu kiểu trước khi ghi.
     *
     * App có thể dùng bản bảng đã nhớ từ phiên trước trong khi firmware đã
     * đổi — lúc đó chỉ số trỏ sang một tham số hoàn toàn khác. Kiểm kiểu bắt
     * được phần lớn trường hợp đó, và rẻ hơn nhiều so với hậu quả của việc
     * ghi nhầm một hệ số PID vào một cờ đảo chiều.
     */
    if (cmd.type != g_param_table[cmd.index].type) {
        ack(TLM_MSG_CMD_PARAM_SET, TLM_ACK_ERR_RANGE, cmd.index);
        return true;
    }

    if (g_param_table[cmd.index].flags & PARAM_FLAG_READONLY) {
        ack(TLM_MSG_CMD_PARAM_SET, TLM_ACK_ERR_READONLY, cmd.index);
        return true;
    }

    /*
     * Không cho đổi tham số khi đang bay. Cùng nguyên tắc với handle_action()
     * trong tlm_stream.c: một hệ số PID đổi giữa chừng có thể làm mất kiểm
     * soát ngay lập tức.
     */
    if (g_fc.mode == FC_MODE_ARMED) {
        ack(TLM_MSG_CMD_PARAM_SET, TLM_ACK_ERR_ARMED, cmd.index);
        return true;
    }

    if (!param_set_f32(cmd.index, cmd.value)) {
        ack(TLM_MSG_CMD_PARAM_SET, TLM_ACK_ERR_RANGE, cmd.index);
        return true;
    }

    fc_params_apply();

    /*
     * Trả về giá trị firmware THỰC SỰ giữ, không phải giá trị app gửi. Bị kẹp
     * min/max hay bị làm tròn ở trường số nguyên đều lộ ra ngay — app hiển
     * thị con số này chứ không phải con số người dùng vừa gõ.
     */
    (void)send_param(cmd.index);
    ack(TLM_MSG_CMD_PARAM_SET, TLM_ACK_OK, cmd.index);
    return true;
}

static bool handle_param_read(const tlm_parser_t *p)
{
    if (p->len < sizeof(tlm_cmd_param_read_t)) {
        ack(TLM_MSG_CMD_PARAM_READ, TLM_ACK_ERR_LENGTH, 0);
        return true;
    }

    tlm_cmd_param_read_t cmd;
    memcpy(&cmd, p->payload, sizeof(cmd));

    if (!send_param(cmd.index)) {
        ack(TLM_MSG_CMD_PARAM_READ, TLM_ACK_ERR_RANGE, cmd.index);
    }
    return true;
}

/* ==========================================================================
 * Điều phối
 * ========================================================================== */

bool param_msg_handle(const tlm_parser_t *p)
{
    switch (p->id) {

    case TLM_MSG_CMD_PARAM_REQ_LIST:
        /*
         * Bắt đầu lại từ đầu. Yêu cầu mới trong lúc đang phát dở là chuyện
         * bình thường — app khởi động lại, hoặc người dùng bấm "đọc lại" —
         * nên nó ghi đè tiến độ cũ thay vì bị từ chối.
         */
        s_send_next = 0;
        ack(TLM_MSG_CMD_PARAM_REQ_LIST, TLM_ACK_OK, g_param_count);
        return true;

    case TLM_MSG_CMD_PARAM_READ:  return handle_param_read(p);
    case TLM_MSG_CMD_PARAM_SET:   return handle_param_set(p);
    case TLM_MSG_CMD_MOTOR_TEST:  return handle_motor_test(p);
    case TLM_MSG_CMD_CLI:         return handle_cli(p);

    case TLM_MSG_CMD_FC_INFO_REQ:
        (void)param_msg_send_fc_info();
        return true;

    default:
        return false;
    }
}

void param_msg_update(uint32_t now_ms)
{
    (void)now_ms;

    /*
     * Bơm bảng tham số. Chỉ phát khi bộ đệm gửi còn dư — đẩy hết một lượt sẽ
     * làm tràn đệm và nuốt mất heartbeat, tức app đang tải cấu hình thì lại
     * tưởng mất kết nối.
     */
    while (s_send_next < g_param_count &&
           tlm_port_tx_free() > PARAM_MSG_RESERVE) {

        if (!send_param(s_send_next)) {
            break;   /* đệm đầy giữa chừng: giữ nguyên chỉ số, thử lại sau */
        }
        s_send_next++;
    }

    /*
     * Bơm nốt phần đầu ra CLI còn dở (dump/diff/get). cli_update() cũng gọi
     * list_pump(), nhưng nó chạy trên sink của lần gọi gần nhất — mà sink đó
     * chính là sink nhị phân khi lệnh đến từ app. Ở đây chỉ cần phát hiện
     * lúc nó vừa hết để đóng phiên bằng một dòng rỗng.
     */
    if (s_cli_active && !cli_output_pending()) {
        cli_binary_write("");
        s_cli_active = false;
    }
}
