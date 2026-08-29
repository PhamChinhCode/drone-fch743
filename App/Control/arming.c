/**
 * @file    arming.c
 * @brief   Hiện thực bộ giám sát arm/disarm.
 */

#include "arming.h"
#include "fc_state.h"
#include "param_table.h"
#include "fc_time.h"

/* ==========================================================================
 * Biến nội bộ
 * ========================================================================== */

static arming_state_t        s_state      = ARMING_LOCKED;
static arming_disarm_cause_t s_last_cause = DISARM_CAUSE_BOOT;

static bool     s_switch_stable;      /* trạng thái công tắc đã lọc nhiễu  */
static bool     s_switch_pending;     /* giá trị vừa đọc, đang chờ đủ thời gian */
static uint32_t s_switch_change_ms;   /* lúc giá trị thô đổi lần gần nhất   */

static uint32_t s_arm_count;
static uint32_t s_disarm_count;

/* ==========================================================================
 * Đọc công tắc
 * ========================================================================== */

/**
 * Đổi giá trị thô của kênh arm thành true/false, có vùng trễ.
 *
 * Giá trị rơi vào khoảng giữa hai ngưỡng thì giữ nguyên trạng thái cũ. Nhờ
 * vậy công tắc ba nấc có nấc giữa ở 992 được coi là OFF một cách dứt khoát,
 * và nhiễu vài chục đơn vị quanh ngưỡng không gây arm/disarm liên tục.
 */
static bool read_switch_raw(bool previous)
{
    const uint16_t raw = g_fc.rc.channel_raw[g_params.arm_switch_channel];

    if (raw >= g_params.arm_switch_on_threshold)  { return true;  }
    if (raw <= g_params.arm_switch_off_threshold) { return false; }
    return previous;
}

/**
 * Lọc nhiễu công tắc: giá trị mới chỉ được công nhận sau khi giữ nguyên đủ
 * arm_hold_time_ms. Chạy mọi vòng lặp, kể cả lúc mất sóng, để trạng thái
 * công tắc luôn bám sát dữ liệu mới nhất.
 */
static void debounce_switch(uint32_t now_ms)
{
    const bool raw = read_switch_raw(s_switch_stable);

    if (raw != s_switch_pending) {
        s_switch_pending   = raw;
        s_switch_change_ms = now_ms;
        return;
    }

    if (raw != s_switch_stable &&
        fc_elapsed_ms(now_ms, s_switch_change_ms) >= g_params.arm_hold_time_ms) {
        s_switch_stable = raw;
    }
}

/** Đường điều khiển còn dùng được hay không. */
static bool rc_link_ok(void)
{
    return g_fc.rc.healthy && !g_fc.rc.failsafe;
}

/* ==========================================================================
 * Chuyển trạng thái
 * ========================================================================== */

static void do_disarm(arming_disarm_cause_t cause)
{
    (void)fc_state_set_mode(FC_MODE_DISARMED);
    s_disarm_count++;
    s_last_cause = cause;
}

/* ==========================================================================
 * API
 * ========================================================================== */

void arming_init(void)
{
    s_state      = ARMING_LOCKED;
    s_last_cause = DISARM_CAUSE_BOOT;

    /*
     * Khởi tạo là ON, KHÔNG phải OFF. Nếu đặt false thì ngay vòng lặp đầu
     * tiên máy trạng thái thấy "công tắc đang OFF" và nhả khoá LOCKED, dù
     * chưa hề có khung RC nào để biết công tắc thật đang ở đâu. Đặt true
     * buộc phải có một khung RC thật xác nhận công tắc OFF mới nhả khoá.
     */
    s_switch_stable    = true;
    s_switch_pending   = true;
    s_switch_change_ms = 0;

    s_arm_count    = 0;
    s_disarm_count = 0;
}

void arming_update(uint32_t now_ms)
{
    debounce_switch(now_ms);

    /* Cập nhật arm_block_flags cho console và cho fc_state_set_mode(). */
    const bool conditions_ok = fc_state_check_arm();

    /*
     * Mất sóng xử lý trước mọi thứ khác, và luôn rơi về LOCKED. Bắt lại được
     * sóng trong khi công tắc vẫn đang ON thì KHÔNG được tự arm lại.
     */
    if (!rc_link_ok()) {
        if (s_state == ARMING_ARMED) {
            /*
             * Đi qua FAILSAFE rồi mới tới DISARMED. Bảng chuyển trạng thái
             * cho phép ARMED -> DISARMED thẳng, nhưng ghé qua FAILSAFE khiến
             * lý do rơi khỏi trạng thái bay hiện ra trong log thay vì biến
             * mất lặng lẽ thành một lần disarm bình thường.
             */
            (void)fc_state_set_mode(FC_MODE_FAILSAFE);
            do_disarm(DISARM_CAUSE_FAILSAFE);
        }
        s_state = ARMING_LOCKED;
        g_fc.sys.arm_block_flags |= ARM_BLOCK_SWITCH;
        return;
    }

    switch (s_state) {

    case ARMING_LOCKED:
        /* Lối ra duy nhất: công tắc thực sự về OFF. */
        if (!s_switch_stable) {
            s_state = ARMING_SAFE;
        }
        break;

    case ARMING_SAFE:
        if (s_switch_stable) {
            if (conditions_ok && fc_state_set_mode(FC_MODE_ARMED)) {
                s_state = ARMING_ARMED;
                s_arm_count++;
            } else {
                /*
                 * Bật công tắc lúc chưa đủ điều kiện. Khoá lại thay vì chờ:
                 * nếu chỉ chờ, máy bay sẽ tự arm đúng vào khoảnh khắc điều
                 * kiện cuối cùng vừa thoả — chẳng hạn ngay khi gyro hiệu
                 * chuẩn xong — mà người lái không hề ra lệnh.
                 */
                s_state      = ARMING_LOCKED;
                s_last_cause = DISARM_CAUSE_BLOCKED;
            }
        }
        break;

    case ARMING_ARMED:
        if (!s_switch_stable) {
            do_disarm(DISARM_CAUSE_SWITCH);
            /* Về SAFE chứ không phải LOCKED: bật lại công tắc là arm ngay. */
            s_state = ARMING_SAFE;
        }
        break;

    default:
        s_state = ARMING_LOCKED;
        break;
    }

    if (s_state == ARMING_LOCKED) {
        g_fc.sys.arm_block_flags |= ARM_BLOCK_SWITCH;
    }
}

arming_state_t        arming_get_state(void)        { return s_state; }
arming_disarm_cause_t arming_last_disarm_cause(void){ return s_last_cause; }
bool                  arming_switch_on(void)        { return s_switch_stable; }
uint32_t              arming_arm_count(void)        { return s_arm_count; }
uint32_t              arming_disarm_count(void)     { return s_disarm_count; }

/* ==========================================================================
 * Chuỗi mô tả
 * ========================================================================== */

const char *arming_state_name(arming_state_t state)
{
    switch (state) {
    case ARMING_LOCKED: return "LOCKED";
    case ARMING_SAFE:   return "SAFE";
    case ARMING_ARMED:  return "ARMED";
    default:            return "?";
    }
}

const char *arming_cause_name(arming_disarm_cause_t cause)
{
    switch (cause) {
    case DISARM_CAUSE_NONE:     return "-";
    case DISARM_CAUSE_BOOT:     return "BOOT";
    case DISARM_CAUSE_SWITCH:   return "CONG_TAC";
    case DISARM_CAUSE_FAILSAFE: return "MAT_SONG";
    case DISARM_CAUSE_BLOCKED:  return "CHUA_DU_DK";
    default:                    return "?";
    }
}

/*
 * Thứ tự kiểm tra ở đây là thứ tự ƯU TIÊN hiển thị, không phải thứ tự bit.
 * Người lái cần biết việc cần làm tiếp theo, nên nguyên nhân gốc đứng trước:
 * mất RC thì mọi thứ khác đều vô nghĩa, và "gạt công tắc về OFF" phải hiện
 * ra trước các điều kiện đã thoả từ lâu.
 */
const char *arming_block_reason(void)
{
    const uint32_t b = g_fc.sys.arm_block_flags;

    if (b == (uint32_t)ARM_BLOCK_NONE)        { return "san sang"; }
    if (b & ARM_BLOCK_NO_RC)                  { return "mat RC"; }
    if (b & ARM_BLOCK_FAILSAFE)               { return "dang failsafe"; }
    if (b & ARM_BLOCK_SWITCH)                 { return "gat cong tac ve OFF"; }
    if (b & ARM_BLOCK_GYRO_CALIB)             { return "gyro chua hieu chuan"; }
    if (b & ARM_BLOCK_SENSOR_FAIL)            { return "cam bien loi"; }
    if (b & ARM_BLOCK_THROTTLE_HIGH)          { return "ga chua ve thap"; }
    if (b & ARM_BLOCK_NOT_LEVEL)              { return "may bay nghieng"; }
    if (b & ARM_BLOCK_LOW_BATTERY)            { return "pin yeu"; }
    if (b & ARM_BLOCK_USB_MSC)                { return "dang doc the qua USB"; }
    return "?";
}
