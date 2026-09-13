/**
 * @file    arming.c
 * @brief   Hiện thực bộ giám sát arm/disarm.
 */

#include "arming.h"
#include "fc_state.h"
#include "param_table.h"
#include "fc_time.h"
#include "ctrl_offboard.h"
#include "ctrl_althold.h"
#include "ctrl_angle.h"

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

/*
 * ch5 đã được gạt lên TRONG chế độ Pi (ch8 bật) và đang chờ lệnh arm từ Pi.
 *
 * Cần nhớ riêng vì cùng một tình trạng "SAFE, ch5 đang ON" có hai nghĩa ngược
 * nhau tuỳ lịch sử: vừa lên trong chế độ Pi thì phải CHỜ; còn nếu ch8 bị gạt
 * xuống sau đó thì tuyệt đối không được rơi xuống nhánh arm tay.
 */
static bool     s_pi_wait;

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
    s_pi_wait      = false;
}

void arming_update(uint32_t now_ms)
{
    debounce_switch(now_ms);

    /*
     * ch8 bật = chế độ Pi: ch5 đổi nghĩa từ "arm ngay" thành "cho phép Pi arm".
     * Đọc ở đầu hàm vì nó quyết định cả chính sách ga lẫn nhánh SAFE bên dưới.
     */
    const bool pi_mode = ctrl_offboard_switch_on();

    /* Điều kiện sẵn sàng của máy bay: cảm biến, sóng, độ nghiêng, pin... */
    (void)fc_state_check_arm();

    /*
     * Chính sách ga — áp ở đây chứ không ở fc_state_check_arm(), xem lý do
     * dài ở hàm đó. Hai chế độ đòi hai vị trí ga loại trừ nhau.
     */
    if (pi_mode) {
        /*
         * Ga phải ở GIỮA. Lệch khỏi giữa thì OFFBOARD coi là người lái chạm
         * cần và thoát ngay nhịp đầu — Pi arm xong là mất quyền luôn. Bắt ở
         * đây thì lỗi hiện ra trên console trước khi cánh quạt quay.
         */
        if (!ctrl_althold_stick_centred()) {
            g_fc.sys.arm_block_flags |= ARM_BLOCK_THR_NOT_CENTRE;
        }
        if (!ctrl_offboard_pi_has_authority()) {
            g_fc.sys.arm_block_flags |= ARM_BLOCK_PI_NO_AUTH;
        }
        /*
         * ch6 phải ở POSHOLD thì Pi mới được arm — không chỉ lúc vào OFFBOARD.
         *
         * Chế độ Pi đòi ga ở GIỮA. Arm xong, trước khi OFFBOARD kịp vào, máy
         * bay chạy theo chế độ ch6. Nếu đó là ANGLE thì ga lấy thẳng từ cần:
         * cần ở giữa = khoảng 50% lực đẩy ngay khoảnh khắc arm. POSHOLD (và cả
         * ALTHOLD) giao ga cho vòng giữ độ cao nên cần ở giữa = giữ nguyên.
         */
        if (ctrl_angle_switch_mode() != FLIGHT_MODE_POSHOLD) {
            g_fc.sys.arm_block_flags |= ARM_BLOCK_PI_NOT_POSHOLD;
        }
        /*
         * Chờ người lái gạt ch5. Không phải lỗi, nhưng Pi PHẢI thấy được: thiếu
         * bit này thì lúc ch8 lên còn ch5 xuống, Pi nhận OB_ARM_RDY = 0 cùng
         * OB_ARM_BLK = 0 — tức "chưa sẵn sàng mà không có lý do nào", không biết
         * đang chờ gì (GIAO_UOC mục 6.3). Không ảnh hưởng quyết định arm: nhánh
         * SAFE ở chế độ Pi không bao giờ tự arm.
         */
        if (s_state == ARMING_SAFE && !s_switch_stable) {
            g_fc.sys.arm_block_flags |= ARM_BLOCK_PI_WAIT_CH5;
        }
    } else {
        /* Arm tay: ga phải THẤP, nếu không máy bay nhảy lên ngay khi arm. */
        if (g_fc.rc.throttle > g_params.arm_throttle_max_norm) {
            g_fc.sys.arm_block_flags |= ARM_BLOCK_THROTTLE_HIGH;
        }
    }

    const bool conditions_ok = (g_fc.sys.arm_block_flags == (uint32_t)ARM_BLOCK_NONE);

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
        s_state   = ARMING_LOCKED;
        s_pi_wait = false;
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
        if (!s_switch_stable) {
            s_pi_wait = false;          /* ch5 đang xuống: chưa có ý định arm */
            break;
        }

        if (pi_mode) {
            /*
             * ch8 BẬT: ch5 nghĩa là "cho phép Pi arm", KHÔNG phải "arm ngay".
             * Đứng yên ở SAFE, chờ arming_request_arm_link().
             *
             * Không khoá khi chưa đủ điều kiện như nhánh arm tay: ở đây không
             * có gì tự arm cả, lệnh arm luôn là hành động tường minh của Pi,
             * và mỗi lệnh đều kiểm tra điều kiện lại từ đầu.
             */
            s_pi_wait = true;
        }
        else if (s_pi_wait) {
            /*
             * ch5 lên TRONG chế độ Pi, rồi ch8 bị gạt xuống.
             *
             * Rơi xuống nhánh arm tay bên dưới thì máy bay ARM NGAY, đúng lúc
             * người lái vừa gạt ch8 xuống với ý "huỷ chế độ Pi". Một thao tác
             * mang nghĩa DỪNG LẠI biến thành lệnh quay cánh quạt. Khoá lại, bắt
             * gạt ch5 một vòng.
             */
            s_pi_wait    = false;
            s_state      = ARMING_LOCKED;
            s_last_cause = DISARM_CAUSE_BLOCKED;
        }
        else if (conditions_ok && fc_state_set_mode(FC_MODE_ARMED)) {
            s_state = ARMING_ARMED;
            s_arm_count++;
        }
        else {
            /*
             * Bật công tắc lúc chưa đủ điều kiện. Khoá lại thay vì chờ: nếu
             * chỉ chờ, máy bay sẽ tự arm đúng vào khoảnh khắc điều kiện cuối
             * cùng vừa thoả — chẳng hạn ngay khi gyro hiệu chuẩn xong — mà
             * người lái không hề ra lệnh.
             *
             * Cũng chính nhánh này bảo vệ sai thứ tự công tắc: gạt ch5 TRƯỚC
             * ch8 trong khi ga đang ở giữa (như chế độ Pi đòi) thì điều kiện
             * "ga thấp" không thoả -> khoá, thay vì arm với ga ở giữa.
             */
            s_state      = ARMING_LOCKED;
            s_last_cause = DISARM_CAUSE_BLOCKED;
        }
        break;

    case ARMING_ARMED:
        /*
         * ch8 KHÔNG ảnh hưởng gì ở đây. Đang bay bằng Pi mà người lái gạt ch8
         * xuống thì chỉ tước quyền Pi (ctrl_offboard lo); máy bay vẫn arm vì
         * ch5 vẫn đang ON. Chỉ ch5 xuống mới disarm.
         */
        if (!s_switch_stable) {
            s_pi_wait = false;
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

/**
 * Kiểm MỌI điều kiện để lệnh ARM từ Pi được chấp nhận — không làm gì cả.
 *
 * Đây là nguồn sự thật DUY NHẤT cho cả lệnh ARM lẫn OB_ARM_RDY. Tính ở hai chỗ
 * thì sớm muộn sẽ có lúc Pi thấy OB_ARM_RDY = 1 mà ARM vẫn bị từ chối — đúng
 * cái hợp đồng hứa sẽ không xảy ra (GIAO_UOC mục 6.3).
 *
 * Người gọi tự xử lý trường hợp đã arm.
 */
static arming_link_result_t link_arm_check(void)
{
    /* Quyền: ch8 bật, sóng đọc được, OFFBOARD không bị khoá. */
    if (!ctrl_offboard_pi_has_authority()) {
        return ARMING_LINK_NO_AUTHORITY;
    }

    /*
     * Người lái phải đã gạt ch5 lên TRONG chế độ Pi. s_pi_wait chỉ bật khi
     * đúng quy trình — xem nhánh SAFE của arming_update().
     */
    if (s_state != ARMING_SAFE || !s_pi_wait) {
        return ARMING_LINK_NO_AUTHORITY;
    }

    /*
     * Điều kiện đã tính trong arming_update() của CHÍNH vòng lặp này (mav_update
     * chạy sau nó). Bỏ cờ PI_NO_AUTH khỏi phép so: cờ đó tính từ trạng thái
     * OFFBOARD của vòng TRƯỚC, còn quyền vừa được kiểm lại tươi ở trên.
     */
    if ((g_fc.sys.arm_block_flags & ~(uint32_t)ARM_BLOCK_PI_NO_AUTH) != 0u) {
        return ARMING_LINK_BLOCKED;
    }

    /*
     * fc_state_set_mode() chỉ cho vào ARMED từ DISARMED (hoặc FAILSAFE). Kiểm ở
     * đây để OB_ARM_RDY không bao giờ bằng 1 trong lúc hệ thống còn đang
     * INIT/CALIBRATING — nếu không, lệnh ARM sẽ bị từ chối ở bước đổi chế độ.
     */
    if (g_fc.mode != FC_MODE_DISARMED) {
        return ARMING_LINK_BLOCKED;
    }

    return ARMING_LINK_OK;
}

bool arming_link_arm_ready(void)
{
    return (g_fc.mode != FC_MODE_ARMED) && (link_arm_check() == ARMING_LINK_OK);
}

uint32_t arming_link_arm_block(void)
{
    return (g_fc.mode == FC_MODE_ARMED) ? 0u : g_fc.sys.arm_block_flags;
}

arming_link_result_t arming_request_arm_link(void)
{
    /* Đã arm rồi: lệnh lặp lại là vô hại, báo thành công. */
    if (g_fc.mode == FC_MODE_ARMED) {
        return ARMING_LINK_OK;
    }

    const arming_link_result_t check = link_arm_check();
    if (check != ARMING_LINK_OK) {
        return check;
    }

    if (!fc_state_set_mode(FC_MODE_ARMED)) {
        return ARMING_LINK_BLOCKED;
    }

    s_state   = ARMING_ARMED;
    s_pi_wait = false;
    s_arm_count++;
    return ARMING_LINK_OK;
}

/**
 * Gần đất theo cổng DISARM: độ cao ước lượng ≤ mốc mặt đất + 20 cm.
 *
 * Dùng độ cao ƯỚC LƯỢNG (EKF: laser đã bù nghiêng + baro), không dùng laser
 * thô: nghiêng quá EST_RANGE_MAX_TILT_DEG thì laser bị loại nhưng ước lượng
 * vẫn chạy tiếp bằng baro — máy bay lật nằm dưới đất vẫn có con số để xét,
 * dù con số đó trôi dần theo baro.
 *
 * Không tin được độ cao thì coi là KHÔNG gần đất: thà từ chối một lệnh đúng
 * (Pi còn mã ép, người lái còn ch5) còn hơn cắt động cơ giữa trời.
 */
static bool link_near_ground(void)
{
    return g_fc.est.altitude_valid &&
           (g_fc.est.altitude_m <= LINK_DISARM_GROUND_M + LINK_DISARM_MAX_HEIGHT_M);
}

static bool link_is_armed(void)
{
    return (s_state == ARMING_ARMED) || (g_fc.mode == FC_MODE_ARMED);
}

bool arming_link_disarm_ready(void)
{
    return link_is_armed() && ctrl_offboard_pi_has_authority() && link_near_ground();
}

arming_link_result_t arming_request_disarm(bool force)
{
    /*
     * Đã ở dưới đất thì báo thành công, TRƯỚC khi xét quyền. Một lệnh cắt khẩn
     * cấp cần câu trả lời trung thực "máy bay đang không arm", không phải
     * "bạn không có quyền".
     */
    if (!link_is_armed()) {
        return ARMING_LINK_OK;
    }

    /*
     * Sau khi người lái chạm cần giành lái, Pi không còn quyền gì — kể cả
     * disarm, kể cả mã ép. Người lái vẫn luôn cắt được bằng ch5.
     */
    if (!ctrl_offboard_pi_has_authority()) {
        return ARMING_LINK_NO_AUTHORITY;
    }

    /*
     * Cổng độ cao. Pi ở GIAO_UOC 11.1 #12 cam kết: DISARM thường chỉ gửi khi
     * đã nhận ra chạm đất; bị từ chối thì thử lại 3 s rồi báo người lái, KHÔNG
     * tự chuyển sang mã ép. Mã ép chỉ đến từ người ra lệnh qua GCS.
     */
    if (!force && !link_near_ground()) {
        return ARMING_LINK_BLOCKED;
    }

    do_disarm(DISARM_CAUSE_LINK);

    /*
     * Về SAFE, không phải LOCKED như trước đây.
     *
     * Lý do cũ để khoá là: công tắc vẫn ON, để ở SAFE thì vòng sau tự arm lại.
     * Nhưng lệnh này chỉ tới được đây khi đang ở chế độ Pi, mà ở chế độ Pi
     * nhánh SAFE KHÔNG tự arm — nó chờ lệnh. Nên khoá không còn bảo vệ gì,
     * chỉ tước mất khả năng Pi tự arm lại.
     */
    s_state   = ARMING_SAFE;
    s_pi_wait = s_switch_stable;

    return (g_fc.mode != FC_MODE_ARMED) ? ARMING_LINK_OK : ARMING_LINK_BLOCKED;
}

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
    case DISARM_CAUSE_LINK:     return "LENH_MAVLINK";
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
    if (b & ARM_BLOCK_PI_NO_AUTH)             { return "Pi mat quyen: gat CH8 xuong roi len"; }
    if (b & ARM_BLOCK_PI_WAIT_CH5)            { return "che do Pi: cho gat CH5 len"; }
    if (b & ARM_BLOCK_GYRO_CALIB)             { return "gyro chua hieu chuan"; }
    if (b & ARM_BLOCK_SENSOR_FAIL)            { return "cam bien loi"; }
    if (b & ARM_BLOCK_THROTTLE_HIGH)          { return "ga chua ve thap"; }
    if (b & ARM_BLOCK_THR_NOT_CENTRE)         { return "che do Pi: dua ga ve giua"; }
    if (b & ARM_BLOCK_PI_NOT_POSHOLD)         { return "che do Pi: dua CH6 ve POSHOLD"; }
    if (b & ARM_BLOCK_NOT_LEVEL)              { return "may bay nghieng"; }
    if (b & ARM_BLOCK_LOW_BATTERY)            { return "pin yeu"; }
    if (b & ARM_BLOCK_USB_MSC)                { return "dang doc the qua USB"; }
    return "?";
}
