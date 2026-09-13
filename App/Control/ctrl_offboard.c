/**
 * @file    ctrl_offboard.c
 * @brief   Hiện thực cổng nhận lệnh vận tốc từ máy tính nhúng.
 */

#include "ctrl_offboard.h"
#include "fc_state.h"
#include "param_table.h"
#include "ctrl_althold.h"
#include "ctrl_angle.h"

/* ==========================================================================
 * Trạng thái module
 * ========================================================================== */

static offboard_state_t s_state;
static offboard_exit_t  s_exit;

static vec3f_t  s_vel_body;      /* x = tới, y = phải, m/s            */
static float    s_climb_mps;     /* dương = lên                        */
static float    s_yaw_rate_dps;

static uint32_t s_last_ms;       /* setpoint hợp lệ gần nhất           */
static bool     s_have_sp;       /* đã từng nhận được setpoint hợp lệ  */

static uint32_t s_accepted;
static uint32_t s_rejected;
static uint32_t s_clamped;
static uint16_t s_clamp_run;     /* số khung LIÊN TIẾP bị kẹp dải      */

/* ==========================================================================
 * Đọc tay điều khiển
 * ========================================================================== */

/**
 * Công tắc cho phép có đang bật không.
 *
 * Người gọi phải tự kiểm tra sóng còn tốt trước — hàm này chỉ đọc giá trị
 * kênh, nó không biết giá trị đó còn mới hay đã ôi.
 */
static bool switch_on(void)
{
    const int8_t ch = g_params.offboard_switch_channel;

    /* -1 = tắt hẳn tính năng. Đây là mặc định xuất xưởng. */
    if (ch < 0) {
        return false;
    }

    return g_fc.rc.channel_raw[ch] >= g_params.offboard_switch_on;
}

/** Sóng còn đọc được không. Cùng điều kiện arming.c dùng. */
static bool rc_usable(void)
{
    return g_fc.rc.healthy && !g_fc.rc.failsafe;
}

/**
 * Người lái có đang chạm BẤT KỲ cần nào không.
 *
 * roll / pitch / yaw đã bị trừ vùng chết ở tầng CRSF nên bằng ĐÚNG 0 khi ở
 * giữa; ngưỡng offboard_stick_override chỉ để khỏi nhạy với rung tay.
 *
 * Cần ga không có lò xo nên không so với 0 được. Hỏi ctrl_althold — nó giữ
 * định nghĩa duy nhất của "cần ga đang ở giữa". Tách hai chỗ tự tính thì chỉ
 * cần lệch nhau một chút là OFFBOARD thoát trong khi vòng độ cao vẫn tưởng cần
 * còn ở giữa.
 */
static bool pilot_touching_sticks(void)
{
    const float t = g_params.offboard_stick_override;

    return (fabsf(g_fc.rc.roll)  > t) ||
           (fabsf(g_fc.rc.pitch) > t) ||
           (fabsf(g_fc.rc.yaw)   > t) ||
           !ctrl_althold_stick_centred();
}

/* ==========================================================================
 * Chuyển trạng thái
 * ========================================================================== */

/**
 * Rời OFFBOARD.
 *
 * Xoá luôn mục tiêu đang giữ. Không xoá thì lần vào lại sẽ dùng con số đọng
 * từ trước lúc hỏng — đúng cái ta vừa bỏ công thoát khỏi.
 */
static void leave(offboard_exit_t reason, offboard_state_t to)
{
    s_state        = to;
    s_exit         = reason;
    s_vel_body     = (vec3f_t){ 0.0f, 0.0f, 0.0f };
    s_climb_mps    = 0.0f;
    s_yaw_rate_dps = 0.0f;
    s_clamp_run    = 0;
}

/* ==========================================================================
 * Vòng đời
 * ========================================================================== */

void ctrl_offboard_init(void)
{
    /*
     * Khởi động vào thẳng LOCKED. Cắm pin trong khi công tắc đang bật sẵn và
     * Pi đã phát setpoint thì máy bay vẫn KHÔNG tự chạy — phải gạt công tắc
     * về OFF rồi lên lại. Cùng lập luận với DISARM_CAUSE_BOOT của arming.c.
     */
    leave(OFFBOARD_EXIT_BOOT, OFFBOARD_LOCKED);

    s_last_ms   = 0;
    s_have_sp   = false;
    s_accepted  = 0;
    s_rejected  = 0;
    s_clamped   = 0;
}

void ctrl_offboard_update(uint32_t now_ms)
{
    const bool rc_ok = rc_usable();
    const bool sw    = rc_ok && switch_on();

    switch (s_state) {

    case OFFBOARD_LOCKED:
        /*
         * Mở khoá đòi ĐỌC ĐƯỢC công tắc và thấy nó đang OFF. Mất sóng KHÔNG
         * mở khoá: không đọc được thì không có sự đồng ý nào cả, và mở khoá
         * lúc đó là mở đúng vào lúc ta biết ít nhất về ý định người lái.
         */
        if (rc_ok && !switch_on()) {
            s_state = OFFBOARD_DISABLED;
        }
        break;

    case OFFBOARD_DISABLED:
        if (!rc_ok) {
            /* Không đọc được công tắc thì không có sự đồng ý nào — khoá. */
            leave(OFFBOARD_EXIT_NO_RC, OFFBOARD_LOCKED);
            break;
        }

        /*
         * ĐÃ ARM + ch8 BẬT + NGƯỜI LÁI CHẠM CẦN -> Pi MẤT QUYỀN, dù OFFBOARD
         * chưa kịp chạy.
         *
         * Không có nhánh này thì có một khe hở: Pi arm xong nhưng chưa kịp
         * phát setpoint, người lái thấy lạ nên đẩy cần giành lái, rồi thả cần
         * ra — đúng lúc đó setpoint của Pi tới và OFFBOARD giật quyền lại.
         * Quy tắc là "sau khi arm, chạm cần là Pi hết quyền", không có ngoại
         * lệ cho trường hợp OFFBOARD chưa bắt đầu.
         */
        if (sw && g_fc.motor.armed && pilot_touching_sticks()) {
            leave(OFFBOARD_EXIT_STICK, OFFBOARD_LOCKED);
            break;
        }

        /*
         * Vào OFFBOARD đòi đủ MỌI điều kiện cùng lúc. Đáng chú ý là đòi có
         * sẵn setpoint còn hạn: gạt công tắc lúc Pi chưa nói gì thì không có
         * chuyện gì xảy ra, thay vì vào rồi hết hạn ngay sau đó.
         */
        if (sw &&
            g_fc.motor.armed &&
            g_fc.mode == FC_MODE_ARMED &&
            g_fc.est.attitude_valid &&
            s_have_sp &&
            (fc_elapsed_ms(now_ms, s_last_ms) < g_params.offboard_timeout_ms) &&
            !pilot_touching_sticks() &&
            /*
             * ch6 PHẢI ở POSHOLD. Rời OFFBOARD là rơi về chế độ ch6 đang chọn,
             * và chỉ POSHOLD mới "phanh rồi treo" khi Pi hỏng — ANGLE chỉ giữ
             * thăng bằng rồi trôi theo quán tính. Luận cứ an toàn của cả giao
             * ước (GIAO_UOC mục 1, 5.2) dựa trên đường lùi này.
             */
            (ctrl_angle_switch_mode() == FLIGHT_MODE_POSHOLD)) {

            s_state     = OFFBOARD_ACTIVE;
            s_exit      = OFFBOARD_EXIT_NONE;
            s_clamp_run = 0;
        }
        break;

    case OFFBOARD_ACTIVE:
        /*
         * Thứ tự kiểm tra là thứ tự ưu tiên khi báo nguyên nhân. Công tắc đặt
         * trước vì đó là ý muốn tường minh của người lái; hết hạn đặt cuối vì
         * nó là thứ dễ kèm theo mọi lỗi khác.
         *
         * CÔNG TẮC VỀ OFF -> DISABLED, không phải LOCKED: đó là lối thoát
         * bình thường, có chủ ý. Mọi nguyên nhân còn lại đều là hỏng hóc và
         * rơi vào LOCKED, bắt gạt công tắc một vòng mới vào lại được.
         *
         * "Mất sóng" và "công tắc về OFF" PHẢI tách riêng — xem ngay dưới.
         */
        if (!rc_ok) {
            /*
             * MẤT SÓNG -> LOCKED, không phải DISABLED.
             *
             * Phân biệt với nhánh dưới là bắt buộc, và đây từng là một lỗ
             * hổng thật: cả hai đều làm sw bằng false, nhưng ý nghĩa ngược
             * nhau. Gộp chung thì mất sóng chỉ đưa về DISABLED, và khi sóng
             * về với công tắc vẫn đang ON cùng Pi vẫn phát setpoint, OFFBOARD
             * TỰ VÀO LẠI — đúng tình huống 2 mà arming.h dựng ra LOCKED để
             * chặn. Không đọc được công tắc thì không có sự đồng ý nào cả.
             */
            leave(OFFBOARD_EXIT_NO_RC, OFFBOARD_LOCKED);
        }
        else if (!switch_on()) {
            /* Người lái chủ động gạt xuống: lối thoát bình thường, không khoá. */
            leave(OFFBOARD_EXIT_SWITCH, OFFBOARD_DISABLED);
        }
        else if (!g_fc.motor.armed || g_fc.mode != FC_MODE_ARMED) {
            /*
             * Disarm -> TAT, KHÔNG khoá. Pi được phép tự disarm rồi arm lại;
             * khoá ở đây thì Pi mất quyền ngay sau lệnh disarm của chính nó.
             *
             * Vẫn an toàn khi người lái disarm bằng ch5: ch5 đã xuống thì
             * arming.c không cho Pi arm, nên TAT cũng không mở ra được gì.
             * Còn mất sóng dẫn tới disarm thì nhánh !rc_ok phía trên đã bắt
             * trước và khoá.
             */
            leave(OFFBOARD_EXIT_DISARM, OFFBOARD_DISABLED);
        }
        else if (ctrl_angle_switch_mode() != FLIGHT_MODE_POSHOLD) {
            /*
             * Người lái gạt ch6 rời POSHOLD giữa chừng -> KHOA, không phải TAT.
             * Về TAT thì lúc gạt ch6 trở lại POSHOLD, OFFBOARD tự vào lại mà
             * không cần gạt ch8 — Pi lấy lại quyền bằng một thao tác không mang
             * nghĩa "cho phép Pi".
             */
            leave(OFFBOARD_EXIT_MODE_SW, OFFBOARD_LOCKED);
        }
        else if (!g_fc.est.attitude_valid) {
            leave(OFFBOARD_EXIT_NO_ATT, OFFBOARD_LOCKED);
        }
        else if (pilot_touching_sticks()) {
            leave(OFFBOARD_EXIT_STICK, OFFBOARD_LOCKED);
        }
        else if (fc_elapsed_ms(now_ms, s_last_ms) >= g_params.offboard_timeout_ms) {
            leave(OFFBOARD_EXIT_TIMEOUT, OFFBOARD_LOCKED);
        }
        break;

    default:
        leave(OFFBOARD_EXIT_BOOT, OFFBOARD_LOCKED);
        break;
    }
}

/* ==========================================================================
 * Cửa vào
 * ========================================================================== */

/*
 * Lệnh leo/xuống nhỏ hơn mức này coi là "giữ độ cao" khi xét trần độ cao.
 *
 * VÌ SAO CẦN (GIAO_UOC 11.1 #11, Pi đo 09-13): MAVROS đổi FLU -> FRD bằng
 * quaternion quay 180° quanh X, mà cos(π/2) trong double là 6e-17 chứ không
 * phải 0. Lệnh "sang phải 0,5, vz = 0" lên dây thành vz ≈ −6e-17 — lên một
 * lượng vô nghĩa, nhưng vẫn là "> 0" với phép so dấu. Ở trần, khung đó bị
 * đếm là kẹp. (Lỗi được phát hiện ở sàn độ cao — sàn đã bỏ 09-13, trần vẫn
 * cùng kiểu so dấu nên dải chết vẫn cần.)
 *
 * 1 cm/s: lớn hơn mọi sai số làm tròn, nhỏ hơn mọi lệnh leo có chủ đích.
 */
#define OFFBOARD_CLIMB_DEADBAND_MPS  0.01f

bool ctrl_offboard_set_target(float vx_body, float vy_body, float vz_ned,
                              float yaw_rate, uint32_t now_ms)
{
    /*
     * --- Lớp 3: NaN và vô cực ---
     *
     * PHẢI kiểm tra riêng, không được trông vào chặn dải bên dưới. Mọi phép
     * so sánh với NaN đều trả false, nên fc_constrainf() để NaN đi qua nguyên
     * vẹn. Nó sẽ vào khâu tích phân của ctrl_poshold và ở lại đó vĩnh viễn.
     */
    if (!isfinite(vx_body) || !isfinite(vy_body) ||
        !isfinite(vz_ned)  || !isfinite(yaw_rate)) {
        s_rejected++;
        return false;
    }

    /*
     * --- Lớp 4: giới hạn bao ---
     *
     * KẸP chứ không LOẠI. Loại khung sẽ làm bộ đếm hết hạn chạy trong khi Pi
     * vẫn sống và vẫn gửi đều — chẩn đoán ra một nguyên nhân sai. Kẹp rồi đếm,
     * và nếu kẹp liên tục quá lâu thì mới kết luận là hỏng.
     */
    const float vmax = g_params.offboard_max_vel_mps;
    const float cmax = g_params.offboard_max_climb_mps;
    const float ymax = g_params.offboard_max_yaw_dps;

    const float fwd   = fc_constrainf(vx_body, -vmax, vmax);
    const float right = fc_constrainf(vy_body, -vmax, vmax);

    /* MAVLink NED có Z DƯƠNG LÀ XUỐNG; vòng giữ độ cao làm việc với "lên". */
    float climb = fc_constrainf(-vz_ned, -cmax, cmax);

    const float yaw_dps = fc_constrainf(yaw_rate * FC_RAD_TO_DEG, -ymax, ymax);

    /* Kẹp theo GIỚI HẠN BAO: lệnh vượt trần tốc độ — dấu hiệu Pi ra lệnh vô nghĩa. */
    const bool range_clamped = (fwd     != vx_body) ||
                               (right   != vy_body) ||
                               (climb   != -vz_ned) ||
                               (yaw_dps != yaw_rate * FC_RAD_TO_DEG);
    bool alt_clamped = false;

    /*
     * Bao độ cao — chỉ còn TRẦN. Vượt trần thì chỉ còn được đi xuống: chặn
     * một chiều chứ không cấm cả hai, nếu không máy bay mắc kẹt ngoài bao.
     *
     * KHÔNG CÒN SÀN (bỏ 2026-09-13, GIAO_UOC 11.1 #12): Pi hạ cánh bằng chính
     * setpoint vận tốc, xuống tới chạm đất. Thay vào đó, sát đất tốc độ xuống
     * bị kẹp chậm lại, để mọi lần chạm đất trong OFFBOARD đều nhẹ — dù Pi hạ
     * có chủ đích hay Pi lỗi ra lệnh xuống.
     *
     * Không tin được độ cao thì bỏ qua cả hai: kẹp theo một con số sai còn
     * tệ hơn không kẹp. Tầng trên đã có ctrl_althold tự từ chối khi
     * altitude_valid hạ xuống.
     */
    if (g_fc.est.altitude_valid) {
        if (g_fc.est.altitude_m >= g_params.offboard_max_alt_m &&
            climb > OFFBOARD_CLIMB_DEADBAND_MPS) {
            climb       = 0.0f;
            alt_clamped = true;
        }
        if (g_fc.est.altitude_m <= OFFBOARD_SLOW_DESCENT_ALT_M &&
            climb < -OFFBOARD_SLOW_DESCENT_MPS) {
            climb       = -OFFBOARD_SLOW_DESCENT_MPS;
            alt_clamped = true;
        }
    }

    /* OB_RX_CLP đếm CẢ HAI loại kẹp — Pi cần thấy lệnh của mình bị sửa. */
    if (range_clamped || alt_clamped) {
        s_clamped++;
    }

    /*
     * Chỉ kẹp theo GIỚI HẠN BAO mới dồn vào lối ra KEP_DAI.
     *
     * Kẹp vài khung là bình thường — Pi đổi hướng gấp thì lệnh vọt quá trần
     * trong chốc lát. Kẹp LIÊN TIẾP nghĩa là Pi đang rất tự tin ra một lệnh vô
     * nghĩa, và đó là ca duy nhất bắt được bằng cách này.
     *
     * Kẹp theo BAO ĐỘ CAO (trần, xuống chậm sát đất) thì không: bao đã tự thi
     * hành việc của nó, và lệnh bị kẹp ở đó là lệnh hợp lệ — Pi lơ lửng ngay
     * trần dao động quanh 0 m/s, hay Pi hạ cánh gửi cố định −0,5 m/s rồi bị kẹp
     * về −0,3 m/s suốt đoạn cuối. Tính cả vào đây thì Pi mất quyền đúng lúc máy
     * bay đang hạ cánh — chỗ tệ nhất để mất quyền.
     */
    if (range_clamped) {
        if (++s_clamp_run >= g_params.offboard_clamp_limit) {
            ctrl_offboard_request_exit(OFFBOARD_EXIT_CLAMP);
        }
    } else {
        s_clamp_run = 0;
    }

    s_vel_body     = (vec3f_t){ fwd, right, 0.0f };
    s_climb_mps    = climb;
    s_yaw_rate_dps = yaw_dps;

    s_last_ms = now_ms;
    s_have_sp = true;
    s_accepted++;

    return true;
}

void ctrl_offboard_note_reject(void)
{
    s_rejected++;
}

void ctrl_offboard_request_exit(offboard_exit_t reason)
{
    if (s_state == OFFBOARD_ACTIVE) {
        leave(reason, OFFBOARD_LOCKED);
    }
}

/* ==========================================================================
 * Cửa ra
 * ========================================================================== */

bool ctrl_offboard_is_active(void)
{
    return s_state == OFFBOARD_ACTIVE;
}

bool ctrl_offboard_switch_on(void)
{
    return rc_usable() && switch_on();
}

bool ctrl_offboard_pi_has_authority(void)
{
    return ctrl_offboard_switch_on() && (s_state != OFFBOARD_LOCKED);
}

vec3f_t ctrl_offboard_velocity_body(void) { return s_vel_body;     }
float   ctrl_offboard_climb_mps(void)     { return s_climb_mps;    }
float   ctrl_offboard_yaw_rate_dps(void)  { return s_yaw_rate_dps; }

/* ==========================================================================
 * Chẩn đoán
 * ========================================================================== */

offboard_state_t ctrl_offboard_state(void)     { return s_state;    }
offboard_exit_t  ctrl_offboard_last_exit(void) { return s_exit;     }
uint32_t ctrl_offboard_accepted(void)          { return s_accepted; }
uint32_t ctrl_offboard_rejected(void)          { return s_rejected; }
uint32_t ctrl_offboard_clamped(void)           { return s_clamped;  }

uint32_t ctrl_offboard_age_ms(uint32_t now_ms)
{
    return s_have_sp ? fc_elapsed_ms(now_ms, s_last_ms) : UINT32_MAX;
}

const char *ctrl_offboard_state_name(offboard_state_t s)
{
    switch (s) {
    case OFFBOARD_LOCKED:   return "KHOA";
    case OFFBOARD_DISABLED: return "TAT";
    case OFFBOARD_ACTIVE:   return "DANG_CHAY";
    default:                return "?";
    }
}

const char *ctrl_offboard_exit_name(offboard_exit_t r)
{
    switch (r) {
    case OFFBOARD_EXIT_NONE:    return "-";
    case OFFBOARD_EXIT_BOOT:    return "KHOI_DONG";
    case OFFBOARD_EXIT_SWITCH:  return "CONG_TAC";
    case OFFBOARD_EXIT_TIMEOUT: return "HET_HAN";
    case OFFBOARD_EXIT_STICK:   return "DAY_CAN";
    case OFFBOARD_EXIT_DISARM:  return "DISARM";
    case OFFBOARD_EXIT_CLAMP:   return "KEP_DAI";
    case OFFBOARD_EXIT_NO_ATT:  return "MAT_GOC";
    case OFFBOARD_EXIT_NO_RC:   return "MAT_SONG";
    case OFFBOARD_EXIT_MODE_SW: return "CHE_DO";
    default:                    return "?";
    }
}
