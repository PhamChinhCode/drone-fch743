/**
 * @file    ctrl_offboard.h
 * @brief   Cổng nhận lệnh vận tốc từ máy tính nhúng, và toàn bộ lớp phòng vệ
 *          quanh nó.
 *
 * MODULE NÀY KHÔNG ĐIỀU KHIỂN GÌ CẢ. Nó chỉ giữ một mục tiêu vận tốc, quyết
 * định lúc nào mục tiêu đó được phép dùng, và mở cổng cho ctrl_poshold /
 * ctrl_althold đọc. Mọi vòng PID vẫn nằm nguyên chỗ cũ.
 *
 * VÌ SAO LÀM NHƯ VẬY:
 *   ctrl_poshold đã là một vòng GIỮ VẬN TỐC hoàn chỉnh, đã chỉnh, đã có sẵn
 *   đường lùi khi mất optical flow. OFFBOARD chỉ cần đổi NGUỒN của mục tiêu
 *   vận tốc từ cần điều khiển sang MAVLink. Làm OFFBOARD thành một nhánh điều
 *   khiển song song thì phải nhân bản từng chốt an toàn của ctrl_poshold —
 *   mà nhân bản chốt an toàn là cách chắc chắn nhất để bỏ sót một cái.
 *
 * THANG TỤT CẤP — nối dài cái đã có trong ctrl_angle.c:
 *
 *   OFFBOARD --lệnh Pi hỏng--> POSHOLD --mất flow--> ANGLE --mất góc--> ACRO
 *
 *   Rơi về POSHOLD là câu trả lời đúng cho mọi lỗi từ Pi: POSHOLD lấy mục
 *   tiêu vận tốc từ cần điều khiển, mà cần đang ở giữa (người lái không chạm)
 *   nghĩa là mục tiêu 0 — máy bay PHANH LẠI VÀ TREO. Không phải viết thêm
 *   một dòng logic "hover khi lỗi" nào.
 *
 * BA TRẠNG THÁI:
 *
 *      +----------+  công tắc OFF  +----------+  công tắc ON + đủ đk  +--------+
 *      |  LOCKED  | -------------> | DISABLED | --------------------> | ACTIVE |
 *      +----------+                +----------+                       +--------+
 *           ^                           ^                                 |
 *           |     lỗi bất kỳ            |        công tắc OFF             |
 *           +---------------------------+---------------------------------+
 *
 *   LOCKED DÍNH, và đó là điểm mấu chốt. Thoát khỏi LOCKED CHỈ có một cách:
 *   gạt công tắc về OFF. Cùng lập luận với ARMING_LOCKED trong arming.h, áp
 *   vào đúng tình huống 2 của nó: một node ROS2 crash rồi tự khởi động lại
 *   trong khi công tắc vẫn đang ON. Không có LOCKED thì máy bay tự lao đi
 *   lại ngay khi node sống dậy, đúng lúc người lái tưởng đã xử lý xong.
 *
 *   Khởi động máy bay cũng vào thẳng LOCKED, nên sau mỗi lần arm đều phải
 *   gạt công tắc một lần nữa. Cắm pin trong khi công tắc đang bật sẵn và Pi
 *   đang phát setpoint KHÔNG làm máy bay tự chạy.
 *
 * NĂM LỚP PHÒNG VỆ, xếp theo thứ tự đáng tin giảm dần:
 *
 *   0. CÔNG TẮC RC — phần cứng, không phụ thuộc phần mềm nào chạy đúng.
 *      Gạt xuống là thoát ngay. Công tắc ARM vẫn là dao cắt cuối cùng.
 *   1. CẦN ĐIỀU KHIỂN GHI ĐÈ — người lái đẩy cần theo bản năng chứ không đi
 *      tìm công tắc. Quá offboard_stick_override là thoát.
 *   2. HẾT HẠN SETPOINT — bắt "Pi chết, dây đứt, node treo".
 *   3. LỌC LỆNH Ở CỬA VÀO — bắt "Pi còn sống nhưng gửi số rác".
 *   4. GIỚI HẠN BAO — lớp duy nhất bắt được "Pi gửi đều, số hợp lệ, lệnh sai".
 *
 * VÌ SAO PHẢI LỌC NaN RIÊNG, KHÔNG DỰA VÀO CHẶN DẢI:
 *   Mọi phép so sánh với NaN đều trả false, nên `if (v > MAX) v = MAX;` để
 *   NaN lọt qua nguyên vẹn. Nó đi thẳng vào khâu tích phân của ctrl_poshold
 *   và ở lại đó vĩnh viễn — tụt về ANGLE cũng KHÔNG xoá, vì điều kiện xoá ở
 *   ctrl_angle.c chỉ kích hoạt khi công tắc chế độ đổi. Phải chặn ở cửa.
 *
 * MẶC ĐỊNH LÀ BẬT, kênh 7 (= ch8 trên tay cầm, tham số đếm từ 0):
 *   Ban đầu mặc định là -1 (tắt hẳn). Đổi sang 7 sau khi nghiệm thu trên bàn.
 *   Tắt hẳn đường ra lệnh từ máy tính nhúng: `set offboard_switch_channel=-1`
 *   rồi `save`.
 */
#ifndef CTRL_OFFBOARD_H
#define CTRL_OFFBOARD_H

#include "fc_types.h"
#include "fc_config.h"

/* ==========================================================================
 * Kiểu dữ liệu
 * ========================================================================== */

typedef enum {
    OFFBOARD_LOCKED = 0,  /**< khoá — phải gạt công tắc về OFF mới thoát */
    OFFBOARD_DISABLED,    /**< công tắc OFF, sẵn sàng nhận lệnh vào      */
    OFFBOARD_ACTIVE       /**< đang lái theo lệnh từ máy tính nhúng      */
} offboard_state_t;

/** Vì sao lần rời OFFBOARD gần nhất xảy ra. */
typedef enum {
    OFFBOARD_EXIT_NONE = 0,
    OFFBOARD_EXIT_BOOT,      /**< khoá sẵn từ lúc khởi động            */
    OFFBOARD_EXIT_SWITCH,    /**< người lái gạt công tắc về OFF        */
    OFFBOARD_EXIT_TIMEOUT,   /**< quá lâu không có setpoint hợp lệ     */
    OFFBOARD_EXIT_STICK,     /**< người lái đẩy cần, giành lại quyền   */
    OFFBOARD_EXIT_DISARM,    /**< disarm khi đang OFFBOARD             */
    OFFBOARD_EXIT_CLAMP,     /**< bị kẹp dải liên tục quá lâu          */
    OFFBOARD_EXIT_NO_ATT,    /**< bộ ước lượng mất góc tin cậy         */
    OFFBOARD_EXIT_NO_RC,     /**< mất sóng, không đọc được công tắc    */

    /*
     * Số trên đây là HỢP ĐỒNG TRÊN DÂY (NAMED_VALUE_INT OB_EXIT, GIAO_UOC mục
     * 6.3 và 9.4). Mã mới chỉ được THÊM VÀO CUỐI, không chèn giữa, không đổi
     * số cũ — mav_link.c có static assert giữ việc này.
     */
    OFFBOARD_EXIT_MODE_SW    /**< 9: ch6 rời nấc POSHOLD                */
} offboard_exit_t;

/* ==========================================================================
 * Vòng đời
 * ========================================================================== */

/** Đặt về LOCKED, xoá mục tiêu và bộ đếm. Gọi một lần lúc khởi động. */
void ctrl_offboard_init(void);

/**
 * Chạy máy trạng thái: đọc công tắc, kiểm tra hết hạn, kiểm tra cần ghi đè.
 * Gọi MỘT LẦN mỗi nhịp từ ctrl_angle_update(), TRƯỚC khi chọn chế độ bay.
 *
 * @param now_ms  mốc mili giây, cùng đồng hồ với ctrl_offboard_set_target()
 */
void ctrl_offboard_update(uint32_t now_ms);

/* ==========================================================================
 * Cửa vào — gọi từ lớp MAVLink
 * ========================================================================== */

/**
 * Nhận một setpoint vận tốc từ máy tính nhúng.
 *
 * Đây là CỬA DUY NHẤT vào vòng điều khiển. Phân công kiểm tra:
 *   - mav_link.c lọc phần thuộc về MAVLink: target_system, coordinate_frame,
 *     type_mask. Nó biết giao thức, module này thì không.
 *   - hàm này lọc phần thuộc về điều khiển: NaN/Inf và giới hạn bao.
 * Khung bị mav_link.c loại thì báo về đây bằng ctrl_offboard_note_reject()
 * để bộ đếm chẩn đoán nằm gọn một chỗ.
 *
 * Khung bị từ chối thì mục tiêu cũ GIỮ NGUYÊN và mốc thời gian KHÔNG được
 * làm mới — tức là khung hỏng đếm y như khung không tới, và sẽ tự dẫn tới hết
 * hạn nếu Pi cứ gửi rác. Đó là hành vi mong muốn: không cần thêm một đường
 * thoát riêng cho "lệnh hỏng".
 *
 * @param vx_body     vận tốc tới, m/s, hệ THÂN (dương = bay tới)
 * @param vy_body     vận tốc phải, m/s, hệ THÂN (dương = bay sang phải)
 * @param vz_ned      vận tốc theo trục Z hệ NED, m/s (DƯƠNG = ĐI XUỐNG)
 * @param yaw_rate    tốc độ yaw, rad/s (dương = quay phải)
 * @param now_ms      mốc mili giây lúc nhận khung
 * @return false nếu khung bị từ chối
 */
bool ctrl_offboard_set_target(float vx_body, float vy_body, float vz_ned,
                              float yaw_rate, uint32_t now_ms);

/** Ghi nhận một khung bị lớp MAVLink loại (sai target / frame / type_mask). */
void ctrl_offboard_note_reject(void);

/**
 * Yêu cầu rời OFFBOARD ngay. Không làm gì nếu đang không ACTIVE.
 * Dùng cho những lớp phát hiện lỗi nằm ngoài module này — ví dụ ctrl_althold
 * thấy người lái chạm cần ga.
 */
void ctrl_offboard_request_exit(offboard_exit_t reason);

/* ==========================================================================
 * Cửa ra — gọi từ các vòng điều khiển
 * ========================================================================== */

/** Có đang lái theo lệnh máy tính nhúng hay không. */
bool ctrl_offboard_is_active(void);

/**
 * Công tắc OFFBOARD (ch8) đang bật VÀ sóng RC đọc được.
 *
 * Đây là thứ đổi nghĩa của công tắc ARM trong arming.c: bật thì ch5 không còn
 * là "arm ngay" mà thành "cho phép Pi arm".
 */
bool ctrl_offboard_switch_on(void);

/**
 * Máy tính nhúng có quyền ra lệnh xuống không — gồm cả ARM và DISARM.
 *
 * Đúng khi công tắc ch8 bật và trạng thái KHÔNG phải KHOA. Mất quyền khi:
 *   - người lái chạm cần sau khi đã arm  (DAY_CAN)
 *   - mất sóng                           (MAT_SONG)
 *   - các lỗi khác đưa về KHOA
 * Lấy lại quyền CHỈ bằng cách gạt ch8 xuống rồi lên lại.
 */
bool ctrl_offboard_pi_has_authority(void);

/** Mục tiêu vận tốc ngang hệ THÂN, m/s. x = tới, y = phải. */
vec3f_t ctrl_offboard_velocity_body(void);

/** Mục tiêu tốc độ LÊN, m/s (dương = lên). Đã đảo dấu khỏi quy ước NED. */
float ctrl_offboard_climb_mps(void);

/** Mục tiêu tốc độ yaw, độ/giây. */
float ctrl_offboard_yaw_rate_dps(void);

/* ==========================================================================
 * Chẩn đoán — phục vụ CLI và log
 * ========================================================================== */

offboard_state_t ctrl_offboard_state(void);
offboard_exit_t  ctrl_offboard_last_exit(void);

/** Số khung hợp lệ đã nhận. */
uint32_t ctrl_offboard_accepted(void);

/** Số khung bị từ chối (NaN, sai type_mask, sai frame). */
uint32_t ctrl_offboard_rejected(void);

/** Số khung phải kẹp dải vì vượt giới hạn bao. */
uint32_t ctrl_offboard_clamped(void);

/** Số mili giây kể từ setpoint hợp lệ gần nhất. */
uint32_t ctrl_offboard_age_ms(uint32_t now_ms);

const char *ctrl_offboard_state_name(offboard_state_t s);
const char *ctrl_offboard_exit_name(offboard_exit_t r);

#endif /* CTRL_OFFBOARD_H */
