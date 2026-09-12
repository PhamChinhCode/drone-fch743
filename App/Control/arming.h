/**
 * @file    arming.h
 * @brief   Bộ giám sát arm/disarm — quyết định khi nào máy bay được phép bay.
 *
 * VÌ SAO TÁCH RIÊNG:
 *   fc_state.c đã có sẵn `fc_state_check_arm()` (liệt kê điều kiện chặn) và
 *   `fc_state_set_mode()` (bảng chuyển trạng thái hợp lệ), nhưng cả hai đều
 *   thụ động — không có gì gọi chúng. Module này là phần chủ động: đọc công
 *   tắc trên tay điều khiển, lọc nhiễu, rồi ra lệnh chuyển trạng thái.
 *
 * BA TRẠNG THÁI:
 *
 *      +-----------+  công tắc về OFF   +----------+  công tắc lên ON   +-------+
 *      |  LOCKED   | -----------------> |   SAFE   | -----------------> | ARMED |
 *      +-----------+                    +----------+   (đủ điều kiện)   +-------+
 *            ^                               ^                              |
 *            |  mất sóng, hoặc bật công tắc  |      công tắc về OFF         |
 *            |  lúc chưa đủ điều kiện        +------------------------------+
 *            +----------------------------------------------------------+
 *
 *   LOCKED là trạng thái khởi đầu và là nơi mọi tình huống bất thường rơi về.
 *   Thoát khỏi LOCKED CHỈ có một cách: gạt công tắc về OFF. Đây là điểm mấu
 *   chốt của toàn bộ thiết kế, giải quyết ba tình huống nguy hiểm:
 *
 *     1. Cắm pin trong khi công tắc arm đang bật sẵn. Không có LOCKED thì
 *        máy bay arm ngay giây đầu tiên, cánh quạt quay khi tay bạn còn đang
 *        ở gần nó.
 *     2. Mất sóng giữa chừng rồi bắt lại được, công tắc vẫn đang ON. Không có
 *        LOCKED thì nó tự arm lại ngay khi sóng về.
 *     3. Bật công tắc lúc chưa đủ điều kiện (ga còn cao, gyro chưa hiệu
 *        chuẩn). Nếu chỉ chặn mà không khoá, máy bay sẽ tự arm đúng vào lúc
 *        điều kiện cuối cùng vừa thoả — không ai lường trước được thời điểm
 *        đó. Bắt gạt lại công tắc khiến hành động arm luôn là chủ ý của người
 *        lái. Betaflight cũng làm đúng như vậy.
 *
 * ĐIỀU KIỆN ARM:
 *   Uỷ quyền hoàn toàn cho fc_state_check_arm() — mất RC, đang failsafe, ga
 *   chưa về thấp, gyro chưa hiệu chuẩn, cảm biến hỏng, máy bay nghiêng quá,
 *   pin nguy hiểm. Module này chỉ thêm đúng một bit: ARM_BLOCK_SWITCH khi
 *   đang ở LOCKED.
 *
 * MẤT SÓNG KHI ĐANG BAY:
 *   Chuyển ARMED -> FAILSAFE -> DISARMED, tức CẮT MOTOR. Với trạng thái hiện
 *   tại của firmware (chưa có bộ ước lượng, chưa có bộ điều khiển) thì đây là
 *   lựa chọn an toàn duy nhất — không có gì để giữ máy bay trên không cả.
 *   KHI NÀO có althold chạy được thì nên đổi thành hạ độ cao có kiểm soát;
 *   cắt motor ở độ cao 30 m là rơi tự do.
 *
 * PIN YẾU KHÔNG LÀM DISARM:
 *   Cố ý. Pin nguy hiểm chặn arm lúc còn dưới đất, nhưng không bao giờ cắt
 *   motor giữa không trung — disarm khi đang bay là rơi. Đó là việc của cảnh
 *   báo và của người lái, không phải của bộ giám sát này.
 */
#ifndef ARMING_H
#define ARMING_H

#include "fc_types.h"
#include "fc_config.h"

/* ==========================================================================
 * Kiểu dữ liệu
 * ========================================================================== */

typedef enum {
    ARMING_LOCKED = 0,  /**< khoá — phải gạt công tắc về OFF mới thoát  */
    ARMING_SAFE,        /**< công tắc đang OFF, sẵn sàng nhận lệnh arm  */
    ARMING_ARMED        /**< đang arm                                    */
} arming_state_t;

/** Vì sao lần disarm gần nhất xảy ra. */
typedef enum {
    DISARM_CAUSE_NONE = 0,
    DISARM_CAUSE_BOOT,      /**< khoá sẵn từ lúc khởi động             */
    DISARM_CAUSE_SWITCH,    /**< người lái gạt công tắc về OFF         */
    DISARM_CAUSE_FAILSAFE,  /**< mất sóng khi đang bay                 */
    DISARM_CAUSE_BLOCKED,   /**< bật công tắc lúc chưa đủ điều kiện    */
    DISARM_CAUSE_LINK       /**< lệnh disarm từ đường MAVLink           */
} arming_disarm_cause_t;

/* ==========================================================================
 * API
 * ========================================================================== */

/** Đặt về trạng thái khoá. Gọi một lần sau crsf_init(). */
void arming_init(void);

/**
 * Đọc công tắc, lọc nhiễu, quyết định arm/disarm. Gọi đều đặn trong vòng lặp
 * chính, NGAY SAU crsf_update() để luôn nhìn thấy khung RC mới nhất.
 * @param now_ms  mốc thời gian mili giây, lấy từ HAL_GetTick()
 */
void arming_update(uint32_t now_ms);

/**
 * Ra lệnh disarm từ bên ngoài (hiện tại: MAV_CMD_COMPONENT_ARM_DISARM trên
 * UART8). Đây là lối vào DUY NHẤT cho phép mã ngoài module này disarm.
 *
 * VÌ SAO KHÔNG GỌI THẲNG fc_state_set_mode(FC_MODE_DISARMED):
 *   Gọi thẳng thì g_fc.mode về DISARMED nhưng s_state vẫn kẹt ở ARMING_ARMED.
 *   Máy trạng thái và trạng thái thật lệch nhau, và nhánh "công tắc về OFF"
 *   không còn chạy nữa — tức công tắc trên tay điều khiển mất tác dụng.
 *
 * SAU KHI GỌI, MÁY TRẠNG THÁI VỀ LOCKED: người lái phải gạt công tắc về OFF
 * rồi bật lại mới bay tiếp được. Cố ý làm vậy — một lệnh cắt từ máy tính
 * nhúng không được phép tự phục hồi khi công tắc vẫn đang ON.
 *
 *  true nếu sau lệnh này máy bay chắc chắn đã disarm.
 */
bool arming_request_disarm(void);

arming_state_t        arming_get_state(void);
arming_disarm_cause_t arming_last_disarm_cause(void);

/** Trạng thái công tắc sau khi đã lọc nhiễu (true = đang ở vị trí ON). */
bool arming_switch_on(void);

uint32_t arming_arm_count(void);
uint32_t arming_disarm_count(void);

/* --- Chuỗi mô tả, phục vụ console và log --------------------------------- */

const char *arming_state_name(arming_state_t state);
const char *arming_cause_name(arming_disarm_cause_t cause);

/**
 * Lý do chặn arm cấp thiết nhất, dạng chữ đọc được.
 * Trả về "san sang" khi không còn gì chặn.
 */
const char *arming_block_reason(void);

#endif /* ARMING_H */
