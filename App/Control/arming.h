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

/** Kết quả một lệnh arm/disarm đến từ máy tính nhúng. */
typedef enum {
    ARMING_LINK_OK = 0,        /**< đã làm, hoặc vốn đã ở trạng thái đó     */
    ARMING_LINK_NO_AUTHORITY,  /**< Pi không có quyền: ch8/ch5 chưa đúng,
                                    hoặc người lái đã giành lái             */
    ARMING_LINK_BLOCKED        /**< có quyền nhưng chưa đủ điều kiện arm    */
} arming_link_result_t;

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

/*
 * --- LỆNH ARM / DISARM TỪ MÁY TÍNH NHÚNG --------------------------------------
 *
 * CHẾ ĐỘ PI: bật khi công tắc OFFBOARD (ch8) đang ON. Lúc đó công tắc ARM
 * (ch5) đổi nghĩa từ "arm ngay" thành "cho phép Pi arm". Quy trình:
 *
 *   1. Đưa cần ga về GIỮA   (chế độ Pi đòi ga ở giữa, không phải ở thấp)
 *   2. Gạt ch8 lên          (PHẢI TRƯỚC ch5 — xem dưới)
 *   3. Gạt ch5 lên          -> FC không tự arm, đứng chờ
 *   4. Pi gửi lệnh ARM      -> arm
 *
 * Sai thứ tự (ch5 trước ch8) thì nhánh arm tay chạy, thấy ga không thấp ->
 * khoá. Vô hại, gạt ch5 một vòng là xong.
 *
 * Pi MẤT QUYỀN (cả arm lẫn disarm) khi người lái chạm bất kỳ cần nào sau khi
 * đã arm, hoặc mất sóng. Lấy lại quyền CHỈ bằng cách gạt ch8 xuống rồi lên.
 * Người lái thì LUÔN cắt được bằng ch5, bất kể Pi có quyền hay không.
 *
 * VÌ SAO KHÔNG GỌI THẲNG fc_state_set_mode():
 *   Gọi thẳng thì g_fc.mode đổi nhưng s_state không đổi theo. Máy trạng thái
 *   và trạng thái thật lệch nhau, và nhánh "công tắc về OFF" không còn chạy
 *   nữa — tức công tắc trên tay điều khiển mất tác dụng.
 */

/** Pi yêu cầu arm. Chỉ thành công ở chế độ Pi, đúng quy trình trên. */
arming_link_result_t arming_request_arm_link(void);

/**
 * true nếu gửi ARM từ Pi NGAY LÚC NÀY sẽ được ACCEPTED. Phát lên dây thành
 * NAMED_VALUE_INT OB_ARM_RDY (GIAO_UOC mục 6.3).
 *
 * Dùng CHUNG đúng một hàm kiểm với arming_request_arm_link(), nên hai thứ không
 * thể lệch nhau trong cùng một vòng lặp. Ngoại lệ duy nhất là thời gian: giá
 * trị lên dây 2 Hz, trong nửa giây đó người lái vẫn có thể chạm cần, và lúc
 * lệnh ARM tới nơi thì FC kiểm lại từ đầu. COMMAND_ACK là câu trả lời cuối.
 *
 * false khi đã arm.
 */
bool arming_link_arm_ready(void);

/**
 * Lý do chưa sẵn sàng arm: bitmask fc_arm_block_t. Phát lên dây thành
 * NAMED_VALUE_INT OB_ARM_BLK. Trả 0 khi đã arm — lúc đó các cờ chặn arm không
 * còn nghĩa gì.
 */
uint32_t arming_link_arm_block(void);

/**
 * Pi yêu cầu disarm. Sau khi disarm, máy trạng thái về SAFE (không khoá) để
 * Pi arm lại được mà người lái không phải làm gì.
 *
 * Cổng độ cao (GIAO_UOC 11.1 #12): chỉ disarm khi máy bay gần đất — xem
 * LINK_DISARM_* trong fc_config.h. Cao hơn thì trả ARMING_LINK_BLOCKED.
 * force = true (Pi gửi param2 = 21196) bỏ qua cổng độ cao, KHÔNG bỏ qua quyền.
 */
arming_link_result_t arming_request_disarm(bool force);

/**
 * true nếu gửi DISARM thường (không ép) NGAY LÚC NÀY sẽ được ACCEPTED VÀ thật
 * sự cắt động cơ. Phát lên dây thành NAMED_VALUE_INT OB_DIS_RDY.
 *
 * Cùng một hàm kiểm với arming_request_disarm(), cùng cách hiểu với
 * OB_ARM_RDY: false khi chưa arm hoặc Pi không có quyền — dù DISARM lúc chưa
 * arm vẫn trả ACCEPTED (không có gì để cắt, 6.2).
 */
bool arming_link_disarm_ready(void);

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
