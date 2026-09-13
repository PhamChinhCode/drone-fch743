/**
 * @file    ctrl_althold.h
 * @brief   Giữ độ cao: cần ga điều khiển TỐC ĐỘ LÊN thay vì lực đẩy.
 *
 * VÌ SAO CẦN
 *
 *   Không có nó thì mọi chế độ, kể cả POSHOLD, đều để ga đi thẳng từ cần
 *   xuống khâu trộn. Máy bay giữ được ngang bằng và giữ được vị trí ngang,
 *   nhưng độ cao vẫn phải rà bằng tay suốt chuyến — mà lực nâng thay đổi theo
 *   điện áp pin, theo nhiệt độ không khí và theo hiệu ứng đệm khí gần mặt đất,
 *   nên "ga treo" không phải một con số cố định.
 *
 * CẤU TRÚC — HAI VÒNG LỒNG NHAU, GIỐNG HỆT VÒNG TƯ THẾ
 *
 *       cần ga ──> tốc độ lên mong muốn ──> PID ──> ga
 *                        ▲                            │
 *              (P trên sai số độ cao)                 │
 *                        └── độ cao, tốc độ lên ──────┘
 *
 *   Vòng ngoài chỉ có P: sai số độ cao đổi ra tốc độ lên mong muốn.
 *   Vòng trong là PID đầy đủ trên tốc độ lên, cho ra lượng ga.
 *
 *   Cùng một lý lẽ với ANGLE/ACRO: vòng ngoài chậm và hiền, vòng trong nhanh
 *   và mạnh. Gộp làm một vòng PID trên độ cao thì D phải lấy đạo hàm bậc hai
 *   của một tín hiệu có baro trong đó — tức khuếch đại nhiễu lên rất nhiều.
 *
 * CẦN GA LÀM GÌ
 *
 *   Ở GIỮA (trong vùng chết) -> giữ nguyên độ cao đang có.
 *   Ra khỏi vùng chết       -> lên/xuống với tốc độ tỉ lệ, và mốc độ cao
 *                              BÁM THEO độ cao hiện tại. Thả cần ra là nó
 *                              chốt ngay tại chỗ vừa tới.
 *
 *   Nếu mốc không bám theo lúc đang đẩy cần, thì mỗi lần lên cao 5 m rồi thả
 *   tay, máy bay sẽ lao ngược xuống chỗ cũ.
 *
 * CHUYỂN VÀO MƯỢT (BUMPLESS)
 *
 *   Lúc vừa bật chế độ, tích phân được nạp sẵn sao cho đầu ra bằng ĐÚNG mức
 *   ga người lái đang giữ. Không có bước này thì đầu ra nhảy về "ga treo mặc
 *   định" ngay khoảnh khắc gạt công tắc — tức một cú giật lực đẩy giữa không
 *   trung, đúng lúc người lái vừa buông quyền điều khiển.
 *
 * AN TOÀN
 *
 *   Mất tin cậy độ cao (baro hỏng, EKF chưa hội tụ) thì hàm trả false và bên
 *   gọi PHẢI trả quyền ga về cho cần. Giữ độ cao theo một ước lượng sai là
 *   cách chắc chắn nhất để đâm xuống đất hoặc bay mất hút lên trời.
 */
#ifndef CTRL_ALTHOLD_H
#define CTRL_ALTHOLD_H

#include "fc_types.h"

void ctrl_althold_init(void);

/**
 * Nạp lại trạng thái khi VÀO chế độ giữ độ cao.
 *
 * @param throttle_now  mức ga đang phát ra (0..1). Tích phân được nạp để đầu
 *                      ra khớp đúng giá trị này — xem "chuyển vào mượt".
 */
void ctrl_althold_enter(float throttle_now);

/** Xoá trạng thái. Gọi khi rời chế độ hoặc khi disarm. */
void ctrl_althold_reset(void);

/**
 * Chạy một nhịp.
 *
 * @param dt            bước thời gian, giây
 * @param throttle_out  nhận mức ga 0..1 khi trả về true
 * @return false nếu không đủ điều kiện giữ độ cao — bên gọi phải trả ga về cần
 */
bool ctrl_althold_update(float dt, float *throttle_out);

/* --- Soi bằng mắt trên console --- */
float ctrl_althold_target_m(void);       /**< mốc độ cao đang giữ        */
float ctrl_althold_climb_target(void);   /**< tốc độ lên mong muốn, m/s  */
float ctrl_althold_integral(void);       /**< phần tích phân của ga      */

/**
 * Cần ga có đang nằm trong vùng chết quanh điểm giữa không.
 *
 * Một nguồn sự thật duy nhất cho câu hỏi "người lái có đang chạm cần ga
 * không". Dùng chung bởi vòng giữ độ cao, bởi OFFBOARD (chạm cần = thoát),
 * và bởi arming.c (Pi chỉ được arm khi ga ở giữa). Không phụ thuộc chế độ
 * bay đang chạy — chỉ đọc cần và tham số.
 */
bool ctrl_althold_stick_centred(void);

#endif /* CTRL_ALTHOLD_H */
