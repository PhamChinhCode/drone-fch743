/**
 * @file    mixer.h
 * @brief   Khâu trộn quad X: đổi lệnh điều khiển thành mức ga bốn motor.
 *
 * BỐ CỤC (đo thực tế trên bo này, KHÔNG theo quy ước mặc định của hãng nào):
 *
 *      trước-trái = M2        trước-phải = M1
 *      sau-trái   = M3        sau-phải   = M4
 *
 *   Đường chéo {M1, M3} quay một chiều, {M2, M4} quay chiều ngược lại.
 *
 * CÔNG THỨC:
 *   out[i] = ga + roll·Kr[i] + pitch·Kp[i] + yaw·Ky[i]
 *
 *   Roll dương = nghiêng phải, nên hai motor bên PHẢI (M1, M4) giảm ga và hai
 *   motor bên TRÁI (M2, M3) tăng ga.
 *   Pitch dương = ngóc mũi lên, nên hai motor TRƯỚC (M1, M2) tăng và hai motor
 *   SAU (M3, M4) giảm. Motor chỉ đẩy LÊN được, nên nó là bập bênh: đẩy mạnh
 *   phía trước thì đầu trước bị nâng, mũi ngóc lên.
 *   Yaw dương = mũi quay phải. Mô-men phản lực ngược chiều cánh quạt nên dấu
 *   phụ thuộc chiều quay thật — xem MIX_YAW_SIGN trong fc_config.h.
 *
 * XỬ LÝ BÃO HOÀ — phần quan trọng nhất của module này:
 *
 *   Cách ngây thơ là cộng hết rồi cắt về [0,1]. Làm vậy thì khi một motor
 *   chạm trần, phần lệnh điều khiển của nó bị mất, còn ba cái kia vẫn nhận
 *   đủ — tỉ lệ giữa bốn motor sai đi và máy bay tự nghiêng theo hướng không
 *   ai yêu cầu. Đúng lúc cần điều khiển nhất (ga cao, lệnh lớn) thì nó lại
 *   phản ứng sai nhất.
 *
 *   Ở đây làm khác: tính riêng phần điều khiển trước, đo biên độ của nó, nếu
 *   vượt quá 1 thì thu nhỏ TOÀN BỘ theo cùng một hệ số. Bốn motor mất quyền
 *   lực như nhau nên TỈ LỆ giữa chúng được giữ nguyên — máy bay vẫn nghiêng
 *   đúng hướng, chỉ là yếu hơn. Sau đó mới đẩy ga vào phần còn trống.
 *
 *   Hệ quả có chủ ý: khi hết dư địa, GA bị hy sinh trước, quyền điều khiển
 *   tư thế được giữ. Máy bay mất độ cao còn cứu được; mất điều khiển thì không.
 */
#ifndef MIXER_H
#define MIXER_H

#include "fc_types.h"
#include "fc_config.h"

void mixer_init(void);

/**
 * Dung lai bang tron theo mix_yaw_sign trong g_params.
 *
 * Bang tron la mot BAN SAO dan xuat (giu dung san de vong nong khong phai
 * nhan them), nen phai goi lai moi khi dau yaw doi. param_apply.c lo viec do.
 */
void mixer_apply_params(void);

/**
 * Đọc g_fc.ctrl (pid_output + throttle_cmd) rồi ghi g_fc.motor.output_norm[].
 * Gọi trong vòng lặp chính, TRƯỚC dshot_update().
 *
 * Chưa arm thì ghi 0 cho cả bốn và thoát — khâu trộn không bao giờ là nơi
 * quyết định motor có quay hay không, việc đó thuộc về arming.c và dshot.c.
 */
void mixer_update(void);

/** true khi lần trộn gần nhất phải thu nhỏ lệnh vì hết dư địa. */
bool mixer_saturated(void);

/** Hệ số thu nhỏ của lần trộn gần nhất (1,0 = không phải thu nhỏ). */
float mixer_scale(void);

#endif /* MIXER_H */
