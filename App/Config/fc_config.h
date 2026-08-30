/**
 * @file    fc_config.h
 * @brief   Hằng số cấu hình toàn hệ thống (compile-time).
 *
 * Chỉ chứa #define. Không include HAL, không khai báo biến.
 * Đây là nơi duy nhất để chỉnh số lượng motor, tần số vòng lặp,
 * tốc độ telemetry... nhằm tránh "magic number" rải rác trong code.
 */
#ifndef FC_CONFIG_H
#define FC_CONFIG_H

/* ==========================================================================
 * Phần cứng — bám theo FCH743_V1.0 (STM32H743VIT6, LQFP100)
 * ========================================================================== */
#define FC_BOARD_NAME "FCH743_V1.0"
#define FC_FIRMWARE_VERSION_MAJOR 0
#define FC_FIRMWARE_VERSION_MINOR 1
#define FC_FIRMWARE_VERSION_PATCH 0

/* Clock thực tế sau SystemClock_Config() */
#define FC_SYSCLK_HZ 480000000UL
#define FC_TIMER_CLK_HZ 240000000UL /* TIM1..TIM8 (APB x2) */

/* ==========================================================================
 * Vòng lặp điều khiển
 * ========================================================================== */
#define FC_LOOP_RATE_HZ 4000 /* vòng lặp gyro/rate    */
#define FC_LOOP_PERIOD_US (1000000UL / FC_LOOP_RATE_HZ)

#define FC_ATTITUDE_RATE_HZ 1000   /* vòng ước lượng góc    */
#define FC_ALTITUDE_RATE_HZ 100    /* vòng giữ độ cao       */
#define FC_HOUSEKEEPING_RATE_HZ 50 /* pin, LED, telemetry   */

/* ==========================================================================
 * Động cơ / DShot  (TIM1 @ 240 MHz, ARR = 399)
 * ========================================================================== */
#define FC_MOTOR_COUNT 4
/*
 * Tốc độ DShot. Đây là NGUỒN SỰ THẬT DUY NHẤT — dshot_init() ghi ARR của TIM1
 * theo DSHOT_ARR, nên KHÔNG cần sửa Period trong CubeMX.
 *
 *   600000 -> DShot600, ARR= 399, T0H= 150, T1H= 300, chu kỳ bit 1,667 µs
 *   300000 -> DShot300, ARR= 799, T0H= 300, T1H= 600, chu kỳ bit 3,333 µs
 *   150000 -> DShot150, ARR=1599, T0H= 600, T1H=1200, chu kỳ bit 6,667 µs
 *
 * ĐANG DÙNG DShot300 — ĐÃ THỬ THỰC TẾ trên bộ ESC BLHeli_S + pin 3S của bo
 * này. Kết quả đo được, đừng đổi mà không thử lại:
 *
 *     DShot600   motor giật rồi dừng, lặp đi lặp lại   KHÔNG DÙNG ĐƯỢC
 *     DShot300   chạy ổn định                          <-- đang dùng
 *     DShot150   giật y hệt DShot600                   KHÔNG DÙNG ĐƯỢC
 *
 * Có một CỬA SỔ hoạt động, chậm hơn không đồng nghĩa an toàn hơn. Hai đầu
 * hỏng vì hai lý do khác nhau:
 *
 *   - DShot600 quá NHANH cho BLHeli_S. Nó chạy trên EFM8 8-bit, chỉ có
 *     1,67 µs để phân biệt xung 0,625 µs với 1,25 µs.
 *   - DShot150 quá CHẬM nên bị nhận nhầm giao thức. Xung của nó rộng
 *     2,50 µs và 5,00 µs, rơi trúng dải Multishot (5–25 µs) mà BLHeli_S dò
 *     lúc khởi động. Bị đọc thành Multishot thì ra mức ga hoàn toàn vô nghĩa.
 *
 * ĐỔI TỐC ĐỘ THÌ PHẢI RÚT PIN CẮM LẠI CHO ESC. BLHeli_S dò giao thức MỘT LẦN
 * lúc nó khởi động. Nạp lại firmware mạch bay chỉ reset vi điều khiển, còn
 * ESC vẫn đang chạy và vẫn giữ giao thức nó dò được từ trước — không ngắt
 * nguồn ESC thì sửa hằng số này chẳng có tác dụng gì.
 *
 * ESC dùng BLHeli_32 hoặc AM32 thì đặt 600000 được thoải mái.
 */
#define DSHOT_BITRATE_HZ 300000UL                            /* DShot300              */
#define DSHOT_ARR ((FC_TIMER_CLK_HZ / DSHOT_BITRATE_HZ) - 1) /* 799 */
#define DSHOT_T0H (((DSHOT_ARR + 1) * 375) / 1000)           /* 300 */
#define DSHOT_T1H (((DSHOT_ARR + 1) * 750) / 1000)           /* 600 */
#define DSHOT_FRAME_BITS 16
#define DSHOT_GAP_BITS 2                                     /* giữ đường về mức thấp */
#define DSHOT_BUFFER_LEN (DSHOT_FRAME_BITS + DSHOT_GAP_BITS) /* 18  */

#define DSHOT_MIN_THROTTLE 48 /* 0..47 là lệnh đặc biệt */
#define DSHOT_MAX_THROTTLE 2047

/*
 * Mức ga khi đã arm nhưng cần ga ở đáy (%).
 *
 * KHÔNG dùng thẳng DSHOT_MIN_THROTTLE (48) cho mức này. 48 là giá trị ga nhỏ
 * nhất mà giao thức biểu diễn được, không phải mức nhỏ nhất motor quay được.
 * Ở 48 motor thiếu mô-men để khởi động sạch: nó giật cục từng nhịp, và động
 * cơ nào ma sát nhỉnh hơn sẽ đứng im hẳn rồi ESC báo stall bằng tiếng bíp.
 *
 * 5,5 % là mặc định của Betaflight (dshot_idle_value 550) và là điểm khởi đầu
 * tốt. Motor to hoặc cánh nặng có thể cần 7-8 %. Chỉnh cho tới khi cả bốn
 * motor quay đều và êm ngay khi arm.
 */
#define DSHOT_IDLE_PERCENT 5.5f

/*
 * Chốt an toàn cho chế độ quay thử từng motor.
 * Trần ga cố ý đặt thấp: quay thử là để nghe tiếng và nhìn chiều quay, không
 * phải để chạy hết công suất. Thời gian tối đa chặn trường hợp code gọi rồi
 * quên tắt.
 */
#define DSHOT_TEST_MAX_PERCENT 12.0f
#define DSHOT_TEST_MAX_MS 2000

/*
 * Nhịp phát khung. Một khung DShot600 dài 18 bit × 1,667 µs ≈ 30 µs nên
 * 1 kHz còn rất nhiều dư địa. Điều quan trọng là nhịp phải ĐỀU: ESC coi như
 * mất tín hiệu khi ngừng nhận khung vài chục mili giây.
 *
 * TẠM THỜI vòng lặp chính giữ nhịp này. Khi nào có vòng điều khiển chạy ở
 * FC_LOOP_RATE_HZ thì chuyển dshot_update() vào đó và bỏ phần tự đếm thời
 * gian trong driver — lúc ấy khung phát đồng bộ với chu kỳ PID thay vì lệch
 * pha ngẫu nhiên như bây giờ.
 */
#define DSHOT_UPDATE_RATE_HZ 1000
#define DSHOT_PERIOD_US (1000000UL / DSHOT_UPDATE_RATE_HZ)

/* ESC bỏ qua lệnh đặc biệt chỉ gửi một lần; phải lặp lại mới ăn. */
#define DSHOT_CMD_REPEAT 10

/*
 * Motor thứ n ra kênh TIM1 nào:
 *   0 = CH1 (PE9)   1 = CH2 (PE11)   2 = CH3 (PE13)   3 = CH4 (PE14)
 * Đổi bốn số này để sửa thứ tự motor bằng phần mềm thay vì tháo đổi dây.
 */
#define DSHOT_MOTOR_MAP_1 0
#define DSHOT_MOTOR_MAP_2 1
#define DSHOT_MOTOR_MAP_3 2
#define DSHOT_MOTOR_MAP_4 3

/*
 * Vị trí thực tế trên khung, ĐÃ ĐO trên bo này bằng nút quay thử K1:
 *
 *      trước-trái  = M2        trước-phải = M1
 *      sau-trái    = M3        sau-phải   = M4
 *
 * Hai đường chéo là {1,3} và {2,4}. Hai motor cùng một đường chéo PHẢI quay
 * cùng chiều, hai đường chéo quay ngược nhau — có vậy mô-men phản lực mới
 * triệt tiêu và mới điều khiển được yaw.
 */

/*
 * Bitmask các motor sẽ bị đảo chiều khi bấm nút K2.
 *   bit0 = motor 1, bit1 = motor 2, bit2 = motor 3, bit3 = motor 4
 *
 * Đặt 0x0A (motor 2 và 4) -> nạp firmware -> THÁO CÁNH QUẠT -> bấm K2 một
 * lần -> nghe ESC bíp xác nhận đã lưu -> ĐẶT LẠI VỀ 0x00 rồi nạp lại.
 *
 * Để nguyên khác 0 thì mỗi lần bấm nhầm K2 lại đảo chiều thêm lần nữa.
 *
 * LƯU Ý: BLHeli_S đời cũ KHÔNG hiểu lệnh DShot. Bấm K2 mà chiều quay không
 * đổi thì dùng BLHeliSuite, hoặc đơn giản là hoán hai trong ba dây pha của
 * motor đó — cách vật lý luôn hiệu quả và vĩnh viễn.
 */
#define DSHOT_REVERSE_MASK 0x00

/*
 * GHI LẠI CHO LẦN SAU — đã thử trên chính bộ phần cứng này:
 *
 * Bộ ESC BLHeli_S ở đây KHÔNG hiểu lệnh DShot; bấm K2 không có tác dụng.
 * Chiều quay đã sửa bằng cách HOÁN DÂY PHA của motor 2 và motor 4.
 * Đã kiểm lại: {M1, M3} cùng chiều, {M2, M4} cùng chiều, hai cặp ngược nhau.
 *
 * Nút K2 và dshot_reverse_motors() vẫn giữ lại vì chúng đúng theo chuẩn và
 * sẽ dùng được nếu sau này đổi sang ESC chạy BLHeli_32 hoặc AM32.
 */

/* ==========================================================================
 * Khâu trộn (mixer) — quad X
 *
 * Bố cục lấy từ số đo thực tế ở trên:
 *      trước-trái = M2        trước-phải = M1
 *      sau-trái   = M3        sau-phải   = M4
 * ========================================================================== */

/*
 * Dấu của trục yaw. Mô-men phản lực tác dụng lên thân NGƯỢC chiều cánh quạt,
 * nên dấu này phụ thuộc đường chéo nào quay theo chiều nào — không suy ra
 * được từ code, phải nhìn.
 *
 *   +1 nếu cặp {M1, M3} quay NGƯỢC chiều kim đồng hồ (nhìn từ trên xuống)
 *   -1 nếu cặp {M1, M3} quay THUẬN chiều kim đồng hồ
 *
 * SUY RA TỪ ĐỊNH LUẬT III NEWTON, không phải quy ước tuỳ ý:
 *   Cánh quay THUẬN chiều -> nó tác dụng mô-men NGƯỢC chiều lên thân
 *   -> mũi quay sang TRÁI. Vậy muốn mũi sang PHẢI (yaw dương) thì phải tăng
 *   ga các motor quay NGƯỢC chiều. MIX_YAW_SIGN = +1 làm pid_output.z dương
 *   tăng {M1,M3}, nên nó đúng khi và chỉ khi {M1,M3} quay ngược chiều.
 *
 * (Chú thích cũ ở đây ghi ngược — đã sửa. Nếu bạn từng chọn dấu dựa vào nó
 *  thì kiểm lại bằng phép thử mô tả bên dưới.)
 *
 * CÁCH KIỂM AN TOÀN, KHÔNG CẦN CÁNH QUẠT: tháo cánh, arm, bật DBG_MODE_MOTOR,
 * đẩy cần yaw sang PHẢI rồi xem cột m1..m4. Muốn mũi quay sang phải thì mô-men
 * phản lực phải hướng thuận chiều kim đồng hồ, tức cặp quay NGƯỢC chiều kim
 * đồng hồ phải tăng ga. Nếu console cho thấy cặp sai đang tăng thì đảo dấu này.
 */
#define MIX_YAW_SIGN (+1)

/*
 * CHỈ DÙNG LÚC BRING-UP — ĐẶT VỀ 0 TRƯỚC KHI LẮP CÁNH QUẠT.
 *
 * Bằng 1 thì cần điều khiển đi THẲNG vào khâu trộn, bỏ qua toàn bộ PID. Nhờ
 * vậy kiểm được dấu của khâu trộn trước khi có vòng điều khiển: tháo cánh,
 * arm, đẩy từng cần và xem cột m1..m4 trên console đổi đúng hướng không.
 *
 * Đây KHÔNG phải chế độ bay. Không có PID thì không có gì giữ thăng bằng —
 * lắp cánh quạt vào mà bật cái này là máy bay lật ngay khi rời đất.
 */
#define MIX_STICK_PASSTHROUGH 0
#define MIX_PASSTHROUGH_GAIN 0.35f /* để tay nhẹ, chỉ cần thấy hướng đổi */

/* ==========================================================================
 * Vòng PID tốc độ góc (chế độ ACRO)
 *
 * Vòng trong cùng và quan trọng nhất: nó bám tốc độ quay mà người lái yêu cầu.
 * Mọi chế độ khác (ANGLE, ALTHOLD) đều chồng lên vòng này.
 * ========================================================================== */

/* Tốc độ góc tối đa khi cần đẩy hết, độ/giây. */
#define RATE_MAX_ROLL_DPS 300.0f
#define RATE_MAX_PITCH_DPS 300.0f
#define RATE_MAX_YAW_DPS 250.0f

/*
 * HỆ SỐ PID — BẮT BUỘC PHẢI CHỈNH THEO KHUNG CỦA BẠN.
 *
 * Giá trị dưới đây đặt CỐ Ý THẤP. Máy bay sẽ ì và trôi, đó là chủ ý: hệ số
 * thấp làm máy bay khó lái nhưng an toàn, hệ số cao làm nó dao động và đập
 * xuống đất. Luôn chỉnh từ dưới lên, không bao giờ từ trên xuống.
 *
 * Đơn vị: sai số tính bằng độ/giây, đầu ra là phần trăm dải motor (-1..1).
 *   kp 0,0015 nghĩa là sai 300 độ/giây thì đẩy ra 0,45 tức 45 % dải.
 *
 * TRÌNH TỰ CHỈNH (xem hướng dẫn kèm theo):
 *   1. Chỉ P, tăng dần tới khi bắt đầu rung rồi lùi lại 30 %
 *   2. Thêm D để dập rung, tăng tới khi motor kêu rít thì lùi lại
 *   3. Thêm I sau cùng, chỉ đủ để hết trôi
 */
#define RATE_PID_ROLL_KP 0.0010f
#define RATE_PID_ROLL_KI 0.0001f
#define RATE_PID_ROLL_KD 0.00000001f

#define RATE_PID_PITCH_KP 0.0010f
#define RATE_PID_PITCH_KI 0.0001f
#define RATE_PID_PITCH_KD 0.00000001f

/* Trục yaw thường KHÔNG cần D: nó bị hãm sẵn bởi lực cản khí động của cánh. */
#define RATE_PID_YAW_KP 0.0002f
/*
 * Trục YAW là trục CẦN KHÂU I NHẤT, và cũng là trục duy nhất không có gì tự
 * kéo về vị trí cũ.
 *
 * Roll và pitch ở chế độ ANGLE được vòng ngoài kéo về ngang, lấy chuẩn từ
 * trọng lực. Yaw thì không có chuẩn nào — không la bàn, không GPS. Nên mọi
 * nhiễu loạn không đổi (motor lắp hơi nghiêng, cánh không đều, mô-men phản
 * lực hai đường chéo lệch nhau) đều biến thành TRÔI VĨNH VIỄN nếu chỉ có P.
 *
 * Lý do: P chỉ sinh ra đầu ra khi CÒN sai số. Muốn giữ đầu ra 1% để chống
 * nhiễu loạn thì phải chấp nhận sai số 20 °/s, tức máy bay quay 3,3 vòng/phút
 * mãi mãi. Khâu I tích luỹ cho tới khi tự nó sinh đủ đầu ra, lúc đó sai số
 * mới về 0 và máy bay mới đứng yên.
 *
 * Đặt bằng KP là điểm khởi đầu hợp lý. Còn trôi thì tăng dần; trôi rồi lắc
 * chậm biên độ lớn dần là dấu hiệu đã quá cao.
 */
#define RATE_PID_YAW_KI 0.0015f
#define RATE_PID_YAW_KD 0.0f

/*
 * Chặn tích phân. Không có nó thì khi máy bay bị giữ nghiêng (vướng tay, kẹt
 * chân đế) tích phân dồn lên vô hạn, và lúc thả ra máy bay lật úp vì phải xả
 * hết chỗ đã dồn.
 */
#define RATE_PID_I_LIMIT 0.25f
#define RATE_PID_OUT_LIMIT 1.0f

/*
 * Lọc thông thấp khâu vi phân. Đạo hàm khuếch đại nhiễu rất mạnh, không lọc
 * thì D biến thành máy phát nhiễu làm motor nóng và kêu rít.
 */
#define RATE_DTERM_LPF_HZ 80.0f

/* ==========================================================================
 * Vòng PID góc (chế độ ANGLE) — vòng NGOÀI
 *
 * Cấu trúc tầng: vòng này nhìn GÓC nghiêng, tính ra TỐC ĐỘ QUAY cần thiết để
 * kéo góc về mục tiêu, rồi giao con số đó xuống làm setpoint cho vòng tốc độ
 * góc. Nó không hề chạm tới motor — mọi thứ vẫn đi qua vòng trong.
 * ========================================================================== */

/* Góc nghiêng tối đa khi cần đẩy hết, độ. */
#define ANGLE_MAX_LEAN_DEG 30.0f

/*
 * Hệ số P của vòng góc, đơn vị: độ/giây yêu cầu trên mỗi độ sai lệch.
 * 5,0 nghĩa là nghiêng sai 10° thì vòng ngoài đòi quay về với 50 °/s.
 *
 * VÒNG NGOÀI CHỈ CÓ P, CỐ Ý KHÔNG CÓ I VÀ D:
 *   - D là thừa: vòng trong đã dập dao động rồi, thêm D ở đây là vi phân hai
 *     lần một tín hiệu vốn đã nhiễu.
 *   - I là nguy hiểm: khi máy bay bị giữ nghiêng (vướng tay, chạm vật), tích
 *     phân góc dồn lên, thả ra là nó lộn vòng để trả nợ.
 *   Sai lệch tĩnh của vòng góc do vòng trong lo, không cần I ở đây.
 */
#define ANGLE_PID_KP 5.0f

/*
 * Trần tốc độ quay mà vòng ngoài được phép yêu cầu.
 * Không có nó thì lệch 60° sẽ đòi 300 °/s và máy bay giật rất mạnh khi bạn
 * bật chế độ ANGLE lúc đang nghiêng nhiều.
 */
/*
 * Bu do lech lap dat IMU so voi khung, don vi DO.
 *
 * Cong thang vao goc muc tieu nen no bu duoc o ca ANGLE, ALTHOLD va POSHOLD.
 *
 * CACH DO: dat may bay len mat da kiem phang bang nivo (dung tin cai ban),
 * doc roll/pitch o `mode 16`, roi dat trim bang chinh so doc duoc DOI DAU.
 * Vi du doc ra roll +0,4 va pitch +0,9 thi dat -0,4 va -0,9.
 *
 * Dai +-10 do la rong hon nhieu so voi moi sai so lap dat hop ly - lech qua
 * vai do thi phai ke lai bo mach chu khong bu bang phan mem.
 */
#define ANGLE_TRIM_ROLL_DEG 1.0f
#define ANGLE_TRIM_PITCH_DEG 0.0f

#define ANGLE_MAX_RATE_DPS 200.0f

/*
 * Kênh chọn chế độ bay (đánh số từ 0). Đặt -1 để luôn dùng ANGLE.
 *
 * DƯỚI ngưỡng = ANGLE (tự cân bằng), TRÊN ngưỡng = ACRO.
 * Chiều này là có chủ ý: công tắc mất tín hiệu hay chưa gán thì rơi về ANGLE,
 * tức chế độ AN TOÀN hơn. Đừng đảo lại.
 */
#define RC_MODE_CHANNEL 5

/*
 * Ngưỡng phân nấc. Công tắc ba nấc qua CRSF chỉ cho ĐÚNG BA giá trị:
 *     nấc dưới = 172,  nấc giữa = 992,  nấc trên = 1811
 * nên ngưỡng phải nằm ở KHOẢNG GIỮA hai nấc liền kề, không phải một con số
 * tròn nào đó. Đặt 1300 thì nấc giữa (992) rơi tuột xuống ANGLE và chế độ
 * POSHOLD không bao giờ chọn được.
 *
 * Tính thẳng từ ba hằng số CRSF để nếu sau này đổi dải thì ngưỡng tự theo.
 */
/*
 * BA NẤC CÔNG TẮC, theo mức hỗ trợ tăng dần:
 *
 *     nấc THẤP  (172)  ->  ANGLE     tự cân bằng, ga bằng tay
 *     nấc GIỮA  (992)  ->  ALTHOLD   thêm giữ độ cao
 *     nấc CAO  (1811)  ->  POSHOLD   thêm giữ vị trí ngang
 *
 * Ngưỡng phải nằm ở KHOẢNG GIỮA hai nấc liền kề, không phải một con số tròn
 * nào đó: công tắc ba nấc chỉ cho ĐÚNG ba giá trị 172 / 992 / 1811.
 *
 * Thứ tự này giữ nguyên nguyên tắc an toàn cũ — kênh mất tín hiệu hoặc chưa
 * gán đều cho giá trị thấp, và giá trị thấp rơi vào chế độ được hỗ trợ nhiều
 * nhất mà không cần cảm biến nào ngoài IMU. Đừng đảo lại.
 *
 * Tính thẳng từ ba hằng số CRSF để nếu sau này đổi dải thì ngưỡng tự theo.
 */
#define RC_MODE_ALTHOLD_THRESHOLD ((RC_CRSF_CHANNEL_MIN + RC_CRSF_CHANNEL_MID) / 2) /* 582  */
#define RC_MODE_POSHOLD_THRESHOLD ((RC_CRSF_CHANNEL_MID + RC_CRSF_CHANNEL_MAX) / 2) /* 1401 */

/*
 * Ngưỡng ACRO trùng với ngưỡng POSHOLD là CÓ Ý.
 *
 * Mặc định `rc_mode_acro_enable = 0` nên dòng này không có tác dụng gì. Ai cố
 * ý bật ACRO lên thì nấc CAO đổi từ POSHOLD thành ACRO — vì thang chọn xét
 * ACRO trước. Đó là hành vi hợp lý cho người biết mình đang làm gì: nấc mạnh
 * nhất trở thành chế độ thô nhất.
 */
#define RC_MODE_ACRO_THRESHOLD ((RC_CRSF_CHANNEL_MID + RC_CRSF_CHANNEL_MAX) / 2) /* 1401 */

/*
 * Có cho phép chọn ACRO bằng công tắc hay không. 0 = KHÔNG (mặc định).
 *
 * ACRO không tự cân bằng — buông cần thì máy bay giữ nguyên góc nghiêng và
 * tiếp tục lật. Với một khung đang trong giai đoạn chỉnh, gạt nhầm vào đó là
 * mất máy bay.
 *
 * Tắt nấc này KHÔNG gỡ ACRO khỏi firmware: nó vẫn là chế độ dự phòng tự động
 * khi bộ ước lượng mất góc tin cậy, và đó là lựa chọn đúng vì ACRO là chế độ
 * duy nhất chạy được mà không cần biết góc.
 */
#define RC_MODE_ACRO_ENABLE 0

/* ==========================================================================
 * Giữ vận tốc bằng optical flow (chế độ POSHOLD)
 *
 * Vòng ngoài cùng: nhìn VẬN TỐC, ra GÓC NGHIÊNG, giao xuống vòng góc. Buông
 * cần thì nó ghì máy bay đứng yên so với mặt đất thay vì để trôi theo gió.
 * ========================================================================== */

/* Tốc độ tối đa khi đẩy cần hết. Đây không phải chế độ bay nhanh. */
#define POSHOLD_MAX_VEL_MPS 1.5f

/*
 * Độ nghiêng yêu cầu trên mỗi m/s sai số vận tốc.
 * 6°/(m/s) nghĩa là trôi 1 m/s thì nghiêng 6° để hãm, cho gia tốc hãm
 * g·tan(6°) ≈ 1,0 m/s² — dập hết 1 m/s trong khoảng một giây. Êm, không giật.
 */
#define POSHOLD_VEL_KP 8.0f

/*
 * Khâu I chống GIÓ. Gió thổi đều là một nhiễu loạn không đổi; chỉ có P thì
 * máy bay đứng ở một độ nghiêng cân bằng nhưng VẪN TRÔI đều — đúng bài toán
 * đã gặp ở trục yaw.
 */
#define POSHOLD_VEL_KI 0.0f
#define POSHOLD_I_LIMIT_DEG 8.0f

/* Trần nghiêng. Nghiêng nhiều thì flow bị cổng nghiêng từ chối, mất luôn
 * nguồn đo — nên trần này phải THẤP hơn EST_FLOW_MAX_TILT_DEG. */
#define POSHOLD_MAX_TILT_DEG 15.0f

/*
 * Vòng NGOÀI CÙNG: sai số vị trí -> vận tốc mong muốn.
 *
 * 1,0 nghĩa là trôi 1 m thì đòi bò về với 1 m/s. Giữ THẤP: đây là vòng ngoài
 * của một vòng ngoài (vị trí -> vận tốc -> góc -> tốc độ góc), và mỗi tầng
 * thêm vào một lượng trễ. Đặt cao thì máy bay đảo qua đảo lại quanh điểm giữ
 * với chu kỳ vài giây.
 *
 * Chỉnh SAU CÙNG, khi vòng vận tốc đã đứng yên gọn gàng.
 */
#define POSHOLD_POS_KP 0.0f

/* ==========================================================================
 * Giữ độ cao (ALTHOLD) — cần ga điều khiển TỐC ĐỘ LÊN thay vì lực đẩy
 *
 * Cấu trúc hai vòng lồng nhau, xem App/Control/ctrl_althold.h.
 *
 * TRÌNH TỰ CHỈNH — làm đúng thứ tự này, đừng nhảy cóc:
 *   1. Đo ALTHOLD_HOVER_THR trước tiên. Treo máy bay ở chế độ ANGLE, đọc cột
 *      `thr` trên console lúc nó đứng yên độ cao. KHÔNG đoán con số này.
 *   2. Chỉ ALTHOLD_CLIMB_KP, tăng dần tới khi bắt đầu nhấp nhô rồi lùi 30 %.
 *   3. Thêm ALTHOLD_CLIMB_KI cho tới khi hết trôi chậm theo pin yếu dần.
 *   4. ALTHOLD_ALT_KP sau cùng, và giữ THẤP — nó là vòng ngoài.
 * ========================================================================== */

/*
 * Ga treo — số hạng NUÔI TIẾN, gánh phần lớn công việc.
 *
 * 0,35 chỉ là chỗ khởi đầu cho một khung 5 inch thông thường. PHẢI đo lại
 * trên chính máy bay này: sai 10 % ở đây là tích phân phải bù 10 %, mất vài
 * giây, và trong vài giây đó máy bay lên hoặc xuống mất kiểm soát.
 */
#define ALTHOLD_HOVER_THR 0.35f

/* Điểm giữa của cần ga. Cần có lò xo về giữa thì để 0,5. */
#define ALTHOLD_STICK_CENTRE 0.5f

/*
 * Vùng chết quanh điểm giữa, tính theo nửa hành trình cần.
 *
 * Rộng hơn vùng chết thường của cần lái vì đây là chỗ người lái BUÔNG tay và
 * mong máy bay đứng yên. Cần ga rẻ tiền trôi vài phần trăm là chuyện thường,
 * và mỗi phần trăm trôi ở đây biến thành một lệnh leo dai dẳng.
 */
#define ALTHOLD_STICK_DEADBAND 0.10f

/* Tốc độ lên/xuống tối đa khi đẩy cần hết hành trình. */
#define ALTHOLD_MAX_CLIMB_MPS 2.0f

/*
 * Vòng ngoài: sai số độ cao -> tốc độ lên mong muốn.
 * 1,0 nghĩa là lệch 1 m thì đòi leo 1 m/s. Giữ thấp — vòng ngoài mà mạnh thì
 * nó đánh nhau với vòng trong và sinh dao động chu kỳ dài.
 */
#define ALTHOLD_ALT_KP 1.0f

/*
 * Vòng trong: sai số tốc độ lên -> lượng ga.
 * 0,10 nghĩa là lệch 1 m/s thì thêm 10 % dải ga.
 */
#define ALTHOLD_CLIMB_KP 0.25f
#define ALTHOLD_CLIMB_KI 0.03f
#define ALTHOLD_CLIMB_KD 0.01f

/*
 * Lọc khâu vi phân. Đặt THẤP hơn nhiều so với vòng tốc độ góc: tốc độ lên
 * suy ra từ baro nên vốn đã chậm và ồn, lấy đạo hàm nó mà không lọc mạnh thì
 * ra toàn nhiễu.
 */
#define ALTHOLD_DTERM_LPF_HZ 10.0f

/*
 * Chặn tích phân. Đây cũng chính là dư địa mà chuyển-vào-mượt được phép dùng,
 * nên đừng đặt quá nhỏ: người lái vào chế độ lúc đang giữ ga 0,6 mà ga treo
 * là 0,35 thì cần 0,25 dư địa.
 */
#define ALTHOLD_I_LIMIT 0.30f

/*
 * Trần và sàn ga khi đang giữ độ cao.
 *
 * Sàn KHÁC 0 là có chủ ý: để 0 thì một lần ước lượng độ cao sai có thể cắt
 * hẳn motor giữa không trung. 0,10 giữ cho cánh vẫn quay và máy bay rơi có
 * kiểm soát thay vì rơi tự do.
 */
#define ALTHOLD_THR_MIN 0.10f
#define ALTHOLD_THR_MAX 0.85f

/* ==========================================================================
 * Điều khiển từ xa (CRSF / ELRS 2.4G trên USART2 @ 420000)
 * ========================================================================== */
#define RC_CHANNEL_COUNT 16
#define RC_CRSF_CHANNEL_MIN 172 /* giá trị thô CRSF      */
#define RC_CRSF_CHANNEL_MID 992
#define RC_CRSF_CHANNEL_MAX 1811
#define RC_FAILSAFE_TIMEOUT_MS 500 /* mất tín hiệu -> failsafe */
#define RC_DEADBAND_NORM 0.02f     /* vùng chết quanh giữa  */

/*
 * Đệm DMA vòng tròn cho USART2. ELRS ở chế độ nhanh nhất (1000 Hz) phát
 * khoảng 26 kB/s, nên 512 byte tương ứng ~20 ms dữ liệu. Vòng lặp chính chỉ
 * cần gọi crsf_update() nhanh hơn mức đó là không mất byte nào.
 */
#define RC_RX_BUFFER_SIZE 512

/*
 * Kênh nào mang cần điều khiển nào (đánh số từ 0).
 * Mặc định AETR — thứ tự chuẩn của ELRS và cũng là mặc định của EdgeTX.
 * Nếu tay điều khiển của bạn đặt TAER thì đổi THROTTLE thành 0, ROLL 1,
 * PITCH 2, YAW 3. Xem console ở DBG_MODE_RC để biết kênh nào đang nhúc nhích.
 */
#define RC_CHANNEL_ROLL 0
#define RC_CHANNEL_PITCH 1
#define RC_CHANNEL_THROTTLE 2
#define RC_CHANNEL_YAW 3

/*
 * Đảo chiều từng cần. Đặt 1 để đảo, 0 để giữ nguyên.
 *
 * Firmware quy ước dấu DƯƠNG như sau — bám theo quy ước hàng không:
 *     roll  dương = nghiêng phải
 *     pitch dương = NGÓC MŨI LÊN
 *     yaw   dương = mũi quay phải
 *
 * Nhưng tay điều khiển thì tuỳ hãng và tuỳ cách gán kênh. Chỗ hay lệch nhất
 * là PITCH: đa số người lái muốn ĐẨY cần ra xa = bay TỚI = chúc mũi XUỐNG,
 * tức cần đẩy ra phải cho giá trị kênh THẤP. Radio nào gán ngược lại thì bật
 * RC_INVERT_PITCH lên 1.
 *
 * Sửa ở đây thay vì sửa trong radio là có chủ ý: đổi radio hay reset model
 * cũng không làm máy bay đổi hành vi.
 *
 * CẨN THẬN VỚI THROTTLE: đảo nhầm nghĩa là cần ga ở đáy thành ga tối đa.
 * Chỉ đổi khi đã tháo cánh quạt và đã kiểm bằng console.
 */
#define RC_INVERT_ROLL 0
#define RC_INVERT_PITCH 1 /* ĐÃ ĐO: radio này gán pitch ngược quy ước */
#define RC_INVERT_YAW 0
#define RC_INVERT_THROTTLE 0

/*
 * GHI LẠI: trên bộ tay điều khiển này, đẩy cần pitch ra xa (ý muốn bay tới,
 * tức chúc mũi xuống) lại làm giá trị CH2 TĂNG, mà firmware quy ước CH2 tăng
 * là ngóc mũi lên. Nên phải đảo. Roll thì khớp sẵn, không cần đảo — radio gán
 * chiều từng kênh độc lập nên lệch một kênh mà đúng kênh kia là bình thường.
 */

/* ==========================================================================
 * Cảm biến
 * ========================================================================== */
/* ICM20602 trên SPI1, DRDY = PC4 (EXTI4) */
#define IMU_SAMPLE_RATE_HZ 8000
#define IMU_GYRO_FS_DPS 2000        /* 250|500|1000|2000 °/s */
#define IMU_ACCEL_FS_G 16           /* 2|4|8|16 g            */
#define IMU_CALIB_SAMPLE_COUNT 2000 /* số mẫu lấy bias gyro  */

/*
 * Ngưỡng phát hiện "máy bay chưa đứng yên" trong lúc lấy bias gyro, tính bằng
 * ĐỘ LỆCH CHUẨN chứ không phải biên độ đỉnh-đỉnh.
 *
 * VÌ SAO ĐỔI: bản cũ so biên độ đỉnh-đỉnh với ngưỡng 2,0 °/s. Cách đó có hai
 * chỗ hỏng.
 *
 * Thứ nhất, biên độ đỉnh-đỉnh TĂNG THEO SỐ MẪU — càng lấy nhiều mẫu càng dễ
 * gặp giá trị ngoại lai. Với nhiễu Gauss trên 1000-2000 mẫu, biên độ vào
 * khoảng 4,5σ. Nghĩa là đổi IMU_CALIB_SAMPLE_COUNT là ngưỡng đổi ý nghĩa,
 * dù chẳng ai sờ tới nó.
 *
 * Thứ hai, hệ quả thực tế: LSM6DSV có σ = 1,82 °/s nên biên độ đỉnh-đỉnh của
 * riêng nhiễu nền đã là ~8,2 °/s, vượt xa ngưỡng 2,0. Hiệu chuẩn lặp VÔ HẠN
 * — đã đo được 174 lần huỷ liên tiếp — mà máy bay thì nằm im hoàn toàn.
 *
 * σ không phụ thuộc số mẫu, nên ngưỡng đặt theo nó mới có ý nghĩa ổn định.
 *
 * ICM20602 có nhiễu nền σ ≈ 0,09 °/s. Ngưỡng 1,0 cho dư địa gấp 11 lần mà
 * vẫn bắt được va chạm hay rung thật (những thứ đó cho σ vài °/s).
 */
#define IMU_CALIB_MOVE_SD_DPS 1.0f

/*
 * Cần bao nhiêu mẫu thì độ lệch chuẩn mới có nghĩa để đem so ngưỡng. Dưới
 * mức này thì bỏ qua phép kiểm — vài mẫu đầu không nói lên điều gì.
 * Dùng chung cho cả hai IMU.
 */
#define IMU_CALIB_SD_MIN_SAMPLES 64u
#define IMU_SPI_TIMEOUT_MS 10  /* timeout SPI lúc init  */
#define IMU_GYRO_LPF_HZ 100.0f /* lọc gyro cho vòng PID */
#define IMU_ACCEL_LPF_HZ 30.0f /* lọc accel cho ước lượng */

/*
 * Xoay trục cảm biến sang trục thân máy bay.
 * Quy ước thân: X = mũi trước, Y = cánh phải, Z = hướng xuống (NED body).
 *
 * IMU_AXIS_MAP_x = chỉ số trục CẢM BIẾN (0=X, 1=Y, 2=Z) gán cho trục THÂN x.
 * IMU_AXIS_SIGN_x = +1 hoặc -1 để đảo chiều.
 *
 * Giá trị dưới đây giả định chip dán đúng chiều mặc định. Kiểm tra thực tế:
 * nghiêng mũi lên -> pitch phải dương; nghiêng phải -> roll phải dương;
 * để yên trên bàn -> accel Z ≈ -9.81 m/s².
 *
 * Dấu ÂM mới là đúng, và đây là chỗ hay nhầm nhất: gia tốc kế đo lực riêng
 * chứ không đo trọng lực. Lúc đứng yên, lực riêng hướng LÊN (phản lực của
 * mặt bàn) trong khi trục Z thân hướng XUỐNG, nên số đo phải âm.
 *
 * Bộ ba dấu (+1, -1, -1) là một phép quay hợp lệ: định thức bằng +1. Đừng
 * sửa lẻ một dấu để "cho ra số đẹp" — đảo lẻ sẽ biến hệ trục thành tay trái
 * và làm sai chiều quay của gyro.
 */
#define IMU_AXIS_MAP_X 0
#define IMU_AXIS_MAP_Y 1
#define IMU_AXIS_MAP_Z 2
#define IMU_AXIS_SIGN_X (+1)
#define IMU_AXIS_SIGN_Y (-1)
#define IMU_AXIS_SIGN_Z (-1)

/* --------------------------------------------------------------------------
 * LSM6DSV trên SPI3, DRDY = PD7 (EXTI9_5, ưu tiên 4) — IMU PHỤ
 *
 * Trên cùng module còn có từ kế QMC6309 nối vào sensor hub của chip này.
 * Xem App/Docs/KE_HOACH_LSM6DSV.md để biết lộ trình từng giai đoạn.
 *
 * IMU này ghi vào g_fc.imu2 và KHÔNG tham gia vòng điều khiển. Vòng PID
 * 4 kHz và bộ ước lượng vẫn chạy hoàn toàn bằng ICM20602.
 * -------------------------------------------------------------------------- */

/*
 * ODR đặt 1920 Hz chứ không phải 8 kHz như ICM20602. Cố ý:
 *   - Dữ liệu bị lọc xuống 100 Hz trước khi dùng, tốc độ cao hơn vô ích.
 *   - Mỗi ngắt đều tốn CPU, mà vòng PID 4 kHz không được phép chậm lại.
 * Nếu DBG_MODE_PID báo hz tụt dưới 4000 thì hạ xuống 960.
 * Giá trị hợp lệ: 120 | 240 | 480 | 960 | 1920
 */
/*
 * Cong tac A/B de do CHI PHI CPU cua IMU phu.
 *
 * Dat 0 -> khong goi init/start, khong co ngat DRDY nao tu LSM6DSV, chi phi
 * bang 0. Nap lai roi doc cot rhz trong DBG_MODE_STATUS de co so nen, sau do
 * dat lai 1 va so sanh. Chenh lech chinh la gia phai tra cho IMU thu hai.
 */
#define IMU2_ENABLE 1

#define IMU2_ODR_HZ 1920
#define IMU2_GYRO_FS_DPS 2000        /* 125|250|500|1000|2000 °/s */
#define IMU2_ACCEL_FS_G 16           /* 2|4|8|16 g                */
#define IMU2_CALIB_SAMPLE_COUNT 1000 /* ~0,52 s ở 1920 Hz         */

/*
 * Ngưỡng độ lệch chuẩn cho IMU phụ — xem giải thích dài ở IMU_CALIB_MOVE_SD_DPS.
 *
 * Đặt CAO HƠN HẲN của ICM20602 vì con LSM6DSV trên bo này có nhiễu nền
 * σ ≈ 1,82 °/s, tức gấp 20 lần ICM và gấp ~20 lần chính nó lúc mới lắp
 * (đo được 0,09 °/s ở giai đoạn 1). Nhiều khả năng chip đã suy giảm sau sự
 * kiện cắm nguồn pin — cùng sự kiện đã giết QMC6309.
 *
 * 5,0 cho dư địa ~2,7 lần trên nền nhiễu hiện tại. Thay module mới thì hạ
 * về 1,0 cho bằng ICM.
 */
#define IMU2_CALIB_MOVE_SD_DPS 5.0f
#define IMU2_SPI_TIMEOUT_MS 10
#define IMU2_GYRO_LPF_HZ 100.0f /* giữ giống ICM để so sánh công bằng */
#define IMU2_ACCEL_LPF_HZ 30.0f

/*
 * Xoay trục cảm biến sang trục thân. Ý nghĩa giống IMU_AXIS_* ở trên.
 *
 * ĐÃ ĐO 2026-08-26 bằng DBG_MODE_AXISCAL, kiểm lại bằng phép đo tay ở đúng
 * 90°. Hai lần cho cùng một kết quả. Định thức = +1, phép quay hợp lệ.
 *
 * Số thô trung bình ở ba tư thế (20 mẫu mỗi tư thế, đặt sát 90°):
 *
 *   nằm phẳng, mặt trên lên   -> (    6,    33,  2056)  trục cảm biến Z, dấu +
 *   dựng trên đuôi, mũi lên   -> (   73, -2032,    67)  trục cảm biến Y, dấu -
 *   nghiêng, cánh phải xuống  -> ( 2039,    97,    70)  trục cảm biến X, dấu +
 *
 * Trục chiếm ưu thế đạt 0,998-1,000 độ lớn vector, tức tư thế rất sát chuẩn.
 *
 * Kiểm tra kèm theo:
 *   |a| trung bình 2044 count so với kỳ vọng 2049 (16 g, 0,488 mg/LSB)
 *   -> lệch 0,24%, xác nhận lần nữa hệ số thang accel đúng.
 *   |a| chênh nhau 1,05% giữa ba tư thế -> độ nhạy chéo trục, mức bình thường.
 *   Tư thế nằm phẳng có thành phần phụ chỉ 0,16° và 0,91° -> module lắp gần
 *   như song song với mặt bo. (Hai tư thế kia cầm tay nên 2-3° là sai số tay.)
 *
 * Suy ra module lắp theo hướng:
 *
 *   thân X (mũi trước) = -cảm biến Y      tức cảm biến Y chỉ về phía SAU
 *   thân Y (cánh phải) = -cảm biến X      tức cảm biến X chỉ sang TRÁI
 *   thân Z (hướng xuống) = -cảm biến Z    tức cảm biến Z chỉ LÊN
 *
 * Khác hẳn ICM20602 (MAP 0,1,2 / SIGN +1,-1,-1) — module này vừa xoay vừa
 * lật, nên X và Y hoán vị cho nhau chứ không chỉ đổi dấu.
 *
 * Bộ ba dấu phải là một phép QUAY hợp lệ — định thức bằng +1. Đảo 0 hoặc 2
 * dấu thì hợp lệ; đảo 1 hoặc 3 dấu sẽ biến hệ trục thành tay trái và làm
 * sai chiều quay của gyro. Đừng sửa lẻ một dấu để "cho ra số đẹp".
 *
 * Muốn đo lại: đặt console về DBG_MODE_AXISCAL rồi làm lại ba tư thế.
 */
#define IMU2_AXIS_MAP_X 1
#define IMU2_AXIS_MAP_Y 0
#define IMU2_AXIS_MAP_Z 2
#define IMU2_AXIS_SIGN_X (-1)
#define IMU2_AXIS_SIGN_Y (-1)
#define IMU2_AXIS_SIGN_Z (-1)

/* --------------------------------------------------------------------------
 * TỪ KẾ — chọn nguồn dữ liệu
 *
 * Toàn bộ phần phía sau (g_fc.mag, hiệu chuẩn, DBG_MODE_MAG/MAGCAL, và sau
 * này là hợp nhất yaw vào EKF) đều KHÔNG phụ thuộc nguồn. Đổi nguồn chỉ đụng
 * tới tầng driver.
 *
 *   MAG_SOURCE_NONE  không có từ kế. g_fc.mag đứng im, healthy = false,
 *                    SENSOR_MAG không bao giờ bật. Mọi thứ khác chạy bình thường.
 *   MAG_SOURCE_SHUB  QMC6309 sau sensor hub của LSM6DSV. ĐÃ HIỆN THỰC và đã
 *                    chạy được, nhưng phần cứng module hiện hỏng — xem
 *                    App/Docs/KE_HOACH_LSM6DSV.md mục sự cố phần cứng.
 *   MAG_SOURCE_I2C   module từ kế rời trên bus I2C riêng. CHƯA HIỆN THỰC.
 *                    Bo còn I2C2 chưa dùng (PB10/PB11) và nhãn PE15 = I2C2_INT.
 * -------------------------------------------------------------------------- */
#define MAG_SOURCE_NONE 0
#define MAG_SOURCE_SHUB 1
#define MAG_SOURCE_I2C 2

/*
 * Đang đặt NONE: QMC6309 trên module không còn trả lời (NACK vĩnh viễn, quét
 * 8 địa chỉ đều im). Giữ nguyên toàn bộ code hiệu chuẩn và đường xử lý để lắp
 * module rời vào sau — chỉ cần viết driver mới rồi đổi hằng số này.
 */
#define MAG_SOURCE MAG_SOURCE_NONE

/* --------------------------------------------------------------------------
 * QMC6309 — từ kế, nối vào SENSOR HUB (bus I2C phụ) của LSM6DSV
 *
 * Không có dây I2C nào ra MCU. LSM6DSV làm I2C master trên chân SDX/SCX của
 * nó, tự đọc QMC6309 rồi để kết quả vào dãy thanh ghi SENSOR_HUB_1..6 để host
 * lấy qua SPI3.
 *
 * Xem App/Docs/KE_HOACH_LSM6DSV.md giai đoạn 3 và 4.
 * -------------------------------------------------------------------------- */

/*
 * Dải đo. Càng hẹp thì phân giải càng cao nhưng càng dễ bão hoà khi ở gần
 * động cơ và dây nguồn.
 *
 *   8  -> 4000 LSB/G, từ trường Trái Đất (~0,5 G) cho ~2000 count
 *   16 -> 2000 LSB/G
 *   32 -> 1000 LSB/G, an toàn nhất khi nhiễu từ mạnh
 *
 * Bắt đầu ở 8 G cho phân giải tốt. Console DBG_MODE_MAG in cờ OVFL — nếu nó
 * bật khi tăng ga thì nới lên 16 hoặc 32.
 */
#define MAG_RANGE_G 32 /* 8 | 16 | 32 Gauss */

/*
 * Tốc độ ra của chính từ kế. Vòng lặp chính đọc ở MAG_UPDATE_RATE_HZ, đặt
 * THẤP HƠN để mỗi lần đọc luôn được mẫu mới thay vì đọc lại mẫu cũ.
 */
#define MAG_ODR_HZ 200 /* 1 | 10 | 50 | 100 | 200 */

/*
 * Nhịp vòng lặp chính đọc dãy SENSOR_HUB.
 *
 * ⚠️ KHÔNG được đặt cao. Muốn đọc SENSOR_HUB thì phải MỞ bank sensor hub,
 * và việc bật/tắt FUNC_CFG_ACCESS quá thường xuyên khiến bộ máy I2C master
 * bị cắt ngang trước khi chạy trọn một chu kỳ — nó sẽ không bao giờ nạp được
 * dữ liệu mới, và ta đọc mãi ra nội dung cũ.
 *
 * ĐÃ ĐO trên bo này: một chu kỳ hub mất khoảng 90 ms với MAG_SHUB_ODR = 0.
 * Đọc ở 10 Hz (100 ms) là vừa đủ chỗ cho nó.
 *
 * Muốn đọc nhanh hơn thì phải nới MAG_SHUB_ODR trước, rồi đo lại cột hz
 * trong DBG_MODE_MAG chứ đừng đoán.
 */
#define MAG_UPDATE_RATE_HZ 10

/*
 * Nhịp của chính bộ máy sensor hub — trường shub_odr, bit 7..5 của
 * SLV0_CONFIG. Giá trị 0 là mặc định và ĐÃ ĐO là ~90 ms mỗi chu kỳ.
 *
 * Bảng quy đổi sang Hz chưa xác minh được (file driver của ST quá lớn để
 * tra hết), nên cách duy nhất đáng tin là đổi số rồi đọc cột hz.
 */
#define MAG_SHUB_ODR 4 /* 0..7 */

/*
 * OSR1 hẹp băng thông bộ lọc số, OSR2 là độ sâu bộ lọc thông thấp phía sau.
 * Cả hai đặt 8 cho nhiễu thấp nhất — la bàn không cần băng thông cao, và
 * datasheet cho 2,5 mGauss ở mức này so với 7,0 mGauss ở OSR 8/1.
 */
#define MAG_OSR1 8 /* 1 | 2 | 4 | 8 */
#define MAG_OSR2 8 /* 1 | 2 | 4 | 8 | 16 */

/*
 * Xoay trục từ kế sang trục thân. Ý nghĩa giống IMU_AXIS_* và IMU2_AXIS_*.
 *
 * ⚠️ CHƯA ĐO. QMC6309 nằm cùng module với LSM6DSV nhưng là chip RIÊNG, hướng
 * đặt trên đế của nó không nhất thiết trùng với LSM6DSV. Phải xác định ở
 * GIAI ĐOẠN 4 rồi mới tin được hướng mũi.
 *
 * Bộ ba dấu phải cho định thức +1 — xem ghi chú dài ở khối IMU2 phía trên.
 */
#define MAG_AXIS_MAP_X 0
#define MAG_AXIS_MAP_Y 1
#define MAG_AXIS_MAP_Z 2
#define MAG_AXIS_SIGN_X (+1)
#define MAG_AXIS_SIGN_Y (+1)
#define MAG_AXIS_SIGN_Z (+1)

/*
 * Hiệu chuẩn sắt cứng và sắt mềm — tính trong HỆ CẢM BIẾN, áp TRƯỚC khi xoay
 * trục. Nhờ vậy sửa MAG_AXIS_* về sau không làm hỏng bộ số này.
 *
 * ĐÃ ĐO 2026-08-26, khớp ellipsoid bình phương tối thiểu trên 1176 mẫu.
 *
 *   |B| sau hiệu chuẩn  0,386 G   -> đúng dải Trái Đất 0,25-0,65 G
 *   dao động |B| khi xoay mọi hướng: 1,7%  (mục tiêu dưới 5%)
 *
 * Ba hệ số tỉ lệ đều ~1,00 nghĩa là từ kế này gần như KHÔNG có méo sắt mềm.
 * Toàn bộ sai lệch đến từ TÂM, tức lệch sắt cứng.
 *
 * Offset trục Y bằng 0,267 G, tức 69% độ lớn từ trường — lệch sắt cứng khá
 * lớn. Không sao chừng nào nó CỐ ĐỊNH, nhưng nếu dời module, đổi cách bắt bo
 * hay đi lại dây nguồn thì phải hiệu chuẩn lại.
 *
 * Từ kế chưa hiệu chuẩn TỆ HƠN là không có: nó kéo yaw sai một cách tự tin,
 * và EKF sẽ tin nó. Không được bật hợp nhất yaw (giai đoạn 6) khi các số này
 * còn là mặc định.
 */
#define MAG_OFFSET_X_G 0.0028f
#define MAG_OFFSET_Y_G 0.2674f
#define MAG_OFFSET_Z_G 0.0955f
#define MAG_SCALE_X 1.0005f
#define MAG_SCALE_Y 0.9977f
#define MAG_SCALE_Z 1.0117f

/** Mất bao lâu không có mẫu mới thì coi từ kế là chết. */
#define MAG_TIMEOUT_MS 200

/*
 * Bật bộ đo nền nhiễu gyro cho ICM20602 (xem App/Common/imu_noise.h).
 * Chỉ tốn ~0,03% ngân sách ISR, nhưng là công cụ chính của giai đoạn 2.
 * Đặt 0 để biên dịch bỏ hẳn, trả icm20602.c về đúng như trước.
 */
#define IMU_NOISE_STATS_ENABLE 1

/* --------------------------------------------------------------------------
 * BMP388 trên I2C1 @ 400 kHz (PB8 = SCL, PB7 = SDA)
 *
 * Địa chỉ 7 bit do chân SDO quyết định: nối 3V3 -> 0x77, nối GND -> 0x76.
 * Chân CS phải nối 3V3 để chip chạy giao diện I2C.
 * -------------------------------------------------------------------------- */
#define BARO_I2C_ADDR_7BIT 0x77 /* SDO nối 3V3           */
#define BARO_I2C_TIMEOUT_MS 20  /* timeout I2C lúc init  */

#define BARO_SAMPLE_RATE_HZ 50                      /* ODR của chip          */
#define BARO_POLL_RATE_HZ (BARO_SAMPLE_RATE_HZ * 2) /* nhịp hỏi vòng */
#define BARO_TIMEOUT_MS 200                         /* mất mẫu -> hạ cờ khoẻ */

/*
 * Bộ lọc siêu lấy mẫu (oversampling). Chỉ nhận 1, 2, 4, 8, 16, 32.
 * Cấu hình dưới đây là bộ tham số Bosch khuyến nghị cho drone:
 * áp suất x8, nhiệt độ x1, IIR hệ số 3, ODR 50 Hz.
 *   Thời gian đo = 234 + (392 + 8*2020) + (163 + 1*2020) ≈ 19,0 ms
 * vẫn nằm dưới chu kỳ 20 ms của ODR 50 Hz. Tăng BARO_OSR_PRESSURE lên 16
 * thì PHẢI hạ BARO_SAMPLE_RATE_HZ xuống 25, nếu không chip báo lỗi cấu hình
 * (bit conf_err trong thanh ghi ERR) và tự bỏ mẫu.
 */
#define BARO_OSR_PRESSURE 8
#define BARO_OSR_TEMPERATURE 1

/* Hệ số lọc IIR trong chip. Chỉ nhận 0, 1, 3, 7, 15, 31, 63, 127 (0 = tắt). */
#define BARO_IIR_COEF 3

#define BARO_SEA_LEVEL_PA 101325.0f /* mốc khí quyển chuẩn    */
#define BARO_CALIB_SAMPLE_COUNT 50  /* số mẫu lấy mốc mặt đất */
#define BARO_ALT_LPF_HZ 2.0f        /* lọc độ cao cho mượt    */

/* MTF01P trên UART4 @ 115200, giao thức MSP V2 */
#define FLOW_SAMPLE_RATE_HZ 100
#define FLOW_RX_BUFFER_SIZE 256 /* đệm DMA vòng tròn      */
#define FLOW_RANGE_MAX_MM 8000
#define FLOW_QUALITY_MIN 64 /* dưới ngưỡng coi là xấu */
#define FLOW_RANGE_TIMEOUT_MS 200

/*
 * Hệ số quy đổi số đếm optical flow sang radian.
 *
 * CẦN HIỆU CHUẨN BẰNG THỰC NGHIỆM — giá trị dưới đây chỉ là điểm khởi đầu.
 * Cách làm: giữ độ cao cố định đã biết (ví dụ 1,00 m), rê ngang đúng 1,00 m
 * trong khoảng 2 giây, cộng dồn flow_x_raw. Khi đó
 *     FLOW_RAD_PER_COUNT = (quãng đường / độ cao) / tổng số đếm
 * Vận tốc suy ra: v = tốc_độ_góc_rad_s * độ_cao_m
 */
/*
 * HIỆU CHUẨN BẰNG PHÉP THỬ NGHIÊNG — cách chính xác nhất, và không cần đo
 * quãng đường hay độ cao gì cả.
 *
 * NGUYÊN LÝ: khi máy bay chỉ QUAY tại chỗ, phần ảnh dịch do quay không phụ
 * thuộc độ cao — nó bằng đúng góc quay. Nên lúc đó flow phải bằng đúng tốc độ
 * góc gyro (ngược dấu). Gyro thì đã hiệu chuẩn sẵn và cả chuỗi điều khiển đã
 * chứng minh nó đúng, nên nó làm chuẩn tham chiếu rất tốt.
 *
 * ĐO ĐƯỢC trên 341 mẫu có |gyro| > 0,3 rad/s, khớp bình phương tối thiểu:
 *   trục X (217 mẫu): flow = -2,003 x gyro
 *   trục Y (124 mẫu): flow = -1,998 x gyro
 *   gộp   (341 mẫu): flow = -2,001 x gyro
 *
 * Cần flow = -1,000 x gyro thì phép bù mới triệt tiêu, nên hệ số cũ lớn gấp
 * ĐÚNG 2 lần: 0,00388 / 2,00 = 0,00194.
 *
 * Hai trục cho ra cùng một con số tới 0,25% — đây không phải trùng hợp.
 *
 * VÌ SAO CÁCH NÀY HƠN HẲN ĐO QUÃNG ĐƯỜNG:
 *   - Không cần đo quãng đường (nguồn sai số lớn nhất khi làm bằng tay)
 *   - Không cần đo độ cao (ảnh dịch do quay độc lập với độ cao)
 *   - Hàng trăm mẫu thay vì một phép đo duy nhất
 *   Hai lần đo quãng đường trước đó cho 0,0206 (trong nhà) và 0,00388 (ngoài
 *   sân) — lệch nhau 5 lần. Phép thử nghiêng chốt được con số cuối cùng.
 *
 * TINH CHỈNH LẦN HAI: sau khi đặt 0,00194 và đo lại, khớp ra -1,05 thay vì
 * -1,00, nên chia tiếp cho 1,05 -> 0,00185. Phần dư còn lại chủ yếu là nhiễu
 * riêng của PMW3901 (94% phương sai), không sửa được bằng firmware.
 *
 * LƯU Ý CÒN BỎ NGỎ: hệ số lệch đúng 2,00 chứ không phải một số lẻ, nghi có
 * sai số hệ thống trong dt của gói flow chứ không phải sai số đo. Dù nguyên
 * nhân là gì thì việc chia đôi hệ số vẫn khắc phục đúng cả hai trường hợp
 * (quay lẫn tịnh tiến), nên để lại xem xét sau.
 */
#define FLOW_RAD_PER_COUNT 0.00185f

/*
 * Xoay trục cảm biến flow sang trục thân, cùng quy ước với IMU.
 * Sau khi có số liệu thật, đẩy máy bay về phía trước: velocity_mps.x phải dương.
 */
#define FLOW_AXIS_MAP_X 0
#define FLOW_AXIS_MAP_Y 1
#define FLOW_AXIS_SIGN_X (-1) /* ĐÃ ĐO: rê phải cho đếm ÂM, phải đảo */
#define FLOW_AXIS_SIGN_Y (-1) /* ĐÃ ĐO: rê tới cho vb_x âm, phải đảo */

/*
 * ĐÃ ĐO trên bộ phần cứng này: rê sang PHẢI (không xoay) làm sum_x đổi -90
 * còn sum_y chỉ đổi -5.
 *
 *   - Chuyển động NGANG hiện trên trục X của cảm biến -> ánh xạ ĐÚNG, giữ
 *     nguyên MAP_X = 0, MAP_Y = 1.
 *   - Nhưng công thức bù quay yêu cầu rê phải cho đếm DƯƠNG (ω_x = +vy/h),
 *     mà đo được là âm -> phải đảo SIGN_X.
 *   - Trục Y chốt bằng DBG_MODE_VEL, đọc thẳng dấu vận tốc thay vì đếm dồn:
 *     rê TỚI cho vb_x ÂM (phải dương) -> đảo SIGN_Y. Rê PHẢI cho vb_y dương,
 *     rê TRÁI cho âm -> trục X vốn đã đúng.
 *
 *     Cách này đáng tin hơn hẳn phép đếm dồn: không phải đo quãng đường, chỉ
 *     nhìn dấu. Bản đo đếm dồn trong nhà từng cho kết luận NGƯỢC LẠI về trục
 *     Y, và nó sai — sàn kém kết cấu làm cảm biến bỏ sót đếm loạn xạ.
 *
 *   Lưu ý: đảo dấu ở đây là ĐÚNG chỗ, kể cả khi có bù quay. Cảm biến lắp lệch
 *   thì cả phần dịch ảnh do TỊNH TIẾN lẫn do QUAY đều đổi dấu như nhau, nên
 *   một phép đảo khôi phục đúng cả hai số hạng của công thức bù.
 */

/* ==========================================================================
 * Bộ ước lượng EKF
 *
 * Các con số dưới đây là ĐỘ TIN CẬY, không phải hệ số chỉnh tay như PID.
 * Quy tắc: nhiễu đo (NOISE) càng lớn thì bộ lọc càng ít tin cảm biến đó;
 * nhiễu quá trình (WALK) càng lớn thì càng cho phép đại lượng đổi nhanh.
 * Đặt sai vẫn chạy nhưng ra kết quả trễ hoặc rung.
 * ========================================================================== */
#define EST_RATE_HZ 1000 /* nhịp chạy bộ lọc      */
#define EST_PERIOD_US (1000000UL / EST_RATE_HZ)

/*
 * Quy ước dấu của gia tốc kế trên mạch này.
 *
 * ĐO THỰC TẾ trên FCH743_V1.0: nằm yên thăng bằng thì az = -9,81 m/s².
 * Đó là con số ĐÚNG về mặt vật lý. Gia tốc kế đo lực riêng (specific force),
 * không đo trọng lực: trục Z thân hướng xuống, mà lực riêng lúc đứng yên lại
 * hướng lên, nên nó phải ra số âm.
 *
 * Bộ lọc EKF được viết theo quy ước "vector chỉ hướng trọng lực" (nằm yên ra
 * +9,81), nên hằng số này đảo dấu CẢ vector accel cho khớp. Đảo cả vector chứ
 * không riêng trục Z là đúng: lực riêng bằng -Rᵀ·g, tức cả ba trục đều ngược
 * dấu so với vector trọng lực, không riêng gì Z.
 *
 * KIỂM TRA LẠI khi đổi mạch hoặc đổi cách dán chip: bật DBG_MODE_IMU, để máy
 * bay nằm yên, đọc cột az. Ra -9,81 thì để -1; ra +9,81 thì đổi thành +1.
 * KHÔNG sửa IMU_AXIS_SIGN_Z để chữa việc này — hằng số đó đảo cả gyro trục
 * yaw và sẽ làm hỏng chiều quay yaw.
 */
#define EST_ACCEL_Z_SIGN (-1)

/* --- Ước lượng góc (ESKF 6 trạng thái: sai số góc + bias gyro) ---------- */
#define EST_GYRO_NOISE_DPS 0.05f      /* mật độ nhiễu gyro      */
#define EST_GYRO_BIAS_WALK_DPS 0.002f /* tốc độ trôi bias gyro  */
#define EST_ACCEL_NOISE_MPS2 0.5f     /* nhiễu đo accel         */
#define EST_GYRO_BIAS_MAX_DPS 10.0f   /* chặn bias phi lý       */

/*
 * Cho bộ lọc tự học bias con quay trục YAW hay không. 0 = KHÔNG (mặc định).
 *
 * Gia tốc kế không nhìn thấy yaw, nên trạng thái này KHÔNG QUAN SÁT ĐƯỢC và
 * để nó tự do là để một bước ngẫu nhiên điều khiển hướng của máy bay.
 *
 * ĐO ĐƯỢC 28/08/2026, máy giữ cố định trên giá:
 *     con quay thật   +0,011 °/s  =   +0,7 độ/phút
 *     bgz bộ lọc tự ra  3,610 °/s  = −216,6 độ/phút
 *     trôi thực tế                   −218,2 độ/phút
 * Bộ lọc tự chế ra 99,7 % lượng trôi.
 *
 * BẬT LẠI (đặt 1) khi đã lắp từ kế — lúc đó bias yaw mới quan sát được và
 * việc học nó trở thành có ích, vì nó sẽ bám theo trôi nhiệt thật.
 */
#define EST_YAW_BIAS_LEARN 0

/*
 * Accel chỉ đo đúng hướng trọng lực khi máy bay KHÔNG tăng tốc. Lệch khỏi
 * 9,81 m/s² càng nhiều thì số đo càng vô dụng, nên độ tin cậy bị hạ theo
 * bình phương độ lệch thay vì cắt phăng — cắt cứng làm bộ lọc giật mỗi lần
 * vượt ngưỡng.
 */
#define EST_ACCEL_REJECT_MPS2 2.0f

/* --- Ước lượng độ cao (EKF 3 trạng thái: cao độ, tốc độ lên, bias accel) - */
#define EST_ACC_Z_NOISE_MPS2 0.6f    /* nhiễu gia tốc thẳng đứng */
#define EST_ACC_Z_BIAS_WALK 0.02f    /* trôi bias accel Z      */
#define EST_BARO_NOISE_M 0.6f        /* baro ồn nhưng không trôi */
#define EST_RANGE_NOISE_M 0.05f      /* laser chính xác hơn nhiều */
#define EST_RANGE_MAX_TILT_DEG 25.0f /* nghiêng quá -> bỏ range */
#define EST_RANGE_MAX_M 6.0f         /* quá xa -> tin baro hơn */

/* --- Ước lượng vận tốc ngang từ optical flow ----------------------------
 *
 * Flow chỉ dùng được trong một cửa sổ hẹp, và ngoài cửa sổ đó số đo là rác
 * chứ không phải kém chính xác:
 *   - Quá thấp: cảm biến không lấy nét được, và sai số độ cao chiếm tỉ lệ lớn
 *   - Quá cao : laser ToF hết tầm, mà thiếu độ cao thì không quy đổi được
 *   - Nghiêng nhiều: tia laser bắn xiên nên độ cao sai, và phép chiếu vận tốc
 *     từ hệ thân sang hệ NED cũng mất chính xác
 */
#define EST_FLOW_MIN_HEIGHT_M 0.20f
#define EST_FLOW_MAX_HEIGHT_M 5.0f
#define EST_FLOW_MAX_TILT_DEG 20.0f

/* Độ tin cậy. Flow ồn hơn baro nhiều vì phụ thuộc kết cấu mặt sàn. */
/*
 * Bao lau khong chap nhan duoc mau flow thi coi nhu MAT uoc luong van toc.
 *
 * Khong co phep do thi bo loc chi tich phan gia toc ke. O 15 do nghieng, gia
 * toc ngang khoang 2,6 m/s^2, tuc chi NUA GIAY da tich luy 1,3 m/s sai so.
 *
 * 300 ms la thoa hiep: du dai de bo qua vai mau bi tu choi le te khi nghieng
 * thoang qua, du ngan de cat vong phan hoi duong truoc khi no kip lon.
 *
 * Qua NGAN -> co bao mat lien tuc, POSHOLD chop tat giua giu vi tri va ANGLE.
 * Qua DAI  -> khong cat duoc vong chay tron, drone lac vong tron ban kinh lon dan.
 */
#define EST_FLOW_TIMEOUT_MS 300

#define EST_FLOW_NOISE_MPS 0.20f
#define EST_ACC_XY_NOISE_MPS2 0.8f
#define EST_ACC_XY_BIAS_WALK 0.02f

/* ==========================================================================
 * Nguồn — chia áp trên ADC1_INP11 (PC1) và ADC1_INP10 (PC0)
 * ========================================================================== */
#define PWR_ADC_VREF_V 3.3f
#define PWR_ADC_FULL_SCALE 65535.0f /* ADC 16-bit            */
#define PWR_VBAT_DIVIDER 11.0f      /* HIỆU CHỈNH theo mạch  */
#define PWR_CURRENT_MV_PER_A 50.0f  /* HIỆU CHỈNH theo sensor */
#define PWR_CELL_MIN_V 3.30f
#define PWR_CELL_WARN_V 3.50f
#define PWR_CELL_FULL_V 4.20f

/* ==========================================================================
 * Điều kiện arm (an toàn)
 * ========================================================================== */
#define ARM_THROTTLE_MAX_NORM 0.05f /* ga phải dưới 5%      */
#define ARM_MAX_TILT_DEG 25.0f      /* nghiêng tối đa khi arm */
#define ARM_SWITCH_CHANNEL 4        /* kênh AUX1 (0-based)   */
#define ARM_HOLD_TIME_MS 100        /* chống nhiễu công tắc  */

/*
 * Ngưỡng đọc công tắc arm, tính trên giá trị thô CRSF (172..1811).
 *
 * Hai ngưỡng lệch nhau tạo vùng trễ: giá trị nằm giữa 1300 và 1500 thì giữ
 * nguyên trạng thái trước đó, nhờ vậy công tắc rung hay nhiễu đường truyền
 * không làm arm/disarm liên tục.
 *
 * Đặt ngưỡng ON cao hơn hẳn điểm giữa (992) là có chủ ý: công tắc ba nấc
 * dùng làm arm sẽ coi nấc giữa là OFF, chỉ nấc trên cùng mới arm được.
 */
#define ARM_SWITCH_ON_THRESHOLD 1500
#define ARM_SWITCH_OFF_THRESHOLD 1300

/* ==========================================================================
 * Telemetry
 * ========================================================================== */
#define TLM_TX_BUFFER_SIZE 2048 /* ring buffer TX        */
#define TLM_MAX_PAYLOAD 64
#define TLM_DEFAULT_UART_BAUD 921600 /* USART1 -> máy tính    */

/* ==========================================================================
 * Console gỡ lỗi dạng chữ (đọc bằng PuTTY / terminal bất kỳ)
 * ========================================================================== */
#define DBG_TX_BUFFER_SIZE 1024 /* ring buffer gửi        */
/* ==========================================================================
 * THẺ SD — CÔNG TẮC TỔNG
 *
 * Đặt 0 để BỎ HẲN mọi thứ đụng tới thẻ: khởi tạo SDMMC, FATFS, blackbox, và
 * chế độ đọc thẻ qua USB.
 *
 * VÌ SAO ĐANG ĐẶT 0 — ĐO ĐƯỢC BẰNG DEBUGGER, KHÔNG PHẢI PHỎNG ĐOÁN:
 *
 *   Với thẻ hiện tại, HAL_SD_Init() không trả về. CPU quay vòng vĩnh viễn
 *   trong SD_SendSDStatus() (stm32h7xx_hal_sd.c:3363) — một vòng chờ CHẶN.
 *
 *   Hậu quả không phải là "mất log": main() dừng lại ngay tại đó, nên
 *   tlm_port_init(), dbg_console_set_mode(), và toàn bộ phần khởi tạo cảm
 *   biến KHÔNG BAO GIỜ CHẠY. Không telemetry, không CLI, không điều khiển.
 *   Một cái thẻ hỏng làm liệt cả mạch bay.
 *
 *   Cách nhận ra lúc đó: đọc s_port và s_mode trong RAM, cả hai đều bằng 0
 *   dù chúng được đặt ở hai dòng khác nhau trong main() — tức là main() chưa
 *   chạy tới dòng nào trong hai dòng đó.
 *
 * Ghi chú ở main.c (mục "THE SD HONG THI KHONG DUOC TREO CA BO DIEU KHIEN
 * BAY") đã lường trước chuyện này, nhưng phép chặn ở đó chỉ bắt Error_Handler
 * — nó không bắt được việc HAL_SD_Init tự nó chặn vô hạn.
 *
 * MUỐN DÙNG LẠI THẺ: đặt 1, và trước đó phải sửa nguyên nhân gốc — thay thẻ,
 * hoặc thay HAL_SD_Init bằng bản có hạn thời gian thật sự.
 * ========================================================================== */
#define FC_SD_ENABLE 0

/* ==========================================================================
 * Blackbox — ghi log chuyến bay ra thẻ SD
 *
 * NGUYÊN TẮC AN TOÀN QUAN TRỌNG NHẤT: KHÔNG BAO GIỜ chạm vào thẻ SD trong
 * lúc đang ARM.
 *
 * f_write() là hàm CHẶN, và thẻ SD có thể khựng hàng chục tới hàng trăm mili
 * giây khi nó tự dọn khối bên trong. Vòng lặp chính phải xong dưới 250 µs —
 * một cú khựng như vậy sẽ làm ngừng phát khung DShot, ESC coi như mất tín
 * hiệu và cắt motor giữa không trung.
 *
 * Nên: bay thì ghi vào RAM, hạ xuống rồi mới xả ra thẻ.
 * ========================================================================== */
#define BB_ENABLE 1

/*
 * Nhịp ghi. 100 Hz đủ để nhìn dao động PID (băng thông quan tâm dưới 30 Hz)
 * mà vẫn cho thời lượng dài. Nâng lên 200 thì thời lượng còn một nửa.
 */
#define BB_RATE_HZ 100

/*
 * Bộ đệm nằm ở AXI SRAM (0x24000000) — vùng còn trống 507 KB, luôn được cấp
 * clock, và section .dma_buffer đã có sẵn trong linker script.
 *
 * KHÔNG dùng RAM_D2 dù nó trống 288 KB: vùng đó cần thêm một section mới
 * trong file .ld và phải bật RCC_AHB2ENR, mà CubeMX thì ghi đè .ld mỗi lần
 * Generate Code. Đổi lấy sự mong manh đó để được thêm RAM là không đáng.
 *
 * 256 KB / 48 byte mỗi bản ghi = 5461 bản ghi = ~55 giây ở 100 Hz.
 * Đủ cho các chuyến bay ngắn để chỉnh PID.
 */
#define BB_BUFFER_BYTES (256u * 1024u)

/*
 * Mỗi lần xả ghi bấy nhiêu byte văn bản rồi trả quyền cho vòng lặp. Chỉ chạy
 * lúc đã DISARM nên khựng không nguy hiểm, nhưng chia nhỏ để console vẫn cập
 * nhật và người dùng thấy tiến độ.
 */
#define BB_FLUSH_CHUNK_BYTES 4096u

#define DBG_LINE_MAX 200          /* do dai toi da mot dong (STATUS dai nhat) */
#define DBG_DEFAULT_RATE_HZ 20    /* 20 dòng/giây, mắt đọc kịp */
#define DBG_HEADER_EVERY_LINES 20 /* in lại dòng tiêu đề     */

/* ==========================================================================
 * Ghi log ra thẻ microSD
 * ========================================================================== */
#define LOG_RATE_HZ 500
#define LOG_BUFFER_SIZE 4096
#define LOG_FLUSH_INTERVAL_MS 1000

#endif /* FC_CONFIG_H */
