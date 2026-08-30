/**
 * @file    param_list.h
 * @brief   DANH SÁCH THAM SỐ RUNTIME — nguồn sự thật duy nhất.
 *
 * File này KHÔNG có include guard và KHÔNG tự đứng một mình được. Nó là một
 * danh sách thuần tuý, được include nhiều lần với macro P() định nghĩa khác
 * nhau mỗi lần (kỹ thuật "X-macro"):
 *
 *   - param_table.h dùng nó để dựng struct param_storage_t
 *   - param_table.c dùng nó để dựng bảng metadata g_param_table[]
 *
 * Nhờ vậy struct và bảng KHÔNG BAO GIỜ lệch nhau. Thêm một tham số = thêm
 * đúng MỘT dòng ở đây, không phải sửa hai chỗ rồi quên một chỗ.
 *
 * ĐỊNH DẠNG MỘT DÒNG:
 *
 *   P(tên, kiểu_C, mã_kiểu, cờ, min, max, mặc_định)
 *
 *   tên       tên tham số dùng cho CLI và app PC. QUY TẮC: bằng đúng tên
 *             #define trong fc_config.h viết thường. RATE_PID_ROLL_KP ->
 *             rate_pid_roll_kp. Grep ngược được, không phải học tên mới.
 *   kiểu_C    kiểu thật của trường trong struct
 *   mã_kiểu   PT_* — để app PC biết cách hiển thị (số nguyên hay số thực)
 *   cờ        PARAM_FLAG_* hoặc 0, ghép bằng |
 *   min, max  chặn cứng. FIRMWARE LUÔN KẸP, không tin app PC gửi gì.
 *   mặc_định  LẤY THẲNG TỪ MACRO trong fc_config.h
 *
 * VÌ SAO MẶC ĐỊNH VẪN TRỎ VỀ fc_config.h:
 *   File đó chứa hàng trăm dòng ghi chú của những phép đo đã làm thật — cửa
 *   sổ tốc độ DShot, phép thử nghiêng hiệu chuẩn flow, ma trận trục IMU2 đo
 *   ngày 26/08/2026. Đó là tri thức đắt nhất trong repo này. Nó ở lại nguyên
 *   chỗ cũ và tiếp tục là "cấu hình xuất xưởng"; bảng dưới đây chỉ thêm khả
 *   năng chỉnh lúc chạy, không thay thế nó.
 *
 * ĐẶT MIN/MAX THẾ NÀO:
 *   Không phải trang trí. Đây là thứ chặn một cú gõ nhầm phím biến thành một
 *   vụ rơi. Đặt đủ rộng cho mọi khung máy hợp lý, đủ hẹp để chặn số vô nghĩa.
 *   Ví dụ kp của vòng tốc độ: 0,0010 là giá trị đang dùng, trần 0,02 cho dư
 *   địa gấp 20 lần, còn 0,2 thì chắc chắn là gõ thừa một số 0.
 *
 * THỨ TỰ TRƯỜNG:
 *   Xếp float/u32 (4 byte) trước, rồi u16, rồi u8. param_storage_t KHÔNG
 *   packed — xem lý do dài trong param_table.h. Giữ nhóm này để struct không
 *   sinh lỗ đệm vô ích.
 */

/* ==========================================================================
 * PID vòng tốc độ góc (ACRO) — vòng trong cùng, quan trọng nhất
 * ========================================================================== */
P(rate_pid_roll_kp,      float, PT_F32, 0, 0.0f,  0.02f,   RATE_PID_ROLL_KP)
P(rate_pid_roll_ki,      float, PT_F32, 0, 0.0f,  0.01f,   RATE_PID_ROLL_KI)
P(rate_pid_roll_kd,      float, PT_F32, 0, 0.0f,  0.001f,  RATE_PID_ROLL_KD)
P(rate_pid_pitch_kp,     float, PT_F32, 0, 0.0f,  0.02f,   RATE_PID_PITCH_KP)
P(rate_pid_pitch_ki,     float, PT_F32, 0, 0.0f,  0.01f,   RATE_PID_PITCH_KI)
P(rate_pid_pitch_kd,     float, PT_F32, 0, 0.0f,  0.001f,  RATE_PID_PITCH_KD)
P(rate_pid_yaw_kp,       float, PT_F32, 0, 0.0f,  0.02f,   RATE_PID_YAW_KP)
P(rate_pid_yaw_ki,       float, PT_F32, 0, 0.0f,  0.01f,   RATE_PID_YAW_KI)
P(rate_pid_yaw_kd,       float, PT_F32, 0, 0.0f,  0.001f,  RATE_PID_YAW_KD)
P(rate_pid_i_limit,      float, PT_F32, 0, 0.0f,  1.0f,    RATE_PID_I_LIMIT)
P(rate_pid_out_limit,    float, PT_F32, 0, 0.1f,  1.0f,    RATE_PID_OUT_LIMIT)

/*
 * Lọc D. Hệ số alpha được tính SẴN một lần trong ctrl_rate_init(), nên đổi
 * xong phải gọi lại hàm đó. Đánh cờ REBOOT thay vì thêm một đường cập nhật
 * nóng cho tham số hiếm khi đụng tới.
 */
P(rate_dterm_lpf_hz,     float, PT_F32, PARAM_FLAG_REBOOT, 10.0f, 300.0f, RATE_DTERM_LPF_HZ)

/* Tốc độ góc tối đa khi cần đẩy hết. */
P(rate_max_roll_dps,     float, PT_F32, 0, 30.0f, 1200.0f, RATE_MAX_ROLL_DPS)
P(rate_max_pitch_dps,    float, PT_F32, 0, 30.0f, 1200.0f, RATE_MAX_PITCH_DPS)
P(rate_max_yaw_dps,      float, PT_F32, 0, 30.0f, 1200.0f, RATE_MAX_YAW_DPS)

/* ==========================================================================
 * Vòng góc (ANGLE) — vòng ngoài, cố ý chỉ có P (xem ghi chú fc_config.h)
 * ========================================================================== */
P(angle_pid_kp,          float, PT_F32, 0, 0.5f,  20.0f,   ANGLE_PID_KP)
P(angle_max_lean_deg,    float, PT_F32, 0, 5.0f,  60.0f,   ANGLE_MAX_LEAN_DEG)
P(angle_max_rate_dps,    float, PT_F32, 0, 30.0f, 600.0f,  ANGLE_MAX_RATE_DPS)

/* Bu do lech lap dat IMU. Do tren mat phang roi dat bang so doc duoc DOI DAU. */
P(angle_trim_roll_deg,   float, PT_F32, 0, -10.0f, 10.0f, ANGLE_TRIM_ROLL_DEG)
P(angle_trim_pitch_deg,  float, PT_F32, 0, -10.0f, 10.0f, ANGLE_TRIM_PITCH_DEG)

/* ==========================================================================
 * Giữ vận tốc bằng optical flow (POSHOLD) — vòng ngoài cùng
 * ========================================================================== */
P(poshold_max_vel_mps,   float, PT_F32, 0, 0.2f,  5.0f,    POSHOLD_MAX_VEL_MPS)
P(poshold_vel_kp,        float, PT_F32, 0, 0.0f,  30.0f,   POSHOLD_VEL_KP)
P(poshold_vel_ki,        float, PT_F32, 0, 0.0f,  20.0f,   POSHOLD_VEL_KI)
P(poshold_i_limit_deg,   float, PT_F32, 0, 0.0f,  20.0f,   POSHOLD_I_LIMIT_DEG)
/* PHẢI thấp hơn est_flow_max_tilt_deg, nếu không flow bị từ chối giữa chừng. */
P(poshold_max_tilt_deg,  float, PT_F32, 0, 3.0f,  30.0f,   POSHOLD_MAX_TILT_DEG)
P(poshold_pos_kp,        float, PT_F32, 0, 0.0f,  3.0f,    POSHOLD_POS_KP)

/* ==========================================================================
 * Giữ độ cao (ALTHOLD)
 *
 * althold_hover_thr là số PHẢI ĐO trên chính máy bay này, không đoán — xem
 * trình tự chỉnh trong fc_config.h.
 * ========================================================================== */
P(althold_hover_thr,       float, PT_F32, 0, 0.10f, 0.80f, ALTHOLD_HOVER_THR)
P(althold_stick_centre,    float, PT_F32, 0, 0.20f, 0.80f, ALTHOLD_STICK_CENTRE)
P(althold_stick_deadband,  float, PT_F32, 0, 0.02f, 0.40f, ALTHOLD_STICK_DEADBAND)
P(althold_max_climb_mps,   float, PT_F32, 0, 0.2f,  5.0f,  ALTHOLD_MAX_CLIMB_MPS)
P(althold_alt_kp,          float, PT_F32, 0, 0.0f,  4.0f,  ALTHOLD_ALT_KP)
P(althold_climb_kp,        float, PT_F32, 0, 0.0f,  0.5f,  ALTHOLD_CLIMB_KP)
P(althold_climb_ki,        float, PT_F32, 0, 0.0f,  0.5f,  ALTHOLD_CLIMB_KI)
P(althold_climb_kd,        float, PT_F32, 0, 0.0f,  0.1f,  ALTHOLD_CLIMB_KD)
P(althold_dterm_lpf_hz,    float, PT_F32, 0, 1.0f,  50.0f, ALTHOLD_DTERM_LPF_HZ)
P(althold_i_limit,         float, PT_F32, 0, 0.0f,  0.5f,  ALTHOLD_I_LIMIT)
P(althold_thr_min,         float, PT_F32, 0, 0.0f,  0.4f,  ALTHOLD_THR_MIN)
P(althold_thr_max,         float, PT_F32, 0, 0.4f,  1.0f,  ALTHOLD_THR_MAX)

/* ==========================================================================
 * Động cơ và khâu trộn
 * ========================================================================== */
P(dshot_idle_percent,    float, PT_F32, 0, 3.0f,  15.0f,   DSHOT_IDLE_PERCENT)

/*
 * Đổi tốc độ DShot BẮT BUỘC rút pin cắm lại cho ESC — BLHeli_S dò giao thức
 * đúng một lần lúc nó khởi động. Reboot mạch bay là chưa đủ. Cờ REBOOT ở đây
 * chỉ để app hiện cảnh báo; phần "rút pin" thì con người phải tự làm.
 */
P(dshot_bitrate_hz,      uint32_t, PT_U32, PARAM_FLAG_REBOOT | PARAM_FLAG_DANGER, 150000.0f, 600000.0f, DSHOT_BITRATE_HZ)

/* ==========================================================================
 * Hiệu chuẩn nguồn — hai số đầu quyết định độ chính xác của cảnh báo pin
 * ========================================================================== */
P(pwr_vbat_divider,      float, PT_F32, 0, 1.0f,  50.0f,   PWR_VBAT_DIVIDER)
P(pwr_current_mv_per_a,  float, PT_F32, 0, 1.0f,  500.0f,  PWR_CURRENT_MV_PER_A)
P(pwr_cell_min_v,        float, PT_F32, 0, 2.8f,  4.0f,    PWR_CELL_MIN_V)
P(pwr_cell_warn_v,       float, PT_F32, 0, 3.0f,  4.1f,    PWR_CELL_WARN_V)
P(pwr_cell_full_v,       float, PT_F32, 0, 3.8f,  4.3f,    PWR_CELL_FULL_V)

/* ==========================================================================
 * Điều kiện arm
 * ========================================================================== */
P(arm_throttle_max_norm, float, PT_F32, 0, 0.0f,  0.2f,    ARM_THROTTLE_MAX_NORM)
P(arm_max_tilt_deg,      float, PT_F32, 0, 5.0f,  60.0f,   ARM_MAX_TILT_DEG)

/* ==========================================================================
 * Điều khiển từ xa — vùng chết quanh giữa cần
 * ========================================================================== */
P(rc_deadband_norm,      float, PT_F32, 0, 0.0f,  0.2f,    RC_DEADBAND_NORM)

/* ==========================================================================
 * IMU chính (ICM20602) — bộ lọc và hiệu chuẩn
 * ========================================================================== */
P(imu_gyro_lpf_hz,       float, PT_F32, PARAM_FLAG_REBOOT, 10.0f, 500.0f, IMU_GYRO_LPF_HZ)
P(imu_accel_lpf_hz,      float, PT_F32, PARAM_FLAG_REBOOT, 5.0f,  200.0f, IMU_ACCEL_LPF_HZ)
P(imu_calib_move_sd_dps, float, PT_F32, 0, 0.1f,  20.0f,   IMU_CALIB_MOVE_SD_DPS)

/* IMU phụ (LSM6DSV) — không tham gia vòng điều khiển */
P(imu2_gyro_lpf_hz,      float, PT_F32, PARAM_FLAG_REBOOT, 10.0f, 500.0f, IMU2_GYRO_LPF_HZ)
P(imu2_accel_lpf_hz,     float, PT_F32, PARAM_FLAG_REBOOT, 5.0f,  200.0f, IMU2_ACCEL_LPF_HZ)
P(imu2_calib_move_sd_dps,float, PT_F32, 0, 0.1f,  20.0f,   IMU2_CALIB_MOVE_SD_DPS)

/* ==========================================================================
 * Từ kế — hiệu chuẩn sắt cứng / sắt mềm
 *
 * Sáu số này do lệnh hiệu chuẩn ghi vào, không gõ tay. Từ kế CHƯA hiệu chuẩn
 * tệ hơn là không có từ kế: nó kéo yaw sai một cách tự tin và EKF sẽ tin nó.
 * ========================================================================== */
P(mag_offset_x_g,        float, PT_F32, 0, -2.0f, 2.0f,    MAG_OFFSET_X_G)
P(mag_offset_y_g,        float, PT_F32, 0, -2.0f, 2.0f,    MAG_OFFSET_Y_G)
P(mag_offset_z_g,        float, PT_F32, 0, -2.0f, 2.0f,    MAG_OFFSET_Z_G)
P(mag_scale_x,           float, PT_F32, 0, 0.5f,  2.0f,    MAG_SCALE_X)
P(mag_scale_y,           float, PT_F32, 0, 0.5f,  2.0f,    MAG_SCALE_Y)
P(mag_scale_z,           float, PT_F32, 0, 0.5f,  2.0f,    MAG_SCALE_Z)

/* ==========================================================================
 * Khí áp kế
 * ========================================================================== */
P(baro_sea_level_pa,     float, PT_F32, 0, 90000.0f, 110000.0f, BARO_SEA_LEVEL_PA)
P(baro_alt_lpf_hz,       float, PT_F32, 0, 0.2f,  20.0f,   BARO_ALT_LPF_HZ)

/* ==========================================================================
 * Optical flow — hệ số quy đổi số đếm sang radian
 *
 * Đo bằng PHÉP THỬ NGHIÊNG (so flow với gyro khi quay tại chỗ). Xem ghi chú
 * dài trong fc_config.h: cách đó không cần đo quãng đường hay độ cao.
 * ========================================================================== */
P(flow_rad_per_count,    float, PT_F32, 0, 0.0001f, 0.05f, FLOW_RAD_PER_COUNT)

/* ==========================================================================
 * EKF — đây là ĐỘ TIN CẬY, không phải hệ số chỉnh tay như PID
 *
 * Quy tắc: NOISE càng lớn thì bộ lọc càng ít tin cảm biến đó; WALK càng lớn
 * thì càng cho phép đại lượng đổi nhanh. Đặt sai vẫn bay nhưng trễ hoặc rung.
 * ========================================================================== */
P(est_gyro_noise_dps,      float, PT_F32, 0, 0.001f,  1.0f,  EST_GYRO_NOISE_DPS)
P(est_gyro_bias_walk_dps,  float, PT_F32, 0, 0.0001f, 0.1f,  EST_GYRO_BIAS_WALK_DPS)
P(est_accel_noise_mps2,    float, PT_F32, 0, 0.05f,   5.0f,  EST_ACCEL_NOISE_MPS2)
P(est_gyro_bias_max_dps,   float, PT_F32, 0, 1.0f,    50.0f, EST_GYRO_BIAS_MAX_DPS)
P(est_accel_reject_mps2,   float, PT_F32, 0, 0.5f,    10.0f, EST_ACCEL_REJECT_MPS2)
P(est_acc_z_noise_mps2,    float, PT_F32, 0, 0.05f,   5.0f,  EST_ACC_Z_NOISE_MPS2)
P(est_acc_z_bias_walk,     float, PT_F32, 0, 0.001f,  0.5f,  EST_ACC_Z_BIAS_WALK)
P(est_baro_noise_m,        float, PT_F32, 0, 0.05f,   5.0f,  EST_BARO_NOISE_M)
P(est_range_noise_m,       float, PT_F32, 0, 0.005f,  1.0f,  EST_RANGE_NOISE_M)
P(est_range_max_tilt_deg,  float, PT_F32, 0, 5.0f,    60.0f, EST_RANGE_MAX_TILT_DEG)
P(est_range_max_m,         float, PT_F32, 0, 0.5f,    12.0f, EST_RANGE_MAX_M)
P(est_flow_min_height_m,   float, PT_F32, 0, 0.05f,   1.0f,  EST_FLOW_MIN_HEIGHT_M)
P(est_flow_max_height_m,   float, PT_F32, 0, 1.0f,    12.0f, EST_FLOW_MAX_HEIGHT_M)
P(est_flow_max_tilt_deg,   float, PT_F32, 0, 5.0f,    45.0f, EST_FLOW_MAX_TILT_DEG)
P(est_flow_noise_mps,      float, PT_F32, 0, 0.02f,   2.0f,  EST_FLOW_NOISE_MPS)
P(est_acc_xy_noise_mps2,   float, PT_F32, 0, 0.05f,   5.0f,  EST_ACC_XY_NOISE_MPS2)
P(est_acc_xy_bias_walk,    float, PT_F32, 0, 0.001f,  0.5f,  EST_ACC_XY_BIAS_WALK)

/* ==========================================================================
 * Trường 16 bit
 *
 * Các ngưỡng CRSF nằm trong dải thô 172..1811. Công tắc ba nấc chỉ cho ĐÚNG
 * ba giá trị 172 / 992 / 1811, nên ngưỡng phải đặt ở khoảng giữa hai nấc
 * liền kề chứ không phải một con số tròn nào đó.
 * ========================================================================== */
P(rc_mode_poshold_threshold, uint16_t, PT_U16, 0, 172.0f, 1811.0f, RC_MODE_POSHOLD_THRESHOLD)
P(rc_mode_acro_threshold,    uint16_t, PT_U16, 0, 172.0f, 1811.0f, RC_MODE_ACRO_THRESHOLD)
/* 2000 = tat han nac ALTHOLD, xem fc_config.h. Dai rong hon hai nguong kia. */
P(rc_mode_althold_threshold, uint16_t, PT_U16, 0, 172.0f, 2047.0f, RC_MODE_ALTHOLD_THRESHOLD)
P(arm_switch_on_threshold,   uint16_t, PT_U16, 0, 172.0f, 1811.0f, ARM_SWITCH_ON_THRESHOLD)
P(arm_switch_off_threshold,  uint16_t, PT_U16, 0, 172.0f, 1811.0f, ARM_SWITCH_OFF_THRESHOLD)
P(arm_hold_time_ms,          uint16_t, PT_U16, 0, 0.0f,   2000.0f, ARM_HOLD_TIME_MS)
P(imu_gyro_fs_dps,           uint16_t, PT_U16, PARAM_FLAG_REBOOT, 250.0f, 2000.0f, IMU_GYRO_FS_DPS)
P(imu_calib_sample_count,    uint16_t, PT_U16, 0, 100.0f, 8000.0f, IMU_CALIB_SAMPLE_COUNT)
P(imu2_odr_hz,               uint16_t, PT_U16, PARAM_FLAG_REBOOT, 120.0f, 1920.0f, IMU2_ODR_HZ)
P(imu2_gyro_fs_dps,          uint16_t, PT_U16, PARAM_FLAG_REBOOT, 125.0f, 2000.0f, IMU2_GYRO_FS_DPS)
P(imu2_calib_sample_count,   uint16_t, PT_U16, 0, 100.0f, 8000.0f, IMU2_CALIB_SAMPLE_COUNT)
P(mag_odr_hz,                uint16_t, PT_U16, PARAM_FLAG_REBOOT, 1.0f, 200.0f, MAG_ODR_HZ)
P(mag_update_rate_hz,        uint16_t, PT_U16, 0, 1.0f,   100.0f,  MAG_UPDATE_RATE_HZ)
P(baro_calib_sample_count,   uint16_t, PT_U16, 0, 10.0f,  500.0f,  BARO_CALIB_SAMPLE_COUNT)
P(flow_range_max_mm,         uint16_t, PT_U16, 0, 500.0f, 12000.0f, FLOW_RANGE_MAX_MM)

/* Mat flow bao lau thi ha co tin cay. Xem giai thich trong fc_config.h. */
P(est_flow_timeout_ms,       uint16_t, PT_U16, 0, 50.0f,  2000.0f, EST_FLOW_TIMEOUT_MS)

/*
 * BB_ENABLE va BB_RATE_HZ CO Y KHONG nam trong bang nay.
 *
 * Chung khong phai num chinh ma la lua chon LUC BIEN DICH: `#if BB_ENABLE`
 * bao ca module blackbox (dat 0 la khong nap chut ma nao vao firmware), con
 * BB_RATE_HZ quyet dinh 256 KB dem RAM chua duoc bao nhieu giay - doi luc
 * chay thi con so "54s" in tren console thanh sai.
 *
 * Dua chung vao bang de "cho du bo" se tao ra hai tham so sua duoc ma khong
 * co tac dung gi - kieu hua hao te hon la khong co.
 */

/* ==========================================================================
 * Trường 8 bit — map kênh, đảo chiều, ma trận trục
 *
 * PHẦN LỚN GẮN CỜ DANGER. Đảo nhầm throttle nghĩa là cần ga ở đáy thành ga
 * tối đa. Sửa nhầm MỘT dấu trục IMU biến hệ toạ độ thành tay trái và làm sai
 * chiều quay gyro (định thức phải bằng +1 — đảo 0 hoặc 2 dấu thì hợp lệ, đảo
 * 1 hoặc 3 dấu thì không). App PC phải hỏi xác nhận trước khi gửi.
 * ========================================================================== */
P(rc_channel_roll,       uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 15.0f, RC_CHANNEL_ROLL)
P(rc_channel_pitch,      uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 15.0f, RC_CHANNEL_PITCH)
P(rc_channel_throttle,   uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 15.0f, RC_CHANNEL_THROTTLE)
P(rc_channel_yaw,        uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 15.0f, RC_CHANNEL_YAW)
P(rc_invert_roll,        uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 1.0f,  RC_INVERT_ROLL)
P(rc_invert_pitch,       uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 1.0f,  RC_INVERT_PITCH)
P(rc_invert_yaw,         uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 1.0f,  RC_INVERT_YAW)
P(rc_invert_throttle,    uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 1.0f,  RC_INVERT_THROTTLE)
P(arm_switch_channel,    uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 15.0f, ARM_SWITCH_CHANNEL)

/* -1 = luôn dùng ANGLE, nên phải là kiểu có dấu. */
P(rc_mode_channel,       int8_t,  PT_I8, PARAM_FLAG_DANGER, -1.0f, 15.0f, RC_MODE_CHANNEL)

/* 0 = go ACRO khoi cong tac (mac dinh). Van giu ACRO lam che do du phong. */
P(rc_mode_acro_enable,   uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 1.0f, RC_MODE_ACRO_ENABLE)

P(imu_accel_fs_g,        uint8_t, PT_U8, PARAM_FLAG_REBOOT, 2.0f, 16.0f, IMU_ACCEL_FS_G)
P(imu_axis_map_x,        uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 2.0f,  IMU_AXIS_MAP_X)
P(imu_axis_map_y,        uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 2.0f,  IMU_AXIS_MAP_Y)
P(imu_axis_map_z,        uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 2.0f,  IMU_AXIS_MAP_Z)
P(imu_axis_sign_x,       int8_t,  PT_I8, PARAM_FLAG_DANGER, -1.0f, 1.0f, IMU_AXIS_SIGN_X)
P(imu_axis_sign_y,       int8_t,  PT_I8, PARAM_FLAG_DANGER, -1.0f, 1.0f, IMU_AXIS_SIGN_Y)
P(imu_axis_sign_z,       int8_t,  PT_I8, PARAM_FLAG_DANGER, -1.0f, 1.0f, IMU_AXIS_SIGN_Z)

P(imu2_enable,           uint8_t, PT_U8, PARAM_FLAG_REBOOT, 0.0f, 1.0f,  IMU2_ENABLE)
P(imu2_accel_fs_g,       uint8_t, PT_U8, PARAM_FLAG_REBOOT, 2.0f, 16.0f, IMU2_ACCEL_FS_G)
P(imu2_axis_map_x,       uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 2.0f,  IMU2_AXIS_MAP_X)
P(imu2_axis_map_y,       uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 2.0f,  IMU2_AXIS_MAP_Y)
P(imu2_axis_map_z,       uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 2.0f,  IMU2_AXIS_MAP_Z)
P(imu2_axis_sign_x,      int8_t,  PT_I8, PARAM_FLAG_DANGER, -1.0f, 1.0f, IMU2_AXIS_SIGN_X)
P(imu2_axis_sign_y,      int8_t,  PT_I8, PARAM_FLAG_DANGER, -1.0f, 1.0f, IMU2_AXIS_SIGN_Y)
P(imu2_axis_sign_z,      int8_t,  PT_I8, PARAM_FLAG_DANGER, -1.0f, 1.0f, IMU2_AXIS_SIGN_Z)

P(mag_source,            uint8_t, PT_U8, PARAM_FLAG_REBOOT, 0.0f, 2.0f,  MAG_SOURCE)
P(mag_range_g,           uint8_t, PT_U8, PARAM_FLAG_REBOOT, 8.0f, 32.0f, MAG_RANGE_G)
P(mag_shub_odr,          uint8_t, PT_U8, PARAM_FLAG_REBOOT, 0.0f, 7.0f,  MAG_SHUB_ODR)
P(mag_osr1,              uint8_t, PT_U8, PARAM_FLAG_REBOOT, 1.0f, 8.0f,  MAG_OSR1)
P(mag_osr2,              uint8_t, PT_U8, PARAM_FLAG_REBOOT, 1.0f, 16.0f, MAG_OSR2)
P(mag_axis_map_x,        uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 2.0f,  MAG_AXIS_MAP_X)
P(mag_axis_map_y,        uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 2.0f,  MAG_AXIS_MAP_Y)
P(mag_axis_map_z,        uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 2.0f,  MAG_AXIS_MAP_Z)
P(mag_axis_sign_x,       int8_t,  PT_I8, PARAM_FLAG_DANGER, -1.0f, 1.0f, MAG_AXIS_SIGN_X)
P(mag_axis_sign_y,       int8_t,  PT_I8, PARAM_FLAG_DANGER, -1.0f, 1.0f, MAG_AXIS_SIGN_Y)
P(mag_axis_sign_z,       int8_t,  PT_I8, PARAM_FLAG_DANGER, -1.0f, 1.0f, MAG_AXIS_SIGN_Z)

/*
 * Siêu lấy mẫu baro chỉ nhận 1, 2, 4, 8, 16, 32 — min/max không diễn tả được
 * ràng buộc "phải là luỹ thừa của 2", nên tầng driver vẫn phải tự kiểm. Tăng
 * osr_pressure lên 16 thì PHẢI hạ BARO_SAMPLE_RATE_HZ xuống 25, nếu không
 * chip báo conf_err và tự bỏ mẫu.
 */
P(baro_osr_pressure,     uint8_t, PT_U8, PARAM_FLAG_REBOOT, 1.0f, 32.0f, BARO_OSR_PRESSURE)
P(baro_osr_temperature,  uint8_t, PT_U8, PARAM_FLAG_REBOOT, 1.0f, 32.0f, BARO_OSR_TEMPERATURE)
/* Chỉ nhận 0, 1, 3, 7, 15, 31, 63, 127 (0 = tắt). */
P(baro_iir_coef,         uint8_t, PT_U8, PARAM_FLAG_REBOOT, 0.0f, 127.0f, BARO_IIR_COEF)

P(flow_quality_min,      uint8_t, PT_U8, 0, 0.0f, 255.0f, FLOW_QUALITY_MIN)
P(flow_axis_map_x,       uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 1.0f,  FLOW_AXIS_MAP_X)
P(flow_axis_map_y,       uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 1.0f,  FLOW_AXIS_MAP_Y)
P(flow_axis_sign_x,      int8_t,  PT_I8, PARAM_FLAG_DANGER, -1.0f, 1.0f, FLOW_AXIS_SIGN_X)
P(flow_axis_sign_y,      int8_t,  PT_I8, PARAM_FLAG_DANGER, -1.0f, 1.0f, FLOW_AXIS_SIGN_Y)

/* Đổi thứ tự motor bằng phần mềm thay vì tháo đổi dây. */
P(dshot_motor_map_1,     uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 3.0f,  DSHOT_MOTOR_MAP_1)
P(dshot_motor_map_2,     uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 3.0f,  DSHOT_MOTOR_MAP_2)
P(dshot_motor_map_3,     uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 3.0f,  DSHOT_MOTOR_MAP_3)
P(dshot_motor_map_4,     uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 3.0f,  DSHOT_MOTOR_MAP_4)

/*
 * Bitmask motor bị đảo chiều khi bấm K2. ĐẶT VỀ 0 sau khi đã đảo xong, nếu
 * không thì mỗi lần bấm nhầm K2 lại đảo thêm một lần nữa.
 */
P(dshot_reverse_mask,    uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 15.0f, DSHOT_REVERSE_MASK)

/*
 * Dấu trục yaw. +1 nếu cặp {M1, M3} quay NGƯỢC chiều kim đồng hồ nhìn từ
 * trên xuống. Suy ra từ định luật III Newton, không phải quy ước tuỳ ý.
 */
P(mix_yaw_sign,          int8_t,  PT_I8, PARAM_FLAG_DANGER, -1.0f, 1.0f, MIX_YAW_SIGN)

/* Nằm yên thăng bằng đọc az = -9,81 thì để -1; ra +9,81 thì đổi thành +1. */
P(est_accel_z_sign,      int8_t,  PT_I8, PARAM_FLAG_DANGER, -1.0f, 1.0f, EST_ACCEL_Z_SIGN)

/*
 * 0 = dong bang bias con quay truc yaw (mac dinh, khi CHUA co tu ke).
 * Xem giai thich kem so lieu do trong fc_config.h va ekf_attitude.c.
 */
P(est_yaw_bias_learn,    uint8_t, PT_U8, PARAM_FLAG_DANGER, 0.0f, 1.0f, EST_YAW_BIAS_LEARN)

