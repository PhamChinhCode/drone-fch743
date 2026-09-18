# HƯỚNG DẪN TUNING DRONE — FCH743_V1.0

Tổng hợp các bộ PID và thông số phải chỉnh theo máy thật.
Số "đang dùng" lấy từ `App/Config/param_backup/hien_tai_toan_bo.txt` (sao lưu 2026-09-13).
Danh sách đầy đủ 167 tham số: `App/Config/param_list.h`. Ghi chú đo đạc chi tiết: `App/Config/fc_config.h`.

Đổi tham số qua CLI: `set <tên>=<giá trị>` rồi `save`. Tham số có cờ REBOOT phải khởi động lại mới có hiệu lực.

---

## 1. Các bộ PID (chỉnh từ vòng trong ra ngoài)

```
POSHOLD (vị trí → vận tốc → góc nghiêng)
  └─ ALTHOLD (độ cao → tốc độ leo → ga)        [song song, trục Z]
       └─ ANGLE (góc → tốc độ quay)
            └─ RATE (tốc độ quay → motor)   ← vòng trong cùng, chỉnh đầu tiên
```

| Vòng | Tham số | Đang dùng | Ghi chú |
|---|---|---|---|
| RATE roll/pitch | `rate_pid_{roll,pitch}_kp/ki/kd` | 0.0008 / 0.001 / 1e-8 | D gần như tắt (xem mục 4) |
| RATE yaw | `rate_pid_yaw_kp/ki/kd` | 0.0004 / 0.005 / 0 | Yaw không cần D |
| RATE phụ | `rate_pid_i_limit`, `rate_dterm_lpf_hz`, `rate_max_{roll,pitch,yaw}_dps` | 0.25, 80 Hz, 300/300/250 | |
| ANGLE | `angle_pid_kp` (chỉ P, cố ý) | 5 | `angle_max_lean_deg`=30, `angle_max_rate_dps`=200 |
| ALTHOLD | `althold_climb_kp/ki/kd`, `althold_alt_kp` | 0.25 / 0.03 / 0.01, 1.0 | |
| POSHOLD | `poshold_vel_kp/ki`, `poshold_pos_kp` | 10 / 1, 0.8 | |

### Trình tự chỉnh

1. **RATE** — chỉ P, tăng dần tới khi bắt đầu rung rồi lùi 30 % → thêm D tới khi motor kêu rít rồi lùi → thêm I sau cùng, chỉ đủ để hết trôi.
2. **ANGLE** — tăng `angle_pid_kp` tới khi góc về nhanh mà không vọt lố.
3. **ALTHOLD** — đo `althold_hover_thr` trước tiên → `althold_climb_kp` (tăng tới khi nhấp nhô rồi lùi 30 %) → `althold_climb_ki` (hết trôi theo pin yếu) → `althold_alt_kp` sau cùng, giữ thấp.
4. **POSHOLD** — `poshold_vel_kp` → `poshold_vel_ki` (chống gió) → `poshold_pos_kp` sau cùng, khi vòng vận tốc đã đứng yên gọn.

Nguyên tắc chung: luôn chỉnh **từ thấp lên**, không bao giờ từ cao xuống. Hệ số thấp làm máy ì nhưng an toàn; hệ số cao làm máy dao động và rơi.

---

## 2. Thông số phải ĐO trên máy thật

| Nhóm | Tham số | Cách đo | Trạng thái |
|---|---|---|---|
| Ga treo | `althold_hover_thr` | Bay ANGLE, đọc cột `thr` khi đứng yên độ cao. Làm tròn xuống một chút để tránh nhấp nhô do hiệu ứng mặt đất | 0.65 (đo 2026-09-14) |
| Lệch lắp IMU | `angle_trim_roll_deg`, `angle_trim_pitch_deg` | Đặt máy trên mặt phẳng đã kiểm bằng nivo, đọc roll/pitch ở `mode 16`, đặt trim bằng số đọc được **đổi dấu** | 0.7 / 0.5 |
| Thứ tự motor, chiều yaw | `dshot_motor_map_1..4`, `mix_yaw_sign` | Tháo cánh, quay thử bằng K1; arm, bật DBG_MODE_MOTOR, đẩy cần yaw phải và xem m1..m4 | Đã đo |
| Idle motor | `dshot_idle_percent` | Tăng tới khi cả 4 motor quay đều, êm ngay khi arm | 5.5 |
| Trục cảm biến | `imu_axis_*`, `imu2_axis_*`, `mag_axis_*`, `flow_axis_*` | DBG_MODE_AXISCAL, ba tư thế. Định thức phải bằng +1 (đảo 0 hoặc 2 dấu) | Đã đo |
| Hiệu chuẩn la bàn | `mag_offset_{x,y,z}_g`, `mag_scale_{x,y,z}` | Lệnh hiệu chuẩn, không gõ tay. La bàn chưa hiệu chuẩn tệ hơn không có | Đã có |
| Độ lệch từ | `est_mag_declination_deg` | Tra NOAA theo vị trí bay (Việt Nam khoảng −1°) | **Đang để 0** |
| Hệ số optical flow | `flow_rad_per_count` | Phép thử nghiêng: so flow với gyro khi quay tại chỗ | 0.002 — cần xác nhận đã đo |
| Điện áp pin | `pwr_vbat_divider` | So với đồng hồ vạn năng | 11 |
| Dòng điện | `pwr_current_mv_per_a` | Datasheet cảm biến dòng hoặc so với ampe kế | 50 |
| Mốc mặt đất laser | `LINK_DISARM_GROUND_M` (hằng số biên dịch) | Số laser đọc khi máy nằm trên đất | 0.17 m |

---

## 3. Bộ lọc và EKF (chỉnh sau khi đã bay ổn)

- **Bộ lọc:** `imu_gyro_lpf_hz`=100, `rate_dterm_lpf_hz`=80 (cả hai cần reboot). Chỉ đổi khi log blackbox cho thấy rung.
- **Độ tin cậy EKF:** các tham số `est_*_noise`, `est_*_bias_walk`. Đây KHÔNG phải hệ số PID.
  - NOISE càng lớn → bộ lọc càng ít tin cảm biến đó.
  - WALK càng lớn → cho phép đại lượng đổi nhanh hơn.
  - Chỉ chỉnh khi độ cao / vận tốc ước lượng bị trễ hoặc rung so với thực tế.
- Ràng buộc: `poshold_max_tilt_deg` phải **thấp hơn** `est_flow_max_tilt_deg`, nếu không flow bị từ chối giữa chừng.

---

## 4. Lưu ý

1. **D của roll/pitch gần như đang tắt.** Khâu D tính trên đạo hàm gyro đơn vị °/s² (`App/Control/ctrl_rate.c`). Với kd = 1e-8, gia tốc góc vài nghìn °/s² chỉ ra khoảng 0.00005 đầu ra. Nếu máy lắc hoặc vọt lố khi dừng cần, thử D trong khoảng 1e-6 … 1e-5, tăng dần và sờ nhiệt độ motor sau mỗi lần bay.
2. **Mặc định trong `fc_config.h` khác bộ đã chỉnh.** Ví dụ: roll kp 0.0010 vs 0.0008, yaw ki 0.0002 vs 0.005, `poshold_vel_ki` 0 vs 1, `poshold_pos_kp` 0 vs 0.8. Nếu flash mất tham số, máy quay về bộ chưa chỉnh → nạp lại file backup rồi `save`. Nếu cập nhật `#define` cho khớp thì `table_crc` đổi và cấu hình đã lưu bị bỏ qua — phải nạp lại và `save` ngay.
3. **Đổi phần cứng thì đo lại số tương ứng:**
   - Đổi pin / thêm tải → đo lại `althold_hover_thr`.
   - Đổi vị trí dây nguồn, pin → hiệu chuẩn lại la bàn.
   - Đổi cánh / motor → chỉnh lại RATE PID và `dshot_idle_percent`.
   - Tháo lắp lại mạch → kiểm lại trục cảm biến và trim.
4. **Sao lưu sau mỗi lần chỉnh tốt** vào `App/Config/param_backup/`.
