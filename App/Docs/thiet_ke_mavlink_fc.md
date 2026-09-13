> **⚠️ ĐÃ CŨ — KHÔNG DÙNG LÀM ĐẶC TẢ.**
> Tài liệu này đã được hợp nhất vào **[GIAO_UOC_FC_ROS2.md](GIAO_UOC_FC_ROS2.md)**
> (hợp đồng 1.0, 2026-09-13). Khi hai bên mâu thuẫn, **giao ước đúng**. Chỉ giữ lại
> để tra **lý do** của một quyết định cũ. Nhiều chỗ bên dưới đã sai so với firmware
> hiện tại (hợp đồng ARM, `custom_mode = 4`, mặc định `offboard_switch_channel`...).

# Giao ước MAVLink giữa Flight Controller và Raspberry Pi 4

Tài liệu này là **hợp đồng hai chiều**, không phải yêu cầu một phía. Mỗi mục đều
ghi rõ trạng thái: đã chạy, hay còn phải làm, và làm ở bên nào.

Bản gốc do phía Pi soạn ngày 2026-09-12, mô tả những gì ROS 2 cần. Bản này đã
đối chiếu với firmware STM32H743 thật và sửa lại những chỗ giả định sai — chủ
yếu ở mục 4 (giao diện điều khiển) và mục 5 (bảng chế độ bay).

Nguồn sự thật phía firmware: [mav_link.c](../Mavlink/mav_link.c),
[fc_state.h](../State/fc_state.h), [ctrl_poshold.h](../Control/ctrl_poshold.h),
[estimator.c](../Estimator/estimator.c).

---

## 1. Liên kết vật lý — đã thông, không cần đổi

| Hạng mục | Giá trị |
|---|---|
| Đường truyền | UART, Pi GPIO14 (TX) ↔ FC **PE0/RX**, GPIO15 (RX) ↔ FC **PE1/TX**, **GND chung** |
| Ngoại vi phía FC | **UART8**, DMA2 (TX Normal, RX Circular) |
| Thiết bị trên Pi | `/dev/ttyAMA0` (PL011, bật bằng `dtoverlay=disable-bt`) |
| Baudrate | 921600, 8N1, không flow control |
| Phiên bản giao thức | MAVLink v2 (byte mở đầu `0xFD`) |
| System ID / Component ID của FC | `1` / `1` (`MAV_COMP_ID_AUTOPILOT1`) |

Cấu hình phía Pi nằm ở [mavros.yaml](../src/drone_bringup/config/mavros.yaml).

Lưu ý: `/dev/ttyS0` (mini UART) **không** dùng được ở 921600 vì baud của nó bám
theo xung nhịp VPU và sẽ trôi.

UART8 dùng riêng cho MAVLink. USART3 đang chở khung nhị phân riêng của dự án tới
ESP32, USART1 là console CLI — không trộn.

---

## 2. Nguyên tắc phân chia trách nhiệm

Mục này quyết định mọi thứ còn lại, nên đặt lên trước.

**Vòng nào đóng ở đâu:**

```
  Pi 4 (ROS 2, Linux không thời gian thực)
    └─ nhiệm vụ, AprilTag, VIO ─> VÒNG VỊ TRÍ ─> lệnh VẬN TỐC ─┐
                                                   10-20 Hz     │
                                                                ▼
  FC (STM32H743, vòng lặp cứng)                        SET_POSITION_TARGET
    └─ VÒNG VẬN TỐC ─> góc nghiêng ─> VÒNG GÓC ─> VÒNG TỐC ĐỘ GÓC ─> trộn
       (ctrl_poshold)                  (ctrl_angle)   (ctrl_rate)
```

**Pi gửi xuống VẬN TỐC — không phải góc nghiêng, không phải vị trí.** Ba lý do,
đều rút ra từ code hiện có chứ không phải lý thuyết chung:

1. **FC đã có sẵn đúng vòng đó.** [ctrl_poshold.h](../Control/ctrl_poshold.h)
   tuy mang tên "poshold" nhưng chính header nói rõ *"Đây là giữ VẬN TỐC, chưa
   phải giữ VỊ TRÍ"*. Chuỗi của nó là `vận tốc mong muốn → P+I → góc nghiêng`.
   Cho Pi điều khiển vận tốc chỉ là đổi nguồn setpoint, mọi tầng dưới và mọi
   tham số PID đã chỉnh đều giữ nguyên.

2. **Lệnh vị trí bị chính bộ ước lượng cấm.** `est.position_m.x/y` là tích phân
   vận tốc optical flow — dẫn đường suy tính thuần tuý. [estimator.c](../Estimator/estimator.c)
   ghi: sai số tích luỹ và *"KHÔNG BAO GIỜ tự hết"*. Ra lệnh vị trí tuyệt đối
   lên một ước lượng đang trôi nghĩa là máy bay bay để bù cho phần trôi không
   có thật. Vòng P vị trí trên FC cũng chưa tồn tại.

3. **Lệnh góc sai ở chế độ hỏng.** Gửi góc tức là đẩy vòng vận tốc lên Pi, qua
   ROS 2 trên Linux không thời gian thực. Khi Pi khựng: lệnh góc → máy bay giữ
   nguyên độ nghiêng và **tăng tốc đi mất**; lệnh vận tốc → FC hết hạn setpoint,
   đặt mục tiêu về 0 → **phanh lại rồi treo**. Khác biệt này là toàn bộ vấn đề.

Thêm một điểm không hiển nhiên: `ctrl_poshold_update()` trả `false` khi mất
optical flow, và `ctrl_angle` tự lùi về chế độ ANGLE. **Lưới an toàn đó chỉ tồn
tại ở tầng vận tốc.** Nếu Pi ra lệnh góc, nó hoàn toàn không biết flow đã chết.

Hệ quả: **vòng vị trí vẫn có, nhưng nằm trên Pi** — nơi có AprilTag và VIO làm
nguồn tham chiếu tuyệt đối. Pi quy sai số vị trí ra lệnh vận tốc. Vì vòng ngoài
vốn chậm, Pi chỉ cần 10–20 Hz và miễn nhiễm với jitter đường truyền.

---

## 3. FC → Pi: bảng phát

### 3.1 Đang phát

Bốn dòng đầu đo thật bằng `pymavlink` ngày 2026-09-12. Năm dòng sau là tần số
**đã cấu hình trong firmware nhưng chưa đo lại trên Pi** — xem checklist mục 9.

| Message | ID | Tần số | Trạng thái | Topic ROS sinh ra |
|---|---|---|---|---|
| `HEARTBEAT` | 0 | 1 Hz | đo được 1.0 | `/mavros/state` |
| `SYS_STATUS` | 1 | 2 Hz | đo được 2.1 | `/mavros/battery` |
| `ATTITUDE` | 30 | 50 Hz | đo được 50.6 | `/mavros/imu/data` |
| `VFR_HUD` | 74 | 10 Hz | đo được 10.1 | `/mavros/vfr_hud` |
| `GLOBAL_POSITION_INT` | 33 | 10 Hz | **mới, chưa đo** | `/mavros/global_position/rel_alt` |
| `LOCAL_POSITION_NED` | 32 | 30 Hz | **mới, chưa đo** | `/mavros/local_position/pose` |
| `HIGHRES_IMU` | 105 | 50 Hz | **mới, chưa đo** | `/mavros/imu/data_raw`, `/mavros/imu/mag` |
| `BATTERY_STATUS` | 147 | 1 Hz | **mới, chưa đo** | `/mavros/battery` |
| `EXTENDED_SYS_STATE` | 245 | 1 Hz | **mới, chưa đo** | `/mavros/extended_state` |
| `COMMAND_ACK` | 77 | theo sự kiện | đã có | kết quả `/mavros/cmd/*` |
| `AUTOPILOT_VERSION` | 148 | khi được hỏi | **mới, chưa đo** | trả lời lệnh 520 |

Tổng khoảng **7,9 KB/s**. Ở 921600 baud (92 KB/s) là 8,6% băng thông — còn rất
rộng, không cần cắt giảm tần số.

### 3.2 Ghi chú từng bản tin

**`GLOBAL_POSITION_INT`.** Bo mạch **không có GPS**. Bản tin vẫn phát vì
[optical_flow_node.py:45](../src/drone_perception/drone_perception/optical_flow_node.py#L45)
dùng `relative_alt` (mm, so với mặt đất lúc khởi động) để quy đổi dịch chuyển
pixel ra mét.

Quy ước "không biết" theo đúng đặc tả: `lat = lon = 0`, `hdg = 65535`. Chỉ ba
nhóm trường sau mang thông tin thật:

| Trường | Nguồn | Ghi chú |
|---|---|---|
| `relative_alt` | `est.altitude_m` × 1000 | EKF hợp nhất baro + laser MTF01P |
| `alt` | `baro.altitude_m` × 1000 | độ cao áp suất, **không phải MSL thật** |
| `vx`/`vy`/`vz` | `est.velocity_mps` × 100 | hệ NED, cm/s |

**`LOCAL_POSITION_NED`.** Độ tin cậy **không đồng đều giữa các trục**:

- `z`, `vz` — từ EKF độ cao, đáng tin.
- `x`, `y` — tích phân vận tốc optical flow. Chỉ dùng được để biết "đã rời chỗ
  cũ bao xa" trong vài chục giây gần đây. **Tuyệt đối không dùng làm gốc toạ độ
  cho bay theo lộ trình hay quay về điểm xuất phát.**
- `vx`, `vy` — từ optical flow, đáng tin **khi còn flow**. Mất flow thì
  `est.position_valid` hạ xuống, nhưng MAVLink không có chỗ chở cờ này. Phía Pi
  theo dõi gián tiếp qua `SYS_STATUS`: bit `MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW`
  trong trường `onboard_control_sensors_health`.

**`HIGHRES_IMU` thay cho `SCALED_IMU`.** Bản gốc đề nghị `SCALED_IMU` (26) hoặc
`HIGHRES_IMU` (105). Chọn cái sau vì `SCALED_IMU` nhét gyro vào `int16` đơn vị
mrad/s, mà 2000 dps = 34900 mrad/s — **tràn kiểu ngay trong dải đo bình thường
của ICM20602**. `HIGHRES_IMU` dùng `float` nên không có bẫy đó, và chở luôn cả
từ kế lẫn khí áp trong cùng một khung.

Trường `fields_updated` nói rõ trường nào có thật. Bit từ kế (6–8) **chỉ bật khi
QMC6309 vừa `healthy` vừa `calibrated`** — từ kế chưa hiệu chuẩn lệch hướng hàng
chục độ, báo "có dữ liệu" lúc đó tệ hơn báo "không có". Phía Pi phải đọc
`fields_updated` chứ đừng giả định mọi trường đều hợp lệ.

**`BATTERY_STATUS`.** Dự án đo điện áp tổng và dòng điện, **không đo từng cell**.
`voltages[]` điền điện áp tổng chia số cell vào đúng N ô đầu để MAVROS hiện đúng
số cell; các ô còn lại là `UINT16_MAX`. `battery_remaining = -1` ("không biết")
là cố ý: suy phần trăm từ điện áp lúc đang tải sai lệch lớn, báo không biết
trung thực hơn báo một con số bịa.

**`EXTENDED_SYS_STATE`.** Chưa arm → `ON_GROUND`. Đã arm → dựa vào độ cao với
ngưỡng 0,5 m. Không có `est.altitude_valid` thì trả `UNDEFINED`.

### 3.3 Đã bỏ, kèm lý do

**`GPS_RAW_INT` (24) — không phát.** Bản gốc xếp vào nhóm "nên có" với lý do
*"thiếu thì `NavSatFix.status` luôn báo no fix"*. Nhưng bo mạch không có GPS, nên
bản tin sẽ mang `fix_type = 0`, và MAVROS cũng cho ra đúng "no fix". Phát một
bản tin rỗng vĩnh viễn không đổi được gì, chỉ thêm rác trên đường truyền. Khi nào
lắp GPS thật thì bật lại.

---

## 4. Pi → FC: giao diện điều khiển

### 4.1 Đang xử lý

| Message / lệnh | ID | FC làm gì |
|---|---|---|
| `HEARTBEAT` | 0 | mốc theo dõi đường truyền, hết 3000 ms coi như mất Pi |
| `COMMAND_LONG` / `MAV_CMD_COMPONENT_ARM_DISARM` (400) | 76 | **chỉ chiều DISARM** — xem mục 6 |
| `COMMAND_LONG` / `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES` (520) | 76 | trả `AUTOPILOT_VERSION` |

**Mọi `COMMAND_LONG` đều được trả `COMMAND_ACK`**, kể cả lệnh không hỗ trợ
(`MAV_RESULT_UNSUPPORTED`). Đây là hợp đồng bắt buộc: MAVROS chặn service
`/mavros/cmd/*` cho tới khi có ACK, thiếu ACK thì mọi lệnh treo tới timeout và
nhìn từ ROS sẽ giống "lệnh thất bại" dù FC đã làm xong.

`COMMAND_ACK` chứa đúng `command` vừa nhận (không để 0), và `target_system` /
`target_component` lấy thẳng từ `sysid` / `compid` của khung gửi tới — với MAVROS
là 255 / 190.

### 4.2 Trạng thái các bản tin điều khiển

| Message | ID | Sinh từ | Tình trạng |
|---|---|---|---|
| `SET_POSITION_TARGET_LOCAL_NED` | 84 | `/mavros/setpoint_raw/local` | **đã làm** — xem 4.3 |
| `SET_MODE` | 11 | `/mavros/set_mode` | **chưa làm** |
| `LANDING_TARGET` | 149 | `/mavros/landing_target/pose` | **chưa làm** |
| `COMMAND_LONG` / `MAV_CMD_NAV_TAKEOFF` (22) | 76 | `/mavros/cmd/takeoff` | ACK `UNSUPPORTED` |

Hiện gọi `/mavros/set_mode` sẽ **không có tác dụng và cũng không báo lỗi rõ**:
`SET_MODE` không dùng `COMMAND_ACK`, MAVROS xác nhận bằng cách chờ `custom_mode`
trong `HEARTBEAT` đổi — mà FC chưa đổi. Đừng dựa vào nó cho tới khi mục này xong.

### 4.3 `SET_POSITION_TARGET_LOCAL_NED` — đặc tả chốt trước khi viết code

Ghi ở đây để hai bên viết song song mà vẫn khớp.

**FC chỉ đọc vận tốc và `yaw_rate`. Bỏ qua mọi trường vị trí, gia tốc và
`yaw` tuyệt đối.**

| Trường | FC dùng | Ánh xạ |
|---|---|---|
| `vx`, `vy` | có | mục tiêu vận tốc ngang → `ctrl_poshold` |
| `vz` | có | mục tiêu tốc độ lên → `ctrl_althold` (`vz = 0` tự chốt độ cao) |
| `yaw_rate` | có | tốc độ quay, rad/s |
| `x`,`y`,`z` | **bỏ** | lý do ở mục 2 |
| `afx`,`afy`,`afz` | **bỏ** | không có vòng gia tốc |
| `yaw` | **bỏ** | xem ngay dưới |

> **`yaw` tuyệt đối không dùng được ở giai đoạn 1.** Firmware không có vòng góc
> yaw — [ctrl_angle.c:186](../Control/ctrl_angle.c#L186) chỉ điều khiển *tốc độ*
> yaw, kèm lý do: *"Không có la bàn thì yaw ước lượng trôi dần, giữ hướng theo
> nó là tự làm máy bay quay đi."* Từ kế QMC6309 trên nhánh này chưa chứng minh
> được, nên phía Pi phải tự đóng vòng yaw: tính sai số hướng rồi gửi xuống
> `yaw_rate`.

**`coordinate_frame` = `MAV_FRAME_BODY_NED` (8).** `ctrl_poshold` vốn xoay vận
tốc về hệ thân để tính, và sai số AprilTag cũng sinh ra ở hệ camera — dùng hệ
thân bỏ được một phép xoay ở mỗi đầu. Chuyển sang `MAV_FRAME_LOCAL_NED` (1) sau,
khi yaw từ từ kế QMC6309 đã chứng minh đáng tin.

**`type_mask` — sửa lỗi trong bản gốc.** Bản gốc ghi `0b0000111111000111` kèm chú
thích *"FC chỉ được đọc vx, vy, vz, yaw"*. Hai thứ này mâu thuẫn: **bit 10 trong
mask đó đang bật, tức yaw bị bỏ qua.** Mask đúng là:

```
type_mask = 0b0000011111000111 = 0x07C7
```

| Bit | Trường | Giá trị | Nghĩa |
|---|---|---|---|
| 0–2 | `x`, `y`, `z` | `111` | bỏ qua |
| 3–5 | `vx`, `vy`, `vz` | `000` | **dùng** |
| 6–8 | `afx`, `afy`, `afz` | `111` | bỏ qua |
| 9 | `force` | `1` | bỏ qua |
| 10 | `yaw` | `1` | bỏ qua |
| **11** | **`yaw_rate`** | **`0`** | **dùng** |

Bản gốc ghi `0b0000111111000111` kèm chú thích *"FC chỉ được đọc vx, vy, vz,
yaw"* — mâu thuẫn với chính nó, vì bit 10 trong mask đó đang bật. Giá trị chốt
là `0x07C7`.

**Khung sai `type_mask` bị TỪ CHỐI CẢ KHUNG**, không phải bỏ qua từng trường.
Sai mask nghĩa là hai bên đang hiểu hợp đồng khác nhau; nhặt bừa phần đọc được
rồi bay theo là bay theo một lệnh mình không hiểu. Từ chối thì FC hết hạn
setpoint và về POSHOLD — máy bay phanh lại và treo, còn nguyên nhân hiện rõ ở
bộ đếm `loai` của lệnh CLI `offboard`.

Gửi mask như bản gốc thì FC sẽ lờ yaw, và sẽ không ai hiểu vì sao máy bay không
quay. FC **bắt buộc** phải tôn trọng `type_mask` — đọc hết mọi trường kể cả ô Pi
cố tình để trống là lỗi kinh điển, máy bay sẽ lái theo số rác.

**Tần số:** Pi gửi 10–20 Hz là đủ. Vòng vận tốc đóng trên FC ở tốc độ vòng lặp
chính nên không cần Pi chạy nhanh.

**Hết hạn setpoint.** Quá `offboard_timeout_ms` (mặc định **500 ms**) không nhận
được khung hợp lệ, FC rời OFFBOARD về POSHOLD. Vì POSHOLD lấy mục tiêu vận tốc từ
cần điều khiển, mà cần đang ở giữa, mục tiêu thành 0 — **máy bay phanh lại và
treo**. Đây chính là thứ làm cho lệnh vận tốc an toàn hơn lệnh góc.

**Công tắc cho phép trên tay điều khiển.** OFFBOARD chỉ vào được khi người lái
gạt công tắc `offboard_switch_channel`. **Mặc định là `-1`, tức TẮT HẲN** — nạp
firmware mới không tự mở thêm đường ra lệnh nào.

**Toàn bộ hành vi khi lệnh từ Pi bị lỗi** — năm lớp phòng vệ, máy trạng thái, và
bảng tham số — nằm ở [thiet_ke_offboard_failsafe.md](thiet_ke_offboard_failsafe.md).
Phía Pi nên đọc mục 5 của tài liệu đó để biết FC sẽ phản ứng thế nào với từng
kiểu hỏng.

---

## 5. Bảng `custom_mode` — đã sửa theo firmware

Firmware khai `autopilot = MAV_AUTOPILOT_GENERIC`, nên MAVROS **không biết** tên
chế độ bay và `/mavros/state` trả về `mode: "CMODE(2)"` thay vì một cái tên.

Hệ quả cho phía Pi: gọi `/mavros/set_mode` phải truyền `custom_mode` dạng **số**
và để `base_mode = 0`. Truyền chuỗi `"GUIDED"` hay `"OFFBOARD"` không có tác dụng.

> **Bản gốc đọc nhầm chỗ này.** Nó thấy `custom_mode = 2` rồi suy ra là POSITION,
> và dựng một bảng 6 mục quanh giả định đó. Thực tế `custom_mode` chở thẳng
> `flight_mode_t` của firmware, và **2 = ALTHOLD**. Máy bay lúc đo đang ở chế độ
> giữ độ cao, không phải giữ vị trí.

Bản gốc cũng đã thống nhất *"firmware chốt con số, Pi bám theo"*. Con số thật,
lấy từ [fc_state.h](../State/fc_state.h):

| `custom_mode` | Tên firmware | Ý nghĩa | Tình trạng |
|---|---|---|---|
| 0 | `FLIGHT_MODE_ACRO` | điều khiển tốc độ góc trực tiếp | đã có |
| 1 | `FLIGHT_MODE_ANGLE` | tự cân bằng theo góc nghiêng | đã có |
| 2 | `FLIGHT_MODE_ALTHOLD` | giữ độ cao (baro + laser) | đã có |
| 3 | `FLIGHT_MODE_POSHOLD` | giữ vận tốc bằng optical flow | đã có |
| 4 | `FLIGHT_MODE_OFFBOARD` | nhận lệnh vận tốc từ Pi | đã có, mặc định tắt |

Không có LAND và RTL. RTL cần một nguồn vị trí tuyệt đối mà bo mạch không có
(xem mục 2) — đừng đưa vào bảng cho tới khi có GPS hoặc VIO.

**`base_mode`.** FC luôn bật `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED` (1) và
`MANUAL_INPUT_ENABLED` (64); bật thêm `STABILIZE_ENABLED` (16) ở mọi chế độ trừ
ACRO. Bit `MAV_MODE_FLAG_SAFETY_ARMED` (128) bám đúng `motor.armed` — đây là
nguồn duy nhất cho trường `armed` của `/mavros/state`.

Giá trị 81 đo được trong bản gốc (`64 | 16 | 1`) là trạng thái chưa arm, ứng với
một chế độ có cân bằng. Đúng như thiết kế.

---

## 6. Arming — vì sao Pi không arm được

**FC từ chối mọi lệnh ARM đến qua MAVLink. Đây là chủ ý, không phải thiếu sót.**

Lệnh `MAV_CMD_COMPONENT_ARM_DISARM` với `param1 = 1` luôn nhận
`MAV_RESULT_TEMPORARILY_REJECTED`. Chiều DISARM (`param1 = 0`) thì luôn được chấp
nhận — cắt khẩn cấp từ Pi phải luôn hoạt động.

Lý do đầy đủ nằm trong [mav_link.c](../Mavlink/mav_link.c), tóm tắt: toàn bộ
[arming.c](../Control/arming.c) đặt trên một quy tắc — arm phải là chủ ý của
người đang đứng cạnh máy bay, thể hiện bằng cách gạt công tắc trên tay điều
khiển. Cho arm qua MAVLink là mở đúng cái cửa đó: một node ROS 2 lỗi, hoặc chỉ
đơn giản là khởi động lại đúng lúc, sẽ làm cánh quạt quay khi có người đang cầm
máy bay.

**Nghĩa là với phía Pi:**

- `/mavros/cmd/arming` với `value: true` sẽ trả về **nhanh** (có ACK, không treo)
  nhưng `success: false`. **Đây là hành vi đúng, đừng báo lỗi.**
- `/mavros/cmd/arming` với `value: false` hoạt động bình thường — dùng làm nút
  cắt khẩn cấp.
- Quy trình bay: người lái arm bằng công tắc RC, sau đó Pi mới ra lệnh vận tốc.
  OFFBOARD **không cần** arm từ xa.

**Còn để ngỏ.** Nếu sau này thật sự cần arm từ Pi, cách giữ được nguyên tắc an
toàn là lấy công tắc RC làm cổng đồng ý: FC chỉ chấp nhận lệnh ARM khi công tắc
arming trên tay điều khiển **đang bật** — Pi chỉ "bấm nút" sau khi người lái đã
cho phép. Cần sửa `arming.c` thêm một API arm có điều kiện. Chưa làm.

---

## 7. Điểm cần lưu ý khác

**`AUTOPILOT_VERSION`.** MAVROS gửi `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES`
(520) lúc kết nối, thử 5 lần rồi bỏ cuộc với cảnh báo `your FCU don't support
AUTOPILOT_VERSION, switched to default capabilities`. FC nay đã trả lời.

FC **chỉ khai `MAV_PROTOCOL_CAPABILITY_MAVLINK2`, không khai gì thêm** — cố ý.
Khai thừa (MISSION_FLOAT, FTP, SET_POSITION_TARGET_LOCAL_NED...) sẽ khiến MAVROS
bật những plugin mà firmware chưa hỗ trợ, rồi chờ phản hồi không bao giờ tới.
Khi giai đoạn 2 xong thì bổ sung đúng cờ tương ứng.

**Timestamp.** `ATTITUDE`, `LOCAL_POSITION_NED`, `GLOBAL_POSITION_INT` dùng
`time_boot_ms` lấy từ `HAL_GetTick()`. `HIGHRES_IMU` dùng `time_usec` tính bằng
mili giây × 1000 — **cố ý không dùng `micros()`**, vì hàm đó đọc TIM2 32 bit nên
tràn vòng sau 71 phút, đủ để EKF trên Pi sắp sai thứ tự phép đo.

**Ngắt UART8.** TX chạy DMA chế độ Normal, nên `HAL_UART_TxCpltCallback` tới từ
ngắt UART8 chứ không phải ngắt DMA. **UART8 global interrupt bắt buộc phải bật**,
nếu không đường gửi chết sau đúng một gói. Đã bật (ưu tiên 10).

---

## 8. Việc phải sửa ở phía Pi

Ghi lại ở đây để hai bên không đổ lỗi nhầm chỗ khi nghiệm thu.

**8.1 — Bảng `custom_mode`.** Sửa theo mục 5. Bảng 6 mục trong bản gốc
(STABILIZE / ALT_HOLD / POSITION / OFFBOARD / LAND / RTL) không khớp firmware.
Đặc biệt: đừng coi `custom_mode = 2` là "giữ vị trí".

**8.2 — `type_mask` sai một bit.** Sửa theo mục 4.3: `0x0BC7`, không phải
`0b0000111111000111`. Bit 10 phải bằng 0 thì yaw mới có hiệu lực.

**8.3 — Vòng điều khiển ra vận tốc, không ra vị trí.** Node điều khiển phải quy
sai số vị trí (từ AprilTag / VIO) thành lệnh vận tốc rồi mới gửi xuống. Đừng gửi
`x`,`y`,`z` — FC bỏ qua. Lý do ở mục 2.

**8.4 — Sai topic hạ cánh chính xác.**
[landing_target_bridge_node.py:44](../src/drone_control/drone_control/landing_target_bridge_node.py#L44)
đang publish `mavros_msgs/LandingTarget` lên `/mavros/landing_target/raw`. Topic
đó **không tồn tại** trong MAVROS. Plugin `landing_target` thật sự cung cấp:

| Topic | Kiểu | Chiều |
|---|---|---|
| `/mavros/landing_target/pose` | `geometry_msgs/PoseStamped` | Pi → FC (sinh `LANDING_TARGET`) |
| `/mavros/landing_target/pose_in` | `geometry_msgs/PoseStamped` | FC → Pi |
| `/mavros/landing_target/lt_marker` | `geometry_msgs/Vector3Stamped` | FC → Pi |

Sai cả tên topic lẫn kiểu message. Ở trạng thái hiện tại dữ liệu AprilTag không
bao giờ tới được FC. Phải sửa trước khi thử hạ cánh chính xác — dù FC cũng chưa
xử lý `LANDING_TARGET` (mục 4.2), nên hai bên cùng còn việc.

**8.5 — Kỳ vọng đúng về arming.** `/mavros/cmd/arming{value: true}` trả
`success: false` là **đúng thiết kế**, không phải lỗi. Xem mục 6.

**8.6 — `optical_flow_node` nên kiểm tra sức khoẻ flow.** `vx`/`vy` trong
`LOCAL_POSITION_NED` mất ý nghĩa khi hết flow. Theo dõi bit
`MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW` trong `SYS_STATUS` thay vì giả định số luôn
hợp lệ.

---

## 9. Checklist nghiệm thu

Chạy trên Pi, đánh dấu từng mục.

### 9.1 FC phát đủ bản tin

```bash
python3 -c "
from pymavlink import mavutil; from collections import Counter; import time
m = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600); m.wait_heartbeat()
c = Counter(); t0 = time.time()
while time.time()-t0 < 8:
    x = m.recv_match(blocking=True, timeout=2)
    if x: c[x.get_type()] += 1
for k,v in sorted(c.items()): print('%-24s %5.1f Hz' % (k, v/8.0))
"
```

- [ ] `GLOBAL_POSITION_INT` ~10 Hz
- [ ] `LOCAL_POSITION_NED` ~30 Hz
- [ ] `HIGHRES_IMU` ~50 Hz
- [ ] `BATTERY_STATUS` ~1 Hz, `EXTENDED_SYS_STATE` ~1 Hz
- [ ] `relative_alt` đổi đúng chiều khi nhấc drone lên
- [ ] Không còn cảnh báo `VER: ... switched to default capabilities` trong log MAVROS

### 9.2 Topic ROS có dữ liệu

```bash
ros2 launch drone_bringup estimation.launch.py
ros2 topic hz /mavros/global_position/rel_alt
ros2 topic hz /mavros/imu/data_raw
ros2 topic echo /mavros/state --once
```

- [ ] `/mavros/state` báo `connected: true`
- [ ] `/mavros/global_position/rel_alt` có tần số ổn định
- [ ] `armed` đổi theo trạng thái thật khi gạt công tắc RC
- [ ] `mode` hiện `CMODE(n)` khớp bảng mục 5 khi đổi công tắc chế độ

### 9.3 Luồng lệnh có ACK — CHỈ chạy khi đã tháo cánh quạt

```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
```

- [ ] Lệnh `true` trả về **dưới 1 giây** với `success: false` — có ACK, không treo
- [ ] Lệnh `false` trả về `success: true` và máy bay disarm thật
- [ ] Arm bằng công tắc RC làm `/mavros/state` đổi `armed: true`

### 9.4 Giai đoạn 2 — chưa áp dụng

Bỏ qua cho tới khi mục 4.2 hoàn thành: `/mavros/set_mode`,
`/mavros/setpoint_raw/local`, `/mavros/cmd/takeoff`, hạ cánh theo AprilTag.

---

## Phụ lục — tra cứu nhanh

- Định nghĩa bản tin chuẩn: <https://mavlink.io/en/messages/common.html>
- Ánh xạ plugin ↔ topic của MAVROS: <https://wiki.ros.org/mavros>
- Cấu hình plugin đang bật: [mavros.yaml](../src/drone_bringup/config/mavros.yaml)
- Lớp bản tin phía FC: [mav_link.c](../Mavlink/mav_link.c) / [mav_link.h](../Mavlink/mav_link.h)
- Lớp truyền tải UART8: [mav_port.c](../Mavlink/mav_port.c)
