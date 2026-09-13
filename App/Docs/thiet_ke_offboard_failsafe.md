> **⚠️ ĐÃ CŨ — KHÔNG DÙNG LÀM ĐẶC TẢ.**
> Tài liệu này đã được hợp nhất vào **[GIAO_UOC_FC_ROS2.md](GIAO_UOC_FC_ROS2.md)**
> (hợp đồng 1.0, 2026-09-13). Khi hai bên mâu thuẫn, **giao ước đúng**. Chỉ giữ lại
> để tra **lý do** của một quyết định cũ. Nhiều chỗ bên dưới đã sai so với firmware
> hiện tại (hợp đồng ARM, `custom_mode = 4`, mặc định `offboard_switch_channel`...).

# OFFBOARD và lớp phòng vệ khi lệnh từ Pi bị lỗi

Tài liệu này mô tả **những gì firmware đang làm**, không phải dự định. Mọi mục
đều trỏ tới code thật.

Câu hỏi nó trả lời: *máy tính nhúng ra lệnh cho máy bay, vậy chuyện gì xảy ra
khi lệnh đó sai, đến muộn, hoặc ngừng hẳn?*

Đọc kèm [thiet_ke_mavlink_fc.md](thiet_ke_mavlink_fc.md) mục 4.3 (định dạng bản
tin) và [ctrl_offboard.h](../Control/ctrl_offboard.h) (chi tiết từng hàm).

---

## 1. Ý tưởng gốc: nối dài thang tụt cấp đã có

Firmware vốn đã có một thang tụt cấp, mỗi tầng mất đầu vào của mình thì rơi
xuống tầng dưới — xem [ctrl_angle.c:207](../Control/ctrl_angle.c#L207):

```
POSHOLD ──mất optical flow──> ANGLE ──mất góc tin cậy──> ACRO
```

OFFBOARD chỉ thêm một bậc lên trên:

```
OFFBOARD ──lệnh Pi hỏng──> POSHOLD ──mất flow──> ANGLE ──mất góc──> ACRO
```

**Vì sao rơi về POSHOLD là câu trả lời đúng cho mọi lỗi từ Pi.** POSHOLD là vòng
giữ *vận tốc*, lấy mục tiêu từ cần điều khiển. Người lái không chạm cần → cần ở
giữa → mục tiêu bằng 0 → **máy bay phanh lại và treo tại chỗ**. Không có một
dòng logic "hover khi lỗi" nào được viết ra cả; hành vi đó là hệ quả tự nhiên
của việc chọn đúng tầng để rơi về.

Và người lái giành lại quyền chỉ bằng cách đẩy cần, không phải đi tìm công tắc.

## 2. Hiện thực: một lớp phủ, không phải nhánh song song

`ctrl_offboard` **không điều khiển gì cả**. Nó giữ một mục tiêu vận tốc, quyết
định lúc nào mục tiêu đó được dùng, và mở cổng cho hai vòng có sẵn đọc:

| Vòng | Lấy gì | Chỗ nối |
|---|---|---|
| `ctrl_poshold` | `vx`, `vy` hệ thân | [ctrl_poshold.c](../Control/ctrl_poshold.c) — thêm một nhánh trước nhánh cần điều khiển |
| `ctrl_althold` | tốc độ lên | [ctrl_althold.c](../Control/ctrl_althold.c) — thêm một nhánh trước nhánh giữ độ cao |
| `ctrl_angle` | `yaw_rate` | [ctrl_angle.c](../Control/ctrl_angle.c) — thay nguồn của `yaw_rate` |

Làm OFFBOARD thành một nhánh điều khiển riêng thì phải nhân bản từng chốt an
toàn của `ctrl_poshold` — mà nhân bản chốt an toàn là cách chắc chắn nhất để bỏ
sót một cái. Kiểu phủ lên này khiến OFFBOARD **thừa hưởng miễn phí** mọi thứ
`ctrl_poshold` đã có: trần nghiêng, chống bão hoà tích phân, và quan trọng nhất
là đường lùi khi `ekf_velocity_is_valid()` trả false.

## 3. Máy trạng thái

```
   +----------+  công tắc đọc được và OFF   +----------+
   |  KHOA    | --------------------------> |   TAT    |
   | (LOCKED) |                             | (DISABLED)|
   +----------+                             +----------+
        ^                                        |
        |                                        | công tắc ON + đủ mọi điều kiện
        |  mọi hỏng hóc                          v
        |                                   +-----------+
        +---------------------------------- | DANG_CHAY |
                                            | (ACTIVE)  |
              công tắc người lái gạt OFF    +-----------+
                    -> về TAT, không khoá
```

**KHOA dính, và đó là điểm mấu chốt.** Thoát khỏi KHOA chỉ có một cách: gạt công
tắc về OFF. Đây đúng là lập luận của `ARMING_LOCKED` trong
[arming.h](../Control/arming.h), áp vào tình huống 2 của nó — một node ROS 2
crash rồi tự khởi động lại trong khi công tắc vẫn đang ON. Không có KHOA thì máy
bay tự lao đi lại ngay khi node sống dậy, đúng lúc người lái tưởng đã xử lý xong.

**Khởi động vào thẳng KHOA.** Cắm pin trong khi công tắc bật sẵn và Pi đang phát
setpoint vẫn không làm máy bay tự chạy.

**Mất sóng đi vào KHOA, không phải TAT.** Hai chuyện này dễ gộp nhầm vì cả hai
đều làm "công tắc đọc ra không bật", nhưng ý nghĩa ngược nhau: gạt OFF là ý muốn
tường minh, còn mất sóng là *không biết* ý muốn. Gộp chung thì khi sóng về với
công tắc vẫn ON, OFFBOARD tự vào lại — đúng lỗ hổng mà KHOA sinh ra để bịt.

**Vào OFFBOARD đòi có sẵn setpoint còn hạn.** Gạt công tắc lúc Pi chưa nói gì
thì không có chuyện gì xảy ra, thay vì vào rồi hết hạn ngay sau đó.

## 4. Năm lớp phòng vệ

Xếp theo thứ tự đáng tin giảm dần.

### Lớp 0 — Công tắc RC

`offboard_switch_channel`. Gạt xuống là thoát ngay, bất kể Pi đang làm gì. Lớp
này không bao giờ hỏng vì nó không phụ thuộc vào phần mềm nào chạy đúng. Công
tắc ARM vẫn là dao cắt cuối cùng.

**Mặc định `-1` = tắt hẳn.** Nạp firmware này lên một máy bay đang bay tốt không
tự mở thêm đường ra lệnh nào.

### Lớp 1 — Cần điều khiển ghi đè

Quan trọng nhất về phản xạ: khi máy bay lao đi sai hướng, người lái **đẩy cần**
theo bản năng chứ không đi tìm công tắc.

- roll / pitch / yaw: vượt `offboard_stick_override` (0..1) là thoát. Ba cần này
  đã bị trừ vùng chết ở tầng CRSF nên bằng đúng 0 khi ở giữa.
- cần ga: xử lý riêng trong `ctrl_althold` bằng vùng chết của chính vòng đó — cần
  ga không về giữa theo lò xo nên ngưỡng kiểu trên vô nghĩa với nó.

Chạm cần nào cũng thoát **hẳn** OFFBOARD, không chia đôi thẩm quyền giữa người
và máy trên các trục khác nhau.

### Lớp 2 — Hết hạn setpoint

Bắt "Pi chết, dây đứt, node treo". Quá `offboard_timeout_ms` không có khung hợp
lệ thì rời OFFBOARD.

Khung bị từ chối **không làm mới mốc thời gian**, nên khung hỏng đếm y như khung
không tới. Nhờ vậy không cần một đường thoát riêng cho "lệnh sai định dạng" —
nó tự dẫn tới hết hạn.

### Lớp 3 — Lọc lệnh ở cửa vào

Bắt "Pi còn sống nhưng gửi số rác". Phân công:

| Kiểm tra | Ở đâu | Xử lý |
|---|---|---|
| `target_system` | `mav_link.c` | bỏ im lặng |
| `coordinate_frame` ≠ `BODY_NED` | `mav_link.c` | **loại cả khung** |
| `type_mask` sai hợp đồng | `mav_link.c` | **loại cả khung** |
| `NaN` / `Inf` | `ctrl_offboard.c` | **loại cả khung** |

**Vì sao phải lọc NaN riêng, không dựa vào chặn dải.** Mọi phép so sánh với NaN
đều trả `false`:

```c
if (v >  MAX) v =  MAX;      /* NaN >  MAX  -> false */
if (v < -MAX) v = -MAX;      /* NaN < -MAX  -> false */
/* NaN đi qua nguyên vẹn */
```

NaN lọt qua sẽ vào khâu tích phân của `ctrl_poshold` và ở lại đó vĩnh viễn — tụt
về ANGLE cũng **không** xoá, vì điều kiện xoá ở
[ctrl_angle.c:111](../Control/ctrl_angle.c#L111) chỉ kích hoạt khi công tắc chế
độ đổi. Phải chặn ở cửa, không trông vào tầng sau dọn hộ.

### Lớp 4 — Giới hạn bao

Lớp **duy nhất** bắt được ca khó nhất: *Pi gửi đều, số hợp lệ, nhưng lệnh sai* —
AprilTag nhận nhầm, VIO trôi, lỗi logic. FC không thể biết một lệnh "sai" về mặt
ngữ nghĩa, chỉ có thể giới hạn hậu quả.

Vượt bao thì **KẸP, không LOẠI**. Loại khung sẽ làm bộ đếm hết hạn chạy trong khi
Pi vẫn sống và vẫn gửi đều — chẩn đoán ra một nguyên nhân sai. Kẹp rồi đếm; kẹp
**liên tiếp** quá `offboard_clamp_limit` khung mới kết luận là hỏng và thoát.

Bao độ cao chặn **một chiều**: trên trần chỉ còn được đi xuống, dưới sàn chỉ còn
được đi lên. Cấm cả hai thì máy bay mắc kẹt ngoài bao mà không có đường về.

### Lớp 5 — Không nới lỏng arming

Người lái arm bằng công tắc ARM. OFFBOARD **không cần** arm từ xa, nên lệnh ARM
qua MAVLink vẫn bị từ chối như cũ — xem
[thiet_ke_mavlink_fc.md](thiet_ke_mavlink_fc.md) mục 6.

## 5. Bảng tra: kiểu hỏng → phản ứng

Phần này dành cho phía Pi.

| Chuyện gì xảy ra | FC phát hiện bằng | Nguyên nhân báo ra | Máy bay làm gì |
|---|---|---|---|
| Node ROS 2 chết / treo | hết hạn | `HET_HAN` | phanh, treo, về POSHOLD |
| Dây UART đứt | hết hạn | `HET_HAN` | phanh, treo |
| Pi gửi `NaN` | lọc cửa vào → hết hạn | `HET_HAN` | phanh, treo |
| Pi gửi sai `type_mask` | lọc cửa vào → hết hạn | `HET_HAN` | phanh, treo |
| Pi đòi vận tốc 50 m/s | kẹp dải liên tiếp | `KEP_DAI` | phanh, treo |
| Người lái đẩy cần | ghi đè | `DAY_CAN` | người lái nắm quyền ngay |
| Người lái gạt công tắc | công tắc | `CONG_TAC` | về POSHOLD, **không khoá** |
| Mất sóng RC | không đọc được công tắc | `MAT_SONG` | **khoá** |
| Disarm giữa chừng | trạng thái arm | `DISARM` | **khoá** |
| Mất optical flow | `ctrl_poshold` trả false | — | tụt về ANGLE |
| Mất góc tin cậy | `est.attitude_valid` | `MAT_GOC` | **khoá**, rồi ACRO |

Nguyên nhân nào khiến trạng thái thành **khoá** thì phải gạt công tắc OFF rồi ON
mới vào lại được. `CONG_TAC` là lối thoát bình thường nên không khoá.

## 6. Tham số

Chỉnh bằng CLI (`set <tên>=<giá trị>`, rồi `save`).

| Tham số | Mặc định | Dải | Ý nghĩa |
|---|---|---|---|
| `offboard_switch_channel` | **-1** | -1..15 | kênh AUX cho phép. -1 = tắt hẳn |
| `offboard_switch_on` | 1500 | 1000..2000 | ngưỡng bật, giá trị CRSF thô |
| `offboard_timeout_ms` | 500 | 100..2000 | quá hạn này coi như Pi hỏng |
| `offboard_max_vel_mps` | 2,0 | 0,1..3,0 | trần vận tốc ngang |
| `offboard_max_climb_mps` | 1,0 | 0,1..2,0 | trần tốc độ lên/xuống |
| `offboard_max_alt_m` | 5,0 | 0,5..30 | trần độ cao |
| `offboard_min_alt_m` | 0,3 | 0..3,0 | sàn độ cao |
| `offboard_max_yaw_dps` | 90 | 0..180 | trần tốc độ yaw |
| `offboard_stick_override` | 0,15 | 0,05..0,5 | ngưỡng cần ghi đè |
| `offboard_clamp_limit` | 20 | 1..200 | số khung kẹp liên tiếp trước khi thoát |

AUX1 (kênh 4) đã dành cho ARM, AUX2 (5) cho chọn chế độ, AUX3 (6) cho ghi log —
nên **AUX4 = kênh 7** là chỗ trống tiếp theo.

`offboard_min_alt_m = 0,3` nghĩa là **không hạ cánh được dưới OFFBOARD**. Hạ cánh
theo AprilTag sẽ cần hạ giá trị này; làm khi tới việc đó, đừng hạ sẵn.

## 7. Xem trạng thái

Lệnh CLI `offboard` trên console USART1:

```
KHOA                    <- trạng thái: KHOA / TAT / DANG_CHAY
KHOI_DONG               <- nguyên nhân rời gần nhất
kenh_congtac = -1
het_han_ms = 500
tuoi_ms = -1            <- tuổi setpoint gần nhất; -1 = chưa từng nhận
nhan = 0                <- khung hợp lệ
loai = 0                <- khung bị loại
kep = 0                 <- khung bị kẹp dải
vtoi_mms = 0            <- mục tiêu đang giữ, mm/s
vphai_mms = 0
vlen_mms = 0
yaw_mdps = 0
```

Ba bộ đếm phân biệt ba kiểu hỏng khác hẳn nhau:

- `nhan` tăng, `loai` = 0 → Pi khoẻ, đường truyền tốt
- `loai` tăng → Pi gửi sai hợp đồng (`type_mask`, hệ toạ độ)
- `kep` tăng → Pi gửi đúng định dạng nhưng lệnh vượt bao

## 8. Trình tự bay thử

Làm **đúng thứ tự**, không nhảy cóc.

**1. Trên bàn, đã tháo cánh quạt.**

```
set offboard_switch_channel=7
save
```

- [ ] `offboard` báo `KHOA` / `KHOI_DONG` ngay sau khi cấp nguồn
- [ ] Gạt AUX4 lên rồi xuống → vẫn `KHOA` khi chưa arm
- [ ] Rút điện Pi → `nhan` ngừng tăng, `tuoi_ms` tăng dần

**2. Treo trong nhà, có dây buộc.**

- [ ] Arm bằng công tắc, gạt AUX4 lên → `DANG_CHAY`
- [ ] Pi gửi vận tốc 0 → máy bay đứng yên
- [ ] Gạt AUX4 xuống → về POSHOLD, `CONG_TAC`, không khoá
- [ ] Đẩy cần roll → thoát ngay, `DAY_CAN`, và **KHOA**

**3. Thử từng chế độ hỏng, vẫn có dây buộc.**

- [ ] **Rút điện Pi giữa chừng** → `HET_HAN`, máy bay phanh và treo
- [ ] **Tắt tay điều khiển** → `MAT_SONG` và khoá (cẩn thận: mất sóng hiện vẫn
      cắt motor, xem mục 9)
- [ ] Bật lại Pi → OFFBOARD **không** tự vào lại. Phải gạt AUX4 một vòng

**4. Ngoài trời, vẫn giữ tay trên cần.**

## 9. Việc còn treo

**Mất sóng RC vẫn cắt motor.** [arming.h](../Control/arming.h) ghi rõ: mất sóng
chuyển ARMED → FAILSAFE → DISARMED, kèm ghi chú *"khi nào có althold chạy được
thì nên đổi thành hạ độ cao có kiểm soát; cắt motor ở độ cao 30 m là rơi tự do"*.
ALTHOLD nay đã có nhưng việc này **chưa làm**.

Đây là việc độc lập với OFFBOARD — nó đổi hành vi của **mọi** chế độ bay, nên
phải bay thử riêng. Nhưng dùng OFFBOARD mà không sửa nó thì một lỗi kép (Pi hỏng
*và* mất sóng) vẫn kết thúc bằng rơi.

**Chưa có `yaw` tuyệt đối.** Cần một vòng góc yaw, và cần từ kế QMC6309 chứng
minh được trước. Giai đoạn 1 chỉ nhận `yaw_rate`.

**Chưa có `LANDING_TARGET` và `MAV_CMD_NAV_TAKEOFF`.**

**Chú thích cũ trong `ctrl_poshold.h`.** Header đó vẫn ghi *"CHƯA LÀM — GIỮ VỊ
TRÍ THẬT"*, nhưng phần hiện thực đã có `s_tgt_n` / `s_tgt_e` và `poshold_pos_kp`,
tức đã giữ vị trí thật. Chú thích lạc hậu, chưa sửa vì nằm ngoài phạm vi lần này.
