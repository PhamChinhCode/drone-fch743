# Kế hoạch tích hợp LSM6DSV + QMC6309

Module IMU thứ hai trên **SPI3**, chân ngắt **PD7**. Từ kế QMC6309 nối vào
**sensor hub** (I2C master nội bộ) của LSM6DSV, không có dây I2C ra ngoài.

---

## Mục tiêu và điều KHÔNG phải mục tiêu

Mục đích ban đầu là "giảm nhiễu bằng IMU thứ hai". Cần nói rõ ngay từ đầu
cái gì mua được và cái gì không, vì nó quyết định thứ tự công việc.

| Kỳ vọng | Thực tế |
|---|---|
| Trung bình 2 IMU giảm nhiễu điện tử | ✅ Đúng — hai nguồn độc lập cho σ/√2, khoảng 30% |
| Trung bình 2 IMU giảm rung khung | ❌ **Sai** — rung là tín hiệu THẬT, cả hai chip trên cùng bo đo ra giống hệt nhau. Trung bình hai số giống nhau vẫn ra chính số đó. |
| Từ kế mở khoá giữ hướng | ✅ Đây mới là phần thưởng lớn nhất |
| IMU thứ hai phát hiện hỏng hóc | ✅ Giá trị thật của dự phòng |

Trên drone đang bay, nhiễu gyro **bị chi phối bởi rung khung**. Bộ lọc 100 Hz
hiện tại đã lấy trung bình ~80 mẫu mỗi lần, nên phần nhiễu trắng còn lại vốn
đã rất nhỏ.

→ Kế hoạch **ưu tiên từ kế trước, hợp nhất IMU sau**, và **giai đoạn 2 là cổng
quyết định dựa trên số đo thật** chứ không phải giả định.

---

## Bảng tiến độ

| GĐ | Nội dung | Đụng vào code đang bay? | Trạng thái |
|---|---|---|---|
| 1 | Driver LSM6DSV đọc thô | Không | ✅ Chạy được, còn nợ 2 mục kiểm |
| 2 | Căn trục + đo nhiễu — **CỔNG QUYẾT ĐỊNH** | Không | ✅ XONG — cổng đã đóng |
| **R** | **Chống rung khung — cộng hưởng** | Cơ khí | 🔴 **ƯU TIÊN CAO NHẤT** |
| 3 | Sensor hub + QMC6309 | Không | ✅ **XONG** — đo được 0,45 G |
| 4 | Hiệu chuẩn từ kế | Không | ✅ XONG — nhưng 🔴 **mất từ kế khi cắm pin** |
| 5 | ~~Hợp nhất gyro/accel~~ | — | ❌ **HUỶ** — 2C cho thấy rung khung chi phối |
| 6 | Yaw từ mag vào EKF → giữ hướng | **Có** | ⏸ |

Giai đoạn 1–4 **không chạm** vào `App/Control/`, `App/Estimator/`, hay
`g_fc.imu`. Firmware vẫn bay được y như trước trong suốt bốn giai đoạn đầu.

---

## Giai đoạn 1 — Driver LSM6DSV đọc thô

**Mục tiêu:** chứng minh đường SPI3 + DRDY hoạt động. Không rủi ro.

### Thiết kế

Sao chép kiến trúc của `icm20602.c`: máy trạng thái, cờ `s_busy` chống chồng
mẫu, `FC_DMA_BUFFER` cho bộ đệm, chốt timestamp ngay trong DRDY ISR.

```
PD7 DRDY (EXTI9_5, prio 4) ──> lsm6dsv_drdy_isr()
                                   │ SPI3 DMA burst 14 byte (0x20..0x2D)
                                   ▼
                        lsm6dsv_spi_complete_isr() → process_sample()
                                   │
                                   ▼
                              g_fc.imu2      ← KHÔNG phải g_fc.imu
```

### Khác biệt so với ICM20602 — những chỗ dễ sai

| Điểm | ICM20602 | LSM6DSV |
|---|---|---|
| Chế độ SPI | mode 0 | **mode 3** (CPOL=1, CPHA=1) |
| Thứ tự byte | **big-endian** (H trước) | **little-endian** (L trước) |
| Thứ tự khối dữ liệu | accel → temp → gyro | **temp → gyro → accel** |
| WHO_AM_I | `0x75` = `0x12` | **`0x0F` = `0x70`** |
| Nhiệt độ | raw/326.8 + 25 | **raw/256 + 25** |
| Auto-increment | mặc định | phải bật `IF_INC` trong CTRL3 |

Cả hai đều dùng bit 7 = 1 để đọc, và cả hai đều đọc đúng 14 byte.

### ODR đặt 1920 Hz, KHÔNG phải 8 kHz

Cố ý. Ba lý do:

1. Dữ liệu bị lọc xuống 100 Hz trước khi dùng — tốc độ cao hơn vô ích.
2. Mỗi ngắt tốn CPU. Vòng PID 4 kHz là thứ không được phép chậm lại.
3. 1920 Hz vẫn cao gấp ~10 lần băng thông quan tâm.

### Nguồn thanh ghi

Lấy từ driver chính thức của ST (`STMicroelectronics/lsm6dsv-pid`), không
viết theo trí nhớ. Các giá trị đã đối chiếu:

```
WHO_AM_I = 0x0F, giá trị = 0x70
CTRL1 = 0x10 (bit 0-3 = odr_xl, bit 4-6 = op_mode_xl)
CTRL2 = 0x11 (bit 0-3 = odr_g,  bit 4-6 = op_mode_g)
CTRL3 = 0x12 (bit0 sw_reset, bit2 if_inc, bit6 bdu, bit7 boot)
CTRL6 = 0x15 (bit 0-3 = fs_g)
CTRL8 = 0x17 (bit 0-1 = fs_xl)
INT1_CTRL = 0x0D (bit0 drdy_xl, bit1 drdy_g)
ODR: 120Hz=6, 240=7, 480=8, 960=9, 1920=0x0A
fs_g:  125dps=0, 250=1, 500=2, 1000=3, 2000=4
fs_xl: 2g=0, 4g=1, 8g=2, 16g=3
```

### ⚠️ Hai hằng số CHƯA được kiểm chứng

Hệ số đổi thang lấy theo bảng chuẩn của họ LSM6DS. Chúng **có thể sai** và
driver được thiết kế để tự phát hiện:

- **Thang accel** → kiểm bằng trọng lực. Để yên trên bàn, `|a|` phải ≈ 9.81.
  Console `IMU2` in sẵn cột này. Lệch > 2% là sai hệ số.
- **Thang gyro** → kiểm bằng cách so với ICM20602 ở giai đoạn 2. Xoay bo bằng
  tay, hai cột `gz` phải bằng nhau. Lệch theo một tỉ lệ cố định = sai hệ số.

Đây là lý do giai đoạn 2 tồn tại và không được bỏ qua.

### Tiêu chí đạt — kết quả đo 2026-08-26 (200 dòng, drone đứng im, đã tháo động cơ)

- [x] `WHO_AM_I` đọc ra `0x70` → console in `WHO_AM_I 0x = 112`
- [x] `count` tăng đều 47 → 7392, không lùi lần nào
- [x] `err = 0` và `ovr = 0` trên cả 200 dòng
- [x] Cột `|a|/g` = **1.0028** (dải 0.993–1.005) → **thang accel ĐÚNG**
- [x] Không mất mẫu: `dt` dao động 535–550 µs, không lần nào gấp đôi
- [~] Cột `hz` = **1846**, không phải 1920 → xem ghi chú dưới, chấp nhận được
- [x] `DBG_MODE_STATUS`: `imu_err = 0` qua 27607 mẫu, `dt = 125 µs` (đúng 8 kHz),
      `drop = 0`. `err = 0x0018` = `FLOW_TIMEOUT | RC_TIMEOUT` — **không có bit
      IMU nào**. Cả hai là dư âm của bàn thử không có tay điều khiển; `error_flags`
      chỉ cộng dồn chứ không tự xoá, nên `FLOW_TIMEOUT` còn treo từ lúc khởi động
      dù `health` cho thấy flow đang sống.
- [~] **`DBG_MODE_PID` cột `hz` = 3771–3788, không phải 4000** — thiếu 5,5%.
      Xem mục dưới.

### Vòng PID chạy 3780 Hz thay vì 4000 Hz

ICM20602 chạy **đúng** 8000 Hz (`dt = 125 µs`, và `imu_n` tăng 160 mẫu mỗi
dòng console 50 Hz = 8000/giây). Bộ chia là 2, nên đáng ra phải ra 4000.

Nghĩa là khoảng **5,5% số nhịp chẵn bị lỡ** — vòng lặp chính không kịp gọi
`ctrl_rate_update()` trong cửa sổ 250 µs.

**Hậu quả thực tế: nhỏ.** `dt` của PID lấy từ mốc thời gian IMU thật
(`ctrl_rate.c:136`) nên phép tính vẫn đúng; chỉ là băng thông vòng giảm 5,5%.

**Một điểm cần để mắt:** `s_dterm_alpha` được tính sẵn theo nhịp danh định
1/4000 và không tính lại theo `dt` thực (`ctrl_rate.c:98`). Khi lỡ một nhịp,
`dt` thành 500 µs nhưng hệ số lọc vẫn là của 250 µs, tức khâu D lọc nhẹ hơn
mức đáng ra. Sai lệch nhỏ, chưa cần sửa, nhưng ghi lại ở đây.

### Kết quả phép thử A/B — LSM6DSV gần như vô can

| | `IMU2_ENABLE = 0` | `IMU2_ENABLE = 1` | Chênh |
|---|---|---|---|
| `rhz` trung bình | **3801** | **3774** | **−27 Hz (−0,7%)** |
| `lp` trung bình | 36 µs | 40 µs | +4 µs |
| `lp` đỉnh | 215 µs | 243 µs | +28 µs |
| `health` | `0x001F` | `0x041F` | bit 10 `SENSOR_IMU2` lên ✓ |

**Kết luận: khoảng thiếu 5% đã có TRƯỚC khi thêm IMU phụ.** Số nền là 3801
chứ không phải 4000. LSM6DSV chỉ lấy thêm 0,7% — rẻ hơn nhiều so với lo ngại,
và ngân sách còn dư cho giai đoạn 3 và 5.

### Nguyên nhân thật của 3801 vs 4000

`lp` trung bình chỉ 36 µs nhưng **đỉnh 215 µs**. Với ~27 700 vòng lặp mỗi
giây, con số này chỉ hợp lý nếu có khoảng **1000 vòng/giây kéo dài ~200 µs**
xen giữa các vòng ngắn ~30 µs.

Đúng bằng nhịp của `estimator_update()` — **EKF 6×6 chạy 1 kHz**.

Cơ chế lỡ nhịp: giá trị `count` chẵn chỉ "hiện diện" trong 125 µs trước khi
bị `count+1` thay thế. Một vòng lặp bận 200 µs có thể trùm hết cửa sổ đó, và
nhịp ấy mất luôn. Ước lượng thô cho ~200–300 lần/giây, khớp với 199 lần đo
được (4000 − 3801).

**Ngưỡng cảnh báo phải là 125 µs, không phải 250 µs.** Bản đo đầu đặt 250 µs
nên `ovr` đọc ra 0 trong khi thực tế vẫn mất 5% nhịp — dụng cụ nói dối còn tệ
hơn không có. Đã sửa: `ovr` giờ đếm số vòng vượt **125 µs** (một chu kỳ mẫu
IMU) trong **1 giây vừa rồi**.

**Cách kiểm chứng giả thuyết EKF** — một dòng cấu hình, không đụng code:
hạ `EST_RATE_HZ` từ 1000 xuống 200 rồi đọc lại `rhz`. Nếu nhảy lên gần 4000
thì đã xác nhận.

**Có cần sửa không:** chưa. 3801 Hz là mất 5% băng thông, vô hại — nhiều máy
bay ở 2 kHz. Ghi lại để biết trần thực tế của kiến trúc siêu vòng lặp hiện
tại. Nếu sau này cần đủ 4000 Hz thì hướng đúng là đưa vòng PID tốc độ góc vào
thẳng ISR của IMU, nhưng đó là thay đổi kiến trúc trong đường bay quan trọng,
không làm chen ngang giữa chừng công việc IMU phụ.

Nền nhiễu gyro đo được (sau hiệu chuẩn, đứng im):

```
gx  sigma 0.092 dps      gy  sigma 0.083 dps      gz  sigma 0.079 dps
```

Bước lượng tử quan sát được đúng bằng **0,07 °/s** — khớp với `70 mdps/LSB`
của dải ±2000 °/s, xác nhận hệ số đang được áp dụng đúng như khai báo. (Việc
hằng số 70 có đúng với chip hay không thì giai đoạn 2B mới kết luận được.)

### Ghi chú: ODR thực tế 1846 Hz thay vì 1920 Hz

Lệch **−3,9%**. Đã loại trừ khả năng mất mẫu:

- `1/dt` = 1846 Hz, khớp chính xác với bộ đếm `hz`
- `dt` không lần nào vượt 800 µs — nếu bỏ mẫu thì `dt` phải nhảy lên ~1084 µs
- `hz` chỉ nhận hai giá trị: 1801 (đúng một cửa sổ đầu, lúc chưa ổn định)
  và 1846 (toàn bộ phần còn lại)

Kết luận: **dao động nội của chip chạy chậm ~4%.** Không phải lỗi phần mềm.

Không cần sửa, vì:
- Mọi mốc thời gian lấy từ `micros()` (TIM2, từ thạch anh), không lấy từ ODR
  danh định — nên bộ ước lượng và phần hợp nhất sau này đều không bị ảnh hưởng
- Hệ số lọc tính theo 1920 nên tần số cắt thực tế là ~96 Hz thay vì 100 Hz.
  Không đáng kể.

### ⚠️ Phát hiện sẵn cho giai đoạn 2A: DẤU TRỤC Z SAI

Đứng yên trên mặt phẳng, đo được:

```
ax = +0.025    ay = +0.128    az = +9.834  m/s²
```

`az` phải là **−9.81** theo quy ước thân NED (trục Z hướng xuống, gia tốc kế
đo lực riêng hướng lên). Vậy `IMU2_AXIS_SIGN_Z` hiện tại `(+1)` là sai.

**Không được chỉ đảo mỗi Z** — đảo lẻ một dấu sẽ cho định thức −1, tức hệ trục
tay trái, và gyro sẽ quay ngược. Phải làm đủ quy trình 2A để xác định cả ba.

---

## Giai đoạn 2 — Căn trục và đo nhiễu ⭐ CỔNG QUYẾT ĐỊNH

**Mục tiêu:** trả lời bằng số liệu câu hỏi *"hợp nhất hai IMU có đáng không"*.

### 2A. Căn trục — dùng `DBG_MODE_AXISCAL`

LSM6DSV nằm trên module rời, hướng lắp gần như chắc chắn khác ICM20602.

Việc này từng phải làm thủ công rồi tự suy ra dấu — dễ sai và khó kiểm. Giờ
có `DBG_MODE_AXISCAL` tự làm hết. Console đã đặt sẵn mode này.

**Cách dùng:** đặt máy bay lần lượt vào ba tư thế mà cột `huong dan` yêu cầu,
giữ yên mỗi tư thế khoảng một giây. Cột `giu` chạy từ 0% lên 100% rồi tự
chuyển bước. Xong ba bước, console in thẳng sáu dòng cần dán vào `fc_config.h`.

| Bước | Tư thế | Thân đọc ra |
|---|---|---|
| 1 | Nằm phẳng, mặt trên hướng **lên** | Z = **−1 g** |
| 2 | Dựng đứng trên đuôi, mũi hướng **lên** | X = **+1 g** |
| 3 | Nằm nghiêng, cánh **phải** chạm đất | Y = **−1 g** |

Dấu **âm** ở bước 1 là đúng: hệ thân là NED, trục Z hướng **xuống**, mà gia
tốc kế đo **lực riêng** (phản lực mặt bàn) hướng **lên**.

Nếu chưa bắt được, cột cuối nói rõ vì sao: `[DANG RUNG]`, `[|a| KHONG BANG 1g]`,
hay `[CHUA DUNG HAN MOT TRUC]`.

**Công thức suy ra:** gọi `k` là trục CẢM BIẾN chiếm ưu thế và `s = ±1` là dấu
của nó. Cần `SIGN × sensor[k]` = giá trị thân mong đợi, nên `MAP = k` và
`SIGN = (giá trị thân mong đợi) × s`.

> **Bộ ba dấu phải là phép quay hợp lệ — định thức bằng +1.** Đảo lẻ một dấu
> sẽ biến hệ trục thành tay trái và làm sai chiều quay gyro. Đảo 0 hoặc 2 dấu
> thì hợp lệ; đảo 1 hoặc 3 dấu thì không. Công cụ tự kiểm và **từ chối** kết
> quả có định thức khác +1.

Thuật toán đã được kiểm bằng mô phỏng qua **cả 24 phép quay hợp lệ**, và đối
chiếu ngược với cấu hình đã biết đúng của ICM20602 (`MAP 0,1,2` / `SIGN +1,−1,−1`)
— cho ra đúng bộ đó.

#### ✅ Kết quả 2A — XONG, đã áp vào `fc_config.h`

| Bước | Tư thế | Số thô lúc chốt | Trục | Dấu | Suy ra |
|---|---|---|---|---|---|
| 1 | Nằm phẳng, mặt trên lên | `(−1, 22, 2055)` | Z | **+** | `MAP_Z = 2`, `SIGN_Z = (−1)` |
| 2 | Dựng trên đuôi, mũi lên | `(27, −1993, 453)` | Y | **−** | `MAP_X = 1`, `SIGN_X = (−1)` |
| 3 | Nghiêng, cánh phải xuống | `(1897, 259, 748)` | X | **+** | `MAP_Y = 0`, `SIGN_Y = (−1)` |

```c
#define IMU2_AXIS_MAP_X 1
#define IMU2_AXIS_MAP_Y 0
#define IMU2_AXIS_MAP_Z 2
#define IMU2_AXIS_SIGN_X (-1)
#define IMU2_AXIS_SIGN_Y (-1)
#define IMU2_AXIS_SIGN_Z (-1)
```

**Định thức = +1**, phép quay hợp lệ. Ba tư thế rơi vào ba trục cảm biến khác
nhau đúng như yêu cầu.

Hướng lắp thực tế của module:

```
thân X (mũi trước)   = -cảm biến Y     -> cảm biến Y chỉ về phía SAU
thân Y (cánh phải)   = -cảm biến X     -> cảm biến X chỉ sang TRÁI
thân Z (hướng xuống) = -cảm biến Z     -> cảm biến Z chỉ LÊN
```

Khác hẳn ICM20602 (`MAP 0,1,2` / `SIGN +1,−1,−1`) — module này vừa xoay vừa
lật, nên X và Y **hoán vị** cho nhau chứ không chỉ đổi dấu. Nếu làm thủ công
theo cách "xem dấu rồi đảo" như dự định ban đầu thì gần như chắc chắn sẽ bỏ
sót phần hoán vị này và ra một hệ trục sai.

#### Kiểm lại bằng phép đo tay ở đúng 90° — cùng kết quả

Lần chạy `AXISCAL` có bước 3 hơi cẩu thả (độ chiếm ưu thế chỉ 0,92, lệch ~23°),
nên đã đo lại thủ công 20 mẫu mỗi tư thế đặt sát 90°:

| Tư thế | Số thô trung bình | Ưu thế | Trục |
|---|---|---|---|
| Nằm phẳng | `(6, 33, 2056)` | 1,000 | Z, + |
| Dựng trên đuôi | `(73, −2032, 67)` | 0,999 | Y, − |
| Nghiêng cánh phải | `(2039, 97, 70)` | 0,998 | X, + |

**Cho ra đúng cùng một bộ `MAP (1,0,2)` / `SIGN (−1,−1,−1)`** — kết quả cũ vẫn
đúng dù số đo lệch, vì trục chiếm ưu thế không hề mơ hồ (thành phần lớn thứ hai
chỉ 748 so với 1897). Không phải sửa gì trong cấu hình.

Ba thông tin phụ rút ra từ bộ số sạch này:

- `|a|` trung bình **2044 count** so với kỳ vọng 2049 (16 g, 0,488 mg/LSB)
  → lệch **0,24%**. Xác nhận lần thứ ba hệ số thang accel đúng.
- `|a|` chênh nhau **1,05%** giữa ba tư thế → độ nhạy chéo trục, mức bình thường.
- Tư thế nằm phẳng (đặt trên bàn, đáng tin nhất) có thành phần phụ chỉ **0,16°
  và 0,91°** → module lắp gần như song song với mặt bo. Hai tư thế kia cầm tay
  nên 2–3° gần như chắc chắn là sai số tay chứ không phải lệch cơ khí.

#### ⚠️ Lỗi của chính công cụ — đã sửa

Lần chạy đầu ra kết quả `MAP_X = MAP_Y = MAP_Z = 2`, và **công cụ tự từ chối**
vì định thức khác +1. Cổng kiểm định thức đã làm đúng việc của nó.

Nguyên nhân: log cho thấy ba bước chuyển ở dòng 1 → 41 → 82 → 123, **cách nhau
đúng 40 tick** = `AXCAL_HOLD_TICKS`. Máy bay nằm yên, nên sau khi chốt bước 1
thì bước 2 lập tức tích luỹ trên **cùng số đọc không đổi** và tự chốt 0,8 giây
sau, trước khi người dùng kịp xoay.

Đã thêm **hai cổng chặn**:

1. **Phải thấy máy bay đã bị nhấc lên** (số đọc mất ổn định) thì mới bắt đầu đo
   bước kế tiếp. Console báo `[DA BAT BUOC TRUOC - HAY XOAY MAY BAY]`.
2. **Trục cảm biến đã dùng ở bước trước thì không nhận lại.** Ba tư thế bắt buộc
   rơi vào ba trục khác nhau — đó là hệ quả trực tiếp của việc phép xoay là một
   hoán vị. Console báo `[TRUC NAY DA DUNG O BUOC TRUOC - XOAY TIEP]`.

Máy trạng thái sau khi sửa đã mô phỏng lại qua ba kịch bản: máy nằm yên suốt
(kẹt ở bước 1, đúng), lật ngược máy (từ chối vì trùng trục, đúng), và ba tư thế
làm đúng (ra đủ ba trục, định thức +1, khớp cấu hình gốc).

### 2B. Kiểm thang gyro — ✅ XONG, hệ số ĐÚNG

Phép kiểm dứt điểm không phải nhìn hai cột bằng mắt, mà là **tích phân tốc độ
góc qua từng đoạn xoay** rồi so tỉ số. Tỉ số không phụ thuộc vào việc console
lấy mẫu thưa hay người dùng xoay không đúng 90°, vì cả hai IMU đều được đọc
tại **cùng thời điểm**.

Log 1761 dòng, ba lần xoay (mỗi lần có cả chiều đi và chiều về):

| Đoạn | Trục | ICM20602 | LSM6DSV | Tỉ số |
|---|---|---|---|---|
| 1 | Y (pitch) | +63,52° | +63,79° | 1,0043 |
| 3 | Y | −79,43° | −79,90° | 1,0060 |
| 4 | X (roll) | +85,50° | +85,79° | 1,0034 |
| 5 | X | −89,49° | −89,90° | 1,0045 |
| 6 | Z (yaw) | +90,51° | +90,19° | 0,9965 |
| 7 | Z | −93,82° | −93,12° | 0,9926 |

**Trung bình 1,0020 — lệch 0,20%.** Hệ số suy ra là 69,86 mdps/LSB so với
70,0 đang dùng. Sai lệch nằm gọn trong sai số phép đo (xoay tay, lấy mẫu 50 Hz,
dung sai thang của chính hai chip).

→ **`LSM_GYRO_MDPS_PER_LSB = 70.0` là ĐÚNG.** Đây là hằng số cuối cùng còn
chưa xác minh; cả thang accel lẫn thang gyro giờ đều đã kiểm chứng.

**2A cũng được xác nhận độc lập trong miền gyro:** xoay roll ra trục X chiếm
ưu thế, pitch ra Y, yaw ra Z, và **dấu của trục chính khớp nhau ở cả 7 đoạn**.
Trước đó 2A chỉ dựa trên gia tốc kế; giờ có thêm bằng chứng từ con quay.

Bias còn lại lúc đứng yên (596 dòng): ICM `+0,025 / +0,061 / −0,095` dps,
LSM `+0,008 / +0,014 / −0,042` dps. Cả hai đều rất nhỏ.

### ⚠️ Phát hiện mới: hai IMU LỆCH NHAU 1,34°

Các thành phần phụ của hai IMU **ngược dấu nhau một cách có hệ thống**. Đo góc
giữa hai vector xoay (nếu thẳng hàng hoàn hảo thì góc này phải bằng 0):

| Trục xoay | Lần 1 | Lần 2 |
|---|---|---|
| Y | 0,80° | 0,68° |
| X | 1,47° | 1,50° |
| Z | 1,84° | 1,74° |

**Lặp lại rất sát trong từng cặp** — đây là lệch lắp đặt thật, không phải nhiễu.
Trung bình **1,34°**.

**Hệ quả cho giai đoạn 5 — đây là lý lẽ mạnh CHỐNG lại việc lấy trung bình:**

| Tốc độ xoay | Sai số chéo trục bơm vào | Nhiễu trắng giảm được |
|---|---|---|
| 100 °/s | ~1,2 °/s | ~0,03 °/s |
| 300 °/s | ~3,5 °/s | ~0,03 °/s |

Lấy trung bình hai gyro lệch nhau 1,34° sẽ **bơm vào nhiều sai số hơn hàng
trăm lần** so với lượng nhiễu trắng nó loại bỏ được, mỗi khi máy bay xoay nhanh.

Muốn hợp nhất thì **bắt buộc phải bù lệch trước** bằng một ma trận xoay nhỏ —
thêm một tầng phức tạp nữa. Cộng với việc giai đoạn 2C nhiều khả năng cho thấy
rung khung mới là nguồn nhiễu chính, cán cân đang nghiêng hẳn về phía **bỏ
giai đoạn 5** và giữ IMU phụ cho đúng hai việc: **từ kế** và **phát hiện hỏng**.

### 2C. Đo nền nhiễu — ✅ XONG. 🚦 **CỔNG ĐÃ ĐÓNG: BỎ GIAI ĐOẠN 5**

Log 2880 dòng: 24 giây đứng yên, rồi ~30 giây chạy động cơ (đã tháo cánh quạt)
với nhiều mức ga khác nhau.

| | Đứng yên | Động cơ chạy (TB) | Đỉnh | Tăng |
|---|---|---|---|---|
| **ICM20602** | 0,09 | 15,74 | **45,58** | **175×** |
| **LSM6DSV** | 1,79 | 7,62 | 21,03 | 4× |

**Cả hai IMU đều tăng vọt.** ICM tăng 175 lần. Đây chính là chữ ký của **rung
khung** — chuyển động vật lý THẬT mà cả hai chip cùng chịu, không phải nhiễu
điện tử độc lập.

Gia tốc kế cũng bị: `|a|` của ICM có σ nhảy từ ~0 lên **0,045 g**. Mức đó đủ
làm hỏng ước lượng góc của EKF, không riêng gì vòng PID.

Tỉ số `sd_2/sd_1` ≈ 0,45 **không** có nghĩa LSM sạch hơn — hai chip lấy mẫu ở
hai tốc độ khác nhau (8000 vs 1846 Hz) nên rung tần số cao gấp xuống (aliasing)
theo cách khác nhau, và bộ lọc nội của chúng cũng khác.

#### Kết luận cổng quyết định

> **Giai đoạn 5 (hợp nhất / lấy trung bình hai IMU) ĐÓNG LẠI.**

Ba lý lẽ độc lập cùng chỉ về một hướng:

1. Nhiễu bị **rung khung** chi phối — cả hai chip đo ra cùng một chuyển động
   thật, lấy trung bình hai số giống nhau vẫn ra chính số đó.
2. Hai IMU **lệch nhau 1,34°** (đo ở 2B) — lấy trung bình sẽ bơm vào 3,5 °/s
   sai số chéo trục khi xoay 300 °/s, trong khi chỉ giảm được ~0,03 °/s nhiễu
   trắng. Lỗ vốn hơn trăm lần.
3. Nền nhiễu của ICM lúc đứng yên đã là **0,09 °/s** — không còn gì đáng để
   cải thiện ở đó.

**IMU phụ giữ lại đúng hai việc nó thật sự làm tốt: từ kế QMC6309 (giai đoạn
3–4 → mở khoá giữ hướng) và phát hiện hỏng hóc.**

### 🔴 PHÁT HIỆN QUAN TRỌNG HƠN CẢ MỤC TIÊU BAN ĐẦU: KHUNG BỊ CỘNG HƯỞNG

Người dùng mô tả: *"đẩy ga có một mức drone rung rất mạnh, nhưng đẩy cao hơn
mức đó lại bớt rung"*. Số liệu xác nhận chính xác điều đó — `sd_1` có các đỉnh
nhọn rồi tụt xuống:

| Thời điểm | Kéo dài | Đỉnh `sd_1` | Đỉnh `sd_2` |
|---|---|---|---|
| 30,7 s | 2,0 s | **39,3** °/s | 17,6 |
| 36,7 s | 1,0 s | **26,4** °/s | 13,2 |
| 50,7 s | 4,0 s | **45,6** °/s | 21,0 |

Giữa các đỉnh, `sd_1` tụt về 5–9 °/s. Rung **không** tăng đều theo ga — nó
vọt lên ở vài dải vòng tua nhất định rồi giảm. Đó là định nghĩa của **cộng
hưởng**: tần số động cơ quét qua tần số riêng của khung hoặc của đế bắt bo.

**Vì sao đây là vấn đề nghiêm trọng nhất hiện nay:**

- 45 °/s nhiễu gyro đi thẳng vào khâu P: `0,001 × 45 = 4,5%` dải ga dao động
  liên tục trên cả bốn motor.
- Khâu D khuếch đại nhiễu tần số cao còn mạnh hơn nữa.
- Motor nóng, ESC nóng, pin tụt nhanh, và ở đúng dải ga cộng hưởng thì máy
  bay có thể mất kiểm soát.
- `|a|` nhiễu 0,045 g làm EKF ước lượng góc sai theo.

**Phải xử lý TRƯỚC khi bay** — trước cả althold, trước cả từ kế. Thứ tự:

1. **Đế mềm cho bo bay** — đệm gel hoặc silicon giữa bo và khung. Đây là biện
   pháp hiệu quả nhất và rẻ nhất.
2. **Cân cánh quạt**, kiểm tra cánh nứt/cong, kiểm tra motor có rơ không.
3. **Siết lại toàn bộ ốc** — ốc lỏng là nguồn cộng hưởng kinh điển.
4. Sau khi cơ khí đã tốt mà vẫn còn: thêm **bộ lọc chặn dải (notch)** ở tần số
   cộng hưởng. Muốn làm đúng thì cần biết tần số — hoặc lấy từ eRPM qua DShot
   hai chiều (notch động), hoặc đo bằng FFT từ log tốc độ cao.

**Đo lại sau khi sửa:** chạy đúng bài này rồi so `sd_1`. Mục tiêu đưa đỉnh
xuống dưới ~10 °/s.

### ⚠️ Bất thường chưa lý giải: gyro LSM6DSV ồn khi đứng yên

Lúc đứng yên hoàn toàn (750 mẫu):

| Trục thân | ICM20602 | LSM6DSV |
|---|---|---|
| gx | TB +0,003 · σ **0,079** | TB **−4,570** · σ **1,847** · biên độ 8,19 |
| gy | TB +0,041 · σ 0,087 | TB −0,219 · σ 0,311 |
| gz | TB +0,053 · σ 0,080 | TB **+3,369** · σ 0,742 |

Hai vấn đề:

1. **Bias không được trừ** (−4,57 và +3,37 °/s). Nguyên nhân: biên độ dao động
   8,19 °/s vượt xa `IMU2_CALIB_MOVE_LIMIT_DPS = 2,0`, nên quá trình hiệu chuẩn
   **huỷ và làm lại vô hạn trong im lặng**. Đã thêm bộ đếm
   `lsm6dsv_calib_restarts()`; console `IMU_CMP` giờ báo rõ thay vì để lỗi này
   diễn ra thầm lặng.
2. **Nhiễu cao gấp ~20 lần ICM**, và cao hơn hẳn chính nó ở giai đoạn 1
   (σ 0,092/0,083/0,079 khi đó).

Điều đáng chú ý: **gia tốc kế của LSM vẫn sạch** (σ `|a|` = 0,0017 g lúc đứng
yên). Nếu là nhiễu nguồn hay nhiễu điện chung thì cả hai khối phải cùng bị.

Chưa đủ dữ kiện để kết luận. **Phép thử tách bạch:** chạy lại đúng bài đo đứng
yên trong hai điều kiện — (a) chỉ cấp nguồn USB, **rút pin**; (b) có cắm pin.
Nếu (a) sạch còn (b) ồn thì thủ phạm là nhiễu từ ESC/mạch nguồn coupling vào
module qua dây nối, và cách xử lý là dời module ra xa dây nguồn, xoắn đôi dây
tín hiệu, hoặc thêm tụ lọc ngay tại module.

Việc này **không chặn** giai đoạn 3–4: từ kế đọc qua sensor hub, không phụ
thuộc chất lượng gyro của LSM.

### Tiêu chí đạt

- [ ] Ba trục của hai IMU khớp nhau về dấu và độ lớn khi xoay tay
- [ ] `IMU2_AXIS_*` đã chốt và ghi vào `fc_config.h`
- [ ] Đã ghi lại 4 con số: `sd_1`/`sd_2` lúc yên và lúc động cơ chạy
- [ ] Đã ra quyết định về giai đoạn 5

---

## Giai đoạn 3 — Sensor hub + QMC6309

**Mục tiêu:** đọc được từ trường.

### Cách hoạt động

LSM6DSV làm I2C master trên chân SDX/SCX, tự đọc QMC6309 theo nhịp, rồi để
kết quả trong dãy `SENSOR_HUB_1..6` để host lấy qua SPI.

Các thanh ghi sensor hub nằm ở **bank riêng**, phải bật bit `shub_reg_access`
(bit 6 của `FUNC_CFG_ACCESS = 0x01`) mới truy cập được, và **phải tắt lại**
sau khi xong nếu không các thanh ghi thường sẽ đọc ra rác.

```
FUNC_CFG_ACCESS = 0x01   bit6 = shub_reg_access
Bank sensor hub:
  SENSOR_HUB_1..6 = 0x02..0x07   (dữ liệu mag đọc về)
  MASTER_CONFIG   = 0x14
  SLV0_ADD        = 0x15
  SLV0_SUBADD     = 0x16
  SLV0_CONFIG     = 0x17
  DATAWRITE_SLV0  = 0x21
  STATUS_MASTER   = 0x22
```

### Trình tự khởi tạo

1. Mở bank sensor hub
2. Dùng kênh ghi-một-lần nạp cấu hình vào QMC6309 (chế độ liên tục, ODR, dải đo)
3. Cấu hình SLV0: địa chỉ QMC6309, thanh ghi dữ liệu đầu, số byte
4. `MASTER_CONFIG`: bật master, chọn nguồn kích là DRDY của accel
5. Đóng bank, quay về đọc `SENSOR_HUB_1..6` như thanh ghi thường

### Bản đồ thanh ghi QMC6309 — lấy từ datasheet, KHÔNG từ thư viện

Nguồn: *QMC6309 Preliminary Datasheet Rev A*, QST, Document #13-52-22, mục 9.

```
0x00  CHIP_ID   R    = 0x90
0x01..0x06     R    XOUT_L, XOUT_H, YOUT_L, YOUT_H, ZOUT_L, ZOUT_H
                    16 bit bù hai, BYTE THẤP Ở ĐỊA CHỈ THẤP
0x09  STATUS    R    bit0 DRDY, bit1 OVFL, bit2 ST_RDY, bit3 NVM_RDY, bit4 NVM_LOAD
0x0A  CTRL1     R/W  bit7..5 OSR2 | bit4..3 OSR1 | bit2 - | bit1..0 MODE
0x0B  CTRL2     R/W  bit7 SOFT_RST | bit6..4 ODR | bit3..2 RNG | bit1..0 SET/RESET
0x0E  CTRL3     R/W  bit7 SELFTEST

MODE: 00 Suspend · 01 Normal · 10 Single · 11 Continuous
OSR1: 00→8 · 01→4 · 10→2 · 11→1        OSR2: 000→1 · 001→2 · 010→4 · 011→8 · 100→16
ODR:  000→1Hz · 001→10 · 010→50 · 011→100 · 100→200
RNG:  00→32G (1000 LSB/G) · 01→16G (2000) · 10→8G (4000)
Địa chỉ I2C: 0x7C, dạng 7 bit (datasheet mục 5.4)
```

#### ⚠️ Thư viện phổ biến trên GitHub đặt SAI vị trí bit

`SensorLib` (lewisxhe) — thư viện QMC6309 hay được dùng nhất — để ODR trong
thanh ghi `0x0A` và dùng ba mặt nạ **chồng lấn nhau** trong cùng một thanh ghi
8 bit: OSR `0x18` (bit 3–4), ODR `0x70` (bit 4–6), LPF `0xE0` (bit 5–7). Ba
trường này không thể cùng đúng.

Datasheet mới là đúng, và nó **tự kiểm chứng** bằng chính hai ví dụ trong đó:

| Ví dụ trong datasheet | Giải mã theo bảng thanh ghi |
|---|---|
| §7.2 `0x0A = 0x63` "Continuous, OSR1=8, OSR2=8" | MODE=11 ✓ OSR1=00→8 ✓ OSR2=011→8 ✓ |
| §7.1 `0x0B = 0x40` "ODR=200Hz, 32 Gauss" | ODR=100→200Hz ✓ RNG=00→32G ✓ |

Cả hai khớp tuyệt đối. Nếu viết theo thư viện thì từ kế sẽ chạy sai chế độ mà
vẫn trả về số trông có vẻ hợp lý — kiểu lỗi mất rất nhiều ngày để lần ra.

### Cấu hình đã chọn

| Tham số | Giá trị | Lý do |
|---|---|---|
| Chế độ | **Normal** | nhịp xác định, đúng ví dụ §7.1 của datasheet |
| ODR | **200 Hz** | cao hơn nhịp đọc 50 Hz nên luôn được mẫu mới |
| Dải đo | **±8 G** | 4000 LSB/G, Trái Đất (~0,5 G) cho ~2000 count |
| OSR1 / OSR2 | **8 / 8** | 2,5 mGauss nhiễu, so với 7,0 ở mức 8/1 |
| SET/RESET | **bật cả hai** | offset làm mới mỗi lần đo, không trôi |

Cột `Btot` trong console in cờ tràn dải — nếu bật khi tăng ga thì nới
`MAG_RANGE_G` lên 16 hoặc 32.

### Kiến trúc: tranh chấp bus SPI3

Đây là chỗ khó nhất của giai đoạn này. Sensor hub và đường dữ liệu IMU **dùng
chung một ngoại vi SPI3**:

- IMU chạy DMA, được kích bởi ngắt DRDY ở 1920 Hz
- Từ kế đọc kiểu hỏi vòng từ vòng lặp chính ở 50 Hz

Chen vào nhau là hỏng cả hai. Cách xử lý (`bus_acquire`/`bus_release`):

1. Tạm **khoá ngắt DRDY**
2. Đợi nốt transfer DMA đang chạy — 15 byte ở 8 MHz mất ~15 µs, trần 200 µs
3. Làm giao dịch của mình (~40 µs)
4. **Mở lại ngắt**, KHÔNG xoá cờ EXTI đang treo — một sườn xảy ra trong lúc
   khoá vẫn là mẫu hợp lệ, để nó chạy thì đỡ mất một mẫu

Mất một hai mẫu IMU mỗi 20 ms là ~0,1% số mẫu. Và nhờ `DRDY_PULSED` (bài học
từ giai đoạn 1) nên khoá/mở ngắt không gây kẹt cứng như chế độ chốt mức.

Cột `busy` trong console đếm số lần không giành được bus — vài lần là bình
thường, tăng liên tục thì có vấn đề.

### Trình tự khởi tạo

`lsm6dsv_mag_init()` chạy **sau `lsm6dsv_init()` và trước `lsm6dsv_start()`**,
lúc chưa có ngắt DRDY nào nên không cần tranh chấp bus.

```
1. RST_MASTER_REGS  -> xoá cấu hình sensor hub còn sót từ lần nạp trước
2. Đọc CHIP_ID qua kênh đọc-một-lần        -> phải ra 0x90
3. Ghi CTRL2 = 0x80 rồi 0x00               -> soft reset (không tự xoá)
4. Ghi CTRL2 = ODR|RNG|SETRESET
5. Ghi CTRL1 = OSR2|OSR1|MODE              -> đặt chế độ chạy SAU CÙNG
6. Cấu hình SLV0 đọc liên tục 6 byte từ 0x01, bật master
```

Mỗi lệnh ghi dùng cờ `write_once` — không có nó thì mỗi nhịp accel lại ghi đè
thanh ghi từ kế. Bộ máy sensor hub **lấy nhịp từ DRDY của accel**, nên accel
phải đang chạy thì lệnh mới thực thi; bước 4–5 của `lsm6dsv_init()` đã bật ODR
trước rồi.

Cờ `slave0_nack` trong `STATUS_MASTER` cho biết từ kế có trả lời hay không —
đếm ra cột `nack`, phải đứng yên ở 0.

### ⚠️ Bẫy đã gặp: bus I2C phụ thiếu điện trở kéo lên

**Triệu chứng:** `lsm6dsv_init()` thành công, `WHO_AM_I = 0x70`, nhưng chip ID
của QMC6309 đọc ra `0x00` và mọi giao dịch sensor hub đều "thành công".

**Nguyên nhân:** hai chân SDX/SCX nối tới QMC6309 là một **bus I2C thật**, và
I2C cần điện trở kéo lên mới hoạt động. Module tích hợp thường **không** có
điện trở ngoài cho đường này vì nó nằm hoàn toàn bên trong con LSM6DSV. Không
bật kéo lên nội thì bus chết câm — không có gì kéo đường lên mức cao nên mọi
byte đọc về đều là `0x00`.

**Khắc phục:** bật `shub_pu_en` — **bit 6 của `IF_CFG` (0x03)**. Không phải
trong `MASTER_CONFIG` như ở đời LSM6DSO; ST đã dời nó ở LSM6DSV.

Ghi bằng đọc-sửa-ghi để không đụng các bit khác của `IF_CFG`.

### ⚠️ Bẫy thứ hai: accel phải TẮT rồi BẬT LẠI mới kích được hub

**Triệu chứng:** đã bật pull-up, nhưng `STATUS_MASTER` đọc mãi ra `0x00` —
không có `ENDOP`, không có `NACK`, không có gì cả. Hub nằm im hoàn toàn.

**Nguyên nhân:** bộ máy sensor hub chỉ khởi động ở **sườn DRDY đầu tiên của
accel SAU KHI master được bật**. Accel đã chạy sẵn từ `lsm6dsv_init()` nên
không có sườn "đầu tiên" nào, và hub không bao giờ bắt đầu.

Mọi ví dụ chính thức của ST đều theo đúng thứ tự này, và giờ mới hiểu vì sao:

```
1. Cấu hình SLV0                (trong bank shub)
2. TẮT accel        CTRL1 = 0x00
3. Bật master       MASTER_CONFIG.master_on = 1
4. BẬT LẠI accel    CTRL1 = ODR      <- sườn này mới là thứ kích hub
5. Đợi cờ ENDOP / WR_ONCE
6. Tắt master
```

**Bài học rút ra:** tôi đã lấy đúng *bản đồ thanh ghi* từ nguồn chính thức
nhưng lại tự nghĩ ra *trình tự thao tác*. Bản đồ thanh ghi và trình tự là hai
thứ khác nhau — cái thứ hai không suy ra được từ cái thứ nhất, và cũng phải
lấy từ ví dụ của hãng.

`xl_on()` được gọi trên MỌI nhánh thoát, kể cả nhánh lỗi, nên IMU không bao
giờ bị bỏ lại ở trạng thái tắt.

### Địa chỉ I2C: dò cả hai ứng viên

Datasheet mục 5.4 ghi *"7-bit serial address ... default value is 7CH"*, nhưng
rất nhiều datasheet của hãng khác lại ghi dạng **8 bit** ở chính chỗ đó. Nếu
0x7C là dạng 8 bit thì địa chỉ 7 bit phải là **0x3E**.

Driver dò lần lượt cả hai và báo địa chỉ nào trả lời đúng `0x90`, thay vì để
một chữ số mơ hồ làm mất thêm một vòng gỡ lỗi. Console in địa chỉ tìm được.

### 🔬 NĂM nguyên nhân phải gỡ lần lượt — gỡ trực tiếp trên phần cứng

Giai đoạn này mất năm vòng, mỗi vòng một nguyên nhân KHÁC NHAU nhưng triệu
chứng bên ngoài **giống hệt nhau**. Đoán từ triệu chứng là vô vọng; chỉ khi
nạp một bản đổ thanh ghi và đọc thẳng từ chip qua UART mới lần ra được.

| # | Triệu chứng | Nguyên nhân thật |
|---|---|---|
| 1 | `chip_id = 0x00` | Thiếu **pull-up bus I2C phụ** — `shub_pu_en`, bit 6 của `IF_CFG` (0x03). Ở đời LSM6DSO bit này nằm trong `MASTER_CONFIG`; ST đã dời sang `IF_CFG` |
| 2 | `STATUS_MASTER = 0x00` mãi | Accel phải **TẮT rồi BẬT LẠI** mới sinh sườn kích hub. Accel đang chạy sẵn thì không có sườn "đầu tiên" nào |
| 3 | Vẫn `0x00`, chập chờn | **Hỏi vòng phá chính thứ mình đang chờ.** Đọc `STATUS_MASTER` đòi mở bank shub, và bật/tắt `FUNC_CFG_ACCESS` liên tục cắt ngang bộ máy I2C. Hỏi 1 ms → không bao giờ xong; 30 ms → xong sau ~90 ms. Phải **đợi một khoảng cố định, không đụng bank**, rồi mở ra đọc đúng một lần |
| 4 | Đọc chạy, **ghi** hỏng | Hai lỗi cùng lúc: chờ nhầm cờ `WR_ONCE` (ST chờ `ENDOP` cho **cả hai** chiều), và `SLV0_CONFIG = 0x00` ở đường ghi vô tình đặt `shub_odr = 0` — nhịp chậm nhất, ~600 ms một chu kỳ |
| 5 | Ghi xong thì **đọc chết hẳn** | Cơ chế `write_once` để lại bộ máy KẸT. Phải xung **`rst_master_regs`** (bit 7 `MASTER_CONFIG`) mới gỡ ra. Không có nó thì mọi lệnh đọc sau đều hỏng — dù vừa đọc được chip ID ngay trước đó |

Bằng chứng dứt điểm cho #5: sau khi thêm xung reset, đọc ngược `CTRL1`/`CTRL2`
của QMC6309 ra đúng `0x61` và `0x40` — khớp chằn chặn giá trị đã ghi.

### Không dùng chế độ đọc liên tục

Cấu hình SLV0 đọc 6 byte rồi để master bật liên tục **không quay vòng**: mọi
thanh ghi đọc lại đều đúng (`MASTER_CONFIG=0x04`, `SLV0_ADD=0xF9`,
`SLV0_SUBADD=0x01`, `SLV0_CONFIG=0x86`) nhưng `STATUS_MASTER` đứng ở `0x00`
và dãy `SENSOR_HUB` giữ nguyên giá trị cũ mãi.

Đường đọc **một lượt** thì tin cậy tuyệt đối. Nên `lsm6dsv_mag_update()` dùng
đúng đường đó, chia làm **ba pha** để không chặn vòng lặp chính:

```
pha 1  nạp lệnh đọc vào SLV0, bật master, tắt accel     (~40 µs)
pha 2  bật lại accel  -> sinh sườn kích hub             (~10 µs)
pha 3  đọc STATUS_MASTER + 6 byte, xung rst_master_regs (~50 µs)
```

Thời gian chờ nằm **giữa** các lần gọi từ vòng lặp chính chứ không phải trong
một hàm chặn. Ba pha × `MAG_PHASE_US` cho ra nhịp cuối cùng.

### ✅ Kết quả đo trên phần cứng

```
QMC6309: OK qua sensor hub, chip id 0x = 144        (0x90)

     mx     my     mz |    Btot |  raw_x  raw_y  raw_z |    hz |     count |  err  nack  busy
  0.389  0.228 -0.035 |   0.452 |    389    228    -35 |    10 |       124 |    0     0     0
  0.386  0.230 -0.035 |   0.451 |    386    230    -35 |    10 |       125 |    0     0     0
```

- **`Btot` = 0,45 G** — đúng dải từ trường Trái Đất (0,25–0,65 G)
- Số liệu đổi liên tục với nhiễu tự nhiên, `count` tăng đều
- `err` / `nack` / `busy` đều **0**

Vòng bay không bị ảnh hưởng:

```
che do       |  health     err  armblk |     imu_n  dt_us   ierr |   loop   lmax   slow |    rhz   rsk
DISARMED     |  0x051F  0x0018  0x00C1 |    106351    125      0 |     11    311   1002 |   3756     0
```

`health = 0x051F` có thêm **bit 8 `SENSOR_MAG`** và bit 10 `SENSOR_IMU2`.
`rhz = 3756` so với nền 3774 — **từ kế tốn 0,5%**.

### Chẩn đoán khi init từ kế thất bại

Lần chạy đầu chỉ hiện `chip_id 0x00`, không đủ để biết hỏng ở tầng nào. Đã
thêm `lsm6dsv_mag_init_result()` tách bạch bảy trường hợp:

| Kết quả | Nghĩa là |
|---|---|
| `khong bat duoc pull-up bus phu` | ghi `IF_CFG` thất bại — lỗi SPI3, không phải lỗi từ kế |
| `sensor hub khong bao xong` | không có `ENDOP`/`WR_ONCE`. Bộ máy hub lấy nhịp từ **DRDY của accel** — accel phải đang chạy |
| `khong ai tra loi o dia chi 0x7C` | cờ `slave0_nack` bật — bus sống nhưng không có ai ở địa chỉ đó |
| `doc duoc nhung chip ID khac 0x90` | bus và địa chỉ đúng, nhưng không phải QMC6309 |
| `ghi thanh ghi cau hinh that bai` | đọc được ID nhưng ghi không ăn |
| `bat che do doc lien tuc that bai` | mọi thứ ổn tới bước cuối |

Console `DBG_MODE_MAG` in thẳng lý do cùng giá trị `STATUS_MASTER` thô, nên
lần gỡ lỗi sau không phải đoán.

### Một lỗi hiển thị kéo theo lệch cột

`wr_hex()` **tự thêm tiền tố `0x`**. Chỗ nào ghi `"...0x"` rồi gọi `wr_hex()`
sẽ ra `0x0x00`. Lỗi này cũng nằm trong `wr_hex_col()` của `DBG_MODE_STATUS`
mới viết: nó cộng thêm hai ký tự nên **tràn cột và làm lệch toàn bộ phần sau**.
Đã sửa cả hai chỗ và kiểm lại căn cột bằng script mô phỏng.

### Tiêu chí đạt

- [ ] Console in `QMC6309: OK qua sensor hub, chip id 0x = 144` (144 = 0x90)
- [ ] `DBG_MODE_MAG`: `count` tăng đều, `hz` ≈ 50
- [ ] `nack` = 0 và `err` = 0
- [ ] `busy` không tăng liên tục
- [ ] Xoay máy bay quanh trục yaw → ba cột `mx/my/mz` đổi mượt theo hình sin
- [ ] **`Btot` gần như không đổi ở mọi hướng** — phép thử tốt nhất. Từ trường
      Trái Đất khoảng 0,25–0,65 G tuỳ vị trí địa lý.
- [ ] `DBG_MODE_STATUS`: `rhz` vẫn ≈ 3780, `health` có thêm bit 8 (`SENSOR_MAG`)

Nếu `chip id` đọc ra `0x00`: không ai trả lời trên bus I2C phụ. Kiểm tra module
có thật sự nối QMC6309 vào chân SDX/SCX của LSM6DSV không — có module chỉ đưa
từ kế ra chân I2C riêng.

### Tiêu chí đạt

- [ ] Đọc đúng chip-ID của QMC6309 qua sensor hub
- [ ] Xoay quanh trục yaw → ba giá trị mag đổi mượt theo hình sin
- [ ] **`|B|` gần như không đổi ở mọi hướng** — phép thử tốt nhất. Lệch lớn
      nghĩa là chưa hiệu chuẩn hoặc đọc sai byte.

---

## Giai đoạn 4 — Hiệu chuẩn từ kế 🔴 KHÔNG ĐƯỢC BỎ QUA

**Từ kế chưa hiệu chuẩn TỆ HƠN là không có.** Nó kéo yaw sai một cách tự tin,
và EKF sẽ tin nó. Không được bật hợp nhất yaw (giai đoạn 6) khi các số này còn
là mặc định.

### Nguyên lý

Từ trường Trái Đất có **độ lớn không đổi**. Xoay máy bay theo mọi hướng thì đầu
mút vector từ trường phải vẽ ra một **mặt cầu** tâm gốc toạ độ.

Thực tế nó vẽ ra một **ellipsoid bị dời tâm**:

| Méo | Nguồn | Sửa bằng |
|---|---|---|
| **Tâm bị dời** (sắt cứng) | nam châm trong động cơ cộng một hằng số vào mọi số đo | 3 offset |
| **Méo thành ellipsoid** (sắt mềm) | kim loại quanh chip bẻ cong đường sức, độ nhạy mỗi trục khác nhau | 3 hệ số tỉ lệ |

Dùng min/max từng trục thay vì khớp ellipsoid đầy đủ — đơn giản, bền, và đủ tốt
khi ellipsoid **không bị xoay**, tức khi trục cảm biến trùng hướng méo.

### ⚙️ Hiệu chuẩn tính trong HỆ CẢM BIẾN

Đã đổi thứ tự xử lý trong `lsm6dsv_mag_update()`:

```
số thô -> đổi thang -> HIỆU CHUẨN (hệ cảm biến) -> xoay trục -> hệ thân
```

Méo sắt cứng và sắt mềm gắn với **chip** và khối kim loại quanh nó, không gắn
với hướng lắp. Hiệu chuẩn ở hệ cảm biến rồi mới xoay trục nghĩa là sau này sửa
`MAG_AXIS_*` cũng **không làm hỏng** bộ số hiệu chuẩn.

Thêm trường `g_fc.mag.raw_gauss` — số đã đổi thang, hệ cảm biến, **chưa** hiệu
chuẩn. Đó là thứ công cụ hiệu chuẩn đọc.

### Dùng `DBG_MODE_MAGCAL`

Console đã đặt sẵn mode này.

```
     mx     my     mz |    Btot |     mau | phu |  off_x  off_y  off_z |   sc_x   sc_y   sc_z | huong dan
  0.390  0.232 -0.034 |   0.455 |     250 |   0 |  0.389  0.231 -0.035 |  1.167  1.167  0.778 | XOAY TIEP - con thieu 8/8 huong
```

**Quy trình:**

1. **Mang máy bay ra xa động cơ, xa dây nguồn, xa bàn kim loại.** Hiệu chuẩn
   cạnh một khối sắt sẽ nhét chính khối sắt đó vào bộ số.
2. Cầm máy bay lên, **xoay thật chậm theo mọi hướng** trong ~30 giây: vẽ hình
   số 8 trong không gian, lật bụng, dựng đầu, nghiêng cánh.
3. Cột **`phu`** đếm số góc phần tám đã chạm tới. **Phải đạt đủ 8** thì kết quả
   mới đáng tin. Dưới 8 nghĩa là còn hướng chưa xoay tới, và min/max của trục
   đó sẽ sai.
4. Khi console báo `DU DIEU KIEN`, **bấm nút K1** để chốt và in kết quả.
5. Chép sáu dòng in ra vào `fc_config.h`.

Công cụ tự cảnh báo hai trường hợp đáng ngờ: chưa phủ đủ 8 hướng, và ba bán
trục lệch nhau hơn 1,5 lần (méo sắt mềm quá nặng để bù bằng ba hệ số đường chéo).

### ⚠️ Bẫy trong chính công cụ đo độ phủ — đã sửa

Bản đầu báo `phu = 8` **ngay khi máy bay còn nằm yên**. Lý do: tâm được lấy là
trung điểm min/max đang chạy, nên nhiễu vài phần nghìn Gauss quanh một điểm vẫn
vượt qua tâm theo cả tám hướng.

Đã sửa: chỉ đếm góc phần tám khi mẫu nằm **cách tâm ít nhất 0,3 lần độ lớn
vector**. Xoay 180° làm điểm dịch chuyển ~2 lần độ lớn, còn nhiễu đứng yên gần
bằng không — cách biệt rất rộng nên ngưỡng này không nhạy cảm.

### ✅ Kết quả đo 2026-08-26

```c
#define MAG_OFFSET_X_G 0.0475f
#define MAG_OFFSET_Y_G 0.2315f
#define MAG_OFFSET_Z_G 0.1270f
#define MAG_SCALE_X    0.9917f
#define MAG_SCALE_Y    0.9138f
#define MAG_SCALE_Z    1.1144f
```

| Chỉ số | Giá trị | Đánh giá |
|---|---|---|
| Số mẫu | 2405 | ✅ vượt xa mức tối thiểu 300 |
| Độ phủ | **8/8** góc phần tám | ✅ |
| Bán trục X / Y / Z | 0,3405 / 0,3695 / 0,3030 G | |
| Lệch giữa ba bán trục | **1,22 lần** | ✅ dưới ngưỡng cảnh báo 1,5 |
| Bán kính trung bình | **0,338 G** | ✅ đúng dải Trái Đất 0,25–0,65 G |

**Đáng lưu ý: offset trục Y = 0,2315 G, bằng 68% độ lớn từ trường.** Lệch sắt
cứng khá lớn. Không sao chừng nào nó CỐ ĐỊNH — nhưng nếu dời module hoặc đổi
cách bắt bo thì **phải hiệu chuẩn lại**.

### 🔬 Kiểm chứng bằng phép xoay — và min/max hoá ra là phương pháp TỆ

Xoay máy bay mọi hướng với bộ số min/max, đo `Btot` trên 1176 mẫu:

```
Btot: trung bình 0,3796 G   dao động 9,8%    <- muc tieu la duoi 5%
```

Không đạt. Phân tích sâu trên chính bộ dữ liệu đó:

| Cách hiệu chỉnh | Dao động `Btot` |
|---|---|
| min/max (đang dùng) | **9,8%** |
| chỉ sửa **tâm** | 3,5% |
| tâm + 3 hệ số đường chéo | **1,7%** ✅ |
| ma trận 3×3 đầy đủ | 1,5% |

**Hai kết luận:**

1. **Định dạng config 3+3 là ĐỦ.** Ma trận 3×3 đầy đủ chỉ hơn 0,2% — không
   đáng thêm phức tạp. Ellipsoid gần như không bị xoay (phần tử ngoài đường
   chéo lớn nhất 0,0125 so với đường chéo ~0,995).
2. **Phương pháp min/max mới là chỗ yếu.** Nó chỉ dùng đúng SÁU điểm cực trị
   trong hàng nghìn mẫu; không xoay tới đúng cực trị của một trục thì tâm và
   bán trục trục đó sai ngay.

Tệ hơn: min/max **hiểu nhầm việc xoay không đều thành méo sắt mềm**. Nó cho ra
hệ số tỉ lệ `(0,99 · 0,91 · 1,11)` trong khi giá trị đúng là `(1,00 · 1,00 ·
1,01)` — tức từ kế này gần như KHÔNG có méo sắt mềm, toàn bộ sai lệch nằm ở
TÂM.

Đã thay thuật toán bằng **khớp ellipsoid bình phương tối thiểu**, dùng MỌI mẫu
nên không phụ thuộc việc có chạm đúng cực trị hay không:

```
Mo hinh:  dᵀv = 1  voi  d = [x², y², z², 2xy, 2xz, 2yz, 2x, 2y, 2z]
```

Tích luỹ trực tiếp phương trình chuẩn tắc AᵀA và Aᵀ1 nên **không cần lưu mẫu**
— 45 phép nhân cộng mỗi mẫu, ở 10 Hz là không đáng kể. Dùng `double` cho bộ
tích luỹ vì x⁴ cộng dồn hàng nghìn lần sẽ làm float 32 bit mất chính xác; M7
giả lập double bằng phần mềm nhưng ở 10 Hz thì vài micro giây là quá rẻ.

### ✅ Bộ số sau khi sửa thuật toán

```c
#define MAG_OFFSET_X_G 0.0028f
#define MAG_OFFSET_Y_G 0.2674f
#define MAG_OFFSET_Z_G 0.0955f
#define MAG_SCALE_X    1.0005f
#define MAG_SCALE_Y    0.9977f
#define MAG_SCALE_Z    1.0117f
```

`|B|` kỳ vọng **0,386 G**, dao động **1,7%**.

Ba hệ số tỉ lệ đều ≈1,00 — xác nhận từ kế không có méo sắt mềm đáng kể.
Offset trục Y = 0,267 G, bằng 69% độ lớn từ trường: lệch sắt cứng lớn, phải
hiệu chuẩn lại nếu dời module hay đi lại dây nguồn.

### ✅ Kiểm chứng cuối — ĐẠT

Xoay mọi hướng với bộ số mới, 1756 mẫu:

| Chỉ số | Trước (min/max) | Sau (bình phương tối thiểu) |
|---|---|---|
| `Btot` trung bình | 0,3796 G | **0,3834 G** (kỳ vọng 0,386) |
| **Dao động** | 9,8% | **2,3%** ✅ mục tiêu <5% |
| Phụ thuộc hướng | 19,3% | **6,3%** |

Ba trục đều quét ~0,7 G nên phép kiểm này đã xoay đủ.

Phần phụ thuộc hướng còn lại 6,3% nhiều khả năng là **từ trường tại chỗ thay
đổi thật** — cầm máy bay xoay thì nó cũng di chuyển trong không gian, và gần
bàn kim loại hay máy tính thì từ trường khác nhau theo vị trí. Đó không phải
lỗi cảm biến và không sửa được bằng hiệu chuẩn.

### Kiểm tra không có lỗi đọc rách dữ liệu

Trong 1175 cặp mẫu liên tiếp, **0 trường hợp** `Btot` nhảy quá 0,03 G trong khi
hướng đổi dưới 3°. Máy trạng thái 3 pha đọc sensor hub sạch, không bị lấy nhầm
dữ liệu nửa cũ nửa mới.

### 🔴 Một lỗi AN TOÀN phát lộ khi đo — đã sửa

Trong log hiệu chuẩn có dòng `Quay thu motor = 1`: nút K1 vừa chốt hiệu chuẩn
**vừa khởi động quay thử động cơ**. Hai chức năng dùng chung một nút.

Với cánh quạt đã tháo thì chỉ là phiền, nhưng **nếu còn cánh thì đó là tai nạn**
— người dùng đang cầm máy bay trên tay để xoay hiệu chuẩn.

Đã chặn: bộ xử lý nút K1 trong `main.c` bỏ qua hoàn toàn khi console đang ở
`DBG_MODE_MAGCAL`.

Đây là loại xung đột chỉ lộ ra khi chạy thật, không lộ ra khi đọc code — mỗi
chức năng riêng lẻ đều đúng, chỉ có việc dùng chung nút là sai.

### Một lỗi hiển thị nhỏ

`dbg_print_float()` tự chèn `" = "`, nên các dòng in ra thành
`#define MAG_OFFSET_X_G = 0.0475` — không phải C hợp lệ, không chép thẳng được.
Đã đổi sang dựng chuỗi thủ công để in ra đúng dạng `#define TÊN giá_trị`.

### Tiêu chí đạt

- [x] `phu` = **8/8** trước khi chốt
- [x] Số mẫu ≥ 300 (đạt 2405)
- [x] Ba bán trục không lệch nhau quá 1,5 lần (đạt 1,22)
- [x] Sau khi nạp bộ số: xoay mọi hướng, `Btot` dao động **1,7%** (mục tiêu <5%)
- [ ] **THÁO CÁNH QUẠT**, arm, tăng ga 0 → 50%: `Btot` không đổi quá 10%.
      Đổi nhiều nghĩa là dây động cơ tạo từ trường theo dòng — phải dời module
      ra xa dây nguồn, hoặc bù theo dòng điện (cần driver ADC pin).

### 🔴 SỰ CỐ PHẦN CỨNG: cắm pin thì mất từ kế

**Triệu chứng:** chỉ cấp nguồn USB thì QMC6309 chạy tốt. Cắm pin vào mạch là
mất hẳn:

```
KHONG THAY - khong ai tra loi o dia chi 0x7C | id=0x00 STATUS_MASTER=0x09
```

**`STATUS_MASTER = 0x09`** = bit0 `ENDOP` + bit3 `slave0_nack`. Đọc ra: bộ máy
I2C master **đã chạy xong** một giao dịch, nhưng **không ai trả lời**.

#### Đã đo, và loại trừ được gần hết khả năng

| Kiểm tra | Kết quả | Loại trừ được |
|---|---|---|
| `WHO_AM_I` của LSM6DSV | `0x70` ✓ | SPI3 hỏng |
| `CTRL1`/`CTRL2` | `0x0A` ✓ | accel/gyro không chạy |
| `IF_CFG` | `0x40` ✓ | quên bật pull-up |
| `STATUS_REG` | `0x07` ✓ | không có nhịp kích hub |
| Chuyển bank shub | được ✓ | không truy cập được bank |
| `ENDOP` | luôn có ✓ | hub không chạy |
| Dò chip ID **10 lần** | **0/10** đúng | nhiễu chập chờn |
| Quét **8 địa chỉ** ứng viên | **không cái nào** trả lời | sai địa chỉ |

**Kết luận: đây là sự cố PHẦN CỨNG, không phải firmware.**

`NACK` nghĩa là đường SDA **vẫn ở mức cao** trong bit ACK — không ai kéo nó
xuống. Đó là chữ ký của **chip mất nguồn hoặc không hoạt động**, chứ không
phải nhiễu (nhiễu sẽ cho kết quả lúc được lúc không, mà ta đo được 0/10).

LSM6DSV nằm **cùng module** vẫn chạy hoàn hảo, nên nguồn chung của module vẫn
có. Vấn đề khu trú ở riêng QMC6309.

#### Khớp với phát hiện ở giai đoạn 2C

Đã biết từ trước: cắm pin thì nhiễu gyro của LSM6DSV **tăng 20 lần**
(σ 0,08 → 1,85 °/s) trong khi gia tốc kế vẫn sạch. Module này rõ ràng rất
nhạy với nhiễu từ ESC. Giờ thêm bằng chứng: QMC6309 chết hẳn khi có pin.

Hai triệu chứng khác nhau, cùng một nguyên nhân gốc: **nhiễu điện từ mạch
công suất lọt vào module qua dây nối.**

#### Việc cần làm — theo thứ tự

1. **Đo điện áp VDD tại chân module** bằng đồng hồ, lúc có pin và lúc không.
   QMC6309 cần 2,5–3,6 V. Sụt dưới 2,5 V là ra ngay nguyên nhân.
2. **Thử tắt bật nguồn hoàn toàn** khi vẫn cắm pin (rút hẳn pin, đợi 10 giây,
   cắm lại). Nếu từ kế sống lại thì đó là **latch-up** do xung quá độ lúc cắm
   pin, chứ không phải nhiễu liên tục.
3. **Thêm tụ lọc ngay tại module** — 100 nF gốm song song 10 µF, sát chân VDD.
   Đây là biện pháp rẻ và hiệu quả nhất cho cả hai triệu chứng.
4. **Dời module ra xa dây nguồn và ESC.** Dây tín hiệu SPI3 nên xoắn đôi với
   dây đất.
5. Nếu vẫn còn: cấp nguồn riêng cho module từ một LDO khác, tách khỏi rail
   dùng chung với mạch công suất.

#### Đã tách nguồn từ kế thành lựa chọn cấu hình

Thêm phần cứng ngoài (tụ lọc, bỏ nguồn pin) **không cứu được** — QMC6309 vẫn
NACK vĩnh viễn ngay cả khi chỉ dùng nguồn USB, tức tình huống trước đó đã chạy
tốt. Nghĩa là chip đã hỏng hoặc dây bị đứt trong lúc lắp thêm linh kiện.

Nên chuyển sang một lựa chọn cấu hình thay vì gỡ code:

```c
#define MAG_SOURCE_NONE 0     /* khong co tu ke                          */
#define MAG_SOURCE_SHUB 1     /* QMC6309 sau sensor hub - DA HIEN THUC   */
#define MAG_SOURCE_I2C  2     /* module roi tren I2C rieng - CHUA CO     */

#define MAG_SOURCE MAG_SOURCE_NONE
```

**Toàn bộ hạ tầng từ kế giữ nguyên**, không xoá dòng nào: `mag_data_t`, bộ số
hiệu chuẩn đã đo, `DBG_MODE_MAG`, `DBG_MODE_MAGCAL` với thuật toán bình phương
tối thiểu, phần nối vào `sensor_health`, và cả driver sensor hub đã chạy được.

Mọi thứ phía sau tầng driver đều **không phụ thuộc nguồn**. Lắp module rời sau
này chỉ cần viết driver mới rồi đổi hằng số — hiệu chuẩn, hiển thị, và phần
hợp nhất yaw ở giai đoạn 6 dùng lại nguyên vẹn.

Với `MAG_SOURCE_NONE`: `g_fc.mag` đứng im, `healthy = false`, bit `SENSOR_MAG`
không bao giờ bật. Đã kiểm trên bo: `health = 0x041F` (bit 8 tắt đúng như
mong đợi), không sinh cờ lỗi nào.

**Lợi ra:** bỏ từ kế lấy lại `rhz` 3756 → **3790**, và `lmax` 311 → **231 µs**.

#### Vì sao KHÔNG thêm cơ chế tự thử lại trong firmware

Đã cân nhắc và quyết định không làm. `lsm6dsv_mag_init()` chặn khoảng 2–3 giây
(mỗi giao dịch sensor hub cần chờ cố định 400 ms). Gọi lại nó từ vòng lặp
chính sẽ **treo vòng PID và ngừng phát khung DShot** trong chừng ấy thời gian.

Mà phép đo cho thấy lỗi là **vĩnh viễn 0/10**, nên thử lại cũng không cứu được
gì. Thêm một đường code rủi ro để che một sự cố phần cứng là đánh đổi sai.

Thay vào đó console báo rõ ràng và chính xác tầng đang hỏng — đó mới là thứ
hữu ích.

### ⛔ Còn nợ trước khi sang giai đoạn 6

`MAG_AXIS_MAP_*` và `MAG_AXIS_SIGN_*` vẫn là **mặc định, chưa đo**. QMC6309 là
chip RIÊNG trên cùng module với LSM6DSV, hướng đặt của nó không nhất thiết trùng
với LSM6DSV (mà LSM6DSV thì đã đo được là vừa xoay vừa lật).

Chưa xác định hướng trục thì heading tính ra sẽ sai — có thể quay ngược chiều
hoặc lệch 90°. Phải làm xong việc này trước khi đưa yaw vào EKF.

Cách làm dự kiến: xoay quanh từng trục thân, trục cảm biến nào **ít thay đổi
nhất** chính là trục trùng với trục xoay; dấu lấy bằng cách đối chiếu chiều
biến thiên heading với tốc độ yaw của con quay.

## Giai đoạn 5 — Hợp nhất gyro/accel

*Chỉ làm nếu giai đoạn 2 cho đèn xanh.*

### 🔒 Nguyên tắc bất di bất dịch: ICM20602 vẫn là đồng hồ chủ

`ctrl_rate_update()` bám nhịp bằng `g_fc.imu.sample_count` với bộ chia 2
(`ctrl_rate.c:132`). **Đụng vào ngữ nghĩa trường này là đụng vào vòng 4 kHz.**

Thiết kế:

- LSM6DSV chạy ODR riêng, ghi vào `g_fc.imu2`
- Tầng hợp nhất chạy **trong `process_sample()` của ICM**, lấy mẫu LSM mới
  nhất đang có, trộn theo trọng số nghịch đảo phương sai (đo ở GĐ2)
- `sample_count`, `timestamp_us`, `dt_us` **giữ nguyên của ICM**

Lệch thời gian giữa hai mẫu < 0.5 ms, trong khi bộ lọc chỉ có băng thông
100 Hz — không đáng kể.

### Kiểm tra bất đồng

Nếu hai IMU lệch nhau quá ngưỡng kéo dài → bỏ cái nghi ngờ, hạ `healthy`,
đặt cờ lỗi. **Đây mới là giá trị thật của IMU thứ hai: phát hiện hỏng.**

### Tiêu chí đạt

- [ ] σ gyro sau hợp nhất giảm đo được so với ICM đơn lẻ
- [ ] `hz` vẫn ≈ 4000
- [ ] Rút phích module LSM giữa chừng → tự chuyển về ICM đơn, không giật

---

## Giai đoạn 6 — Yaw từ mag vào EKF ⭐ phần thưởng thật sự

Hiện `ekf_attitude` có 6 trạng thái và accel chỉ sửa được roll/pitch — ma
trận H hạng 2, nhân không gian nằm dọc trục trọng lực (xem chú thích ở
`ekf_attitude.c:287`). Từ kế cho đúng chiều còn thiếu.

### Thiết kế: cập nhật VÔ HƯỚNG chỉ trên hướng mũi

Không phải cập nhật 3 chiều. Lý do:

- Cập nhật 3 chiều để sai số từ trường (độ từ khuynh sai, sắt mềm còn sót)
  **ngấm sang roll/pitch** — mà roll/pitch đang được accel sửa rất tốt rồi.
- Chỉ lấy đúng thứ accel không cho được.
- Rẻ hơn nhiều: một phép chia thay vì nghịch đảo ma trận 3×3.

```
1. Nghiêng-bù vector mag về mặt phẳng ngang bằng s_R có sẵn
2. Hướng đo được = atan2(-m_east, m_north)
3. Sai lệch với yaw ước lượng, GÓI VỀ (−π, π]
4. Cập nhật vô hướng với H = [0 0 1 0 0 0]
```

### Cổng loại bỏ mẫu

Theo đúng tinh thần các cổng đã dựng cho flow/range:

- `|B|` lệch quá ngưỡng so với lúc hiệu chuẩn → gần vật sắt, bỏ
- Nghiêng quá ngưỡng → bù nghiêng mất chính xác, bỏ
- Sai lệch hướng nhảy đột ngột → nhiễu xung, bỏ

### Dọn dẹp sau khi xong

- **Bỏ đoạn chặn cứng bias yaw** ở `ekf_attitude.c:402` — có mag thì bias
  trục Z quan sát được, không cần chặn nữa
- `ctrl_angle.c` bỏ ghi chú "yaw luôn theo tốc độ vì không có la bàn",
  làm **giữ hướng** thật

### Tiêu chí đạt

- [ ] Để yên 10 phút, yaw không trôi quá vài độ *(hiện tại trôi hàng chục độ)*
- [ ] Xoay 360° rồi về vị trí cũ → yaw về đúng giá trị ban đầu

---

## Rủi ro và cách chặn

| Rủi ro | Chặn thế nào |
|---|---|
| **Tải CPU** làm tụt vòng 4 kHz | ODR 1920 Hz chứ không 8 kHz. Đo `hz` sau **mỗi** giai đoạn. |
| **EXTI9_5 dùng chung** EXTI5..9 | Hiện chỉ PD7 dùng. Thêm ngắt trong dải này thì `HAL_GPIO_EXTI_IRQHandler` phải gọi cho từng chân. |
| **DMA không thấy bộ nhớ** | Đệm bắt buộc `FC_DMA_BUFFER` — DMA1/DMA2 trên H743 không truy cập được DTCM. |
| **Hệ số đổi thang sai** | Kiểm trọng lực (accel) + so ICM (gyro) ở GĐ2. |
| **Từ kế nhiễu do động cơ** | Đo ở GĐ4 trước khi tin nó. Cổng `|B|` ở GĐ6. |
| **Làm hỏng cái đang bay** | GĐ1–4 không đụng `g_fc.imu`, `App/Control/`, `App/Estimator/`. |

---

## Gỡ lỗi giai đoạn 1

Khi chưa có mẫu nào về, dòng `IMU2` tự nối thêm ba số chẩn đoán:

```
[CHUA CO MAU edges=0 st=0x03 pd7=1]
```

Ba số này tách bạch ba tầng có thể hỏng:

| Số | Ý nghĩa |
|---|---|
| `edges` | Số sườn lên bắt được trên PD7, đếm **trước** mọi kiểm tra trạng thái |
| `st` | `STATUS_REG` — bit0 `XLDA`, bit1 `GDA`, bit2 `TDA` |
| `pd7` | Mức logic hiện tại của chân PD7 |

| Tổ hợp | Kết luận |
|---|---|
| `edges=0`, `st` có `GDA`, `pd7=1` **dai dẳng** | **INT1 bị chốt ở mức cao** — thiếu `DRDY_PULSED`. Đây là lỗi đã gặp thật. |
| `edges=0`, `st=0x00` | **Chip không lấy mẫu.** Lỗi ở ODR hoặc chế độ hoạt động, không phải đường ngắt. |
| `edges=0`, `st` có `GDA`, `pd7=0` | Dây INT chưa nối, hoặc nối vào INT2 chứ không phải INT1 |
| `edges>0` nhưng `count=0` | EXTI có kích nhưng driver chặn, hoặc DMA hỏng |

### ⚠️ Bẫy đã gặp: DRDY chốt mức thay vì phát xung

**Triệu chứng:** init báo OK, `WHO_AM_I = 0x70`, mọi thanh ghi đọc lại đúng,
nhưng `count`, `err`, `ovr` **đều đứng yên ở 0** — ISR chưa vào lần nào.

**Nguyên nhân:** `CTRL4` (0x13) bit 1 là `drdy_pulsed`, mặc định = 0 tức chế
độ **CHỐT**. INT1 lên cao khi có mẫu và chỉ hạ xuống **khi dữ liệu được đọc**.
Với EXTI bắt sườn lên thì khoá chết ngay từ mẫu đầu:

```
có mẫu -> INT1 lên cao và GIỮ NGUYÊN
       -> không ai đọc dữ liệu vì ISR chưa từng chạy
       -> INT1 không bao giờ hạ -> không còn sườn lên nào nữa
```

ICM20602 không dính vì `INT_PIN_CFG = 0x00` cho nó phát xung 50 µs rồi tự về
thấp.

**Khắc phục:** ghi `CTRL4 = LSM_CTRL4_DRDY_PULSED`, cộng thêm một lượt đọc mồi
khối dữ liệu ở cuối `init()` để xoá trạng thái còn treo. Chế độ phát xung còn
quan trọng ở chỗ: lỡ bỏ một mẫu thì mẫu sau vẫn sinh sườn mới, không khoá chết
vĩnh viễn.

### Bảng chung

| Triệu chứng | Nguyên nhân thường gặp |
|---|---|
| `WHO_AM_I` = `0x00` hoặc `0xFF` | Sai dây, CS không xuống, hoặc module chưa có nguồn |
| `WHO_AM_I` lúc được lúc không | Tốc độ chân GPIO, dây quá dài, thiếu đất chung |
| `WHO_AM_I` ra giá trị lạ nhất quán | Sai chế độ SPI — thử CPOL=0/CPHA=0 |
| `count` không tăng | Đọc ba số chẩn đoán ở trên |
| `count` tăng nhưng số toàn 0 | MISO không về — kiểm PB4 |
| `\|a\|/g` ≈ 0.5 hoặc 2.0 | Sai `fs_xl` hoặc sai hệ số đổi thang |
| `hz` ≈ 960 thay vì 1920 | Sai mã ODR |
| `hz` của PID tụt dưới 4000 | Hạ ODR LSM6DSV xuống 960 Hz |

---

## Nhật ký

| Ngày | Việc |
|---|---|
| 2026-08-26 | Cấu hình CubeMX xong: SPI3 mode 3 @8MHz, PB3/PB4/PD6 Very High, PD7 EXTI rising, DMA2 S2/S3, NVIC prio 4. Viết giai đoạn 1. |
| 2026-08-26 | Chạy thử lần 1: `WHO_AM_I = 0x70` đúng, init OK, nhưng `count = 0`. Nguyên nhân: thiếu `DRDY_PULSED` trong `CTRL4` — INT1 bị chốt mức cao. Đã sửa, thêm ba số chẩn đoán `edges`/`st`/`pd7`. |
| 2026-08-26 | Chạy thử lần 2 — **giai đoạn 1 chạy được**. 200 dòng, `err`/`ovr` đều 0, không mất mẫu. Thang accel xác nhận đúng (`\|a\|/g` = 1.0028). ODR thực 1846 Hz (−3,9%, dao động nội của chip, chấp nhận). Nhiễu gyro σ ≈ 0.08–0.09 °/s. **Phát hiện: dấu trục Z sai** — `az` = +9.83 thay vì −9.81. Còn nợ: kiểm `hz` của PID và `DBG_MODE_STATUS`. |
| 2026-08-26 | **Đổi tiêu chí phát hiện chuyển động khi hiệu chuẩn bias: đỉnh-đỉnh → ĐỘ LỆCH CHUẨN.** Triệu chứng: `huy 174 lan` liên tục, bias LSM không bao giờ được trừ (`gx_2` lệch −5 °/s). Nguyên nhân: biên độ đỉnh-đỉnh **tăng theo số mẫu** (~4,5σ trên 1000 mẫu), mà nhiễu nền LSM σ=1,82 cho biên độ ~8,2 °/s — vượt xa ngưỡng 2,0 nên **bất khả thi**. σ không phụ thuộc số mẫu. Sửa cả hai driver, so bình phương để khỏi gọi `sqrtf` trong ISR. Ngưỡng mới: ICM 1,0 (nền 0,09), IMU2 5,0 (nền 1,82). **Đã kiểm: `gx_2` từ −5 về dao động quanh 0, thông báo lỗi biến mất.** |
| 2026-08-26 | **Tách nguồn từ kế thành `MAG_SOURCE`.** Bỏ nguồn pin và lắp tụ đều không cứu được — QMC6309 vẫn NACK vĩnh viễn ngay cả khi chỉ dùng USB (tình huống trước đó chạy tốt), nên chip hỏng hoặc dây đứt. Chuyển sang lựa chọn cấu hình `NONE/SHUB/I2C`, **giữ nguyên toàn bộ hạ tầng** để lắp module rời sau. Đã kiểm: `health = 0x041F` (bit `SENSOR_MAG` tắt đúng), `rhz` 3756 → **3790**, `lmax` 311 → **231 µs**. |
| 2026-08-26 | 🔴 **Sự cố phần cứng: cắm pin thì mất QMC6309.** `STATUS_MASTER = 0x09` = ENDOP + NACK, tức hub chạy xong nhưng không ai trả lời. Đo trực tiếp qua ST-Link: LSM6DSV hoàn hảo, dò chip ID **0/10**, quét **8 địa chỉ không cái nào** trả lời. Không phải nhiễu chập chờn, không phải sai địa chỉ — **chip mất nguồn hoặc không hoạt động**. Khớp với phát hiện GĐ2C (nhiễu ESC làm gyro LSM6DSV tăng 20 lần). Quyết định KHÔNG thêm tự-thử-lại vì `mag_init` chặn 2–3 giây, sẽ treo vòng PID và ngừng DShot. |
| 2026-08-26 | **GĐ4 XONG.** Kiểm chứng bộ số bình phương tối thiểu trên 1756 mẫu: `Btot` = 0,3834 G, **dao động 2,3%** (mục tiêu <5%, trước đó 9,8%). Phụ thuộc hướng giảm từ 19,3% xuống 6,3% — phần dư nhiều khả năng là từ trường tại chỗ thay đổi thật, không phải lỗi cảm biến. **Còn nợ: `MAG_AXIS_*` chưa đo**, và phép thử tăng ga chưa làm. |
| 2026-08-26 | **GĐ4: phát hiện min/max là phương pháp tệ.** Kiểm chứng bằng xoay thật: bộ số min/max cho `Btot` dao động **9,8%**, quá mục tiêu 5%. Phân tích lại trên chính dữ liệu đó: chỉ sửa tâm → 3,5%; tâm + 3 hệ số → **1,7%**; ma trận 3×3 đầy đủ → 1,5%. Nên định dạng 3+3 là đủ, lỗi nằm ở THUẬT TOÁN. Thay bằng **khớp ellipsoid bình phương tối thiểu** (tích luỹ AᵀA 9×9, không lưu mẫu). Bộ số mới cho 1,7%. Xác nhận từ kế gần như không có méo sắt mềm — min/max đã hiểu nhầm việc xoay không đều thành sắt mềm. |
| 2026-08-26 | **GĐ4: đã hiệu chuẩn.** 2405 mẫu, phủ 8/8, ba bán trục lệch 1,22 lần, bán kính TB 0,338 G. Offset Y = 0,2315 G (68% độ lớn từ trường) — lệch sắt cứng lớn, phải đo lại nếu dời module. **Phát lộ lỗi AN TOÀN: nút K1 vừa chốt hiệu chuẩn vừa quay thử động cơ** — đã chặn khi ở `DBG_MODE_MAGCAL`. Sửa luôn dòng `#define` in ra bị thừa dấu `=`. |
| 2026-08-26 | **GĐ4: công cụ xong.** Chuyển hiệu chuẩn sang **hệ cảm biến** (áp trước khi xoay trục) để sau này sửa `MAG_AXIS_*` không làm hỏng bộ số; thêm `g_fc.mag.raw_gauss`. Thêm `DBG_MODE_MAGCAL` gom min/max, đếm độ phủ 8 góc phần tám, chốt bằng nút K1. Sửa lỗi trong chính công cụ: nhiễu lúc đứng yên bị đếm nhầm thành "phủ đủ 8 hướng" — nay phải cách tâm ≥0,3 lần độ lớn mới tính. **Còn nợ: `MAG_AXIS_*` chưa đo**, chặn GĐ6. |
| 2026-08-26 | **GĐ3 XONG.** Gỡ trực tiếp trên phần cứng qua ST-Link + UART. **Năm** nguyên nhân riêng biệt cùng cho một triệu chứng: (1) thiếu pull-up `IF_CFG` bit6; (2) accel phải tắt/bật mới kích hub; (3) hỏi vòng phá chính bộ máy đang chờ — phải đợi cố định; (4) chờ nhầm `WR_ONCE` thay vì `ENDOP`, và `shub_odr=0` ở đường ghi; (5) `write_once` để lại hub KẸT, phải xung `rst_master_regs`. Bỏ chế độ đọc liên tục (không quay vòng), chuyển sang máy trạng thái 3 pha không chặn. **Đo được Btot = 0,45 G**, `rhz` chỉ giảm 0,5%. |
| 2026-08-26 | **GĐ3 chạy thử lần 2: `STATUS_MASTER = 0x00`, hub nằm im.** Pull-up đã đúng nhưng thiếu trình tự: bộ máy sensor hub chỉ khởi động ở sườn DRDY **đầu tiên của accel sau khi bật master**, mà accel đã chạy sẵn. Sửa theo đúng ví dụ chính thức của ST: tắt accel → bật master → bật lại accel. Thêm dò hai địa chỉ I2C ứng viên (0x7C và 0x3E) vì datasheet mơ hồ giữa dạng 7 bit và 8 bit. |
| 2026-08-26 | **GĐ3 chạy thử lần 1: `chip_id = 0x00`.** Nguyên nhân: bus I2C phụ thiếu điện trở kéo lên — phải bật `shub_pu_en`, **bit 6 của `IF_CFG` (0x03)**, không phải trong `MASTER_CONFIG` như đời LSM6DSO. Thêm `lsm6dsv_mag_init_result()` tách bạch 7 tầng hỏng để lần sau không phải đoán. Sửa luôn lỗi `wr_hex()` tự thêm `0x` gây ra `0x0x00` — lỗi này cũng làm **lệch cột** trong `DBG_MODE_STATUS` mới. |
| 2026-08-26 | **GĐ3 viết xong.** Lấy bản đồ thanh ghi QMC6309 từ datasheet chính thức QST — phát hiện thư viện `SensorLib` trên GitHub đặt SAI vị trí bit (ba mặt nạ chồng lấn trong một thanh ghi 8 bit). Datasheet tự kiểm chứng khớp với hai ví dụ trong chính nó. Thêm `qmc6309.h`, khối sensor hub trong `lsm6dsv.c`, `mag_data_t`, `DBG_MODE_MAG`, và cơ chế tranh chấp bus SPI3 giữa vòng lặp chính và DMA của IMU. Build sạch, **chưa test phần cứng**. |
| 2026-08-26 | **2C XONG — CỔNG ĐÓNG, HUỶ GĐ5.** Động cơ chạy: `sd_1` 0,09 → 45,6 °/s (175×), `sd_2` 1,79 → 21,0. Cả hai cùng tăng vọt = rung khung. **Phát hiện lớn hơn: KHUNG BỊ CỘNG HƯỞNG** — ba đỉnh nhọn 26–46 °/s ở vài dải ga, giữa các đỉnh chỉ 5–9. Khớp đúng mô tả của người dùng. Đây là việc phải xử lý trước cả althold. Ngoài ra gyro LSM ồn bất thường lúc đứng yên (σ 1,85 vs 0,08 của ICM) khiến hiệu chuẩn bias lặp vô hạn — đã thêm bộ đếm `calib_restarts` để lỗi không còn im lặng. |
| 2026-08-26 | **2B XONG — thang gyro ĐÚNG.** Tích phân 7 đoạn xoay, tỉ số LSM/ICM trung bình 1,0020 (lệch 0,20%). Hằng số cuối cùng đã xác minh. 2A được xác nhận lại độc lập trong miền gyro. **Phát hiện mới: hai IMU lệch nhau 1,34°**, lặp lại sát trong từng cặp trục — lý lẽ mạnh chống lại việc lấy trung bình ở GĐ5. 2C mới xong nửa (thiếu phép đo lúc động cơ chạy). |
| 2026-08-26 | **2A XONG.** `AXISCAL` lần 2 ra `MAP (1,0,2)` / `SIGN (−1,−1,−1)`, định thức +1. Module lắp vừa xoay vừa lật: thân X = −cảm biến Y, thân Y = −cảm biến X, thân Z = −cảm biến Z. Đã áp vào `fc_config.h`. Console chuyển sang `DBG_MODE_IMU_CMP` cho 2B/2C. |
| 2026-08-26 | Chạy `AXISCAL` lần 1: ra `MAP_X=MAP_Y=MAP_Z=2`, **công cụ tự từ chối** vì định thức ≠ +1 — cổng kiểm đã làm đúng việc. Nguyên nhân là lỗi của công cụ: ba bước tự chốt cách nhau đúng 40 tick trong khi máy nằm yên. Thêm hai cổng chặn (phải nhấc máy lên mới đo bước sau; không nhận lại trục đã dùng), mô phỏng lại qua 3 kịch bản. **Kết quả giữ được: bước 1 → `MAP_Z = 2`, `SIGN_Z = (−1)`.** |
| 2026-08-26 | `DBG_MODE_STATUS` chia cột như các mode khác (có dòng tiêu đề, thêm `wr_str_pad`/`wr_hex_col`). Thêm `DBG_MODE_AXISCAL` tự nhận chiều trục IMU2 cho giai đoạn 2A — thay quy trình thủ công dễ sai. Thuật toán kiểm bằng mô phỏng qua cả 24 phép quay hợp lệ. |
| 2026-08-26 | **Phép thử A/B xong.** `rhz` 3801 (tắt) → 3774 (bật) = LSM6DSV chỉ tốn **0,7%**. Khoảng thiếu 5% so với 4000 đã có từ trước, thủ phạm là EKF 6×6 chạy 1 kHz (`lp` đỉnh 215 µs). `SENSOR_IMU2` đã lên trong `health = 0x041F`. Sửa ngưỡng `ovr` từ 250 µs xuống **125 µs** — ngưỡng cũ đọc ra 0 trong khi vẫn mất 5% nhịp. |
| 2026-08-26 | Chạy thử lần 3 (PID + STATUS). `imu_err = 0`, `dt = 125 µs`, `drop = 0`, không có cờ lỗi IMU nào. **Nhưng `rhz` = 3780 chứ không phải 4000.** Chưa rõ do LSM6DSV hay đã vậy từ trước. Thêm dụng cụ đo: `loop_time_us`/`loop_time_max_us`/`loop_overruns` (đã khai báo trong `system_data_t` và stream ra telemetry từ trước nhưng **chưa ai ghi vào**), nối `SENSOR_IMU2` vào `fc_state_update_health()` (bị sót), thêm công tắc `IMU2_ENABLE` để làm A/B. `DBG_LINE_MAX` 160 → 200 vì dòng STATUS sẽ bị cắt cụt. |

---

## Blackbox ra thẻ SD + đọc thẻ qua cổng USB — 27/08/2026

Việc này nằm ngoài kế hoạch LSM6DSV, nhưng ghi ở đây vì nó lặp lại đúng một
bài học của dự án và vì nó vá một lỗi hạ tầng đã âm thầm tồn tại từ đầu.

### Cái đã làm được

| Phần | Trạng thái | Bằng chứng đo được |
|---|---|---|
| Ghi log ra thẻ SD | ✅ | `f_mount = 0`, ghi 25 byte rồi `f_stat` đọc lại đúng 25 byte |
| File tự tăng số thứ tự | ✅ | qua nhiều lần khởi động: `LOG0000` → `LOG0006` |
| Đọc thẻ qua USB (MSC) | ✅ | Windows thấy đĩa `FCH743 Blackbox SD`, 7,95 GB, FAT32, gắn ở `E:` |
| Chế độ bay không hồi quy | ✅ | không giữ K1 → CDC lên `COM17`, blackbox mount bình thường |

### Lỗi 1 — `f_mount` trả về `FR_DISK_ERR` dù thẻ hoàn toàn khoẻ

Triệu chứng đánh lừa: mọi phép đo trước khi mount đều sạch.
`HAL_SD_GetCardState = 4` (TRANSFER), `BSP_SD_GetCardState = 0`,
`HAL_SD_GetError = 0`, `BlockNbr = 15.523.840`. Thẻ tốt, mà mount vẫn hỏng.

Gốc rễ: `SD_read()` truyền **thẳng** con trỏ bộ đệm của FatFs xuống
`BSP_SD_ReadBlocks_DMA()`. Mà `SDFatFS` và `SDFile` do CubeMX sinh ra là biến
toàn cục thường → `.bss` → **DTCMRAM ở 0x20000000**. DTCM chỉ nối trực tiếp
với lõi Cortex-M7; IDMA của SDMMC là bus master trên AHB và **không với tới
được vùng đó**. Lệnh đọc phát ra rồi không bao giờ hoàn tất.

Đúng cùng bài học đã ghi sẵn trong linker script cho `.dma_buffer`, chỉ khác
là lần này nạn nhân nằm trong code CubeMX sinh chứ không phải code mình viết.

Cách chữa: `blackbox.c` **tự khai báo** `FATFS`, `FIL` và cả `FIL` dùng để dò
file, đặt trong `.dma_buffer` (AXI SRAM). Không phải sửa file CubeMX sinh ra
nên Generate Code lại cũng không mất.

> Cái `FIL` dùng để dò tên file lúc đầu là **biến cục bộ**. Ngăn xếp cũng ở
> DTCM, nên đó là đúng cùng một lỗi, chỉ chưa phát tác. Đã chuyển thành static.

### Lỗi 2 — lệnh SDMMC chập chờn khi luồng ngắt 8 kHz đang chạy

Bisect 7 điểm qua trình tự khởi động cho thấy trạng thái thẻ đi
`4 → 4 → 0 → 0 → 4 → 4 → 4`, các điểm hỏng trùng với lúc luồng DRDY 8 kHz của
IMU đã bật. Lệnh SDMMC đều có hạn thời gian, mà ngắt mức ưu tiên 0 chạy liên
tục thì chúng trượt hạn.

Cách chữa: gọi `blackbox_init()` **ngay sau** `dbg_console_set_rate(50)`,
trước mọi driver cảm biến.

Hệ quả kéo theo: chế độ đọc thẻ qua USB cũng đi qua đúng những lệnh SDMMC ấy.
Lần thử đầu Windows thấy thiết bị nhưng **dung lượng bằng 0**, vì vòng lặp bay
vẫn chạy. Nên chế độ MSC **dừng hẳn** trước khi khởi tạo cảm biến — xem
`Core/Src/main.c`.

### Lỗi 3 — USB chưa bao giờ có xung 48 MHz 🔴

Cắm cổng USB của bo vào máy tính thì **không có gì hiện ra**, kể cả CDC.

Gốc rễ: trong `SystemClock_Config()`, `PeriphClockSelection` chỉ có
`RCC_PERIPHCLK_ADC`, và bộ dao động chỉ bật HSE. **HSI48 không hề được bật**,
bộ chọn xung USB không hề được đặt. OTG_FS chạy không có xung hợp lệ nên không
bao giờ enumerate được.

Nghĩa là đường telemetry qua CDC ở `App/Telemetry/tlm_port.c` **chưa từng chạy
được** kể từ đầu dự án. Không phải lỗi cáp, không phải lỗi lớp thiết bị.

Cách chữa: bật HSI48 + CRS (bám theo gói SOF của máy chủ, vì USB FS đòi sai số
±0,25% mà HSI48 chạy trần chỉ đạt ±1%) và chọn `RCC_USBCLKSOURCE_HSI48`. Đặt
trong vùng `USER CODE` của `MX_USB_DEVICE_Init()` — chạy trước `USBD_Init()`
nên đúng thứ tự, và Generate Code lại không xoá mất.

> **Nên làm trong CubeMX cho gọn:** RCC → bật HSI48, Clock Configuration → USB
> clock mux → HSI48. Làm rồi thì khối trong `USER CODE` thành thừa nhưng vô
> hại (đặt lại đúng giá trị cũ).

### Cách chọn chế độ USB

Cổng USB chỉ có một, hai lớp không cùng sống được nếu không dùng mô tả ghép mà
CubeMX không sinh ra kiểu đó. Nên chọn lúc khởi động:

- **Giữ K1 rồi cấp điện / bấm reset** → enumerate thành ổ đĩa di động, đọc thẻ
  từ máy tính.
- **Không giữ** → CDC như cũ, bay bình thường.

Trong chế độ MSC: **không** mount FatFs (máy tính đang toàn quyền ghi từng
sector, hai bên cùng ghi thì hỏng bảng FAT), **chặn ARM** qua cờ
`ARM_BLOCK_USB_MSC`, và **dừng hẳn** trước khi khởi tạo cảm biến.

### Hai cái bẫy an toàn đã gỡ

1. **K1 giữ lúc khởi động làm quay động cơ.** Trình xử lý K1 bắt sườn xuống
   bằng `k1 && !k1_prev`, mà `k1_prev` khởi tạo là `false`. Nút đang bị giữ khi
   vào tới vòng lặp → vòng đầu tiên thấy "sườn xuống" giả và chạy thử động cơ,
   đúng lúc tay người dùng đang đặt trên bo mạch. Đã chặn thêm `!usb_msc_active()`.
2. **`USBD_static_malloc()` cấp phát thiếu chỗ.** Bản CubeMX sinh cấp đúng
   `sizeof(USBD_CDC_HandleTypeDef)`, trong khi handle MSC lớn hơn (riêng
   `bot_data[]` đã 512 byte). Để nguyên thì lớp MSC ghi tràn ra ngoài mảng
   static và đâm thẳng vào biến kế bên. Đã đổi thành lấy kích thước lớn hơn
   trong hai, tính lúc biên dịch.

> ⚠️ `USB_DEVICE/Target/usbd_conf.c` là **file CubeMX sinh**. Chỗ sửa trên
> **phải đặt lại sau mỗi lần Generate Code**. Trong file đã ghi rõ `SUA TAY`.

### File đã đụng tới

| File | Việc |
|---|---|
| `App/Storage/blackbox.c/.h` | ghi log; `FATFS`/`FIL` đặt trong `.dma_buffer` |
| `App/Storage/usb_msc.c/.h` | lớp ghép MSC ↔ thẻ SD, có bộ đệm trung chuyển AXI SRAM |
| `USB_DEVICE/App/usb_device.c` | xung HSI48+CRS, chọn lớp MSC/CDC theo K1 (trong `USER CODE`) |
| `USB_DEVICE/Target/usbd_conf.c` | ~~sửa tay~~ — **đã gỡ bỏ**, xem mục cuối tài liệu |
| `Core/Src/main.c` | mount sớm; chế độ MSC dừng trước cảm biến; chặn K1 |
| `App/State/fc_state.c/.h`, `App/Control/arming.c` | cờ `ARM_BLOCK_USB_MSC` |
| `Middlewares/.../Class/MSC/` | chép từ gói H7 V1.12.1 (`usbd_core.c` giống hệt từng byte) |
| `cmake/stm32cubemx/CMakeLists.txt` | thêm nguồn và include của MSC |

### Còn phải làm

- Chưa ghi được log của một chuyến bay thật — mới chỉ xác nhận dòng tiêu đề
  xuống thẻ. Cần một lần ARM → bay → DISARM để kiểm tra đường xả bộ đệm.
- `BB_BUFFER_BYTES` 256 KB ở 100 Hz với bản ghi 48 byte cho khoảng **54 giây**.
  Bay lâu hơn thì `blackbox_dropped()` sẽ khác 0. Cần đo xem một chuyến bay
  thật dài bao nhiêu rồi mới quyết có phải hạ `BB_RATE_HZ` hay không.

### Đã thử Generate Code lại — kết quả thật (27/08/2026)

Người dùng bật HSI48 trong CubeMX rồi Generate Code. Đo lại toàn bộ:

| Chỗ sửa | Kết quả thật |
|---|---|
| `App/**` | ✅ nguyên vẹn |
| `CMakeLists.txt` gốc (khai báo nguồn MSC) | ✅ nguyên vẹn |
| `Core/Src/main.c` — vùng `USER CODE` | ✅ nguyên vẹn |
| `USB_DEVICE/App/usb_device.c` | ✅ **không đổi một byte** |
| `usbd_conf.c` — include trong `USER CODE` | ✅ nguyên vẹn |
| `usbd_conf.c` — `USBD_static_malloc()` | ❌ bị trả về bản CubeMX → **đã xử lý dứt điểm, xem mục dưới** |
| `Middlewares/.../Class/MSC/` | ✅ CubeMX không xoá |
| `cmake/stm32cubemx/CMakeLists.txt` | ✅ không còn gì của mình ở đó |

Tức là dự đoán đúng hoàn toàn: **chỉ một mục phải đặt lại**. Đã đặt lại và
nạp kiểm tra — blackbox mount OK (file số 9), CDC lên `COM17`.

`Core/Src/main.c` hiện ra 267 dòng thay đổi nhưng gần như toàn bộ là CubeMX
định dạng lại comment và khoảng trắng theo kiểu của nó. Thay đổi chức năng
đúng hai dòng:

```c
RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
RCC_OscInitStruct.HSI48State     = RCC_HSI48_ON;
```

⚠️ Lưu ý: CubeMX bật HSI48 **nhưng không** phát mã đặt bộ mux xung USB —
`PeriphClockSelection` vẫn chỉ có `RCC_PERIPHCLK_ADC`, dù trong `.ioc` đã có
`RCC.USBCLockSelection=RCC_USBCLKSOURCE_HSI48`. Nghĩa là khối cấp xung trong
vùng `USER CODE` của `MX_USB_DEVICE_Init()` **vẫn đang gánh việc**, không phải
đồ thừa. Đừng xoá nó.

### Gỡ nốt chỗ sửa tay cuối cùng — không còn gì phải nhớ (27/08/2026)

Mọi kết luận ở trên về việc "phải đặt lại `USBD_static_malloc()` sau mỗi lần
Generate Code" **đã hết hiệu lực**. Chỗ sửa đó đã bị gỡ bỏ hoàn toàn.

Lần Generate Code vừa rồi cho một dữ kiện mới: `Middlewares/.../Class/MSC/`
**nguyên vẹn từng byte**. CubeMX không hề đụng tới nó, vì trong `.ioc`
`USB_DEVICE.CLASS_NAME_FS` vẫn là `CDC` nên nó không biết lớp MSC tồn tại.

Nghĩa là thư mục đó giờ thuộc về mình, và đó mới là chỗ đúng để sửa.

Gốc rễ của vấn đề là `usbd_msc.c` gọi `USBD_malloc()`, mà macro đó trỏ tới
`USBD_static_malloc()` bên `usbd_conf.c` — file CubeMX sinh. Thay vì đi nới
cái bộ cấp phát dùng chung ấy, cho MSC **bộ nhớ tĩnh của riêng nó** ngay trong
`usbd_msc.c`:

```c
static USBD_MSC_BOT_HandleTypeDef s_msc_handle;
hmsc = &s_msc_handle;
```

và bỏ lời gọi `USBD_free()` tương ứng ở `USBD_MSC_DeInit()`.

Không mất mát gì: `USBD_static_malloc()` bản gốc cũng chỉ trả về một vùng tĩnh
đúng như vậy, và chỉ có một thiết bị USB nên một handle là đủ.

Sau đó `USB_DEVICE/Target/usbd_conf.c` được trả về **đúng nguyên bản CubeMX**,
không còn dòng sửa tay nào.

**Đã kiểm chứng trên phần cứng sau khi đổi:**

| | |
|---|---|
| Chế độ MSC | đĩa `FCH743 Blackbox SD` 7,95 GB Online, đọc được `LOG0000..LOG0009` |
| Chế độ bay | CDC lên `COM17`, blackbox mount, mở `LOG0010.CSV` |
| `usbd_conf.c` | 0 chỗ sửa tay |

> Bài học: khi buộc phải sửa file do công cụ sinh ra, hãy tìm xem có file nào
> **công cụ không quản lý** mà đạt được cùng mục đích không. Ở đây thư mục
> middleware chép tay chính là chỗ đó.

**Hệ quả cho quy trình:** Generate Code lại bây giờ **không cần làm gì thêm**.
Nhưng vẫn giữ nguyên hai điều kiện:

1. `.ioc` phải giữ `USB_DEVICE.CLASS_NAME_FS = CDC`. Đổi sang MSC thì CubeMX
   sẽ chiếm lại thư mục `Class/MSC/`, ghi đè chỗ sửa trên, bỏ CDC khỏi build
   và sinh `usbd_storage_if.c` trùng tên với `App/Storage/usb_msc.c`.
2. Đừng xoá khối cấp xung HSI48 trong vùng `USER CODE` của
   `MX_USB_DEVICE_Init()` — CubeMX vẫn không phát mã đặt bộ mux xung USB.

---

## Thẻ SD hỏng làm treo cả bộ điều khiển bay 🔴 (27/08/2026)

Triệu chứng người dùng báo: bo mạch **im hoàn toàn**, không UART, không USB,
và **bấm reset không cứu được**.

### Gốc rễ

CubeMX sinh ra trong `MX_SDMMC1_SD_Init()`:

```c
if (HAL_SD_Init(&hsd1) != HAL_OK) { Error_Handler(); }
```

và `Error_Handler()` nguyên bản là:

```c
__disable_irq();
while (1) { }
```

Tắt ngắt rồi quay vòng vĩnh viễn. `MX_SDMMC1_SD_Init()` đứng ở dòng 179, tức
**trước** cả `MX_USB_DEVICE_Init()` và trước `dbg_console_init()`. Nên khi thẻ
lỗi thì:

- không dòng UART nào kịp ra → nhìn như chip chết,
- USB không enumerate → không cắm máy tính kiểm tra được,
- bấm reset lại chạy đúng vào chỗ đó → không thoát được.

Ghi log ra thẻ chỉ là **phụ kiện**. Máy bay phải bay được khi không có thẻ,
khi thẻ hỏng, hay khi quên cắm thẻ.

### Cách chữa

Đặt cờ quanh lời gọi, ngay trong hai vùng `USER CODE` mà CubeMX chừa sẵn:

```c
/* USER CODE BEGIN SDMMC1_Init 1 */
g_sd_init_in_progress = true;
/* USER CODE END SDMMC1_Init 1 */
    ...
/* USER CODE BEGIN SDMMC1_Init 2 */
g_sd_init_in_progress = false;
/* USER CODE END SDMMC1_Init 2 */
```

rồi trong `Error_Handler()` (cũng nằm trong `USER CODE`):

```c
if (g_sd_init_in_progress) {
    g_sd_init_in_progress = false;
    g_sd_init_failed      = true;
    return;                 /* thẻ hỏng thì bay tiếp, không treo */
}
__disable_irq();
while (1) { }               /* mọi lỗi khác vẫn treo — có chủ ý */
```

Mọi lỗi khác **vẫn treo như cũ**, và đó là có chủ ý: một ngoại vi bắt buộc
hỏng mà vẫn cho bay mới là nguy hiểm.

Cả ba chỗ đều trong vùng `USER CODE` nên Generate Code lại không mất.

### Đã đo được sau khi sửa

Với thẻ đang ở trạng thái kẹt thật, bo khởi động trọn vẹn:

```
THE SD: KHONG KHOI TAO DUOC
ICM20602: OK          LSM6DSV: OK, WHO_AM_I 0x70
BMP388: OK            MTF-01P / ELRS / DShot300 / Telemetry: OK
console: TAT / khong mount duoc the
```

### Vì sao thẻ kẹt

Nhiều khả năng do nạp lại firmware / bấm reset **trong khi Windows còn đang
gắn ổ đĩa** ở chế độ MSC — dữ liệu ghi còn nằm trong bộ đệm, thẻ đang dở một
thao tác thì mất lệnh.

Reset **không** gỡ được, vì reset không cắt điện cho thẻ. Phải **rút hẳn nguồn**.

Đã thêm cảnh báo vào banner chế độ MSC: eject ổ đĩa trong Windows trước, rồi
mới rút USB và reset.

### Một bài học về công cụ đo, không phải về firmware

Trong lúc lần ra lỗi này tôi đã kết luận nhầm "bo mạch chết" vì UART im 0 byte
qua nhiều lần thử. Thực ra lõi đang bị **debugger giữ dừng**: đọc `DHCSR`
(0xE000EDF0) ra `0x00030003`, tức `C_DEBUGEN` và `C_HALT` đều bằng 1. Chuỗi
lệnh `STM32_Programmer_CLI ... mode=UR` để lại lõi ở trạng thái halt.

`STM32_Programmer_CLI -c port=SWD mode=HOTPLUG -g` thả lõi ra chạy.

Đọc `CFSR`/`HFSR` (0xE000ED28) cũng đáng làm sớm: `CFSR = 0` loại ngay giả
thuyết hard fault, đỡ mất công đi tìm sai hướng.

---

## Giữ độ cao và giữ vị trí — 28/08/2026

### Trạng thái trước khi làm

`throttle_cmd` được ghi đúng **một chỗ** trong toàn bộ chuỗi điều khiển:
`ctrl_angle.c` lấy thẳng từ cần, ở mọi chế độ kể cả POSHOLD. Và POSHOLD giữ
**vận tốc** chứ không giữ **vị trí** — sai số vận tốc dù nhỏ vẫn tích luỹ
thành trôi, không có gì kéo máy bay về chỗ cũ.

`FLIGHT_MODE_ALTHOLD` đã có trong enum và có tên, nhưng không ai chọn và không
ai cài đặt. `position_m.x/.y` chưa từng được ghi, `position_valid` đặt cứng
`false`.

### Đã làm

**Giữ độ cao** — `App/Control/ctrl_althold.c/.h`, hai vòng lồng nhau giống hệt
cấu trúc vòng tư thế:

```
cần ga ──> tốc độ lên mong muốn ──> PID ──> ga
                 ▲                            │
       (P trên sai số độ cao)                 │
                 └── độ cao, tốc độ lên ──────┘
```

Điểm đáng chú ý trong thiết kế:

- **Ga treo là số hạng nuôi tiến.** Không có nó thì tích phân phải tự dựng
  toàn bộ lực nâng, mất vài giây — trong đó máy bay rơi.
- **Chuyển vào mượt.** Lúc vào chế độ, tích phân được nạp sao cho đầu ra bằng
  ĐÚNG mức ga người lái đang giữ. Thiếu bước này thì đầu ra nhảy về ga treo
  mặc định ngay khoảnh khắc gạt công tắc.
- **Mốc độ cao bám theo khi đang đẩy cần.** Không bám thì lên cao 5 m rồi thả
  tay là máy bay lao ngược về chỗ cũ.
- **D lấy trên số đo, lọc 10 Hz.** Tốc độ lên suy từ baro nên vốn chậm và ồn.
- **Sàn ga khác 0** (0,10). Để 0 thì một lần ước lượng độ cao sai có thể cắt
  hẳn motor giữa không trung.

**Giữ vị trí** — thêm vòng ngoài cùng vào `ctrl_poshold.c`:

```
vị trí ──> vận tốc ──> góc nghiêng ──> tốc độ góc ──> motor
```

`estimator.c` giờ tích phân vận tốc thành `position_m.x/.y`. Ý nghĩa của
`position_valid` được định nghĩa rõ: **"dùng được để giữ chỗ trong thời gian
ngắn"**, KHÔNG phải "biết mình đang ở đâu" — đây là dẫn đường suy tính thuần
tuý, sai số không bao giờ tự hết, tuyệt đối không dùng để bay về nhà.

Buông cần thì chốt mốc; cầm lái thì mốc bám theo chỗ hiện tại.

**POSHOLD nay bao gồm giữ độ cao.** Giữ được vị trí ngang mà độ cao vẫn phải
rà tay thì mới xong một nửa việc.

### 🔴 Một lỗi an toàn tìm thấy trong lúc làm

`ctrl_poshold_reset()` chỉ được gọi khi ĐỔI chế độ, không gọi khi disarm.
Nhưng `ctrl_angle_update()` chạy bất kể đã arm hay chưa.

Hậu quả đo được: máy bay nằm trên bàn ở chế độ POSHOLD, tích phân vận tốc dồn
tới **I_y = −0,98 độ** chỉ sau ít phút, trần là ±8 độ. Arm lúc đó là máy bay
nghiêng ngay theo một lệnh tích cóp từ khi còn nằm im.

`ctrl_rate.c` đã có chốt này từ đầu, `ctrl_althold.c` mới viết cũng có — riêng
poshold thiếu. Đã thêm. Đo lại: `I_x`/`I_y` giữ nguyên 0 sau hơn 12 giây.

### Tham số mới (13)

12 cho ALTHOLD, 1 cho vòng vị trí (`poshold_pos_kp`). `PARAM_SCHEMA_VERSION`
lên 3. Đã kiểm bằng lệnh `diff`: không tham số nào đang khác mặc định nên
việc đổi bảng **không làm mất cấu hình nào**.

Nấc công tắc ALTHOLD riêng **mặc định TẮT** (`rc_mode_althold_threshold = 2000`,
ngoài dải CRSF): công tắc ba nấc đã dùng hết chỗ, mà POSHOLD đã bao gồm giữ
độ cao rồi.

### Đã kiểm trên bàn

| | |
|---|---|
| `mode 25` (ALTHOLD) | `alt = 0,19`, `valid = co`, mốc/tích phân = 0 khi chưa arm ✓ |
| `mode 13` (POSHOLD) | cột `dN`/`dE`/`giu` chạy; vòng ngoài sinh lệnh vận tốc để bò về mốc ✓ |
| CLI | `get poshold_pos_kp` → 1, `get althold_hover_thr` → 0,35 ✓ |

### CÒN PHẢI LÀM TRƯỚC KHI BAY

1. **Đo `althold_hover_thr` trên chính máy bay này.** Treo ở chế độ ANGLE, đọc
   cột `thr`. ĐỪNG đoán — sai 10 % là tích phân phải bù 10 %, mất vài giây.
2. Chỉnh theo đúng thứ tự: `althold_climb_kp` → `_ki` → `althold_alt_kp` →
   `poshold_pos_kp`. Vòng ngoài chỉnh sau cùng và giữ thấp.
3. **Thẻ SD đang tắt** (`FC_SD_ENABLE = 0`) nên chưa ghi blackbox được — mà
   chỉnh hai vòng này thì rất cần log. Nên xử lý thẻ trước.
4. Theo dõi `loop_overruns`: đang là 993/giây với `loop_max_us = 240`. Vòng
   giữ độ cao chạy ở nhịp 1 kHz cùng vòng góc nên có thêm tải.

---

## Trôi yaw: bộ lọc tự chế ra 99,7 % lượng trôi — 28/08/2026

### Triệu chứng

Máy bay giữ cố định trên giá, roll và pitch đứng yên, nhưng yaw trôi tới
**180 độ mỗi phút**. Con số báo về giữa các lần thử lại khác nhau: 180, rồi
9,45, rồi 218.

### Chuỗi đo dẫn tới nguyên nhân

| Đại lượng | Đo được | Quy ra độ/phút |
|---|---|---|
| Con quay thật `gz`, 508 mẫu, đã trừ bias driver | +0,011 °/s | **+0,7** |
| Bias yaw EKF **tự ước lượng** `bgz` | 3,610 °/s | **−216,6** |
| Trôi yaw quan sát, 2262 mẫu / 49 s | | **−218,2** |

Hai dòng cuối lệch nhau **0,7 %**. Cảm biến đóng góp 0,3 %; phần còn lại do
bộ lọc tự tạo ra.

Phép đo lặp lại hai mươi phút sau cho `bgz = 0,12` và trôi 9,45 độ/phút —
**vẫn đúng tỉ lệ trôi ≈ bgz × 60**. Chính sự thất thường đó là chữ ký của một
bước ngẫu nhiên không bị ràng buộc, và là lý do các lần đo trước không khớp
nhau.

### Nguyên nhân

Gia tốc kế đo hướng trọng lực. Xoay quanh trục thẳng đứng **không làm đổi
hướng trọng lực**, nên phép đo này không mang một chút thông tin nào về yaw —
ma trận H hạng 2, nhân không gian nằm đúng dọc trục yaw.

Trạng thái bias yaw vì thế **không quan sát được**. Nó chỉ nhúc nhích qua hiệp
phương sai chéo với sai số góc, và thứ đẩy nó khi máy bay nằm im chính là
nhiễu của gia tốc kế. Một bước ngẫu nhiên không có lực kéo về.

Rồi `ekf_attitude.c` trừ thẳng nó khỏi con quay:

```c
gyro_dps.z * FC_DEG_TO_RAD - s_bias.z
```

Cái chặn duy nhất là `EST_GYRO_BIAS_MAX_DPS = 10.0` — rộng hơn bias thật
**900 lần**, nên nó chưa bao giờ chạm tới.

### Cách chữa

Đóng băng trạng thái bias yaw. Ba chỗ, đều trong `ekf_attitude.c`:

1. **Không bơm nhiễu quá trình** vào `P[5][5]` — bơm nhiễu vào một trạng thái
   không phép đo nào chạm tới nghĩa là bảo bộ lọc "ta ngày càng không biết nó
   bằng bao nhiêu", và độ lợi Kalman phình theo.
2. **Không áp dụng `dx[5]`**.
3. **Xoá hàng và cột 5 của ma trận hiệp phương sai.** Chỉ bỏ qua `dx[5]` là
   chưa đủ: các số hạng chéo `P[5][k]` vẫn để trạng thái đó bóp méo phép cập
   nhật của những trạng thái khác.

Roll và pitch **không** bị đóng băng — trọng lực quan sát được hai trục đó nên
ước lượng bias ở đấy là đúng đắn và có ích.

Điều khiển bằng tham số `est_yaw_bias_learn` (mặc định 0). Lắp từ kế vào thì
bias yaw trở nên quan sát được, lúc đó `set est_yaw_bias_learn=1`.

### Kết quả đo sau khi sửa

Máy giữ cố định, 5009 mẫu trong 100 giây:

```
  t(s)     d_yaw  do/ph(20s)      bgz
   0.0     -0.25        0.00   0.0000
  20.0     -0.64       -1.64   0.0000
  50.0     -0.53       -0.37   0.0000
  80.0     -1.42       -2.42   0.0000
 100.0     -0.56        1.57   0.0000
```

- `bgz` đứng yên tuyệt đối ở **0,0000**, biên độ 0
- Từ giây 20 tới giây 100: **0,08 độ trong 80 giây = 0,06 độ/phút**
- Tốc độ tức thời dao động **cả hai dấu** quanh 0 (−2,4 tới +1,6)

Dấu đổi chiều là điều quan trọng nhất: trước khi sửa, tốc độ luôn một dấu và
gần như không đổi (−216,6 tới −219,7). Giờ nó tản đều quanh không. **Thành
phần trôi hệ thống đã biến mất**, còn lại chỉ là bước ngẫu nhiên của nhiễu con
quay.

**218 → 0,06 độ/phút, tức tốt hơn khoảng 3600 lần.**

### Công cụ đo đi kèm

Thêm hai cột `d_yaw` và `do/ph` vào `DBG_MODE_EST`, kèm lệnh CLI `yawzero` để
chốt mốc.

Hai chi tiết trong cách làm:

- **Cộng dồn từng bước có gỡ gói**, không trừ hai góc. Yaw do `atan2` sinh ra
  nên nằm trong ±180°; trừ thẳng thì trôi quá nửa vòng sẽ đọc ra số âm nhỏ dần
  thay vì số dương lớn dần.
- **Tốc độ tính trên cửa sổ trượt 20 giây**, không phải tổng chia thời gian.
  Đã gặp thật: người dùng xoay drone 18° lúc bắt đầu đo, và cách tính cũ để cú
  xoay đó nằm mãi trong tử số — tổng phẳng lì suốt hai phút mà cột tốc độ vẫn
  đọc −8,8 độ/phút.

Toàn bộ nằm ở tầng console, **không thêm một dòng nào vào vòng 1 kHz hay
4 kHz** — dụng cụ đo không được làm chậm thứ nó đang đo.

---

## Bỏ ACRO khỏi công tắc, chia lại ba nấc — 28/08/2026

### ACRO có hai vai trò, chỉ bỏ một

**Nấc công tắc — đã gỡ.** `read_mode_switch()` giờ đòi thêm cờ
`rc_mode_acro_enable`, mặc định 0. Ở ACRO máy bay không tự cân bằng: buông cần
là nó giữ nguyên góc nghiêng và tiếp tục lật. Gạt nhầm giữa chuyến bay thì chỉ
có vài giây để nhận ra.

**Chế độ dự phòng tự động — GIỮ NGUYÊN.** Khi bộ ước lượng mất góc tin cậy,
`ctrl_angle.c` vẫn tụt về ACRO. Đó là lựa chọn đúng, vì ACRO là chế độ duy
nhất chạy được mà không cần biết góc — bắt nó bay ANGLE với một góc sai còn
nguy hiểm hơn nhiều.

Kiểm chứng: đặt ngưỡng ACRO xuống 582 trong khi kênh đang 992, tức kênh **đã
vượt ngưỡng**.

| | Kết quả |
|---|---|
| `acro_enable = 0` | POSHOLD — bị chặn đúng |
| `acro_enable = 1` | ACRO — đường cũ vẫn nguyên |

### Ba nấc mới

```
nấc THẤP  (172)  ->  ANGLE     tự cân bằng, ga bằng tay
nấc GIỮA  (992)  ->  ALTHOLD   thêm giữ độ cao
nấc CAO  (1811)  ->  POSHOLD   thêm giữ vị trí ngang
```

Ngưỡng tính từ hằng số CRSF nên đổi dải là ngưỡng tự theo:
`ALTHOLD = (MIN+MID)/2 = 582`, `POSHOLD = (MID+MAX)/2 = 1401`.

Thứ tự giữ nguyên nguyên tắc an toàn cũ: kênh mất tín hiệu hay chưa gán đều
cho giá trị thấp, và giá trị thấp rơi vào chế độ cần ít cảm biến nhất.

Ngưỡng ACRO để trùng 1401 là có ý — ai cố ý bật lên thì nấc CAO đổi từ POSHOLD
thành ACRO, vì thang chọn xét ACRO trước. Hợp lý cho người biết mình làm gì.

Kiểm chứng cả ba nhánh bằng cách dịch ngưỡng quanh kênh cố định 992:

| Cấu hình | Chế độ ra |
|---|---|
| althold=582, poshold=1401 (thật) | ALTHOLD |
| althold=1200 → 992 dưới ngưỡng | ANGLE |
| poshold=900 → 992 trên ngưỡng | POSHOLD |

### Còn tồn

Nhãn `[DU PHONG - chua co goc]` hiện ra cả khi nguyên nhân là **mất flow**.
Hai nguyên nhân khác hẳn nhau dùng chung một dòng chữ — chưa sửa.
