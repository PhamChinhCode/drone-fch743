# Driver la bàn rời trên I2C1 (dùng chung bus với BMP388)

Module từ kế nhãn **HMC5883L** đấu vào **I2C1** — đúng bus mà BMP388 đang
dùng (PB8 = SCL, PB7 = SDA, 400 kHz).

> ✅ **ĐÃ HIỆN THỰC VÀ ĐÃ KIỂM TRÊN PHẦN CỨNG** (2026-09-03). Chip thật hoá
> ra **KHÔNG phải HMC5883L mà là QMC5883P @ 0x2C** — xem mục "Kết quả
> triển khai" ở cuối. GĐ0–GĐ3 xong; còn GĐ4 (hiệu chuẩn) vì phải xoay
> máy bay bằng tay.

---

## Phát hiện quan trọng: hạ tầng từ kế ĐÃ CÓ SẴN

Đây không phải việc viết từ đầu. Giai đoạn 3 và 4 của
[KE_HOACH_LSM6DSV.md](KE_HOACH_LSM6DSV.md) đã dựng xong toàn bộ phần phía sau
tầng driver, và cố tình dựng theo kiểu **không phụ thuộc nguồn dữ liệu**:

| Thứ đã có | Ở đâu | Sửa gì |
|---|---|---|
| `mag_data_t` (raw / raw_gauss / field_gauss / magnitude) | `App/State/fc_state.h:201` | không |
| `g_fc.mag` | `App/State/fc_state.h:339` | không |
| Bit `SENSOR_MAG` + kiểm tra sống/chết | `fc_state.h:68`, `fc_state.c:121` | không |
| `MAG_TIMEOUT_MS` | `fc_config.h:854` | không |
| Bộ chọn nguồn `MAG_SOURCE_NONE/SHUB/I2C` | `fc_config.h:735` | đổi giá trị |
| 20 tham số runtime `mag_*` | `param_list.h:165-300` | nới 1 dải, xem GĐ0 |
| Hiệu chuẩn sắt cứng/mềm trong **hệ cảm biến** | công thức ở `lsm6dsv.c:1355` | bê sang |
| `DBG_MODE_MAGCAL` — công cụ hiệu chuẩn, đếm phủ 8 góc, chốt bằng nút K1 | `dbg_console.c:2345` | **không** — nó chỉ đọc `g_fc.mag.raw_gauss` |
| `DBG_MODE_MAG` — bảng theo dõi | `dbg_console.c:1956` | sửa cột phụ, xem GĐ3 |

`MAG_SOURCE_I2C = 2` đã được đặt tên và đăng ký sẵn trong bảng tham số từ
2026-08-26, chỉ chưa có ai hiện thực. **Việc cần làm đúng là điền vào chỗ
trống đó**, không phải mở đường mới.

> ⚠️ Một chỗ trong tài liệu cần sửa: chú thích ở `fc_config.h:736` viết
> `MAG_SOURCE_I2C` sẽ dùng **I2C2 (PB10/PB11)**. Thực tế bạn đã đấu vào
> **I2C1**. Phải sửa chú thích, nếu không lần gỡ lỗi sau sẽ đi soi sai bus.

---

## Việc thật sự khó: hai thiết bị trên một bus I2C1

Đây là phần duy nhất có rủi ro kiến trúc. Ba phần còn lại chỉ là gõ code.

### Hiện trạng

`bmp388.c` coi I2C1 là của riêng nó, và hai callback phân phối vô điều kiện:

```
HAL_I2C_MemRxCpltCallback(hi2c)          drv_hal_callbacks.c:119
    if (hi2c->Instance == I2C1) -> bmp388_i2c_complete_isr();
                                   ^ "Chưa có thiết bị I2C nào khác trên bus này"

HAL_I2C_ErrorCallback(hi2c)              drv_hal_callbacks.c:127
    if (hi2c->Instance == I2C1) -> bmp388_i2c_error_isr();
```

Nếu thêm từ kế mà không sửa chỗ này: mỗi lượt đọc từ kế xong sẽ đi gọi
`bmp388_i2c_complete_isr()`, đặt `s_new_raw = true`, và BMP388 sẽ **giải mã
6 byte từ trường thành áp suất**. Baro chết một cách rất khó tìm.

### Thiết kế đề xuất — phân phối theo địa chỉ, KHÔNG cần module trọng tài

Hai điều kiện đã kiểm trong chính mã HAL của project:

1. `HAL_I2C_Mem_Read_IT()` lưu `hi2c->Devaddress = DevAddress`
   (`stm32h7xx_hal_i2c.c:2916`), và trong cả file **không có chỗ nào xoá nó**
   (chỉ 4 chỗ ghi, đều là lúc bắt đầu truyền). → còn đúng lúc callback chạy.
2. `HAL_I2C_Mem_Read_IT()` trả `HAL_BUSY` nếu `hi2c->State != HAL_I2C_STATE_READY`.
   → bản thân HAL đã là cơ chế loại trừ.

Nhờ vậy:

```
  ISR phân phối theo hi2c->Devaddress:
      == BMP_I2C_ADDR   -> bmp388_i2c_complete_isr()
      == s_mag_hal_addr -> mag_i2c_complete_isr()

  Khởi phát: cả hai driver đều gọi từ VÒNG LẶP CHÍNH (một luồng, không
  giành nhau). Ai gọi trước thì HAL nhận; người sau nhận HAL_BUSY và
  bỏ lượt, thử lại vòng sau.
```

**Không thêm file `i2c1_bus.c`, không thêm token sở hữu.** Lý do: nguồn tranh
chấp duy nhất là ISR *kết thúc* một lượt truyền, còn việc *bắt đầu* thì cả hai
đều ở main loop nên không có preemption giữa lúc kiểm và lúc gọi. `HAL_BUSY`
là đủ. (Khác hẳn `bus_acquire()` của SPI3 trong `lsm6dsv.c` — ở đó DMA của IMU
khởi phát từ **ISR DRDY** nên bắt buộc phải có khoá.)

Ngân sách bus, ở 400 kHz:

| | nhịp | byte/lượt | thời gian bus | IRQ/s (IT mode, ~1 IRQ/byte) |
|---|---|---|---|---|
| BMP388 (đang chạy) | 100 Hz | 8 | 2,8% | ~1100 |
| Từ kế (đề xuất) | 50 Hz | 7 | 1,3% | ~500 |

Tổng ~4% duty và ~1600 IRQ/s ở ưu tiên 5 (`stm32h7xx_hal_msp.c:242`) — thấp
hơn gyro (0) và DShot (1) nên không đụng đường điều khiển. Phải **đo lại cột
`lmax`** trong `DBG_MODE_STATUS` để xác nhận, chứ không tin bảng này.

---

## Bẫy phần cứng: nhãn "HMC5883L" thường KHÔNG phải HMC5883L

Đã chốt: **driver dò cả hai chip.** Hai chip khác nhau hoàn toàn, không có
thanh ghi nào trùng nghĩa:

| | HMC5883L (Honeywell) | QMC5883L (clone, GY-271) |
|---|---|---|
| Địa chỉ 7 bit | **0x1E** | **0x0D** |
| Nhận dạng | 0x0A..0x0C = `'H','4','3'` | 0x0D = 0xFF |
| Dữ liệu | 0x03..0x08, **MSB trước**, thứ tự **X, Z, Y** | 0x00..0x05, **LSB trước**, thứ tự **X, Y, Z** |
| Status | 0x09, bit0 RDY | 0x06, bit0 DRDY, bit1 OVL |
| Cấu hình | 0x00 CONFIG_A, 0x01 CONFIG_B (gain), 0x02 MODE | 0x09 CTRL1 (mode+ODR+RNG+OSR), 0x0A CTRL2 |
| Soft reset | **không có** | 0x0A bit7 |
| Bắt buộc riêng | — | **phải ghi 0x01 vào 0x0B** (SET/RESET period), thiếu là số trôi |
| Thang | gain 8 mức, 1370..230 LSB/G | ±2 G = 12000 LSB/G, ±8 G = 3000 LSB/G |
| Báo bão hoà | mã −4096 trên trục tràn | bit OVL |

Ghi chú về nhận dạng: **địa chỉ mới là dấu hiệu chính**, không phải chip ID.
ID của QMC5883L là `0xFF` — cũng đúng bằng thứ đọc được từ một bus chết, nên
một mình nó không chứng minh gì. Trình tự đúng là dò `IsDeviceReady(0x1E)`
trước, rồi `0x0D`, và chỉ dùng ID để xác nhận sau khi đã có ACK.

Không đụng BMP388 ở 0x77 — cả 0x1E và 0x0D đều rảnh.

---

## Bảng công việc

### GĐ0 — Sửa cấu hình và tham số (không có code chạy)

1. `fc_config.h`: sửa chú thích `MAG_SOURCE_I2C` từ "I2C2 PB10/PB11" thành
   "I2C1, dùng chung với BMP388". Thêm khối hằng số cho module rời
   (`MAG_I2C_UPDATE_RATE_HZ 50`, timeout, v.v.) — **đặt riêng**, không sửa
   khối QMC6309 vì `MAG_SOURCE_SHUB` vẫn phải biên dịch được.
2. `param_list.h:291`: `mag_range_g` đang chặn dải `8.0f .. 32.0f`. HMC5883L
   chỉ tới ±8,1 G và dải hợp cho la bàn là **±1,3 G**; QMC5883L chỉ có ±2 và
   ±8 G. → **nới min xuống `1.0f`**, mỗi driver tự làm tròn xuống mức hợp lệ
   gần nhất (đúng lối `osr_index()` của `bmp388.c` đang làm).
   Phải kiểm lại bảng tra `s_mag_lsb_per_gauss` trong `lsm6dsv.c` còn xử lý
   đúng khi gặp giá trị mới.
3. `mag_update_rate_hz`: mặc định 10 Hz là do chu kỳ sensor hub 90 ms bắt
   buộc. Bus riêng không có ràng buộc đó → mặc định **50 Hz** cho nguồn I2C.

→ **kiểm:** build sạch với cả `MAG_SOURCE_NONE` và `MAG_SOURCE_SHUB`;
`get mag_range_g` in đúng dải mới.

### GĐ1 — Bản đồ thanh ghi

Thêm `App/Drivers/hmc5883.h` + `App/Drivers/qmc5883.h`, hoặc một file
`mag_i2c_regs.h` chứa cả hai khối. Chỉ hằng số, không code — đúng khuôn
`qmc6309.h` đang có.

→ **kiểm:** so từng hằng số với datasheet, ghi rõ nguồn vào đầu file (đúng
như `qmc6309.h` đã ghi rõ chỗ thư viện SensorLib đặt sai bit).

### GĐ2 — Driver `App/Drivers/mag_i2c.c` + `.h`

Khuôn theo `bmp388.c` vì cùng bus, cùng kiểu ngắt:

```
mag_i2c_init()          chặn, chỉ gọi lúc khởi động (như bmp388_init)
    HAL_Delay(5)
    dò 0x1E -> đọc 0x0A..0x0C, khớp 'H43'  -> s_variant = HMC5883L
    dò 0x0D -> đọc 0x0D                     -> s_variant = QMC5883L
    không ai ACK                            -> MAG_I2C_INIT_NACK, state ERROR
    ghi cấu hình bằng reg_write_verify (chế độ liên tục, gain, ODR)
    QMC: nhớ ghi 0x01 vào 0x0B

mag_i2c_update(now_us)  không chặn, gọi từ main loop
    xử lý mẫu cũ TRƯỚC   (giữ bất biến "s_buf chỉ có một chủ" của bmp388.c)
    quá hạn -> HAL_I2C_Master_Abort_IT
    tới nhịp && !busy -> HAL_I2C_Mem_Read_IT 7 byte; HAL_BUSY -> bỏ lượt
    quá MAG_TIMEOUT_MS không có mẫu -> healthy = false

mag_i2c_complete_isr() / mag_i2c_error_isr()    ngắn như của bmp388
```

Đường xử lý mẫu **bê nguyên công thức** từ `lsm6dsv.c:1345-1380`: đổi thang →
hiệu chuẩn sắt cứng rồi sắt mềm **trong hệ cảm biến** → mới xoay sang hệ thân.
Thứ tự này là có lý do (sửa `MAG_AXIS_*` về sau không làm hỏng bộ số hiệu
chuẩn) và phải giữ, nếu không `DBG_MODE_MAGCAL` sẽ hiệu chuẩn sai hệ.

Khác biệt cần chú ý so với QMC6309: HMC5883L trả **MSB trước** và thứ tự trục
là **X, Z, Y**. Đây là chỗ dễ sai nhất trong cả kế hoạch.

→ **kiểm:** variant in ra đúng lúc khởi động; `sample_count` tăng đều;
`error_count` đứng ở 0.

### GĐ3 — Nối vào hệ thống

1. `drv_hal_callbacks.c:119,127` — phân phối theo `hi2c->Devaddress` như trên.
   Sửa luôn chú thích "Chưa có thiết bị I2C nào khác trên bus này".
2. `Core/Src/main.c:545` — gọi `mag_i2c_init()` **sau** `bmp388_init()`
   (lúc đó BMP388 ở `RUNNING` nhưng chưa phát lượt đọc nào, bus rảnh).
   Bọc trong `#if MAG_SOURCE == MAG_SOURCE_I2C`.
3. `Core/Src/main.c:803` — thêm `mag_i2c_update(micros())` cạnh
   `bmp388_update()`. Lưu ý khối `MAG_SOURCE_SHUB` hiện **nằm trong
   `#if IMU2_ENABLE`**; nhánh I2C phải ra NGOÀI khối đó vì nó không liên quan
   tới IMU phụ.
4. `dbg_console.c:1956` `emit_mag()` — sáu chỗ gọi hàm riêng của LSM
   (`lsm6dsv_mag_rate_hz/nacks/bus_busy/init_result_name/addr`) và một chỗ
   so cứng `chip_id != 0x90`. Tách bằng `#if MAG_SOURCE == ...` cho các cột
   phụ. **Không dựng tầng trừu tượng chung** cho hai hiện thực — `#if` tốn
   ~20 dòng, con trỏ hàm tốn nhiều hơn mà chẳng mua thêm gì.
5. `CMakeLists.txt` — **không cần sửa**, đã `GLOB_RECURSE CONFIGURE_DEPENDS`
   trên `App/*.c`.

→ **kiểm:** `DBG_MODE_BARO` — `press_pa` vẫn hợp lý, `stale` không nhảy vọt,
`err` = 0 (chứng minh phân phối ISR đúng, baro không ăn byte từ trường).
`DBG_MODE_STATUS` — `health` có thêm bit 8, `rhz` và `lmax` không xấu đi.
`DBG_MODE_MAG` — `Btot` trong khoảng **0,25..0,65 G** và **gần như không đổi
khi xoay máy bay theo mọi hướng**. Đây là cột quan trọng nhất.

### GĐ4 — Hiệu chuẩn lại

🔴 **Bắt buộc, không được bỏ.** Sáu hằng số ở `fc_config.h:838-843`
(`MAG_OFFSET_*`, `MAG_SCALE_*`) đo ngày 2026-08-26 **trên chip QMC6309 nằm
trên module LSM6DSV**. Chúng vô nghĩa với module mới: offset Y = 0,267 G là
lệch sắt cứng của *cái đế cũ*.

Tương tự, `MAG_AXIS_MAP_*`/`MAG_AXIS_SIGN_*` đang là ma trận đơn vị và chú
thích đã ghi rõ **"CHƯA ĐO"**.

Quy trình: đặt cả sáu hằng số về 0 / 1,0 và ma trận trục về đơn vị → chạy
`DBG_MODE_MAGCAL` (công cụ đã có, chốt bằng nút K1) → nạp bộ số mới → xác
định `MAG_AXIS_*` bằng cách so hướng mũi thật với `raw`.

→ **kiểm:** dao động `|B|` khi xoay mọi hướng **dưới 5%** (lần trước đạt 1,7%).

### Ngoài phạm vi lần này

Hợp nhất yaw từ từ kế vào EKF — đó là **giai đoạn 6** của kế hoạch LSM6DSV,
đụng vào `App/Estimator/` và `App/Control/` tức code đang bay. Tài liệu đó đã
chặn rõ: *"Từ kế chưa hiệu chuẩn TỆ HƠN là không có: nó kéo yaw sai một cách
tự tin, và EKF sẽ tin nó."* Không mở giai đoạn 6 trước khi GĐ4 ở trên xong.

---

## Rủi ro đã lường

| Rủi ro | Xử lý |
|---|---|
| Baro giải mã byte từ trường thành áp suất | GĐ3.1 — phân phối theo `Devaddress`; kiểm bằng cột `err`/`stale` của `DBG_MODE_BARO` |
| Module là QMC5883L chứ không phải HMC5883L | Dò cả hai địa chỉ, chọn bản đồ thanh ghi theo địa chỉ ACK |
| Đọc sai byte (MSB/LSB, thứ tự X-Z-Y) | Cột `Btot` phải nằm trong dải Trái Đất — sai byte thì con số vô lý ngay |
| Module GY-271 có điện trở kéo lên 4,7k trên bo, song song với điện trở đang có | Thường vẫn chạy ở 400 kHz; nếu lỗi thì tháo điện trở trên module |
| Thêm ~500 IRQ/s làm hỏng nhịp vòng lặp | Đo `lmax` trước/sau trong `DBG_MODE_STATUS`, không đoán |
| Dùng lại bộ hiệu chuẩn của chip cũ → yaw sai tự tin | GĐ4, và không bật giai đoạn 6 trước khi xong |

---

## Nhật ký

| Ngày | Nội dung |
|---|---|
| 2026-09-04 | **Đối chiếu toàn bộ 149 tham số giữa board và `fc_config.h`: khớp hoàn toàn, không phải sửa gì** (3 khác biệt chỉ là định dạng, 3 macro là biểu thức). **Xoá bộ hiệu chuẩn từ kế cũ** cả trong config lẫn trong flash của board — `Btot` từ 0,61 G về đúng 0,398 G. **Sửa khoá pha giữa hai bộ hỏi vòng I2C1**: chỉ đóng dấu mốc khi lượt đọc thật sự khởi phát, nên thua bus thì thử lại sau <1 ms thay vì mất trọn 20 ms — `dt_ms` từ 32–72 ms xuống 10–17 ms, `rhz` 3725 lên 3742. Đổi lại ý nghĩa cột `lost` (nay là số lần gặp bus bận, gồm cả thử lại — không phải số mẫu mất) và đã ghi rõ. Sửa hai dòng log từ kế mâu thuẫn; nhánh I2C nay cũng kiểm `g_params.mag_source`. Còn lại đúng phần hiệu chuẩn phải xoay máy bay bằng tay. |
| 2026-09-03 | **Hiện thực xong GĐ0–GĐ3 và kiểm trên phần cứng.** Chip thật là **QMC5883P @ 0x2C**, không phải HMC5883L (0x1E) hay QMC5883L (0x0D) — tìm ra bằng hàm quét bus mới `mag_i2c_scan_dump()`, với BMP388 @ 0x77 làm phép đối chứng chứng minh bus tốt. Thêm `qmc5883p.h` lấy từ driver ArduPilot (datasheet QST chỉ có bản scan), kèm ghi chú lỗi gõ dải đo của chính ArduPilot. Đo được độ lớn từ trường thô 0,336 G (đúng dải Trái Đất), `err`/`stale` = 0, khoảng 48,5 Hz, `SENSOR_MAG` bật, `lmax` 220 µs, baro không hỏng. **Phát hiện và sửa một defect do thay đổi này gây ra:** BMP388 đếm tranh chấp bus thành lỗi thật và chốt `FC_ERR_BARO_I2C` — nay đếm riêng qua `bmp388_bus_lost()`. Còn nợ GĐ4 (hiệu chuẩn, cần xoay máy bay bằng tay). |
| 2026-09-03 | Lập kế hoạch. Phát hiện hạ tầng từ kế đã dựng sẵn từ GĐ3/GĐ4 của kế hoạch LSM6DSV và `MAG_SOURCE_I2C` đã đăng ký nhưng chưa hiện thực → việc cần làm là điền chỗ trống, không mở đường mới. Chốt thiết kế chia sẻ bus I2C1 bằng cách phân phối ISR theo `hi2c->Devaddress` (đã kiểm HAL lưu và không xoá trường này) thay vì thêm module trọng tài. Chốt dò cả HMC5883L (0x1E) và QMC5883L (0x0D). Chưa viết dòng code nào. |

---

## Kết quả triển khai (2026-09-03)

### Chip thật: QMC5883P, không phải HMC5883L cũng không phải QMC5883L

Nạp bản đầu tiên (chỉ dò 0x1E và 0x0D) thì console báo:

```
Tu ke I2C: LOI - khong ai tra loi o 0x1E lan 0x0D
```

Nên đã thêm `mag_i2c_scan_dump()` — quét cả dải 0x08..0x77, đúng vai trò
`lsm6dsv_mag_dump()` ở nhánh SHUB. Kết quả quyết định:

```
  Quet bus I2C1 (0x08..0x77):
    tra loi @ 0x2C     <- co thiet bi
    tra loi @ 0x77     <- BMP388, PHEP DOI CHUNG: bus + dien tro keo len OK
    tong so thiet bi tra loi = 2
```

BMP388 trả lời chứng minh bus và điện trở kéo lên tốt, dây đấu đúng — vấn đề
chỉ là **địa chỉ 0x2C, tức QMC5883P**, chip thế hệ mới của QST. Đúng cái bẫy
kế hoạch đã lường, chỉ nặng hơn: nhãn "HMC5883L" không phải QMC5883L như
đoán, mà là chip thứ BA.

Đã thêm `App/Drivers/qmc5883p.h` và nhánh `init_qmc5883p()`. Driver giờ dò
**cả ba** địa chỉ 0x1E rồi 0x0D rồi 0x2C.

Bốn chip tên gần giống nhau, đừng nhầm:

| Chip | Địa chỉ | Chip ID | Bản đồ thanh ghi |
|---|---|---|---|
| **QMC5883P** | **0x2C** | 0x00 = **0x80** | `qmc5883p.h` — **đang dùng** |
| QMC5883L | 0x0D | 0x0D = 0xFF | `qmc5883.h` |
| QMC6309 | 0x7C | 0x00 = 0x90 | `qmc6309.h` (nguồn SHUB cũ) |
| HMC5883L | 0x1E | 0x0A..0x0C = 'H','4','3' | `hmc5883.h` |

### Nguồn bản đồ thanh ghi QMC5883P

Datasheet QST tải được là **ảnh scan tiếng Trung**, không trích được text để
đối chiếu từng bit. Nên lấy từ driver sản xuất của ArduPilot
(`AP_Compass_QMC5883P.cpp`) — code đã chạy thật trên phần cứng.

Ba chỗ dễ sai, đã ghi rõ trong `qmc5883p.h`:

1. **Ghi giá trị `0x29` vào thanh ghi `0x06`**, không phải ngược lại. Thanh
   ghi 0x06 vốn là byte cao trục Z nên nhìn code dễ tưởng ghi nhầm chỗ. Bắt
   buộc làm trước khi bật chế độ liên tục, để chốt dấu ba trục.
2. **ArduPilot có lỗi gõ** trong bảng dải đo của họ: `RNG_8G (0x10 << 2)` —
   dãy 0x00/0x01/0x10/0x11 rõ ràng có ý là NHỊ PHÂN nhưng viết dạng HEX, nên
   `0x10 << 2 = 0x40` tràn khỏi trường 2 bit. Họ không lộ lỗi vì không dùng
   define đó (lúc init ghi thẳng `CONF2 = 0x08`, tình cờ đúng bằng
   `0b10 << 2` = RNG 8 G). File này dùng nhị phân cho đúng.
3. **Chỉ dùng dải 8 G.** Đó là dải duy nhất có hệ số đổi thang đã kiểm chứng
   (3000 LSB/Gauss, khớp `range_scale = 1000/3000` của ArduPilot). Ba dải kia
   suy theo tỉ lệ nghịch thì được 800/2000/12000 nhưng KHÔNG nguồn nào xác
   nhận, nên driver CỐ TÌNH bỏ qua `mag_range_g` cho riêng chip này thay vì
   đoán. 8 G dù sao cũng đúng: 0,5 G cho khoảng 1500 count, còn thừa nhiều
   chỗ cho nhiễu động cơ.

### Số đo thực tế

Khởi động:

```
BMP388: OK, chip id 0x = 80
QMC5883P @ 0x2C
  CHUA HIEU CHUAN - chay DBG_MODE_MAGCAL truoc khi dung cho giu huong.
```

`DBG_MODE_MAG` (`mode 21`):

```
     mx     my     mz |    Btot |  raw_x  raw_y  raw_z | dt_ms |  count |  err  lost stale
  0.045 -0.536 -0.294 |   0.613 |    142   -810   -584 |    12 |   3489 |    0   286     0
```

| Tiêu chí | Kết quả |
|---|---|
| Chip nhận dạng | ✅ QMC5883P @ 0x2C, chip ID 0x80 |
| `err` từ kế | ✅ **0** |
| `stale` từ kế | ✅ **0** |
| Nhịp đọc | ✅ 388 mẫu / 8 s ≈ **48,5 Hz** (đặt 50 Hz) |
| Độ lớn từ trường thô | ✅ căn(142²+810²+584²) = 1009 count / 3000 = **0,336 G** — đúng dải Trái Đất 0,25–0,65 G |
| `SENSOR_MAG` (bit 8) | ✅ `health = 0x0507` |
| BMP388 sau khi chia bus | ✅ `err = 0`, `count` tăng ~49 Hz, `alt_rel` ~ 0,00 m |
| `lmax` | ✅ **220 µs** (trước khi có từ kế: 231 µs ghi trong KE_HOACH_LSM6DSV.md) |
| `rhz` | ⚠️ **3725–3730** so với 3790 ghi trong docs — thấp hơn khoảng 1,6% |

⚠️ Cột `Btot` đang in **0,61** chứ không phải 0,336 vì bộ hiệu chuẩn sắt cứng
trong `g_params` vẫn là **số của chip QMC6309 cũ** (`mag_offset_y_g = 0.2674`
đang bị trừ khỏi trục Y). Đây chính là lý do GĐ4 bắt buộc.

### Sửa thêm ngoài kế hoạch: BMP388 đếm tranh chấp bus thành lỗi thật

Lần kiểm đầu thấy `err = 0x0014` — ngoài `FC_ERR_RC_TIMEOUT` (đúng, đã tháo
ELRS) còn **`FC_ERR_BARO_I2C` bị chốt**, và cột `err` của baro đứng ở 52.

Nguyên nhân do chính thay đổi này: `bmp388.c start_read()` gọi
`record_error()` với **mọi** giá trị trả về khác `HAL_OK`. Trước đây I2C1 chỉ
có một chủ nên `HAL_BUSY` gần như không bao giờ xảy ra và điều đó vô hại. Từ
khi thêm từ kế, BMP388 có thể thua tranh chấp bus và đếm đó thành lỗi thật,
rồi `fc_state_set_error()` **chốt** cờ lỗi toàn cục, không tự xoá.

Đúng vấn đề đã xử lý ở phía từ kế nhưng bỏ sót ở phía baro. Đã sửa: thêm bộ
đếm riêng `bmp388_bus_lost()`, không đặt cờ lỗi khi chỉ là nhường bus, và
thêm cột `lost` vào bảng `DBG_MODE_BARO` để mức tranh chấp không vô hình.

Sau khi sửa: `err = 0x0010` (chỉ còn RC timeout), baro `err = 0`, baro
`lost` = 4 lên 5 trong 8 giây, từ kế `lost` khoảng 1/giây. Thiết kế chia bus
bằng `Devaddress` hoạt động đúng như dự tính.

### Còn lại: GĐ4 — hiệu chuẩn (cần xoay máy bay bằng tay)

Phần này không tự động hoá được. Quy trình:

1. Xoá bộ hiệu chuẩn của chip cũ:
   `set mag_offset_x_g 0` (và `y`, `z`), `set mag_scale_x 1` (và `y`, `z`)
2. `mode 22` (`DBG_MODE_MAGCAL`)
3. Xoay máy bay theo **mọi hướng** cho đủ 8 góc phần tám (cột "goc phan tam
   da phu" phải đạt 8/8)
4. Bấm **nút K1** để chốt — console in ra các dòng `set mag_offset_...` để
   copy, rồi `save`
5. `mode 21` kiểm lại: **`Btot` phải bám quanh 0,3–0,4 G và dao động dưới 5%
   khi xoay mọi hướng**
6. `MAG_AXIS_MAP_*` và `MAG_AXIS_SIGN_*` vẫn **CHƯA ĐO** — xác định bằng cách
   so hướng mũi thật với cột `raw`

Chưa hiệu chuẩn xong thì **không mở GĐ6** (hợp nhất yaw vào EKF).

### Điểm không nhất quán còn nợ

Console in **hai dòng từ kế mâu thuẫn** lúc khởi động:

```
Tu ke: TAT bang mag_source = NONE.     <- nhanh SHUB, doc g_params.mag_source (=0 trong flash)
QMC5883P @ 0x2C                        <- nhanh I2C, gac bang #if MAG_SOURCE
```

Nguyên nhân: `mag_source` tồn tại **cả** dạng macro biên dịch (`MAG_SOURCE`,
quyết định code nào được biên dịch) **và** dạng tham số runtime
(`g_params.mag_source`, mà nhánh SHUB kiểm). Tham số lưu trong flash vẫn là 0.
Không sai chức năng nhưng đọc log dễ tưởng từ kế đang tắt. Chưa sửa vì cần
chốt hướng trước: bỏ hẳn tham số runtime `mag_source`, hay cho nhánh I2C cũng
kiểm nó?

---

## Bổ sung 2026-09-04

### Đối chiếu toàn bộ tham số: `fc_config.h` đã khớp board 149/149

Lấy `dump` thẳng từ board qua CLI rồi so máy móc với giá trị mặc định trong
`fc_config.h` (script so khớp tên tham số → tên macro → giá trị, so theo SỐ
chứ không so chuỗi).

Kết quả: **143 giống hệt, 3 khác định dạng nhưng cùng giá trị, 3 macro là
biểu thức**. Tức là **không có gì phải cập nhật** — file config và flash đang
đồng bộ sẵn.

| Trường hợp | Giải thích |
|---|---|
| `DSHOT_BITRATE_HZ` = `300000UL` vs dump `300000` | cùng giá trị, khác hậu tố |
| `DSHOT_REVERSE_MASK` = `0x00` vs dump `0` | cùng giá trị, hex vs thập phân |
| `MAG_SOURCE` = `MAG_SOURCE_I2C` vs dump `2` | `MAG_SOURCE_I2C` chính là 2 |
| `RC_MODE_ALTHOLD_THRESHOLD` | biểu thức `(MIN+MID)/2` = 582 ✓ |
| `RC_MODE_POSHOLD_THRESHOLD` | biểu thức `(MID+MAX)/2` = 1401 ✓ |
| `RC_MODE_ACRO_THRESHOLD` | biểu thức `(MID+MAX)/2` = 1401 ✓ |

### Xoá bộ hiệu chuẩn cũ (GĐ4 bước 1) — ĐÃ LÀM

Sáu giá trị `mag_offset_*` / `mag_scale_*` trong dump *có* khớp `fc_config.h`,
nhưng cả hai đều là số của **chip QMC6309 cũ**, sai với QMC5883P. Đã:

1. Đặt mặc định trong `fc_config.h` về **đơn vị** (0 / 1,0), giữ bộ số cũ
   trong chú thích để tra cứu.
2. Ghi đè luôn trong flash của board qua CLI (`set ... ` rồi `save`), vì
   tham số đã `save` thì mặc định biên dịch không ghi đè được.

Kết quả đo lại: **`Btot` từ 0,61 G xuống 0,398 G**, khớp đúng độ lớn thô
√(562² + 986² + 374²) / 3000 = 0,398 G. Giờ `Btot` phản ánh từ trường thật
chứ không còn bị bộ offset sai kéo lệch.

### Sửa khoá pha giữa hai bộ hỏi vòng trên I2C1

Quan sát trong `DBG_MODE_MAG`: ba dòng liên tiếp cùng một `raw`, `dt_ms` leo
32 → 52 → 72 ms, `lost` tăng đều. Từ kế thua bus ba lượt liên tiếp.

Nguyên nhân: `mag_i2c_update()` đóng dấu `s_last_poll_us` **trước** khi biết
lượt đọc có khởi phát được không. Thua bus là mất trọn một chu kỳ 20 ms. Và
vì cả hai bộ hỏi vòng chạy từ cùng vòng lặp chính với chu kỳ cố định (baro
10 ms, từ kế 20 ms) nên chúng **khoá pha** được — từ kế cứ rơi đúng vào lúc
baro đang truyền.

Đã sửa: `start_read()` trả về `bool`, chỉ đóng dấu mốc khi lượt đọc thật sự
khởi phát. Thua bus thì thử lại ngay ở vòng lặp kế tiếp (dưới 1 ms sau).

Đo trước / sau:

| | trước | sau |
|---|---|---|
| `dt_ms` | 32–72 ms (có lúc kẹt) | 10–17 ms |
| nhịp mẫu | ~48,5 Hz | ~49 Hz |
| `rhz` | 3725 | **3742** |
| `lmax` | 220 µs | 225 µs |

Chi phí gần như bằng 0 — `rhz` còn nhích lên (trong khoảng nhiễu), `lmax`
thêm 5 µs.

⚠️ **Đổi lại ý nghĩa cột `lost`:** giờ nó đếm *số lần gọi đọc gặp bus bận*,
gồm cả các lần thử lại trong cùng một chu kỳ — **không** phải số mẫu bị mất.
Đo được khoảng 9 lần thử mỗi mẫu (~450/giây ở 50 Hz). Muốn biết có mất mẫu
thật hay không thì xem cột `count` có tăng đủ nhịp không. Đã ghi rõ trong
`mag_i2c.h` và phần mô tả cột của console.

(BMP388 vẫn đóng dấu vô điều kiện nên `bmp388_bus_lost()` đúng nghĩa "số chu
kỳ đã bỏ" — nó hỏi vòng 100 Hz trên ODR 50 Hz nên bỏ một lượt vẫn kịp bắt
mẫu ở lượt sau, không cần thử lại nhanh.)

### Sửa hai dòng log mâu thuẫn

Trước đây console in `Tu ke: TAT bang mag_source = NONE.` ngay trên dòng
`QMC5883P @ 0x2C`. Câu đầu là của nhánh SHUB và nói sai — `mag_source` thật
sự bằng 2 (I2C) chứ không phải NONE. Đã đổi thành
`Tu ke qua sensor hub: TAT (mag_source khac SHUB).`

Đồng thời cho nhánh I2C **cũng kiểm `g_params.mag_source`** giống nhánh SHUB,
để tắt được từ kế lúc chạy mà không phải nạp lại firmware. Log giờ đọc thẳng:

```
Tu ke qua sensor hub: TAT (mag_source khac SHUB).
BMP388: OK, chip id 0x = 80
QMC5883P @ 0x2C
  CHUA HIEU CHUAN - chay DBG_MODE_MAGCAL truoc khi dung cho giu huong.
```

### Trạng thái: chỉ còn phần phải xoay máy bay bằng tay

Board đang chạy firmware đã nạp và verify, hiệu chuẩn đã về đơn vị, sẵn sàng
cho MAGCAL. Việc còn lại đúng bằng GĐ4 bước 2–6 ở trên — không tự động hoá
được vì phải xoay máy bay đủ 8 góc phần tám và bấm K1.

