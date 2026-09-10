# Log bay trên flash QSPI — cách dùng và cách tích hợp vào app cấu hình

Ghi log chuyến bay thẳng vào **flash NOR W25Q64 (U3)** trên QUADSPI, ngay
trong lúc đang bay.

> ✅ **ĐÃ HIỆN THỰC VÀ ĐÃ KIỂM TRÊN PHẦN CỨNG** (2026-09-05, ST-Link + COM4).
> Xem mục [Đã kiểm được gì](#đã-kiểm-được-gì) ở cuối.

Tài liệu này viết cho hai người đọc: người bay (mục 1–3) và người viết app
cấu hình trên PC (mục 4–6).

---

## Vì sao có đường log này bên cạnh thẻ SD

| | Thẻ SD ([blackbox.c](../Storage/blackbox.c)) | Flash QSPI ([flashlog.c](../Storage/flashlog.c)) |
|---|---|---|
| Sức chứa | hàng GB | **8 MB = 21,8 phút** ở 100 Hz |
| Ghi lúc đang ARM | **Không thể** — `f_write` khựng hàng trăm ms | **Được** |
| Giới hạn một chuyến | 41 giây (bằng bộ đệm RAM) | không có |
| Mất điện giữa lúc ghi | hỏng FAT, nặng thì kẹt luôn thẻ | mất tối đa 256 byte cuối |
| Lấy dữ liệu ra | cắm thẻ vào máy tính | qua lệnh `flash dump` |

Điểm cốt lõi: flash NOR **không có hệ tập tin và không có bảng ánh xạ nội
bộ** nào để mà hỏng. Không tồn tại trạng thái "lần sau không khởi tạo được".

Ghi được trong lúc bay là vì thao tác tách làm hai: đẩy 256 byte xuống chip
mất **34 µs**, rồi chip tự lập trình thêm 0,4 ms trong khi CPU đi làm việc
khác. Ở nhịp 100 Hz mỗi trang cách nhau 53 ms nên thừa thãi.

---

## 1. Các lệnh

Gõ trên **USART1 / COM4, 921600 baud** (PuTTY, hoặc app của bạn — xem mục 4).

| Lệnh | Làm gì | Chặn bao lâu | Khi ARM |
|---|---|---|---|
| `flash info` | Trạng thái chip và log | không đáng kể | được |
| `flash dump` | Trút toàn bộ log ra CSV | không chặn (in dần) | được |
| `flash erase` | Xoá **toàn bộ** chip | phát lệnh rồi trả về ngay; chip bận 20–100 s | **bị chặn** |
| `flash rescan` | Dò lại điểm cuối dữ liệu | ~1,5 ms | **bị chặn** |
| `flash test` | Tự kiểm phần cứng | tới 400 ms | **bị chặn** |
| `flash stop` | Bỏ ngang bản trút đang chạy | không đáng kể | được |
| `flash sim <n>` | Ghi một chuyến bay giả `n` bản ghi | vài giây | **bị chặn** |

Mọi lệnh có thể chặn đều bị từ chối khi đang ARM — mất khung DShot lâu như
vậy thì ESC coi như mất tín hiệu và cắt motor.

### Lúc bay không cần gõ gì

ARM là tự ghi, DISARM là tự xả nốt trang dở. Chỉ `flash erase` mới là thao
tác thủ công có chủ ý.

### Quy trình dùng điển hình

```
> flash info                 xem con bao nhieu cho
> flash erase                neu can cho trong
  [doi 20-100 giay]
> flash rescan               BAT BUOC sau khi xoa
  ... bay ...
> flash dump > bay.csv       mo thang bang Excel / pandas
```

`flash rescan` sau khi xoá là bắt buộc: con trỏ ghi chỉ được dò **một lần**
lúc khởi động, nên xoá xong mà không rescan (hoặc không khởi động lại) thì
firmware vẫn tưởng chip còn đầy.

### `flash info` đọc thế nào

```
flash jedec = 0xEF4017          EF 40 17 = Winbond W25Q64
flash dung luong MB = 8
flash sr1 = 0x00                bit0 = dang ban
flash sr2 = 0x02                bit1 = co QE (chip nay xuat xuong da bat)
flash: ranh
log da dung KB = 19
log suc chua KB = 8188          8192 - 4 KB de danh cho 'flash test'
log ban ghi chuyen nay = 300
log ban ghi bi bo = 0           KHAC 0 la chip khong theo kip - bao ngay
san sang, cho arm
```

Năm trạng thái: `TAT / khong thay chip`, `san sang, cho arm`,
`DANG GHI vao flash`, `DAY - can 'flash erase'`, `LOI CHIP`.

---

## 2. Định dạng CSV — 34 cột

Dòng bắt đầu bằng `#` là chú thích (mốc chuyến bay, kết thúc), phần còn lại
là dữ liệu. Dòng cuối luôn là `# het`.

```
t_ms,gx,gy,gz,ax,ay,az,sp_x,sp_y,sp_z,pid_x,pid_y,pid_z,m1,m2,m3,m4,
roll,pitch,yaw,alt_m,thr,mode,armed,sat,
sp_roll,sp_pitch,v_fwd,v_right,vt_fwd,vt_right,range_m,flow_q,est_flags
```

| Cột | Đơn vị | Nghĩa |
|---|---|---|
| `t_ms` | ms | kể từ lúc bắt đầu chuyến |
| `gx gy gz` | °/s | tốc độ góc **đã lọc** — cái mà PID nhìn thấy |
| `ax ay az` | g | gia tốc |
| `sp_x sp_y sp_z` | °/s | mục tiêu tốc độ góc (đầu ra vòng góc) |
| `pid_x pid_y pid_z` | −1…1 | đầu ra PID tốc độ góc |
| `m1…m4` | 0…2047 | giá trị DShot thô |
| `roll pitch yaw` | rad | góc **đo được** |
| `alt_m` | m | độ cao đã hợp nhất baro + range |
| `thr` | 0…1 | lệnh ga trước khi trộn |
| `mode` | | `flight_mode_t` |
| `armed` | 0/1 | |
| `sat` | 0/1 | mixer chạm trần — **bão hoà thì mọi phân tích phía trên vô nghĩa** |
| `sp_roll sp_pitch` | rad | góc mà tầng trên **ra lệnh** |
| `v_fwd v_right` | m/s | vận tốc thân **đo được** |
| `vt_fwd vt_right` | m/s | vận tốc thân mà poshold **đang đòi** |
| `range_m` | m | khoảng cách tới mặt đất từ MTF-01P |
| `flow_q` | 0…255 | chất lượng optical flow |
| `est_flags` | số nguyên | xem bảng bit dưới |

### `est_flags`

| Bit | Giá trị | Nghĩa |
|---|---|---|
| 0 | 1 | `position_valid` — **không có bit này thì poshold không hề vào chế độ giữ chỗ** |
| 1 | 2 | `altitude_valid` |
| 2 | 4 | `attitude_valid` |
| 3 | 8 | driver MTF-01P còn sống |
| 4 | 16 | `range_valid` |
| 5 | 32 | poshold đã chốt mốc giữ chỗ |

Ví dụ `est_flags = 30` = `0b011110`: có đủ altitude, attitude, flow, range —
nhưng **thiếu bit 0**. Poshold sẽ từ chối vào và lặng lẽ lùi về lái bằng cần.

### Đọc theo thứ tự nào khi giữ vị trí có vấn đề

1. **`est_flags` bit 0.** Tắt thì dừng ngay ở đây — poshold chưa từng chạy.
   Đi tìm lý do ở `flow_q` và `range_m`.
2. **`vt_*` so với `v_*`** — sai số vận tốc. Còn lệch lớn mà `sp_roll`/
   `sp_pitch` đã kịch trần → hết thẩm quyền nghiêng. Lệch nhỏ mà `sp_*` giật
   mạnh → `poshold_vel_kp` quá cao.
3. **`sp_roll`/`sp_pitch` so với `roll`/`pitch`** — vòng góc có thực hiện nổi
   lệnh không. Không bám thì lỗi nằm ở vòng trong, không phải poshold.
4. **`sat`** — bão hoà thì quay lại bước đầu, mọi thứ trên đều vô nghĩa.

---

## 3. Bố trí nhị phân trên chip

Cần cho app nào muốn đọc thô thay vì phân tích CSV, hoặc đọc chip bằng máy
nạp. **Little-endian**, không có byte đệm căn lề (`__attribute__((packed))`).

Toàn chip là **một dòng bản ghi 64 byte nối tiếp, mọc từ địa chỉ 0**. Không
FAT, không bảng chỉ mục, không siêu dữ liệu.

```
[moc chuyen 1][64B][64B]...[đệm FF][moc chuyen 2][64B]... [FF FF FF ...]
 ^ dau trang                        ^ dau trang
```

### Mốc đầu chuyến bay — 64 byte

| Offset | Kiểu | Trường | |
|---|---|---|---|
| 0 | `uint32` | `magic` | **`0xFCBB0001`** — nhận ra mốc bằng cái này |
| 4 | `uint32` | `boot_ms` | `HAL_GetTick()` lúc bắt đầu chuyến |
| 8 | `uint16` | `rate_hz` | 100 |
| 10 | `uint8` | `rec_bytes` | 64 |
| 11 | `uint8` | `version` | **2** = bản ghi 64 byte |
| 12 | `uint8[52]` | `reserved` | 0 |

`magic` nằm đúng chỗ trường `t_ms` của bản ghi dữ liệu, và giá trị đó tương
ứng 49 ngày bay liên tục nên không thể nhầm với một mốc thời gian thật.

### Bản ghi dữ liệu — 64 byte

| Offset | Kiểu | Trường | Hệ số | Ra đơn vị |
|---|---|---|---|---|
| 0 | `uint32` | `t_ms` | ×1 | ms |
| 4 | `int16[3]` | `gyro` | ÷10 | °/s |
| 10 | `int16[3]` | `accel` | ÷1000 | g |
| 16 | `int16[3]` | `sp` | ÷10 | °/s |
| 22 | `int16[3]` | `pid` | ÷10000 | −1…1 |
| 28 | `uint16[4]` | `motor` | ×1 | 0…2047 |
| 36 | `int16[3]` | `att` | ÷100 | rad |
| 42 | `int16` | `alt_cm` | ÷100 | m |
| 44 | `uint16` | `thr` | ÷10000 | 0…1 |
| 46 | `uint8` | `mode` | | `flight_mode_t` |
| 47 | `uint8` | `flags` | | bit0 armed, bit1 mixer bão hoà |
| 48 | `int16[2]` | `sp_angle` | ÷100 | rad (roll, pitch) |
| 52 | `int16[2]` | `vel_body` | ÷100 | m/s (tới, phải) |
| 56 | `int16[2]` | `vel_tgt` | ÷100 | m/s (tới, phải) |
| 60 | `uint16` | `range_cm` | ÷100 | m |
| 62 | `uint8` | `flow_q` | | 0…255 |
| 63 | `uint8` | `est_flags` | | xem bảng bit ở mục 2 |

Nguồn sự thật là [log_record.h](../Storage/log_record.h) — nếu tài liệu này
lệch với file đó thì file đó đúng.

64 byte chia đúng 256 nên **mỗi trang flash chứa trọn 4 bản ghi**, không bản
ghi nào bị cắt qua ranh giới trang.

### Luật phân tích

```
pos = 0
lặp:
    đọc 64 byte tại pos
    nếu toàn 0xFF:
        nếu pos nằm đúng đầu trang (pos % 256 == 0):  -> HẾT DỮ LIỆU
        ngược lại: pos = làm_tròn_lên(pos + 1, 256)   -> bỏ qua phần đệm
        tiếp
    nếu 4 byte đầu == 0xFCBB0001:  -> mốc chuyến bay mới
    ngược lại:                     -> bản ghi dữ liệu
    pos += 64
```

Phần đệm `0xFF` sinh ra khi DISARM: trang dở dang được đệm cho đủ rồi ghi
xuống ngay, để rút điện liền sau đó cũng không mất dữ liệu. Cái giá là mỗi
chuyến phí tối đa 255 byte và chuyến sau bắt đầu ở ranh giới trang.

### Tìm điểm cuối

Firmware dò bằng **tìm nhị phân trang đã xoá đầu tiên** — 15 lần đọc thay vì
quét 32768 trang. Đúng vì dữ liệu luôn mọc liên tục từ 0 nên các trang chia
làm đúng hai vùng: đã ghi rồi tới đã xoá, không xen kẽ.

App đọc thô cũng nên làm y vậy thay vì đọc tuần tự cả 8 MB.

---

## 4. Tích hợp vào app cấu hình

Có **hai** đường lấy log ra, cả hai đều dùng được. App cấu hình đi đường thứ
hai; đường thứ nhất là để gõ tay trong PuTTY và để đối chiếu khi app cư xử lạ.

### Đường 1: CLI chữ trên USART1

Mở COM của USART1 ở 921600, gửi `flash dump\r\n`, đọc từng dòng cho tới
`# het`. Đo được **85 KB/s**, tức bám sát trần lý thuyết 92 KB/s của
921600 baud — nghẽn ở UART chứ không ở flash hay CPU.

Một dòng CSV đo được 153 byte lúc máy nằm yên, khoảng 180 byte khi đang bay
(số lớn hơn thì nhiều chữ số hơn).

### Đường 2: CLI nhị phân — app cấu hình đi đường này

Qua `TLM_MSG_CMD_CLI`, dùng được **từ giao thức 2**.

> Trước bản 2 thì đường này **cắt cụt mọi dòng CSV mà không báo gì**:
> `tlm_cli_line_t.text` chỉ có 60 ký tự. App nhận về những dòng trông vẫn
> đúng định dạng, chỉ là thiếu cột. Nếu bạn còn firmware giao thức 1 thì
> đừng dùng đường này để trút log.

Bản 2 đổi ba chỗ:

| | Giao thức 1 | Giao thức 2 |
|---|---|---|
| `tlm_cli_line_t.text` | 60 | **245** |
| `TLM_MAX_PAYLOAD` | 64 | **248** |
| Cắt cụt | im lặng | báo bằng cờ `TLM_CLI_FLAG_TRUNC` |

245 là gần hết cỡ chứ không phải chọn cho tròn: **byte `LEN` trong khung chỉ
có một byte** ([tlm_protocol.h](../Telemetry/tlm_protocol.h)), nên payload
không bao giờ vượt được 255. Dòng CSV dài nhất `log_record_to_csv()` sinh ra
được là **241 ký tự** (mọi trường `int16` cùng kịch biên), nên 245 vừa đủ với
một chút dư.

Hệ quả cho ai định thêm cột vào bản ghi (mục 5): `LOG_RECORD_CSV_MAX` là 320,
tức **lớn hơn sức chứa của khung**. Thêm cột tới mức dòng vượt 245 ký tự thì
không nới thêm được nữa — phải đổi sang khung nhị phân đọc log thô. Lúc đó
`TLM_CLI_FLAG_TRUNC` sẽ bật và app kêu, thay vì lặp lại đúng lỗi âm thầm cũ.

Vì `tlm_cli_line_t` chỉ gửi đúng `3 + độ_dài_thật` byte nên dòng ngắn vẫn
tốn đúng như trước; nới trần không làm chậm gì cả.

### Trút mất bao lâu

Đo thật ngày 2026-09-05, 300 bản ghi (19 KB log):

| Đường | Tốc độ | 300 bản ghi | Suy ra cho 8 MB đầy chip |
|---|---|---|---|
| USART1, CLI chữ | 85 KB/s | 0,6 s | ~4–4,5 phút |
| USB CDC, CLI nhị phân | **175 KB/s** | **0,28 s** | ~2 phút |

Con số 175 KB/s vượt trần 92 KB/s của 921600 baud vì đường đó là **USB CDC**,
không phải UART. Qua USART3 → ESP32 thì trần vẫn là 92 KB/s. Gõ `port` để
biết mình đang ở đường nào; app tự gửi `port here` lúc bắt tay nên nó luôn
kéo đường phát về đúng cổng nó đang mở.

Phụ phí khung là 10 byte mỗi dòng (7 byte khung + 3 byte seq/flags) trên một
dòng khoảng 165 byte, tức khoảng 6%.

### `flash stop` — vì sao phải có

Một lệnh mới **không** làm dừng bản trút đang chạy: `s_dump_on` vẫn bật và
đầu ra của hai lệnh trộn vào nhau. Không có `flash stop` thì nút Huỷ bên app
chỉ huỷ được phía app, còn đường truyền kẹt thêm tới bốn phút rưỡi nữa. App
gửi lệnh này ngay sau khi người dùng bấm Huỷ.

Lệnh này đặt **trước** nhánh kiểm ARM: dừng lại thì không chặn vòng lặp, nên
không có lý do gì để từ chối nó khi đang bay.

### Tab "Log bay" trong app cấu hình

Đã hiện thực, xem
[LogTabViewModel.cs](../../../../Configurator/src/FcConfigurator/ViewModels/LogTabViewModel.cs).
Nó làm đúng ba việc mà `flash info` trả lời được mà không cần trút gì cả:

- **Thanh tiến độ `log da dung` / `log suc chua`**, kèm quy đổi ra phút
  (`KB × 1024 ÷ 64 ÷ 100` giây).
- **`log ban ghi bi bo`** khác 0 thì hiện chữ đỏ — chip không theo kịp.
- **Nút Xoá có hộp xác nhận**, mặc định là Huỷ, và nhắc phải `flash rescan`
  sau đó.

Phần đồ thị chia làm chín khung theo đơn vị (gyro °/s, góc rad, motor bậc
DShot…) vì gộp chung một trục thì tất cả trừ một đường đều bẹp vào trục. Mọi
khung **dùng chung một trục thời gian**: phóng to ở khung gyro thì khung
motor phóng theo, nếu không thì đối chiếu `sp_roll` với `roll` là vô nghĩa.

Cặp *đo được* / *mục tiêu* (`roll`/`sp_roll`, `v_fwd`/`vt_fwd`) vẽ **cùng
màu, đường mục tiêu nét đứt**.

Vẽ 131 nghìn mẫu không cần lấy mẫu thưa: mỗi cột điểm ảnh chỉ vẽ một đoạn
min..max. Lấy mẫu thưa sẽ làm **mất đỉnh nhiễu** — đúng thứ người ta mở log
ra để tìm.

App còn nhận `--log <file.csv>` để mở thẳng một bản log đã lưu mà không cần
cắm mạch bay.

## 5. Cấu hình lúc biên dịch

| Tham số | File | Mặc định | |
|---|---|---|---|
| `FLASHLOG_ENABLE` | [fc_config.h](../Config/fc_config.h) | 1 | |
| `FLASHLOG_RATE_HZ` | [fc_config.h](../Config/fc_config.h) | 100 | 500 Hz → còn 4,4 phút |
| `QSPI_FLASH_PRESCALER` | [fc_config.h](../Config/fc_config.h) | 3 | 240 MHz ÷ 4 = 60 MHz |
| `QSPI_FLASH_CS_HIGH_TIME` | [fc_config.h](../Config/fc_config.h) | 4 chu kỳ | 67 ns; W25Q64 đòi ≥ 50 ns |

Đây là cấu hình **lúc biên dịch**, không chỉnh được qua `set` lúc chạy — bảng
`g_param_table` không dính gì tới đây.

### Muốn thêm trường vào bản ghi

Sửa **ba chỗ, cùng lúc**:

1. Struct `bb_record_t` — [log_record.h](../Storage/log_record.h)
2. `log_record_fill()` — lấy giá trị từ `g_fc` ở đâu
3. `g_log_csv_header` và `log_record_to_csv()` — tên cột và cách in

Rồi:

- Giữ `_Static_assert` xanh: `sizeof(bb_record_t)` phải bằng
  `LOG_RECORD_BYTES`, và `flashlog_hdr_t` phải dài bằng đúng như vậy.
- **Tăng `hdr.version`** trong [flashlog.c](../Storage/flashlog.c).
- **`flash erase`** — log cũ khác kích thước sẽ đọc ra số vô nghĩa.
- Kiểm `LOG_RECORD_CSV_MAX` còn đủ cho dòng CSV dài nhất.
- **Kiểm dòng dài nhất còn dưới 245 ký tự** — sức chứa của
  `tlm_cli_line_t.text`. Vượt qua đó thì app trút qua đường nhị phân sẽ nhận
  cờ `TLM_CLI_FLAG_TRUNC` và từ chối dữ liệu; xem mục 4.

---

## 6. Những cái bẫy đã biết

- **Đầy thì DỪNG, không ghi đè vòng tròn.** Có chủ ý: không thể để log của
  một sự cố bị mất chỉ vì bay thêm một chuyến nữa. Xoá phải do người ra lệnh.
- **`flash test` xoá và ghi đè sector CUỐI chip.** Vùng log đã chừa sẵn 4 KB
  đó ra (`log suc chua = 8188 KB` chứ không phải 8192), nên hai bên không
  đụng nhau. Nhưng đừng dùng địa chỉ đó cho việc gì khác.
- **`flash rescan` bắt buộc sau khi xoá** (hoặc khởi động lại).
- **Không có RTC.** `boot_ms` là mili giây kể từ lúc cấp nguồn, không phải
  giờ thực. Muốn biết chuyến nào bay lúc nào thì app phải tự ghi lại.
- **Cần một chuyến bay thật để chốt.** Thời gian vòng lặp lúc đang RECORDING
  chưa đo được trên phần cứng vì ARM cần RC. Phân tích thì phần chặn là 34 µs
  mỗi trang, 53 ms một lần, nằm gọn trong ngân sách 250 µs khi vòng lặp
  thường chỉ tốn 11 µs — nhưng đó là suy luận. **Lần đầu ARM thật, xem
  `loop_max_us` trong `status`.**

---

## Đã kiểm được gì

Trên phần cứng thật, ST-Link + COM4, ngày 2026-09-05:

| Việc | Kết quả |
|---|---|
| JEDEC ID ở 20 MHz **và** 60 MHz | `EF 40 17`, khớp cả hai |
| Xoá sector 4 KB | 60 ms (datasheet 45 ms điển hình / 400 ms xấu nhất) |
| Ghi 256 byte | 451 µs |
| Đọc 256 byte | 58 µs |
| `flash sim 300` | đúng 19 KB như tính trước, không bỏ bản ghi nào |
| Ba lần khởi động lại | dò lại đúng 19 KB cả ba lần |
| `flash dump` | 34 cột khớp giữa tiêu đề và dữ liệu |
| Tốc độ trút, USART1 chữ | 85 KB/s |

Thêm ngày 2026-09-05, sau khi nới giao thức lên bản 2:

| Việc | Kết quả |
|---|---|
| `flash dump` qua đường **nhị phân** (USB CDC, COM17) | 300 bản ghi, 305 dòng, **34 cột đủ, không dòng nào bị cắt** |
| Dòng CSV dài nhất đo được | 155 ký tự (trần lý thuyết 241, sức chứa 245) |
| Tốc độ trút, nhị phân qua USB CDC | 0,28 s cho 49,8 KB → **175 KB/s** |
| Đối chiếu bản trút chữ (COM4) với bản nhị phân | trùng nhau, cùng 300 bản ghi |
| `flash stop` khi không có bản trút nào | trả về `# khong co ban trut nao dang chay` |
| Tab Log bay dựng đủ 9 khung đồ thị, con trỏ đọc số, xuất CSV trùng từng dòng với bản gốc | đạt (`--selftest COM17`) |

**Chưa kiểm:** ghi trong lúc ARM thật (cần RC), và blackbox trên thẻ SD sau
khi tách `log_record.c` ra dùng chung — thẻ đang tắt bằng `FC_SD_ENABLE = 0`
nên mới chỉ kiểm được ở mức biên dịch.

Bên app cũng còn ba chỗ chưa kiểm bằng tay: **cuộn chuột để phóng to, kéo để
dời, và nút Xoá**. Hai cái đầu chỉ kiểm được bằng cách ngồi trước màn hình mà
rê chuột; cái thứ ba thì cố ý không tự động chạy — nó xoá sạch log không hoàn
tác được. Bản trút đầy chip (4,5 phút, ~131 nghìn dòng) cũng chưa chạy thật:
mọi số ở trên suy từ bản 300 bản ghi.
