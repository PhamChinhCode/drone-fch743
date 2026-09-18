# Sao lưu bảng tham số — 2026-09-13, cập nhật 2026-09-18

Bản sao lưu bảng tham số trong flash, dùng để nạp lại khi cấu hình bị mất hoặc
hỏng.

## Bản mới nhất — 2026-09-18

Đọc sau khi nạp firmware `b086514` (hợp đồng 1.7) và reset board. Bảng tham số
**không đổi cấu trúc** so với 09-13 (`table_crc = 0x61D4CE56`, 167 tham số), nên
mọi file dưới đây nạp được lên firmware hiện tại.

| File | Nội dung |
|---|---|
| `2026-09-18_toan_bo.txt` | 167 tham số, lệnh `dump` — **cấu hình đang chạy trên drone** |
| `2026-09-18_khac_mac_dinh.txt` | 10 tham số khác mặc định, lệnh `diff` |
| `flash_o_A_seq13.srec` | Ô flash A, thô, 480 byte @ `0x081C0000`, `seq = 13` (mới nhất) |
| `flash_o_B_seq12.srec` | Ô flash B, thô, 480 byte @ `0x081E0000`, `seq = 12` |

Hai ô đều đúng CRC (`data_crc` A `0xCA64B907`, B `0x812633B3`), schema 8,
`sizeof(param_storage_t) = 460`; `.srec` đã giải ngược và so từng byte với dữ liệu
đọc qua SWD. Chỉ nạp `.srec` khi firmware có **đúng `table_crc = 0x61D4CE56`**
(xem cảnh báo ở Cách 2).

**Khác bản 09-13 (`hien_tai_toan_bo.txt`) ở 6 tham số** — đã lưu vào flash sau
09-13 (seq 11 → 13), chưa ghi lại là đã kiểm chứng khi bay:

| Tham số | 09-13 | 09-18 |
|---|---|---|
| `rate_pid_roll_ki` | 0.001 | 0.0001 |
| `rate_pid_pitch_kp` | 0.0008 | 0.001 |
| `rate_pid_pitch_ki` | 0.001 | 0.0001 |
| `poshold_vel_kp` | 10 | 15 |
| `poshold_vel_ki` | 1 | 2 |
| `althold_climb_kp` | 0.25 | 0.3 |

Bộ chỉnh này hỏng thì nạp `2026-09-18_toan_bo.txt`; muốn quay về bộ đã bay 09-13
thì nạp `hien_tai_toan_bo.txt`.

---

*Phần dưới là bản gốc 2026-09-13.*

## Vì sao có thư mục này

Ngày 2026-09-13 dự án thêm 10 tham số OFFBOARD. Việc thêm tham số làm **đổi
`table_crc`**, vì [param_table.c](../param_table.c) băm cả tên, giới hạn và giá
trị mặc định. Lúc khởi động, `param_store_load()` thấy CRC lệch nên **bỏ qua
khối đã lưu mà không báo gì**, và board chạy toàn bằng giá trị mặc định.

Bộ chỉnh thật của máy bay (PID, trim, POSHOLD, hiệu chuẩn flow) suýt mất vì
chuyện đó. Dữ liệu cứu được vì `load` chỉ bỏ qua chứ không xoá flash. Tất cả
file dưới đây được đọc thẳng từ flash và RAM qua SWD.

**Chuyện này sẽ lặp lại** mỗi khi thêm, xoá tham số hoặc đổi giá trị mặc định.
Trước lần đổi bảng tham số tiếp theo, hãy lưu lại cấu hình bằng lệnh `dump`.

## Các file

| File | Nội dung | Dùng khi nào |
|---|---|---|
| `hien_tai_toan_bo.txt` | 167 tham số, cấu hình **đang tốt** hiện tại | **Mặc định chọn file này** khi cấu hình hiện tại hỏng |
| `truoc_offboard_khac_mac_dinh.txt` | 12 tham số đã chỉnh khác mặc định | Chỉ muốn lấy lại bộ chỉnh, giữ mọi thứ khác |
| `truoc_offboard_toan_bo.txt` | 157 tham số của bảng cũ | Muốn trả toàn bộ về đúng trạng thái trước OFFBOARD |
| `flash_o_A_seq11.srec` | Ô flash A, thô, 448 byte @ `0x081C0000` | Chỉ khi quay lại firmware cũ — xem cảnh báo |
| `flash_o_B_seq10.srec` | Ô flash B, thô, 448 byte @ `0x081E0000` | Như trên |
| `nap_tham_so.ps1` | Script nạp file `.txt` qua console | Dùng cho ba file `.txt` |

Bộ chỉnh 12 tham số là cấu hình thật của máy bay:

| Nhóm | Tham số |
|---|---|
| PID rate | `rate_pid_{roll,pitch,yaw}_{kp,ki}` |
| Trim | `angle_trim_roll_deg` 0.7, `angle_trim_pitch_deg` 0.5 |
| POSHOLD | `poshold_vel_kp` 10, `poshold_vel_ki` 1, `poshold_pos_kp` 0.8 |
| Optical flow | `flow_rad_per_count` 0.002 |

## Cách 1 — nạp file `.txt` (khuyên dùng)

Chạy được trên **mọi phiên bản firmware** có cùng tên tham số, vì nạp theo tên
chứ không theo vị trí byte. Tham số nào không còn tồn tại thì dòng đó báo `ERR`,
còn các dòng khác vẫn nạp bình thường.

```powershell
cd App\Config\param_backup

# Xem trước: nạp vào RAM, CHƯA ghi flash
.\nap_tham_so.ps1 -File hien_tai_toan_bo.txt

# Kết quả đúng rồi mới ghi
.\nap_tham_so.ps1 -File hien_tai_toan_bo.txt -Save
```

Mỗi dòng in ra **giá trị firmware thực sự nhận**. Firmware tự kẹp theo min/max,
nên nếu số in ra khác số trong file thì phải xem lại. Script **từ chối `save`**
nếu có bất kỳ dòng nào báo lỗi.

Cổng mặc định là `COM4`; đổi bằng `-Port COM3`. Nhớ đóng terminal đang mở cổng
đó trước khi chạy.

Nếu không có PowerShell: mở terminal 921600 baud, gõ `mode 0`, rồi **gõ từng
dòng một**. Đừng dán cả file một lượt — đệm nhận của CLI chỉ 256 byte, dán
nhiều dòng sẽ mất dòng. Dòng bắt đầu bằng `#` được bỏ qua.

## Cách 2 — nạp thẳng ô flash (`.srec`)

> **CẢNH BÁO.** Chỉ dùng khi đã quay firmware về đúng bản có
> **`table_crc = 0x4C71C864`**, tức bản trước khi thêm tham số OFFBOARD.
>
> Nạp `.srec` lên firmware hiện tại thì lúc khởi động khối này bị bỏ qua vì lệch
> CRC — **vô dụng**. Tệ hơn, nó **ghi đè lên cấu hình đang lưu** trong hai ô
> flash. Tức là mất cấu hình tốt mà không được gì.

Kiểm tra trước khi nạp — gõ `version` trên console và so dòng `table_crc`:

```
table_crc = 0x4C71C864     <- đúng bản cũ, được phép nạp
table_crc = 0x61D4CE56     <- firmware hiện tại (2026-09-13), KHÔNG nạp
```

Khi đã chắc chắn:

```powershell
STM32_Programmer_CLI -c port=SWD -w flash_o_A_seq11.srec -w flash_o_B_seq10.srec -rst
```

## Nguồn gốc dữ liệu

| Hạng mục | Giá trị |
|---|---|
| Ngày đọc | 2026-09-13 |
| Ô A | `seq = 11`, CRC dữ liệu đúng — bản lưu mới nhất |
| Ô B | `seq = 10`, CRC dữ liệu đúng |
| Schema | `PARAM_SCHEMA_VERSION = 8`, `sizeof(param_storage_t) = 428` |
| `table_crc` bảng cũ | `0x4C71C864` |
| `table_crc` bảng hiện tại | `0x61D4CE56` |

Bố cục bảng cũ dựng lại từ `App/Config/param_list.h` ở commit `8b9b35f`, tính ra
đúng 428 byte, khớp trường `size` trong header. Hai file `.srec` đã được giải mã
ngược và so từng byte với dữ liệu gốc đọc từ flash.

`hien_tai_toan_bo.txt` đọc từ RAM **sau khi reset board**, nên nó là cấu hình
firmware thực sự nạp từ flash lúc khởi động — không phải giá trị gõ tay còn
nằm trong RAM.
