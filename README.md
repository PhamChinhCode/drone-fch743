# FCH743_V1.0 — firmware bay STM32H743

Firmware điều khiển bay cho drone giao hàng dùng AprilTag và GPS. Vòng điều khiển
(tốc độ góc, góc, độ cao, giữ vị trí) chạy trên bo này; định vị tuyệt đối theo tag,
nhiệm vụ giao hàng và liên lạc với trạm mặt đất chạy trên Raspberry Pi 4.

| Repo | Vai trò |
|---|---|
| **FCH743_V1.0** (repo này) | Firmware FC |
| [`drone-ros2-jazzy`](https://github.com/PhamChinhCode/drone-ros2-jazzy) | ROS 2 trên Pi 4: EKF, AprilTag, nhiệm vụ, MAVROS |
| [`drone-gcs`](https://github.com/PhamChinhCode/drone-gcs) | Trạm mặt đất: backend FastAPI + giao diện React/Three.js |

Hợp đồng liên thông FC ↔ Pi: `App/Docs/GIAO_UOC_FC_ROS2.md` (bản sao chỉ đọc;
**bản gốc nằm trong repo `drone-ros2-jazzy`**).

## Phần cứng

| Khối | Chi tiết |
|---|---|
| MCU | STM32H743VIT6 (bo DevEBox H7), vòng tốc độ góc 4 kHz |
| IMU chính | **ICM-42688-P**, SPI1, DRDY ở PC4, đọc 8 kHz bằng DMA |
| IMU phụ | LSM6DSV, SPI3 — chỉ để so sánh, không tham gia điều khiển |
| La bàn | **IST8310** trên module GPS (I2C1). QMC5883P trên bo là phương án dự phòng, chọn bằng `MAG_I2C_USE_GPS_MAG` |
| Khí áp | BMP388, I2C1 |
| Optical flow + laser | MTF-01P, UART4, MSP V2 |
| GPS | **MicoAir MG-F10-A** (u-blox NEO-F10N, L1+L5), UART7, UBX NAV-PVT 10 Hz |
| Điều khiển từ xa | ELRS/CRSF, USART2 |
| Động cơ | DShot300 trên TIM1 + DMA, 4 motor |
| Console / CLI | USART1 @ 921600 |
| Telemetry | USART3 → ESP32 (ESP-NOW); UART8 → MAVLink cho Pi |
| Lưu trữ | Flash NOR QSPI (log bay), thẻ SD, USB MSC |

## Mã nguồn

```
App/Config/     tham số runtime (param_list.h là nguồn sự thật) + fc_config.h
App/Common/     kiểu dữ liệu dùng chung, thời gian, thống kê nhiễu
App/Drivers/    icm42688, lsm6dsv, mag_i2c (+ist8310/qmc/hmc), bmp388, mtf01p,
                gps_ubx, crsf, dshot, qspi_flash
App/Estimator/  EKF tư thế, EKF độ cao, EKF vận tốc (optical flow)
App/Control/    vòng tốc độ góc, vòng góc, giữ độ cao, giữ vị trí, offboard, arming, mixer
App/State/      fc_state — "bảng đen" dùng chung cho toàn firmware
App/Storage/    blackbox (thẻ SD), flashlog (QSPI), tham số trong flash nội
App/Telemetry/  console gỡ lỗi, CLI, khung nhị phân cho PC
App/Mavlink/    MAVLink tới Pi
App/Docs/       tài liệu thiết kế, kế hoạch, hợp đồng
Core/ Drivers/ Middlewares/ USB_DEVICE/ FATFS/   phần do CubeMX sinh (.ioc ở gốc)
tools/          script phân tích chạy trên PC (flow_check.py)
```

Đọc trước khi sửa: `App/README.md`, và các khối ghi chú dài trong
`App/Config/fc_config.h` — phần lớn con số ở đó là **kết quả đo thật**, có ghi ngày,
cách đo và cả những lần đo sai đã bị bác bỏ.

## Build và nạp

```bash
cmake --preset Debug            # chỉ khi mới clone hoặc vừa thêm file nguồn
cmake --build --preset Debug
```

Nạp bằng ST-Link: dùng task `OpenOCD Flash` trong `.vscode/tasks.json`. Bốn biến
`AP_NUM / CORE_RESET / DUAL_BANK / CLOCK_FREQ` là bắt buộc với `target/stm32h7x.cfg`.

## Console

USART1, 921600 baud, gõ `help`. Hay dùng nhất:

| Lệnh | Việc |
|---|---|
| `status` | arm, cờ lỗi, thời gian vòng lặp |
| `gps` | GPS: baud, fix, số vệ tinh, sai số, vị trí |
| `get <tiền tố>` · `set <tên>=<giá trị>` · `save` | xem và sửa tham số runtime |
| `diff` · `dump` | xuất tham số ra chữ để sao lưu (`App/Config/param_backup/`) |
| `mode <số>` | chế độ in: 1 IMU, 12 vận tốc/flow, 16 EKF, 21 la bàn, 26 khớp trục la bàn |

## Ba cái bẫy đã mất thời gian nhất

1. **`table_crc` băm cả giá trị mặc định.** Đổi một `#define` mặc định có mặt trong
   `param_list.h` sẽ làm firmware **bỏ sạch cấu hình đã lưu trong flash** ở lần khởi động
   kế tiếp. Luôn `dump` vào `App/Config/param_backup/` trước khi nạp bản có đổi bảng tham số.
   (CRC gộp bằng XOR nên hai thay đổi giống hệt nhau có thể triệt tiêu — đừng tin nó tuyệt đối.)
2. **Chiều trục phải đo lại mỗi khi đổi cảm biến hoặc đổi cách lắp.** Xem khối
   `IMU_AXIS_*` và `MAG_*` trong `fc_config.h` để biết cách đo và số đo gần nhất.
3. **Từ kế chưa hiệu chuẩn tệ hơn là không có từ kế**: nó kéo yaw sai một cách tự tin,
   và EKF sẽ tin nó.

## Log bay

`Log/` bị `.gitignore` bỏ qua từ 2026-09-23 (125 MB và vẫn tăng). Các log cũ commit
trước mốc đó vẫn nằm trong lịch sử. Sao lưu log ra ngoài repo nếu cần giữ.
