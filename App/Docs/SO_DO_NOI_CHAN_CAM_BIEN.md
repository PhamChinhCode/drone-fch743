# Sơ đồ nối chân và hướng lắp cảm biến — FCH743_V1.0

MCU: **STM32H743VIT6** (LQFP100), SYSCLK 480 MHz.

> **2026-09-29:** bus cảm biến I2C chuyển từ I2C1 (PB8/PB7) sang **I2C2 (PB10 SCL / PB11 SDA)** cho bo FC mới. Firmware hiện tại chỉ chạy trên bo mới.

Tài liệu này được **đọc ngược từ code** (ngày 2026-09-28), không phải từ sơ đồ
mạch. Nguồn:

| Thông tin | Lấy từ |
|---|---|
| Chân, chức năng thay thế (AF) | `FCH743_V1.0.ioc`, `Core/Src/stm32h7xx_hal_msp.c`, `Core/Src/main.c` |
| UART nào dùng cho thiết bị nào | `App/Drivers/*.c`, `App/Telemetry/tlm_port.c`, `App/Mavlink/mav_port.c`, `App/Drivers/drv_hal_callbacks.c` |
| Hướng trục, vị trí lắp | `App/Config/fc_config.h` (khối `*_AXIS_*`, `FLOW_OFFSET_*`, `DSHOT_MOTOR_MAP_*`) |

> Hướng trục và dấu trong `fc_config.h` chỉ là **mặc định lúc biên dịch**. Những
> giá trị có trong bảng tham số (ví dụ `mag_axis_*`, `mag_offset_*`) nếu đã
> `save` vào flash sẽ ghi đè lúc khởi động. Muốn biết giá trị đang chạy thật
> thì đọc bằng console, không đọc file này.

---

## 1. Quy ước hệ trục thân

Toàn bộ firmware dùng hệ **NED thân** (Front-Right-Down):

```
            X (mũi, trước)
                ▲
                │
     M2 ●───────┼───────● M1
        (FL)    │    (FR)
                │
   ─────────────┼────────────▶ Y (cánh phải)
                │
        (RL)    │    (RR)
     M3 ●───────┼───────● M4
                │
                        Z hướng XUỐNG đất (đi vào mặt giấy)
```

- Roll dương = nghiêng cánh phải xuống. Pitch dương = ngóc mũi lên. Yaw dương =
  mũi quay sang phải (thuận chiều kim đồng hồ nhìn từ trên xuống).
- Nằm yên thăng bằng: accel Z thân ≈ **−9,81 m/s²** (gia tốc kế đo lực riêng,
  hướng lên). Bộ ước lượng tự đảo bằng `EST_ACCEL_Z_SIGN (-1)`.

Mỗi cảm biến được xoay sang hệ này bằng hai bộ hằng số:

```
trục_thân[i] = SIGN_i × trục_cảm_biến[ MAP_i ]      (MAP: 0 = X, 1 = Y, 2 = Z)
```

Ma trận xoay luôn phải có **định thức +1** — không được sửa lẻ một dấu.

---

## 2. Tổng quan cảm biến và bus

| Cảm biến | Vai trò | Bus | Chân MCU | Địa chỉ / CS | Driver |
|---|---|---|---|---|---|
| **ICM-42688-P** | IMU chính (vòng PID 4 kHz + EKF) | SPI1 | PA5 SCK, PA6 MISO, PA7 MOSI | CS = **PA4**, INT1/DRDY = **PC4** (EXTI4) | `icm42688.c` |
| **LSM6DSV** | IMU phụ (chỉ ghi `g_fc.imu2`, không vào điều khiển) | SPI3 | PB3 SCK, PB4 MISO, PD6 MOSI | CS = **PA15**, DRDY = **PD7** (EXTI9_5) | `lsm6dsv.c` |
| **BMP388** | Khí áp kế (độ cao) | **I2C2** 400 kHz | **PB10 SCL, PB11 SDA** | 0x77 (SDO → 3V3, CS → 3V3) | `bmp388.c` |
| **IST8310** | La bàn đang dùng (trên module GPS) | **I2C2** (chung bus với BMP388) | **PB10 SCL, PB11 SDA** | 0x0E (dự phòng 0x0C) | `mag_i2c.c` |
| **u-blox NEO-F10N** (MicoAir MG-F10-A) | GPS | UART7, 115200 | PE7 RX ← TX module, PE8 TX → RX module | — | `gps_ubx.c` |
| **MTF-01P** | Optical flow (PMW3901) + đo xa laser | UART4, 115200, MSP V2 | PA1 RX ← TX cảm biến, PA0 TX → RX cảm biến | — | `mtf01p.c` |
| Máy thu **ExpressLRS** | Điều khiển RC (CRSF) | USART2, 420000 | PA3 RX, PA2 TX | — | `crsf.c` |
| **W25Q64** | Flash ghi log 8 MB | QUADSPI (bank 1) | PB2 CLK, **PB6 NCS**, PD11 IO0, PD12 IO1, PE2 IO2, PD13 IO3 | — | `qspi_flash.c` |

Hai chip SPI dùng chung nhịp: ghi cấu hình ở 1 MHz (prescaler /64), sau đó
chạy dữ liệu ở 8 MHz (prescaler /8) bằng DMA, kích bởi ngắt DRDY.

### Chip la bàn dự phòng

`mag_i2c.c` hỗ trợ bốn chip trên cùng I2C2. Chọn bằng `MAG_I2C_USE_GPS_MAG`:

| Chip | Địa chỉ | Vị trí | Trạng thái |
|---|---|---|---|
| IST8310 | 0x0E | Trên module GPS | **Đã hỏng** (2026-09-29), không còn trả lời |
| QMC5883P | 0x2C | Trên bo | **Đang dùng** từ 2026-09-29 (`MAG_I2C_USE_GPS_MAG 0`), **chưa hiệu chuẩn, chưa đo trục** trên bo mới |
| QMC5883L | 0x0D | Module rời | Driver có sẵn |
| HMC5883L | 0x1E | Module rời | Driver có sẵn |
| QMC6309 | (sensor hub của LSM6DSV) | Trên module LSM6DSV | **Đã hỏng**, `MAG_SOURCE_SHUB` không còn dùng |

---

## 3. Hướng lắp từng cảm biến

### 3.1 ICM-42688-P — IMU chính (SPI1)

```c
IMU_AXIS_MAP  = 1, 0, 2
IMU_AXIS_SIGN = +1, +1, -1
```

| Trục thân | = trục chip | Nghĩa là |
|---|---|---|
| X (mũi) | **+Y chip** | Mũi tên Y trên chip chỉ về **MŨI** |
| Y (phải) | **+X chip** | Mũi tên X trên chip chỉ sang **PHẢI** |
| Z (xuống) | **−Z chip** | Z của chip chỉ **LÊN** (chip nằm ngửa, mặt trên hướng lên) |

```
      nhìn từ TRÊN xuống            mũi ▲
                                        │
               Y chip ▲                  │
                      │
                      └──▶ X chip        (Z chip hướng ra khỏi mặt giấy = lên)
```

Kiểm chứng: đo 2026-09-21 bằng `DBG_MODE_IMU_CMP` (mode 24), xoay tay quanh
từng trục và so gyro với LSM6DSV, 986 mẫu, hệ số tương quan ±1,00 cả ba trục.

> Lịch sử: ICM20602 cũ (trước 2026-09-21) có trục chip X = mũi, Y = trái,
> Z = lên (`MAP 0,1,2 / SIGN +1,-1,-1`). Chip mới quay 90° quanh trục đứng so
> với chip cũ.

### 3.2 LSM6DSV — IMU phụ (SPI3)

```c
IMU2_AXIS_MAP  = 1, 0, 2
IMU2_AXIS_SIGN = -1, -1, -1
```

| Trục thân | = trục chip | Nghĩa là |
|---|---|---|
| X (mũi) | **−Y chip** | Y chip chỉ về phía **SAU** (đuôi) |
| Y (phải) | **−X chip** | X chip chỉ sang **TRÁI** |
| Z (xuống) | **−Z chip** | Z chip chỉ **LÊN** |

```
      nhìn từ TRÊN xuống            mũi ▲
                                        │
               X chip ◀──┐
                         │
                         ▼ Y chip       (Z chip hướng lên)
```

Tức là module LSM6DSV quay **180° quanh trục đứng** so với ICM-42688-P.
Module lắp gần như song song mặt bo (lệch 0,16° / 0,91°).

Kiểm chứng: 2026-08-26 bằng `DBG_MODE_AXISCAL`, ba tư thế (nằm phẳng / dựng
mũi lên / nghiêng cánh phải xuống), đo tay lại ở 90° cho cùng kết quả.

### 3.3 IST8310 — la bàn trên module GPS (I2C2)

```c
MAG_AXIS_MAP  = 0, 1, 2
MAG_AXIS_SIGN = -1, -1, +1
```

| Trục thân | = trục chip | Nghĩa là |
|---|---|---|
| X (mũi) | **−X chip** | X chip chỉ về phía **SAU** |
| Y (phải) | **−Y chip** | Y chip chỉ sang **TRÁI** |
| Z (xuống) | **+Z chip** | Z chip chỉ **XUỐNG** (trùng chiều thân) |

- **Vị trí:** trên cột GPS, cách động cơ > 20 cm, bắt cứng vào khung.
- Kiểm chứng 2026-09-22 ngoài trời: khớp động học theo gyro ở hai lượt xoay độc
  lập; so với hướng đi GPS 4 hướng lệch +7 / 0 / −2 / −2°.
- Hiệu chuẩn sắt cứng/sắt mềm (`MAG_OFFSET_*`, `MAG_SCALE_*`, `MAG_SOFT_*`) tính
  trong **hệ cảm biến, trước khi xoay trục**. Dời module GPS, đổi cách bắt, hay
  đi lại dây nguồn thì **phải hiệu chuẩn lại**.

> Bộ trục này ban đầu đo cho QMC5883P trên bo (2026-09-04, "xoay 180° quanh
> trục đứng") và được xác nhận lại đúng cho IST8310 ngày 2026-09-22.

### 3.4 BMP388 — khí áp kế (I2C2)

Khí áp kế không có hướng trục. Chỉ cần:

- **SDO → 3V3** (địa chỉ 0x77; nếu nối GND thì là 0x76 và phải sửa `BARO_I2C_ADDR_7BIT`).
- **CS → 3V3** để chip chạy chế độ I2C.
- Nên che xốp để tránh gió cánh quạt.

### 3.5 MTF-01P — optical flow + laser (UART4)

```c
FLOW_AXIS_MAP  = 0, 1
FLOW_AXIS_SIGN = -1, -1
FLOW_OFFSET_X_M = -0.067   /* lùi về đuôi 67 mm */
FLOW_OFFSET_Z_M = +0.019   /* thấp hơn tâm 19 mm */
```

- **Vị trí:** mặt kính hướng **xuống đất**, nằm đúng đường giữa thân (Y = 0),
  **lùi về đuôi 67 mm** và **thấp hơn tâm 19 mm**. Firmware dùng hai số này để
  trừ phần vận tốc ảo do quay (v = v_tâm + ω × r).
- Số đếm flow được hiểu là **tốc độ góc** quanh trục thân: rê máy bay sang
  phải → thay đổi hiện trên trục X của cảm biến (ω_x = +v_y/h). Cả hai trục
  đều phải đảo dấu:
  - Rê sang **phải** cho đếm X **âm** → `SIGN_X = −1`.
  - Rê **tới** cho `vb_x` **âm** (`DBG_MODE_VEL`) → `SIGN_Y = −1`.
- Hệ số `FLOW_RAD_PER_COUNT = 0.00185` chốt bằng phép thử nghiêng (so với gyro).
- Dải laser dùng tới 8 m; bộ ước lượng chỉ dùng flow ở 0,20–5,0 m và nghiêng < 20°.

### 3.6 GPS MG-F10-A (UART7)

- Module u-blox NEO-F10N (L1 + L5), chạy 10 Hz, mô hình động học airborne < 1g.
- Lắp trên cột, anten hướng lên trời. Cùng module chứa la bàn IST8310 (mục 3.3)
  nhưng la bàn đi **I2C2**, không liên quan tới UART7.
- Driver tự dò baud và gửi cấu hình vào RAM của module mỗi lần khởi động
  (không ghi flash module).

---

## 4. Động cơ (DShot600 trên TIM1)

| Motor | Vị trí khung | Kênh | Chân |
|---|---|---|---|
| M1 | Trước-phải | TIM1_CH1 | **PE9** |
| M2 | Trước-trái | TIM1_CH2 | **PE11** |
| M3 | Sau-trái | TIM1_CH3 | **PE13** |
| M4 | Sau-phải | TIM1_CH4 | **PE14** |

- Đổi thứ tự bằng `DSHOT_MOTOR_MAP_1..4` thay vì tháo dây.
- Chiều quay: cặp chéo {M1, M3} cùng chiều, {M2, M4} cùng chiều ngược lại.
  Với `MIX_YAW_SIGN (+1)`, {M1, M3} phải quay **ngược chiều kim đồng hồ** (nhìn
  từ trên). Chiều của M2 và M4 đã được sửa bằng **hoán dây pha** (ESC BLHeli_S
  không hiểu lệnh DShot đảo chiều).

---

## 5. Cổng giao tiếp (UART / USB)

| Cổng | Chân | Baud | Nối với | Dùng cho |
|---|---|---|---|---|
| USART1 | PA9 TX, PA10 RX | 921600 | Máy tính (qua USB-UART, COM4) | Console chữ `dbg_console` + CLI |
| USART2 | PA2 TX, PA3 RX | 420000 | Máy thu ExpressLRS | CRSF |
| USART3 | PD8 TX, PD9 RX (pull-up) | 921600 | ESP32 "air" | Telemetry nhị phân → ESP-NOW → máy tính |
| UART4 | PA0 TX, PA1 RX | 115200 | MTF-01P | Optical flow + laser |
| UART7 | PE8 TX, PE7 RX (pull-up) | 115200 | GPS MG-F10-A | UBX |
| UART8 | PE1 TX, PE0 RX | 921600 | Máy tính nhúng (Pi 5, ROS2) | MAVLink |
| USB OTG FS | PA11 DM, PA12 DP | — | Máy tính | CDC (xuất log) / MSC |

Quy ước nối UART: **TX của FC → RX thiết bị, RX của FC ← TX thiết bị**, chung GND.

---

## 6. Ngoại vi khác trên bo

| Chức năng | Chân | Ghi chú |
|---|---|---|
| Nút K1 | PE3 (pull-up, nhấn = 0) | Quay thử motor, vào USB MSC |
| Nút K2 | PC5 (pull-up, nhấn = 0) | Gửi lệnh đảo chiều motor (DShot) |
| SD card | SDMMC1: PC8–PC11 D0–D3, PC12 CK, PD2 CMD | 4-bit |
| Thạch anh | PH0/PH1 (HSE), PC14/PC15 (LSE 32 kHz) | |
| SWD | PA13 SWDIO, PA14 SWCLK | Nạp bằng ST-Link / OpenOCD |

### Chân đã khai báo trong CubeMX nhưng App chưa dùng

| Chân | Nhãn | Ghi chú |
|---|---|---|
| PC0, PC1 | ADC1_INP10/11 | ADC đã cấu hình, chưa có code đọc pin/dòng (`PWR_VBAT_DIVIDER`, `PWR_CURRENT_MV_PER_A` là số chờ hiệu chỉnh) |
| PC2_C, PC3_C | ADC3_INP0/1 | Như trên |
| PC6 | BUZZER (TIM8_CH1) | |
| PC7 | WS2812_LED (TIM3_CH2) | |
| PD14, PD15 | SERVO1/2 (TIM4_CH3/4) | |
| PB9, PB5 | I2C1_INT1, I2C1_INT2 | Chân ngắt I2C, chưa gắn ISR (tên nhãn còn giữ từ thời I2C1) |
| PB7, PB8 | — | Trước 2026-09-29 là I2C1 SDA/SCL, nay bỏ trống |
| PB12–PB15 | SPI2 SS/SCK/MISO/MOSI | Khối SPI2 không được bật |
| PE4, PC13 | SPI4_SS, SPI4_INT | Dự phòng |
| PE15 | I2C2_INT | Dự phòng |
| PB0, PB1 | LCD_BLK, LCD_RS | Di sản bo DevEBox |

---

## 7. Những điểm cần lưu ý

1. **Chân NCS của QSPI:** firmware (`.ioc` / MSP) dùng **PB6**. Ghi chú vụ cứu
   log 2026-09-23 cho biết trên bo H743 thay thế, chip W25Q64 được nối NCS vào
   **PB10**. Nếu chạy firmware này trên bo đó thì phải sửa chân NCS trong
   CubeMX, nếu không log QSPI sẽ không hoạt động.
2. Chú thích ở khối LSM6DSV trong `fc_config.h` vẫn ghi "vòng PID chạy bằng
   ICM20602" — thực tế IMU chính đã là ICM-42688-P từ 2026-09-21.
3. Sau khi đổi chip hoặc tháo lắp lại bất kỳ cảm biến nào, kiểm lại:
   - Nằm yên: accel Z ≈ −9,81; ngóc mũi lên: pitch dương; nghiêng phải: roll dương.
   - La bàn: mũi chỉ bắc → hướng ≈ 0°; xoay 360° → chỉ trục Z đứng yên.
   - Flow: đẩy máy bay về trước → `velocity_mps.x` dương.
