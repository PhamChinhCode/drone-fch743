# App/ — Code ứng dụng FCH743_V1.0

## Vì sao đặt ở đây

CubeMX chỉ ghi đè các thư mục sau khi bấm **Generate Code**:

```
Core/  Drivers/  Middlewares/  FATFS/  USB_DEVICE/  cmake/stm32cubemx/
```

Toàn bộ `App/` nằm ngoài danh sách đó nên **không bao giờ bị mất**.
`CMakeLists.txt` cũng an toàn — CubeMX ghi rõ ở đầu file là chỉ sinh một lần.

Code duy nhất nằm trong vùng CubeMX quản lý là vài dòng gọi hàm trong
`Core/Src/main.c`, và chúng phải đặt giữa cặp `/* USER CODE BEGIN */` …
`/* USER CODE END */` — CubeMX giữ nguyên phần này.

---

## Cấu trúc thư mục

```
App/
├── Config/      fc_config.h      Hằng số biên dịch. Chỉ có #define.
├── Common/      fc_types.h       vec3/quaternion, hàm toán, FC_DMA_BUFFER
│                fc_time.{h,c}    micros() / millis() lấy từ TIM2
├── State/       fc_state.{h,c}   Struct trạng thái trung tâm + máy trạng thái
├── Telemetry/   tlm_protocol.*   Khung gói tin, CRC16, bộ giải mã
│                tlm_messages.h   Định nghĩa payload từng bản tin
│                tlm_stream.*     Bảng luồng + bộ lập lịch phát
│                tlm_port.*       Truyền tải: USART1 DMA hoặc USB CDC
├── Drivers/     icm20602.{h,c}   IMU 6 trục trên SPI1, đọc bằng DMA theo DRDY
│                drv_hal_callbacks.c   gom mọi HAL callback về một chỗ
│                mtf01p.{h,c}     optical flow + laser trên UART4 (MSP V2)
│                (còn thiếu)      bmp388, crsf, dshot
├── Estimator/   (trống)          bộ lọc bù / EKF, hợp nhất độ cao
├── Control/     (trống)          PID, khâu trộn động cơ
└── Storage/     (trống)          ghi log blackbox ra thẻ SD
```

Thêm file `.c` mới vào bất kỳ thư mục con nào — CMake tự phát hiện nhờ
`GLOB_RECURSE ... CONFIGURE_DEPENDS`, không phải sửa `CMakeLists.txt`.
Chỉ khi tạo **thư mục con mới** mới cần thêm một dòng vào
`target_include_directories`.

---

## Mô hình dữ liệu

Toàn hệ thống dùng chung đúng một biến: `fc_t g_fc` (khai báo trong
`fc_state.c`). Quy tắc là mỗi module chỉ ghi vào phần của mình:

| Module | Ghi vào | Đọc từ |
|---|---|---|
| Driver cảm biến | `.imu` `.baro` `.flow` `.rc` `.power` | — |
| Bộ ước lượng | `.est` | `.imu` `.baro` `.flow` |
| Bộ điều khiển | `.ctrl` | `.est` `.rc` |
| Khâu trộn động cơ | `.motor` | `.ctrl` |
| Telemetry / Log | — | tất cả (chỉ đọc) |

Nhờ vậy không có biến toàn cục rải rác, và khi debug chỉ cần xem một struct
duy nhất là biết toàn bộ tình trạng máy bay.

### Quy ước đơn vị

Ghi rõ trong `fc_types.h`, tóm tắt:

| Đại lượng | Đơn vị | Hậu tố |
|---|---|---|
| Tốc độ góc | độ/giây | `_dps` |
| Góc | radian | `_rad` |
| Gia tốc | m/s² | `_mps2` |
| Vận tốc | m/s | `_mps` |
| Vị trí, độ cao | mét | `_m` |
| Áp suất | Pascal | `_pa` |
| Thời gian | µs hoặc ms | `_us` / `_ms` |

Tên trường **luôn** mang hậu tố đơn vị. Đây là cách rẻ nhất để tránh lỗi
nhầm đơn vị — loại lỗi đã làm rơi không ít máy bay.

### Mở rộng (ví dụ thêm la bàn)

```c
/* 1. Thêm struct vào fc_state.h, đủ 3 trường bắt buộc */
typedef struct {
    vec3f_t  mag_gauss;
    float    heading_rad;
    uint32_t timestamp_us;   /* bắt buộc */
    uint16_t error_count;    /* bắt buộc */
    bool     healthy;        /* bắt buộc */
} mag_data_t;

/* 2. Thêm trường vào fc_t — đặt ở CUỐI */
typedef struct {
    ...
    fc_mode_t  mode;
    mag_data_t mag;          /* <-- thêm ở đây */
} fc_t;

/* 3. SENSOR_MAG đã có sẵn bit trong fc_sensor_mask_t */
```

Không có dòng code cũ nào phải sửa.

---

## Telemetry

### Khung gói tin

```
byte 0    SYNC0 = 0xFE
byte 1    SYNC1 = 0x5A
byte 2    LEN   = số byte payload (0..64)
byte 3    ID    = mã bản tin
byte 4    SEQ   = bộ đếm tăng dần (phát hiện mất gói)
byte 5..  PAYLOAD
2 byte cuối     CRC-16/CCITT-FALSE trên byte 2 → hết payload
```

Phụ phí 7 byte/gói. Little-endian, không cần đảo byte ở phía PC.

### Gửi "một phần" dữ liệu

Mỗi bản tin là một *luồng* có chu kỳ riêng, khai báo trong bảng
`g_tlm_streams[]`. Đặt `period_ms = 0` là tắt luồng đó. Có 4 hồ sơ dựng sẵn:

| Hồ sơ | Nội dung | Băng thông |
|---|---|---|
| `TLM_PROFILE_SILENT` | chỉ heartbeat 1 Hz | ~25 B/s |
| `TLM_PROFILE_FLIGHT` | heartbeat + tư thế 10 Hz + pin + RC | ~350 B/s |
| `TLM_PROFILE_TUNING` | thêm PID 100 Hz, IMU 50 Hz | ~4 kB/s |
| `TLM_PROFILE_DEBUG` | bật tất cả | ~9 kB/s |

`FLIGHT` đủ nhẹ để nhét vào đường telemetry ELRS. `DEBUG` dùng khi cắm USB.

Đổi lúc đang chạy:

```c
tlm_stream_apply_profile(TLM_PROFILE_TUNING);
tlm_stream_set_period(TLM_MSG_FLOW, 50);     /* bật riêng luồng flow 20 Hz */
tlm_stream_send_text(0, "gyro calib xong");
```

Máy tính cũng có thể tự bật/tắt luồng bằng lệnh `TLM_MSG_CMD_SET_RATE`.

### Thêm bản tin mới

1. Cấp ID mới ở **cuối** `tlm_msg_id_t` (không tái dùng mã cũ).
2. Khai báo struct `packed` trong `tlm_messages.h`.
3. Viết hàm `pack_xxx()` và thêm một dòng vào `g_tlm_streams[]`.
4. Thêm một cột vào bảng `table[][]` trong `tlm_stream_apply_profile()`.

---

## Nối vào `main.c`

Chỉ chèn vào các khối `USER CODE`, CubeMX sẽ giữ nguyên khi Generate lại.

```c
/* USER CODE BEGIN Includes */
#include "fc_state.h"
#include "fc_time.h"
#include "icm20602.h"
#include "tlm_stream.h"
#include "tlm_port.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 2  — sau tất cả MX_xxx_Init() */
  fc_state_init();
  fc_time_init();                       /* TIM2 chạy -> micros() dùng được */

  tlm_port_init(TLM_PORT_UART);
  tlm_stream_init();
  tlm_stream_apply_profile(TLM_PROFILE_FLIGHT);
  tlm_stream_send_text(0, "FCH743 boot");

  if (icm20602_init()) {                /* chặn ~150 ms */
      icm20602_start();
      fc_state_set_mode(FC_MODE_CALIBRATING);
      icm20602_start_gyro_calibration();
      tlm_stream_send_text(0, "ICM20602 OK, dang hieu chuan gyro");
  } else {
      fc_state_set_mode(FC_MODE_FAULT);
      tlm_stream_send_text(2, "ICM20602 FAIL");
  }
/* USER CODE END 2 */

/* USER CODE BEGIN WHILE */
  while (1) {
      const uint32_t now_ms = HAL_GetTick();

      g_fc.sys.uptime_ms = now_ms;
      fc_state_update_health(micros());

      /* Hiệu chuẩn xong thì chuyển sang trạng thái sẵn sàng. */
      if (g_fc.mode == FC_MODE_CALIBRATING && g_fc.imu.calibrated) {
          fc_state_set_mode(FC_MODE_DISARMED);
          tlm_stream_send_text(0, "Gyro calib xong");
      }

      tlm_stream_update(now_ms);
      tlm_port_flush();
/* USER CODE END WHILE */
```

**Không cần thêm callback nào vào `main.c`.** Toàn bộ `HAL_GPIO_EXTI_Callback`,
`HAL_SPI_TxRxCpltCallback`, `HAL_SPI_ErrorCallback` và `HAL_UART_TxCpltCallback`
đã nằm trong `App/Drivers/drv_hal_callbacks.c`.

---

## Driver ICM20602

### Đường dữ liệu

```
Chip có mẫu mới  →  chân INT (PC4) lên  →  EXTI4 (ưu tiên 0)
                                              ↓
                          icm20602_drdy_isr(): CS xuống, ghi mốc micros(),
                          phát lệnh đọc 15 byte bằng DMA rồi trả CPU ngay
                                              ↓
                    DMA1_S0/S1 xong  →  icm20602_spi_complete_isr():
                    CS lên, tách số, đổi thang, xoay trục, lọc,
                    ghi thẳng vào g_fc.imu
```

Sau khi `icm20602_init()` chạy xong, không còn hàm nào chặn. Vòng lặp chính
chỉ việc đọc `g_fc.imu`.

### Cấu hình đã áp dụng

| Mục | Giá trị | Lý do |
|---|---|---|
| SPI lúc init | 1 MHz | ghi thanh ghi cho chắc, dù chip chịu được 10 MHz |
| SPI lúc đọc | 8 MHz | trong giới hạn 10 MHz của ICM-20602 |
| Gyro | ±2000 °/s, DLPF 250 Hz, ODR 8 kHz | chuẩn cho vòng điều khiển 4–8 kHz |
| Accel | ±16 g, DLPF 218 Hz | đủ dải cho động tác mạnh |
| `USER_CTRL` | `I2C_IF_DIS` | **bắt buộc** — không tắt I2C thì chip có thể tự chuyển sang I2C khi gặp nhiễu và ngừng đáp ứng SPI |
| `INT_PIN_CFG` | `0x00` | xung 50 µs, tích cực cao, tự xoá — hợp EXTI sườn lên + PULLDOWN trên PC4 |

Mọi thanh ghi cấu hình đều được **ghi rồi đọc lại để kiểm tra**; sai một
thanh ghi là `icm20602_init()` trả về `false`.

### Xoay trục — việc bạn phải tự kiểm chứng

Chip dán trên mạch hầu như không bao giờ trùng hệ trục thân máy bay.
Sửa `IMU_AXIS_MAP_*` và `IMU_AXIS_SIGN_*` trong `fc_config.h`.

Quy ước thân: **X = mũi trước, Y = cánh phải, Z = hướng xuống**.

Cách kiểm tra bằng luồng telemetry `TLM_MSG_IMU`:

| Động tác | Kết quả đúng |
|---|---|
| Để yên trên bàn | `accel_mg[2]` ≈ **−1000** (≈ −9,81 m/s²) — dấu âm vì Z hướng xuống |
| Lăn phải (roll) | `gyro_ddps[0]` dương |
| Chúc mũi lên (pitch) | `gyro_ddps[1]` dương |
| Xoay phải (yaw) | `gyro_ddps[2]` dương |

Sai dấu thì đổi `IMU_AXIS_SIGN_*`; sai thứ tự trục thì đổi `IMU_AXIS_MAP_*`.
**Chưa kiểm chứng xong thì tuyệt đối không gắn cánh quạt.**

### Hiệu chuẩn bias gyro

`icm20602_start_gyro_calibration()` không chặn. Quá trình chạy trong ngắt,
lấy 2000 mẫu (~250 ms ở 8 kHz). Nếu biên độ dao động vượt
`IMU_CALIB_MOVE_LIMIT_DPS` (máy bay bị rung) thì **tự khởi động lại từ đầu**
— nên nếu không bao giờ xong thì là do đặt trên mặt phẳng không đủ yên.

Theo dõi bằng `icm20602_calibration_progress()` (0..100).

### Chẩn đoán khi không chạy

| Triệu chứng | Nguyên nhân thường gặp |
|---|---|
| `icm20602_init()` trả `false` | sai chân CS, SPI chưa init, hoặc WHO_AM_I ≠ `0x12` |
| `g_fc.imu.sample_count` đứng yên | chân INT chưa nối, EXTI4 chưa bật, hoặc quên gọi `icm20602_start()` |
| `error_count` tăng nhanh | bộ đệm DMA không nằm ở AXI SRAM, hoặc SPI quá nhanh |
| Số liệu toàn 0 | MISO hở — driver tự đếm vào `error_count` và bỏ mẫu |
| `dt_us` không đều ~125 µs | ngắt khác giữ CPU quá lâu; xem lại bản đồ ưu tiên |

---

## Driver MTF-01P

### Giao thức

MTF-01P mặc định xuất **MSP V2** ở 115200 8N1 trên UART4 (PA1 = RX), lặp
lại hai bản tin:

| Mã | Tên | Payload |
|---|---|---|
| `0x1F01` | `MSP2_SENSOR_RANGEFINDER` | `uint8 quality` + `int32 distance_mm` (5 byte) |
| `0x1F02` | `MSP2_SENSOR_OPTIC_FLOW` | `uint8 quality` + `int32 motion_x` + `int32 motion_y` (9 byte) |

Khung: `'$' 'X' '<' flag func_lo func_hi size_lo size_hi payload... crc8`
với CRC8 kiểu DVB-S2 (đa thức `0xD5`) tính từ `flag` tới hết payload.
`distance_mm` âm nghĩa là ngoài tầm đo.

> Dòng MTF-01 còn cấu hình được sang MAVLink bằng phần mềm của hãng. Nếu
> parser không ra dữ liệu, bật `DBG_MODE_FLOW_RAW` để xem byte thực tế.

### Không dùng ngắt

UART4 chạy DMA vòng tròn liên tục; `mtf01p_update()` gọi từ vòng lặp chính
sẽ rút byte mới ra khỏi đệm rồi nạp vào bộ phân tích. Đệm 256 byte ứng với
~160 ms dữ liệu (100 Hz × ~16 byte), vòng lặp chỉ cần gọi nhanh hơn mức đó.
Chỉ có một ngắt duy nhất: `HAL_UART_ErrorCallback` khởi động lại DMA khi
gặp lỗi khung hoặc tràn đệm — thiếu nó thì luồng dữ liệu đứng vĩnh viễn.

### `FLOW_RAD_PER_COUNT` — cần hiệu chuẩn

Hệ số trong `fc_config.h` **chưa đúng**, mới chỉ là điểm khởi đầu. Cách đo:

1. Giữ cảm biến ở độ cao cố định đã biết, ví dụ **1,00 m**
2. Rê ngang đúng **1,00 m** trong khoảng 2 giây
3. Cộng dồn `flow_x_raw` suốt quãng đường đó
4. `FLOW_RAD_PER_COUNT = (quãng đường / độ cao) / tổng số đếm`

Vận tốc suạt ra: `v = (số_đếm × FLOW_RAD_PER_COUNT / dt) × độ_cao`.

Driver chỉ tính `velocity_mps` khi **có số đo khoảng cách hợp lệ và
`quality >= FLOW_QUALITY_MIN`**; thiếu một trong hai thì trả về 0 thay vì số rác.

---

## Lưu ý phần cứng quan trọng

### Bộ đệm DMA phải khai báo bằng `FC_DMA_BUFFER`

Đây là cái bẫy lớn nhất của STM32H7 trong dự án này.

Trong `STM32H743XX_FLASH.ld`, **cả `.bss`, `.data` lẫn stack đều nằm ở
`DTCMRAM (0x20000000)`**. DTCM chỉ nối trực tiếp với lõi Cortex-M7 —
**DMA1 và DMA2 không truy cập được vùng này**. Nghĩa là biến toàn cục,
biến `static` thông thường và mảng cục bộ đều *không* dùng làm bộ đệm DMA
được: transfer sẽ ra dữ liệu rác hoặc gây bus fault.

Vì vậy `.ld` đã được thêm section `.dma_buffer` trỏ vào
`RAM (0x24000000, AXI SRAM)` — vùng DMA đọc ghi bình thường và luôn có clock
(khác `RAM_D2` phải bật `RCC_AHB2ENR` trước).

```c
FC_DMA_BUFFER static uint8_t rx_buf[256];   /* -> 0x24000000, DMA OK   */
static uint8_t                bad_buf[256]; /* -> 0x20000xxx, DMA HỎNG */
```

Đã kiểm chứng bằng bản đồ ký hiệu:

```
24000000 b s_tx_buf     <- AXI SRAM, DMA truy cập được
24000800 b s_rx_buf
20000010 B g_fc         <- DTCM, chỉ CPU đụng tới nên để đây cho nhanh
```

Hai điểm cần nhớ:
- Section là `NOLOAD` nên biến **không được startup code xoá về 0**. Hãy
  `memset()` trong hàm init của driver (`tlm_port_init()` đã làm).
- Không gán giá trị khởi tạo cho biến dùng macro này.
- `SDMMC1` dùng IDMA riêng và `QUADSPI` dùng MDMA — hai khối này truy cập
  được DTCM nên bộ đệm của chúng không bắt buộc dùng macro.

> ⚠️ **CubeMX ghi đè `STM32H743XX_FLASH.ld` mỗi lần Generate Code.**
> Nếu sau khi generate mà build báo lỗi `section .dma_buffer will not fit`
> hoặc các biến DMA quay về `0x200xxxxx`, hãy dán lại khối `.dma_buffer`
> vào `.ld` (nằm ngay sau section `.bss`).

**D-Cache đang tắt** (`main.c` chỉ gọi `SCB_EnableICache()`), nên **không cần**
`SCB_CleanDCache_by_Addr` / `InvalidateDCache_by_Addr`. Nếu sau này bật
D-Cache thì phải rà lại toàn bộ đường DMA.

**Không gọi `HAL_Delay()` trong ISR.** SysTick ở mức ưu tiên 15 (thấp nhất),
mọi ngắt khác đều chặn nó → treo cứng.

---

## Bản đồ ưu tiên ngắt

| Mức | Nguồn |
|---|---|
| 0 | EXTI4 (ICM20602 DRDY), DMA1_S0/S1 (SPI1), SPI1 |
| 1 | DMA1_S2 + TIM1_UP (DShot600) |
| 2 | DMA1_S3/S4 + USART2 (CRSF) |
| 3 | DMA1_S5 + UART4 (MTF01P) |
| 5 | I2C1_EV/ER (BMP388) |
| 8 | SDMMC1 |
| 10 | DMA2_S0/S1 + USART1 (telemetry/debug) |
| 12 | OTG_FS (USB) |
| 15 | SysTick |
