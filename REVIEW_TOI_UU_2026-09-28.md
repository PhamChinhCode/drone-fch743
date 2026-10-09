# Review code FCH743_V1.0 — phần cần tối ưu & phần dư thừa

- Ngày: 2026-09-28
- Công cụ: OpenCodeReview (`ocr delegate`) chế độ range `a1142d7..HEAD` (toàn bộ code tự viết từ commit đầu).
  OCR chọn 349 file; đã bỏ qua code sinh tự động (HAL, `mavlink_lib/`, `Middlewares/`) và tập trung vào `App/`, `Core/Src/main.c`, `tools/`.
- Phạm vi: **chỉ ghi chú, chưa sửa code.**
- Mức độ: **Cao** = ảnh hưởng rõ tới vòng bay/CPU · **Vừa** = đáng sửa · (mục Thấp bị OCR loại bỏ, không ghi).

---

## A. CẦN TỐI ƯU

### A1. [Cao] Firmware bay đang build `-O0` (Debug)
- `cmake/gcc-arm-none-eabi.cmake:31` → `CMAKE_C_FLAGS_DEBUG "-O0 -g3"`.
- `.vscode/tasks.json` task **OpenOCD Flash** nạp `build/Debug/FCH743_V1.0.elf` ⇒ mọi chuyến bay đều chạy bản không tối ưu.
- Ở `-O0` mọi biến nằm trên stack, hàm `static inline` không được inline, vòng `m66_mul` của EKF, `process_sample()` 8 kHz, định dạng float của console… đều chậm gấp 3–5 lần. Đây nhiều khả năng là nguyên nhân chính của “rate loop chậm”/`loop_overruns` đã ghi nhận.
- Đề xuất: thêm preset/tác vụ nạp bản `-O2 -g` (giữ debug symbol), hoặc đổi Debug sang `-Og -g3`. So sánh `loop_time_max_us`, `loop_overruns`, `ctrl_rate_hz()` trước/sau trên bàn.
- Lưu ý khi bật tối ưu: `g_fc` (`App/State/fc_state.c:19`) không `volatile`; hiện an toàn vì được đọc qua lời gọi hàm khác file, nhưng nếu bật LTO hoặc có vòng `while` chờ cờ do ISR đặt thì phải kiểm lại.

### A2. [Vừa] Lịch chạy PID bằng `count % RATE_LOOP_DIVIDER` làm mất nhịp khi vòng lặp chậm
- `App/Control/ctrl_rate.c:173`: `if (count == s_last_sample || (count % RATE_LOOP_DIVIDER) != 0u) return false;`
- Khi vòng `main` > 125 µs, giá trị `count` chẵn có thể bị nhảy qua ⇒ bỏ hẳn một bước PID (comment ở `main.c:954-964` đã thừa nhận).
- Đề xuất: chạy khi `(count - s_last_sample) >= RATE_LOOP_DIVIDER`. `dt` đã lấy từ timestamp thật nên vẫn đúng. Về lâu dài có thể chạy PID ngay trong `icm42688_spi_complete_isr()` hoặc từ timer để không phụ thuộc độ dài vòng `main`.

### A3. [Vừa] Ghi ring buffer TX: chép từng byte với `%` trong khi TẮT NGẮT
- `App/Mavlink/mav_port.c:147-156`, `App/Telemetry/tlm_port.c:142-150`, `App/Telemetry/dbg_console.c:126-133`.
- Cả vòng chép (gói MAVLink tới ~280 byte, dòng console ~150 byte) nằm trong `__disable_irq()` ⇒ trễ ngắt DRDY 8 kHz của IMU ⇒ jitter `timestamp_us`/`dt`.
- Đây là hàng đợi 1 producer (main) / 1 consumer (ISR): chỉ main ghi `head`, ISR chỉ ghi `tail` ⇒ phần chép KHÔNG cần khoá ngắt. Chép bằng 2 lần `memcpy` (đoạn tới cuối đệm + đoạn quấn), `__DMB()` rồi mới cập nhật `head`; chỉ khoá quanh `tx_kick()`.
- Kích thước đệm là lũy thừa 2 (1024/2048) ⇒ thay `% N` bằng `& (N-1)` nếu vẫn giữ vòng byte.

### A4. [Vừa] `flashlog` chặn vài ms NGAY TRONG LÚC BAY khi gạt công tắc log OFF
- `App/Storage/flashlog.c:331-332` gọi `flush_tail()` khi `!want`. `flush_tail()` (`:203-218`) có `qspi_flash_wait_ready(50u)` + `qspi_flash_write_page()` chặn (tPP tới 3 ms).
- Comment ở `:198` ghi “Chỉ gọi khi đã DISARM”, nhưng từ khi có `log_switch_channel` thì tắt log trong lúc ARM là hợp lệ ⇒ mất ~12 nhịp PID 4 kHz.
- Đề xuất: đệm 0xFF vào trang đang ghi, đẩy sang `s_pending` rồi để `pump()` không chặn tự xử lý ở các vòng sau; chỉ chuyển trạng thái READY khi `!s_pending`.

### A5. [Vừa] Race giữa `micros()` và timestamp IMU trong `fc_state_update_health()`
- `Core/Src/main.c:763` truyền `micros()` lấy TRƯỚC, sau đó `App/State/fc_state.c:104` mới đọc `g_fc.imu.timestamp_us`. Nếu ngắt IMU chen vào giữa, `timestamp > now` ⇒ `now - ts` tràn thành ~4,29e9 ⇒ báo “IMU timeout” giả.
- Hệ quả: bit `FC_ERR_IMU_TIMEOUT` bị bật và **không bao giờ xoá** (`fc_state_clear_error()` không có ai gọi); `SENSOR_GYRO` nhấp nháy ⇒ có thể chặn arm một vòng (`fc_state.c:206`) và MAVLink `SYS_STATUS` báo sai.
- Đề xuất: trong `sensor_alive()` coi `(int32_t)(now_us - stamp_us) < 0` là “còn mới”, hoặc đọc `micros()` sau khi đã đọc timestamp.

### A6. [Vừa] IMU phụ LSM6DSV vẫn chạy 1920 Hz ngắt + SPI DMA nhưng không ai dùng cho bay
- `App/Config/fc_config.h:858` `IMU2_ENABLE 1`, `IMU2_ODR_HZ 1920`. `g_fc.imu2` chỉ được đọc bởi `dbg_console.c` và `fc_state.c` (health). Chip lại đã ghi nhận nhiễu nền σ≈1,82 °/s (suy giảm).
- Nếu giai đoạn so sánh 2 IMU đã xong: `set imu2_enable=0` (hoặc đổi mặc định) ⇒ bớt ~1920 ngắt/s + callback HAL SPI/DMA, giảm jitter cho ICM42688.

### A7. [Vừa] Console text mặc định bật trong lúc bay
- `Core/Src/main.c:510` `dbg_console_set_mode(DBG_MODE_ARM)` ⇒ mỗi chu kỳ console dựng một dòng bằng `wr_fix()` (float → chuỗi), rất tốn ở `-O0`.
- Đề xuất: mặc định `DBG_MODE_OFF` khi bay thật (hoặc tự tắt khi ARM), bật lại qua CLI khi cần.

### A8. [Vừa] D-Cache đang tắt
- `Core/Src/main.c:207` chỉ `SCB_EnableICache()`. Biến nóng phần lớn ở DTCM nên ảnh hưởng hiện nhỏ; nhưng các đệm ở AXI SRAM (`.dma_buffer`, flashlog, MAVLink) và hằng trong flash (bảng tham số `g_param_table` 3,4 KB, `mavlink_message_crcs`) sẽ nhanh hơn khi bật.
- Muốn bật phải cấu hình MPU cho vùng `.dma_buffer` là non-cacheable (hoặc clean/invalidate thủ công). Làm SAU A1, không gấp.

---

## B. DƯ THỪA / CODE CHẾT

### B1. Driver `icm20602.c` (635 dòng) + `icm20602.h` — không còn ai gọi
- IMU chính đã là ICM42688 (`main.c:515`). Không có call site nào tới `icm20602_*` (kể cả `drv_hal_callbacks.c`). Linker `--gc-sections` loại bỏ khỏi binary nhưng vẫn biên dịch và gây nhầm lẫn.
- Đề xuất: xoá (git history vẫn giữ).

### B2. Trùng lặp code giữa các driver IMU
- `reg_read/reg_write/reg_write_verify`, `spi_set_baud`, `calibration_feed`, `align_axes`, `notch_init/notch_apply`, `fs_pick`, `*_gyro_sigma_*`, `start_gyro_calibration/calibration_progress` có bản riêng trong `icm42688.c`, `lsm6dsv.c` (và `icm20602.c`).
- Đề xuất: tách `imu_common.c` (lọc notch/LPF, hiệu chuẩn bias, xoay trục) — sửa một chỗ thay vì 2–3 chỗ dễ lệch nhau.

### B3. Ba bản sao y hệt của ring buffer UART TX
- `critical_enter/critical_exit/tx_used/tx_kick/write/tx_complete_isr` lặp lại ở `tlm_port.c`, `mav_port.c`, `dbg_console.c`. Gộp thành một module dùng chung (kết hợp luôn tối ưu A3).

### B4. Khối nút K1/K2 trong `main()` không làm gì — ✅ ĐÃ XOÁ 2026-09-28 (giữ nguyên cấu hình GPIO)
- `Core/Src/main.c:795-903`: vẫn đọc GPIO, chống dội, bắt sườn mỗi vòng, nhưng toàn bộ hành động bên trong đã bị comment (21 dòng `//`). ~100 dòng chạy mỗi vòng mà không có tác dụng.
- Đề xuất: xoá hẳn, hoặc khôi phục có chủ đích (lưu ý K1 giữ lúc khởi động cho chế độ USB MSC nằm ở chỗ khác, không phụ thuộc khối này).

### B5. Blackbox thẻ SD chiếm 256 KB RAM dù thẻ SD đã tắt
- `fc_config.h:1665` `FC_SD_ENABLE 0` nhưng `:1680` `BB_ENABLE 1` ⇒ `s_buf` 256 KB (0x40000) + `s_txt` 4,4 KB + `s_fs`/`s_fil` vẫn nằm trong AXI SRAM (một nửa RAM 512 KB), `blackbox_update()` vẫn gọi mỗi vòng (`main.c:923`).
- Kéo theo cả nhánh USB MSC / FATFS không dùng được.
- Đề xuất: `#define BB_ENABLE FC_SD_ENABLE` (hoặc bọc `#if`), trả lại RAM.

### B6. Mã sensor-hub từ kế (QMC6309 qua LSM6DSV) khi `MAG_SOURCE = I2C`
- `fc_config.h:951` chọn `MAG_SOURCE_I2C`, từ kế thật là IST8310. Khoảng `lsm6dsv.c:727-1450` (`lsm6dsv_mag_*`) + `qmc6309.h` chỉ phục vụ nhánh SHUB. Tương tự `mag_i2c.c` còn tự dò HMC5883L/QMC5883L/QMC5883P.
- Không sai, nhưng nếu phần cứng đã chốt IST8310 thì có thể gỡ để driver gọn hơn nhiều.

### B7. Hàm public không có nơi gọi
Chỉ có định nghĩa (+ khai báo header), không ai dùng:
`dshot_send_command_motor`, `dshot_command_busy`, `bmp388_reset_ground_level`, `icm42688_stop`, `icm42688_gyro_sigma_axes_dps`, `lsm6dsv_stop`, `lsm6dsv_gyro_sigma_axes_dps`, `mav_link_ok`, `fc_state_snapshot`, `fc_state_clear_error`, `dbg_console_dropped`, `dbg_console_yaw_elapsed_ms`, `param_msg_motor_test_active`, `tlm_port_dropped`, `tlm_stream_send_now`, `tlm_stream_rx_stats`, `estimator_steps`, `ctrl_rate_loops`.
(Một số là API “đối xứng” — giữ hay bỏ tuỳ ý; riêng `fc_state_clear_error` nên được DÙNG, xem A5.)

### B8. Biến/trường chỉ ghi không đọc
- `App/Estimator/estimator.c:26,188` `s_mag_updates` — chỉ tăng, không ai đọc (`ekf_attitude.c` đã có bộ đếm riêng `ekf_attitude_mag_updates()`).
- `pid_gains_t.kff` — luôn gán 0 ở `ctrl_rate.c:61,69,77`, không tham gia tính toán; `tlm_stream.c:462` cũng bỏ qua. Xoá hoặc hiện thực feed-forward thật.

### B9. Comment lỗi thời nhắc ICM20602 như IMU hiện tại
- 27 chỗ ngoài driver cũ, ví dụ `main.c:533` (“máy bay vẫn bay bằng ICM20602”), `fc_state.c:113`, `fc_config.h:840-845`, `mav_link.c:513`, `param_apply.c:30`. Dễ gây hiểu sai khi chẩn đoán — nên đổi thành ICM42688.

### B10. File backup bị commit vào git
- `.$ekf_block_diagram.drawio.bkp`, `App/Docs/.$so_do_dieu_khien.drawio.bkp`, `Log/STM32H743XX_FLASH.ld.bak` đang được track. Thêm `*.bkp` vào `.gitignore` và `git rm --cached`.

---

## C. Ghi nhận tốt (không cần sửa)
- Toán trong vòng nóng dùng hoàn toàn `float` (`sqrtf/sinf/powf`); `double` chỉ có trong hiệu chuẩn từ kế ở `dbg_console.c` — hợp lý.
- `flashlog` dùng double-buffer + `pump()` không chặn (trừ A4).
- `imu_snapshot()` khoá ngắt ngắn để đọc nhất quán gyro/accel.
- `tools/flow_check.py`: không phát hiện lỗi đáng kể (import đều được dùng, `except ValueError` cụ thể).

## D. Thứ tự đề xuất
1. A1 (build `-O2`) → đo lại vòng lặp. Nhiều mục khác có thể bớt cấp thiết sau bước này.
2. A5, A4, A2 (đúng đắn/độ trễ trong lúc bay).
3. B5, A6, A7 (trả RAM/CPU bằng cấu hình, gần như không rủi ro).
4. A3 + B3, B2, B1, B4 (dọn dẹp/refactor).
