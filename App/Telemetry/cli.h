/**
 * @file    cli.h
 * @brief   Dòng lệnh chỉnh tham số, chạy trên console chữ ở USART1.
 *
 * DÙNG ĐỂ LÀM GÌ
 *
 *   Đây là mặt tiền đầu tiên của hệ tham số runtime: mở PuTTY / Tera Term ở
 *   921600 baud là chỉnh và lưu được cấu hình, không cần phần mềm riêng nào.
 *
 *   Về sau khi app PC ra đời, nó gửi đúng những chuỗi lệnh này qua bản tin
 *   nhị phân và nhận lại đúng những dòng chữ này. Nhờ vậy tab CLI của app gần
 *   như không tốn thêm công, và quan trọng hơn: khi app cư xử lạ, ta gõ tay
 *   đúng lệnh đó vào PuTTY để biết lỗi nằm ở firmware hay ở app.
 *
 * LỆNH
 *
 *   help                  danh sách lệnh
 *   version               board, firmware, số tham số, table_crc
 *   status                arm, cờ lỗi, sức khoẻ cảm biến, thời gian vòng lặp
 *   get [tiền_tố]         in tham số khớp tiền tố, kèm min/max/mặc định
 *   set <tên>=<giá_trị>   đặt giá trị, in lại con số firmware THỰC SỰ nhận
 *   dump                  in tất cả dưới dạng lệnh `set`
 *   diff                  chỉ in tham số khác mặc định
 *   defaults              nạp lại mặc định (CHƯA ghi flash)
 *   save                  ghi flash
 *
 * VÌ SAO `set` IN LẠI GIÁ TRỊ
 *
 *   Giá trị bị kẹp về [min, max] mà không báo gì là cách êm ái nhất để người
 *   dùng tin rằng mình đã đặt 50 trong khi firmware đang chạy 20. In lại con
 *   số thật làm việc kẹp lộ ra ngay. App PC sẽ theo đúng quy ước này.
 *
 * ĐƯỜNG NHẬP
 *
 *   USART1 RX (PA10) qua DMA2_Stream0 vòng tròn — CubeMX đã cấu hình sẵn
 *   stream này từ trước, chỉ chưa ai dùng. Cùng kiểu với USART3 trong
 *   tlm_port.c và USART2 trong crsf.c.
 *
 * ĐIỀU TIẾT ĐẦU RA
 *
 *   `dump` in 132 dòng (~5 KB) trong khi bộ đệm console chỉ có
 *   DBG_TX_BUFFER_SIZE = 1 KB. Nên việc in được chia nhỏ qua nhiều lần gọi
 *   cli_update(), mỗi lần chỉ in khi còn chỗ trống. Đẩy hết một lượt thì
 *   phần thừa bị bỏ lặng lẽ và bản dump thiếu dòng — đúng loại hỏng mà người
 *   ta chỉ phát hiện ra sau khi đã nạp firmware mới và mất cấu hình.
 */
#ifndef CLI_H
#define CLI_H

#include "fc_types.h"
#include "fc_config.h"
#include "main.h"

/**
 * Noi nhan dau ra cua CLI.
 *
 * VI SAO PHAI TRUU TUONG HOA
 *
 *   Cung mot bo ma lenh phuc vu HAI duong: console chu tren USART1 (nguoi
 *   doc bang mat trong PuTTY) va khung nhi phan TLM_MSG_CLI_LINE (app PC
 *   doc). Neu hai duong co hai ban hien thuc rieng thi chung se lech nhau,
 *   va luc app cu xu la ta khong con cach nao biet loi nam o dau.
 *
 *   `free_space` la duong PHAN HOI NGUOC: lenh `dump` in 130 dong (~5 KB)
 *   trong khi ca hai duong deu chi co bo dem vai KB. CLI hoi con bao nhieu
 *   cho truoc moi dong va dung lai khi het, in tiep o lan goi sau. Khong co
 *   no thi ban dump thieu dong mot cach lang le - dung loai hong ma nguoi ta
 *   chi phat hien ra sau khi da nap firmware moi va mat cau hinh.
 */
typedef struct {
    void     (*write)(const char *line);  /**< in mot dong, tu them xuong dong */
    uint16_t (*free_space)(void);         /**< so byte con trong o dau ra      */
} cli_sink_t;

/**
 * Khởi động đường nhận. Gọi SAU dbg_console_init() và sau param_store_load().
 * @param huart cùng UART với console (hiện là &huart1).
 */
void cli_init(UART_HandleTypeDef *huart);

/**
 * Rút byte nhận được, thực thi lệnh đã đủ dòng, và in tiếp phần còn dở của
 * lệnh liệt kê. Gọi đều đặn trong vòng lặp chính, cạnh dbg_console_update().
 */
void cli_update(void);

/**
 * Thực thi một dòng lệnh đã hoàn chỉnh.
 *
 * Tách riêng khỏi phần nhận để đường nhị phân từ app PC gọi lại được cùng bộ
 * mã này. Đầu ra vẫn đi qua console.
 *
 * @return false nếu không nhận ra lệnh.
 */
bool cli_execute(const char *line);

/**
 * Nhu cli_execute() nhung dua ket qua ra `sink` thay vi console.
 *
 * Sink duoc GHI NHO cho ca phan in dang do: lenh `dump` trai qua nhieu lan
 * goi cli_update(), va tat ca phai di ve cung mot noi. Lenh moi den thi sink
 * doi theo lenh do.
 *
 * @param sink NULL de dung console.
 */
bool cli_execute_ex(const char *line, const cli_sink_t *sink);

/** true neu con dong chua in het (dump/diff/get dang do). */
bool cli_output_pending(void);

/**
 * Định dạng số thực ra chuỗi, tối đa 9 chữ số có nghĩa, tự chọn dạng thập
 * phân hay dạng mũ.
 *
 * KHÔNG dùng printf: newlib-nano mặc định bỏ hỗ trợ %f và in ra chuỗi rỗng
 * mà không báo lỗi gì (xem ghi chú đầu dbg_console.h).
 *
 * 9 chữ số là số ÍT NHẤT bảo đảm một `float` bất kỳ in ra rồi đọc lại cho ra
 * đúng bit cũ. Ít hơn thì rate_pid_roll_kd = 1e-8 in thành "0.000000" và bản
 * dump không khôi phục lại được cấu hình.
 *
 * @return số ký tự đã ghi (không kể '\0').
 */
int cli_format_float(char *out, int cap, float value);

#endif /* CLI_H */
