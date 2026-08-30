/**
 * @file    param_store.h
 * @brief   Lưu và nạp lại g_params từ flash nội của STM32H743.
 *
 * BỐ TRÍ FLASH
 *
 *   H743 có 2 MB chia hai bank, mỗi bank 8 sector 128 KB. Firmware hiện chiếm
 *   ~230 KB nên nằm gọn trong bank 1. Hai sector CUỐI của bank 2 để trống và
 *   được dùng làm hai ô lưu cấu hình:
 *
 *       Slot A  0x081C0000  bank 2, sector 6, 128 KB
 *       Slot B  0x081E0000  bank 2, sector 7, 128 KB
 *
 *   Dùng cả một sector 128 KB cho vài trăm byte trông rất phí, nhưng 128 KB
 *   là ĐƠN VỊ XOÁ NHỎ NHẤT của chip này — không có lựa chọn nào khác, và
 *   2 MB thì thừa chỗ.
 *
 * VÌ SAO HAI Ô CHỨ KHÔNG PHẢI MỘT
 *
 *   Ghi luân phiên (ping-pong). Lúc lưu, ta xoá và ghi vào ô KHÔNG chứa bản
 *   mới nhất, còn bản cũ vẫn nguyên vẹn suốt quá trình. Mất điện giữa chừng
 *   thì lần khởi động sau vẫn nạp được bản cũ.
 *
 *   Một ô duy nhất thì có một cửa sổ vài giây kể từ lúc bắt đầu xoá cho tới
 *   khi ghi xong, mà trong cửa sổ đó mất điện là mất sạch cấu hình.
 *
 *   Ô nào mới hơn xác định bằng trường `seq` tăng dần, không phải bằng thứ tự
 *   địa chỉ.
 *
 * CHẶN CỨNG TRONG LINKER
 *
 *   STM32H743XX_FLASH.ld khai báo FLASH LENGTH = 1792K thay vì 2048K. Firmware
 *   phình tới mức đè lên vùng này thì LINKER BÁO LỖI LÚC BUILD, thay vì im
 *   lặng phá cấu hình lúc chạy. Đổi địa chỉ ở đây thì phải đổi cả bên đó.
 *
 * AN TOÀN
 *
 *   KHÔNG BAO GIỜ lưu khi đang ARM. Xoá một sector 128 KB mất hàng trăm mili
 *   giây tới vài giây và HAL_FLASHEx_Erase() chờ bận suốt thời gian đó. Vòng
 *   lặp chính đứng lại nghĩa là ngừng phát khung DShot, ESC coi như mất tín
 *   hiệu và cắt motor. Giữa không trung thì đó là rơi.
 *
 *   (Bản thân việc xoá bank 2 KHÔNG chặn lệnh đọc từ bank 1 — H743 cho phép
 *   đọc bank này trong khi ghi bank kia. Nhưng hàm HAL vẫn chờ bận, nên kết
 *   quả với vòng lặp chính là như nhau.)
 *
 * ĐỔI BẢNG THAM SỐ THÌ CẤU HÌNH CŨ ĐI ĐÂU
 *
 *   Khối lưu chỉ chứa GIÁ TRỊ, không chứa tên. Thêm hay bớt một tham số làm
 *   mọi offset phía sau dịch đi, nên dữ liệu cũ không còn đọc được nữa. Điều
 *   này phát hiện bằng `table_crc` và khi đó cấu hình cũ bị BỎ, quay về mặc
 *   định — nạp sai offset còn nguy hiểm hơn nhiều so với mất tinh chỉnh.
 *
 *   Cách sống chung: dùng lệnh CLI `diff` xuất cấu hình ra chữ trước khi nạp
 *   firmware mới, rồi dán lại sau. Chuỗi `set ten=gia_tri` bám theo TÊN nên
 *   sống sót qua mọi thay đổi bố cục.
 */
#ifndef PARAM_STORE_H
#define PARAM_STORE_H

#include "fc_types.h"
#include "param_table.h"

/**
 * Tăng số này mỗi khi sửa param_list.h.
 *
 * Thực ra `table_crc` đã tự bắt được mọi thay đổi rồi, nên trường version là
 * lớp phòng thủ thứ hai và là chỗ để ép bỏ cấu hình cũ một cách có chủ ý —
 * ví dụ khi ý NGHĨA của một tham số đổi mà tên, kiểu và giới hạn giữ nguyên
 * (đổi đơn vị, đổi quy ước dấu). CRC không nhìn thấy loại thay đổi đó.
 */
#define PARAM_SCHEMA_VERSION  8u

typedef enum {
    PARAM_STORE_OK = 0,      /**< đã nạp / đã ghi thành công          */
    PARAM_STORE_EMPTY,       /**< flash trắng, đang chạy mặc định     */
    PARAM_STORE_CORRUPT,     /**< có dữ liệu nhưng CRC sai            */
    PARAM_STORE_MISMATCH,    /**< thuộc firmware khác, đã bỏ          */
    PARAM_STORE_ERR_ARMED,   /**< từ chối: đang ARM                   */
    PARAM_STORE_ERR_FLASH,   /**< HAL báo lỗi khi xoá hoặc ghi        */
    PARAM_STORE_ERR_VERIFY   /**< ghi xong đọc lại không khớp         */
} param_store_result_t;

/**
 * Đọc hai ô, chọn ô hợp lệ có `seq` lớn nhất và nạp đè lên g_params.
 *
 * GỌI SAU param_load_defaults() và TRƯỚC mọi hàm *_init() khác — các driver
 * đọc g_params ngay trong hàm init của chúng.
 *
 * g_params KHÔNG bị đụng tới khi trả về bất cứ giá trị nào khác OK, nên giá
 * trị mặc định vẫn còn nguyên và máy bay vẫn bay được.
 */
param_store_result_t param_store_load(void);

/**
 * Ghi g_params vào ô không chứa bản mới nhất, rồi đọc lại để kiểm.
 *
 * TỰ TỪ CHỐI khi đang ARM (trả PARAM_STORE_ERR_ARMED). Đây là lớp chặn thứ
 * hai — tlm_stream.c đã chặn sẵn ở tầng lệnh — vì hậu quả của việc quên chặn
 * là mất motor giữa không trung.
 */
param_store_result_t param_store_save(void);

/** Số thứ tự của bản đang dùng. 0 nghĩa là chưa từng lưu. */
uint32_t param_store_seq(void);

/** Tên ngắn để in ra console và gửi kèm bản tin chữ. */
const char *param_store_result_name(param_store_result_t result);

#endif /* PARAM_STORE_H */
