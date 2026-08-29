/**
 * @file    tlm_port.c
 * @brief   Hiện thực lớp truyền tải telemetry.
 */

#include "tlm_port.h"
#include "main.h"
#include "usbd_cdc_if.h"

/* ==========================================================================
 * Chọn UART
 *
 * Telemetry nhị phân đi ra USART3 (PD8/PD9) tới ESP32 "air", không đụng tới
 * USART1 vì console dạng chữ (dbg_console) đang giữ cổng đó.
 *
 * Muốn chuyển sang UART khác: sửa đúng ba dòng #define dưới đây, phần còn
 * lại của file không tham chiếu tên handle cụ thể nào nữa. Nhớ sửa kèm nhánh
 * lọc instance trong drv_hal_callbacks.c.
 * ========================================================================== */

#define TLM_UART             huart3
#define TLM_UART_INSTANCE    USART3
#define TLM_UART_DMA_RX      hdma_usart3_rx

/* Handle do CubeMX sinh trong main.c. */
extern UART_HandleTypeDef TLM_UART;
extern DMA_HandleTypeDef  TLM_UART_DMA_RX;

#define TLM_RX_BUFFER_SIZE   256u

/* ==========================================================================
 * Bộ đệm
 *
 * Hai mảng dưới đây là nguồn / đích trực tiếp của DMA2_S1 (TX) và DMA2_S0
 * (RX) nên PHẢI dùng FC_DMA_BUFFER để nằm ở AXI SRAM. Nếu để mặc định chúng
 * sẽ rơi vào DTCMRAM và DMA không đọc ghi được. Xem chú thích ở fc_types.h.
 *
 * Các biến con trỏ ring buffer thì chỉ CPU đụng tới, để ở DTCM cho nhanh.
 * ========================================================================== */

FC_DMA_BUFFER static uint8_t s_tx_buf[TLM_TX_BUFFER_SIZE];
FC_DMA_BUFFER static uint8_t s_rx_buf[TLM_RX_BUFFER_SIZE];

static volatile uint16_t s_tx_head;      /* vị trí ghi tiếp theo   */
static volatile uint16_t s_tx_tail;      /* vị trí đọc tiếp theo   */
static volatile uint16_t s_tx_inflight;  /* số byte DMA đang gửi   */
static volatile bool     s_tx_busy;

static uint16_t s_rx_tail;

static tlm_port_type_t s_port;
static uint32_t        s_dropped;

/* ==========================================================================
 * Tiện ích
 * ========================================================================== */

static inline uint32_t critical_enter(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void critical_exit(uint32_t primask)
{
    __set_PRIMASK(primask);
}

/** Số byte đang chờ gửi. */
static inline uint16_t tx_used(void)
{
    const uint16_t head = s_tx_head;
    const uint16_t tail = s_tx_tail;
    return (uint16_t)((head >= tail) ? (head - tail)
                                     : (TLM_TX_BUFFER_SIZE - tail + head));
}

/* ==========================================================================
 * Khởi tạo
 * ========================================================================== */

void tlm_port_init(tlm_port_type_t port)
{
    /* Section .dma_buffer là NOLOAD -> không được startup code xoá. */
    memset(s_tx_buf, 0, sizeof(s_tx_buf));
    memset(s_rx_buf, 0, sizeof(s_rx_buf));

    s_tx_head     = 0;
    s_tx_tail     = 0;
    s_tx_inflight = 0;
    s_tx_busy     = false;
    s_rx_tail     = 0;
    s_dropped     = 0;

    tlm_port_set(port);
}

void tlm_port_set(tlm_port_type_t port)
{
    s_port = port;

    if (port == TLM_PORT_UART) {
        /* RX chạy DMA vòng tròn liên tục, không cần khởi động lại. */
        HAL_UART_Receive_DMA(&TLM_UART, s_rx_buf, TLM_RX_BUFFER_SIZE);
        s_rx_tail = 0;
    }
}

tlm_port_type_t tlm_port_get(void)
{
    return s_port;
}

uint32_t tlm_port_dropped(void)
{
    return s_dropped;
}

uint16_t tlm_port_tx_free(void)
{
    /* Giữ lại 1 byte để phân biệt trạng thái đầy và rỗng. */
    return (uint16_t)(TLM_TX_BUFFER_SIZE - 1u - tx_used());
}

/* ==========================================================================
 * Gửi
 * ========================================================================== */

bool tlm_port_write(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0 || s_port == TLM_PORT_NONE) {
        return false;
    }

    /* Chỉ nhận trọn gói. Ghi một nửa sẽ làm hỏng khung ở đầu bên kia. */
    if (len > tlm_port_tx_free()) {
        s_dropped++;
        return false;
    }

    const uint32_t primask = critical_enter();

    uint16_t head = s_tx_head;
    for (uint16_t i = 0; i < len; i++) {
        s_tx_buf[head] = data[i];
        head = (uint16_t)((head + 1u) % TLM_TX_BUFFER_SIZE);
    }
    s_tx_head = head;

    critical_exit(primask);

    tlm_port_flush();
    return true;
}

/** Nạp khối liên tiếp kế tiếp cho DMA. Gọi khi đường truyền đang rảnh. */
static void tx_kick(void)
{
    if (s_tx_busy) {
        return;
    }

    const uint16_t head = s_tx_head;
    const uint16_t tail = s_tx_tail;
    if (head == tail) {
        return;                              /* không còn gì để gửi */
    }

    /* DMA chỉ gửi được vùng liên tiếp; nếu vòng qua cuối đệm thì cắt làm hai. */
    const uint16_t chunk = (head > tail) ? (uint16_t)(head - tail)
                                         : (uint16_t)(TLM_TX_BUFFER_SIZE - tail);

    s_tx_busy     = true;
    s_tx_inflight = chunk;

    HAL_StatusTypeDef st = HAL_ERROR;

    if (s_port == TLM_PORT_UART) {
        st = HAL_UART_Transmit_DMA(&TLM_UART, &s_tx_buf[tail], chunk);
    } else if (s_port == TLM_PORT_USB) {
        /* CDC_Transmit_FS chép sang đệm riêng của USB nên trả về ngay. */
        st = (CDC_Transmit_FS(&s_tx_buf[tail], chunk) == USBD_OK)
           ? HAL_OK : HAL_BUSY;
        if (st == HAL_OK) {
            /* USB không có callback hoàn tất ở đây, coi như xong ngay. */
            s_tx_tail     = (uint16_t)((tail + chunk) % TLM_TX_BUFFER_SIZE);
            s_tx_inflight = 0;
            s_tx_busy     = false;
            return;
        }
    }

    if (st != HAL_OK) {
        /* Chưa gửi được (USB chưa nối, DMA bận) — thử lại ở lần flush sau. */
        s_tx_busy     = false;
        s_tx_inflight = 0;
    }
}

void tlm_port_flush(void)
{
    const uint32_t primask = critical_enter();
    tx_kick();
    critical_exit(primask);
}

void tlm_port_tx_complete_isr(void)
{
    s_tx_tail     = (uint16_t)((s_tx_tail + s_tx_inflight) % TLM_TX_BUFFER_SIZE);
    s_tx_inflight = 0;
    s_tx_busy     = false;

    tx_kick();                               /* nạp tiếp phần còn lại */
}

/* ==========================================================================
 * Hàng đợi uplink của USB CDC
 *
 * KHÔNG nằm trong .dma_buffer: khối USB dùng FIFO riêng của nó, và dữ liệu
 * tới đây đã được HAL chép sang UserRxBufferFS rồi. Đây chỉ là bộ nhớ thường.
 *
 * s_usb_head do ngắt USB ghi, s_usb_tail do vòng lặp chính ghi — mỗi biến
 * một chủ nên không cần khoá ngắt, chỉ cần `volatile` cho biến bên kia.
 * ========================================================================== */

static tlm_port_type_t   s_last_rx = TLM_PORT_NONE;

static uint8_t           s_usb_rx[TLM_RX_BUFFER_SIZE];
static volatile uint16_t s_usb_head;
static uint16_t          s_usb_tail;
static volatile uint32_t s_usb_overruns;

void tlm_port_usb_rx(const uint8_t *data, uint32_t len)
{
    if (data == NULL) {
        return;
    }

    for (uint32_t i = 0; i < len; i++) {
        const uint16_t next = (uint16_t)((s_usb_head + 1u) % TLM_RX_BUFFER_SIZE);

        if (next == s_usb_tail) {
            /*
             * Đầy. Bỏ phần CÒN LẠI của cả khối chứ không chỉ byte này: khung
             * đã cụt rồi thì chép thêm nửa khung vào chỉ làm parser tốn công
             * đồng bộ lại. CRC sẽ loại nó, nhưng loại sớm thì rẻ hơn.
             */
            s_usb_overruns += (len - i);
            return;
        }

        s_usb_rx[s_usb_head] = data[i];
        s_usb_head = next;
    }
}

uint32_t tlm_port_usb_overruns(void)
{
    return s_usb_overruns;
}

tlm_port_type_t tlm_port_last_rx(void)
{
    return s_last_rx;
}

void tlm_port_uart_error_isr(void)
{
    /*
     * HAL đã dừng cả hai chiều khi báo lỗi. Bỏ luôn gói TX đang dở — nửa gói
     * đến nơi còn tệ hơn là mất hẳn, đầu bên kia có CRC nên chỉ mất một chu
     * kỳ phát. RX thì bắt buộc phải chạy lại, nếu không uplink chết im.
     */
    s_tx_inflight = 0;
    s_tx_busy     = false;

    if (s_port == TLM_PORT_UART) {
        HAL_UART_Receive_DMA(&TLM_UART, s_rx_buf, TLM_RX_BUFFER_SIZE);
        s_rx_tail = 0;
    }
}

/* ==========================================================================
 * Nhận
 * ========================================================================== */

uint16_t tlm_port_read(uint8_t *dst, uint16_t max_len)
{
    if (dst == NULL || max_len == 0) {
        return 0;
    }

    uint16_t n = 0;

    /*
     * USB trước. Ngắt USB đã nạp sẵn vào ring buffer; ở đây chỉ rút ra.
     *
     * Rút BẤT KỂ đang phát ra đường nào — xem giải thích trong tlm_port.h.
     */
    {
        const uint16_t head = s_usb_head;

        while (s_usb_tail != head && n < max_len) {
            dst[n++]   = s_usb_rx[s_usb_tail];
            s_usb_tail = (uint16_t)((s_usb_tail + 1u) % TLM_RX_BUFFER_SIZE);
            s_last_rx  = TLM_PORT_USB;
        }
    }

    /*
     * Rồi tới UART. DMA vòng tròn đếm lùi: vị trí ghi hiện tại suy ra từ bộ
     * đếm còn lại.
     *
     * Chỉ đọc khi DMA RX thật sự đang chạy — tlm_port_set() chỉ khởi động nó
     * cho TLM_PORT_UART, đọc bộ đếm của một DMA chưa chạy sẽ ra số vô nghĩa.
     */
    if (s_port == TLM_PORT_UART) {
        const uint16_t dma_remaining = (uint16_t)__HAL_DMA_GET_COUNTER(&TLM_UART_DMA_RX);
        const uint16_t head = (uint16_t)(TLM_RX_BUFFER_SIZE - dma_remaining);

        while (s_rx_tail != head && n < max_len) {
            dst[n++]  = s_rx_buf[s_rx_tail];
            s_rx_tail = (uint16_t)((s_rx_tail + 1u) % TLM_RX_BUFFER_SIZE);
            s_last_rx = TLM_PORT_UART;
        }
    }

    return n;
}
