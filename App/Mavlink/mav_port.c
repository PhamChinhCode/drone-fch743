/**
 * @file    mav_port.c
 * @brief   Hiện thực lớp truyền tải MAVLink trên UART8.
 *
 * Cấu trúc bám sát App/Telemetry/tlm_port.c — cùng một bài toán ring buffer
 * + DMA, chỉ bỏ nhánh USB vì đường này luôn là UART.
 */

#include "mav_port.h"
#include "main.h"

/* ==========================================================================
 * Chọn UART
 *
 * Muốn đổi sang UART khác: sửa đúng ba dòng dưới đây, phần còn lại của file
 * không nhắc tên handle cụ thể nào nữa. Nhớ sửa kèm nhánh lọc instance trong
 * drv_hal_callbacks.c.
 * ========================================================================== */

#define MAV_UART             huart8
#define MAV_UART_INSTANCE    UART8
#define MAV_UART_DMA_RX      hdma_uart8_rx

/* Handle do CubeMX sinh trong main.c. */
extern UART_HandleTypeDef MAV_UART;
extern DMA_HandleTypeDef  MAV_UART_DMA_RX;

/* ==========================================================================
 * Bộ đệm
 *
 * Hai mảng này là nguồn / đích trực tiếp của DMA2_S4 (TX) và DMA2_S5 (RX) nên
 * PHẢI dùng FC_DMA_BUFFER để nằm ở AXI SRAM. Để mặc định thì chúng rơi vào
 * DTCMRAM và DMA không đọc ghi được.
 *
 * Các con trỏ ring buffer chỉ CPU đụng tới, để ở DTCM cho nhanh.
 * ========================================================================== */

FC_DMA_BUFFER static uint8_t s_tx_buf[MAV_TX_BUFFER_SIZE];
FC_DMA_BUFFER static uint8_t s_rx_buf[MAV_RX_BUFFER_SIZE];

static volatile uint16_t s_tx_head;      /* vị trí ghi tiếp theo  */
static volatile uint16_t s_tx_tail;      /* vị trí đọc tiếp theo  */
static volatile uint16_t s_tx_inflight;  /* số byte DMA đang gửi  */
static volatile bool     s_tx_busy;

static uint16_t s_rx_tail;
static uint32_t s_dropped;
static bool     s_rx_running;   /**< DMA RX đã khởi động được hay chưa */

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
                                     : (MAV_TX_BUFFER_SIZE - tail + head));
}

/* ==========================================================================
 * Khởi tạo
 * ========================================================================== */

void mav_port_init(void)
{
    /* Section .dma_buffer là NOLOAD -> startup code không xoá giúp. */
    memset(s_tx_buf, 0, sizeof(s_tx_buf));
    memset(s_rx_buf, 0, sizeof(s_rx_buf));

    s_tx_head     = 0;
    s_tx_tail     = 0;
    s_tx_inflight = 0;
    s_tx_busy     = false;
    s_rx_tail     = 0;
    s_dropped     = 0;

    /* RX chạy DMA vòng tròn liên tục, không bao giờ cần khởi động lại. */
    s_rx_running = (HAL_UART_Receive_DMA(&MAV_UART, s_rx_buf, MAV_RX_BUFFER_SIZE) == HAL_OK);
}

uint32_t mav_port_dropped(void)
{
    return s_dropped;
}

uint16_t mav_port_tx_free(void)
{
    /* Giữ lại 1 byte để phân biệt trạng thái đầy và rỗng. */
    return (uint16_t)(MAV_TX_BUFFER_SIZE - 1u - tx_used());
}

/* ==========================================================================
 * Gửi
 * ========================================================================== */

/** Nạp khối liên tiếp kế tiếp cho DMA. Người gọi phải đang khoá ngắt. */
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

    /* DMA chỉ gửi được vùng liên tiếp; vòng qua cuối đệm thì cắt làm hai. */
    const uint16_t chunk = (head > tail) ? (uint16_t)(head - tail)
                                         : (uint16_t)(MAV_TX_BUFFER_SIZE - tail);

    s_tx_busy     = true;
    s_tx_inflight = chunk;

    if (HAL_UART_Transmit_DMA(&MAV_UART, &s_tx_buf[tail], chunk) != HAL_OK) {
        /* Chưa gửi được (DMA còn bận) — thử lại ở lần flush sau. */
        s_tx_busy     = false;
        s_tx_inflight = 0;
    }
}

bool mav_port_write(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) {
        return false;
    }

    /* Chỉ nhận trọn gói. Ghi một nửa sẽ làm hỏng khung ở đầu bên kia. */
    if (len > mav_port_tx_free()) {
        s_dropped++;
        return false;
    }

    const uint32_t primask = critical_enter();

    uint16_t head = s_tx_head;
    for (uint16_t i = 0; i < len; i++) {
        s_tx_buf[head] = data[i];
        head = (uint16_t)((head + 1u) % MAV_TX_BUFFER_SIZE);
    }
    s_tx_head = head;

    tx_kick();
    critical_exit(primask);

    return true;
}

void mav_port_flush(void)
{
    const uint32_t primask = critical_enter();
    tx_kick();
    critical_exit(primask);
}

void mav_port_tx_complete_isr(void)
{
    s_tx_tail     = (uint16_t)((s_tx_tail + s_tx_inflight) % MAV_TX_BUFFER_SIZE);
    s_tx_inflight = 0;
    s_tx_busy     = false;

    tx_kick();                               /* nạp tiếp phần còn lại */
}

void mav_port_error_isr(void)
{
    /*
     * HAL đã dừng cả hai chiều khi báo lỗi. Bỏ luôn gói TX đang dở — nửa gói
     * đến nơi còn tệ hơn mất hẳn, MAVLink có CRC nên đầu kia loại được và ta
     * chỉ mất một chu kỳ phát. RX thì bắt buộc phải chạy lại, nếu không
     * uplink chết im.
     */
    s_tx_inflight = 0;
    s_tx_busy     = false;

    s_rx_running = (HAL_UART_Receive_DMA(&MAV_UART, s_rx_buf, MAV_RX_BUFFER_SIZE) == HAL_OK);
    s_rx_tail = 0;
}

/* ==========================================================================
 * Nhận
 * ========================================================================== */

uint16_t mav_port_read(uint8_t *dst, uint16_t max_len)
{
    if (dst == NULL || max_len == 0 || !s_rx_running) {
        return 0;
    }

    /*
     * DMA vòng tròn đếm LÙI: vị trí ghi hiện tại suy ra từ bộ đếm còn lại.
     * Không có ngắt nào tham gia ở đây — vòng lặp chính là chủ duy nhất của
     * s_rx_tail, nên không cần khoá.
     */
    const uint16_t dma_remaining = (uint16_t)__HAL_DMA_GET_COUNTER(&MAV_UART_DMA_RX);
    const uint16_t head = (uint16_t)(MAV_RX_BUFFER_SIZE - dma_remaining);

    uint16_t n = 0;
    while (s_rx_tail != head && n < max_len) {
        dst[n++]  = s_rx_buf[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1u) % MAV_RX_BUFFER_SIZE);
    }

    return n;
}
