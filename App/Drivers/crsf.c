/**
 * @file    crsf.c
 * @brief   Hiện thực driver CRSF / ExpressLRS (USART2, DMA vòng tròn).
 */

#include "crsf.h"
#include "fc_state.h"
#include "fc_time.h"
#include "main.h"

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef  hdma_usart2_rx;

/* ==========================================================================
 * Bộ đệm
 *
 * s_rx là đích trực tiếp của DMA1_Stream3 nên bắt buộc dùng FC_DMA_BUFFER
 * để nằm ở AXI SRAM. Xem chú thích trong fc_types.h.
 * ========================================================================== */

FC_DMA_BUFFER static uint8_t s_rx[RC_RX_BUFFER_SIZE];

static uint16_t s_tail;                 /* vị trí đã đọc tới trong đệm DMA */

/* Vòng lưu byte gần nhất, chỉ để soi bằng mắt khi dò giao thức. */
static uint8_t  s_raw[CRSF_RAW_SNAPSHOT_LEN];
static uint8_t  s_raw_pos;

static uint32_t s_bytes_rx;
static uint32_t s_frames_ok;
static uint32_t s_crc_errors;
static uint32_t s_rc_frames;
static uint32_t s_prev_rc_us;
static uint32_t s_frame_interval_us;

/* ==========================================================================
 * Bộ phân tích khung
 * ========================================================================== */

typedef enum {
    CRSF_IDLE = 0,      /**< chờ byte địa chỉ mở đầu khung   */
    CRSF_LENGTH,        /**< đang đọc byte độ dài            */
    CRSF_BODY           /**< đang gom kiểu + payload + crc   */
} crsf_state_t;

static struct {
    crsf_state_t state;
    uint8_t      length;                        /* kiểu + payload + crc    */
    uint8_t      index;
    uint8_t      body[CRSF_FRAME_LEN_MAX];      /* body[0] = kiểu khung    */
} s_frame;

/**
 * CRC8 kiểu DVB-S2, đa thức 0xD5 — chuẩn của CRSF.
 *
 * Trùng thuật toán với MSP V2 trong mtf01p.c nhưng cố tình để riêng: hai
 * driver độc lập nhau, gộp lại sẽ tạo ràng buộc giữa hai module không liên
 * quan chỉ để tiết kiệm sáu dòng.
 */
static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t byte)
{
    crc ^= byte;
    for (int i = 0; i < 8; i++) {
        const uint32_t shifted = (uint32_t)crc << 1;
        crc = (uint8_t)((crc & 0x80u) ? (shifted ^ 0xD5u) : shifted);
    }
    return crc;
}

/** true nếu byte này là một địa chỉ CRSF hợp lệ, tức có thể mở đầu khung. */
static bool is_sync_byte(uint8_t b)
{
    return (b == CRSF_ADDR_FLIGHT_CONTROLLER) ||
           (b == CRSF_ADDR_BROADCAST)         ||
           (b == CRSF_ADDR_RECEIVER)          ||
           (b == CRSF_ADDR_TRANSMITTER)       ||
           (b == CRSF_ADDR_RADIO_TRANSMITTER);
}

/* ==========================================================================
 * Chuẩn hoá giá trị cần điều khiển
 * ========================================================================== */

/** Cần hai chiều: 172..1811 -> -1..+1, có vùng chết quanh giữa. */
static float norm_symmetric(uint16_t raw)
{
    const float v = fc_mapf((float)raw,
                            (float)RC_CRSF_CHANNEL_MIN,
                            (float)RC_CRSF_CHANNEL_MAX,
                            -1.0f, 1.0f);
    return fc_deadbandf(fc_constrainf(v, -1.0f, 1.0f), RC_DEADBAND_NORM);
}

/** Cần ga: 172..1811 -> 0..1, KHÔNG có vùng chết. */
static float norm_throttle(uint16_t raw)
{
    const float v = fc_mapf((float)raw,
                            (float)RC_CRSF_CHANNEL_MIN,
                            (float)RC_CRSF_CHANNEL_MAX,
                            0.0f, 1.0f);
    return fc_constrainf(v, 0.0f, 1.0f);
}

/* ==========================================================================
 * Xử lý khung
 * ========================================================================== */

/**
 * Giải nén 16 kênh, mỗi kênh 11 bit, xếp liền nhau kiểu little-endian.
 *
 * Kênh thứ i bắt đầu ở bit 11*i. Vì 11 bit có thể nằm vắt qua ba byte nên
 * phải gom ba byte rồi mới dịch. Kênh cuối (i = 15) bắt đầu ở bit 165, tức
 * byte 20 lệch 5 bit, chỉ cần tới byte 21 — nên byte thứ ba phải kiểm tra
 * biên, nếu không sẽ đọc lố ra ngoài payload 22 byte.
 */
static void handle_rc_channels(const uint8_t *p, uint8_t size)
{
    if (size < CRSF_RC_CHANNELS_PAYLOAD) {
        g_fc.rc.error_count++;
        return;
    }

    for (uint32_t i = 0; i < RC_CHANNEL_COUNT; i++) {
        const uint32_t bit = i * 11u;
        const uint32_t idx = bit >> 3;
        const uint32_t off = bit & 7u;

        const uint32_t b2 = (idx + 2u < CRSF_RC_CHANNELS_PAYLOAD)
                          ? (uint32_t)p[idx + 2u] : 0u;
        const uint32_t v  = (uint32_t)p[idx]
                          | ((uint32_t)p[idx + 1u] << 8)
                          | (b2 << 16);

        g_fc.rc.channel_raw[i] = (uint16_t)((v >> off) & 0x07FFu);
    }

    /*
     * Đảo chiều theo cấu hình. Đặt ngay tại đây — sau chỗ này thì cả firmware
     * chỉ thấy quy ước dấu chuẩn, không module nào phải biết radio của bạn gán
     * kênh kiểu gì.
     */
    g_fc.rc.roll  = norm_symmetric(g_fc.rc.channel_raw[RC_CHANNEL_ROLL])
                  * (RC_INVERT_ROLL ? -1.0f : 1.0f);
    g_fc.rc.pitch = norm_symmetric(g_fc.rc.channel_raw[RC_CHANNEL_PITCH])
                  * (RC_INVERT_PITCH ? -1.0f : 1.0f);
    g_fc.rc.yaw   = norm_symmetric(g_fc.rc.channel_raw[RC_CHANNEL_YAW])
                  * (RC_INVERT_YAW ? -1.0f : 1.0f);

    /* Ga đảo chiều bằng cách lật quanh giữa dải, vì nó là 0..1 chứ không phải -1..1. */
    {
        const float t = norm_throttle(g_fc.rc.channel_raw[RC_CHANNEL_THROTTLE]);
        g_fc.rc.throttle = RC_INVERT_THROTTLE ? (1.0f - t) : t;
    }

    const uint32_t now_us = micros();

    /* Chu kỳ khung cho biết ELRS đang chạy ở tốc độ nào (500 Hz -> ~2000 µs). */
    if (s_rc_frames > 0u) {
        s_frame_interval_us = fc_elapsed_us(now_us, s_prev_rc_us);
    }
    s_prev_rc_us = now_us;
    s_rc_frames++;

    g_fc.rc.last_frame_us = now_us;
    g_fc.rc.frame_count++;
    g_fc.rc.failsafe = false;
    g_fc.rc.healthy  = true;
}

/**
 * Thống kê đường truyền. Bố cục 10 byte theo tài liệu CRSF:
 *   [0] RSSI ăng-ten 1 (dBm đã đổi dấu)   [5] chế độ RF
 *   [1] RSSI ăng-ten 2                    [6] mức công suất phát
 *   [2] chất lượng đường lên (%)          [7] RSSI đường xuống
 *   [3] SNR đường lên (dB)                [8] chất lượng đường xuống (%)
 *   [4] ăng-ten đang dùng                 [9] SNR đường xuống
 */
static void handle_link_statistics(const uint8_t *p, uint8_t size)
{
    if (size < CRSF_LINK_STATS_PAYLOAD) {
        g_fc.rc.error_count++;
        return;
    }

    /* Máy thu báo RSSI dưới dạng số dương đã đổi dấu: 60 nghĩa là -60 dBm. */
    const uint8_t rssi_raw = (p[4] == 0u) ? p[0] : p[1];

    /* int8_t chỉ xuống tới -128; RSSI thực tế không bao giờ tệ tới mức đó. */
    g_fc.rc.rssi_dbm = (rssi_raw > 127u) ? (int8_t)-128
                                         : (int8_t)(-(int16_t)rssi_raw);

    const uint8_t lq = (p[2] > 100u) ? 100u : p[2];
    g_fc.rc.link_quality   = lq;
    g_fc.rc.frame_loss_pct = (uint8_t)(100u - lq);
}

/**
 * Nạp một byte vào bộ phân tích.
 * @return true nếu vừa gom xong một khung và CRC đúng
 */
static bool crsf_push(uint8_t byte)
{
    switch (s_frame.state) {

    case CRSF_IDLE:
        if (is_sync_byte(byte)) {
            s_frame.state = CRSF_LENGTH;
        }
        break;

    case CRSF_LENGTH:
        /*
         * Độ dài vô lý nghĩa là byte trước đó không phải địa chỉ thật mà chỉ
         * là dữ liệu trùng giá trị. Quay về IDLE và thử lại — bộ phân tích tự
         * bắt nhịp lại sau vài khung.
         */
        if (byte < CRSF_FRAME_LEN_MIN || byte > CRSF_FRAME_LEN_MAX) {
            s_frame.state = CRSF_IDLE;
            break;
        }
        s_frame.length = byte;
        s_frame.index  = 0;
        s_frame.state  = CRSF_BODY;
        break;

    case CRSF_BODY:
        s_frame.body[s_frame.index++] = byte;
        if (s_frame.index < s_frame.length) {
            break;
        }

        s_frame.state = CRSF_IDLE;

        /* CRC tính từ byte kiểu tới hết payload, không gồm chính byte CRC. */
        {
            uint8_t crc = 0;
            for (uint8_t i = 0; i + 1u < s_frame.length; i++) {
                crc = crc8_dvb_s2(crc, s_frame.body[i]);
            }

            if (crc == s_frame.body[s_frame.length - 1u]) {
                s_frames_ok++;
                return true;
            }
        }

        s_crc_errors++;
        g_fc.rc.error_count++;
        break;

    default:
        s_frame.state = CRSF_IDLE;
        break;
    }

    return false;
}

/** Ép cần về vị trí an toàn khi mất sóng. */
static void enter_failsafe(void)
{
    g_fc.rc.failsafe = true;
    g_fc.rc.healthy  = false;

    /*
     * Giữ nguyên giá trị cần cuối cùng là cách nhanh nhất để máy bay lao đi
     * mất kiểm soát: nếu mất sóng đúng lúc đang full ga thì nó cứ thế bay
     * thẳng. Ga về 0, ba trục về giữa.
     */
    g_fc.rc.throttle = 0.0f;
    g_fc.rc.roll     = 0.0f;
    g_fc.rc.pitch    = 0.0f;
    g_fc.rc.yaw      = 0.0f;

    g_fc.rc.link_quality   = 0;
    g_fc.rc.frame_loss_pct = 100;
}

/* ==========================================================================
 * API
 * ========================================================================== */

bool crsf_init(void)
{
    /* Section .dma_buffer là NOLOAD nên không được startup code xoá. */
    memset(s_rx, 0, sizeof(s_rx));
    memset(s_raw, 0, sizeof(s_raw));
    memset(&s_frame, 0, sizeof(s_frame));

    s_tail              = 0;
    s_raw_pos           = 0;
    s_bytes_rx          = 0;
    s_frames_ok         = 0;
    s_crc_errors        = 0;
    s_rc_frames         = 0;
    s_prev_rc_us        = 0;
    s_frame_interval_us = 0;

    enter_failsafe();

    /* DMA vòng tròn chạy liên tục, không cần khởi động lại sau mỗi khung. */
    return (HAL_UART_Receive_DMA(&huart2, s_rx, RC_RX_BUFFER_SIZE) == HAL_OK);
}

uint8_t crsf_update(void)
{
    uint8_t frames = 0;

    /* DMA đếm lùi: vị trí ghi hiện tại suy ra từ số byte còn lại. */
    const uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
    const uint16_t head =
        (uint16_t)((RC_RX_BUFFER_SIZE - remaining) % RC_RX_BUFFER_SIZE);

    while (s_tail != head) {
        const uint8_t byte = s_rx[s_tail];
        s_tail = (uint16_t)((s_tail + 1u) % RC_RX_BUFFER_SIZE);

        s_bytes_rx++;
        s_raw[s_raw_pos] = byte;
        s_raw_pos = (uint8_t)((s_raw_pos + 1u) % CRSF_RAW_SNAPSHOT_LEN);

        if (crsf_push(byte)) {
            switch (s_frame.body[0]) {
            case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
                handle_rc_channels(&s_frame.body[1],
                                   (uint8_t)(s_frame.length - 2u));
                frames++;
                break;
            case CRSF_FRAMETYPE_LINK_STATISTICS:
                handle_link_statistics(&s_frame.body[1],
                                       (uint8_t)(s_frame.length - 2u));
                frames++;
                break;
            default:
                break;              /* telemetry, tham số... bỏ qua */
            }
        }
    }

    /* Quá lâu không có khung RC hợp lệ thì vào failsafe. */
    if (fc_elapsed_us(micros(), g_fc.rc.last_frame_us)
            > (RC_FAILSAFE_TIMEOUT_MS * 1000u)) {
        if (!g_fc.rc.failsafe) {
            fc_state_set_error(FC_ERR_RC_TIMEOUT);
        }
        enter_failsafe();
    }

    return frames;
}

uint32_t crsf_bytes_received(void)    { return s_bytes_rx; }
uint32_t crsf_frames_ok(void)         { return s_frames_ok; }
uint32_t crsf_crc_errors(void)        { return s_crc_errors; }
uint32_t crsf_rc_frames(void)         { return s_rc_frames; }
uint32_t crsf_frame_interval_us(void) { return s_frame_interval_us; }

uint8_t crsf_peek_raw(uint8_t *dst, uint8_t max_len)
{
    if (dst == NULL || max_len == 0u) {
        return 0;
    }

    const uint8_t n = (max_len < CRSF_RAW_SNAPSHOT_LEN)
                    ? max_len : (uint8_t)CRSF_RAW_SNAPSHOT_LEN;

    /* Chép theo thứ tự thời gian: byte cũ nhất trước. */
    for (uint8_t i = 0; i < n; i++) {
        const uint8_t idx =
            (uint8_t)((s_raw_pos + CRSF_RAW_SNAPSHOT_LEN - n + i)
                      % CRSF_RAW_SNAPSHOT_LEN);
        dst[i] = s_raw[idx];
    }
    return n;
}

void crsf_uart_error_isr(void)
{
    /*
     * Lỗi khung hoặc tràn bộ đệm sẽ khiến HAL huỷ DMA. Khởi động lại ngay,
     * nếu không luồng dữ liệu sẽ đứng vĩnh viễn — và với đường điều khiển
     * thì đứng luồng nghĩa là rơi.
     */
    HAL_UART_AbortReceive(&huart2);

    s_tail        = 0;
    s_frame.state = CRSF_IDLE;
    g_fc.rc.error_count++;

    if (HAL_UART_Receive_DMA(&huart2, s_rx, RC_RX_BUFFER_SIZE) != HAL_OK) {
        enter_failsafe();
        fc_state_set_error(FC_ERR_RC_TIMEOUT);
    }
}
