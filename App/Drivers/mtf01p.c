/**
 * @file    mtf01p.c
 * @brief   Hiện thực driver MTF-01P (MSP V2 trên UART4).
 */

#include "mtf01p.h"
#include "fc_state.h"
#include "param_table.h"
#include "fc_time.h"
#include "main.h"

extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef  hdma_uart4_rx;

/* ==========================================================================
 * Bộ đệm
 *
 * s_rx là đích trực tiếp của DMA1_Stream5 nên bắt buộc dùng FC_DMA_BUFFER
 * để nằm ở AXI SRAM. Xem chú thích trong fc_types.h.
 * ========================================================================== */

FC_DMA_BUFFER static uint8_t s_rx[FLOW_RX_BUFFER_SIZE];

static uint16_t s_tail;                 /* vị trí đã đọc tới trong đệm DMA */

/* Vòng lưu byte gần nhất, chỉ để soi bằng mắt khi dò giao thức. */
static uint8_t  s_raw[MTF01P_RAW_SNAPSHOT_LEN];
static uint8_t  s_raw_pos;

static uint32_t s_bytes_rx;
static uint32_t s_frames_ok;
static uint32_t s_crc_errors;

/* ==========================================================================
 * Bộ phân tích MSP V2
 * ========================================================================== */

typedef enum {
    MSP_IDLE = 0,
    MSP_HEADER_X,
    MSP_HEADER_DIR,
    MSP_FLAG,
    MSP_FUNC_LO,
    MSP_FUNC_HI,
    MSP_SIZE_LO,
    MSP_SIZE_HI,
    MSP_PAYLOAD,
    MSP_CRC
} msp_state_t;

#define MSP_MAX_PAYLOAD  64u

static struct {
    msp_state_t state;
    uint16_t    function;
    uint16_t    size;
    uint16_t    index;
    uint8_t     crc;
    uint8_t     payload[MSP_MAX_PAYLOAD];
} s_msp;

/** CRC8 kiểu DVB-S2, đa thức 0xD5 — chuẩn của MSP V2. */
static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t byte)
{
    crc ^= byte;
    for (int i = 0; i < 8; i++) {
        const uint32_t shifted = (uint32_t)crc << 1;
        crc = (uint8_t)((crc & 0x80u) ? (shifted ^ 0xD5u) : shifted);
    }
    return crc;
}

/* ==========================================================================
 * Payload các bản tin cảm biến (định dạng của INAV)
 * ========================================================================== */

typedef struct __attribute__((packed)) {
    uint8_t quality;        /**< 0..255                                   */
    int32_t distance_mm;    /**< giá trị âm nghĩa là ngoài tầm đo         */
} msp_rangefinder_t;        /* 5 byte */

typedef struct __attribute__((packed)) {
    uint8_t quality;        /**< 0..255                                   */
    int32_t motion_x;       /**< số đếm dịch chuyển kể từ gói trước       */
    int32_t motion_y;
} msp_opflow_t;             /* 9 byte */

/* ==========================================================================
 * Xử lý bản tin
 * ========================================================================== */

static void handle_rangefinder(const uint8_t *payload, uint16_t size)
{
    if (size < sizeof(msp_rangefinder_t)) {
        g_fc.flow.error_count++;
        return;
    }

    msp_rangefinder_t m;
    memcpy(&m, payload, sizeof(m));

    g_fc.flow.range_timestamp_us = micros();

    /*
     * Cảm biến trả số âm khi mục tiêu ngoài tầm. Giữ lại giá trị đo cuối cùng
     * nhưng hạ cờ range_valid, để tầng ước lượng biết mà không dùng số này.
     */
    if (m.distance_mm < 0 || m.distance_mm > g_params.flow_range_max_mm) {
        g_fc.flow.range_valid   = false;
        g_fc.flow.range_quality = 0;
    } else {
        g_fc.flow.range_mm      = (uint16_t)m.distance_mm;
        g_fc.flow.range_quality = m.quality;
        g_fc.flow.range_valid   = true;
    }
}

static void handle_opflow(const uint8_t *payload, uint16_t size)
{
    if (size < sizeof(msp_opflow_t)) {
        g_fc.flow.error_count++;
        return;
    }

    msp_opflow_t m;
    memcpy(&m, payload, sizeof(m));

    const uint32_t now_us = micros();
    const uint32_t dt_us  = fc_elapsed_us(now_us, g_fc.flow.timestamp_us);

    /* Số đếm thô có thể vượt int16 khi rê rất nhanh — chặn lại cho gọn. */
    const int32_t raw_sensor[2] = { m.motion_x, m.motion_y };

    const int32_t rx = raw_sensor[g_params.flow_axis_map_x] *
                       (int32_t)g_params.flow_axis_sign_x;
    const int32_t ry = raw_sensor[g_params.flow_axis_map_y] *
                       (int32_t)g_params.flow_axis_sign_y;

    g_fc.flow.flow_x_raw  = (int16_t)fc_constrain_i32(rx, -32768, 32767);
    g_fc.flow.flow_y_raw  = (int16_t)fc_constrain_i32(ry, -32768, 32767);
    g_fc.flow.flow_quality = m.quality;

    /*
     * Quy đổi sang vận tốc thân:
     *   tốc độ góc [rad/s] = số_đếm * flow_rad_per_count / dt
     *   vận tốc     [m/s]  = tốc độ góc * độ cao
     *
     * Chỉ tính khi có số đo khoảng cách hợp lệ và chất lượng đủ tốt; thiếu
     * một trong hai thì vận tốc là vô nghĩa nên trả về 0 thay vì số rác.
     */
    if (g_fc.flow.range_valid &&
        m.quality >= g_params.flow_quality_min &&
        dt_us > 0u && dt_us < 200000u) {

        const float dt_s     = (float)dt_us * 1.0e-6f;
        const float height_m = (float)g_fc.flow.range_mm * 0.001f;
        const float k        = (g_params.flow_rad_per_count / dt_s) * height_m;

        /*
         * DOI TRUC SANG QUY UOC THAN: x = TOI TRUOC, y = SANG PHAI.
         *
         * Hai kenh tho khong theo quy uoc do. Cam bien duoc lap sao cho
         * ekf_velocity nhan duoc dung thu no can:
         *
         *     bay PHAI -> flow_x DUONG        (vy =  h*(flow_x + gyro_x))
         *     bay TOI  -> flow_y AM           (vx = -h*(flow_y + gyro_y))
         *
         * Nen o day phai hoan vi va doi dau thi ten truong moi dung nghia.
         * TRUOC DAY gan thang rx->x va ry->y, va chu thich ghi "day may bay
         * ve phia truoc thi velocity_mps.x phai duong" - ca hai deu SAI:
         * .x thuc ra la van toc sang phai, .y la am cua van toc tien.
         *
         * KHONG dung cho dieu khien - estimator.c doc thang flow_x_raw va
         * flow_y_raw. Cho nay chi phuc vu hien thi va telemetry, nhung mot
         * cai nhan sai thi lam nguoi doc ket luan sai, va no da lam that.
         */
        g_fc.flow.velocity_mps.x = -(float)ry * k;   /* toi truoc */
        g_fc.flow.velocity_mps.y =  (float)rx * k;   /* sang phai */
        g_fc.flow.velocity_mps.z = 0.0f;
    } else {
        g_fc.flow.velocity_mps = (vec3f_t){ 0.0f, 0.0f, 0.0f };
    }

    g_fc.flow.timestamp_us = now_us;
    g_fc.flow.sample_count++;
    g_fc.flow.healthy = true;
}

/** Trả về true nếu vừa xử lý xong một khung hợp lệ. */
static bool msp_push(uint8_t byte)
{
    switch (s_msp.state) {

    case MSP_IDLE:
        if (byte == '$') {
            s_msp.state = MSP_HEADER_X;
        }
        break;

    case MSP_HEADER_X:
        /* Chỉ nhận MSP V2 ('X'); MSP V1 ('M') dùng CRC khác, cảm biến không phát. */
        s_msp.state = (byte == 'X') ? MSP_HEADER_DIR
                    : (byte == '$') ? MSP_HEADER_X
                                    : MSP_IDLE;
        break;

    case MSP_HEADER_DIR:
        /* '<' tới FC, '>' trả lời, '!' báo lỗi — chấp nhận cả ba. */
        s_msp.state = (byte == '<' || byte == '>' || byte == '!')
                    ? MSP_FLAG : MSP_IDLE;
        break;

    case MSP_FLAG:
        s_msp.crc   = crc8_dvb_s2(0, byte);
        s_msp.state = MSP_FUNC_LO;
        break;

    case MSP_FUNC_LO:
        s_msp.function = byte;
        s_msp.crc      = crc8_dvb_s2(s_msp.crc, byte);
        s_msp.state    = MSP_FUNC_HI;
        break;

    case MSP_FUNC_HI:
        s_msp.function |= (uint16_t)byte << 8;
        s_msp.crc       = crc8_dvb_s2(s_msp.crc, byte);
        s_msp.state     = MSP_SIZE_LO;
        break;

    case MSP_SIZE_LO:
        s_msp.size  = byte;
        s_msp.crc   = crc8_dvb_s2(s_msp.crc, byte);
        s_msp.state = MSP_SIZE_HI;
        break;

    case MSP_SIZE_HI:
        s_msp.size |= (uint16_t)byte << 8;
        s_msp.crc   = crc8_dvb_s2(s_msp.crc, byte);

        if (s_msp.size > MSP_MAX_PAYLOAD) {
            s_msp.state = MSP_IDLE;         /* gói lạ, bỏ qua */
            break;
        }
        s_msp.index = 0;
        s_msp.state = (s_msp.size > 0u) ? MSP_PAYLOAD : MSP_CRC;
        break;

    case MSP_PAYLOAD:
        s_msp.payload[s_msp.index++] = byte;
        s_msp.crc = crc8_dvb_s2(s_msp.crc, byte);
        if (s_msp.index >= s_msp.size) {
            s_msp.state = MSP_CRC;
        }
        break;

    case MSP_CRC:
        s_msp.state = MSP_IDLE;
        if (byte == s_msp.crc) {
            s_frames_ok++;
            return true;
        }
        s_crc_errors++;
        g_fc.flow.error_count++;
        break;

    default:
        s_msp.state = MSP_IDLE;
        break;
    }

    return false;
}

/* ==========================================================================
 * API
 * ========================================================================== */

bool mtf01p_init(void)
{
    /* Section .dma_buffer là NOLOAD nên không được startup code xoá. */
    memset(s_rx, 0, sizeof(s_rx));
    memset(s_raw, 0, sizeof(s_raw));
    memset(&s_msp, 0, sizeof(s_msp));

    s_tail       = 0;
    s_raw_pos    = 0;
    s_bytes_rx   = 0;
    s_frames_ok  = 0;
    s_crc_errors = 0;

    g_fc.flow.healthy     = false;
    g_fc.flow.range_valid = false;

    /* DMA vòng tròn chạy liên tục, không cần khởi động lại sau mỗi gói. */
    return (HAL_UART_Receive_DMA(&huart4, s_rx, FLOW_RX_BUFFER_SIZE) == HAL_OK);
}

uint8_t mtf01p_update(void)
{
    uint8_t frames = 0;

    /* DMA đếm lùi: vị trí ghi hiện tại suy ra từ số byte còn lại. */
    const uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_uart4_rx);
    const uint16_t head =
        (uint16_t)((FLOW_RX_BUFFER_SIZE - remaining) % FLOW_RX_BUFFER_SIZE);

    while (s_tail != head) {
        const uint8_t byte = s_rx[s_tail];
        s_tail = (uint16_t)((s_tail + 1u) % FLOW_RX_BUFFER_SIZE);

        s_bytes_rx++;
        s_raw[s_raw_pos] = byte;
        s_raw_pos = (uint8_t)((s_raw_pos + 1u) % MTF01P_RAW_SNAPSHOT_LEN);

        if (msp_push(byte)) {
            switch (s_msp.function) {
            case MSP2_SENSOR_RANGEFINDER:
                handle_rangefinder(s_msp.payload, s_msp.size);
                frames++;
                break;
            case MSP2_SENSOR_OPTIC_FLOW:
                handle_opflow(s_msp.payload, s_msp.size);
                frames++;
                break;
            default:
                break;              /* bản tin khác, bỏ qua */
            }
        }
    }

    /*
     * Quá lâu không có gói hợp lệ thì hạ cờ. Hai luồng xét RIÊNG: trước đây
     * range_valid chỉ hết hạn theo gói flow, nên luồng khoảng cách ngừng mà
     * flow còn chạy thì range_mm cũ vẫn được coi là hợp lệ mãi.
     */
    const uint32_t now_us = micros();
    if (fc_elapsed_us(now_us, g_fc.flow.timestamp_us)
            > (FLOW_RANGE_TIMEOUT_MS * 1000u)) {
        g_fc.flow.healthy     = false;
        g_fc.flow.range_valid = false;
    }
    if (fc_elapsed_us(now_us, g_fc.flow.range_timestamp_us)
            > (FLOW_RANGE_TIMEOUT_MS * 1000u)) {
        g_fc.flow.range_valid = false;
    }

    return frames;
}

uint32_t mtf01p_bytes_received(void) { return s_bytes_rx; }
uint32_t mtf01p_frames_ok(void)      { return s_frames_ok; }
uint32_t mtf01p_crc_errors(void)     { return s_crc_errors; }

uint8_t mtf01p_peek_raw(uint8_t *dst, uint8_t max_len)
{
    if (dst == NULL || max_len == 0u) {
        return 0;
    }

    const uint8_t n = (max_len < MTF01P_RAW_SNAPSHOT_LEN)
                    ? max_len : (uint8_t)MTF01P_RAW_SNAPSHOT_LEN;

    /* Chép theo thứ tự thời gian: byte cũ nhất trước. */
    for (uint8_t i = 0; i < n; i++) {
        const uint8_t idx =
            (uint8_t)((s_raw_pos + MTF01P_RAW_SNAPSHOT_LEN - n + i)
                      % MTF01P_RAW_SNAPSHOT_LEN);
        dst[i] = s_raw[idx];
    }
    return n;
}

void mtf01p_uart_error_isr(void)
{
    /*
     * Lỗi khung hoặc tràn bộ đệm sẽ khiến HAL huỷ DMA. Khởi động lại ngay,
     * nếu không luồng dữ liệu sẽ đứng vĩnh viễn.
     */
    HAL_UART_AbortReceive(&huart4);

    s_tail      = 0;
    s_msp.state = MSP_IDLE;
    g_fc.flow.error_count++;

    if (HAL_UART_Receive_DMA(&huart4, s_rx, FLOW_RX_BUFFER_SIZE) != HAL_OK) {
        g_fc.flow.healthy = false;
        fc_state_set_error(FC_ERR_FLOW_TIMEOUT);
    }
}
