/**
 * @file    gps_ubx.c
 * @brief   Hiện thực driver GPS u-blox NEO-F10N (UBX trên UART7).
 */

#include "gps_ubx.h"
#include "fc_state.h"
#include "fc_time.h"
#include "main.h"

#if GPS_ENABLE

extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef  hdma_uart7_rx;

/* ==========================================================================
 * Hằng số UBX
 * ========================================================================== */

#define UBX_SYNC1            0xB5u
#define UBX_SYNC2            0x62u

#define UBX_CLASS_NAV        0x01u
#define UBX_ID_NAV_PVT       0x07u
#define UBX_NAV_PVT_LEN      92u

#define UBX_CLASS_ACK        0x05u
#define UBX_ID_ACK_NAK       0x00u
#define UBX_ID_ACK_ACK       0x01u

#define UBX_CLASS_CFG        0x06u
#define UBX_ID_CFG_VALSET    0x8Au

/*
 * Khoá cấu hình (u-blox M10/F10 interface description). Bit 28..30 của khoá
 * mã hoá cỡ giá trị: 1 = bit (lưu 1 byte), 2 = 1 byte, 3 = 2 byte, 4 = 4 byte.
 */
#define CFG_UART1_BAUDRATE           0x40520001u  /* U4 */
#define CFG_UART1INPROT_UBX          0x10730001u  /* L  */
#define CFG_UART1OUTPROT_UBX         0x10740001u  /* L  */
#define CFG_UART1OUTPROT_NMEA        0x10740002u  /* L  */
#define CFG_MSGOUT_UBX_NAV_PVT_UART1 0x20910007u  /* U1 */
#define CFG_RATE_MEAS                0x30210001u  /* U2, ms */
#define CFG_RATE_NAV                 0x30210002u  /* U2, số lần đo mỗi nghiệm */
#define CFG_NAVSPG_DYNMODEL          0x20110021u  /* E1 */

#define CFG_LAYER_RAM        0x01u

/* Payload lớn nhất giữ lại. Khung dài hơn vẫn được kiểm checksum rồi bỏ. */
#define UBX_MAX_PAYLOAD      100u

/* ==========================================================================
 * Dò baud
 * ========================================================================== */

/* Thứ tự: mặc định xuất xưởng trước, rồi các baud ArduPilot/PX4 hay lưu. */
static const uint32_t k_baud_list[] = {
    115200u, 230400u, 38400u, 9600u, 460800u, 57600u
};
#define BAUD_COUNT  (sizeof(k_baud_list) / sizeof(k_baud_list[0]))

/* Chờ bao lâu ở GPS_BAUD cho khung UBX hợp lệ đầu tiên trước khi đổi baud. */
#define PROBE_WAIT_MS        1200u

typedef enum {
    PROBE_SEND = 0,     /* đặt baud thử, gửi cấu hình                       */
    PROBE_SWITCH,       /* chờ gửi xong, về GPS_BAUD, gửi lại cấu hình      */
    PROBE_WAIT          /* chờ khung UBX hợp lệ                             */
} probe_phase_t;

/* ==========================================================================
 * Trạng thái
 * ========================================================================== */

/* Đích của DMA2_Stream6 -> bắt buộc ở AXI SRAM. Xem fc_types.h. */
FC_DMA_BUFFER static uint8_t s_rx[GPS_RX_BUFFER_SIZE];

/* Đệm gửi phải sống tới khi ngắt TX xong, nên không để trên stack. */
static uint8_t  s_tx[64];

static uint16_t s_tail;

static gps_state_t   s_state;
static probe_phase_t s_phase;
static uint32_t      s_phase_ms;
static uint8_t       s_baud_idx;
static uint32_t      s_baud;
static uint32_t      s_frames_at_switch;
static uint32_t      s_running_since_ms;

static volatile bool     s_uart_err_pending;
static volatile uint32_t s_uart_errors;

static uint32_t s_bytes_rx;
static uint32_t s_frames_ok;
static uint32_t s_ck_errors;
static uint32_t s_acks;
static uint32_t s_naks;
static uint32_t s_cfg_sent;
static uint32_t s_last_pvt_ms;
static bool     s_have_pvt;

/* ==========================================================================
 * Bộ phân tích UBX
 * ========================================================================== */

typedef enum {
    UBX_IDLE = 0,
    UBX_SYNC_2,
    UBX_CLASS,
    UBX_ID,
    UBX_LEN_LO,
    UBX_LEN_HI,
    UBX_PAYLOAD,
    UBX_CK_A,
    UBX_CK_B
} ubx_state_t;

static struct {
    ubx_state_t state;
    uint8_t     cls;
    uint8_t     id;
    uint16_t    len;
    uint16_t    index;
    uint8_t     ck_a;
    uint8_t     ck_b;
    uint8_t     payload[UBX_MAX_PAYLOAD];
} s_ubx;

static inline void ubx_ck(uint8_t byte)
{
    s_ubx.ck_a = (uint8_t)(s_ubx.ck_a + byte);
    s_ubx.ck_b = (uint8_t)(s_ubx.ck_b + s_ubx.ck_a);
}

/** Trả về true khi vừa nhận xong một khung đúng checksum. */
static bool ubx_push(uint8_t byte)
{
    switch (s_ubx.state) {

    case UBX_IDLE:
        if (byte == UBX_SYNC1) {
            s_ubx.state = UBX_SYNC_2;
        }
        break;

    case UBX_SYNC_2:
        s_ubx.state = (byte == UBX_SYNC2) ? UBX_CLASS
                    : (byte == UBX_SYNC1) ? UBX_SYNC_2
                                          : UBX_IDLE;
        break;

    case UBX_CLASS:
        s_ubx.ck_a  = 0;
        s_ubx.ck_b  = 0;
        ubx_ck(byte);
        s_ubx.cls   = byte;
        s_ubx.state = UBX_ID;
        break;

    case UBX_ID:
        ubx_ck(byte);
        s_ubx.id    = byte;
        s_ubx.state = UBX_LEN_LO;
        break;

    case UBX_LEN_LO:
        ubx_ck(byte);
        s_ubx.len   = byte;
        s_ubx.state = UBX_LEN_HI;
        break;

    case UBX_LEN_HI:
        ubx_ck(byte);
        s_ubx.len  |= (uint16_t)byte << 8;
        s_ubx.index = 0;
        /*
         * Không bản tin nào module này phát dài quá 1 kB. Độ dài vô lý gần
         * như chắc chắn là rác trùng mẫu đồng bộ — bỏ luôn, khỏi nuốt oan
         * hàng trăm byte phía sau.
         */
        if (s_ubx.len > 1024u) {
            s_ubx.state = UBX_IDLE;
            break;
        }
        s_ubx.state = (s_ubx.len > 0u) ? UBX_PAYLOAD : UBX_CK_A;
        break;

    case UBX_PAYLOAD:
        ubx_ck(byte);
        if (s_ubx.index < UBX_MAX_PAYLOAD) {
            s_ubx.payload[s_ubx.index] = byte;
        }
        s_ubx.index++;
        if (s_ubx.index >= s_ubx.len) {
            s_ubx.state = UBX_CK_A;
        }
        break;

    case UBX_CK_A:
        if (byte == s_ubx.ck_a) {
            s_ubx.state = UBX_CK_B;
        } else {
            s_ck_errors++;
            g_fc.gps.error_count++;
            s_ubx.state = UBX_IDLE;
        }
        break;

    case UBX_CK_B:
        s_ubx.state = UBX_IDLE;
        if (byte == s_ubx.ck_b) {
            s_frames_ok++;
            return true;
        }
        s_ck_errors++;
        g_fc.gps.error_count++;
        break;

    default:
        s_ubx.state = UBX_IDLE;
        break;
    }

    return false;
}

/* Đọc số little-endian từ payload. MCU cũng little-endian nên memcpy là đủ. */
static inline uint16_t rd_u16(const uint8_t *p) { uint16_t v; memcpy(&v, p, 2); return v; }
static inline uint32_t rd_u32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }
static inline int32_t  rd_i32(const uint8_t *p) { int32_t  v; memcpy(&v, p, 4); return v; }

/* ==========================================================================
 * Xử lý bản tin
 * ========================================================================== */

static void handle_nav_pvt(const uint8_t *p, uint32_t now_ms)
{
    gps_data_t *g = &g_fc.gps;

    g->itow_ms    = rd_u32(&p[0]);
    g->utc_year   = rd_u16(&p[4]);
    g->utc_month  = p[6];
    g->utc_day    = p[7];
    g->utc_hour   = p[8];
    g->utc_min    = p[9];
    g->utc_sec    = p[10];
    /* valid: bit0 validDate, bit1 validTime — cần cả hai mới dùng được. */
    g->utc_valid  = ((p[11] & 0x03u) == 0x03u);

    g->fix_type   = p[20];
    g->fix_ok     = (p[21] & 0x01u) != 0u;
    g->diff_soln  = (p[21] & 0x02u) != 0u;
    g->carr_soln  = (uint8_t)((p[21] >> 6) & 0x03u);
    g->num_sv     = p[23];

    g->lon_e7           = rd_i32(&p[24]);
    g->lat_e7           = rd_i32(&p[28]);
    g->alt_ellipsoid_mm = rd_i32(&p[32]);
    g->alt_msl_mm       = rd_i32(&p[36]);
    g->h_acc_m          = (float)rd_u32(&p[40]) * 0.001f;
    g->v_acc_m          = (float)rd_u32(&p[44]) * 0.001f;

    g->vel_ned_mps.x    = (float)rd_i32(&p[48]) * 0.001f;
    g->vel_ned_mps.y    = (float)rd_i32(&p[52]) * 0.001f;
    g->vel_ned_mps.z    = (float)rd_i32(&p[56]) * 0.001f;
    g->ground_speed_mps = (float)rd_i32(&p[60]) * 0.001f;
    g->course_deg       = (float)rd_i32(&p[64]) * 1.0e-5f;
    g->s_acc_mps        = (float)rd_u32(&p[68]) * 0.001f;
    g->course_acc_deg   = (float)rd_u32(&p[72]) * 1.0e-5f;
    g->pdop_x100        = rd_u16(&p[76]);

    g->timestamp_us = micros();
    g->sample_count++;
    g->healthy = true;

    s_last_pvt_ms = now_ms;
    s_have_pvt    = true;
}

static bool handle_frame(uint32_t now_ms)
{
    if (s_ubx.cls == UBX_CLASS_NAV && s_ubx.id == UBX_ID_NAV_PVT) {
        if (s_ubx.len != UBX_NAV_PVT_LEN) {
            g_fc.gps.error_count++;
            return false;
        }
        handle_nav_pvt(s_ubx.payload, now_ms);
        return true;
    }

    if (s_ubx.cls == UBX_CLASS_ACK && s_ubx.len == 2u &&
        s_ubx.payload[0] == UBX_CLASS_CFG &&
        s_ubx.payload[1] == UBX_ID_CFG_VALSET) {
        if (s_ubx.id == UBX_ID_ACK_ACK) {
            s_acks++;
        } else if (s_ubx.id == UBX_ID_ACK_NAK) {
            s_naks++;
        }
    }

    return false;
}

/* ==========================================================================
 * Gửi cấu hình
 * ========================================================================== */

static uint16_t put_key(uint8_t *p, uint32_t key, uint32_t value, uint8_t size)
{
    memcpy(p, &key, 4);
    memcpy(p + 4, &value, size);     /* little-endian: byte thấp trước */
    return (uint16_t)(4u + size);
}

/**
 * Đóng gói một UBX-CFG-VALSET vào s_tx và gửi bằng ngắt.
 * @return false nếu UART7 còn đang gửi dở.
 */
static bool send_config(void)
{
    if (huart7.gState != HAL_UART_STATE_READY) {
        return false;
    }

    uint8_t *pl = &s_tx[6];
    uint16_t n  = 0;

    pl[n++] = 0x00u;               /* version 0: không dùng transaction */
    pl[n++] = CFG_LAYER_RAM;
    pl[n++] = 0x00u;               /* reserved */
    pl[n++] = 0x00u;

    n += put_key(&pl[n], CFG_UART1_BAUDRATE,           GPS_BAUD,           4);
    n += put_key(&pl[n], CFG_UART1INPROT_UBX,          1u,                 1);
    n += put_key(&pl[n], CFG_UART1OUTPROT_UBX,         1u,                 1);
    n += put_key(&pl[n], CFG_UART1OUTPROT_NMEA,        0u,                 1);
    n += put_key(&pl[n], CFG_MSGOUT_UBX_NAV_PVT_UART1, 1u,                 1);
    n += put_key(&pl[n], CFG_RATE_MEAS,                GPS_MEAS_PERIOD_MS, 2);
    n += put_key(&pl[n], CFG_RATE_NAV,                 1u,                 2);
    n += put_key(&pl[n], CFG_NAVSPG_DYNMODEL,          GPS_DYNMODEL,       1);

    s_tx[0] = UBX_SYNC1;
    s_tx[1] = UBX_SYNC2;
    s_tx[2] = UBX_CLASS_CFG;
    s_tx[3] = UBX_ID_CFG_VALSET;
    s_tx[4] = (uint8_t)(n & 0xFFu);
    s_tx[5] = (uint8_t)(n >> 8);

    uint8_t a = 0, b = 0;
    for (uint16_t i = 2; i < (uint16_t)(6u + n); i++) {
        a = (uint8_t)(a + s_tx[i]);
        b = (uint8_t)(b + a);
    }
    s_tx[6u + n] = a;
    s_tx[7u + n] = b;

    if (HAL_UART_Transmit_IT(&huart7, s_tx, (uint16_t)(8u + n)) != HAL_OK) {
        return false;
    }
    s_cfg_sent++;
    return true;
}

/* ==========================================================================
 * UART
 * ========================================================================== */

static void parser_reset(void)
{
    s_ubx.state = UBX_IDLE;
}

static bool start_rx(void)
{
    s_tail = 0;
    parser_reset();
    return (HAL_UART_Receive_DMA(&huart7, s_rx, GPS_RX_BUFFER_SIZE) == HAL_OK);
}

/** Đổi baud UART7 rồi chạy lại DMA nhận. Huỷ luôn lượt gửi đang dở. */
static void set_baud(uint32_t baud)
{
    HAL_NVIC_DisableIRQ(UART7_IRQn);
    (void)HAL_UART_Abort(&huart7);

    huart7.Init.BaudRate = baud;
    if (HAL_UART_Init(&huart7) != HAL_OK) {
        g_fc.gps.error_count++;
    }
    s_baud = baud;

    HAL_NVIC_EnableIRQ(UART7_IRQn);
    (void)start_rx();
}

/* ==========================================================================
 * Máy trạng thái dò / cấu hình
 * ========================================================================== */

static void probe_restart(uint32_t now_ms)
{
    s_state    = GPS_STATE_PROBE;
    s_phase    = PROBE_SEND;
    s_phase_ms = now_ms;
    s_baud_idx = 0;
}

static void probe_step(uint32_t now_ms)
{
    switch (s_phase) {

    case PROBE_SEND: {
        const uint32_t cand = k_baud_list[s_baud_idx];
        if (s_baud != cand) {
            set_baud(cand);
        }
        if (!send_config()) {
            return;                       /* UART bận, thử lại vòng sau */
        }
        s_phase_ms = now_ms;
        if (cand == GPS_BAUD) {
            s_frames_at_switch = s_frames_ok;
            s_phase = PROBE_WAIT;
        } else {
            s_phase = PROBE_SWITCH;
        }
        break;
    }

    case PROBE_SWITCH:
        /*
         * Đợi gửi xong hẳn (60 byte ở 9600 baud mất ~62 ms) rồi thêm chút cho
         * module kịp đổi baud. Đổi UART sớm hơn thì cắt cụt lệnh đổi baud.
         */
        if (huart7.gState != HAL_UART_STATE_READY ||
            fc_elapsed_ms(now_ms, s_phase_ms) < 20u) {
            return;
        }
        set_baud(GPS_BAUD);
        if (!send_config()) {
            return;
        }
        s_phase_ms = now_ms;
        s_frames_at_switch = s_frames_ok;
        s_phase = PROBE_WAIT;
        break;

    case PROBE_WAIT:
        /*
         * Bất kỳ khung UBX đúng checksum nào (ACK hay NAV-PVT) cũng chứng tỏ
         * module đang nói đúng baud. Rác do lệch baud gần như không thể qua
         * được checksum 16 bit lẫn hai byte đồng bộ.
         */
        if (s_frames_ok != s_frames_at_switch) {
            s_state = GPS_STATE_RUNNING;
            s_running_since_ms = now_ms;
            return;
        }
        if (fc_elapsed_ms(now_ms, s_phase_ms) >= PROBE_WAIT_MS) {
            s_baud_idx = (uint8_t)((s_baud_idx + 1u) % BAUD_COUNT);
            s_phase = PROBE_SEND;
        }
        break;

    default:
        s_phase = PROBE_SEND;
        break;
    }
}

/* ==========================================================================
 * API
 * ========================================================================== */

bool gps_ubx_init(void)
{
    /* Section .dma_buffer là NOLOAD nên không được startup code xoá. */
    memset(s_rx, 0, sizeof(s_rx));
    memset(&s_ubx, 0, sizeof(s_ubx));

    s_bytes_rx = 0;
    s_frames_ok = 0;
    s_ck_errors = 0;
    s_acks = 0;
    s_naks = 0;
    s_cfg_sent = 0;
    s_uart_errors = 0;
    s_uart_err_pending = false;
    s_have_pvt = false;
    s_last_pvt_ms = 0;

    g_fc.gps.healthy = false;

    s_baud = huart7.Init.BaudRate;
    probe_restart(millis());

    return start_rx();
}

bool gps_ubx_update(uint32_t now_ms)
{
    bool got_pvt = false;

    if (s_uart_err_pending) {
        /*
         * Ở chế độ DMA, HAL huỷ luồng nhận ngay khi có lỗi khung/tràn/nhiễu.
         * Khởi động lại ở đây chứ không trong ISR, để s_tail và bộ phân tích
         * chỉ có một nơi ghi. Lúc đang dò baud sai thì lỗi này tới liên tục.
         */
        s_uart_err_pending = false;
        (void)HAL_UART_AbortReceive(&huart7);
        if (!start_rx()) {
            g_fc.gps.error_count++;
        }
    }

    /* DMA đếm lùi: vị trí ghi hiện tại suy ra từ số byte còn lại. */
    const uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_uart7_rx);
    const uint16_t head =
        (uint16_t)((GPS_RX_BUFFER_SIZE - remaining) % GPS_RX_BUFFER_SIZE);

    while (s_tail != head) {
        const uint8_t byte = s_rx[s_tail];
        s_tail = (uint16_t)((s_tail + 1u) % GPS_RX_BUFFER_SIZE);
        s_bytes_rx++;

        if (ubx_push(byte) && handle_frame(now_ms)) {
            got_pvt = true;
        }
    }

    if (s_state == GPS_STATE_PROBE) {
        probe_step(now_ms);
    } else if (s_state == GPS_STATE_RUNNING) {
        /* Mốc là gói NAV-PVT cuối, hoặc lúc vào RUNNING nếu chưa có gói nào. */
        const uint32_t since =
            (s_have_pvt && (int32_t)(s_last_pvt_ms - s_running_since_ms) > 0)
            ? s_last_pvt_ms : s_running_since_ms;
        if (fc_elapsed_ms(now_ms, since) > GPS_RECONFIG_MS) {
            probe_restart(now_ms);
        }
    }

    if (s_have_pvt && fc_elapsed_ms(now_ms, s_last_pvt_ms) > GPS_TIMEOUT_MS) {
        g_fc.gps.healthy = false;
    }

    return got_pvt;
}

gps_state_t gps_ubx_state(void) { return s_state; }

const char *gps_ubx_state_name(void)
{
    switch (s_state) {
    case GPS_STATE_PROBE:   return "PROBE (dang do baud / gui cau hinh)";
    case GPS_STATE_RUNNING: return "RUNNING";
    default:                return "OFF";
    }
}

uint32_t gps_ubx_baud(void)             { return s_baud; }
uint32_t gps_ubx_bytes_received(void)   { return s_bytes_rx; }
uint32_t gps_ubx_frames_ok(void)        { return s_frames_ok; }
uint32_t gps_ubx_checksum_errors(void)  { return s_ck_errors; }
uint32_t gps_ubx_ack_count(void)        { return s_acks; }
uint32_t gps_ubx_nak_count(void)        { return s_naks; }
uint32_t gps_ubx_uart_errors(void)      { return s_uart_errors; }
uint32_t gps_ubx_config_sent(void)      { return s_cfg_sent; }

uint32_t gps_ubx_age_ms(uint32_t now_ms)
{
    return s_have_pvt ? fc_elapsed_ms(now_ms, s_last_pvt_ms) : UINT32_MAX;
}

void gps_ubx_uart_error_isr(void)
{
    s_uart_errors++;
    s_uart_err_pending = true;
}

#else /* GPS_ENABLE == 0 */

bool        gps_ubx_init(void)                  { return false; }
bool        gps_ubx_update(uint32_t now_ms)     { (void)now_ms; return false; }
gps_state_t gps_ubx_state(void)                 { return GPS_STATE_OFF; }
const char *gps_ubx_state_name(void)            { return "OFF"; }
uint32_t    gps_ubx_baud(void)                  { return 0; }
uint32_t    gps_ubx_bytes_received(void)        { return 0; }
uint32_t    gps_ubx_frames_ok(void)             { return 0; }
uint32_t    gps_ubx_checksum_errors(void)       { return 0; }
uint32_t    gps_ubx_ack_count(void)             { return 0; }
uint32_t    gps_ubx_nak_count(void)             { return 0; }
uint32_t    gps_ubx_uart_errors(void)           { return 0; }
uint32_t    gps_ubx_config_sent(void)           { return 0; }
uint32_t    gps_ubx_age_ms(uint32_t now_ms)     { (void)now_ms; return UINT32_MAX; }
void        gps_ubx_uart_error_isr(void)        { }

#endif /* GPS_ENABLE */
