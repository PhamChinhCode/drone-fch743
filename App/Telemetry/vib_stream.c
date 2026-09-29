/**
 * @file    vib_stream.c
 * @brief   Hiện thực luồng rung 8 kHz qua USB CDC.
 */

#include "vib_stream.h"
#include "main.h"
#include "fc_state.h"
#include "tlm_messages.h"
#include "tlm_port.h"
#include "tlm_protocol.h"
#include "tlm_stream.h"

/* ==========================================================================
 * Vòng đệm ngắt -> vòng lặp chính
 *
 * 1024 mẫu = 128 ms ở 8 kHz. Đủ đỡ một lần USB/Windows khựng vài chục ms mà
 * không mất mẫu. 20 byte/mẫu -> 20 KB trong DTCM (chỉ CPU đụng tới).
 *
 * Một đầu ghi (ngắt SPI), một đầu đọc (vòng lặp chính): s_head chỉ ngắt ghi,
 * s_tail chỉ vòng lặp chính ghi. Hai bộ đếm chạy tự do, lấy hiệu là số mẫu
 * đang chờ — không cần khoá ngắt.
 * ========================================================================== */

#define VIB_RING_SIZE  1024u    /* phải là luỹ thừa của 2 */

_Static_assert((VIB_RING_SIZE & (VIB_RING_SIZE - 1u)) == 0u,
               "VIB_RING_SIZE phai la luy thua cua 2");
_Static_assert(sizeof(tlm_vib_t) <= TLM_MAX_PAYLOAD, "tlm_vib_t qua dai");

/*
 * Chừa chỗ trong đệm TX cho khung khác (trả lời CLI, ACK, heartbeat). Không
 * chừa thì luồng này chiếm trọn đệm và lệnh 'vib off' không có đường trả lời.
 */
#define VIB_TX_RESERVE          512u

/* Trần số khung mỗi vòng lặp, để một lần dồn hàng không kéo dài vòng lặp. */
#define VIB_MAX_FRAMES_PER_CALL 4u

typedef struct {
    tlm_vib_sample_t s;
    uint32_t         t_us;
    uint32_t         idx;
} vib_entry_t;

static vib_entry_t       s_ring[VIB_RING_SIZE];
static volatile uint32_t s_head;
static volatile uint32_t s_tail;
static volatile uint32_t s_idx;         /* đếm MỌI mẫu tới, kể cả mẫu bỏ */
static volatile uint32_t s_drops;
static volatile bool     s_enabled;
static uint32_t          s_frames;

static inline int16_t to_i16(float v)
{
    return (int16_t)fc_constrain_i32((int32_t)(v + (v >= 0.0f ? 0.5f : -0.5f)),
                                     INT16_MIN, INT16_MAX);
}

/* ==========================================================================
 * Phía ngắt
 * ========================================================================== */

void vib_stream_push(const vec3f_t *gyro_dps, const vec3f_t *accel_mps2,
                     uint32_t t_us)
{
    if (!s_enabled) {
        return;
    }

    const uint32_t idx = s_idx++;
    const uint32_t h   = s_head;

    if (h - s_tail >= VIB_RING_SIZE) {
        s_drops++;
        return;
    }

    const float ga = TLM_VIB_GYRO_LSB_PER_DPS;
    const float aa = TLM_VIB_ACCEL_LSB_PER_G / FC_GRAVITY_MPS2;

    vib_entry_t *e = &s_ring[h & (VIB_RING_SIZE - 1u)];
    e->s.gyro[0]  = to_i16(gyro_dps->x * ga);
    e->s.gyro[1]  = to_i16(gyro_dps->y * ga);
    e->s.gyro[2]  = to_i16(gyro_dps->z * ga);
    e->s.accel[0] = to_i16(accel_mps2->x * aa);
    e->s.accel[1] = to_i16(accel_mps2->y * aa);
    e->s.accel[2] = to_i16(accel_mps2->z * aa);
    e->t_us       = t_us;
    e->idx        = idx;

    __DMB();            /* nội dung mẫu phải xong trước khi đầu đọc thấy nó */
    s_head = h + 1u;
}

/* ==========================================================================
 * Phía vòng lặp chính
 * ========================================================================== */

void vib_stream_update(void)
{
    if (!s_enabled) {
        return;
    }
    if (tlm_port_get() != TLM_PORT_USB) {
        vib_stream_stop();
        return;
    }

    for (uint32_t k = 0; k < VIB_MAX_FRAMES_PER_CALL; k++) {
        const uint32_t tail  = s_tail;
        const uint32_t avail = s_head - tail;

        if (avail < TLM_VIB_SAMPLES_PER_FRAME) {
            return;
        }
        if (tlm_port_tx_free() < sizeof(tlm_vib_t) + TLM_OVERHEAD + VIB_TX_RESERVE) {
            return;
        }

        tlm_vib_t msg;
        const vib_entry_t *first = &s_ring[tail & (VIB_RING_SIZE - 1u)];

        msg.idx0       = first->idx;
        msg.t0_us      = first->t_us;
        msg.ring_drops = s_drops;
        for (int i = 0; i < FC_MOTOR_COUNT; i++) {
            msg.motor[i] = g_fc.motor.throttle[i];
        }
        msg.flags = g_fc.motor.armed ? 0x01u : 0x00u;

        /*
         * Chỉ gói các mẫu LIỀN NHAU. Ngắt vừa bỏ mẫu thì idx nhảy cóc giữa
         * chừng — khi đó cắt khung ngắn lại để idx0 + i luôn đúng.
         */
        uint8_t n = 0;
        while (n < TLM_VIB_SAMPLES_PER_FRAME) {
            const vib_entry_t *e = &s_ring[(tail + n) & (VIB_RING_SIZE - 1u)];
            if (e->idx != first->idx + n) {
                break;
            }
            msg.samples[n] = e->s;
            n++;
        }
        for (uint8_t i = n; i < TLM_VIB_SAMPLES_PER_FRAME; i++) {
            msg.samples[i] = (tlm_vib_sample_t){ { 0, 0, 0 }, { 0, 0, 0 } };
        }
        msg.n = n;

        if (!tlm_stream_send_payload(TLM_MSG_VIB, &msg, sizeof(msg))) {
            return;
        }

        __DMB();        /* đọc xong mẫu rồi mới trả chỗ cho ngắt */
        s_tail = tail + n;
        s_frames++;
    }
}

bool vib_stream_start(void)
{
    if (tlm_port_get() != TLM_PORT_USB) {
        return false;
    }

    s_enabled = false;
    s_tail    = s_head;     /* bỏ mẫu cũ còn sót từ lần trước */
    s_drops   = 0;
    s_frames  = 0;
    __DMB();
    s_enabled = true;
    return true;
}

void vib_stream_stop(void)
{
    s_enabled = false;
}

bool vib_stream_enabled(void)
{
    return s_enabled;
}

uint32_t vib_stream_frames_sent(void)
{
    return s_frames;
}

uint32_t vib_stream_ring_drops(void)
{
    return s_drops;
}
