/**
 * @file    dshot.c
 * @brief   Hiện thực driver DShot (TIM1 + DMA burst trên DMA1_Stream2).
 *          Tốc độ theo DSHOT_BITRATE_HZ trong fc_config.h.
 */

#include "dshot.h"
#include "param_table.h"
#include "fc_state.h"
#include "fc_time.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_tim1_up;

/* ==========================================================================
 * Bộ đệm DMA
 *
 * BẮT BUỘC dùng FC_DMA_BUFFER: DMA1 không truy cập được DTCMRAM, mà biến
 * static thường lại nằm đúng ở đó. Bỏ macro này thì ESC nhận rác hoặc chương
 * trình dính bus fault. Xem chú thích đầy đủ trong fc_types.h.
 *
 * 18 nhóm × 4 word = 72 word = 288 byte.
 * ========================================================================== */

FC_DMA_BUFFER static uint32_t s_dma[DSHOT_BUFFER_LEN * FC_MOTOR_COUNT];

/**
 * Motor thu n ra kenh TIM1 nao (0 = CCR1 ... 3 = CCR4).
 *
 * KHONG con `const`: dung lai tu g_params trong dshot_apply_params(). Giu
 * dang mang tra san thay vi doc g_params o cho dung, vi mang nay duoc doc
 * trong vong dung khung DShot chay 1000 lan moi giay cho tung bit.
 */
static uint8_t s_motor_map[FC_MOTOR_COUNT] = { 0u, 1u, 2u, 3u };

static volatile bool s_busy;        /* một lượt DMA đang chạy dở        */
static bool          s_ready;       /* init đã xong                     */

static uint32_t s_last_us;
static uint32_t s_frames;
static uint32_t s_errors;
static uint32_t s_skipped;
static uint32_t s_dma_errcode;   /* HAL_DMA_ERROR_* cua lan loi gan nhat */

/* Lệnh đặc biệt đang xếp hàng. s_cmd_motor < 0 nghĩa là gửi cho cả bốn. */
static uint16_t s_cmd;
static uint8_t  s_cmd_repeat;
static int8_t   s_cmd_motor = -1;

/* Chuỗi đảo chiều: mask motor còn phải xử lý, và đang ở bước nào. */
static uint8_t  s_rev_mask;
static uint8_t  s_rev_phase;    /* 0 = gửi REVERSED, 1 = gửi SAVE */

/* Quay thử từng motor. s_test_motor < 0 nghĩa là không chạy. */
static int8_t   s_test_motor = -1;
static uint16_t s_test_value;
static uint32_t s_test_start_us;
static uint32_t s_test_len_us;

/*
 * Muc ga ung voi output_norm = 0 - tuc luc da arm nhung can ga o day.
 * Day KHONG phai DSHOT_MIN_THROTTLE; xem dshot_idle_percent.
 * Voi 5,5 % thi gia tri nay la 48 + 0,055 x 1999 ~ 158.
 */
static float s_idle_value;

/*
 * Do rong xung, tinh tu toc do bit luc chay.
 *
 * Truoc day la hang so bien dich kem mot #error chan truong hop T1H >= ARR.
 * Toc do bit gio la tham so runtime nen phep chan chuyen vao
 * dshot_apply_params() - va no VAN PHAI CO: T1H vuot ARR thi duong tin hieu
 * ket o muc cao, ESC coi nhu mat tin hieu, va bieu hien duy nhat ra ngoai la
 * motor khong quay.
 */
static uint32_t s_arr;
static uint32_t s_t0h;
static uint32_t s_t1h;

void dshot_apply_params(void)
{
    s_motor_map[0] = g_params.dshot_motor_map_1;
    s_motor_map[1] = g_params.dshot_motor_map_2;
    s_motor_map[2] = g_params.dshot_motor_map_3;
    s_motor_map[3] = g_params.dshot_motor_map_4;

    s_idle_value = (float)DSHOT_MIN_THROTTLE +
                   (g_params.dshot_idle_percent * 0.01f) *
                       (float)(DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE);

    /*
     * Cung cong thuc voi DSHOT_ARR / DSHOT_T0H / DSHOT_T1H trong fc_config.h,
     * chi khac la lay toc do bit tu g_params.
     *
     * 37,5 % chu ky cho bit 0 va 75 % cho bit 1 - dung chuan DShot.
     */
    const uint32_t bitrate = (g_params.dshot_bitrate_hz > 0u)
                           ? g_params.dshot_bitrate_hz : DSHOT_BITRATE_HZ;

    s_arr = (FC_TIMER_CLK_HZ / bitrate) - 1u;
    s_t0h = ((s_arr + 1u) * 375u) / 1000u;
    s_t1h = ((s_arr + 1u) * 750u) / 1000u;

    if (s_t1h >= s_arr) {
        /* Bo tham so vo ly - lui ve toc do bien dich da duoc thu thuc te. */
        s_arr = DSHOT_ARR;
        s_t0h = DSHOT_T0H;
        s_t1h = DSHOT_T1H;
    }
}

/* ==========================================================================
 * Dựng khung
 * ========================================================================== */

/**
 * Ghép 16 bit khung DShot.
 *
 *   [11 bit giá trị][1 bit xin telemetry][4 bit CRC]
 *
 * CRC là XOR của ba nibble đứng trước. Đây là biến thể DShot THƯỜNG; DShot
 * hai chiều đảo bit CRC, nếu sau này làm eRPM thì phải sửa chỗ này.
 */
static uint16_t make_frame(uint16_t value, bool telemetry)
{
    const uint16_t packet = (uint16_t)((value << 1) | (telemetry ? 1u : 0u));

    uint16_t csum = 0;
    uint16_t tmp  = packet;
    for (int i = 0; i < 3; i++) {
        csum ^= tmp;
        tmp   = (uint16_t)(tmp >> 4);
    }

    return (uint16_t)((packet << 4) | (csum & 0x0Fu));
}

/**
 * Trải bốn khung ra bộ đệm DMA theo bố cục xen kẽ mà chế độ burst yêu cầu:
 * s_dma[bit * 4 + kênh]. Bit cao phát trước.
 */
static void fill_buffer(const uint16_t value[FC_MOTOR_COUNT], bool telemetry)
{
    for (uint32_t m = 0; m < FC_MOTOR_COUNT; m++) {
        const uint16_t frame = make_frame(value[m], telemetry);
        const uint32_t ch    = s_motor_map[m];

        for (uint32_t b = 0; b < DSHOT_FRAME_BITS; b++) {
            const uint16_t mask = (uint16_t)(1u << (DSHOT_FRAME_BITS - 1u - b));
            s_dma[b * FC_MOTOR_COUNT + ch] =
                ((frame & mask) != 0u) ? s_t1h : s_t0h;
        }

        /* Khoảng lặng cuối khung: CCR = 0 nên đường giữ mức thấp. */
        for (uint32_t b = DSHOT_FRAME_BITS; b < DSHOT_BUFFER_LEN; b++) {
            s_dma[b * FC_MOTOR_COUNT + ch] = 0u;
        }
    }
}

/** Đưa cả bốn đường về mức thấp ngay lập tức. */
static inline void outputs_low(void)
{
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM1->CCR4 = 0;
}

static void start_transfer(void)
{
    /*
     * Đặt cờ bận TRƯỚC khi gọi HAL: ngắt DMA1_Stream2 ở mức ưu tiên 1 có thể
     * chen vào giữa chừng, và ISR là nơi hạ cờ này xuống.
     */
    s_busy = true;

    if (HAL_TIM_DMABurst_MultiWriteStart(
            &htim1,
            TIM_DMABASE_CCR1,               /* DCR.DBA trỏ vào CCR1        */
            TIM_DMA_UPDATE,                 /* mỗi bit một sự kiện UPDATE  */
            s_dma,
            TIM_DMABURSTLENGTH_4TRANSFERS,  /* DCR.DBL = 4 -> CCR1..CCR4   */
            DSHOT_BUFFER_LEN * FC_MOTOR_COUNT) != HAL_OK) {

        s_busy = false;
        s_errors++;
        fc_state_set_error(FC_ERR_DSHOT_DMA);
    }
}

/* ==========================================================================
 * API
 * ========================================================================== */

bool dshot_init(void)
{
    /* Chot map motor, muc ga day va nhip bit TRUOC khi cham vao timer. */
    dshot_apply_params();

    /* Section .dma_buffer là NOLOAD nên không được startup code xoá. */
    memset(s_dma, 0, sizeof(s_dma));

    s_busy       = false;
    s_ready      = false;
    s_frames     = 0;
    s_errors     = 0;
    s_skipped    = 0;
    s_cmd        = 0;
    s_cmd_repeat = 0;
    s_cmd_motor  = -1;
    s_rev_mask   = 0;
    s_rev_phase  = 0;
    s_test_motor = -1;

    s_dma_errcode = 0;

    /*
     * --- Bật FIFO cho kênh DMA của TIM1 ---
     *
     * CubeMX sinh ra kênh này ở chế độ direct (FIFOMode = DISABLE). Chế độ đó
     * KHÔNG hợp với DMA burst của timer: mỗi sự kiện UPDATE, timer bắn liên
     * tiếp 4 yêu cầu (một cho mỗi CCR), mà direct mode chỉ nạp trước được một
     * word nên không kịp phục vụ — phần cứng dựng cờ lỗi direct mode ở MỌI
     * khung. Truyền vẫn xong nên nhìn bề ngoài có vẻ chạy, nhưng word bị trễ
     * làm sai độ rộng một bit và ESC loại khung đó vì sai CRC.
     *
     * Bật FIFO với ngưỡng đầy (4 word) cho DMA gom sẵn trọn một nhóm CCR
     * trước khi timer đòi, nên phục vụ được cả 4 yêu cầu liên tiếp.
     *
     * Sửa ở đây thay vì trong CubeMX là có chủ ý: stm32h7xx_hal_msp.c bị ghi
     * đè mỗi lần Generate Code, còn App/ thì không.
     */
    HAL_DMA_DeInit(&hdma_tim1_up);
    hdma_tim1_up.Init.FIFOMode      = DMA_FIFOMODE_ENABLE;
    hdma_tim1_up.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_tim1_up.Init.MemBurst      = DMA_MBURST_SINGLE;
    hdma_tim1_up.Init.PeriphBurst   = DMA_PBURST_SINGLE;

    if (HAL_DMA_Init(&hdma_tim1_up) != HAL_OK) {
        return false;
    }
    __HAL_LINKDMA(&htim1, hdma[TIM_DMA_ID_UPDATE], hdma_tim1_up);

    /*
     * --- Đặt chu kỳ timer theo fc_config.h ---
     *
     * MX_TIM1_Init() đặt CỨNG Period = 399, con số chỉ đúng cho DShot600.
     * Đổi DSHOT_BITRATE_HZ mà ARR vẫn 399 thì với DShot300, T1H = 600 vượt
     * quá ARR và đầu ra kẹt vĩnh viễn ở mức cao — hỏng hoàn toàn mà không có
     * lấy một cảnh báo nào.
     *
     * Ghi lại ARR ở đây để fc_config.h thành nguồn sự thật duy nhất: đổi tốc
     * độ DShot chỉ cần sửa đúng một dòng, không phải mở CubeMX.
     */
    __HAL_TIM_SET_AUTORELOAD(&htim1, s_arr);
    TIM1->EGR  = TIM_EGR_UG;      /* nạp ARR ngay, bộ đếm còn đang dừng */
    TIM1->SR  &= ~TIM_SR_UIF;     /* xoá cờ mà lệnh UG vừa dựng lên     */

    /* Trước khi bật đầu ra, bảo đảm độ rộng xung bằng 0. */
    outputs_low();

    static const uint32_t channels[FC_MOTOR_COUNT] = {
        TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
    };

    for (int i = 0; i < FC_MOTOR_COUNT; i++) {
        if (HAL_TIM_PWM_Start(&htim1, channels[i]) != HAL_OK) {
            return false;
        }
    }

    /*
     * Timer giờ chạy tự do ở 600 kHz với CCR = 0: bốn đường nằm im ở mức
     * thấp. Cố ý KHÔNG dừng timer giữa các khung — cho nó chạy liên tục thì
     * khung sau nối vào khung trước mà không có nhiễu do khởi động lại.
     */
    s_ready   = true;
    s_last_us = micros();
    return true;
}

bool dshot_update(uint32_t now_us)
{
    if (!s_ready) {
        return false;
    }
    if (fc_elapsed_us(now_us, s_last_us) < DSHOT_PERIOD_US) {
        return false;
    }

    /*
     * Khung trước chưa phát xong. Không cập nhật s_last_us để lần gọi sau thử
     * lại ngay. Một khung dài ~30 µs còn chu kỳ là 1000 µs, nên chuyện này
     * chỉ xảy ra khi DMA kẹt thật.
     */
    if (s_busy) {
        s_skipped++;
        return false;
    }
    s_last_us = now_us;

    /*
     * ĐIỀU KIỆN DUY NHẤT ĐỂ MOTOR QUAY. Kiểm tra cả hai cờ: chúng được
     * fc_state_set_mode() đặt cùng lúc, nên một lần ghi hỏng vào riêng một cờ
     * vẫn không đủ để đầu ra thoát khỏi mức 0.
     */
    const bool armed = g_fc.motor.armed && (g_fc.mode == FC_MODE_ARMED);

    /* Arm trong lúc đang quay thử: bỏ quay thử ngay, arm được ưu tiên. */
    if (armed) {
        s_test_motor = -1;
    }

    /*
     * Tiến chuỗi đảo chiều khi lệnh trước đã phát xong. Mỗi motor cần hai
     * lệnh nối tiếp: REVERSED rồi SAVE_SETTINGS, và phải tách bạch từng motor
     * nên không gộp được.
     */
    if (!armed && s_rev_mask != 0u && s_cmd_repeat == 0u) {
        uint8_t m = 0;
        while ((s_rev_mask & (uint8_t)(1u << m)) == 0u) {
            m++;
        }

        if (s_rev_phase == 0u) {
            s_cmd        = DSHOT_CMD_SPIN_DIRECTION_REVERSED;
            s_rev_phase  = 1u;
        } else {
            s_cmd        = DSHOT_CMD_SAVE_SETTINGS;
            s_rev_mask  &= (uint8_t)~(1u << m);
            s_rev_phase  = 0u;
        }
        s_cmd_motor  = (int8_t)m;
        s_cmd_repeat = DSHOT_CMD_REPEAT;
    }

    uint16_t value[FC_MOTOR_COUNT];

    if (armed) {
        /*
         * output_norm = 0 ra mức ga nghỉ chứ KHÔNG ra 48. Ở 48 motor không đủ
         * mô-men khởi động: giật cục, và cái nào nặng hơn thì đứng im rồi ESC
         * kêu bíp báo stall.
         */
        for (int i = 0; i < FC_MOTOR_COUNT; i++) {
            const float n = fc_constrainf(g_fc.motor.output_norm[i], 0.0f, 1.0f);
            const float t = s_idle_value
                          + n * ((float)DSHOT_MAX_THROTTLE - s_idle_value);
            value[i] = (uint16_t)fc_constrain_i32((int32_t)(t + 0.5f),
                                                  DSHOT_MIN_THROTTLE,
                                                  DSHOT_MAX_THROTTLE);
        }
    } else if (s_test_motor >= 0) {
        /*
         * Quay thử: đúng một motor nhận ga, ba cái còn lại nhận lệnh dừng.
         * Hết giờ thì tự tắt ngay trong nhánh này.
         */
        if (fc_elapsed_us(now_us, s_test_start_us) >= s_test_len_us) {
            s_test_motor = -1;
            for (int i = 0; i < FC_MOTOR_COUNT; i++) {
                value[i] = DSHOT_CMD_MOTOR_STOP;
            }
        } else {
            for (int i = 0; i < FC_MOTOR_COUNT; i++) {
                value[i] = (i == s_test_motor) ? s_test_value
                                               : (uint16_t)DSHOT_CMD_MOTOR_STOP;
            }
        }
    } else if (s_cmd_repeat > 0u) {
        /* Lệnh đặc biệt chỉ chạy khi đã disarm — dshot_send_command() lo việc đó. */
        for (int i = 0; i < FC_MOTOR_COUNT; i++) {
            value[i] = (s_cmd_motor < 0 || i == s_cmd_motor)
                     ? s_cmd : (uint16_t)DSHOT_CMD_MOTOR_STOP;
        }
        s_cmd_repeat--;
    } else {
        for (int i = 0; i < FC_MOTOR_COUNT; i++) {
            value[i] = DSHOT_CMD_MOTOR_STOP;
        }
    }

    for (int i = 0; i < FC_MOTOR_COUNT; i++) {
        g_fc.motor.throttle[i] = value[i];
    }

    fill_buffer(value, false);
    start_transfer();

    g_fc.motor.frame_count++;
    g_fc.motor.timestamp_us = now_us;
    return true;
}

bool dshot_send_command(uint16_t command)
{
    if (command == 0u || command > DSHOT_CMD_MAX) {
        return false;
    }

    /* Đổi chiều quay hay lưu cài đặt giữa không trung là tai nạn. */
    if (g_fc.motor.armed || g_fc.mode == FC_MODE_ARMED) {
        return false;
    }

    s_cmd        = command;
    s_cmd_motor  = -1;                 /* cả bốn motor */
    s_cmd_repeat = DSHOT_CMD_REPEAT;
    return true;
}

bool dshot_send_command_motor(uint8_t motor, uint16_t command)
{
    if (motor >= FC_MOTOR_COUNT || command == 0u || command > DSHOT_CMD_MAX) {
        return false;
    }
    if (g_fc.motor.armed || g_fc.mode == FC_MODE_ARMED) {
        return false;
    }

    s_cmd        = command;
    s_cmd_motor  = (int8_t)motor;
    s_cmd_repeat = DSHOT_CMD_REPEAT;
    return true;
}

bool dshot_command_busy(void)
{
    return (s_cmd_repeat > 0u) || (s_rev_mask != 0u);
}

bool dshot_reverse_motors(uint8_t mask)
{
    if (mask == 0u || (mask >> FC_MOTOR_COUNT) != 0u) {
        return false;
    }
    if (g_fc.motor.armed || g_fc.mode == FC_MODE_ARMED) {
        return false;
    }
    if (s_rev_mask != 0u) {
        return false;              /* chuỗi trước chưa xong */
    }

    s_rev_mask  = mask;
    s_rev_phase = 0;
    return true;
}

bool dshot_motor_test_start(uint8_t motor, float throttle_norm,
                            uint16_t duration_ms)
{
    if (motor >= FC_MOTOR_COUNT) {
        return false;
    }

    /* Chốt số một: không bao giờ quay thử khi đang arm. */
    if (g_fc.motor.armed || g_fc.mode == FC_MODE_ARMED) {
        return false;
    }

    const float max_norm = DSHOT_TEST_MAX_PERCENT * 0.01f;
    const float n = fc_constrainf(throttle_norm, 0.0f, max_norm);

    const float t = s_idle_value
                  + n * ((float)DSHOT_MAX_THROTTLE - s_idle_value);

    s_test_value    = (uint16_t)fc_constrain_i32((int32_t)(t + 0.5f),
                                                 DSHOT_MIN_THROTTLE,
                                                 DSHOT_MAX_THROTTLE);
    s_test_len_us   = (uint32_t)((duration_ms < DSHOT_TEST_MAX_MS)
                                 ? duration_ms : DSHOT_TEST_MAX_MS) * 1000u;
    s_test_start_us = micros();
    s_test_motor    = (int8_t)motor;
    return true;
}

void dshot_motor_test_stop(void)
{
    s_test_motor = -1;
}

int8_t   dshot_motor_test_active(void) { return s_test_motor; }

bool     dshot_is_busy(void)     { return s_busy; }
uint32_t dshot_frames_sent(void) { return s_frames; }
uint32_t dshot_dma_errors(void)  { return s_errors; }
uint32_t dshot_skipped(void)     { return s_skipped; }
uint32_t dshot_dma_errcode(void) { return s_dma_errcode; }

/* ==========================================================================
 * Ngắt
 *
 * Chạy trong DMA1_Stream2_IRQHandler ở mức ưu tiên 1 — ngay dưới gyro (0) và
 * trên mọi thứ còn lại, đúng như thiết kế ưu tiên của project.
 * ========================================================================== */

void dshot_dma_complete_isr(void)
{
    /*
     * Phải dừng burst sau mỗi khung. DMA chạy ở chế độ NORMAL nên nó tự
     * ngừng, nhưng yêu cầu DMA của timer thì vẫn bật; để nguyên thì khung sau
     * sẽ bị lệch. Ngoài ra CCR còn giữ giá trị bit cuối, nên phải ép cả bốn
     * đường về 0 — nếu không đường tín hiệu đứng ở mức 75 % chu kỳ và ESC đọc
     * thành rác.
     *
     * KHÔNG dùng HAL_TIM_DMABurst_WriteStop() ở đây, dù đó là hàm "đúng tên".
     * Nó gọi HAL_DMA_Abort_IT() lên một lượt truyền VỪA KẾT THÚC — HAL đã đặt
     * State = READY trước khi gọi callback này — nên hàm abort trả lỗi và ghi
     * ErrorCode = HAL_DMA_ERROR_NO_XFER (0x80). Ngay sau callback này,
     * HAL_DMA_IRQHandler kiểm `ErrorCode != NONE` rồi gọi tiếp callback lỗi.
     * Kết quả là mỗi khung tự đẻ ra một lỗi giả, và đó chính là hiện tượng
     * err đếm bằng đúng frames.
     *
     * Hai dòng dưới đây là đúng phần việc cần thiết mà WriteStop làm, bỏ đi
     * cái abort thừa. Có chạm trực tiếp vào DMABurstState của HAL, nhưng đó là
     * cách duy nhất mở khoá burst cho khung sau mà không kích hoạt abort.
     */
    __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE);
    htim1.DMABurstState = HAL_DMA_BURST_STATE_READY;
    outputs_low();

    s_frames++;
    s_busy = false;
}

void dshot_dma_error_isr(void)
{
    s_dma_errcode = hdma_tim1_up.ErrorCode;
    s_errors++;
    fc_state_set_error(FC_ERR_DSHOT_DMA);

    /*
     * CHỈ tháo gỡ khi lượt truyền còn đang dở.
     *
     * Lỗi FIFO và lỗi direct mode KHÔNG huỷ truyền — HAL ghi nhận chúng ở đầu
     * HAL_DMA_IRQHandler rồi vẫn xử lý cờ TC bình thường, mãi cuối hàm mới gọi
     * callback lỗi. Nghĩa là hàm này có thể chạy SAU khi khung đã phát xong.
     * Gọi WriteStop lúc ấy là tháo gỡ một lượt truyền đã kết thúc, và tệ hơn,
     * nếu lỗi rơi vào giữa khung sau thì nó cắt cụt khung đó.
     */
    if (s_busy) {
        (void)HAL_TIM_DMABurst_WriteStop(&htim1, TIM_DMA_UPDATE);
        outputs_low();
        s_busy = false;
    }
}
