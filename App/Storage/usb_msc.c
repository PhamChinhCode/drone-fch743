/**
 * @file    usb_msc.c
 * @brief   Lop ghep giua lop USB MSC va the SD. Xem usb_msc.h de biet vi sao.
 */
#include "usb_msc.h"

#include "main.h"
#include "usbd_msc.h"
#include "bsp_driver_sd.h"

#include <string.h>

extern SD_HandleTypeDef hsd1;   /* dinh nghia trong Core/Src/main.c */

#define MSC_BLOCK_SIZE   512U

/** Cho the tra loi. Doc mot sector ma qua 2 s thi coi nhu the hong. */
#define MSC_IO_TIMEOUT_MS 2000U

/*
 * BO DEM TRUNG CHUYEN - day khong phai toi uu vi vui, ma la bat buoc.
 *
 * Lop MSC dua xuong day con tro toi bot_data[] nam trong handle cua no, ma
 * handle do do USBD_static_malloc() cap phat - tuc la mot bien static, tuc la
 * roi vao .bss, tuc la DTCMRAM o 0x20000000.
 *
 * DTCM chi noi thang voi loi Cortex-M7. IDMA cua SDMMC la bus master tren AHB
 * va KHONG voi toi duoc vung do. Dua thang con tro ay cho BSP_SD_ReadBlocks
 * thi lenh doc phat ra roi khong bao gio hoan tat.
 *
 * Dung y het loi da lam f_mount tra ve FR_DISK_ERR luc lam blackbox, va cung
 * la ly do App/Storage/blackbox.c phai tu khai bao FATFS/FIL trong .dma_buffer.
 *
 * O day khong sua duoc cho o cua bot_data vi no nam trong code CubeMX sinh,
 * nen chep qua mot vung AXI SRAM roi moi lam viec voi the.
 *
 * MSC_MEDIA_PACKET la 512 nen moi lan chi mot sector, nhung van viet vong lap
 * cho chac neu sau nay ai do tang MSC_MEDIA_PACKET len.
 */
FC_DMA_BUFFER static uint8_t s_bounce[MSC_BLOCK_SIZE];

static bool s_asked   = false;   /* da doc nut chua                */
static bool s_active  = false;   /* ket qua, nho lai de dung mai   */

/* ------------------------------------------------------------------ */
/*  Chon che do luc khoi dong                                          */
/* ------------------------------------------------------------------ */

bool usb_msc_boot_requested(void)
{
    if (s_asked) {
        return s_active;
    }
    s_asked = true;

    /*
     * Lay mau nhieu lan trong ~60 ms roi moi ket luan. Mot lan doc don le co
     * the trung dung luc nay chan hoac trung nhieu luc vua cap dien; doi hoi
     * nut giu lien tuc suot ca khoang thi khong con cho cho may rui do.
     */
    for (int i = 0; i < 6; i++) {
        if (HAL_GPIO_ReadPin(BUTTON_K1_GPIO_Port, BUTTON_K1_Pin) != GPIO_PIN_RESET) {
            s_active = false;
            return false;
        }
        HAL_Delay(10);
    }

    s_active = true;
    return true;
}

bool usb_msc_active(void)
{
    return s_active;
}

/* ------------------------------------------------------------------ */
/*  Cac ham lop MSC goi xuong                                          */
/* ------------------------------------------------------------------ */

static int8_t STORAGE_Init(uint8_t lun)
{
    (void)lun;
    /*
     * The da duoc MX_SDMMC1_SD_Init() dua vao trang thai TRANSFER roi. O day
     * chi xac nhan lai chu khong khoi tao lai - khoi tao lai giua chung se lam
     * may tinh dang doc bi dut.
     */
    return (BSP_SD_GetCardState() == MSD_OK) ? 0 : -1;
}

static int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t *block_num,
                                  uint16_t *block_size)
{
    HAL_SD_CardInfoTypeDef info;

    (void)lun;

    if (HAL_SD_GetCardInfo(&hsd1, &info) != HAL_OK) {
        return -1;
    }

    *block_num  = info.LogBlockNbr;
    *block_size = (uint16_t)info.LogBlockSize;
    return 0;
}

static int8_t STORAGE_IsReady(uint8_t lun)
{
    (void)lun;
    return (BSP_SD_GetCardState() == MSD_OK) ? 0 : -1;
}

static int8_t STORAGE_IsWriteProtected(uint8_t lun)
{
    (void)lun;
    return 0;      /* 0 = cho ghi */
}

static int8_t STORAGE_Read(uint8_t lun, uint8_t *buf, uint32_t blk_addr,
                           uint16_t blk_len)
{
    (void)lun;

    for (uint16_t i = 0; i < blk_len; i++) {
        if (BSP_SD_ReadBlocks((uint32_t *)(void *)s_bounce,
                              blk_addr + i, 1U, MSC_IO_TIMEOUT_MS) != MSD_OK) {
            return -1;
        }
        /* Doi the ghi xong roi moi chep ra, khong thi lay phai du lieu cu. */
        while (BSP_SD_GetCardState() != MSD_OK) {
            /* cho */
        }
        memcpy(&buf[(uint32_t)i * MSC_BLOCK_SIZE], s_bounce, MSC_BLOCK_SIZE);
    }
    return 0;
}

static int8_t STORAGE_Write(uint8_t lun, uint8_t *buf, uint32_t blk_addr,
                            uint16_t blk_len)
{
    (void)lun;

    for (uint16_t i = 0; i < blk_len; i++) {
        memcpy(s_bounce, &buf[(uint32_t)i * MSC_BLOCK_SIZE], MSC_BLOCK_SIZE);

        if (BSP_SD_WriteBlocks((uint32_t *)(void *)s_bounce,
                               blk_addr + i, 1U, MSC_IO_TIMEOUT_MS) != MSD_OK) {
            return -1;
        }
        /*
         * Cho the ghi xong that su. Bo buoc nay thi lenh ke tiep den luc the
         * con dang ban va se hong - dac biet luc may tinh xa bo dem cuoi cung
         * truoc khi eject.
         */
        while (BSP_SD_GetCardState() != MSD_OK) {
            /* cho */
        }
    }
    return 0;
}

static int8_t STORAGE_GetMaxLun(void)
{
    return 0;      /* mot o dia duy nhat */
}

/*
 * Chuoi nhan dang SCSI, dung 36 byte theo chuan INQUIRY.
 *   byte 0     : 0x00 = o dia truy cap truc tiep
 *   byte 1     : 0x80 = thao roi duoc
 *   byte 2..3  : phien ban chuan
 *   byte 4     : do dai phan con lai
 *   byte 8..15 : ten nha san xuat, dung 8 ky tu
 *   byte 16..31: ten san pham, dung 16 ky tu
 *   byte 32..35: phien ban, dung 4 ky tu
 * Do dai cac truong la CO DINH, thieu mot ky tu la Windows hien ten sai.
 */
static int8_t STORAGE_Inquirydata[] = {
    0x00, 0x80, 0x02, 0x02,
    (int8_t)(36 - 5), 0x00, 0x00, 0x00,
    'F', 'C', 'H', '7', '4', '3', ' ', ' ',
    'B', 'l', 'a', 'c', 'k', 'b', 'o', 'x', ' ', 'S', 'D', ' ', ' ', ' ', ' ', ' ',
    '1', '.', '0', '0'
};

USBD_StorageTypeDef USBD_MSC_fops = {
    STORAGE_Init,
    STORAGE_GetCapacity,
    STORAGE_IsReady,
    STORAGE_IsWriteProtected,
    STORAGE_Read,
    STORAGE_Write,
    STORAGE_GetMaxLun,
    STORAGE_Inquirydata
};
