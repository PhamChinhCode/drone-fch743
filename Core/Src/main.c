/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "fc_state.h"
#include "fc_time.h"
#include "icm20602.h"
#include "lsm6dsv.h"
#include "bmp388.h"
#if MAG_SOURCE == MAG_SOURCE_I2C
#include "mag_i2c.h"
#endif
#include "mtf01p.h"
#include "crsf.h"
#include "dshot.h"
#include "arming.h"
#include "estimator.h"
#include "ctrl_angle.h"
#include "ctrl_rate.h"
#include "mixer.h"
#include "dbg_console.h"
#include "tlm_port.h"
#include "tlm_stream.h"
#include "mav_link.h"
#include "param_msg.h"
#include "blackbox.h"
#include "qspi_flash.h"
#include "flashlog.h"
#include "usb_msc.h"
#include "param_table.h"
#include "param_store.h"
#include "cli.h"
#include "param_apply.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;

QSPI_HandleTypeDef hqspi;
MDMA_HandleTypeDef hmdma_quadspi_fifo_th;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;
DMA_HandleTypeDef hdma_tim1_up;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart8_tx;
DMA_HandleTypeDef hdma_uart8_rx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

/*
 * Danh dau dang o trong MX_SDMMC1_SD_Init(). Xem Error_Handler() de biet vi sao.
 */
volatile bool g_sd_init_in_progress = false;
volatile bool g_sd_init_failed = false;
volatile uint8_t g_sd_retry_count = 0; /* 0 = an ngay lan dau */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_MDMA_Init(void);
static void MX_DMA_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_UART8_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * KENH BAO TIN LUC KHOI DONG - chan, khong DMA, khong ngat.
 *
 * VI SAO CAN: dbg_console chi song sau dbg_console_init(), ma cho do nam SAU
 * TOAN BO cac MX_*_Init(). Bat ky cai nao trong so do treo thi bo mach im
 * hoan toan va khong co cach nao biet no dung o dau - da mat ca buoi vi dung
 * chuyen nay.
 *
 * Ham nay dung HAL_UART_Transmit kieu chan nen chay duoc ngay khi
 * MX_USART1_UART_Init() vua xong, va khong dung gi tra ve DMA hay ngat.
 */
void boot_msg(const char *s)
{
  if (huart1.Instance == NULL)
  {
    return; /* UART chua khoi tao, im lang */
  }

  const uint16_t n = (uint16_t)strlen(s);

  if (n != 0u)
  {
    (void)HAL_UART_Transmit(&huart1, (const uint8_t *)s, n, 50u);
  }
  (void)HAL_UART_Transmit(&huart1, (const uint8_t *)"\r\n", 2u, 10u);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /*
   * Bat USART1 NGAY DAY, truoc moi ngoai vi khac, chi de co duong bao tin.
   *
   * Goi lai o duoi trong day MX_*_Init() binh thuong khong sao: HAL_UART_Init
   * tu deinit roi init lai. Va dbg_console_init() moi la cho gan DMA vao, nen
   * khong xung dot.
   */
  /*
   * PHAI goi MX_DMA_Init() TRUOC.
   *
   * HAL_UART_MspInit() cua USART1 co goi HAL_DMA_Init() cho luong TX/RX. Ma
   * MX_DMA_Init() moi la cho bat XUNG cho bo DMA. Goi nguoc thu tu thi cau
   * hinh DMA duoc ghi vao mot ngoai vi chua co xung - ghi vao khong khi.
   *
   * Hau qua da do duoc: console phat mot khoi DMA roi ket vinh vien o
   * huart1.gState = HAL_UART_STATE_BUSY_TX, va tu do khong in them gi nua.
   * Nhin tu ngoai giong het "treo sau khi khoi tao USB".
   *
   * Goi lai o duoi trong day MX_*_Init() binh thuong khong sao: ca hai ham
   * deu chi bat xung va dat NVIC, chay hai lan cho cung ket qua.
   */
  MX_DMA_Init();
  MX_USART1_UART_Init();
  boot_msg("");
  boot_msg("boot: xung he thong OK, bat dau khoi tao ngoai vi");

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_MDMA_Init();
  MX_DMA_Init();
  MX_QUADSPI_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_USART3_UART_Init();
  MX_SPI3_Init();
  MX_UART8_Init();
  /* USER CODE BEGIN 2 */

  /*
   * --- The SD: thu lai vai lan truoc khi bo cuoc ---
   *
   * MX_SDMMC1_SD_Init() chi thu DUNG MOT LAN, va no chay chi vai ms sau khi
   * bo mach co dien. The SD can toi ~250 ms sau khi Vdd on dinh moi tra loi
   * dang tin. Do la mot cuoc dua, va khong phai lan nao cung thang - dung la
   * ly do the "luc mount duoc luc khong".
   *
   * Thu lai co DeInit xen giua de dua ngoai vi ve trang thai sach. Ba lan,
   * moi lan cach 200 ms: du cho ca the cham nhat, ma neu that su khong co the
   * thi cung chi ton 600 ms luc khoi dong.
   *
   * KHONG cuu duoc truong hop the ket cung tu lan chay truoc - cai do phai
   * RUT HAN NGUON, vi reset khong cat dien cho the.
   */
#if FC_SD_ENABLE
  if (g_sd_init_failed)
  {
    for (int attempt = 1; attempt <= 3 && g_sd_init_failed; attempt++)
    {
      HAL_Delay(200);
      (void)HAL_SD_DeInit(&hsd1);
      HAL_Delay(200);
      g_sd_init_in_progress = true;
      g_sd_init_failed = (HAL_SD_Init(&hsd1) != HAL_OK);
      g_sd_init_in_progress = false;

      g_sd_retry_count = (uint8_t)attempt;
    }
  }
#endif

  /*
   * Tham so chinh duoc luc chay. PHAI nam TRUOC MOI ham *_init() khac -
   * fc_state_init() va cac driver deu doc g_params ngay trong than ham init
   * cua chung. Nap muon mot nhip thi chung doc phai vung .bss toan so 0.
   *
   * Console chua chay o day nen ket qua duoc GIU LAI, in ra ngay sau khi
   * dbg_console_init(). Doi lai: chinh console khong the la tham so runtime -
   * chap nhan duoc, no la cong cu go loi chu khong phai thu chinh khi bay.
   *
   * Nap that bai thi g_params giu nguyen mac dinh tu fc_config.h, may bay van
   * bay duoc, chi mat phan tinh chinh.
   */
  param_load_defaults();
  const param_store_result_t param_load_result = param_store_load();

  fc_state_init();
  fc_time_init(); /* TIM2 chay -> micros() dung duoc */

  /*
   * Console go loi dang chu.
   *   &huart1 = PA9/PA10, 921600 baud, co DMA  -> dung duoc ngay.
   *   Muon dua ra USART3 (PD8/PD9) thi bat USART3 trong CubeMX roi doi
   *   tham so o day thanh &huart3. Module tu chuyen sang che do ngat khi
   *   UART do khong co DMA, khong phai sua gi them.
   */
  dbg_console_init(&huart1);
  dbg_console_set_rate(50);

  /*
   * Bao ket qua nap tham so (da chay o tren, truoc fc_state_init()).
   *
   * Dong "DUNG MAC DINH" la thu DUY NHAT cho biet cau hinh da mat - dung bo
   * qua no. Ly do cu the nam o dong ket qua ngay duoi.
   */
  dbg_println(param_load_result == PARAM_STORE_OK ? "Cau hinh: nap tu flash"
                                                  : "Cau hinh: DUNG MAC DINH");
  dbg_print_int("  so tham so", (int32_t)g_param_count);
  dbg_print_int("  seq", (int32_t)param_store_seq());
  dbg_print_hex("  table_crc", param_table_crc32(), 8);
  dbg_println(param_store_result_name(param_load_result));

  /*
   * Dong lenh chinh tham so. Dung chung UART voi console: USART1 RX (PA10)
   * qua DMA2_Stream0 vong tron, stream ma CubeMX da cau hinh san tu truoc
   * nhung chua ai dung.
   *
   * Go 'help' trong PuTTY o 921600 baud de bat dau.
   */
  cli_init(&huart1);
  dbg_println("CLI san sang - go 'help'.");

  /*
   * In trang thai hai nut ngay luc khoi dong.
   *
   * Ca hai deu la muc thap khi nhan (co dien tro keo len noi). Che do doc the
   * qua USB duoc chon bang K1, va che do do lam bo mach IM HOAN TOAN tren
   * console - rat de nham voi treo may. Dong nay de nhin mot cai la biet
   * firmware da doc duoc gi, khong phai ngoi doan.
   */
  dbg_print_int("Nut luc khoi dong: K1 (PE3) nhan?",
                (HAL_GPIO_ReadPin(BUTTON_K1_GPIO_Port, BUTTON_K1_Pin) == GPIO_PIN_RESET) ? 1 : 0);
  dbg_print_int("                   K2 (PC5) nhan?",
                (HAL_GPIO_ReadPin(BUTTON_K2_GPIO_Port, BUTTON_K2_Pin) == GPIO_PIN_RESET) ? 1 : 0);

  /*
   * Flash NOR tren QUADSPI (U3, W25Q64). Moi tham do JEDEC ID - chua ghi
   * chua doc gi.
   *
   * Dat o day vi no doc lap voi the SD va gan nhu khong ton thoi gian
   * (duoi 1 ms). Ket qua in ra ngay de nhin mot cai la biet co chip hay
   * khong, khong phai ngoi doan nhu hoi do the SD.
   */
#if QSPI_FLASH_ENABLE
  if (qspi_flash_init())
  {
    dbg_print_hex("QSPI flash: JEDEC ID", qspi_flash_jedec(), 6);
    dbg_print_int("            dung luong MB",
                  (int32_t)(qspi_flash_bytes() / (1024u * 1024u)));
    if (flashlog_init())
    {
      dbg_print_int("            log da dung, KB",
                    (int32_t)(flashlog_used_bytes() / 1024u));
      dbg_println(flashlog_state_name());
    }
  }
  else
  {
    dbg_println("QSPI flash: KHONG THAY CHIP.");
  }
#endif

  /*
   * Blackbox. Mount the SD NGAY DAY, truoc moi driver cam bien.
   *
   * VI SAO PHAI SOM NHU VAY: cac lenh cua the SD deu co han thoi gian. Khi
   * luong ngat DRDY 8 kHz cua IMU da chay, no chiem CPU lien tuc va lam cac
   * lenh nay chap chon — do duoc trang thai the nhay 4 -> 0 -> 4 tuy luc.
   *
   * O day chua co ngat toc do cao nao, thoi diem yen tinh nhat trong ca
   * chuong trinh. f_mount va f_open deu chan va co the mat vai tram ms,
   * nhung luc nay chan bao lau cung khong sao.
   *
   * That bai o day chi lam mat log, may bay van bay binh thuong.
   */
  if (g_sd_init_failed)
  {
    /*
     * HAL_SD_Init() that bai. Truoc day cho nay treo ca bo dieu khien - xem
     * Error_Handler(). Gio chi bao roi bay tiep.
     */
    dbg_println("=====================================================");
#if FC_SD_ENABLE
    dbg_println(" THE SD: KHONG KHOI TAO DUOC");
#else
    dbg_println(" THE SD: DA TAT bang FC_SD_ENABLE = 0");
    dbg_println("   Ly do: HAL_SD_Init() treo vinh vien voi the hien tai,");
    dbg_println("   lam main() khong bao gio chay toi phan telemetry.");
    dbg_println("   May bay van bay binh thuong - chi khong co blackbox.");
#endif
    dbg_print_int("   Da thu lai so lan", (int32_t)g_sd_retry_count);
    dbg_println("   Khong ghi log, va khong vao duoc che do doc the.");
    dbg_println("   May bay van bay binh thuong - day chi la phu kien.");
    dbg_println("   Thuong la the bi ket: RUT HAN NGUON roi cam lai.");
    dbg_println("   Bam reset KHONG du, vi reset khong cat dien cho the.");
    dbg_println("=====================================================");
  }
  else if (usb_msc_active())
  {
    /*
     * Che do doc the qua USB. May tinh dang toan quyen ghi tung sector cua
     * the. Firmware TUYET DOI khong duoc mount FatFs luc nay - hai ben cung
     * ghi thi bang FAT hong ngay va mat sach log cu.
     *
     * Nen o day khong goi blackbox_init(). Viec chan ARM da lam o
     * fc_state_can_arm() qua co ARM_BLOCK_USB_MSC.
     */
    dbg_println("=====================================================");
    dbg_println(" CHE DO DOC THE QUA USB (giu K1 luc khoi dong)");
    dbg_println("   Cam cap USB vao may tinh, the SD hien ra nhu o dia.");
    dbg_println("   KHONG ghi log va KHONG ARM duoc trong che do nay.");
    dbg_println("   Muon bay lai: EJECT o dia trong Windows TRUOC, roi rut USB");
    dbg_println("   va bam reset, dung giu K1.");
    dbg_println("   KHONG bam reset khi Windows con dang gan o dia: the co the");
    dbg_println("   ket cung, luc do phai RUT HAN NGUON moi go duoc.");
    dbg_println("=====================================================");

    /*
     * DUNG HAN O DAY - khong bao gio vao vong lap bay.
     *
     * Khong phai cho gon, ma la BAT BUOC de doc duoc the. Cac lenh SDMMC deu
     * co han thoi gian; khi luong ngat DRDY 8 kHz cua IMU chay o muc uu tien
     * 0, no chiem CPU lien tuc va lam cac lenh nay chap chon - da do duoc
     * trang thai the nhay 4 -> 0 -> 4, va do chinh la thu da lam f_mount hong
     * truoc day.
     *
     * May tinh doc the qua USB thi cung di qua dung nhung lenh SDMMC ay,
     * nen neu de vong lap bay chay tiep thi Windows se thay o dia rong khong
     * dung luong. Dung o day thi khong driver cam bien nao duoc khoi tao,
     * khong co ngat toc do cao nao, va SDMMC duoc yen.
     *
     * USB chay hoan toan bang ngat (OTG_FS_IRQHandler) nen vong lap nay khong
     * can lam gi ca. Dong co cung khong the quay: dshot_init() nam sau day.
     */
    /*
     * Bao nhip deu dan. KHONG duoc de vong nay im lang: mot che do khong phan
     * biet duoc voi treo may thi nguoi dung se tuong bo mach chet va di tim
     * loi o cho khac.
     */
    for (uint32_t tick = 0;; tick++)
    {
      dbg_print_int("  [che do the nho] van song, giay thu", (int32_t)(tick * 2u));
      HAL_Delay(2000);
    }
  }
  else if (blackbox_init())
  {
    if (g_sd_retry_count != 0u)
    {
      dbg_print_int("The SD: chi an sau khi thu lai lan thu",
                    (int32_t)g_sd_retry_count);
    }
    dbg_print_int("Blackbox: OK, ghi vao LOG file so", (int32_t)blackbox_file_index());
    dbg_println("  Ghi vao RAM khi ARM, xa ra the sau khi DISARM.");
  }
  else
  {
    dbg_println("Blackbox: LOI - khong ghi log duoc.");
    dbg_println(blackbox_state_name());
    dbg_print_int("  ma loi FRESULT", (int32_t)blackbox_last_error());
  }

  /*
   * Cac che do: DBG_MODE_IMU / IMU_RAW / IMU_CSV / FLOW / FLOW_RAW
   *             BARO / RC / RC_RAW / ARM / MOTOR / EST / STATUS
   * Dang dat EST de xem ket qua bo loc EKF. Doi sang DBG_MODE_MOTOR de xem
   * dau ra DShot, hoac DBG_MODE_ARM de xem may trang thai arm.
   */
  dbg_console_set_mode(DBG_MODE_ARM); /* kiem tra DBG_MODE_ALTHOLD */

  dbg_println("");
  dbg_println("=== FCH743_V1.0 khoi dong ===");

  if (icm20602_init())
  { /* chan ~150 ms */
    dbg_println("ICM20602: OK");
    icm20602_start();
    fc_state_set_mode(FC_MODE_CALIBRATING);
    icm20602_start_gyro_calibration();
    dbg_println("Gyro dang hieu chuan - de yen may bay...");
  }
  else
  {
    dbg_println("ICM20602: LOI - kiem tra CS/SPI/WHO_AM_I");
    fc_state_set_mode(FC_MODE_FAULT);
  }

  /*
   * IMU PHU LSM6DSV tren SPI3 (giai doan 1).
   *
   * Ghi vao g_fc.imu2, KHONG dung vao vong dieu khien. Loi o day chi
   * lam mat IMU phu - may bay van bay binh thuong bang ICM20602, nen
   * KHONG dat FC_MODE_FAULT.
   */
  /*
   * Truoc day la `#if IMU2_ENABLE` - tat IMU phu phai build lai. Gio la phep
   * kiem luc chay: `set imu2_enable=0` roi khoi dong lai. Doi lai la ma cua
   * lsm6dsv.c luon duoc nap vao firmware, nhung do la cai gia dung cho mot
   * cong tac dung de DO CHI PHI CPU - do xong phai bat lai duoc ngay.
   */
  if (!g_params.imu2_enable)
  {
    dbg_println("LSM6DSV: TAT bang imu2_enable = 0.");
  }
  else if (lsm6dsv_init())
  {
    dbg_print_int("LSM6DSV: OK, WHO_AM_I 0x", (int32_t)lsm6dsv_who_am_i());

    /*
     * Tu ke QMC6309 nam tren bus I2C PHU cua LSM6DSV, doc gian tiep qua
     * sensor hub. Phai cau hinh TRUOC khi bat luong DRDY vi ham nay dung
     * SPI3 kieu hoi vong va doi driver o trang thai IDLE.
     *
     * Loi o day chi lam mat tu ke - IMU phu va may bay van chay binh thuong.
     */
    if (g_params.mag_source != MAG_SOURCE_SHUB)
    {
      /*
       * KHONG co nghia la "khong co tu ke" - chi la nguon SHUB khong duoc
       * chon. Tu ke roi tren I2C1 (neu co) bao trang thai rieng o duoi.
       */
      dbg_println("Tu ke qua sensor hub: TAT (mag_source khac SHUB).");
    }
    else if (lsm6dsv_mag_init())
    {
      dbg_print_int("QMC6309: OK qua sensor hub, chip id 0x",
                    (int32_t)g_fc.mag.chip_id);
      dbg_println("  CHUA HIEU CHUAN - chua dung duoc cho giu huong.");
    }
    else
    {
      dbg_println("QMC6309: LOI");
      dbg_println(lsm6dsv_mag_init_result_name());
      dbg_print_int("  chip id doc duoc (thap phan)", (int32_t)g_fc.mag.chip_id);
      dbg_print_int("  STATUS_MASTER (thap phan)",
                    (int32_t)lsm6dsv_mag_last_status());
      dbg_println("  Mong doi chip id 144 (0x90).");
      lsm6dsv_mag_dump();
    }

    lsm6dsv_start();
    lsm6dsv_start_gyro_calibration();
  }
  else
  {
    dbg_println("LSM6DSV: LOI - kiem tra SPI3 (PB3/PB4/PD6), CS PA15, DRDY PD7");
    dbg_print_int("  WHO_AM_I doc duoc = 0x", (int32_t)lsm6dsv_who_am_i());
    dbg_println("  Mong doi 0x70. Xem muc Go loi trong App/Docs/KE_HOACH_LSM6DSV.md");
  }

  if (bmp388_init())
  { /* chan ~40 ms */
    dbg_print_int("BMP388: OK, chip id 0x", (int32_t)bmp388_chip_id());
    /*
     * Lay moc ap suat mat dat ngay: 50 mau o 50 Hz = 1 giay. De may bay
     * nam yen, tranh gio lua va quat gio trong luc nay.
     */
    bmp388_start_ground_calibration();
  }
  else
  {
    dbg_println("BMP388: LOI - kiem tra dia chi 0x77, dien tro keo len PB7/PB8");
  }

#if MAG_SOURCE == MAG_SOURCE_I2C
  /*
   * Tu ke roi tren I2C1, dung CHUNG bus voi BMP388 vua init xong o tren.
   * Goi SAU bmp388_init() de bus con dang ranh (BMP388 chua phat luot doc
   * dau tien). Driver tu do 0x1E (HMC5883L), 0x0D (QMC5883L), 0x2C (QMC5883P).
   *
   * Van kiem g_params.mag_source giong nhanh SHUB o tren, de tat duoc tu ke
   * luc chay ma khong phai nap lai firmware.
   */
  if (g_params.mag_source != MAG_SOURCE_I2C)
  {
    dbg_println("Tu ke I2C: TAT bang tham so mag_source.");
  }
  else if (mag_i2c_init())
  {
    dbg_println(mag_i2c_variant_name());
    /*
     * In theo co THAT, khong in co dinh. Truoc 2026-09-13 dong nay luon bao
     * "CHUA HIEU CHUAN" du g_fc.mag.calibrated = true (bo so do 2026-09-04 da
     * nam trong mac dinh fc_config.h) — va chinh dong log sai do da dan toi
     * mot cau tra loi sai trong hop dong FC<->Pi (GIAO_UOC muc 11.1 #3).
     */
    dbg_println(g_fc.mag.calibrated
                ? "  DA HIEU CHUAN (offset/scale khac don vi) - xem MAG_OFFSET_* trong fc_config.h."
                : "  CHUA HIEU CHUAN - chay DBG_MODE_MAGCAL truoc khi dung cho giu huong.");
  }
  else
  {
    dbg_println("Tu ke I2C: LOI - khong ai tra loi o 0x1E, 0x0D lan 0x2C");
    mag_i2c_scan_dump();
  }
#endif

  if (mtf01p_init())
  {
    dbg_println("MTF-01P: UART4 DMA da chay, dang cho du lieu...");
  }
  else
  {
    dbg_println("MTF-01P: LOI - khong khoi dong duoc DMA UART4");
  }

  if (crsf_init())
  {
    dbg_println("ELRS/CRSF: USART2 DMA da chay @420000, dang cho khung...");
  }
  else
  {
    dbg_println("ELRS/CRSF: LOI - khong khoi dong duoc DMA USART2");
  }

  /*
   * Bo giam sat arm. Khoi dong o trang thai LOCKED: phai gat cong tac arm ve
   * OFF mot lan moi nha khoa, ke ca khi luc cam pin cong tac da dang bat.
   */
#if MIX_STICK_PASSTHROUGH
  dbg_println("!!! MIX_STICK_PASSTHROUGH dang BAT - can lai thang vao mixer,");
  dbg_println("!!! KHONG co PID. Chi dung khi DA THAO CANH QUAT.");
#endif

  arming_init();
  dbg_println("Arming: LOCKED - gat cong tac arm ve OFF de nha khoa.");

  /*
   * DShot600 tren TIM1 (PE9/PE11/PE13/PE14). Sau khi init, bon duong nam im
   * o muc thap; dshot_update() bat dau phat lenh 0 (dung motor) lien tuc de
   * ESC vao trang thai san sang.
   *
   * THAO CANH QUAT truoc khi cam pin lan dau.
   */
  /*
   * Bo uoc luong EKF. Phai init SAU cac driver cam bien vi no doc thang tu
   * g_fc.imu / .baro / .flow.
   */
  ctrl_angle_init();
  ctrl_rate_init();
  mixer_init();
  estimator_init();
  dbg_println("EKF: dang cho mau IMU dau tien de dung goc ban dau...");

  if (dshot_init())
  {
    dbg_println("DShot300: TIM1 DMA burst da chay - THAO CANH QUAT khi thu!");
  }
  else
  {
    dbg_println("DShot300: LOI - khong bat duoc kenh PWM TIM1");
  }

  /*
   * Telemetry nhi phan ra USART3 (PD8/PD9) @921600.
   *
   *   FC --UART--> ESP32 "air" --ESP-NOW--> ESP32 "ground" --USB--> may tinh
   *
   * Cap ESP32 chi gom byte roi tra lai nguyen ven, firmware khong can biet
   * chung ton tai. Khac voi dbg_console (chu, cho nguoi doc) o USART1, luong
   * nay la nhi phan co CRC, danh cho phan mem tren may tinh.
   *
   * Ho so phat:
   *   TLM_PROFILE_FLIGHT ~ 250 B/s  - mac dinh, du cho ESP-NOW o tam xa
   *   TLM_PROFILE_TUNING ~ 2,5 kB/s - bat khi dang chinh PID, tam gan
   *   TLM_PROFILE_DEBUG  ~ 6 kB/s   - chi dung khi ESP32 nam canh may tinh
   * May tinh doi ho so luc dang chay bang TLM_MSG_CMD_SET_RATE, khong can nap
   * lai firmware.
   */
  tlm_port_init(TLM_PORT_UART);
  tlm_stream_init();
  param_msg_init();
  tlm_stream_apply_profile(TLM_PROFILE_FLIGHT);
  tlm_stream_send_text(0, "FCH743 telemetry san sang");
  dbg_println("Telemetry: USART3 @921600 -> ESP32 ESP-NOW, ho so FLIGHT.");

  /*
   * MAVLink ra may tinh nhung ROS2 tren UART8 (PE0/PE1) @921600.
   *
   * Duong nay DOC LAP voi telemetry USART3 o tren: khac day, khac giao thuc,
   * khac muc dich. USART3 cho khung nhi phan rieng cua du an ve phan mem PC;
   * UART8 cho MAVLink cho MAVROS.
   *
   * MUC 1 - chi giam sat va cat khan cap. May tinh nhung DOC duoc trang thai
   * va RA LENH DISARM, nhung KHONG dieu khien duoc chuyen bay. Lenh ARM tu
   * duong nay luon bi tu choi - xem ly do trong App/Mavlink/mav_link.c.
   */
  mav_init();
  dbg_println("MAVLink: UART8 @921600 -> may tinh nhung ROS2 (muc 1).");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static bool calib_done_reported = false;
  static bool baro_calib_reported = false;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    const uint32_t loop_t0_us = micros();
    const uint32_t now_ms = HAL_GetTick();

    g_fc.sys.uptime_ms = now_ms;
    fc_state_update_health(micros());

    /* Hieu chuan xong thi chuyen sang trang thai san sang. */
    if (!calib_done_reported && g_fc.imu.calibrated)
    {
      calib_done_reported = true;
      fc_state_set_mode(FC_MODE_DISARMED);
      dbg_println("Gyro hieu chuan xong.");
      dbg_print_float("  bias x (dps)", g_fc.imu.gyro_bias_dps.x, 3);
      dbg_print_float("  bias y (dps)", g_fc.imu.gyro_bias_dps.y, 3);
      dbg_print_float("  bias z (dps)", g_fc.imu.gyro_bias_dps.z, 3);
    }

    /* Moc ap suat mat dat da lay xong. */
    if (!baro_calib_reported && g_fc.baro.calibrated)
    {
      baro_calib_reported = true;
      dbg_println("BMP388 hieu chuan xong.");
      dbg_print_float("  ap suat mat dat (Pa)", g_fc.baro.ground_pressure_pa, 1);
      dbg_print_float("  nhiet do (C)", g_fc.baro.temperature_c, 2);
    }

    crsf_update();              /* rut byte tu dem DMA USART2 va phan tich */
    estimator_update(micros()); /* EKF: hop nhat gyro+accel+baro+laser   */
    arming_update(now_ms);      /* doc cong tac arm, quyet dinh arm/disarm  */
    /*
     * Nut K1 (PE3, keo len nen bam la muc THAP): moi lan bam quay thu motor
     * ke tiep trong 1,5 giay o 8% ga. Bam khi dang quay thu thi dung ngay.
     *
     * THAO CANH QUAT truoc khi dung. dshot_motor_test_start() tu tu choi khi
     * dang arm nen khong the vua bay vua quay thu.
     */
    {
      static bool k1_prev = false;
      static uint32_t k1_ms = 0;
      // static uint8_t k1_next = 0;
      static bool k1_seeded = false;

      const bool k1 =
          (HAL_GPIO_ReadPin(BUTTON_K1_GPIO_Port, BUTTON_K1_Pin) == GPIO_PIN_RESET);

      /*
       * Chi bat suon xuong, cach lan truoc it nhat 250 ms de chong doi phim.
       *
       * BO QUA khi console dang o che do hieu chuan tu ke: o do K1 duoc dung
       * de CHOT ket qua hieu chuan. Khong co chot nay thi mot lan bam vua
       * chot hieu chuan vua QUAY DONG CO - da xay ra that, va se la tai nan
       * neu con canh quat.
       *
       * BO QUA CA khi dang o che do doc the qua USB. Che do do duoc chon bang
       * cach GIU K1 luc khoi dong, nen nut van dang bi giu khi vao toi vong
       * lap nay. Ma k1_prev khoi tao la false, nen vong dau tien se thay
       * "suon xuong" gia va QUAY DONG CO - dung luc nguoi dung dang cam tay
       * vao bo mach de giu nut.
       */
      /* Lay trang thai that lam moc - xem giai thich o khoi K2 ben duoi. */
      if (!k1_seeded)
      {
        k1_seeded = true;
        k1_prev = k1;
      }

      if (k1 && !k1_prev && (uint32_t)(now_ms - k1_ms) > 250u &&
          dbg_console_get_mode() != DBG_MODE_MAGCAL &&
          !usb_msc_active())
      {
        k1_ms = now_ms;

        // if (dshot_motor_test_active() >= 0)
        // {
        //   dshot_motor_test_stop();
        //   dbg_println("Dung quay thu.");
        // }
        // else if (dshot_motor_test_start(k1_next, 0.08f, 1500))
        // {
        //   dbg_print_int("Quay thu motor", (int32_t)k1_next + 1);
        //   k1_next = (uint8_t)((k1_next + 1u) % FC_MOTOR_COUNT);
        // }
        // else
        // {
        //   dbg_println("Khong quay thu duoc - dang ARM.");
        // }
      }
      k1_prev = k1;
    }

    /*
     * Nut K2 (PC5, keo len nen bam la muc THAP): chay chuoi dao chieu cho cac
     * motor trong dshot_reverse_mask. Voi moi motor: gui SPIN_DIRECTION_REVERSED
     * roi SAVE_SETTINGS. THAO CANH QUAT truoc khi bam.
     *
     * Mask = 0 thi nut nay khong lam gi — do la trang thai binh thuong.
     */
    {
      static bool k2_prev = false;
      static uint32_t k2_ms = 0;
      static bool k2_seeded = false;

      const bool k2 =
          (HAL_GPIO_ReadPin(BUTTON_K2_GPIO_Port, BUTTON_K2_Pin) == GPIO_PIN_RESET);

      /*
       * Vong dau tien phai lay trang thai THAT cua nut lam moc, khong duoc
       * mac dinh la "chua nhan".
       *
       * Neu nut dang bi GIU tu luc khoi dong ma moc lai la false, thi
       * k2 && !k2_prev thanh dung ngay vong dau - mot suon xuong GIA. Voi K2
       * thi do la lenh dao chieu motor tu phat, dung luc tay nguoi dung con
       * dat tren bo mach. Mac dinh dshot_reverse_mask = 0 nen chua no ra,
       * nhung dat mask khac 0 la thanh tai nan.
       *
       * Da tung xay ra that voi K1 (giu K1 luc khoi dong de vao che do the
       * nho thi no quay thu motor).
       */
      if (!k2_seeded)
      {
        k2_seeded = true;
        k2_prev = k2;
      }

      if (k2 && !k2_prev && (uint32_t)(now_ms - k2_ms) > 250u)
      {
        k2_ms = now_ms;

        // if (g_params.dshot_reverse_mask == 0u)
        // {
        //   dbg_println("K2: dshot_reverse_mask dang la 0, khong dao chieu gi.");
        // }
        // else if (dshot_reverse_motors(g_params.dshot_reverse_mask))
        // {
        //   dbg_print_int("K2: dao chieu motor theo mask 0x",
        //                 (int32_t)g_params.dshot_reverse_mask);
        //   dbg_println("  Nghe ESC bip xac nhan, roi 'set dshot_reverse_mask=0' + 'save'.");
        // }
        // else
        // {
        //   dbg_println("K2: khong chay duoc - dang ARM hoac chuoi truoc chua xong.");
        // }
      }
      k2_prev = k2;
    }

    ctrl_angle_update(micros()); /* chon che do -> setpoint toc do goc  */
    ctrl_rate_update();          /* PID toc do goc -> ctrl.pid_output        */
    mixer_update();              /* lenh dieu khien -> muc ga 4 motor        */
    dshot_update(micros());      /* phat khung DShot cho 4 ESC               */
    bmp388_update();             /* xu ly mau baro va phat lenh doc I2C ke tiep */
#if MAG_SOURCE == MAG_SOURCE_I2C
    mag_i2c_update(micros()); /* dung chung I2C1 voi BMP388, tu gioi han theo MAG_I2C_UPDATE_RATE_HZ */
#endif
    mtf01p_update(); /* rut byte tu dem DMA UART4 va phan tich */
#if IMU2_ENABLE
    lsm6dsv_diag_poll(); /* tu tat khi da co mau dau tien */
#if MAG_SOURCE == MAG_SOURCE_SHUB
    lsm6dsv_mag_update(micros()); /* tu gioi han theo MAG_UPDATE_RATE_HZ */
#endif
#endif
    blackbox_update(micros()); /* ghi RAM khi ARM, xa the khi DISARM */
    flashlog_update(micros()); /* ghi thang vao flash NOR, ke ca khi dang ARM */
    dbg_console_update(now_ms);
    cli_update(); /* dong lenh chinh tham so tren USART1 */

    /*
     * --- Do thoi gian mot vong lap ---
     *
     * Vong PID toc do goc chay 4 kHz bang cach BAM NHIP theo sample_count
     * cua IMU, nhung no chi chay khi vong lap chinh goi toi. Vong lap phai
     * xong duoi 250 us, neu khong se lo mat mot nhip va tan so thuc te tut
     * xuong duoi 4000.
     *
     * loop_time_max_us la dinh trong CUA SO 1 GIAY vua roi, khong phai dinh
     * ke tu luc khoi dong - dinh mot lan luc khoi dong khong noi len dieu gi
     * ve trang thai hien tai.
     */
    {
      static uint32_t lt_mark_us;
      static uint32_t lt_peak_us;
      static uint32_t lt_slow;

      const uint32_t lt = fc_elapsed_us(micros(), loop_t0_us);

      g_fc.sys.loop_time_us = lt;
      g_fc.sys.loop_count++;
      if (lt > lt_peak_us)
      {
        lt_peak_us = lt;
      }

      /*
       * NGUONG LA 125 us, KHONG PHAI 250 us.
       *
       * Vong PID lo mot nhip khi vong lap bi ban DAI HON MOT CHU KY MAU IMU
       * (125 us), vi khi do gia tri count chan co the troi qua ma khong ai
       * kip doc. Dat nguong 250 us thi bo dem doc ra 0 trong khi thuc te van
       * dang mat ~5% nhip - mot dung cu do noi doi con te hon khong co.
       *
       * Dem lai moi giay, nen con so doc duoc la "bao nhieu vong CHAM trong
       * mot giay", khong phai tong tich luy tu luc khoi dong.
       */
      if (lt > 125u)
      {
        lt_slow++;
      }

      if (fc_elapsed_us(now_ms * 1000u, lt_mark_us) >= 1000000u)
      {
        g_fc.sys.loop_time_max_us = lt_peak_us;
        g_fc.sys.loop_overruns = lt_slow;
        lt_peak_us = 0;
        lt_slow = 0;
        lt_mark_us = now_ms * 1000u;
      }
    }

    /*
     * Telemetry nhi phan. Dat cuoi vong lap, sau khi moi gia tri trong g_fc
     * da duoc cap nhat trong nhip nay - neu dat truoc, goi gui di se tre mot
     * chu ky va goc/toc do trong cung mot goi lai lech nhau ve thoi gian.
     *
     * Ba ham deu khong chan: chung chi cheo byte vao ring buffer roi giao
     * cho DMA, phan con lai chay trong ngat TxCplt.
     */
    tlm_stream_rx_update();    /* lenh tu may tinh -> doi ho so, nap PID     */
    tlm_stream_update(now_ms); /* quet bang luong, phat cai nao toi han      */
    param_msg_update(now_ms);  /* bom bang tham so va dau ra CLI con do      */
    tlm_port_flush();          /* danh thuc DMA neu no dang ranh             */

    /*
     * MAVLink tren UART8. Dat sau khoi telemetry cu vi cung mot ly do: moi
     * gia tri trong g_fc phai da cap nhat xong trong nhip nay.
     *
     * Khong chan: chep byte vao ring buffer roi giao cho DMA, phan con lai
     * chay trong ngat UART8.
     */
    mav_update(now_ms);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
   */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 15;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
   */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 12;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_4;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 16;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
   */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief ADC3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
   */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.NbrOfConversion = 2;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_4;
  hadc3.Init.OversamplingMode = DISABLE;
  hadc3.Init.Oversampling.Ratio = 16;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00B03FDB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief QUADSPI Initialization Function
 * @param None
 * @retval None
 */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 22;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_2_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */
  boot_msg("boot: QUADSPI OK");

  /* USER CODE END QUADSPI_Init 2 */
}

/**
 * @brief SDMMC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */
#if !FC_SD_ENABLE
  /*
   * BO QUA the SD - xem FC_SD_ENABLE trong fc_config.h.
   *
   * HAL_SD_Init() voi the hien tai KHONG TRA VE: no quay vong trong
   * SD_SendSDStatus(). Goi no nghia la main() dung lai NGAY TAI DAY, va moi
   * thu phia sau - telemetry, CLI, cam bien, dieu khien - khong bao gio khoi
   * dong. Mot cai the hong lam liet ca mach bay.
   *
   * VI SAO CHOT NAM O DAY chu khong o cho GOI ham:
   *   Ban dau no boc quanh MX_SDMMC1_SD_Init() trong main(), tuc trong vung
   *   CubeMX sinh - va Generate Code lan 29/08 da XOA SACH no. Vung
   *   USER CODE nay thi CubeMX giu lai, nen dat o day moi song sot.
   */
  g_sd_init_failed = true;
  boot_msg("boot: SDMMC BO QUA (FC_SD_ENABLE = 0)");
  return;
#endif
  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */
  g_sd_init_in_progress = true;
  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd1.Init.ClockDiv = 2;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDMMC1_Init 2 */
  g_sd_init_in_progress = false;
  boot_msg("boot: SDMMC OK");
  /* USER CODE END SDMMC1_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 399;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 239;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 299;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 239;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);
}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 239;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */
}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 239;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 249;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);
}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */
}

/**
 * @brief UART8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 921600;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart8.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart8, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart8, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 420000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
}

/**
 * Enable MDMA controller clock
 */
static void MX_MDMA_Init(void)
{

  /* MDMA controller clock enable */
  __HAL_RCC_MDMA_CLK_ENABLE();
  /* Local variables */

  /* MDMA interrupt initialization */
  /* MDMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MDMA_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MDMA_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI4_SS_GPIO_Port, SPI4_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_SS_Pin | SPI3_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_BLK_Pin | LCD_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_SS_GPIO_Port, SPI2_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BUTTON_K1_Pin */
  GPIO_InitStruct.Pin = BUTTON_K1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_K1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI4_SS_Pin */
  GPIO_InitStruct.Pin = SPI4_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI4_SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI4_INT_Pin */
  GPIO_InitStruct.Pin = SPI4_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI4_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SS_Pin SPI3_SS_Pin */
  GPIO_InitStruct.Pin = SPI1_SS_Pin | SPI3_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_INT_Pin */
  GPIO_InitStruct.Pin = SPI1_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SPI1_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_K2_Pin */
  GPIO_InitStruct.Pin = BUTTON_K2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_K2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_BLK_Pin LCD_RS_Pin SPI2_SS_Pin */
  GPIO_InitStruct.Pin = LCD_BLK_Pin | LCD_RS_Pin | SPI2_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : I2C2_INT_Pin */
  GPIO_InitStruct.Pin = I2C2_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(I2C2_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI3_INT_Pin */
  GPIO_InitStruct.Pin = SPI3_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI3_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C1_INT2_Pin I2C1_INT1_Pin */
  GPIO_InitStruct.Pin = I2C1_INT2_Pin | I2C1_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(SPI1_INT_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(SPI1_INT_EXTI_IRQn);

  HAL_NVIC_SetPriority(SPI3_INT_EXTI_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(SPI3_INT_EXTI_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
   */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

  /*
   * THE SD HONG THI KHONG DUOC TREO CA BO DIEU KHIEN BAY.
   *
   * CubeMX sinh ra: if (HAL_SD_Init(&hsd1) != HAL_OK) { Error_Handler(); }
   * Ma Error_Handler ban goc la __disable_irq() roi while(1) — tuc la tat
   * ngat va quay vong vinh vien.
   *
   * Hau qua da do duoc that: the SD o trang thai ket (bi rut ra khi may tinh
   * dang mount qua USB MSC) lam bo mach IM HOAN TOAN — khong UART, khong USB,
   * va bam reset thi lai chay dung vao day. Nhin tu ngoai giong het chip chet.
   *
   * Ghi log chi la PHU KIEN. May bay phai bay duoc khi khong co the, khi the
   * hong, hay khi quen cam the. Nen rieng truong hop nay thi QUAY VE va bao
   * loi, khong treo.
   *
   * Moi loi khac van treo nhu cu — do la co y: mot ngoai vi bat buoc hong thi
   * cho bay moi la nguy hiem.
   *
   * Ca khoi nay nam trong vung USER CODE nen Generate Code lai khong mat.
   */
  if (g_sd_init_in_progress)
  {
    g_sd_init_in_progress = false;
    g_sd_init_failed = true;
    return;
  }

  /*
   * NOI TRUOC KHI CHET.
   *
   * Ban goc tat ngat roi quay vong im lang, nhin tu ngoai giong het chip
   * chet. Mot dong chu o day bien mot buoi do dac thanh mot cai nhin.
   */
  boot_msg("!!! Error_Handler - dung han tai day");

  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
