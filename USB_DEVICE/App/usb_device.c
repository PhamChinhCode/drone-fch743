/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_device.c
  * @version        : v1.0_Cube
  * @brief          : This file implements the USB Device
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

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

/* USER CODE BEGIN Includes */
#include "usbd_msc.h"
#include "usb_msc.h"

extern USBD_StorageTypeDef USBD_MSC_fops;
extern void boot_msg(const char *s);
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceFS;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
void MX_USB_DEVICE_Init(void)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */
  boot_msg("  usb: vao MX_USB_DEVICE_Init");

  /*
   * ---- Xung USB: DE CHO CubeMX LO, DUNG TU DAT O DAY ----
   *
   * Truoc day cho nay tu bat HSI48 + CRS roi dat bo chon xung USB, vi ban
   * CubeMX luc do KHONG sinh ra phan cau hinh xung nao ca.
   *
   * Sau lan Generate Code 29/08, HAL_PCD_MspInit() trong usbd_conf.c da tu
   * dat UsbClockSelection = RCC_USBCLKSOURCE_HSI48. Va luc do khoi cu o day
   * tro thanh DOC HAI:
   *
   *   - Truoc kia MspInit ghi de bo chon xung sang PLL3, nen khoi nay chay
   *     xong cung khong anh huong gi - USB chay bang PLL3 va hoat dong tot.
   *   - Bay gio MspInit chon dung HSI48, tuc USB THAT SU dung con HSI48 ma
   *     khoi nay vua chinh trim (HSI48CalibrationValue = 32 ghi de tri hieu
   *     chuan xuat xuong cua chip).
   *
   * Hau qua do duoc bang GDB: ket vinh vien trong USB_CoreReset, GRSTCTL bit
   * CSRST khong bao gio tu xoa, count chay toi 244 trieu. Bo mach im hoan
   * toan tu luc khoi dong.
   *
   * CRS thi mat theo - HSI48 chay tran chi dat +-1% so voi +-0,25% ma USB FS
   * doi hoi. Thuc te van enumerate duoc, nhung neu sau nay gap loi truyen
   * chap chon thi day la cho quay lai, va phai dat CRS SAU khi USB da
   * enumerate chu khong phai truoc.
   */

  /*
   * Chon lop USB theo che do chay. Ca khoi nay nam TRON trong vung USER CODE
   * nen CubeMX Generate Code khong xoa mat.
   *
   * Giu nut K1 luc khoi dong -> enumerate thanh o dia di dong (MSC) de doc
   * the SD tu may tinh. Khong giu -> giu nguyen CDC lam duong telemetry nhu cu.
   *
   * Hai lop khong the cung song tren mot thiet bi neu khong dung mo ta ghep,
   * ma CubeMX khong sinh ra kieu do - nen phai chon mot. Xem App/Storage/usb_msc.h.
   *
   * Mo ta thiet bi (FS_Desc) dung chung duoc cho ca hai vi bDeviceClass = 0,
   * tuc lop duoc khai bao o muc giao dien chu khong phai muc thiet bi.
   */
  boot_msg("  usb: doc nut K1 xong");

  /*
   * Nhanh CDC lam TRON O DAY roi return, thay vi de code CubeMX sinh chay.
   *
   * Ly do: bon loi goi USBD_* nam trong vung CubeMX sinh nen khong chen duoc
   * moc go loi vao giua. Lam o day thi moi buoc deu bao duoc, va nhanh MSC
   * ben duoi von da lam nhu vay roi - hai nhanh gio doi xung nhau.
   */
  if (!usb_msc_boot_requested())
  {
    boot_msg("  usb: [CDC] truoc USBD_Init");
    if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
    {
      Error_Handler();
    }
    boot_msg("  usb: [CDC] USBD_Init xong");

    if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
    {
      Error_Handler();
    }
    boot_msg("  usb: [CDC] RegisterClass xong");

    if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
    {
      Error_Handler();
    }
    boot_msg("  usb: [CDC] RegisterInterface xong");

    if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
    {
      Error_Handler();
    }
    boot_msg("  usb: [CDC] USBD_Start xong");

    HAL_PWREx_EnableUSBVoltageDetector();
    boot_msg("  usb: [CDC] hoan tat");
    return;
  }

  if (usb_msc_boot_requested())
  {
    if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
    {
      Error_Handler();
    }
    if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_MSC) != USBD_OK)
    {
      Error_Handler();
    }
    if (USBD_MSC_RegisterStorage(&hUsbDeviceFS, &USBD_MSC_fops) != USBD_OK)
    {
      Error_Handler();
    }
    if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
    {
      Error_Handler();
    }
    HAL_PWREx_EnableUSBVoltageDetector();
    return;
  }

  /* USER CODE END USB_DEVICE_Init_PreTreatment */

  /* Init Device Library, add supported class and start the library. */
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */
  boot_msg("  usb: USBD_Start xong");
  HAL_PWREx_EnableUSBVoltageDetector();
  boot_msg("  usb: bat bo do dien ap USB xong");

  /* USER CODE END USB_DEVICE_Init_PostTreatment */
}

/**
  * @}
  */

/**
  * @}
  */

