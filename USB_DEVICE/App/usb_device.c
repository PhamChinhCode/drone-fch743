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

  /*
   * ---- Cap xung 48 MHz cho USB ----
   *
   * CubeMX KHONG sinh phan nay ra: trong SystemClock_Config() cua Core/Src/main.c,
   * PeriphClockSelection chi co RCC_PERIPHCLK_ADC va bo dao dong chi bat HSE.
   * HSI48 khong he duoc bat, va bo chon xung USB khong he duoc dat.
   *
   * Hau qua: OTG_FS chay khong co xung hop le nen KHONG BAO GIO enumerate duoc.
   * Do cung la ly do duong CDC tu truoc toi gio chua bao gio hien ra tren may
   * tinh - khong phai loi cap, khong phai loi lop thiet bi.
   *
   * Dat o day thay vi trong SystemClock_Config() vi day la vung USER CODE,
   * CubeMX Generate Code khong xoa mat. Va no chay TRUOC USBD_Init(), tuc
   * truoc khi HAL_PCD_Init() dung toi xung, nen dung thu tu.
   *
   * CRS keo HSI48 bam theo goi SOF cua may chu. USB FS doi sai so +-0,25%,
   * ma HSI48 chay tran chi dat +-1% nen phai co CRS moi chac.
   */
  {
    RCC_OscInitTypeDef       osc   = {0};
    RCC_PeriphCLKInitTypeDef pclk  = {0};
    RCC_CRSInitTypeDef       crs   = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    osc.HSI48State     = RCC_HSI48_ON;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
    {
      Error_Handler();
    }

    pclk.PeriphClockSelection = RCC_PERIPHCLK_USB;
    pclk.UsbClockSelection    = RCC_USBCLKSOURCE_HSI48;
    if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_CRS_CLK_ENABLE();
    crs.Prescaler             = RCC_CRS_SYNC_DIV1;
    crs.Source                = RCC_CRS_SYNC_SOURCE_USB2;
    crs.Polarity              = RCC_CRS_SYNC_POLARITY_RISING;
    crs.ReloadValue           = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000U, 1000U);
    crs.ErrorLimitValue       = 34;
    crs.HSI48CalibrationValue = 32;
    HAL_RCCEx_CRSConfig(&crs);
  }

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
  HAL_PWREx_EnableUSBVoltageDetector();

  /* USER CODE END USB_DEVICE_Init_PostTreatment */
}

/**
  * @}
  */

/**
  * @}
  */

