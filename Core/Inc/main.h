/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUTTON_K1_Pin GPIO_PIN_3
#define BUTTON_K1_GPIO_Port GPIOE
#define SPI4_SS_Pin GPIO_PIN_4
#define SPI4_SS_GPIO_Port GPIOE
#define SPI4_INT_Pin GPIO_PIN_13
#define SPI4_INT_GPIO_Port GPIOC
#define SPI1_SS_Pin GPIO_PIN_4
#define SPI1_SS_GPIO_Port GPIOA
#define SPI1_INT_Pin GPIO_PIN_4
#define SPI1_INT_GPIO_Port GPIOC
#define SPI1_INT_EXTI_IRQn EXTI4_IRQn
#define BUTTON_K2_Pin GPIO_PIN_5
#define BUTTON_K2_GPIO_Port GPIOC
#define LCD_BLK_Pin GPIO_PIN_0
#define LCD_BLK_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_1
#define LCD_RS_GPIO_Port GPIOB
#define I2C2_INT_Pin GPIO_PIN_15
#define I2C2_INT_GPIO_Port GPIOE
#define SPI2_SS_Pin GPIO_PIN_12
#define SPI2_SS_GPIO_Port GPIOB
#define SERVO1_Pin GPIO_PIN_14
#define SERVO1_GPIO_Port GPIOD
#define SERVO2_Pin GPIO_PIN_15
#define SERVO2_GPIO_Port GPIOD
#define BUZZER_Pin GPIO_PIN_6
#define BUZZER_GPIO_Port GPIOC
#define WS2812_LED_Pin GPIO_PIN_7
#define WS2812_LED_GPIO_Port GPIOC
#define SPI3_SS_Pin GPIO_PIN_15
#define SPI3_SS_GPIO_Port GPIOA
#define SPI3_INT_Pin GPIO_PIN_7
#define SPI3_INT_GPIO_Port GPIOD
#define SPI3_INT_EXTI_IRQn EXTI9_5_IRQn
#define I2C1_INT2_Pin GPIO_PIN_5
#define I2C1_INT2_GPIO_Port GPIOB
#define I2C1_INT1_Pin GPIO_PIN_9
#define I2C1_INT1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
