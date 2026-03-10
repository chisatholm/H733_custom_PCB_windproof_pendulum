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
#include "matrix_f64.h"
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AC_USER_CONSTANT 69.0
#define BMI270_CSn_Pin GPIO_PIN_3
#define BMI270_CSn_GPIO_Port GPIOE
#define SOA_Pin GPIO_PIN_0
#define SOA_GPIO_Port GPIOC
#define SOB_Pin GPIO_PIN_2
#define SOB_GPIO_Port GPIOC
#define SOC_Pin GPIO_PIN_3
#define SOC_GPIO_Port GPIOC
#define MP6540H_EN_Pin GPIO_PIN_8
#define MP6540H_EN_GPIO_Port GPIOE
#define MTR_A_Pin GPIO_PIN_9
#define MTR_A_GPIO_Port GPIOE
#define MP6540H_SLEEPn_Pin GPIO_PIN_10
#define MP6540H_SLEEPn_GPIO_Port GPIOE
#define MTR_B_Pin GPIO_PIN_11
#define MTR_B_GPIO_Port GPIOE
#define MP6540H_FAULTn_Pin GPIO_PIN_12
#define MP6540H_FAULTn_GPIO_Port GPIOE
#define MTR_C_Pin GPIO_PIN_13
#define MTR_C_GPIO_Port GPIOE
#define LED_0_Pin GPIO_PIN_10
#define LED_0_GPIO_Port GPIOD
#define LED_1_Pin GPIO_PIN_11
#define LED_1_GPIO_Port GPIOD
#define LED_2_Pin GPIO_PIN_12
#define LED_2_GPIO_Port GPIOD
#define LED_3_Pin GPIO_PIN_13
#define LED_3_GPIO_Port GPIOD
#define AS5048A_SCK_Pin GPIO_PIN_10
#define AS5048A_SCK_GPIO_Port GPIOC
#define AS5048A_MISO_Pin GPIO_PIN_11
#define AS5048A_MISO_GPIO_Port GPIOC
#define AS5048A_MOSI_Pin GPIO_PIN_12
#define AS5048A_MOSI_GPIO_Port GPIOC
#define AS5048A_CSn_Pin GPIO_PIN_0
#define AS5048A_CSn_GPIO_Port GPIOD
#define METAPING_IO_Pin GPIO_PIN_7
#define METAPING_IO_GPIO_Port GPIOD
#define METAPING_IO_EXTI_IRQn EXTI9_5_IRQn
#define SHOULD_BE_NC_Pin GPIO_PIN_1
#define SHOULD_BE_NC_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
