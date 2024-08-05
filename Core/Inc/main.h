/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "debug.h"
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
#define LCD_SDO_Pin GPIO_PIN_2
#define LCD_SDO_GPIO_Port GPIOC
#define LCD_CSX_Pin GPIO_PIN_12
#define LCD_CSX_GPIO_Port GPIOB
#define LCD_SCL_Pin GPIO_PIN_13
#define LCD_SCL_GPIO_Port GPIOB
#define LCD_SDI_Pin GPIO_PIN_15
#define LCD_SDI_GPIO_Port GPIOB
#define LCD_DCX_Pin GPIO_PIN_9
#define LCD_DCX_GPIO_Port GPIOD
#define LCD_RESX_Pin GPIO_PIN_10
#define LCD_RESX_GPIO_Port GPIOD
#define CAM_RET_Pin GPIO_PIN_11
#define CAM_RET_GPIO_Port GPIOD
#define CAM_PWDN_Pin GPIO_PIN_12
#define CAM_PWDN_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
