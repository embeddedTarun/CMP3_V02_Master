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
#include "stm32f1xx_hal.h"

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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define LCD_BRIGHTNESS_Pin GPIO_PIN_14
#define LCD_BRIGHTNESS_GPIO_Port GPIOC
#define PoT_1_Pin GPIO_PIN_1
#define PoT_1_GPIO_Port GPIOA
#define PoT_2_Pin GPIO_PIN_2
#define PoT_2_GPIO_Port GPIOA
#define JS_PoT_V_Pin GPIO_PIN_3
#define JS_PoT_V_GPIO_Port GPIOA
#define JS_PoT_H_Pin GPIO_PIN_4
#define JS_PoT_H_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_0
#define SPI1_CS_GPIO_Port GPIOB
#define JS_SW_Pin GPIO_PIN_10
#define JS_SW_GPIO_Port GPIOB
#define RJ45_LED_G_Pin GPIO_PIN_14
#define RJ45_LED_G_GPIO_Port GPIOB
#define RJ45_LED_Y_Pin GPIO_PIN_15
#define RJ45_LED_Y_GPIO_Port GPIOB
#define MAX485_DI_Pin GPIO_PIN_9
#define MAX485_DI_GPIO_Port GPIOA
#define MAX484_RO_Pin GPIO_PIN_10
#define MAX484_RO_GPIO_Port GPIOA
#define MAX_EN_Pin GPIO_PIN_11
#define MAX_EN_GPIO_Port GPIOA
#define LCD_RS_Pin GPIO_PIN_4
#define LCD_RS_GPIO_Port GPIOB
#define LCD_E_Pin GPIO_PIN_5
#define LCD_E_GPIO_Port GPIOB
#define LCD_D4_Pin GPIO_PIN_6
#define LCD_D4_GPIO_Port GPIOB
#define LCD_D5_Pin GPIO_PIN_7
#define LCD_D5_GPIO_Port GPIOB
#define LCD_D6_Pin GPIO_PIN_8
#define LCD_D6_GPIO_Port GPIOB
#define LCD_D7_Pin GPIO_PIN_9
#define LCD_D7_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
