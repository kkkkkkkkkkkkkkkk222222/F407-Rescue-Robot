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
#include "stm32f4xx_hal.h"

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
#define MOTOR_PWM_KEY_Pin GPIO_PIN_0
#define MOTOR_PWM_KEY_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_5
#define BIN1_GPIO_Port GPIOE
#define BIN2_Pin GPIO_PIN_6
#define BIN2_GPIO_Port GPIOE
#define AIN1_Pin GPIO_PIN_2
#define AIN1_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_3
#define AIN2_GPIO_Port GPIOA
#define M2A_Pin GPIO_PIN_6
#define M2A_GPIO_Port GPIOA
#define M2B_Pin GPIO_PIN_7
#define M2B_GPIO_Port GPIOA
#define TFT_DC_Pin GPIO_PIN_5
#define TFT_DC_GPIO_Port GPIOC
#define TFT_BLK_Pin GPIO_PIN_1
#define TFT_BLK_GPIO_Port GPIOB
#define M1A_Pin GPIO_PIN_9
#define M1A_GPIO_Port GPIOE
#define M1B_Pin GPIO_PIN_11
#define M1B_GPIO_Port GPIOE
#define CIN1_Pin GPIO_PIN_10
#define CIN1_GPIO_Port GPIOB
#define CIN2_Pin GPIO_PIN_11
#define CIN2_GPIO_Port GPIOB
#define TFT_RST_Pin GPIO_PIN_14
#define TFT_RST_GPIO_Port GPIOB
#define TFT_CS_Pin GPIO_PIN_12
#define TFT_CS_GPIO_Port GPIOB
#define M3A_Pin GPIO_PIN_12
#define M3A_GPIO_Port GPIOD
#define M3B_Pin GPIO_PIN_13
#define M3B_GPIO_Port GPIOD
#define PWM1_Pin GPIO_PIN_6
#define PWM1_GPIO_Port GPIOC
#define PWM2_Pin GPIO_PIN_7
#define PWM2_GPIO_Port GPIOC
#define PWM3_Pin GPIO_PIN_8
#define PWM3_GPIO_Port GPIOC
#define PWM4_Pin GPIO_PIN_9
#define PWM4_GPIO_Port GPIOC
#define M4AEXTI_Pin GPIO_PIN_3
#define M4AEXTI_GPIO_Port GPIOD
#define M4BI_Pin GPIO_PIN_4
#define M4BI_GPIO_Port GPIOD
#define DIN1_Pin GPIO_PIN_8
#define DIN1_GPIO_Port GPIOB
#define DIN2_Pin GPIO_PIN_9
#define DIN2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
