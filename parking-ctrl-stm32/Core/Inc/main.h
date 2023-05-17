/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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
#define SERVO_analog_Pin GPIO_PIN_4
#define SERVO_analog_GPIO_Port GPIOA
#define GREEN_3_Pin GPIO_PIN_5
#define GREEN_3_GPIO_Port GPIOA
#define GREEN_4_Pin GPIO_PIN_6
#define GREEN_4_GPIO_Port GPIOA
#define YELLOW_5_Pin GPIO_PIN_7
#define YELLOW_5_GPIO_Port GPIOA
#define SERVO_Pin GPIO_PIN_6
#define SERVO_GPIO_Port GPIOC
#define RED_7_Pin GPIO_PIN_7
#define RED_7_GPIO_Port GPIOC
#define RED_8_Pin GPIO_PIN_9
#define RED_8_GPIO_Port GPIOA
#define Buzzer_pin_Pin GPIO_PIN_4
#define Buzzer_pin_GPIO_Port GPIOB
#define YELLOW_6_Pin GPIO_PIN_6
#define YELLOW_6_GPIO_Port GPIOB
#define BLUE_1_Pin GPIO_PIN_8
#define BLUE_1_GPIO_Port GPIOB
#define BLUE_2_Pin GPIO_PIN_9
#define BLUE_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
