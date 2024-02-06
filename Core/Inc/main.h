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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TRIGGER_Pin GPIO_PIN_13
#define TRIGGER_GPIO_Port GPIOC
#define CONEXT_Pin GPIO_PIN_14
#define CONEXT_GPIO_Port GPIOC
#define EMOS2_Pin GPIO_PIN_15
#define EMOS2_GPIO_Port GPIOC
#define VTO_Pin GPIO_PIN_0
#define VTO_GPIO_Port GPIOB
#define EMOS1_Pin GPIO_PIN_1
#define EMOS1_GPIO_Port GPIOB
#define UICLK_Pin GPIO_PIN_13
#define UICLK_GPIO_Port GPIOB
#define VINV_Pin GPIO_PIN_8
#define VINV_GPIO_Port GPIOA
#define ISCALE_Pin GPIO_PIN_9
#define ISCALE_GPIO_Port GPIOA
#define VSCALE_Pin GPIO_PIN_12
#define VSCALE_GPIO_Port GPIOA
#define ADC_RDY_Pin GPIO_PIN_8
#define ADC_RDY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
//Definicion Hardware
#define ADC_HW	ADS1115
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
