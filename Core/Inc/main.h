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
#define PWM_TENSION_Pin GPIO_PIN_14
#define PWM_TENSION_GPIO_Port GPIOC
#define prueba_Pin GPIO_PIN_15
#define prueba_GPIO_Port GPIOC
#define ETH_RST_Pin GPIO_PIN_3
#define ETH_RST_GPIO_Port GPIOA
#define ETH_CS_Pin GPIO_PIN_4
#define ETH_CS_GPIO_Port GPIOA
#define ETH_SCK_Pin GPIO_PIN_5
#define ETH_SCK_GPIO_Port GPIOA
#define ETH_MISO_Pin GPIO_PIN_6
#define ETH_MISO_GPIO_Port GPIOA
#define ETH_MOSI_Pin GPIO_PIN_7
#define ETH_MOSI_GPIO_Port GPIOA
#define VTO_Pin GPIO_PIN_0
#define VTO_GPIO_Port GPIOB
#define EMOS1_Pin GPIO_PIN_1
#define EMOS1_GPIO_Port GPIOB
#define EMOS1_EXTI_IRQn EXTI1_IRQn
#define UICLK_Pin GPIO_PIN_13
#define UICLK_GPIO_Port GPIOB
#define V_INV_Pin GPIO_PIN_8
#define V_INV_GPIO_Port GPIOA
#define V_INV_EXTI_IRQn EXTI9_5_IRQn
#define ISCALE_Pin GPIO_PIN_9
#define ISCALE_GPIO_Port GPIOA
#define BTN_TEST_Pin GPIO_PIN_10
#define BTN_TEST_GPIO_Port GPIOA
#define BTN_TEST_EXTI_IRQn EXTI15_10_IRQn
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
