/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
#include "adc_function.h"
#include "string.h"
#include "main.h"

#ifdef ADC_HW
	#if ADC_HW == ADS1115
		#include "ads1115.h"
		#define ADC_ADDR ADS1115_ADDR
	#endif
#endif

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variable ----------------------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
static void ADC_set (I2C_HandleTypeDef *hi2c, enum input_to_measure entrada);

float ADC_read_tension (I2C_HandleTypeDef *hi2c){

	uint8_t buffer[2];
	int16_t buffer16;

	ADC_set(hi2c, tension);
	osDelay(1);

	HAL_I2C_Master_Receive(hi2c, ADC_ADDR, buffer, 2, HAL_MAX_DELAY);

	buffer16 = buffer[0] << 8 | buffer[1];

	return (float)buffer16 * SCALE_VOLTAGE + OFFSET_VOLTAGE; //Devuelvo mV
}

float ADC_read_current (I2C_HandleTypeDef *hi2c){

	uint8_t buffer[2];
	int16_t buffer16;

	ADC_set(hi2c, corriente);
	osDelay(1);

	HAL_I2C_Master_Receive(hi2c, ADC_ADDR, buffer, 2, HAL_MAX_DELAY);

	buffer16 = buffer[0] << 8 | buffer[1];

	return (float)buffer16 *  SCALE_CURRENT + OFFSET_CURRENT; //Devuelvo mA //Sumo la mitad por diferencial
}


static void ADC_set (I2C_HandleTypeDef *hi2c, enum input_to_measure entrada){

	uint8_t buffer[3];
	uint16_t buffer16;

	buffer[0] = (uint8_t)CONFIG_REGISTER;
	if( entrada == tension){
		buffer16 = GET_CONFIG_REGISTER(ADC_OS,AIN_VOLTAGE,FSR_4096,ADC_MODE,ADC_SPS,ADC_COMP_MODE,ADC_COMP_POL,ADC_COMP_LAT,ADC_COMP_QUE);
	}
	else if( entrada == corriente ){
		buffer16 = GET_CONFIG_REGISTER(ADC_OS,AIN_CURRENT,FSR_2048,ADC_MODE,ADC_SPS,ADC_COMP_MODE,ADC_COMP_POL,ADC_COMP_LAT,ADC_COMP_QUE);
	}

	buffer[1] = (uint8_t)(buffer16 >> 8);
	buffer[2] = (uint8_t)(buffer16);

	HAL_I2C_Master_Transmit(hi2c, ADC_ADDR, buffer, 3, HAL_MAX_DELAY);
	//delay

	buffer[0] = (uint8_t)CONVERSION_REGISTER;
	HAL_I2C_Master_Transmit(hi2c, ADC_ADDR, buffer, 1, HAL_MAX_DELAY);
}
