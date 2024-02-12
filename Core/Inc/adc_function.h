/*
 * adc_function.h
 *
 *  Created on: 27 sep. 2023
 *      Author: nicol
 */

#ifndef INC_ADC_FUNCTION_H_
#define INC_ADC_FUNCTION_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stdbool.h"

#define ADC_OS				OS_START_1CONVERSION
#define AIN_VOLTAGE			AIN_23
#define AIN_CURRENT			AIN_01
#define ADC_PGA				FSR_2048
#define ADC_MODE			MODE_SINGLE
#define ADC_SPS				DR_860
#define ADC_COMP_MODE		COMP_TRADITIONAL
#define ADC_COMP_POL		COMP_LO
#define ADC_COMP_LAT		COMP_NON_LATCH
#define ADC_COMP_QUE		COMP_1

#define SCALE_CURRENT 		0.94487 //mA por cuenta FSR=2.048V
#define	OFFSET_CURRENT		-198.42205 //Offset mA
#define SCALE_VOLTAGE16		1.05856//mV por cuenta FSR=4.096V
#define SCALE_VOLTAGE160	5.44794//mV por cuenta FSR=4.096V rango 160V
#define OFFSET_VOLTAGE16	95.30728//Offset mV
#define OFFSET_VOLTAGE160	203.14259 // Offset mV para rango 160v


enum input_to_measure {
	tension,
	corriente
};

float ADC_read_tension (I2C_HandleTypeDef *hi2c, bool rango); //return mV

float ADC_read_current (I2C_HandleTypeDef *hi2c); //return mA

void ADC_set_rdypin(I2C_HandleTypeDef *hi2c); //rdy pin active low

#endif /* INC_ADC_FUNCTION_H_ */
