/*
  ******************************************************************************
  * @file           : ads1115.h
  * @brief          : ADS1115 mapa de registros
  *                   Este archivo contiene los registros del ADC - ADS1115
  ******************************************************************************/

#ifndef ADS1115_H_
#define ADS1115_H_

/* ADDRES I2C ADS1115 */
#define ADS1115_ADDR  			0b1001000 << 1


/* ADDRES POINTER REGISTER
 * All four registers are accessed by writing to the Address Pointer register
 */
#define ADDR_POINT_MASK			0x3U

/* States of ADDR POINTER REGISTER */
#define CONVERSION_REGISTER		0b00U
#define CONFIG_REGISTER			0b01U
#define LO_THRESH_REGISTER		0b10U
#define HI_THRESH_REGISTER		0b11U


/* CONFIG REGISTER
 * The 16-bit Config register is used to control the operating mode, input selection, data rate, full-scale range, and
 * comparator modes.
 */
/* Definitions mask */
#define OS_MASK					0x8000U
#define	MUX_MASK				0x7000U
#define	PGA_MASK				0x0E00U
#define	MODE_MASK				0x0100U
#define	DR_MASK					0x00E0U
#define	COMP_MODE_MASK			0x0010U
#define COMP_POL_MASK			0x0008U
#define	COMP_LAT_MASK			0x0004U
#define	COMP_QUE_MASK			0x0003U

/* Definitions offset */
#define OS_OFFSET				15U
#define	MUX_OFFSET				12U
#define	PGA_OFFSET				9U
#define	MODE_OFFSET				8U
#define	DR_OFFSET				5U
#define	COMP_MODE_OFFSET		4U
#define COMP_POL_OFFSET			3U
#define	COMP_LAT_OFFSET			2U
#define	COMP_QUE_OFFSET			0U

/* States of OS */
#define OS_NOT_EFFECT			0b0U /* No effect */
#define OS_START_1CONVERSION	0b1U /* Start a single conversion */

/* States of MUX */
#define AIN_01					0b000U /* AINp = AIN0 and AINn = AIN1 */
#define AIN_03					0b001U /* AINP = AIN0 and AINN = AIN3 */
#define AIN_13 					0b010U /* AINP = AIN1 and AINN = AIN3 */
#define AIN_23 					0b011U /* AINP = AIN2 and AINN = AIN3 */
#define AIN_0 					0b100U /* AINP = AIN0 and AINN = GND */
#define AIN_1 					0b101U /* AINP = AIN1 and AINN = GND */
#define AIN_2 					0b110U /* AINP = AIN2 and AINN = GND */
#define AIN_3 					0b111U /* AINP = AIN3 and AINN = GND */

/* States of PGA */
#define FSR_6144				0b000U /* FSR = 6.144V */
#define FSR_4096				0b001U /* FSR = 4.096V */
#define FSR_2048				0b010U /* FSR = 2.048V */
#define FSR_1024				0b011U /* FSR = 1.024V */
#define FSR_0512				0b100U /* FSR = 0.512V */
#define FSR_0256				0b101U /* FSR = 0.256V */

/* States of MODE */
#define MODE_CONTINUOS			0b0U /* Continuos conversion mode */
#define MODE_SINGLE				0b1U /* Single shot mode or power down state */

/* States of Data rate */
#define DR_8					0b000U /* 8 SPS */
#define DR_16					0b001U /* 16 SPS */
#define DR_32					0b010U /* 32 SPS */
#define DR_64					0b011U /* 64 SPS */
#define DR_128					0b100U /* 128 SPS */
#define DR_250					0b101U /* 250 SPS */
#define DR_475					0b110U /* 475 SPS */
#define DR_860					0b111U /* 860 SPS */

/* States of Comparator mode */
#define COMP_TRADITIONAL		0b0U /* Traditional comparator */
#define COMP_WINDOW				0b1U /* Window comparator */

/* States of Comparator polarity */
#define COMP_LO					0b0U /* Active Low */
#define COMP_HI					0b1U /* Active High */

/* States of Latching comparator */
#define COMP_NON_LATCH 			0b0U /* Nonlatching comparator */
#define COMP_LATCH				0b1U /* Latching comparator */

/* States of comparator queue and disable */
#define COMP_1					0b00U /* Assert after one conversion */
#define COMP_2					0b01U /* Assert after two conversion */
#define COMP_4					0b10U /* Assert after four conversion */
#define COMP_DISABLE			0b11U /* Disable comparator and set ALERT/RDY pin to high-impedance */

/* Macro get config register */
#define GET_CONFIG_REGISTER(OS,MUX,PGA,MODE,DR,COMP_MODE,COMP_POL,COMP_LAT,COMP_QUE)	(uint16_t)(((OS << OS_OFFSET) & OS_MASK) | \
																								   ((MUX << MUX_OFFSET) & MUX_MASK)| \
																								   ((PGA << PGA_OFFSET) & PGA_MASK)| \
																								   ((MODE << MODE_OFFSET) & MODE_MASK) | \
																								   ((DR << DR_OFFSET) & DR_MASK) | \
																								   ((COMP_MODE << COMP_MODE_OFFSET) & COMP_MODE_MASK) | \
																								   ((COMP_POL << COMP_POL_OFFSET) & COMP_POL_MASK) | \
																								   ((COMP_LAT << COMP_LAT_OFFSET) & COMP_LAT_MASK) | \
																								   ((COMP_QUE << COMP_QUE_OFFSET) & COMP_QUE_MASK))


/* LO_THRESH REGISTER
 * HI_THRESH REGISTER
 *
 * The upper and lower threshold values used by the comparator are stored in two 16-bit registers in two's
 * complement format. The comparator is implemented as a digital comparator; therefore, the values in these
 * registers must be updated whenever the PGA settings are changed.
 * The conversion-ready function of the ALERT/RDY pin is enabled by setting the Hi_thresh register MSB to 1 and
 * the Lo_thresh register MSB to 0.
 */

#define LO_THRESH				0x8000U /* Low threshold value */
#define HI_THRESH				0x7FFFU /* High threshold value */


#endif

