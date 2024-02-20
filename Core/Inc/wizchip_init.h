/*
 * wizchip_init.h
 *
 *  Created on: Dec 27, 2022
 *      Author: cogli
 */

#ifndef __WIZCHIP_INIT_H__
#define __WIZCHIP_INIT_H__


#include "main.h"
#include "wizchip_conf.h"


/* CS */
extern SPI_HandleTypeDef hspi1;
#define WIZCHIP_SPI  			hspi1
#define WIZCHIP_CS_PIN			ETH_CS_Pin
#define WIZCHIP_CS_PORT			ETH_CS_GPIO_Port


void WIZCHIPInitialize();

void csEnable(void);
void csDisable(void);
void spiWriteByte(uint8_t tx);
uint8_t spiReadByte(void);

#endif
