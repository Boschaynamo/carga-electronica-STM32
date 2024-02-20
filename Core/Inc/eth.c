/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "wizchip_init.h"
#include "wizchip_conf.h"
#include "httpServer/httpServer.h"
#include "webpage.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_HTTPSOCK	4
#define DATA_BUF_SIZE	2048

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
wiz_NetInfo defaultNetInfo = { .mac = {0x46,0x08,0xdc,0xff,0xee,0xdd},
		.ip = {192,168,0,130},
		.sn = {255,255,255,0},
		.gw = {192,168,0,1},
		//.dns = {168, 126, 63, 1},
		.dns = {181, 30, 140, 135},
		.dhcp = NETINFO_STATIC};
uint8_t ethBuf0[2048];
uint8_t RX_BUF[DATA_BUF_SIZE];
uint8_t TX_BUF[DATA_BUF_SIZE];
uint8_t socknumlist[] = {0, 1, 2, 3};
uint8_t jsonStringToSend[20];
uint8_t DATOX_CAMBIAR = 30;
uint8_t DATOY_CAMBIAR = 25;

/* USER CODE END PV */


/* USER CODE BEGIN PFP */

void print_network_information(void);
void wep_define_func(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
void eth_start(void)
{

	/* USER CODE BEGIN 2 */
	WIZCHIPInitialize();
	HAL_Delay(3000);
	wizchip_setnetinfo(&defaultNetInfo);
	print_network_information();
	wizchip_getnetinfo(&defaultNetInfo);

	/* HTTP Server Initialization  */
	httpServer_init(TX_BUF, RX_BUF, MAX_HTTPSOCK, socknumlist);		// Tx/Rx buffers (1kB) / The number of W5500 chip H/W sockets in use


	sprintf(jsonStringToSend,"{\"x\":%d,\"y\":%d}",DATOX_CAMBIAR,DATOY_CAMBIAR);
	/* Web content registration (web content in webpage.h, Example web pages) */
	wep_define_func();


	/* USER CODE END 2 */

}


void print_network_information(void)
{
	wizchip_getnetinfo(&defaultNetInfo);
	printf("Mac address: %02x:%02x:%02x:%02x:%02x:%02x\n\r",defaultNetInfo.mac[0],defaultNetInfo.mac[1],defaultNetInfo.mac[2],defaultNetInfo.mac[3],defaultNetInfo.mac[4],defaultNetInfo.mac[5]);
	printf("IP address : %d.%d.%d.%d\n\r",defaultNetInfo.ip[0],defaultNetInfo.ip[1],defaultNetInfo.ip[2],defaultNetInfo.ip[3]);
	printf("SM Mask	   : %d.%d.%d.%d\n\r",defaultNetInfo.sn[0],defaultNetInfo.sn[1],defaultNetInfo.sn[2],defaultNetInfo.sn[3]);
	printf("Gate way   : %d.%d.%d.%d\n\r",defaultNetInfo.gw[0],defaultNetInfo.gw[1],defaultNetInfo.gw[2],defaultNetInfo.gw[3]);
	printf("DNS Server : %d.%d.%d.%d\n\r",defaultNetInfo.dns[0],defaultNetInfo.dns[1],defaultNetInfo.dns[2],defaultNetInfo.dns[3]);
}

void wep_define_func(void)
{
	// Index page and netinfo / base64 image demo
	reg_httpServer_webContent((uint8_t *)"index.html", (uint8_t *)index_page);				// index.html 		: Main page example
	reg_httpServer_webContent((uint8_t *)"netinfo.html", (uint8_t *)netinfo_page);			// netinfo.html 	: Network information example page
	reg_httpServer_webContent((uint8_t *)"netinfo.js", (uint8_t *)w5x00web_netinfo_js);	// netinfo.js 		: JavaScript for Read Network configuration 	(+ ajax.js)
	reg_httpServer_webContent((uint8_t *)"img.html", (uint8_t *)img_page);					// img.html 		: Base64 Image data example page

	// Example #1
	reg_httpServer_webContent((uint8_t *)"dio.html", (uint8_t *)dio_page);					// dio.html 		: Digital I/O control example page
	reg_httpServer_webContent((uint8_t *)"dio.js", (uint8_t *)w5x00web_dio_js);			// dio.js 			: JavaScript for digital I/O control 	(+ ajax.js)

	// AJAX JavaScript functions
	reg_httpServer_webContent((uint8_t *)"ajax.js", (uint8_t *)w5x00web_ajax_js);			// ajax.js			: JavaScript for AJAX request transfer

	// Registro nueva ruta
	reg_httpServer_webContent((uint8_t *)"grafico.html", (uint8_t *)grafico_page);

}

/* USER CODE END 4 */
