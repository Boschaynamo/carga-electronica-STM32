/* USER CODE BEGIN Header */
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "adc_function.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Modos funcionamiento carga electronica */
enum modos_carga
{
  m_apagado = 0,
  m_corriente_off,
  m_tension_off,
  m_potenica_off,
  m_fusible_off,
  m_bateria_off,
  m_corriente,
  m_tension,
  m_potencia,
  m_fusible,
  m_bateria
};

/* Estructura estado modo de la carga electronica */
/* valor para corriente_constante en mA */
typedef struct
{

  enum modos_carga modo;         /* Modo de trabajo de la carga electronica */
  uint32_t valorTension;   /* Valor de Tension de la carga electronica */
  uint32_t valorCorriente; /* Valor de Corriente de la carga electronica */
  uint32_t valorPotencia;  /* Set point Potencia de la carga electronica */
  uint32_t setPoint;       /* Set point de la carga electronica */
  char flagTrigger;
  char flagFecha;

} CARGA_HandleTypeDef;

/* Estructura estado modo de la carga electronica */
typedef struct{
	float tension;
	float corriente;
} MEDICIONES_TypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Definiciones DAC - MCP4725
#define MCP4725_ADDR 0b1100000 << 1
#define MASK_DAC_READ 0b00001111
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

osThreadId defaultTaskHandle;
osThreadId medicionHandle;
osThreadId comunicacionHandle;
/* USER CODE BEGIN PV */

/* Memory pool para comunicacion spi */
osPoolDef(mpool, 5, char[21]); // Define memory pool
osPoolId mpool;
osMessageQDef(colaSPI_TX, 5, uint32_t); // Define message queue
osMessageQId colaSPI_TX;

osPoolDef(mpoolCARGA_Handle, 1, sizeof(CARGA_HandleTypeDef)); // Define memory pool
osPoolId mpoolCARGA_Handle;
osMessageQDef(colaCARGA, 1, uint32_t); // Define message queue
osMessageQId colaCARGA;


osPoolDef(mpoolMediciones, 5, sizeof(MEDICIONES_TypeDef)); // Define memory pool
osPoolId mpoolMediciones;
osMessageQDef(colaMediciones, 5, uint32_t); // Define message queue
osMessageQId colaMediciones;



// Definiciones para RTC
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_RTC_Init(void);
void StartDefaultTask(void const *argument);
void medicion_variables(void const *argument);
void comunicacion_spi(void const *argument);

/* USER CODE BEGIN PFP */
/* Prototipos funciones DAC MCP4725 I2C */
void DAC_init(void);
void DAC_set(float setPoint);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  mpool = osPoolCreate(osPool(mpool));                        // create memory pool
  colaSPI_TX = osMessageCreate(osMessageQ(colaSPI_TX), NULL); // create colaSPI_TX queue

  mpoolCARGA_Handle = osPoolCreate(osPool(mpoolCARGA_Handle));// create memory pool
  colaCARGA = osMessageCreate(osMessageQ(colaCARGA), NULL); // create colaCARGA queue

  mpoolMediciones = osPoolCreate(osPool(mpoolMediciones));// create memory pool
  colaMediciones = osMessageCreate(osMessageQ(colaMediciones), NULL); // create colaCARGA queue
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of medicion */
  osThreadDef(medicion, medicion_variables, osPriorityAboveNormal, 0, 128);
  medicionHandle = osThreadCreate(osThread(medicion), NULL);

  /* definition and creation of comunicacion */
  osThreadDef(comunicacion, comunicacion_spi, osPriorityBelowNormal, 0, 128);
  comunicacionHandle = osThreadCreate(osThread(comunicacion), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
   */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
   */
  sTime.Hours = 20;
  sTime.Minutes = 17;
  sTime.Seconds = 10;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 31;
  sDate.Year = 24;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ISCALE_Pin | VSCALE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TRIGGER_Pin CONEXT_Pin EMOS2_Pin */
  GPIO_InitStruct.Pin = TRIGGER_Pin | CONEXT_Pin | EMOS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : EMOS1_Pin ADC_RDY_Pin */
  GPIO_InitStruct.Pin = EMOS1_Pin | ADC_RDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : VINV_Pin */
  GPIO_InitStruct.Pin = VINV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VINV_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ISCALE_Pin VSCALE_Pin */
  GPIO_InitStruct.Pin = ISCALE_Pin | VSCALE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief DAC Inicializacion
 * @param None
 * @retval None
 */
void DAC_init(void)
{
  // Leer EEPROM y checkear que este en 0; Sino cambiarlo.
  uint16_t i2cRX;

  uint8_t buffer[5];
  memset(buffer, 0, 5);
  HAL_I2C_Master_Transmit(&hi2c1, MCP4725_ADDR, buffer, 2, HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(&hi2c1, MCP4725_ADDR, buffer, 5, HAL_MAX_DELAY);
  // Cargo el valor recibido del buffer a una variable para tener el valor de la eeprom.
  i2cRX = (buffer[3] << 8) || buffer[4];
  // Mascareo de rutix
  i2cRX &= MASK_DAC_READ;

  // Si el valor de la eeprom no es 0 lo cargo.
  if (i2cRX)
  {
    buffer[0] = 0b01100000;
    buffer[1] = 0;
    buffer[2] = 0;
    HAL_I2C_Master_Transmit(&hi2c1, MCP4725_ADDR, buffer, 3, HAL_MAX_DELAY);
    do
    {
      i2cRX = HAL_I2C_Master_Receive(&hi2c1, MCP4725_ADDR, buffer, 1, HAL_MAX_DELAY) && 0b10000000;
    } while (!i2cRX);
  }
  else
  {
    // Reset
    buffer[0] = 0;
    buffer[1] = 0;
    HAL_I2C_Master_Transmit(&hi2c1, MCP4725_ADDR, buffer, 2, HAL_MAX_DELAY);
  }
}

/**
 * @brief DAC Set
 * @param Setpoint - valor salida DAC en mV
 * @retval None
 */
void DAC_set(float setPoint)
{ // setPoint = valor en mV.
  uint8_t buffer[2];

  if (setPoint < 0 || setPoint >= 5000)
    return HAL_I2C_Master_Transmit(&hi2c1, MCP4725_ADDR, 0, 2, HAL_MAX_DELAY);

  setPoint = ((setPoint) * 4096) / 5000;
  buffer[0] = ((uint16_t)setPoint) >> 8;
  buffer[1] = ((uint16_t)setPoint);
  HAL_I2C_Master_Transmit(&hi2c1, MCP4725_ADDR, buffer, 2, HAL_MAX_DELAY);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{
  /* USER CODE BEGIN 5 */
	//*****Variables**//
	CARGA_HandleTypeDef CARGAPresente;
	CARGA_HandleTypeDef *CARGAUpdate;
	osEvent evt;


	//****************//

  /* Inicializo DAC */
  DAC_init();
  float set_point = 150;

  /* Infinite loop */
  for (;;)
  {
	  //Recibo de tu colita punto com
	  evt = osMessageGet(colaCARGA, 1);
	  if(evt.status==osEventMessage){
		  CARGAUpdate = evt.value.p;

		  if(CARGAPresente.modo != CARGAUpdate->modo){
			  CARGAPresente.modo = CARGAUpdate->modo;
			  switch(CARGAPresente.modo){
				  case m_apagado:
				  case m_bateria_off:
				  case m_corriente_off:
				  case m_fusible_off:
				  case m_potenica_off:
				  case m_tension_off:
					  DAC_set(0);
					break;



				  case m_corriente:
					  if(CARGAUpdate->setPoint>=0 && CARGAUpdate->setPoint<=30000){
						  CARGAPresente.setPoint = CARGAUpdate->setPoint;
						  DAC_set(CARGAPresente.setPoint/10); //IMPORTANTISIMO EL /10
					  }
					 break;



				  case m_potencia:
					 break;

				  case m_tension:
					 break;

				  case m_bateria:
					  break;
				  case m_fusible:
					  break;

				  default:
			  }
		  }

		  osPoolFree(mpoolCARGA_Handle, CARGAUpdate);
	  }



    DAC_set(set_point); // Seteo corriente en 1500mA

    // A LA BU RAR


    osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_medicion_variables */
/**
 * @brief Function implementing the medicion thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_medicion_variables */
void medicion_variables(void const *argument)
{
  /* USER CODE BEGIN medicion_variables */
  float tension = 5.21, tension_ant = 0;
  float current = 5.44, current_ant = 0;

  MEDICIONES_TypeDef* cargaMediciones;

  /* Infinite loop */
  for (;;)
  {
	cargaMediciones = osPoolAlloc(mpoolMediciones);
    /* Medicion de Corriente */
    current = current_ant * (1 - 0.4) + 0.4 * ADC_read_current(&hi2c1);
    if (current > 9999 || current < 0)
      current = 1234;
    current_ant = current;
    cargaMediciones->corriente=current;

    /* Medicion de Tension */
    tension = tension_ant * (1 - 0.6) + 0.6 * ADC_read_tension(&hi2c1);
    if (tension > 9999 || tension < 0)
      tension = 1234;
    tension_ant = tension;
    cargaMediciones->tension=tension;

    osMessagePut(colaMediciones, (uint32_t) cargaMediciones, osWaitForever);
  }
  /* USER CODE END medicion_variables */
}

/* USER CODE BEGIN Header_comunicacion_spi */
/**
 * @brief Function implementing the comunicacion thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_comunicacion_spi */
void comunicacion_spi(void const *argument)
{
  /* USER CODE BEGIN comunicacion_spi */
  uint8_t copiaArrayFromCola[21] = "HOLA";
  uint8_t array_SPI_RX[21] = "soyTuPuntero";

  CARGA_HandleTypeDef *updateCarga;
  updateCarga = osPoolAlloc(mpoolCARGA_Handle); //Te pongo en la pool bro

  // Queue defitinions
  char *pArrayFromCola;
  osEvent evt;

  /* Infinite loop */
  for (;;)
  {
    // RECIBO POR COLA Y SI NO BLOQUEO
    evt = osMessageGet(colaSPI_TX, osWaitForever);
    // RESPONDO EN BASE LA COLA

    if (evt.status == osEventMessage)
    {
      pArrayFromCola = evt.value.p;
      strcpy((char*)copiaArrayFromCola,pArrayFromCola);

      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive(&hspi2, copiaArrayFromCola, array_SPI_RX, 21, HAL_MAX_DELAY);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

      if (array_SPI_RX[0] == 'r' && array_SPI_RX[1] == 's')
      {

        // Recibo y guardo setpoint y modo
        updateCarga->modo = (enum modos_carga)(((uint8_t)array_SPI_RX[3] - 48) * 10 + ((uint8_t)array_SPI_RX[4] - 48));
        sscanf((char*) array_SPI_RX, "%*[^S]S%4lu", &updateCarga->setPoint); //Actualizo el setpoint en el CargaHandle local de esta tarea
        updateCarga->setPoint *=10;
        //*************************//

        //Aca hago el trigger
        updateCarga->flagTrigger = array_SPI_RX[13];
        if (array_SPI_RX[13] == '1')
        {
          // TRIGGER ACTIVADO Y LISTO PARA EL SERVICIO
        }

        // Si hay que modificar fecha
        updateCarga->flagFecha = array_SPI_RX[11];
        if (array_SPI_RX[11] == '1')
        {
          uint8_t arrayFecha[21] = "hmF";
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
          HAL_SPI_TransmitReceive(&hspi2, arrayFecha, array_SPI_RX, 21, HAL_MAX_DELAY);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

          //Aca guardo la fecha y hora
          sscanf((char*) array_SPI_RX, "%*[^F]F%2c%2c%2c%2c%2c", &sTime.Hours, &sTime.Minutes, &sDate.Date, &sDate.Month, &sDate.Year);
          HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
          HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
          //*********************//
        }
      }

      osPoolFree(mpool, pArrayFromCola); // free memory allocated for message
    }
    osMessagePut(colaCARGA, (uint32_t) updateCarga, osWaitForever);


    // Y NO HAGO NADA MAS PORQUE SOY UNA TAREA SENCILLA Y TONTA

  }
  /* USER CODE END comunicacion_spi */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
