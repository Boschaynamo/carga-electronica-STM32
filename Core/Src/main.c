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
#include "stdbool.h"
#include "stdlib.h"
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
	m_corriente_on,
    m_tension_off,
	m_tension_on,
    m_potencia_off,
	m_potencia_on,
	m_corriente_curva_off,
	m_corriente_curva_on,
	m_max
};
/* Errores que puede detectar la carga electronica */
union errores_carga{
	struct {
		uint8_t e_polaridadinversa :1;
		uint8_t e_mosfet:1;
		uint8_t e_expansion:1;
		uint8_t e_eth:1;
		uint8_t e_control:1;
		uint8_t e_temp:1;
		uint8_t e_corriente:1;
		uint8_t e_tension:1;
		uint8_t e_potencia:1;
	};
	uint32_t completo;
};
/* Estructura estado modo de la carga electronica */
/* valor para corriente_constante en mA */
typedef struct
{
  enum modos_carga modo;        /* Modo de trabajo de la carga electronica */
  union errores_carga error;	/* Errores carga electronica */
  uint32_t valorTension;   		/* Valor de Tension de la carga electronica */
  uint32_t valorCorriente; 		/* Valor de Corriente de la carga electronica */
  uint32_t valorPotencia;  		/* Set point Potencia de la carga electronica */
  uint32_t setPoint;       		/* Set point de la carga electronica */
  char flagTrigger;
  char flagFecha;
  uint32_t temperatura;

} CARGA_HandleTypeDef;

/* Estructura mediciones de la carga electronica */
typedef struct{
	float tension;
	float corriente;
	float potencia;
} MEDICIONES_TypeDef;

/* Estructura pid */
typedef struct{
	uint32_t referencia;
	enum modos_carga modo;

} PID_TypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Definiciones DAC - MCP4725
#define MCP4725_ADDR 0b1100000 << 1
#define MASK_DAC_READ 0b00001111

// Definiciones PID
#define PID_A 0.12
#define PID_B 0.08
#define PID_C 0.01
#define PID_MAX 3000
#define PID_MIN 0
#define PID_SETPOINT 0.0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

osThreadId defaultTaskHandle;
osThreadId medicionHandle;
osThreadId comunicacionHandle;
osThreadId pid_taskHandle;
osThreadId Task_ethHandle;
osThreadId control_curvaHandle;
osSemaphoreId sph_pid_medicionesHandle;
osSemaphoreId sph_eth_medicionesHandle;
osSemaphoreId sph_ctrl_curvaHandle;
/* USER CODE BEGIN PV */

//mpool y cola para transmision de datos a GUI
osPoolDef(mpool, 5, char[21]); // Define memory pool
osPoolId mpool;
osMessageQDef(colaSPI_TX, 5, uint32_t); // Define message queue
osMessageQId colaSPI_TX;

//mpool y cola para transmision de datos a tarea Carga control
osPoolDef(mpoolCARGA_Handle, 5, CARGA_HandleTypeDef); // Define memory pool
osPoolId mpoolCARGA_Handle;
osMessageQDef(colaCARGA, 5, uint32_t); // Define message queue
osMessageQId colaCARGA;

//mpool y cola para transmision de mediciones corriente, tension y potencia
osPoolDef(mpoolMediciones_pid, 5, MEDICIONES_TypeDef); // Define memory pool
osPoolId mpoolMediciones_pid;
osPoolDef(mpoolMediciones, 1, MEDICIONES_TypeDef); // Define memory pool
osPoolId mpoolMediciones;
osPoolDef(mpoolMediciones_eth, 1, MEDICIONES_TypeDef); // Define memory pool
osPoolId mpoolMediciones_eth;
osMessageQDef(colaMediciones, 1, uint32_t); // Define message queue
osMessageQId colaMediciones;
osMessageQDef(colaMediciones_pid, 1, uint32_t); // Define message queue
osMessageQId colaMediciones_pid;
osMessageQDef(colaMediciones_eth, 1, uint32_t); // Define message queue
osMessageQId colaMediciones_eth;

//mpool y cola para transmision de setpoint y modo a pid
osPoolDef(mpoolPID, 15, PID_TypeDef); // Define memory pool
osPoolId mpoolPID;
osMessageQDef(colaPID,15,uint32_t); // Define message queue
osMessageQId colaPID;

//cola para transmision de errores a tarea Carga control
osMessageQDef(colaErrores,5,uint32_t);// Define message queue
osMessageQId colaErrores;

//cola con datos para curva
osPoolDef(mpoolCurva,1,char[100]);
osPoolId mpoolCurva;
osMessageQDef(colaCurva,1,uint32_t);
osMessageQId colaCurva;

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
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
void medicion_variables(void const * argument);
void comunicacion_spi(void const * argument);
void pid_control(void const * argument);
void eth_task(void const * argument);
void cb_control_curva(void const * argument);

/* USER CODE BEGIN PFP */
/* Prototipos funciones DAC MCP4725 I2C */
void DAC_init(void);
void DAC_set(float setPoint);

/* Prototipos funciones ethernet */
extern void httpServer_run(uint8_t seqnum);
extern void eth_start(void);

/* Prototipos tareas */
void eth_task (void const * argument); //tarea ethernet
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
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of sph_pid_mediciones */
  osSemaphoreDef(sph_pid_mediciones);
  sph_pid_medicionesHandle = osSemaphoreCreate(osSemaphore(sph_pid_mediciones), 1);

  /* definition and creation of sph_eth_mediciones */
  osSemaphoreDef(sph_eth_mediciones);
  sph_eth_medicionesHandle = osSemaphoreCreate(osSemaphore(sph_eth_mediciones), 1);

  /* definition and creation of sph_ctrl_curva */
  osSemaphoreDef(sph_ctrl_curva);
  sph_ctrl_curvaHandle = osSemaphoreCreate(osSemaphore(sph_ctrl_curva), 1);

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
  mpoolMediciones_pid = osPoolCreate(osPool(mpoolMediciones_pid));// create memory pool
  mpoolMediciones_eth = osPoolCreate(osPool(mpoolMediciones_eth));// create memory pool

  colaMediciones = osMessageCreate(osMessageQ(colaMediciones), NULL); // create colaMediciones queue
  colaMediciones_pid = osMessageCreate(osMessageQ(colaMediciones_pid), NULL); // create colaMediciones_pid queue
  colaMediciones_eth = osMessageCreate(osMessageQ(colaMediciones_eth), NULL); // create colaMediciones_pid queue

  mpoolPID = osPoolCreate(osPool(mpoolPID));// create memory pool
  colaPID = osMessageCreate(osMessageQ(colaPID), NULL);// create colaPID queue

  colaErrores = osMessageCreate(osMessageQ(colaErrores), NULL);// Create colaErrores queue

  mpoolCurva = osPoolCreate(osPool(mpoolCurva)); // create memory pool
  colaCurva = osMessageCreate(osMessageQ(colaCurva), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of medicion */
  osThreadDef(medicion, medicion_variables, osPriorityAboveNormal, 0, 256);
  medicionHandle = osThreadCreate(osThread(medicion), NULL);

  /* definition and creation of comunicacion */
  osThreadDef(comunicacion, comunicacion_spi, osPriorityBelowNormal, 0, 512);
  comunicacionHandle = osThreadCreate(osThread(comunicacion), NULL);

  /* definition and creation of pid_task */
  osThreadDef(pid_task, pid_control, osPriorityAboveNormal, 0, 256);
  pid_taskHandle = osThreadCreate(osThread(pid_task), NULL);

  /* definition and creation of Task_eth */
  osThreadDef(Task_eth, eth_task, osPriorityNormal, 0, 4096);
  Task_ethHandle = osThreadCreate(osThread(Task_eth), NULL);

  /* definition and creation of control_curva */
  osThreadDef(control_curva, cb_control_curva, osPriorityHigh, 0, 256);
  control_curvaHandle = osThreadCreate(osThread(control_curva), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sTime.Hours = 23;
  sTime.Minutes = 17;
  sTime.Seconds = 10;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_FEBRUARY;
  sDate.Date = 12;
  sDate.Year = 24;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(prueba_GPIO_Port, prueba_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ETH_RST_GPIO_Port, ETH_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ETH_CS_Pin|ISCALE_Pin|VSCALE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : TRIGGER_Pin PWM_TENSION_Pin */
  GPIO_InitStruct.Pin = TRIGGER_Pin|PWM_TENSION_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : prueba_Pin */
  GPIO_InitStruct.Pin = prueba_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(prueba_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ETH_RST_Pin */
  GPIO_InitStruct.Pin = ETH_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ETH_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ETH_CS_Pin */
  GPIO_InitStruct.Pin = ETH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ETH_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EMOS1_Pin */
  GPIO_InitStruct.Pin = EMOS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(EMOS1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : V_INV_Pin */
  GPIO_InitStruct.Pin = V_INV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(V_INV_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ISCALE_Pin VSCALE_Pin */
  GPIO_InitStruct.Pin = ISCALE_Pin|VSCALE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_TEST_Pin */
  GPIO_InitStruct.Pin = BTN_TEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_TEST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_RDY_Pin */
  GPIO_InitStruct.Pin = ADC_RDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ADC_RDY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
  i2cRX = (buffer[3] << 8) | buffer[4];
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

  if (setPoint < 0 || setPoint >= 5000){
	  HAL_I2C_Master_Transmit(&hi2c1, MCP4725_ADDR, 0, 2, HAL_MAX_DELAY);
	  return;
  }

  setPoint = ((setPoint) * 4096) / 5000;
  buffer[0] = ((uint16_t)setPoint) >> 8;
  buffer[1] = ((uint16_t)setPoint);
  HAL_I2C_Master_Transmit(&hi2c1, MCP4725_ADDR, buffer, 2, HAL_MAX_DELAY);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	union errores_carga error;
	error.completo = 0;

	/* Interrupcion prueba btn_test */
	if(GPIO_Pin == BTN_TEST_Pin || GPIO_Pin == V_INV_Pin){
		if(HAL_GPIO_ReadPin(BTN_TEST_GPIO_Port, BTN_TEST_Pin) == GPIO_PIN_RESET || \
				HAL_GPIO_ReadPin(V_INV_GPIO_Port, V_INV_Pin) == GPIO_PIN_SET){
			error.e_polaridadinversa = true;
			osMessagePut(colaErrores,error.completo, 0);
		}
		else{
			error.e_polaridadinversa = true;
			osMessagePut(colaErrores,error.completo, 0);
		}
	} else {
		__NOP();
	}

	/* Interrupcion error mosfet */
	if(GPIO_Pin == EMOS1_Pin){
		if(HAL_GPIO_ReadPin(EMOS1_GPIO_Port, EMOS1_Pin) == GPIO_PIN_SET){
			error.e_mosfet = true;
			osMessagePut(colaErrores,error.completo, 0);
		}
		else{
			error.e_mosfet = true;
			osMessagePut(colaErrores,error.completo, 0);
		}
	} else {
		__NOP();
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	//osDelayUntil
	uint32_t previosWakeTime = osKernelSysTick();

	uint8_t c_error = 0, c_temp = 0;

	// Variables
	static CARGA_HandleTypeDef CARGAPresente;
	CARGA_HandleTypeDef *CARGAUpdate;
	osEvent evt;

	//flag fecha actualizada
	bool fecha_act = false;

	//flag para enviar solo 1 vez el semaforo
	bool flag_ctrl_curva = false;

	//Puntero a cadena transmision
	char *p_tx;

	//Puntero a mediciones de V e I
	MEDICIONES_TypeDef* p_mediciones;

	//Puntero a estructura pid
	PID_TypeDef* p_pid = NULL;
	uint32_t referencia = 0;

	// Inicializo variable carga electronica
	CARGAPresente.modo = m_apagado;
	CARGAPresente.error.completo = 0;
	CARGAPresente.setPoint = 0;
	CARGAPresente.valorCorriente = 0;
	CARGAPresente.valorTension = 0;
	CARGAPresente.valorPotencia = 0;
	CARGAPresente.flagFecha = 0;
	CARGAPresente.flagTrigger = 0;
	CARGAPresente.temperatura = 0;

	/* Inicializo DAC */
	DAC_init();

	/* Inicializo ADC */
	HAL_ADC_Start(&hadc1);

	/* Inicializo Fecha en GUI */
	//Espero 3 segundos que se inicialice GUI
	osDelay(3000);
	if(!fecha_act){
		for (uint8_t i=0; i<3;i++){
			p_tx = osPoolAlloc(mpool);
			if( p_tx != NULL ){
				sprintf(p_tx,"hmU%02d%02d%02d%02d%02d",sTime.Hours,\
						sTime.Minutes, sDate.Date, sDate.Month, sDate.Year);

				// Envio la cadena a transmitir task comunicacion_spi
				osMessagePut(colaSPI_TX, (uint32_t)p_tx, 50);
			}
			p_tx = NULL;
		}
		fecha_act = true;
	}

	/* Infinite loop */
	for (;;)
	{
		//Leo tension y corriente de la cola mediciones que envio la tarea medicion_variables.
		evt = osMessageGet(colaMediciones, 25);
		if(evt.status == osEventMessage){
			//Se tomo la medicion
			p_mediciones = evt.value.p;
			//Actualizo las variables de la carga electronica y verifico
			CARGAPresente.valorCorriente = (uint32_t)p_mediciones->corriente;
			CARGAPresente.valorTension = (uint32_t)p_mediciones->tension;
			CARGAPresente.valorPotencia = (uint32_t)p_mediciones->potencia;
			// Liberar memoria asignada para el mensaje
			osPoolFree(mpoolMediciones, p_mediciones);

		}else if(evt.status == osEventTimeout){
			//Error en la tasa de muestreo de la medicion
		}

		//Leo adc temperatura disipador y la envio cada 1segundo
		HAL_ADC_Start(&hadc1);
		CARGAPresente.temperatura = HAL_ADC_GetValue(&hadc1) * 3300 /4096 /10;
		if(c_temp++ == 10){
			p_tx = osPoolAlloc(mpool);
			if( p_tx != NULL ){

				// Carga la cadena a transmitir
				sprintf(p_tx,"hmR%03lu",(uint32_t)CARGAPresente.temperatura);

				// Envio la cadena a transmitir task comunicacion_spi
				osMessagePut(colaSPI_TX, (uint32_t)p_tx, 25);
			}
			c_temp = 0;//Reseteo flag
		}


		// Envio estado actual de la carga a la GUI
		p_tx = osPoolAlloc(mpool);
		if( p_tx != NULL ){

			// Carga la cadena a transmitir
			sprintf(p_tx,"hmM%02luC%04luV%04luP%04lu",(uint32_t)CARGAPresente.modo,\
					(uint32_t)CARGAPresente.valorCorriente/10,\
					(uint32_t)CARGAPresente.valorTension/10,\
					(uint32_t)CARGAPresente.valorPotencia/100);

			// Envio la cadena a transmitir task comunicacion_spi
			osMessagePut(colaSPI_TX, (uint32_t)p_tx, 25);
		}

		//Recibo msj de la GUI
		while(osMessageAvailableSpace(colaCARGA) < 4){
			evt = osMessageGet(colaCARGA, 40);
			if(evt.status == osEventMessage){
				//Recibi el msj de la GUI
				CARGAUpdate = evt.value.p;

				//Interpreto los modos
				if(CARGAUpdate->modo == m_apagado || \
						CARGAUpdate->modo == m_corriente_off ||\
						CARGAUpdate->modo == m_potencia_off  ||\
						CARGAUpdate->modo == m_tension_off   ||\
						CARGAUpdate->modo == m_corriente_curva_off){
					//Si el modo en GUI es apagado
					c_error = 0;//Reseteo contador error control
					CARGAPresente.error.e_control = 0;//Reseteo error control
				}
				//Modo seteado en GUI
				if(CARGAPresente.modo != CARGAUpdate->modo && !CARGAPresente.error.completo){
					//Si los modos son distintos y no hay error
					if(CARGAUpdate->modo == m_corriente_curva_on && flag_ctrl_curva == true)
						CARGAPresente.modo = m_apagado;
					else
						CARGAPresente.modo = CARGAUpdate->modo;//Cambio de modo
					c_error = 0;//Reseteo contador error control
					//Si apagaron la carga reseteo flag curva
				}
				//Setpoint en GUI
				if(CARGAPresente.setPoint != CARGAUpdate->setPoint){
					//Si cambio el setpoint
					CARGAPresente.setPoint = CARGAUpdate->setPoint;
					c_error = 0;//Reseteo contador error control
				}


				//Libero el msj de la memoria
				osPoolFree(mpoolCARGA_Handle, CARGAUpdate);
				CARGAUpdate = NULL;
			}
			else if(evt.status == osEventTimeout){
				//No recibi msj de la GUI en 200mS
			}
		}

		//Leo cola errores
		evt = osMessageGet(colaErrores, 0);
		if(evt.status == osEventMessage){
			CARGAPresente.error.completo |= (uint32_t)evt.value.v;
			osDelay(1);
			//Error mosfet
			if(CARGAPresente.error.e_mosfet == true)
				CARGAPresente.error.e_mosfet = HAL_GPIO_ReadPin(EMOS1_GPIO_Port, EMOS1_Pin);
			//Error tension inversa
			if(CARGAPresente.error.e_polaridadinversa == true)
				CARGAPresente.error.e_polaridadinversa = HAL_GPIO_ReadPin(V_INV_GPIO_Port, V_INV_Pin) \
				| !HAL_GPIO_ReadPin(BTN_TEST_GPIO_Port, BTN_TEST_Pin);
		}

		//Si hay errores apago la carga y envio a GUI
		if(CARGAPresente.error.completo != 0){
			CARGAPresente.modo = m_apagado;//Apago la carga
			HAL_GPIO_WritePin(prueba_GPIO_Port, prueba_Pin, GPIO_PIN_SET);
			// Envio error actual de la carga a la GUI
			p_tx = osPoolAlloc(mpool);
			if( p_tx != NULL ){

				// Carga la cadena a transmitir
				sprintf(p_tx,"hmE%02lu",(uint32_t)CARGAPresente.error.completo);

				// Envio la cadena a transmitir task comunicacion_spi
				osMessagePut(colaSPI_TX, (uint32_t)p_tx, osWaitForever);
			}
		}else{
			HAL_GPIO_WritePin(prueba_GPIO_Port, prueba_Pin, GPIO_PIN_RESET);
		}

		/* Controlo la carga electronica */
		switch(CARGAPresente.modo){
		/* Apago la carga */
		case  m_corriente_curva_off:
			flag_ctrl_curva = false;
		case m_apagado:
		case m_corriente_off:
		case m_tension_off:
			DAC_set(0);
			referencia = 0;

			break;

		case m_corriente_on:
			/* Modo corriente constante */
			if(CARGAPresente.setPoint > 30000)
				CARGAPresente.setPoint = 0;
			referencia = CARGAPresente.setPoint;
			//Si esta lejos del setpint
			if(abs(CARGAPresente.setPoint - CARGAPresente.valorCorriente) > 25){
				if(++c_error > 20){//Durante un segundo no pudo llegar al setpoint
					CARGAPresente.error.e_control = true;
					referencia = 0;
				}
			}
			else
				c_error = 0;
			break;

		case m_potencia_on:
			/* Modo potencia constante */
			if( CARGAPresente.setPoint > 500000)
				CARGAPresente.setPoint =  0;
			referencia = CARGAPresente.setPoint;
			//Si esta lejos del setpoint
			if(abs(CARGAPresente.setPoint - CARGAPresente.valorPotencia) > 100){
				if(++c_error > 20){//Durante un segundo no pudo llegar al setpoint
					CARGAPresente.error.e_control = true;
					referencia = 0;
				}
			}
			else
				c_error = 0;
			break;

		case m_corriente_curva_on:
			if(flag_ctrl_curva == false){
				osSemaphoreRelease(sph_ctrl_curvaHandle);
				flag_ctrl_curva = true;
			}
			break;

		default:
			DAC_set(0);
			referencia = 0;
		}

		if(CARGAPresente.modo != m_corriente_curva_on){
			//Reservo espacio para enviar msj a cola PID
			p_pid = osPoolAlloc(mpoolPID);
			if( p_pid != NULL){
				//Guardo el valor de referencia y modo a enviar
				p_pid->modo = CARGAPresente.modo;
				p_pid->referencia = referencia;
				osMessagePut(colaPID, (uint32_t)p_pid, 10);
			}
		}

		//Tarea periodica 100mS
		osDelayUntil(&previosWakeTime, 100);

	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_medicion_variables */
/**
 * @brief Function implementing the medicion thread.
 * @param argument: Not used
 * @retval None
 * Esta funcion toma mediciones del ADC de corriente y tension, les hace un
 * filtro EMA y devuelve los valores obtenidos cada 20Hz.
 */
/* USER CODE END Header_medicion_variables */
void medicion_variables(void const * argument)
{
  /* USER CODE BEGIN medicion_variables */
  float tension = 0, tension_ant = 0;
  float current = 0, current_ant = 0;
  float potencia = 0, potencia_ant = 0;
  float tension_gui = 0, tension_gui_ant = 0;
  float current_gui = 0, current_gui_ant = 0;
  float potencia_gui = 0, potencia_gui_ant = 0;

  /* Para poner a punto escalas y offset - Comentar si esta calibrado  */
  //float promedio = 1, c_mediciones=0;
  //HAL_GPIO_WritePin(VSCALE_GPIO_Port, VSCALE_Pin, GPIO_PIN_SET);//Rango 16V
  //HAL_GPIO_WritePin(VSCALE_GPIO_Port, VSCALE_Pin, GPIO_PIN_RESET);//Rango 160V

  // Estructura mediciones
  MEDICIONES_TypeDef *cargaMediciones;

  //contador para tarea carga control
  uint8_t i=0;

  // Rango 160V False - 16V True
  bool rango = false;
  uint8_t c_rango = 0;

  // Configuro ADS1115 para usar pin de RDY
  ADC_set_rdypin(&hi2c1);

  uint32_t previosWakeTime = osKernelSysTick();

  //Limpio semaforos
  osSemaphoreWait(sph_eth_medicionesHandle, 0);
  osSemaphoreWait(sph_pid_medicionesHandle, 0);

  /* Infinite loop */
  for (;;)
  {
	  /* Medicion de Corriente con filtro EMA */
	  current = current_ant * (1 - 0.8) + 0.8 * ADC_read_current(&hi2c1);
	  current_ant = current;
	  current_gui = current_gui_ant * (1-0.8) + 0.8 * current;
	  current_gui_ant = current_gui;
	  //promedio += current;
	  //c_mediciones++;
	  /* Medicion de Tension con filtro EMA*/
	  tension = tension_ant * (1 - 0.8) + 0.8 * ADC_read_tension(&hi2c1, rango);
	  tension_ant = tension;
	  tension_gui = tension_gui_ant * (1-0.4) + 0.4 * tension;
	  tension_gui_ant = tension_gui;

	  /* Medicion de Potencia con filtro EMA */
	  potencia = potencia_ant  * (1 - 0.8) + 0.8 * (tension * current)/1000;
	  potencia_ant = potencia;
	  potencia_gui = potencia_gui_ant * (1-0.3) + 0.3 * potencia;
	  potencia_gui_ant = potencia_gui;

	  // Si la tension < 0 normalizo
	  if (tension < 0)
		  tension = 0;

	  // Verificar en que rango estamos de tension
	  if(rango == true){
		  // Rango 16V

		  if( tension > 16000){
			  // Tension mayor al rango min
			  c_rango++;

			  if(c_rango > 2){
				  //Lo supero 3 veces consecutiva(6ms) -> cambio rango a 160V
				  rango = false;
				  HAL_GPIO_WritePin(VSCALE_GPIO_Port, VSCALE_Pin, GPIO_PIN_RESET);
				  c_rango = 0;
			  }
		  }
		  else // Tension menor al rango max
			  c_rango = 0;

	  } else {
		  // Rango 160V

		  if(tension < 15500){
			  // Tension menor al rango min
			  c_rango++;

			  if(c_rango > 20){
				  // Estuvo por debajo 20 veces consecutivas (40ms)
				  // -> Cambio rango a 16V.
				  rango = true;
				  HAL_GPIO_WritePin(VSCALE_GPIO_Port, VSCALE_Pin, GPIO_PIN_SET);
				  c_rango = 0;
			  }
		  }
		  else
			  c_rango = 0;
	  }

	  // Envio las mediciones tasa 10Hz.
	  if( i++ == 3){
		  cargaMediciones = osPoolAlloc(mpoolMediciones);
		  if ( cargaMediciones != NULL ){
			  // Guardo los valores de las variables a enviar.
			  cargaMediciones->corriente = current_gui;
			  cargaMediciones->tension = tension_gui;
			  cargaMediciones->potencia = potencia_gui;
			  osMessagePut(colaMediciones, (uint32_t)cargaMediciones, 0);
		  }
		  cargaMediciones = NULL;
		  i = 0;
	  }

	  // Envio las mediciones a tarea pid
	  if( osSemaphoreWait(sph_pid_medicionesHandle, 0) == osOK){
		  cargaMediciones = osPoolAlloc(mpoolMediciones_pid);
		  if ( cargaMediciones != NULL ){
			  cargaMediciones->corriente = current;
			  cargaMediciones->tension = tension;
			  cargaMediciones->potencia = potencia;
			  osMessagePut(colaMediciones_pid, (uint32_t)cargaMediciones, 2);
		  }
		  cargaMediciones = NULL;
	  }

	  //Envio las mediciones a tarea eth_task
	  if( osSemaphoreWait(sph_eth_medicionesHandle, 0) == osOK){
		  cargaMediciones = osPoolAlloc(mpoolMediciones_eth);
		  if ( cargaMediciones != NULL ){
			  cargaMediciones->corriente = current_gui;
			  cargaMediciones->tension = tension_gui;
			  cargaMediciones->potencia = potencia_gui;
			  osMessagePut(colaMediciones_eth, (uint32_t)cargaMediciones, 2);
		  }
		  cargaMediciones = NULL;
	  }

	  // Bloqueo la tarea 15ms * 10 veces = 50mS
	  osDelayUntil(&previosWakeTime, 15);
	  //HAL_GPIO_TogglePin(prueba_GPIO_Port, prueba_Pin);
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
void comunicacion_spi(void const * argument)
{
  /* USER CODE BEGIN comunicacion_spi */
	uint8_t array_SPI_RX[21] = "soyTuPuntero";

	CARGA_HandleTypeDef *updateCarga;

	//Obtengo fecha y hora guardados en RTC
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	// Definiciones de cola
	char *pArrayFromCola;
	osEvent evt;

	//Recuerdo de valores anteriores
	enum modos_carga modo_anterior;
	uint32_t setpoint_anterior;

	/* Infinite loop */
	for (;;)
	{
		// Recibo msj a transmitir de la task default
		evt = osMessageGet(colaSPI_TX, osWaitForever);
		// Respondo en base a los datos recibidos del ESP32
		if (evt.status == osEventMessage)
		{
			pArrayFromCola = evt.value.p;

			// Reservo un espacio de memoria para asignar los datos recibidos
			updateCarga = osPoolAlloc(mpoolCARGA_Handle);

			if( updateCarga != NULL){

				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
				HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)pArrayFromCola, array_SPI_RX, 21, HAL_MAX_DELAY);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

				// Interpreto cadena recibida
				if (array_SPI_RX[0] == 'r' && array_SPI_RX[1] == 's')
				{
					// Recibo y guardo setpoint y modo y verifico peticiones
					if(array_SPI_RX[2] == 'M'){
						updateCarga->modo = (enum modos_carga)(((uint8_t)array_SPI_RX[3] - 48) * 10 + ((uint8_t)array_SPI_RX[4] - 48));
						sscanf((char*) array_SPI_RX, "%*[^S]S%4lu", &updateCarga->setPoint); //Actualizo el setpoint en el CargaHandle local de esta tarea
						if(updateCarga->modo == m_corriente_on || updateCarga->modo == m_corriente_off)
							updateCarga->setPoint *=100;
						else if(updateCarga->modo == m_potencia_on|| updateCarga->modo == m_potencia_off)
							updateCarga->setPoint *= 100;
						else if(updateCarga->modo == m_tension_on|| updateCarga->modo == m_tension_off)
							updateCarga->setPoint *= 100;
						//Guardo el modo anterior y setpoint
						modo_anterior = updateCarga->modo;
						setpoint_anterior = updateCarga->setPoint;

						//Aca hago el trigger
						updateCarga->flagTrigger = array_SPI_RX[13];
						if (array_SPI_RX[13] == '1')
						{// TRIGGER ACTIVADO Y LISTO PARA EL SERVICIO
							__NOP();
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
						}

						//Pidieron sincronizar los datos de la curva (recibi A1)
						if(array_SPI_RX[15] == '1' ){
							osDelay(25);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
							HAL_SPI_TransmitReceive(&hspi2,"hmA1", array_SPI_RX, 21, HAL_MAX_DELAY);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
						}
					}

					//Empezaron a llegar los datos de la curva
					if(array_SPI_RX[2] == 'A'){
						//le pido los10 puntos
						char *msg_completo;
						char aux[21];
						msg_completo = osPoolAlloc(mpoolCurva);
						if( msg_completo != NULL){

							for(uint8_t i=0; i<21; i++){
								aux[i] = (char)array_SPI_RX[i];
							}
							strcat(msg_completo, aux);

							for(uint8_t contador = 0; contador < 4 ; contador++){
								osDelay(25);
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
								HAL_SPI_TransmitReceive(&hspi2,"hmA0", array_SPI_RX, 21, HAL_MAX_DELAY);
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
								for(uint8_t i=0; i<21; i++){
									aux[i] = (char)array_SPI_RX[i];
								}
								strcat(msg_completo, aux);
							}
							osMessagePut(colaCurva, (uint32_t)msg_completo, osWaitForever);
							osDelay(25);
						}
						updateCarga->modo = modo_anterior;
						updateCarga->setPoint = setpoint_anterior;
					}//Terminaron de llegar los datos de la curva
				}

				// Envio los datos analizados task default
				osMessagePut(colaCARGA, (uint32_t) updateCarga, osWaitForever);
				osDelay(25);
			}
			// Liberar memoria asignada para el mensaje recibido
			osPoolFree(mpool, pArrayFromCola);

		}
	}

  /* USER CODE END comunicacion_spi */
}

/* USER CODE BEGIN Header_pid_control */
/**
 * @brief Function implementing the pid_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pid_control */
void pid_control(void const * argument)
{
  /* USER CODE BEGIN pid_control */
	float a,b,c;                       		//constantes del PID voltaje
	float rT,eT,iT,dT,yT,yT0,uT,iT0,eT0;   	//variables de ecuaciones de PID voltaje
	float pid_max,pid_min;             		//límites máximo y mínimo de control.
	float setpoint_potencia = 0;			//setpoint potencia

	osEvent evt;

	uint32_t previosWakeTime = osKernelSysTick();

	//Puntero a estructura pid
	PID_TypeDef *p = NULL;
	enum modos_carga modo = m_apagado;

	//Puntero a mediciones de V e I
	MEDICIONES_TypeDef* p_mediciones;

	pid_min = PID_MIN;
	pid_max = PID_MAX;
	iT0 = 0.0;
	eT0 = 0.0;

	rT = PID_SETPOINT;

	a = PID_A;
	b = PID_B;
	c = PID_C;

	/* Infinite loop */
	for(;;)
	{

		//Recibo señal de referencia y modo
		evt = osMessageGet(colaPID, 0);
		if(evt.status == osEventMessage){

			p = evt.value.p;
			modo = (enum modos_carga)p->modo;
			if(modo == m_corriente_on)
				rT = (float)p->referencia; //Señal de referencia
			else if(modo == m_potencia_on)
				setpoint_potencia = (float)p->referencia;

			//Libero memoria asignada para el msj
			osPoolFree(mpoolPID, p);
		}

		//Si la carga esta encendida
		if(modo == m_corriente_on || modo == m_potencia_on){

			//Le digo a mediciones que quiero el dato
			osSemaphoreRelease(sph_pid_medicionesHandle);

			//Recibo valor de corriente medida
			evt = osMessageGet(colaMediciones_pid, osWaitForever);
			if(evt.status == osEventMessage){

				p_mediciones = evt.value.p;
				//Leo el valor de corriente
				yT = p_mediciones->corriente; //Señal de corriente


				if(modo == m_potencia_on){
					if( p_mediciones->tension > 0)
						rT = setpoint_potencia * 1000 / p_mediciones->tension;
					else
						rT = 0;
				}

				// Liberar memoria asignada para el mensaje
				osPoolFree(mpoolMediciones_pid, p_mediciones);
			}

			//Hago las tareas del pid
			if(rT != 0){
				//Si la referencia es distinta de 0

				eT = rT - yT; //Cálculo error corriente

				if(eT > 11 || eT < 11){

					iT = b * ( eT + eT0 ) + iT0; //Cálculo del término integral corriente

					/*Limite termino integral corriente*/
					if ( iT > pid_max )
						iT = pid_max; //Salida integral si es mayor que el MAX
					else if ( iT < pid_min )
						iT = pid_min; //Salida integral si es menor que el MIN

					dT = -c * ( yT - yT0 );	//Cálculo del término derivativo corriente
					uT = iT + a * eT + dT; //Cálculo de la salida PID corriente

					/*Limite PID corriente*/
					if ( uT > pid_max )
						uT = pid_max;           //Salida PID si es mayor que el MAX
					else if ( uT < pid_min )
						uT = pid_min;      //Salida PID si es menor que el MIN

					/* Guardar variables */
					iT0 = iT;
					eT0 = eT;
					yT0 = yT;

					DAC_set(uT);
					//DAC_set(400);
				}
				else
					eT0 = eT;
			}
			else
				DAC_set(0);

			//Bloqueo la tarea 5mS
			osDelayUntil(&previosWakeTime, 15);
		}
		else
			osDelay(70);

	}
  /* USER CODE END pid_control */
}

/* USER CODE BEGIN Header_eth_task */
/**
* @brief Function implementing the Task_eth thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_eth_task */
void eth_task(void const * argument)
{
  /* USER CODE BEGIN eth_task */
  /* Infinite loop */
  for(;;)
  {
	  /* Inicializo ethernet */
	  eth_start();

	  while(1){
		  //HTTP inicio
		  for(uint8_t j = 0; j < 2; j++)	httpServer_run(j); 	// HTTP Server handler
		  osDelay(250);
	  }
  }
  /* USER CODE END eth_task */
}

/* USER CODE BEGIN Header_cb_control_curva */
/**
* @brief Function implementing the control_curva thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_cb_control_curva */
void cb_control_curva(void const * argument)
{
  /* USER CODE BEGIN cb_control_curva */
	osEvent evt;

	//Punteros
	char *p;
	PID_TypeDef *p_pid;
	CARGA_HandleTypeDef *updateCarga;

	int matriz[10][2];

	//Limpio semaforo
	osSemaphoreWait(sph_ctrl_curvaHandle, 0);

  /* Infinite loop */
	for(;;)
	{
		evt = osMessageGet(colaCurva, 100);
		if(evt.status == osEventMessage){
			p = evt.value.p;

			for(uint8_t i=0; i<5; i++){

				sscanf(p + i * 20,"rsA%4d%4dA%4d%4d",&matriz[i*2][0], &matriz[i*2][1],&matriz[i*2+1][0],&matriz[i*2+1][1]);
			}
			//Libero memoria
			osPoolFree(mpoolCurva, p);
		}


		if(osSemaphoreWait(sph_ctrl_curvaHandle, 0) == osOK){

			//Le paso a la tarea pid las referencias cuando pase el tiempo
			for(uint8_t i=0; i<10 && matriz[i][0]!= 0; i++){

				p_pid = osPoolAlloc(mpoolPID);
				if( p_pid != NULL){
					//Guardo el valor de referencia y modo a enviar
					p_pid->modo = m_corriente_on;
					p_pid->referencia = matriz[i][1] * 100;
					osMessagePut(colaPID, (uint32_t)p_pid, osWaitForever);

					osDelay((uint32_t)(matriz[i][0]*100));
				}
			}//fin puntos

			p_pid = osPoolAlloc(mpoolPID);
			if( p_pid != NULL){
				//Guardo el valor de referencia y modo a enviar
				p_pid->modo = m_corriente_off;
				p_pid->referencia = 0;
				osMessagePut(colaPID, (uint32_t)p_pid, osWaitForever);
			}

			//Cuadno termino apago la carga
			// Reservo un espacio de memoria para asignar los datos recibidos
			updateCarga = osPoolAlloc(mpoolCARGA_Handle);
			while(updateCarga == NULL){
				osDelay(100);
				updateCarga = osPoolAlloc(mpoolCARGA_Handle);
			}
			if(updateCarga != NULL){
				updateCarga->modo = m_apagado;

				osMessagePut(colaCARGA, (uint32_t)updateCarga, osWaitForever);
			}

		}//termine de pasarle los datos

	}
  /* USER CODE END cb_control_curva */
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

#ifdef  USE_FULL_ASSERT
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
