/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stdbool.h"
#include "LoRa.h"
#include "liquidcrystal_i2c.h"
#include "ST47_Neo6M.h"
#include "mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Segger Sysview
#define DWT_CTRL (*(volatile uint32_t*)0xE0001000)

// CAN settings
#define CAN_TX_PIN GPIO_PIN_12
#define CAN_RX_PIN GPIO_PIN_11
#define CAN_FILTER_ID 0x123 // Example CAN filter ID
#define CAN_TX_QUEUE_LEN 10

// IMU device address
#define IMU_ADDRESS 0x68 // Example address, adjust as per your IMU

// IMU register addresses
#define IMU_WHO_AM_I_REG 0x75 // WHO_AM_I register to check device ID

// IMU register addresses for accelerometer and gyroscope data
#define IMU_ACCEL_XOUT_H 0x3B
#define IMU_ACCEL_XOUT_L 0x3C
#define IMU_ACCEL_YOUT_H 0x3D
#define IMU_ACCEL_YOUT_L 0x3E
#define IMU_ACCEL_ZOUT_H 0x3F
#define IMU_ACCEL_ZOUT_L 0x40
#define IMU_GYRO_XOUT_H 0x43
#define IMU_GYRO_XOUT_L 0x44
#define IMU_GYRO_YOUT_H 0x45
#define IMU_GYRO_YOUT_L 0x46
#define IMU_GYRO_ZOUT_H 0x47
#define IMU_GYRO_ZOUT_L 0x48

// Motor driver pins
#define MOTOR1_DIR1_PIN GPIO_PIN_12 // direction pin 1 for motor 1
#define MOTOR1_DIR2_PIN GPIO_PIN_13 // direction pin 2 for motor 1

#define MOTOR2_DIR1_PIN GPIO_PIN_14 // direction pin 1 for motor 2
#define MOTOR2_DIR2_PIN GPIO_PIN_15 // direction pin 2 for motor 2

// Motor parameter
#define MOTOR_MAX_SPEED 3500 // Max DAC val is 12-bit (4096)

// DAC settings
#define DAC_RESOLUTION 4095 // 12-bit DAC resolution

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* Definitions for runTask */
osThreadId_t runTaskHandle;
const osThreadAttr_t runTask_attributes = {
  .name = "runTask",
  .stack_size = 350 * 4,
  .priority = (osPriority_t) osPriorityHigh3,
};
/* Definitions for standbyTask */
osThreadId_t standbyTaskHandle;
const osThreadAttr_t standbyTask_attributes = {
  .name = "standbyTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for commTask */
osThreadId_t commTaskHandle;
const osThreadAttr_t commTask_attributes = {
  .name = "commTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for sensorTask */
osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
  .name = "sensorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* USER CODE BEGIN PV */

// Motor Speed
uint16_t rightMotorSpeed = 0;
uint16_t leftMotorSpeed = 0;

// GPS data
double latitude, longitude;
double home_lat, home_long;
double targ_lat, targ_long;

// IMU data
uint8_t imu_who_am_i;
//int16_t accel_data[3];
//int16_t prev_accel_data[3];
int16_t gyro_data[3];
//float roll = 0.0f;
//float pitch = 0.0f;
float curr_heading = 0.0f;
float target_heading = 0.0f;
float heading_err = 0.0f;
float prev_heading_err = 0.0f;

// LCD data
char* lcd_status = "IDLE";

// run var
uint8_t curr_run_mode = MAIN;
uint8_t prev_run_mode = MAIN;
uint8_t home_coordinate[25];
uint8_t end_coordinate[25];

float battery_val = 0;

LoRa myLoRa;
uint8_t read_data[100];
uint8_t send_data[100];

MPU6050_t MPU6050;

ai_module ai_mod;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM4_Init(void);
void StartRunTask(void *argument);
void StartStandbyTask(void *argument);
void StartCommTask(void *argument);
void StartSensorTask(void *argument);

/* USER CODE BEGIN PFP */
void PID_init(PIDController *pid, float kp, float ki, float kd);
float PID_update(PIDController *pid, float setpoint, float current_value, float dt);

void LCD_Send_Command(uint8_t cmd);
void LCD_Send_Data(uint8_t data);
void LCD_Init(void);
void LCD_Print(const char *str);

void Motor_Set(int8_t speedA, int8_t speedB);

extern  void SEGGER_UART_init(uint32_t);

void vTaskDelete();
void vTaskSuspend();
void vTaskResume();

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
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  NVIC_SetPriorityGrouping( 0 );
  //Enable the CYCCNT counter
  DWT_CTRL |= ( 1 << 0 );
  //SEGGER_UART_init(500000);
  SEGGER_SYSVIEW_Conf();
  SEGGER_SYSVIEW_Start();

  // MODULE SETTINGS ----------------------------------------------
  myLoRa = newLoRa();

  myLoRa.hSPIx                 		= &hspi2;
  myLoRa.CS_port               		= NSS_GPIO_Port;
  myLoRa.CS_pin                		= NSS_Pin;
  myLoRa.reset_port            		= RESET_GPIO_Port;
  myLoRa.reset_pin             		= RESET_Pin;
  myLoRa.DIO0_port			   		= DIO0_GPIO_Port;
  myLoRa.DIO0_pin			   		= DIO0_Pin;

  myLoRa.frequency             		= 433;					// default = 433 MHz
  myLoRa.spredingFactor        		= SF_7;					// default = SF_7
  myLoRa.bandWidth			       	= BW_125KHz;			// default = BW_125KHz
  myLoRa.crcRate				    = CR_4_5;				// default = CR_4_5
  myLoRa.power					    = POWER_20db;			// default = 20db
  myLoRa.overCurrentProtection 		= 100; 					// default = 100 mA
  myLoRa.preamble				    = 10;		  			// default = 8;

  HD44780_Init(2);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of runTask */
  runTaskHandle = osThreadNew(StartRunTask, NULL, &runTask_attributes);

  /* creation of standbyTask */
  standbyTaskHandle = osThreadNew(StartStandbyTask, NULL, &standbyTask_attributes);

  /* creation of commTask */
  commTaskHandle = osThreadNew(StartCommTask, NULL, &commTask_attributes);

  /* creation of sensorTask */
  sensorTaskHandle = osThreadNew(StartSensorTask, NULL, &sensorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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
  hi2c1.Init.ClockSpeed = 50000;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_GREEN_Pin|LoRA_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BlueButton_Pin HomeButton_Pin StartButton_Pin */
  GPIO_InitStruct.Pin = BlueButton_Pin|HomeButton_Pin|StartButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AI_Trigger_Pin Voice_Dir_1_Pin Voice_Dir_2_Pin Voice_Dir_3_Pin */
  GPIO_InitStruct.Pin = AI_Trigger_Pin|Voice_Dir_1_Pin|Voice_Dir_2_Pin|Voice_Dir_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LoRA_RST_Pin */
  GPIO_InitStruct.Pin = LoRA_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LoRA_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LoRA_IRQ_Pin */
  GPIO_InitStruct.Pin = LoRA_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(LoRA_IRQ_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void PID_init(PIDController *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->prevError = 0.0f;
    pid->integral = 0.0f;
}

float PID_update(PIDController *pid, float setpoint, float current_value, float dt) {
    float error = setpoint - current_value;
    pid->integral += error * dt;
    float derivative = (error - pid->prevError) / dt;
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    pid->prevError = error;
    return output;
}

void Motor_Set(int8_t speedA, int8_t speedB) {
	/* user defined speed should be between (-)100 to 100*/

	speedA = (speedA > 100) ? 100 : speedA;
	speedA = (speedA < -100) ? -100 : speedA;
	speedB = (speedB > 100) ? 100 : speedB;
	speedB = (speedB < -100) ? -100 : speedB;

	bool signBitA = (speedA & 0b10000000) >> 7;
	uint32_t absValueA = (speedA > 0) ? (uint8_t)speedA : (uint8_t)(-speedA);

	bool signBitB = (speedB & 0b10000000) >> 7;
	uint32_t absValueB = (speedB > 0) ? (uint8_t)speedB : (uint8_t)(-speedB);

	if(signBitA == 0)
	{	// Right Motor Forward
		TIM4->CCR1 = 0;
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		TIM4->CCR2 = absValueA;
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	}
	else if(signBitA == 1)
	{	// Right Motor Reverse
		TIM4->CCR1 = absValueA;
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		TIM4->CCR2 = 0;
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	}

	if(signBitB == 0)
	{	// Left Motor Forward
		TIM3->CCR3 = absValueB;
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
		TIM3->CCR4 = 0;
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	}
	else if(signBitB == 1)
	{	// Left Motor Reverse
		TIM3->CCR3 = 0;
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
		TIM3->CCR4 = absValueB;
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	}

	if (speedA == 0 && speedB == 0)
	{
	  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
	}

//	char textToWrite[ 30 ];
//	sprintf(textToWrite,"A:%3.lu B:%3.lu", absValueA, absValueB);
//	SEGGER_SYSVIEW_PrintfTarget(textToWrite);
//	sprintf(textToWrite,"SA:%d SB:%d", signBitA, signBitB);
//	SEGGER_SYSVIEW_PrintfTarget(textToWrite);
}

uint16_t read_adc_value(void)
{
  uint16_t adc_value = 0;
  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
  {
    adc_value = HAL_ADC_GetValue(&hadc1);
  }
  HAL_ADC_Stop(&hadc1);
  return adc_value;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	GPS_Callback();
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartRunTask */
/**
  * @brief  Function implementing the runTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartRunTask */
void StartRunTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  int8_t speedA = 0, speedB = 0, imu_reset_counter = 0;
  char sv_runtask_msg[40];
  double dt_runtask = 0;
  uint32_t timer_runtask;
  vTaskSuspend(NULL); // Suspend the task
  /* Infinite loop */
  for(;;)
  {
	setOnBoardLED;

	if((heading_err >= 2 || heading_err <= -2) && curr_run_mode != ROTATE){
		prev_run_mode = curr_run_mode;
		curr_run_mode = ROTATE;
	}else if ((heading_err <= 2 && heading_err >= -2) && curr_run_mode == ROTATE){
		speedA = 0.0;
		speedB = 0.0;
		curr_run_mode = prev_run_mode;
	}

	switch(curr_run_mode){
		case MAIN:
			lcd_status = "MAIN";
			target_heading = 0.0f;
			dt_runtask = 0;
			speedA = 70 + 10 * heading_err;
			speedB = 70 + 10 * -heading_err;

			if(mic_detected){				// Check sound pass threshold
				Motor_Set(0, 0);
				curr_run_mode = SEARCH;
				timer_runtask = HAL_GetTick();
			}
			break;
		case SEARCH:
			lcd_status = "SEARCH";
			speedA = 0.0;
			speedB = 0.0;
			if(mic_read_001)				// Check sound source direction
			{
				target_heading = -42.0f;	// 42 IMU val = 180 degree real-world value
			}else if(mic_read_010)
			{
				target_heading = -28.0f;
			}else if(mic_read_011)
			{
				target_heading = -14.0f;
			}else if(mic_read_100)
			{
				target_heading = 0.0f;
			}else if(mic_read_101)
			{
				target_heading = 14.0f;
			}else if(mic_read_110)
			{
				target_heading = 28.0f;
			}
			if(ai_mod.cam_pin1 == 1)		// Check if human detected
			{
				target_heading = 0.0f;
				curr_run_mode = MAIN;
			}
			dt_runtask = (double)(HAL_GetTick() - timer_runtask) / 1000;
			if(dt_runtask > 8){			// Check if function reach timeout
				curr_run_mode = MAIN;
				speedA = 0.0;
				speedB = 0.0;
			}
			break;
		case ROTATE:
			lcd_status = "ROTATE";
			timer_runtask = HAL_GetTick();

			if(heading_err > 5)
			{
				speedA = 100;
				speedB = -100;
			}
			else if(heading_err < -5)
			{
				speedA = -100;
				speedB = 100;
			}
			else
			{
				speedA = heading_err * 20;
				speedB = heading_err * -20;
			}

			if(speedA != 0 && speedB != 0){
				if(imu_reset_counter > 5){
					MX_I2C1_Init();
					imu_reset_counter = 0;
				}
				else
				{
					if(prev_heading_err != heading_err){
						imu_reset_counter = 0;
					}else{
						imu_reset_counter++;
					}

				}
			}
			prev_heading_err = heading_err;
			break;
		case RETURN:
			lcd_status = "RETURN";
			target_heading = 42.0f;
			break;
		case FINISH:
			speedA = 0.0;
			speedB = 0.0;
			vTaskResume(standbyTaskHandle);
			vTaskSuspend(NULL);
			break;
	}

	Motor_Set(speedA, speedB);

	if(checkStartButton)
	{
		while(checkStartButton);
		vTaskSuspend(sensorTaskHandle);
		vTaskResume(standbyTaskHandle);
		vTaskSuspend(NULL);
	}

	snprintf(sv_runtask_msg, sizeof(sv_runtask_msg), "dt: %2f | MOD: %1.d | H_Err: %2.2f", dt_runtask, curr_run_mode + 1, heading_err);
	SEGGER_SYSVIEW_PrintfTarget(sv_runtask_msg);

	snprintf(sv_runtask_msg, sizeof(sv_runtask_msg), "SPA: %3.1d | SPB: %3.1d | res_c: %3.0d", speedA, speedB, imu_reset_counter);
	SEGGER_SYSVIEW_PrintfTarget(sv_runtask_msg);

    osDelay(80);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartStandbyTask */
/**
* @brief Function implementing the standbyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStandbyTask */
void StartStandbyTask(void *argument)
{
  /* USER CODE BEGIN StartStandbyTask */
  char sv_standbytask_msg[40];
  char sv_standbytask_msg_len = sizeof(sv_standbytask_msg);
  bool LD4_toggle = 0;
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, LD4_toggle);
	LD4_toggle = !LD4_toggle;
	// Stop motors
	Motor_Set(0, 0);

	// Show status
	lcd_status = "IDLE";
	MPU6050.HeadingZ = 0;
	target_heading = 0.0f;
	heading_err = 0.0f;
	curr_run_mode = MAIN;
	prev_run_mode = MAIN;

	if(checkStartButton)
	{
		while(checkStartButton);
		// Reset IMU
		MPU6050.HeadingZ = 0;
		vTaskResume(runTaskHandle);
		vTaskResume(sensorTaskHandle);
		vTaskSuspend(NULL);
	}

	if(checkHomeButton)
	{
		home_lat = latitude;
		home_long = longitude;
		targ_lat = home_lat + 100;
		targ_long = home_long + 100;
	}

	snprintf(sv_standbytask_msg, sv_standbytask_msg_len, "Home: %f, %f", home_lat, home_long);
	SEGGER_SYSVIEW_PrintfTarget(sv_standbytask_msg);
	snprintf(sv_standbytask_msg, sv_standbytask_msg_len, "Targ: %f, %f", targ_lat, targ_long);
	SEGGER_SYSVIEW_PrintfTarget(sv_standbytask_msg);

	osDelay(200);
  }
  /* USER CODE END StartStandbyTask */
}

/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Function implementing the commTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommTask */
void StartCommTask(void *argument)
{
  /* USER CODE BEGIN StartCommTask */
  LoRa_reset(&myLoRa);
  LoRa_init(&myLoRa);
  LoRa_startReceiving(&myLoRa);
  char sv_commtask_msg[50];
  char sv_commtask_msg_len = sizeof(sv_commtask_msg);
//  char lora_tx_buffer[100];
//  char lora_tx_buffer_len = sizeof(lora_tx_buffer);
//  char lcd_buffer[15];
//  char lcd_buffer_len = sizeof(lcd_buffer);
  /* Infinite loop */
  for(;;)
  {
	// Battery Voltage
	battery_val = read_adc_value() * 1.15 / 4096.0 * 16.5;

//	snprintf(lcd_buffer, lcd_buffer_len, "S: %s", lcd_status);
//    HD44780_Clear();
//    HD44780_SetCursor(0,0);
//    HD44780_PrintStr(lcd_buffer);
//    snprintf(lcd_buffer, lcd_buffer_len, "B:%2.1f H:%2.0f", battery_val, heading_err);
//    HD44780_SetCursor(0,1);
//    HD44780_PrintStr(lcd_buffer);

	snprintf(sv_commtask_msg, sv_commtask_msg_len, "MODE: %s\n | BATT: %2.2f\n", lcd_status, battery_val);
	SEGGER_SYSVIEW_PrintfTarget(sv_commtask_msg);

//	osDelay(200);

	// SENDING DATA - - - - - - - - - - - - - - - - - - - - - - - - -
	// Package data to be trasmitted
//	snprintf(lora_tx_buffer, lora_tx_buffer_len, "SHome Coordinate: %s | End Coordinate: %s", (char*)home_coordinate, (char*)end_coordinate);
//	memcpy(send_data, lora_tx_buffer, lora_tx_buffer_len);
//	send_data[0] = 0x3B; // MY ADDRESS
//	LoRa_transmit(&myLoRa, send_data, 100, 500);

//	snprintf(sv_commtask_msg, sv_commtask_msg_len, "TxLORA: %s", (char*)send_data);
//	SEGGER_SYSVIEW_PrintfTarget(sv_commtask_msg);
//
//	osDelay(200);
//
//	// RECEIVING DATA - - - - - - - - - - - - - - - - - - - - - - - -
//	LoRa_receive(&myLoRa, read_data, 100);
//
//	snprintf(sv_commtask_msg, sv_commtask_msg_len, "RxLORA: %s", (char*)read_data);
//	SEGGER_SYSVIEW_PrintfTarget(sv_commtask_msg);

    osDelay(500);
  }
  /* USER CODE END StartCommTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the sensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
  char sv_sensortask_msg[50];
  char sv_sensortask_msg_len = sizeof(sv_sensortask_msg);
  GPS_Init();
  MPU6050_Init(&hi2c1);
  vTaskSuspend(NULL); // Suspend the task
  /* Infinite loop */
  for(;;)
  {
	// Read IMU (I2C)
	MPU6050_Read_Gyro(&hi2c1, &MPU6050);

	curr_heading = MPU6050.HeadingZ;
	heading_err = curr_heading - target_heading;

	// Read GPS (UART)
//	latitude = GPS_GetLatitude();
//	longitude = GPS_GetLongtitude();

	// Process received data here
//	snprintf(sv_sensortask_msg, sv_sensortask_msg_len, "LAT: %f | LONG: %f | IMU: %6.2f", latitude, longitude, curr_heading);
	snprintf(sv_sensortask_msg, sv_sensortask_msg_len, "IMU: %6.2f", curr_heading);
	SEGGER_SYSVIEW_PrintfTarget(sv_sensortask_msg);

	// Read AI module
	ai_mod.cam_pin1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
	ai_mod.mic_pin1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
	ai_mod.mic_pin2 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);
	ai_mod.mic_pin3 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);

	snprintf(sv_sensortask_msg, sv_sensortask_msg_len, "C: %u | MC3: %u | MC2: %u | MC1: %u", ai_mod.cam_pin1, ai_mod.mic_pin3, ai_mod.mic_pin2, ai_mod.mic_pin1);
	SEGGER_SYSVIEW_PrintfTarget(sv_sensortask_msg);

	osDelay(100);
  }
  /* USER CODE END StartSensorTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
