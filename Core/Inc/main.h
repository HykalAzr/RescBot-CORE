/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
typedef struct {
    float kp;  // Proportional gain
    float ki;  // Integral gain
    float kd;  // Derivative gain
    float prevError;  // Previous error
    float integral;  // Integral term
} PIDController;

typedef struct{
	unsigned mic_pin1	:1;
	unsigned mic_pin2	:1;
	unsigned mic_pin3	:1;
	unsigned cam_pin1	:1;
}ai_module;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
enum{
	MAIN, SEARCH, ROTATE, RETURN, FINISH
};

#define setOnBoardLED		(HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET))
#define checkStartButton 	((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) == GPIO_PIN_RESET)
#define checkHomeButton 	(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET)

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BlueButton_Pin GPIO_PIN_13
#define BlueButton_GPIO_Port GPIOC
#define HomeButton_Pin GPIO_PIN_0
#define HomeButton_GPIO_Port GPIOC
#define StartButton_Pin GPIO_PIN_1
#define StartButton_GPIO_Port GPIOC
#define Bat_In_Pin GPIO_PIN_0
#define Bat_In_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_5
#define LED_GREEN_GPIO_Port GPIOA
#define M2A_Pin GPIO_PIN_0
#define M2A_GPIO_Port GPIOB
#define M2B_Pin GPIO_PIN_1
#define M2B_GPIO_Port GPIOB
#define LCD_SCL_Pin GPIO_PIN_10
#define LCD_SCL_GPIO_Port GPIOB
#define LCD_SDA_Pin GPIO_PIN_11
#define LCD_SDA_GPIO_Port GPIOB
#define LoRA_NSS_Pin GPIO_PIN_12
#define LoRA_NSS_GPIO_Port GPIOB
#define LoRA_SCK_Pin GPIO_PIN_13
#define LoRA_SCK_GPIO_Port GPIOB
#define LoRA_MISO_Pin GPIO_PIN_14
#define LoRA_MISO_GPIO_Port GPIOB
#define LoRA_MOSI_Pin GPIO_PIN_15
#define LoRA_MOSI_GPIO_Port GPIOB
#define AI_Trigger_Pin GPIO_PIN_6
#define AI_Trigger_GPIO_Port GPIOC
#define Voice_Dir_1_Pin GPIO_PIN_7
#define Voice_Dir_1_GPIO_Port GPIOC
#define Voice_Dir_2_Pin GPIO_PIN_8
#define Voice_Dir_2_GPIO_Port GPIOC
#define Voice_Dir_3_Pin GPIO_PIN_9
#define Voice_Dir_3_GPIO_Port GPIOC
#define GPS_TX_Pin GPIO_PIN_9
#define GPS_TX_GPIO_Port GPIOA
#define GPS_RX_Pin GPIO_PIN_10
#define GPS_RX_GPIO_Port GPIOA
#define LoRA_RST_Pin GPIO_PIN_11
#define LoRA_RST_GPIO_Port GPIOA
#define LoRA_IRQ_Pin GPIO_PIN_12
#define LoRA_IRQ_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define M1A_Pin GPIO_PIN_6
#define M1A_GPIO_Port GPIOB
#define M1B_Pin GPIO_PIN_7
#define M1B_GPIO_Port GPIOB
#define IMU_SCL_Pin GPIO_PIN_8
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_9
#define IMU_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define DIO0_Pin GPIO_PIN_12
#define DIO0_GPIO_Port GPIOA
#define DIO0_EXTI_IRQn EXTI15_10_IRQn
#define RESET_Pin GPIO_PIN_11
#define RESET_GPIO_Port GPIOA
#define NSS_Pin GPIO_PIN_12
#define NSS_GPIO_Port GPIOB

#define mic_detected	ai_mod.mic_pin3 == 1 || ai_mod.mic_pin2 == 1 || ai_mod.mic_pin1 == 1
#define mic_read_000	ai_mod.mic_pin3 == 0 && ai_mod.mic_pin2 == 0 && ai_mod.mic_pin1 == 0
#define mic_read_001	ai_mod.mic_pin3 == 0 && ai_mod.mic_pin2 == 0 && ai_mod.mic_pin1 == 1
#define mic_read_010	ai_mod.mic_pin3 == 0 && ai_mod.mic_pin2 == 1 && ai_mod.mic_pin1 == 0
#define mic_read_011	ai_mod.mic_pin3 == 0 && ai_mod.mic_pin2 == 1 && ai_mod.mic_pin1 == 1
#define mic_read_100	ai_mod.mic_pin3 == 1 && ai_mod.mic_pin2 == 0 && ai_mod.mic_pin1 == 0
#define mic_read_101	ai_mod.mic_pin3 == 1 && ai_mod.mic_pin2 == 0 && ai_mod.mic_pin1 == 1
#define mic_read_110	ai_mod.mic_pin3 == 1 && ai_mod.mic_pin2 == 1 && ai_mod.mic_pin1 == 0

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
