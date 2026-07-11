/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define PWM_MC_IN1_Pin GPIO_PIN_5
#define PWM_MC_IN1_GPIO_Port GPIOE
#define PWM_MC_IN2_Pin GPIO_PIN_6
#define PWM_MC_IN2_GPIO_Port GPIOE
#define VIN_ADC_Pin GPIO_PIN_0
#define VIN_ADC_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOB
#define PWM_MA_IN1_Pin GPIO_PIN_9
#define PWM_MA_IN1_GPIO_Port GPIOE
#define PWM_MA_IN2_Pin GPIO_PIN_11
#define PWM_MA_IN2_GPIO_Port GPIOE
#define PWM_MB_IN1_Pin GPIO_PIN_13
#define PWM_MB_IN1_GPIO_Port GPIOE
#define PWM_MB_IN2_Pin GPIO_PIN_14
#define PWM_MB_IN2_GPIO_Port GPIOE
#define PWM_MD_IN1_Pin GPIO_PIN_14
#define PWM_MD_IN1_GPIO_Port GPIOB
#define PWM_MD_IN2_Pin GPIO_PIN_15
#define PWM_MD_IN2_GPIO_Port GPIOB
#define ENC_MD_A_Pin GPIO_PIN_12
#define ENC_MD_A_GPIO_Port GPIOD
#define ENC_MD_B_Pin GPIO_PIN_13
#define ENC_MD_B_GPIO_Port GPIOD
#define ENC_MA_A_Pin GPIO_PIN_15
#define ENC_MA_A_GPIO_Port GPIOA
#define UART4_TX_TTL_Pin GPIO_PIN_10
#define UART4_TX_TTL_GPIO_Port GPIOC
#define UART4_RX_TTL_Pin GPIO_PIN_11
#define UART4_RX_TTL_GPIO_Port GPIOC
#define USART2_TX_ROS_Pin GPIO_PIN_5
#define USART2_TX_ROS_GPIO_Port GPIOD
#define USART2_RX_ROS_Pin GPIO_PIN_6
#define USART2_RX_ROS_GPIO_Port GPIOD
#define IMU_INT_Pin GPIO_PIN_7
#define IMU_INT_GPIO_Port GPIOD
#define IMU_INT_EXTI_IRQn EXTI9_5_IRQn
#define ENC_MA_B_Pin GPIO_PIN_3
#define ENC_MA_B_GPIO_Port GPIOB
#define ENC_MB_A_Pin GPIO_PIN_4
#define ENC_MB_A_GPIO_Port GPIOB
#define ENC_MB_B_Pin GPIO_PIN_5
#define ENC_MB_B_GPIO_Port GPIOB
#define IMU_I2C1_SCL_Pin GPIO_PIN_6
#define IMU_I2C1_SCL_GPIO_Port GPIOB
#define IMU_I2C1_SDA_Pin GPIO_PIN_7
#define IMU_I2C1_SDA_GPIO_Port GPIOB
#define BEEP_Pin GPIO_PIN_0
#define BEEP_GPIO_Port GPIOE
#define KEY_Pin GPIO_PIN_1
#define KEY_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
