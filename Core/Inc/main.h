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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
#include "motor_control.h"
#include "servo_control.h"
#include <stdbool.h>
#include "hcsr04.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern struct encoder_s encoder_1;
extern struct encoder_s encoder_2;

extern struct hcsr04_s ultra_1;
extern struct hcsr04_s ultra_2;
extern struct hcsr04_s ultra_3;
extern struct hcsr04_s ultra_4;

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define MD_ENCB_Pin GPIO_PIN_0
#define MD_ENCB_GPIO_Port GPIOC
#define MD_ENCA_Pin GPIO_PIN_1
#define MD_ENCA_GPIO_Port GPIOC
#define MD_ENCA_EXTI_IRQn EXTI1_IRQn
#define MD_IPROPI_Pin GPIO_PIN_2
#define MD_IPROPI_GPIO_Port GPIOC
#define MD_DIR_Pin GPIO_PIN_3
#define MD_DIR_GPIO_Port GPIOC
#define Ultra_4_Echo_Pin GPIO_PIN_0
#define Ultra_4_Echo_GPIO_Port GPIOA
#define Ultra_1_Trig_Pin GPIO_PIN_1
#define Ultra_1_Trig_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define SMPS_EN_Pin GPIO_PIN_4
#define SMPS_EN_GPIO_Port GPIOA
#define SMPS_V1_Pin GPIO_PIN_5
#define SMPS_V1_GPIO_Port GPIOA
#define SMPS_PG_Pin GPIO_PIN_6
#define SMPS_PG_GPIO_Port GPIOA
#define SMPS_SW_Pin GPIO_PIN_7
#define SMPS_SW_GPIO_Port GPIOA
#define MA_IPROPI_Pin GPIO_PIN_4
#define MA_IPROPI_GPIO_Port GPIOC
#define BAT_Voltage_Pin GPIO_PIN_0
#define BAT_Voltage_GPIO_Port GPIOB
#define Ultra_3_Echo_Pin GPIO_PIN_1
#define Ultra_3_Echo_GPIO_Port GPIOB
#define Ultra_3_Trig_Pin GPIO_PIN_2
#define Ultra_3_Trig_GPIO_Port GPIOB
#define Ultra_2_Echo_Pin GPIO_PIN_10
#define Ultra_2_Echo_GPIO_Port GPIOB
#define Ultra_1_Echo_Pin GPIO_PIN_11
#define Ultra_1_Echo_GPIO_Port GPIOB
#define NSLEEP_Pin GPIO_PIN_13
#define NSLEEP_GPIO_Port GPIOB
#define NFAULT_Pin GPIO_PIN_14
#define NFAULT_GPIO_Port GPIOB
#define MA_PWM_Pin GPIO_PIN_15
#define MA_PWM_GPIO_Port GPIOB
#define MA_ENCB_Pin GPIO_PIN_7
#define MA_ENCB_GPIO_Port GPIOC
#define Servo_PWM_Pin GPIO_PIN_8
#define Servo_PWM_GPIO_Port GPIOC
#define MA_DIR_Pin GPIO_PIN_8
#define MA_DIR_GPIO_Port GPIOA
#define Ultra_2_Trig_Pin GPIO_PIN_15
#define Ultra_2_Trig_GPIO_Port GPIOA
#define Ultra_4_Trig_Pin GPIO_PIN_12
#define Ultra_4_Trig_GPIO_Port GPIOC
#define MD_PWM_Pin GPIO_PIN_3
#define MD_PWM_GPIO_Port GPIOB
#define LED_Strip_Data_1_Pin GPIO_PIN_4
#define LED_Strip_Data_1_GPIO_Port GPIOB
#define LED_Strip_Data_2_Pin GPIO_PIN_5
#define LED_Strip_Data_2_GPIO_Port GPIOB
#define MA_ENCA_Pin GPIO_PIN_6
#define MA_ENCA_GPIO_Port GPIOB
#define MA_ENCA_EXTI_IRQn EXTI9_5_IRQn
#define USER_BTN_Pin GPIO_PIN_8
#define USER_BTN_GPIO_Port GPIOB
#define ONOFF_LED_Strip_Pin GPIO_PIN_9
#define ONOFF_LED_Strip_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define LOGGER_NAME "MOBI"
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
