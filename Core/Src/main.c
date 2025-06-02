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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "encoder.h"
#include "motor_control.h"
#include "servo_control.h"
#include "motion_control.h"
#include "robot_config.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
encoder_t encoder_1;
encoder_t encoder_2;

hcsr04_t ultra_1;
hcsr04_t ultra_2;
hcsr04_t ultra_3;
hcsr04_t ultra_4;

Motor_HandleTypeDef motor_left, motor_right;
MotionState motion_state;
Servo_HandleTypeDef steering_servo;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(NSLEEP_GPIO_Port, NSLEEP_Pin, GPIO_PIN_SET);

  motor_left.timer = &htim1;
  motor_left.pwm_channel = TIM_CHANNEL_3;
  motor_left.dir_gpio_port = MA_DIR_GPIO_Port;
  motor_left.dir_gpio_pin = MA_DIR_Pin;

  motor_right.timer = &htim2;
  motor_right.pwm_channel = TIM_CHANNEL_2;
  motor_right.dir_gpio_port = MD_DIR_GPIO_Port;
  motor_right.dir_gpio_pin = MD_DIR_Pin;

  Motor_Init(&motor_left);
  Motor_Init(&motor_right);

  // Servo initialization
  steering_servo.timer = &htim3;  // Example: TIM3 for servo
  steering_servo.channel = TIM_CHANNEL_3;
  Servo_Init(&steering_servo);

  // Motion Control initialization
  InitMotionControl(&motion_state);

  // Initialize Encoders
  encoder_init(&encoder_1, MA_ENCA_GPIO_Port, MA_ENCA_Pin, MA_ENCB_GPIO_Port, MA_ENCB_Pin);
  encoder_init(&encoder_2, MD_ENCA_GPIO_Port, MD_ENCA_Pin, MD_ENCB_GPIO_Port, MD_ENCB_Pin);

  hcsr04_init(&ultra_1, &htim2, TIM_CHANNEL_4, HAL_TIM_ACTIVE_CHANNEL_4, TIM_IT_CC4, Ultra_1_Trig_GPIO_Port, Ultra_1_Trig_Pin);
  hcsr04_init(&ultra_2, &htim2, TIM_CHANNEL_3, HAL_TIM_ACTIVE_CHANNEL_3, TIM_IT_CC3, Ultra_2_Echo_GPIO_Port, Ultra_2_Echo_Pin);
  hcsr04_init(&ultra_3, &htim3, TIM_CHANNEL_4, HAL_TIM_ACTIVE_CHANNEL_4, TIM_IT_CC4, Ultra_3_Trig_GPIO_Port, Ultra_3_Trig_Pin);
  hcsr04_init(&ultra_4, &htim2, TIM_CHANNEL_2, HAL_TIM_ACTIVE_CHANNEL_2, TIM_IT_CC2, Ultra_4_Trig_GPIO_Port, Ultra_4_Trig_Pin);

  HAL_GPIO_WritePin(MA_DIR_GPIO_Port, MA_DIR_Pin, GPIO_PIN_SET);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM16 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM16) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Channel == ultra_1.active_channel && htim == ultra_1.htim) { // US 1
    hcsr04_handle_period_elapsed_interrupt(&ultra_1);
  } else if (htim->Channel == ultra_2.active_channel && htim == ultra_2.htim) { // US 2
    hcsr04_handle_period_elapsed_interrupt(&ultra_2);
  } else if (htim->Channel == ultra_3.active_channel && htim == ultra_3.htim) { // US 3
    hcsr04_handle_period_elapsed_interrupt(&ultra_3);
  } else if (htim->Channel == ultra_4.active_channel && htim == ultra_4.htim) { // US 4
    hcsr04_handle_period_elapsed_interrupt(&ultra_4);
  }
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
