/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>


#include "usart.h"
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "usart.h"
#include <geometry_msgs/msg/twist.h>

#include <micro_ros_utilities/string_utilities.h>
#include <mobi_interfaces/msg/encoders_stamped.h>
#include <mobi_interfaces/msg/ultra_ranges.h>
#include <mobi_interfaces/srv/get_calib_status.h>
#include <mobi_interfaces/srv/get_imu_calib_data.h>
#include <mobi_interfaces/srv/get_pozyx_info.h>
#include <mobi_interfaces/srv/set_imu_calib_data.h>
#include <mobi_interfaces/srv/set_led_strip.h>
#include <mobi_interfaces/srv/set_power.h>
#include "encoder.h"
#include "utils.h"
#include "motor_control.h"
#include "servo_control.h"
#include "motion_control.h"
#include "robot_config.h"

#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */
#define RCCHECK(fn){                                                                                                                    \
  rcl_ret_t temp_rc = fn;                                                                                            \
  if ((temp_rc != RCL_RET_OK)) {                                                                                     \
    RCUTILS_LOG_DEBUG_NAMED(LOGGER_NAME, "Failed status on line %d: %d. Aborting.", __LINE__, (int)temp_rc);         \
    return;                                                                                                          \
  }                                                                                                                  \
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// Declare Publishers
rcl_publisher_t encoders_pub;
rcl_publisher_t ultra_ranges_pub;

// Declare Publisher msgs
mobi_interfaces__msg__EncodersStamped encoders_msg;
mobi_interfaces__msg__UltraRanges ultra_ranges_msg;

// Declare Subscribers
rcl_subscription_t cmd_vel_sub;

// Declare Subscriber msgs
geometry_msgs__msg__Twist cmd_vel_msg;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 3000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);

void *microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void *microros_reallocate(void *pointer, size_t size, void *state);
void *microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void *state);

void timer_100ms_callback(rcl_timer_t *timer, int64_t last_call_time);
void timer_250ms_callback(rcl_timer_t *timer, int64_t last_call_time);
void timer_1s_callback(rcl_timer_t *timer, int64_t last_call_time);

void cmd_vel_callback(const void *argument);



extern Motor_HandleTypeDef motor_left, motor_right;
extern Servo_HandleTypeDef steering_servo;
extern MotionState motion_state;
static bool motors_initialized = false;
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */



  rmw_uros_set_custom_transport(
    true,
    (void *) &huart2,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);
    
    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        printf("Error on default allocators (line %d)\n", __LINE__);
    }


  rclc_support_t support;
  //rcl_ret_t rc;

  // After transport setup, before node creation
  while (rmw_uros_ping_agent(100, 3) != RCL_RET_OK) {
    printf("Waiting for agent...\n");
    osDelay(100);  // Don't block indefinitely
  }

  hcsr04_init_range_msg(&ultra_ranges_msg.front_left, micro_ros_string_utilities_init("us_front_left"));
  hcsr04_init_range_msg(&ultra_ranges_msg.front_right, micro_ros_string_utilities_init("us_front_right"));
  hcsr04_init_range_msg(&ultra_ranges_msg.center_left, micro_ros_string_utilities_init("us_center_left"));
  hcsr04_init_range_msg(&ultra_ranges_msg.center_right, micro_ros_string_utilities_init("us_center_right"));
  hcsr04_init_range_msg(&ultra_ranges_msg.rear_left, micro_ros_string_utilities_init("us_rear_left"));
  hcsr04_init_range_msg(&ultra_ranges_msg.rear_right, micro_ros_string_utilities_init("us_rear_right"));

  encoders_msg.header.frame_id = micro_ros_string_utilities_init("encoders");


  // Initialize init_options
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

  rcl_init_options_init(&init_options, freeRTOS_allocator);
  rcl_init_options_set_domain_id(&init_options, 255);

  rclc_support_init_with_options(&support, 0, NULL, &init_options, &freeRTOS_allocator);

  // create rcl_node
  rcl_node_t my_node;
  rclc_node_init_default(&my_node, "stm32", "", &support);
  //if (rc != RCL_RET_OK) {
    //printf("Error in rclc_node_init_default\n");
    //return;
  //}

  rmw_uros_sync_session(1000);

  rclc_publisher_init_default(&encoders_pub, &my_node, ROSIDL_GET_MSG_TYPE_SUPPORT(mobi_interfaces, msg, EncodersStamped), "/encoder");

  rclc_publisher_init_default(&ultra_ranges_pub, &my_node, ROSIDL_GET_MSG_TYPE_SUPPORT(mobi_interfaces, msg, UltraRanges), "/ultra_ranges");

  // Initialize subscribers
  rclc_subscription_init_default(&cmd_vel_sub, &my_node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");


  rcl_timer_t timer_100ms;
  
  rclc_timer_init_default2(&timer_100ms, &support, RCL_MS_TO_NS(100), timer_100ms_callback, true);
  
  rcl_timer_t timer_250ms;
  rclc_timer_init_default2(&timer_250ms, &support, RCL_MS_TO_NS(250), timer_250ms_callback, true);

  rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
  

  rclc_executor_init(&executor, &support.context, 3, &freeRTOS_allocator);

  rclc_executor_add_timer(&executor, &timer_100ms);
  //if (rc != RCL_RET_OK) {
    //printf("Error in rclc_executor_add_timer.\n");
  //}

  rclc_executor_add_timer(&executor, &timer_250ms);

  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);

  rclc_executor_prepare(&executor);


  /* Infinite loop */
  for(;;)
  {

    //steering_servo.timer->Instance->CCR3 = 1000;
    //HAL_Delay(2000);
    //steering_servo.timer->Instance->CCR3 = 2000;
    //HAL_Delay(2000);


    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  // Safety: Update motion control even without new messages
    //if(HAL_GetTick() - motion_state.last_update > 100) {
      //  float dt = 0.1f; // 100ms
        //UpdateMotionControl(&motion_state, dt);
    //}
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static int32_t fake_left = 0;
static int32_t fake_right = 1000;

void timer_100ms_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;

  encoders_msg.encoders.front_left = encoder_1.counter;
  encoders_msg.encoders.front_right = encoder_2.counter;

  stamp_header(&encoders_msg.header.stamp);
  rcl_publish(&encoders_pub, &encoders_msg, NULL);
}

void cmd_vel_callback(const void *msgin) {

  if (!motors_initialized){
    EnableMotorOutputs(&motor_right);
    motors_initialized = true;
  }
  
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  
  // Convert Twist to Ackermann commands
    float linear_x = fmaxf(fminf(msg->linear.x, 1.0f), -1.0f);
    float angular_z = msg->angular.z;

    float target_angle = CENTER_STEERING_ANGLE;

    // Calculate Ackermann steering angle
    if(fabsf(linear_x) > 0.1f) {
      float direction_sign = (linear_x >= 0) ? 1.0f : -1.0f;
      float angle_rad = atan2f(WHEELBASE * angular_z , fabsf(linear_x)) * direction_sign;
      target_angle = RAD_TO_DEG(angle_rad) + CENTER_STEERING_ANGLE;
    } else if(fabsf(angular_z) > 0.01f) {
        // Handle stationary steering
        target_angle = copysignf(MAX_STEERING_ANGLE, angular_z) + CENTER_STEERING_ANGLE;
    }

    motion_state.target_steering = fmaxf(fminf(target_angle, SERVO_MAX_ANGLE), SERVO_MIN_ANGLE);
    //float angle_deg = RAD_TO_DEG(angle_rad) + CENTER_STEERING_ANGLE;

    //angle_deg = fmax(fmin(angle_deg, SERVO_MAX_ANGLE), SERVO_MIN_ANGLE);
    //if (angular_z == 0){angle_deg = CENTER_STEERING_ANGLE;}

    // Set target speed
    motion_state.target_speed = linear_x;

    // Calculate time delta
    uint32_t now = HAL_GetTick();
    float dt = (now - motion_state.last_update) / 1000.0f;
    motion_state.last_update = now;

    // Update motion control
    UpdateMotionControl(&motion_state, dt);

    // Convert to motor control (differential steering)
    float speed_percent = (motion_state.current_speed / MAX_LINEAR_SPEED) * 100.0f;
    bool direction = speed_percent >= 0;

    // Set motor speeds (same for both wheels in Ackermann steering)
    SetMotorDirection(&motor_left, direction);
    SetMotorDirection(&motor_right, direction);
    SetMotorSpeed(&motor_left, fabsf(speed_percent));
    SetMotorSpeed(&motor_right, fabsf(speed_percent));
    // Set steering angle
    SetSteeringAngle(&steering_servo, motion_state.current_steering);

}

void timer_250ms_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;

  hcsr04_measure(&ultra_1);
  ultra_ranges_msg.front_left.range = fake_left++;
  stamp_header(&ultra_ranges_msg.front_left.header.stamp);
  hcsr04_measure(&ultra_2);
  ultra_ranges_msg.front_right.range = fake_right++;
  stamp_header(&ultra_ranges_msg.front_right.header.stamp);
  hcsr04_measure(&ultra_3);
  ultra_ranges_msg.rear_left.range = fake_left++;
  stamp_header(&ultra_ranges_msg.rear_left.header.stamp);
  hcsr04_measure(&ultra_4);
  ultra_ranges_msg.rear_right.range = fake_right++;
  stamp_header(&ultra_ranges_msg.rear_right.header.stamp);

  rcl_publish(&ultra_ranges_pub, &ultra_ranges_msg, NULL);


}

/* USER CODE END Application */

