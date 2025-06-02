#include "motor_control.h"
#include "main.h"
#include <stdbool.h>    
#include "robot_config.h"

void Motor_Init(Motor_HandleTypeDef* motor) {

switch (motor->pwm_channel){
    case TIM_CHANNEL_2:
        HAL_TIM_PWM_Stop(motor->timer, motor->pwm_channel);
        motor->timer->Instance->CCR2 = 0;
        break;
    case TIM_CHANNEL_3:
        HAL_TIMEx_PWMN_Stop(motor->timer, motor->pwm_channel);
        motor->timer->Instance->CCR3 = 0;
}



    switch(motor->pwm_channel){
        case TIM_CHANNEL_2:
            HAL_TIM_PWM_Start(motor->timer, motor->pwm_channel);
            motor->timer->Instance->CCER &= ~(TIM_CCER_CC1E << (4*(motor->pwm_channel-1)));
            break;
        case TIM_CHANNEL_3:
            HAL_TIMEx_PWMN_Start(motor->timer, motor->pwm_channel);
            motor->timer->Instance->CCER &= ~(TIM_CCER_CC1E << (4*(motor->pwm_channel-1)));
            break;
    }
}

void SetMotorDirection(Motor_HandleTypeDef* motor, bool forward) {
    HAL_GPIO_WritePin(motor->dir_gpio_port, motor->dir_gpio_pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void SetMotorSpeed(Motor_HandleTypeDef* motor, uint16_t speed) {
    
    speed = (speed > 100) ? 100 : speed;
    uint32_t arr = motor->timer->Instance->ARR;
    switch(motor->pwm_channel) {
        case TIM_CHANNEL_2:
            motor->timer->Instance->CCR2 = (speed * (arr+1)) / 100;
            break;
        case TIM_CHANNEL_3:
            motor->timer->Instance->CCR3 = (speed * (arr+1)) / 100;
            break;
    }
}

void EnableMotorOutputs(Motor_HandleTypeDef* motor) {
    motor->timer->Instance->CCER |= (TIM_CCER_CC1E << (4 * (motor->pwm_channel - 1)));
}