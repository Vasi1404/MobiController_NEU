#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"
#include <stdbool.h>

typedef struct {
    TIM_HandleTypeDef* timer;
    uint32_t pwm_channel;
    GPIO_TypeDef* dir_gpio_port;
    uint16_t dir_gpio_pin;
} Motor_HandleTypeDef;

void Motor_Init(Motor_HandleTypeDef* motor);
void SetMotorDirection(Motor_HandleTypeDef* motor, bool forward);
void SetMotorSpeed(Motor_HandleTypeDef* motor, uint16_t speed);
void EnableMotorOutputs(Motor_HandleTypeDef* motor);

#endif  //MOTOR_CONTROL_H