#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H


#include "main.h"

typedef struct {
    TIM_HandleTypeDef* timer;
    uint32_t channel;
    float current_angle;
} Servo_HandleTypeDef;

void Servo_Init(Servo_HandleTypeDef* servo);
void SetSteeringAngle(Servo_HandleTypeDef* servo, float angle);

#endif