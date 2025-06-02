#ifndef SERVO_CONTROL_C
#define SERVO_CONTROL_C

#include "servo_control.h"
#include "robot_config.h"
#include "main.h"
#include <math.h>

void Servo_Init(Servo_HandleTypeDef* servo) {
    HAL_TIM_PWM_Start(servo->timer, servo->channel);
    SetSteeringAngle(servo, CENTER_STEERING_ANGLE);  // Center steering
}

void SetSteeringAngle(Servo_HandleTypeDef* servo, float angle) {
    // Constrain input angle
    angle = fmaxf(fminf(angle, SERVO_MAX_ANGLE), SERVO_MIN_ANGLE);

    // Map angle to PWM pulse width
    float pulse_width = SERVO_PWM_MIN + 
                       ((angle / 180.0f) * 
                       (SERVO_PWM_MAX - SERVO_PWM_MIN));
                       
    
    // Set PWM compare value
    switch(servo->channel) {
        case TIM_CHANNEL_3:
            servo->timer->Instance->CCR3 = (uint32_t)pulse_width;
            break;
        // Add other channels as needed
    }
    servo->current_angle = angle;
}


#endif