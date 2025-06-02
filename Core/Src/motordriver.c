/*
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 * File: motordriver.h
 * Created Date: Monday, July 28th 2024, 12:01:21 pm
 * Author: Mustafa Algan
 * Description: This file defines the MotorDriver functions.
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 */

#include "motordriver.h"
#define PI 3.14159


void MotorDriverInit(motorDriver_t *motorDriver, TIM_TypeDef *TIM, TIM_HandleTypeDef *htim, uint8_t rpm, float wheelDiameter, float wheelDistance)
{
    motorDriver->actualVel = 0.0;
    motorDriver->direction = 0;
    motorDriver->rpm = rpm;
    motorDriver->wheelDiameter = wheelDiameter;
    motorDriver->wheelDistance = wheelDistance;
    motorDriver->timMotorDriver = TIM;
    motorDriver->htimMotorDriver = htim;

    HAL_TIM_PWM_Start(motorDriver->htimMotorDriver, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(motorDriver->htimMotorDriver, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(motorDriver->htimMotorDriver, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(motorDriver->htimMotorDriver, TIM_CHANNEL_4);

    motorDriver->timMotorDriver->CCR1 = 0;
    motorDriver->timMotorDriver->CCR2 = 0;
    motorDriver->timMotorDriver->CCR3 = 0;
    motorDriver->timMotorDriver->CCR4 = 0;

    motorDriver->maxVel = PI * motorDriver->wheelDiameter * motorDriver->rpm / 60.0;

}



void MotorDriverVelocity(motorDriver_t *motorDriver, float linearX, float angularZ)
{   
    if (linearX > 200)
    {
        linearX = 200;
    }

    float vLeft = linearX - (angularZ * motorDriver->wheelDistance /2);
    float vRight = linearX + (angularZ * motorDriver->wheelDistance /2);

    float leftPWM = (vLeft / motorDriver->maxVel) * 100.0f;
    float rightPWM = (vRight / motorDriver->maxVel) * 100.0f;

    if (vLeft >= 0)
    {
        motorDriver->timMotorDriver->CCR2 = 0;
        motorDriver->timMotorDriver->CCR1 = leftPWM;
        
    }   
    else 
    {
        motorDriver->timMotorDriver->CCR2 = leftPWM*-1;
        motorDriver->timMotorDriver->CCR1 = 0;
    }

    if (vRight >= 0)
    {
        motorDriver->timMotorDriver->CCR3 = 0;
        motorDriver->timMotorDriver->CCR4 = rightPWM;        
    }   
    else 
    {
        motorDriver->timMotorDriver->CCR3 = rightPWM*-1;
        motorDriver->timMotorDriver->CCR4 = 0;
    }
    
}

void EncoderInit(motorDriver_t *motorDriver, TIM_HandleTypeDef *htim, uint32_t channel){
  motorDriver->htimEncoder = htim;
  motorDriver->channelEncoder = channel;
  motorDriver->counterEncoder = 0; 

  HAL_TIM_Encoder_Start(motorDriver->htimEncoder, motorDriver->channelEncoder);
}

void EncoderCallback(motorDriver_t *motorDriver) {
  // Get the state of the B pin
  
  motorDriver->counterEncoder = __HAL_TIM_GET_COUNTER(motorDriver->htimEncoder);
  
}