/*
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 * File: motordriver.h
 * Created Date: Monday, July 28th 2024, 12:01:21 pm
 * Author: Mustafa Algan
 * Description: This file defines the MotorDriver functions.
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 */

#ifndef __MOTORDRIVER_H_
#define __MOTORDRIVER_H_

#include "stm32l4xx.h"
#include <stdbool.h>

typedef struct motorDriver_s {

    TIM_HandleTypeDef *htimEncoder;
    TIM_HandleTypeDef *htimMotorDriver;
    TIM_TypeDef *timMotorDriver;

    uint32_t channelEncoder;
    int32_t counterEncoder;
    uint8_t rpm;

    float maxVel;
    float actualVel;
    float wheelDiameter;
    float wheelDistance;
    
    bool direction;

} motorDriver_t;

void MotorDriverInit(motorDriver_t *motorDriver, TIM_TypeDef *TIM, TIM_HandleTypeDef *htim, uint8_t rpm, float wheelDiameter, float wheelDistance);

void MotorDriverVelocity(motorDriver_t *motorDriver, float linearX, float angularZ);

void EncoderInit(motorDriver_t *motorDriver, TIM_HandleTypeDef *htim, uint32_t channel);

void EncoderCallback(motorDriver_t *motorDriver);


#endif /* __MOTORDRIVER_H_ */