#ifndef MOTION_CONTROL_C
#define MOTION_CONTROL_C

#include "motion_control.h"
#include "robot_config.h"
#include <math.h>
#include "main.h"

void InitMotionControl(MotionState* state) {
    state->current_speed = 0.0f;
    state->target_speed = 0.0f;
    state->current_acceleration = 0.0f;
    state->current_steering = 0.0f;
    state->target_steering = 0.0f;
    state->last_update = HAL_GetTick();
}

void UpdateMotionControl(MotionState* state, float dt) {
    // Handle zero dt
    if(dt <= 0) return;

    // 1. Speed control with jerk limitation
    float req_acceleration = (state->target_speed - state->current_speed) / dt;
    req_acceleration = fmaxf(fminf(req_acceleration, MAX_LINEAR_ACCEL), -MAX_LINEAR_ACCEL);
    
    float jerk = (req_acceleration - state->current_acceleration) / dt;
    if(fabsf(jerk) > MAX_LINEAR_JERK) {
        jerk = copysignf(MAX_LINEAR_JERK, jerk);
        req_acceleration = state->current_acceleration + jerk * dt;
    }

    state->current_acceleration = req_acceleration;
    state->current_speed += state->current_acceleration * dt;
    state->current_speed = fmaxf(fminf(state->current_speed, MAX_LINEAR_SPEED), -MAX_LINEAR_SPEED);

    // 2. Steering rate limitation
    float steering_error = state->target_steering - state->current_steering;
    if (fabsf(steering_error) > STEERING_DEADBAND){
        float max_steering_change = STEERING_RATE * dt;
        steering_error = fmaxf(fminf(steering_error, max_steering_change), -max_steering_change);
        state->current_steering += steering_error;
    }
}

#endif