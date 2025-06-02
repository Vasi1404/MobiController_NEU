#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdint.h>

typedef struct {
    float current_speed;        // m/s
    float target_speed;         // m/s
    float current_acceleration; // m/s²
    float current_steering;     // degrees
    float target_steering;      // degrees
    uint32_t last_update;
} MotionState;

void InitMotionControl(MotionState* state);
void UpdateMotionControl(MotionState* state, float dt);

#endif