#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// Kinematic Parameters
#define WHEELBASE              0.4f  // Distance between axles (meters)
#define TRACK_WIDTH            0.50f  // Distance between wheels (meters)

// Motion Constraints
#define MAX_LINEAR_SPEED       1.0f   // m/s
#define CENTER_STEERING_ANGLE  90.0f   // 90° = center position
#define MAX_STEERING_ANGLE     30.0f  // degrees
#define SERVO_MIN_ANGLE        (CENTER_STEERING_ANGLE - MAX_STEERING_ANGLE)  // 60°
#define SERVO_MAX_ANGLE        (CENTER_STEERING_ANGLE + MAX_STEERING_ANGLE)  // 120°
#define MAX_LINEAR_ACCEL       2.0f   // m/s²
#define MAX_LINEAR_JERK        5.0f   // m/s³ 
#define STEERING_RATE      90.0f  // deg/s
#define STEERING_DEADBAND      2.0f

// PWM Configuration
#define SERVO_PWM_MIN          500     // 0.5ms pulse
#define SERVO_PWM_MAX          2500    // 2.5ms pulse

// Conversion factors
#define DEG_TO_RAD(angle)      ((angle) * 0.0174532925f)
#define RAD_TO_DEG(radians)    ((radians) * 57.2957795f)

#endif