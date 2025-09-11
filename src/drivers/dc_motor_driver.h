#ifndef DC_MOTOR_DRIVER_H
#define DC_MOTOR_DRIVER_H

#include <stdint.h>
#include "../common/joints.h"

#if defined(ROBOTIC_ARM)
#define NUM_DC_MOTOR_JOINTS 5
#elif defined(ARM_SLEEVE)
#define NUM_DC_MOTOR_JOINTS 2
#else
#error "Missing mode define for NUM_DC_MOTOR_JOINTS"
#endif

void smart_motor_init(void);
void smart_motor_set_angle(joint_e joint, uint16_t angle);
void smart_motor_set_pid(joint_e joint, float Kp, float Ki, float Kd, float Ts);
void smart_motor_update(void);

#endif