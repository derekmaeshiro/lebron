#ifndef DC_MOTOR_DRIVER_H
#define DC_MOTOR_DRIVER_H

#include <stdint.h>

#define NUM_DC_MOTOR_JOINTS 5

void smart_motor_init(void);
void smart_motor_set_angle(joint_e joint, uint16_t angle);
void smart_motor_set_pid(joint_e joint, float Kp, float Ki, float Kd, float Ts);
void smart_motor_update(void);

#endif