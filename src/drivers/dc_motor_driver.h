#ifndef DC_MOTOR_DRIVER_H
#define DC_MOTOR_DRIVER_H

#include <stdint.h>
#include "../common/joints.h"

#define NUM_DC_MOTOR_JOINTS 5

typedef enum {
    PWM_L_WRIST_NAE_NAE = 0,
    PWM_L_ELBOW,
    PWM_L_BICEP,
    PWM_L_SHOULDER_FRONT_RAISE,
    PWM_L_SHOULDER_LAT_RAISE,

    PWM_R_WRIST_NAE_NAE,
    PWM_R_ELBOW,
    PWM_R_BICEP,
    PWM_R_SHOULDER_FRONT_RAISE,
    PWM_R_SHOULDER_LAT_RAISE,

    PWM_CHANNEL_COUNT
} pwm_channel_e;

void smart_motor_init(void);
void smart_motor_set_angle(joint_e joint, uint16_t angle);
void smart_motor_set_pid(joint_e joint, float Kp, float Ki, float Kd, float Ts);
void smart_motor_update(void);

#endif