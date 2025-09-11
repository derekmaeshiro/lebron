#include <stdint.h>
#include <stm32f4xx.h>
#include "../common/joints.h"

#ifndef PWM_H
#define PWM_H

#if defined(ROBOTIC_ARM)
typedef enum {
    PWM_L_WRIST_NAE_NAE,
    PWM_R_WRIST_NAE_NAE,
    PWM_L_ELBOW,
    PWM_R_ELBOW,
    PWM_L_BICEP,
    PWM_R_BICEP,
    PWM_L_SHOULDER_FRONT_RAISE,
    PWM_R_SHOULDER_FRONT_RAISE,
    PWM_L_SHOULDER_LAT_RAISE,
    PWM_R_SHOULDER_LAT_RAISE,
    NUM_PWM_CHANNELS,
} pwm_e;
#endif

#if defined(ARM_SLEEVE)
typedef enum {
    PWM_CHANNEL_1,
    PWM_CHANNEL_2,
    PWM_CHANNEL_3,
    PWM_CHANNEL_4,
    NUM_PWM_CHANNELS,
} pwm_e;
#endif

void pwm_init(void);
void pwm_set_duty_cycle(pwm_e pwm_channel, uint8_t duty_cycle_percent);

#endif
