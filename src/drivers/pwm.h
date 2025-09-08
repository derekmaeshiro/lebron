#include <stdint.h>
#include <stm32f4xx.h>
#include "../common/joints.h"

#ifndef PWM_H
#define PWM_H

extern const joint_e pwm_joints[];

#if defined(ROBOTIC_ARM)
#define NUM_PWM_CHANNELS 5
#endif

#if defined(ARM_SLEEVE)
#define NUM_PWM_CHANNELS 4
#endif

int get_pwm_channel_index(joint_e joint);
void pwm_init(void);
void pwm_set_duty_cycle(joint_e joint, uint8_t duty_cycle_percentt);

#endif
