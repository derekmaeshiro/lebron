#include <stdint.h>
#include <stm32f4xx.h>

#ifndef PWM_H
#define PWM_H

typedef enum {
    PWM_DISTAL_INTERPHALANGEAL_JOINT,
    PWM_PROXIMAL_INTERPHALANGEAL_JOINT,
    PWM_METACARPOPHALANGEAL_JOINT_1,
    PWM_METACARPOPHALANGEAL_JOINT_2
} pwm_e;

void pwm_init(void);
void pwm_set_duty_cycle(pwm_e pwm, uint8_t duty_cycle_percent);

#endif