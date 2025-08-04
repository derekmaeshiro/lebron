#include "pwm.h"
#include "servo.h"
#include "../common/assert_handler.h"
#include <stdint.h>
#include <stdbool.h>

#if defined(ROBOTIC_ARM)
static bool initialized = false;
static pwm_e servo_e_to_pwm_e[] = {
    [SERVO_DISTAL_INTERPHALANGEAL_JOINT] = PWM_DISTAL_INTERPHALANGEAL_JOINT,
    [SERVO_PROXIMAL_INTERPHALANGEAL_JOINT] = PWM_PROXIMAL_INTERPHALANGEAL_JOINT,
    [SERVO_METACARPOPHALANGEAL_JOINT_1] = PWM_METACARPOPHALANGEAL_JOINT_1,
    [SERVO_METACARPOPHALANGEAL_JOINT_2] = PWM_METACARPOPHALANGEAL_JOINT_2,
};

void servo_init(void)
{
    ASSERT(!initialized);

    pwm_init();
    initialized = true;
}

void servo_set_pwm_duty_cycle(servo_e servo, uint8_t angle)
{
    ASSERT(angle <= 180);
    pwm_e pwm = servo_e_to_pwm_e[servo];
    uint8_t duty_cycle_percent = (angle * 100) / 180;
    pwm_set_duty_cycle(pwm, duty_cycle_percent);
}
#endif