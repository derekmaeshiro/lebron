#include <stdint.h>

#ifndef SERVO
#define SERVO

#if defined(ROBOTIC_ARM)
typedef enum {
    SERVO_DISTAL_INTERPHALANGEAL_JOINT,
    SERVO_PROXIMAL_INTERPHALANGEAL_JOINT,
    SERVO_METACARPOPHALANGEAL_JOINT_1,
    SERVO_METACARPOPHALANGEAL_JOINT_2,
} servo_e;

void servo_init(void);
void servo_set_pwm_duty_cycle(servo_e servo, uint8_t angle);

#endif
#endif