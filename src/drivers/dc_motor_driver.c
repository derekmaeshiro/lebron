#include "dc_motor_driver.h"
#include "potentiometer.h"
#include "pwm.h"
#include "../common/joints.h"
#include "../common/trace.h"
#include <math.h>

#if defined(ROBOTIC_ARM)
#define NUM_DC_MOTOR_JOINTS 5

static const joint_e motor_joints[NUM_DC_MOTOR_JOINTS] = { WRIST_NAE_NAE, ELBOW, BICEP,
                                                           SHOULDER_FRONT_RAISE,
                                                           SHOULDER_LAT_RAISE };

static const pwm_e pwm_r_channels[NUM_DC_MOTOR_JOINTS] = { PWM_R_WRIST_NAE_NAE, PWM_R_ELBOW,
                                                           PWM_R_BICEP, PWM_R_SHOULDER_FRONT_RAISE,
                                                           PWM_R_SHOULDER_LAT_RAISE };

static const pwm_e pwm_l_channels[NUM_DC_MOTOR_JOINTS] = { PWM_L_WRIST_NAE_NAE, PWM_L_ELBOW,
                                                           PWM_L_BICEP, PWM_L_SHOULDER_FRONT_RAISE,
                                                           PWM_L_SHOULDER_LAT_RAISE };

#elif defined(ARM_SLEEVE)
#define NUM_DC_MOTOR_JOINTS 2

static const joint_e motor_joints[NUM_DC_MOTOR_JOINTS] = {
    ELBOW, BICEP // Only two elements!
};

static const pwm_e pwm_r_channels[NUM_DC_MOTOR_JOINTS] = { PWM_CHANNEL_1, PWM_CHANNEL_2 };

static const pwm_e pwm_l_channels[NUM_DC_MOTOR_JOINTS] = { PWM_CHANNEL_3, PWM_CHANNEL_4 };

#else
#error "Missing mode define for NUM_DC_MOTOR_JOINTS"
#endif

typedef struct
{
    // PID parameters
    float Kp, Ki, Kd, Ts;
    float Outmin, Outmax;
    float Anti_windup_error;
    int Anti_windup;
    // PID state
    float error_sum;
    float prev_input;
    // Control setpoint
    uint16_t setpoint;
    // Hardware mapping
    pwm_e rpwm_channel;
    pwm_e lpwm_channel;
} smart_motor_t;

static smart_motor_t motors[NUM_DC_MOTOR_JOINTS];

void smart_motor_init(void)
{
    for (int i = 0; i < NUM_DC_MOTOR_JOINTS; ++i) {
        motors[i].Kp = 15.0;
        motors[i].Ki = 0.2;
        motors[i].Kd = 0.05;
        motors[i].Ts = 0.02; // 20ms update period
        motors[i].Outmin = -100;
        motors[i].Outmax = 100;
        motors[i].Anti_windup_error = 10;
        motors[i].Anti_windup = 1;

        motors[i].error_sum = 0;
        motors[i].prev_input = 0;
        motors[i].setpoint = 0;

        motors[i].rpwm_channel = pwm_r_channels[i];
        motors[i].lpwm_channel = pwm_l_channels[i];
    }
}

void smart_motor_set_angle(joint_e joint, uint16_t angle)
{
    for (int i = 0; i < NUM_DC_MOTOR_JOINTS; ++i) {
        if (motor_joints[i] == joint) {
            motors[i].setpoint = angle;
            break;
        }
    }
}

void smart_motor_set_pid(joint_e joint, float Kp, float Ki, float Kd, float Ts)
{
    for (int i = 0; i < NUM_DC_MOTOR_JOINTS; ++i) {
        if (motor_joints[i] == joint) {
            motors[i].Kp = Kp;
            motors[i].Ki = Ki;
            motors[i].Kd = Kd;
            motors[i].Ts = Ts;
            break;
        }
    }
}

static float motor_pid_calc(smart_motor_t *m, float measured_input)
{
    float error = (float)m->setpoint - measured_input;
    int apply_integrator = 1;
    if (m->Anti_windup && fabsf(error) > m->Anti_windup_error)
        apply_integrator = 0;

    if (apply_integrator)
        m->error_sum += error;
    if (m->error_sum > m->Outmax)
        m->error_sum = m->Outmax;
    if (m->error_sum < m->Outmin)
        m->error_sum = m->Outmin;

    float derivative = (measured_input - m->prev_input) / m->Ts;
    float output = m->Kp * error + m->Ki * m->error_sum * m->Ts - m->Kd * derivative;

    if (output > m->Outmax)
        output = m->Outmax;
    if (output < m->Outmin)
        output = m->Outmin;

    m->prev_input = measured_input;

    return output;
}

// void smart_motor_update(void)
// {
//     for (int i = 0; i < NUM_DC_MOTOR_JOINTS; ++i) {
//         uint16_t measured_angle = potentiometer_read(motor_joints[i]);
//         TRACE("Joint %d: measured_angle=%u\n", i, measured_angle);
//     }
// }

void smart_motor_update(void)
{
    for (int i = 0; i < NUM_DC_MOTOR_JOINTS; ++i) {
        smart_motor_t *m = &motors[i];
        uint16_t measured_angle = potentiometer_read(motor_joints[i]);
        float pid_out = motor_pid_calc(m, (float)measured_angle);

        uint8_t speed = (uint8_t)fabsf(pid_out);
        if (speed > 100)
            speed = 100;

        if (pid_out > 0) {
            pwm_set_duty_cycle(m->rpwm_channel, speed);
            pwm_set_duty_cycle(m->lpwm_channel, 0);
        } else if (pid_out < 0) {
            pwm_set_duty_cycle(m->rpwm_channel, 0);
            pwm_set_duty_cycle(m->lpwm_channel, speed);
        } else {
            pwm_set_duty_cycle(m->rpwm_channel, 0);
            pwm_set_duty_cycle(m->lpwm_channel, 0);
        }
    }
}