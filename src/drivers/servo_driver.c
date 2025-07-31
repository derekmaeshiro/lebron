#include "servo_driver.h"
#include "i2c.h"

#include <stddef.h>

#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2500
#define PCA9685_PWM_RESOLUTION 4096
#define PCA9685_PWM_FREQUENCY 50

static void servo_driver_set_pwm_channel(uint8_t slave_address, uint8_t channel, uint16_t on,
                                         uint16_t off)
{
    uint8_t base_reg = 0x06 + 4 * channel;
    uint8_t payload[5];
    payload[0] = base_reg;
    payload[1] = on & 0xFF;
    payload[2] = on >> 8;
    payload[3] = off & 0xFF;
    payload[4] = off >> 8;

    i2c_write(slave_address, payload, 5, NULL);
    while (i2c_is_busy()) { }
}

uint16_t angle_to_off_converter(uint8_t angle)
{
    if (angle > 180)
        angle = 180;
    uint16_t pulse_width_us =
        SERVO_MIN_PULSE_US + ((SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * angle) / 180;
    uint32_t pulse_period_us = 1000000UL / PCA9685_PWM_FREQUENCY;
    float us_per_count = (float)pulse_period_us / PCA9685_PWM_RESOLUTION;
    uint16_t off_count = (uint16_t)(pulse_width_us / us_per_count);
    return off_count;
}

void servo_driver_set_servo_angle(servo_driver_t *driver, servo_channel_t channel, uint8_t angle)
{
    servo_driver_set_pwm_channel(driver->slave_address, channel, 0, angle_to_off_converter(angle));
    driver->servos[channel].servo_angle = angle;
    driver->servos[channel].channel_number = (servo_channel_t)channel;
}

// Set all channels' angles at once
void servo_driver_set_all_servo_angles(servo_driver_t *driver, const uint8_t angles[])
{
    for (int i = 0; i < SERVO_DRIVER_MAX_CHANNELS; i++) {
        servo_driver_set_servo_angle(driver, i, angles[i]);
    }
}

void servo_driver_init(servo_driver_t *driver, uint8_t slave_address)
{
    driver->slave_address = slave_address;
    const uint8_t sleep[] = { 0x00, 0x10 };
    const uint8_t prescale[] = { 0xFE, 121 };
    const uint8_t wake[] = { 0x00, 0x20 };

    // Sleep
    i2c_write(slave_address, sleep, 2, NULL);
    while (i2c_is_busy()) { }

    // Prescale for 50Hz
    i2c_write(slave_address, prescale, 2, NULL);
    while (i2c_is_busy()) { }

    // Wake, auto-increment
    i2c_write(slave_address, wake, 2, NULL);
    while (i2c_is_busy()) { }

    // Set all to 90 degrees
    uint8_t angles[SERVO_DRIVER_MAX_CHANNELS];
    for (int i = 0; i < SERVO_DRIVER_MAX_CHANNELS; i++)
        angles[i] = 90;
    servo_driver_set_all_servo_angles(driver, angles);
}

uint8_t servo_driver_get_servo_angle(servo_driver_t *driver, servo_channel_t channel)
{
    return driver->servos[channel].servo_angle;
}