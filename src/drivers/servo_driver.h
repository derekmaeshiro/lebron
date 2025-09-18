#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <stdint.h>

typedef enum {
    CHANNEL_0 = 0,
    CHANNEL_1,
    CHANNEL_2,
    CHANNEL_3,
    CHANNEL_4,
    CHANNEL_5,
    CHANNEL_6,
    CHANNEL_7,
    CHANNEL_8,
    CHANNEL_9,
    CHANNEL_10,
    CHANNEL_11,
    CHANNEL_12,
    CHANNEL_13,
    CHANNEL_14,
    CHANNEL_15,
    SERVO_DRIVER_MAX_CHANNELS,
} servo_channel_t;

typedef enum {
    SERVO_THUMB_PROXIMAL = CHANNEL_0,
    SERVO_THUMB_DISTAL = CHANNEL_1,
    SERVO_THUMB_METACARPAL = CHANNEL_2,
    SERVO_INDEX_FINGER_PROXIMAL = CHANNEL_3,
    SERVO_INDEX_FINGER_DISTAL = CHANNEL_4,
    SERVO_INDEX_FINGER_METACARPAL = CHANNEL_5,
    SERVO_MIDDLE_FINGER_PROXIMAL = CHANNEL_6,
    SERVO_MIDDLE_FINGER_DISTAL = CHANNEL_7,
    SERVO_MIDDLE_FINGER_METACARPAL = CHANNEL_8,
    SERVO_RING_FINGER_PROXIMAL = CHANNEL_9,
    SERVO_RING_FINGER_DISTAL = CHANNEL_10,
    SERVO_RING_FINGER_METACARPAL = CHANNEL_11,
    SERVO_PINKY_FINGER_PROXIMAL = CHANNEL_12,
    SERVO_PINKY_FINGER_DISTAL = CHANNEL_13,
    SERVO_PINKY_FINGER_METACARPAL = CHANNEL_14,
} servo_channel_name_t;

typedef struct
{
    servo_channel_t channel_number;
    uint8_t servo_angle; // 0-180 degrees (or whatever range you define)
} servo_t;

typedef struct
{
    uint8_t slave_address; // I2C address of the PCA9685
    servo_t servos[SERVO_DRIVER_MAX_CHANNELS];
} servo_driver_t;

void servo_driver_init(servo_driver_t *driver, uint8_t slave_address);
void servo_driver_set_servo_angle(servo_driver_t *driver, servo_channel_t channel, uint16_t angle);
void servo_driver_set_all_servo_angles(servo_driver_t *driver, const uint8_t angles[]);
uint8_t servo_driver_get_servo_angle(servo_driver_t *driver, servo_channel_t channel);
uint16_t angle_to_off_converter(uint16_t angle);

#endif // SERVO_DRIVER_H