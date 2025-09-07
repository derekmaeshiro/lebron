#include <stdint.h>
#include "io.h"
#include "analog_mux.h"

#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

struct potentiometer_config
{
    analog_mux_e mux;
    uint8_t mux_pin;
};

// List all potentiometers
#if defined ROBOTIC_ARM
typedef enum {
    THUMB_DISTAL,
    THUMB_MIDDLE,
    THUMB_METACARPAL,
    INDEX_DISTAL,
    INDEX_MIDDLE,
    INDEX_PROXIMAL,
    INDEX_METACARPAL,
    MIDDLE_DISTAL,
    MIDDLE_MIDDLE,
    MIDDLE_PROXIMAL,
    MIDDLE_METACARPAL,
    RING_DISTAL,
    RING_MIDDLE,
    RING_PROXIMAL,
    RING_METACARPAL,
    PINKY_DISTAL,
    PINKY_MIDDLE,
    PINKY_PROXIMAL,
    PINKY_METACARPAL,
    WRIST_WAVE,
    WRIST_FLICK,
    WRIST_NAE_NAE,
    ELBOW,
    BICEP,
    SHOULDER_FRONT_RAISE,
    SHOULDER_LAT_RAISE,
    NUM_OF_POTENTIOMETERS, // Always keep last: represents total number of potentiometers
} potentiometer_e;
#endif

#if defined ARM_SLEEVE
typedef enum {
    THUMB_DISTAL,
    THUMB_MIDDLE,
    THUMB_METACARPAL,
    INDEX_DISTAL,
    INDEX_MIDDLE,
    INDEX_PROXIMAL,
    INDEX_METACARPAL,
    MIDDLE_DISTAL,
    MIDDLE_MIDDLE,
    MIDDLE_PROXIMAL,
    MIDDLE_METACARPAL,
    RING_DISTAL,
    RING_MIDDLE,
    RING_PROXIMAL,
    RING_METACARPAL,
    PINKY_DISTAL,
    PINKY_MIDDLE,
    PINKY_PROXIMAL,
    PINKY_METACARPAL,
    NUM_OF_POTENTIOMETERS,
} potentiometer_e;
#endif

void potentiometer_init(void);
uint16_t potentiometer_read(potentiometer_e potentiometer);
void read_all_potentiometers(uint16_t *angles);
void calibrate_potentiometers(void);
#endif
