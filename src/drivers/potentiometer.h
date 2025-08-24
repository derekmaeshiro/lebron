#include <stdint.h>
#include "io.h"
#include "analog_mux.h"

#ifndef POTENTIOMETER_BOARD_H
#define POTENTIOMETER_BOARD_H

struct potentiometer_config
{
    analog_mux_e mux;
    uint8_t mux_pin;
};

// List all potentiometers
// TODO: I'm assuming potentiometer_e is the same in ROBOTIC_ARM and ARM_SLEEVE. Adjust if needed
typedef enum {
    THUMB_PROXIMAL,
    THUMB_DISTAL,
    THUMB_METACARPAL,
    INDEX_PROXIMAL,
    INDEX_DISTAL,
    INDEX_METACARPAL,
    MIDDLE_PROXIMAL,
    MIDDLE_DISTAL,
    MIDDLE_METACARPAL,
    RING_PROXIMAL,
    RING_DISTAL,
    RING_METACARPAL,
    PINKY_PROXIMAL,
    PINKY_DISTAL,
    PINKY_METACARPAL,
} potentiometer_e;

void potentiometer_init(void);
uint16_t potentiometer_read(potentiometer_e potentiometer);
void read_all_potentiometers(uint16_t *angles);
#endif

