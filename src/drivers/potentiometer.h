#include <stdint.h>
#include "io.h"
#include "mux.h"

#ifndef POTENTIOMETER_BOARD_H
#define POTENTIOMETER_BOARD_H

struct potentiometer_config{
    mux_e mux;
    mux_pin_e mux_pin;
}

// List all potentiometers
typedef enum {
    MIDDLE_FINGER_DISTAL_JOINT,
    MIDDLE_FINGER_PROXIMAL_JOINT
} potentiometer_e;

void potentiometer_init(void);
uint16_t potentiometer_read(potentiometer_e potentiometer);

#endif
