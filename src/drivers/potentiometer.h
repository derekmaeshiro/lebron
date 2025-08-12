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
typedef enum {
    POTENTIOMETER_1,
    POTENTIOMETER_2
} potentiometer_e;

void potentiometer_init(void);
uint16_t potentiometer_read(potentiometer_e potentiometer);

#endif
