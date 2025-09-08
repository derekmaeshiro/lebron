#include "io.h"
#include "adc.h"

#ifndef ANALOG_MUX_H
#define ANALOG_MUX_H

typedef enum {
    MUX_BOARD_1,
    MUX_BOARD_2,
} analog_mux_e;

void analog_mux_init(void);
void toggle_analog_mux(analog_mux_e mux, uint8_t mux_pin);

#endif
