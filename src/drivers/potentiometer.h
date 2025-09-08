#include <stdint.h>
#include "io.h"
#include "analog_mux.h"
#include "../common/joints.h"

#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

struct potentiometer_config
{
    analog_mux_e mux;
    uint8_t mux_pin;
};

void potentiometer_init(void);
uint16_t potentiometer_read(joint_e joint);
void read_all_potentiometers(uint16_t *angles);
void calibrate_potentiometers(void);
#endif
