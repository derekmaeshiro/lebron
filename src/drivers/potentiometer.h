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

extern const struct potentiometer_config potentiometer_configs[NUM_OF_JOINTS];
extern int16_t potentiometer_bias[NUM_OF_JOINTS];

void potentiometer_init(void);
uint16_t potentiometer_read(joint_e joint);
void read_all_potentiometers(uint16_t *angles);
void calibrate_potentiometers(void);
uint16_t convert_adc_value_to_angle(uint16_t adc_value);
uint16_t potentiometer_read_adc(joint_e joint);

#endif
