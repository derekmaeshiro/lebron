#include <stdint.h>
#include "io.h"

#ifndef POTENTIOMETER_BOARD_H
#define POTENTIOMETER_BOARD_H

typedef enum {
    POTENTIOMETER_BOARD_1,
    POTENTIOMETER_BOARD_2
} potentiometer_e;

void potentiometer_init(void);
uint16_t potentiometer_board_read(potentiometer_e potentiometer);
uint16_t convert_adc_value_to_angle(uint16_t adc_value);

#endif
