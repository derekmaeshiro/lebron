#include "adc.h"
#include "io.h"
#include "mux.h"
#include "potentiometer.h"
#include "../common/assert_handler.h"

const struct potentiometer_config potentiometer_configs[] = {
    [MIDDLE_FINGER_DISTAL_JOINT] = {
        .mux = MUX_BOARD_1,
        .mux_pin = PIN1,
    }, 
    [MIDDLE_FINGER_PROXIMAL_JOINT] = {
        .mux = MUX_BOARD_1,
        .mux_pin = PIN2,
    }, 
};

uint16_t convert_adc_value_to_angle(uint16_t adc_value)
{
    uint16_t adc_max = 4095;
    if (adc_value > adc_max) {
        adc_value = adc_max;
    }
    return (uint16_t)(((float)adc_value / adc_max) * 360.0f);
}

static bool initialized = false;
void potentiometer_init()
{
    ASSERT(!initialized);
    initialized = true;
}

uint16_t potentiometer_read(potentiometer_e potentiometer)
{
    adc_channel_values_t adc_values;
    adc_get_channel_values(adc_values);

    mux_e mux = potentiometer_configs[potentiometer].mux;
    mux_pin_e mux_pin = potentiometer_configs[potentiometer].mux_pin;

    uint16_t adc_value = get_adc_value_from_mux(mux, mux_pin);
    uint16_t angle = convert_adc_value_to_angle(adc_value);
    return angle;
}