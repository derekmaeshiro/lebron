#include "adc.h"
#include "io.h"
#include "mux.h"
#include "potentiometer.h"
#include "../common/assert_handler.h"

const uint8_t potentiometer_to_adc_channel[] = {
    [POTENTIOMETER_BOARD_1] = 0, [POTENTIOMETER_BOARD_2] = 1
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
uint16_t potentiometer_board_read(potentiometer_e potentiometer)
{
    adc_channel_values_t adc_values;
    adc_get_channel_values(adc_values);
    uint8_t adc_pin = potentiometer_to_adc_channel[potentiometer];
    uint16_t angle = convert_adc_value_to_angle(adc_values[adc_pin]);
    return angle;
}