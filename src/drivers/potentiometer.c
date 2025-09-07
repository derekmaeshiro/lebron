#include "adc.h"
#include "io.h"
#include "analog_mux.h"
#include "potentiometer.h"
#include "../common/assert_handler.h"
#include "../common/defines.h"
#include "../common/trace.h"

#if defined ROBOTIC_ARM
const struct potentiometer_config potentiometer_configs[] = {
    [THUMB_DISTAL] = { .mux = MUX_BOARD_1, .mux_pin = 0 },
    [THUMB_MIDDLE] = { .mux = MUX_BOARD_1, .mux_pin = 1 },
    [THUMB_METACARPAL] = { .mux = MUX_BOARD_1, .mux_pin = 2 },
    [INDEX_DISTAL] = { .mux = MUX_BOARD_1, .mux_pin = 3 },
    [INDEX_MIDDLE] = { .mux = MUX_BOARD_1, .mux_pin = 4 },
    [INDEX_PROXIMAL] = { .mux = MUX_BOARD_1, .mux_pin = 5 },
    [INDEX_METACARPAL] = { .mux = MUX_BOARD_1, .mux_pin = 6 },
    [MIDDLE_DISTAL] = { .mux = MUX_BOARD_1, .mux_pin = 7 },
    [MIDDLE_MIDDLE] = { .mux = MUX_BOARD_1, .mux_pin = 8 },
    [MIDDLE_PROXIMAL] = { .mux = MUX_BOARD_1, .mux_pin = 9 },
    [MIDDLE_METACARPAL] = { .mux = MUX_BOARD_1, .mux_pin = 10 },
    [RING_DISTAL] = { .mux = MUX_BOARD_1, .mux_pin = 11 },
    [RING_MIDDLE] = { .mux = MUX_BOARD_1, .mux_pin = 12 },
    [RING_PROXIMAL] = { .mux = MUX_BOARD_1, .mux_pin = 13 },
    [RING_METACARPAL] = { .mux = MUX_BOARD_1, .mux_pin = 14 },
    [PINKY_DISTAL] = { .mux = MUX_BOARD_1, .mux_pin = 15 },

    [PINKY_MIDDLE] = { .mux = MUX_BOARD_2, .mux_pin = 0 },
    [PINKY_PROXIMAL] = { .mux = MUX_BOARD_2, .mux_pin = 1 },
    [PINKY_METACARPAL] = { .mux = MUX_BOARD_2, .mux_pin = 2 },
    [WRIST_WAVE] = { .mux = MUX_BOARD_2, .mux_pin = 3 },
    [WRIST_FLICK] = { .mux = MUX_BOARD_2, .mux_pin = 4 },
    [WRIST_NAE_NAE] = { .mux = MUX_BOARD_2, .mux_pin = 5 },
    [ELBOW] = { .mux = MUX_BOARD_2, .mux_pin = 6 },
    [BICEP] = { .mux = MUX_BOARD_2, .mux_pin = 7 },
    [SHOULDER_FRONT_RAISE] = { .mux = MUX_BOARD_2, .mux_pin = 8 },
    [SHOULDER_LAT_RAISE] = { .mux = MUX_BOARD_2, .mux_pin = 9 },
};
#endif

#if defined ARM_SLEEVE
const struct potentiometer_config potentiometer_configs[] = {
    [THUMB_DISTAL] = { .mux = MUX_BOARD_1, .mux_pin = 0 },
    [THUMB_MIDDLE] = { .mux = MUX_BOARD_1, .mux_pin = 1 },
    [THUMB_METACARPAL] = { .mux = MUX_BOARD_1, .mux_pin = 2 },
    [INDEX_DISTAL] = { .mux = MUX_BOARD_1, .mux_pin = 3 },
    [INDEX_MIDDLE] = { .mux = MUX_BOARD_1, .mux_pin = 4 },
    [INDEX_PROXIMAL] = { .mux = MUX_BOARD_1, .mux_pin = 5 },
    [INDEX_METACARPAL] = { .mux = MUX_BOARD_1, .mux_pin = 6 },
    [MIDDLE_DISTAL] = { .mux = MUX_BOARD_1, .mux_pin = 7 },
    [MIDDLE_MIDDLE] = { .mux = MUX_BOARD_1, .mux_pin = 8 },
    [MIDDLE_PROXIMAL] = { .mux = MUX_BOARD_1, .mux_pin = 9 },
    [MIDDLE_METACARPAL] = { .mux = MUX_BOARD_1, .mux_pin = 10 },
    [RING_DISTAL] = { .mux = MUX_BOARD_1, .mux_pin = 11 },
    [RING_MIDDLE] = { .mux = MUX_BOARD_1, .mux_pin = 12 },
    [RING_PROXIMAL] = { .mux = MUX_BOARD_1, .mux_pin = 13 },
    [RING_METACARPAL] = { .mux = MUX_BOARD_1, .mux_pin = 14 },
    [PINKY_DISTAL] = { .mux = MUX_BOARD_1, .mux_pin = 15 },

    [PINKY_MIDDLE] = { .mux = MUX_BOARD_2, .mux_pin = 0 },
    [PINKY_PROXIMAL] = { .mux = MUX_BOARD_2, .mux_pin = 1 },
    [PINKY_METACARPAL] = { .mux = MUX_BOARD_2, .mux_pin = 2 },
};
#endif

uint16_t convert_adc_value_to_angle(uint16_t adc_value)
{
    uint16_t adc_max = 4095;
    if (adc_value > adc_max) {
        adc_value = adc_max;
    }
    return (uint16_t)(((adc_value * 360UL) + (adc_max / 2)) / adc_max);
}

static bool initialized = false;
void potentiometer_init(void)
{
    ASSERT(!initialized);
    initialized = true;
}

uint16_t potentiometer_read(potentiometer_e potentiometer)
{
    analog_mux_e mux = potentiometer_configs[potentiometer].mux;
    uint8_t mux_pin = potentiometer_configs[potentiometer].mux_pin;
    toggle_analog_mux(mux, mux_pin);

    // Flush to allow mux & ADC to settle
    adc_read_single(mux == MUX_BOARD_1 ? 0 : 1);

    uint16_t adc_value = adc_read_single(mux == MUX_BOARD_1 ? 0 : 1);
    return convert_adc_value_to_angle(adc_value);
}

#if defined ROBOTIC_ARM
int16_t potentiometer_bias[NUM_OF_POTENTIOMETERS] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // 26
#endif

#if defined ARM_SLEEVE
int16_t potentiometer_bias[NUM_OF_POTENTIOMETERS] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                      0, 0, 0, 0, 0, 0, 0, 0, 0 }; // 19
#endif

void calibrate_potentiometers(void)
{
    uint16_t trials = 5;
    for (uint8_t potentiometer = 0; potentiometer < (uint8_t)NUM_OF_POTENTIOMETERS;
         potentiometer++) {
        uint16_t potentiometer_average = 0;
        for (uint8_t trial = 0; trial < trials; trial++) {
            potentiometer_average += (uint16_t)potentiometer_read((potentiometer_e)potentiometer);
            BUSY_WAIT_ms(5);
        }
        potentiometer_average /= trials;
        potentiometer_bias[potentiometer] = potentiometer_average;
    }
}

void read_all_potentiometers(uint16_t *angles)
{
    for (uint8_t i = 0; i < (uint8_t)NUM_OF_POTENTIOMETERS; i++) {
        angles[i] = potentiometer_read((potentiometer_e)i) - potentiometer_bias[i];
    }
}
// #endif
