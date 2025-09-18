#include "adc.h"
#include "io.h"
#include "analog_mux.h"
#include "potentiometer.h"
#include "../common/assert_handler.h"
#include "../common/defines.h"
#include "../common/trace.h"

#if defined ROBOTIC_ARM
const struct potentiometer_config potentiometer_configs[NUM_OF_JOINTS] = {
    [THUMB_DISTAL] = { MUX_BOARD_1, 0 },
    [THUMB_MIDDLE] = { MUX_BOARD_1, 1 },
    [THUMB_METACARPAL] = { MUX_BOARD_1, 2 },
    [INDEX_DISTAL] = { MUX_BOARD_1, 3 },
    [INDEX_MIDDLE] = { MUX_BOARD_1, 4 },
    [INDEX_PROXIMAL] = { MUX_BOARD_1, 5 },
    [INDEX_METACARPAL] = { MUX_BOARD_1, 6 },
    [MIDDLE_DISTAL] = { MUX_BOARD_1, 7 },
    [MIDDLE_MIDDLE] = { MUX_BOARD_1, 8 },
    [MIDDLE_PROXIMAL] = { MUX_BOARD_1, 9 },
    [MIDDLE_METACARPAL] = { MUX_BOARD_1, 10 },
    [RING_DISTAL] = { MUX_BOARD_1, 11 },
    [RING_MIDDLE] = { MUX_BOARD_1, 12 },
    [RING_PROXIMAL] = { MUX_BOARD_1, 13 },
    [RING_METACARPAL] = { MUX_BOARD_1, 14 },
    [PINKY_DISTAL] = { MUX_BOARD_1, 15 },

    [PINKY_MIDDLE] = { MUX_BOARD_2, 0 },
    [PINKY_PROXIMAL] = { MUX_BOARD_2, 1 },
    [PINKY_METACARPAL] = { MUX_BOARD_2, 2 },
    [WRIST_WAVE] = { MUX_BOARD_2, 3 },
    [WRIST_FLICK] = { MUX_BOARD_2, 4 },
    [WRIST_NAE_NAE] = { MUX_BOARD_2, 5 },
    [ELBOW] = { MUX_BOARD_2, 6 },
    [BICEP] = { MUX_BOARD_2, 7 },
    [SHOULDER_FRONT_RAISE] = { MUX_BOARD_2, 8 },
    [SHOULDER_LAT_RAISE] = { MUX_BOARD_2, 9 },
};
#endif

#if defined ARM_SLEEVE
const struct potentiometer_config potentiometer_configs[NUM_OF_JOINTS] = {
    [THUMB_DISTAL] = { MUX_BOARD_1, 0 },       [THUMB_MIDDLE] = { MUX_BOARD_1, 1 },
    [THUMB_METACARPAL] = { MUX_BOARD_1, 2 },   [INDEX_DISTAL] = { MUX_BOARD_1, 3 },
    [INDEX_MIDDLE] = { MUX_BOARD_1, 4 },       [INDEX_PROXIMAL] = { MUX_BOARD_1, 5 },
    [INDEX_METACARPAL] = { MUX_BOARD_1, 6 },   [MIDDLE_DISTAL] = { MUX_BOARD_1, 7 },
    [MIDDLE_MIDDLE] = { MUX_BOARD_1, 8 },      [MIDDLE_PROXIMAL] = { MUX_BOARD_1, 9 },
    [MIDDLE_METACARPAL] = { MUX_BOARD_1, 10 }, [RING_DISTAL] = { MUX_BOARD_1, 11 },
    [RING_MIDDLE] = { MUX_BOARD_1, 12 },       [RING_PROXIMAL] = { MUX_BOARD_1, 13 },
    [RING_METACARPAL] = { MUX_BOARD_1, 14 },   [PINKY_DISTAL] = { MUX_BOARD_1, 15 },

    [PINKY_MIDDLE] = { MUX_BOARD_2, 0 },       [PINKY_PROXIMAL] = { MUX_BOARD_2, 1 },
    [PINKY_METACARPAL] = { MUX_BOARD_2, 2 },
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

uint16_t potentiometer_read(joint_e joint)
{
    analog_mux_e mux = potentiometer_configs[joint].mux;
    uint8_t mux_pin = potentiometer_configs[joint].mux_pin;

    toggle_analog_mux(mux, mux_pin);
    adc_read_single(mux == MUX_BOARD_1 ? 0 : 1);
    uint16_t adc_value = adc_read_single(mux == MUX_BOARD_1 ? 0 : 1);
    return convert_adc_value_to_angle(adc_value);
}

#if defined ROBOTIC_ARM
int16_t potentiometer_bias[NUM_OF_JOINTS] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // 26
#endif

#if defined ARM_SLEEVE
int16_t potentiometer_bias[NUM_OF_JOINTS] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0, 0, 0, 0 }; // 19
#endif

void calibrate_potentiometers(void)
{
    uint16_t trials = 5;
    for (uint8_t potentiometer = 0; potentiometer < (uint8_t)NUM_OF_JOINTS; potentiometer++) {
        uint16_t potentiometer_average = 0;
        for (uint8_t trial = 0; trial < trials; trial++) {
            potentiometer_average += (uint16_t)potentiometer_read((joint_e)potentiometer);
            BUSY_WAIT_ms(5);
        }
        potentiometer_average /= trials;
        potentiometer_bias[potentiometer] = potentiometer_average;
    }
}

void read_all_potentiometers(uint16_t *angles)
{
    for (uint8_t i = 0; i < (uint8_t)NUM_OF_JOINTS; i++) {
        uint16_t read = potentiometer_read((joint_e)i); // 0...360
        uint16_t bias = potentiometer_bias[i]; // 0...360
        angles[i] = (read + 360 - bias) % 360;
    }
}

uint16_t potentiometer_read_adc(joint_e joint)
{
    analog_mux_e mux = potentiometer_configs[joint].mux;
    uint8_t mux_pin = potentiometer_configs[joint].mux_pin;
    toggle_analog_mux(mux, mux_pin);
    adc_read_single(mux == MUX_BOARD_1 ? 0 : 1); // discard first reading after switching
    return adc_read_single(mux == MUX_BOARD_1 ? 0 : 1); // return raw ADC value
}
