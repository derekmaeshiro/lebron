#include "adc.h"
#include "io.h"
#include "analog_mux.h"
#include "potentiometer.h"
#include "../common/assert_handler.h"
#include "../common/defines.h"
#include "../common/trace.h"

#if defined ROBOTIC_ARM
const struct potentiometer_config potentiometer_configs[] = {
    [THUMB_PROXIMAL] = { .mux = MUX_BOARD_1, .mux_pin = 0 },
    [THUMB_DISTAL] = { .mux = MUX_BOARD_1, .mux_pin = 1 },
    [THUMB_METACARPAL] = { .mux = MUX_BOARD_1, .mux_pin = 2 },
    [INDEX_PROXIMAL] = { .mux = MUX_BOARD_1, .mux_pin = 3 },
    [INDEX_DISTAL] = { .mux = MUX_BOARD_1, .mux_pin = 4 },
    [INDEX_METACARPAL] = { .mux = MUX_BOARD_1, .mux_pin = 5 },
    [MIDDLE_PROXIMAL] = { .mux = MUX_BOARD_1, .mux_pin = 6 },
    [MIDDLE_DISTAL] = { .mux = MUX_BOARD_1, .mux_pin = 7 },
    [MIDDLE_METACARPAL] = { .mux = MUX_BOARD_1, .mux_pin = 8 },
    [RING_PROXIMAL] = { .mux = MUX_BOARD_1, .mux_pin = 9 },
    [RING_DISTAL] = { .mux = MUX_BOARD_1, .mux_pin = 10 },
    [RING_METACARPAL] = { .mux = MUX_BOARD_1, .mux_pin = 12 },
    [PINKY_PROXIMAL] = { .mux = MUX_BOARD_1, .mux_pin = 13 },
    [PINKY_DISTAL] = { .mux = MUX_BOARD_1, .mux_pin = 14 },
    [PINKY_METACARPAL] = { .mux = MUX_BOARD_1, .mux_pin = 15 },
};
#endif

#if defined ARM_SLEEVE
const struct potentiometer_config potentiometer_configs[] = {
    [THUMB_PROXIMAL] = { .mux_pin = 0 },    [THUMB_DISTAL] = { .mux_pin = 1 },
    [THUMB_METACARPAL] = { .mux_pin = 2 },  [INDEX_PROXIMAL] = { .mux_pin = 3 },
    [INDEX_DISTAL] = { .mux_pin = 4 },      [INDEX_METACARPAL] = { .mux_pin = 5 },
    [MIDDLE_PROXIMAL] = { .mux_pin = 6 },   [MIDDLE_DISTAL] = { .mux_pin = 7 },
    [MIDDLE_METACARPAL] = { .mux_pin = 8 }, [RING_PROXIMAL] = { .mux_pin = 9 },
    [RING_DISTAL] = { .mux_pin = 10 },      [RING_METACARPAL] = { .mux_pin = 12 },
    [PINKY_PROXIMAL] = { .mux_pin = 13 },   [PINKY_DISTAL] = { .mux_pin = 14 },
    [PINKY_METACARPAL] = { .mux_pin = 15 },
};
#endif

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
#if defined ROBOTIC_ARM
    analog_mux_e mux = potentiometer_configs[potentiometer].mux;
    uint8_t mux_pin = potentiometer_configs[potentiometer].mux_pin;
    toggle_analog_mux(mux, mux_pin);

    // Flush adc channel. Then scan adc values and retrieve index based on current mux
    mux == MUX_BOARD_1 ? adc_read_single(0) : adc_read_single(1);
    BUSY_WAIT_ms(20);
    uint16_t adc_value = mux == MUX_BOARD_1 ? adc_read_single(0) : adc_read_single(1);

    uint16_t angle = convert_adc_value_to_angle(adc_value);
    return angle;
#endif

#if defined ARM_SLEEVE
    uint8_t mux_pin = potentiometer_configs[potentiometer].mux_pin;
    toggle_analog_mux(mux_pin);

    // Flush adc channel. Then scan adc value
    adc_read_single(0);
    BUSY_WAIT_ms(20);
    uint16_t adc_value = adc_read_single(0);

    uint16_t angle = convert_adc_value_to_angle(adc_value);
    return angle;
#endif
}

void read_all_potentiometers(uint16_t *angles)
{
    for (uint8_t i = 0; i < (uint8_t)NUM_OF_POTENTIOMETERS; i++) {
        angles[i] = potentiometer_read((potentiometer_e)i);
    }
}
