#include <stm32f4xx.h>
#include "led.h"
#include "io.h"
#include "../common/defines.h"
#include <stdbool.h>
#include "../common/assert_handler.h"

static const struct io_config led_config = {
    .used = true,
    .select = IO_SELECT_OUTPUT,
    .io_alt_function = IO_ALT_FUNCTION_0,
    .resistor = IO_RESISTOR_DISABLED,
    .out = IO_OUT_LOW,
};

static bool initialized = false;
void led_init(void)
{
    ASSERT(!initialized);
    struct io_config current_config;
    io_get_current_config(IO_TEST_LED, &current_config);
    ASSERT(io_config_compare(&led_config, &current_config));
    initialized = true;
}

void led_set(led_e led, led_state_e state)
{
    ASSERT(initialized);
    const io_out_e out = (state == LED_STATE_ON) ? IO_OUT_HIGH : IO_OUT_LOW;
    switch (led) {
    case LED_TEST:
        io_set_out(IO_TEST_LED, out);
        break;
    }
}
