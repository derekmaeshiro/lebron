#include "adc.h"
#include "analog_mux.h"
#include "../common/assert_handler.h"
#include "../common/defines.h"
#include "../common/trace.h"
#include <stdbool.h>

// #if defined ROBOTIC_ARM
static bool initialized = false;
void analog_mux_init(void)
{
    ASSERT(!initialized);
    initialized = true;
}

void toggle_analog_mux(analog_mux_e mux, uint8_t mux_pin)
{
    if (mux == MUX_BOARD_1) {
        if (mux_pin & 0x01) {
            io_set_out(IO_ANALOG_MUX_1_S0, IO_OUT_HIGH);
        } else {
            io_set_out(IO_ANALOG_MUX_1_S0, IO_OUT_LOW);
        }
        if (mux_pin & 0x02) {
            io_set_out(IO_ANALOG_MUX_1_S1, IO_OUT_HIGH);
        } else {
            io_set_out(IO_ANALOG_MUX_1_S1, IO_OUT_LOW);
        }
        if (mux_pin & 0x04) {
            io_set_out(IO_ANALOG_MUX_1_S2, IO_OUT_HIGH);
        } else {
            io_set_out(IO_ANALOG_MUX_1_S2, IO_OUT_LOW);
        }
        if (mux_pin & 0x08) {
            io_set_out(IO_ANALOG_MUX_1_S3, IO_OUT_HIGH);
        } else {
            io_set_out(IO_ANALOG_MUX_1_S3, IO_OUT_LOW);
        }
    } else if (mux == MUX_BOARD_2) {
        if (mux_pin & 0x01) {
            io_set_out(IO_ANALOG_MUX_2_S0, IO_OUT_HIGH);
        } else {
            io_set_out(IO_ANALOG_MUX_2_S0, IO_OUT_LOW);
        }
        if (mux_pin & 0x02) {
            io_set_out(IO_ANALOG_MUX_2_S1, IO_OUT_HIGH);
        } else {
            io_set_out(IO_ANALOG_MUX_2_S1, IO_OUT_LOW);
        }
        if (mux_pin & 0x04) {
            io_set_out(IO_ANALOG_MUX_2_S2, IO_OUT_HIGH);
        } else {
            io_set_out(IO_ANALOG_MUX_2_S2, IO_OUT_LOW);
        }
        if (mux_pin & 0x08) {
            io_set_out(IO_ANALOG_MUX_2_S3, IO_OUT_HIGH);
        } else {
            io_set_out(IO_ANALOG_MUX_2_S3, IO_OUT_LOW);
        }
    }
    BUSY_WAIT_ms(20);
}
// #endif