#include "adc.h"
#include "io.h"
#include "mux.h"
#include <stdbool.h>

#if defined(ROBOTIC_ARM)
static struct mux_config mux_configs[] = {
    [MUX_BOARD_1] = {
        .adc_pin = IO_ANALOG_MUX_COM_1,
    },
    [MUX_BOARD_2] = {
        .adc_pin = IO_ANALOG_MUX_COM_2,
    },
};
#endif

bool initialized = false;
void mux_init(void){
    ASSERT(!initialized);
    initialized = true;
}
uint16_t get_adc_value_from_mux(mux_e mux, mux_pin_e mux_pin){
    // TODO
}