#include "io.h"
#include "adc.h"

// Edit pins later for particular mux board
typedef enum{
    PIN1,
    PIN2,
} mux_pin_e;

struct mux_config{
    io_e adc_pin;
    mux_pin_e current_input_pin;
    // other config parameters
};

typedef enum{
    MUX_BOARD_1,
    MUX_BOARD_2,
} mux_e;

void mux_init(void);
void switch_mux_input(mux_e mux, mux_pin_e mux_pin);