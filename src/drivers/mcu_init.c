#include "mcu_init.h"
#include "io.h"
#include <stm32f4xx.h>

void mcu_init(void)
{
    // watchdog is already off by default
    io_init();

    // Globally enable interrupts
    __enable_irq();
}