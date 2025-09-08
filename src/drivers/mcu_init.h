#ifndef MCU_INIT_H
#define MCU_INIT_H

#include <stdint.h>

/* Initialization of common microcontroller functionality that affects all peripherals.
 * (e.g. watchdog, clocks, pins) */

void mcu_init(void);
uint32_t get_ms(void);

#endif // MCU_INIT