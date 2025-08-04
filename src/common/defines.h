#ifndef DEFINES_H
#define DEFINES_H

#include <stdint.h>
#include <stm32f4xx.h> // for __NOP()

#define PCLK1 45000000U
#define PCLK2 84000000U
#define BAUD 115200U
#define I2C_SPEED 100000U // in kHz

#define SYSTEM_CORE_CLOCK 168000000UL
#define UNUSED(x) (void)(x)
#define SUPPRESS_UNUSED __attribute__((unused))
#define ARRAY_SIZE(array) (sizeof(array) / sizeof(array[0]))

#define BUSY_WAIT_ms(ms)                                                                           \
    do {                                                                                           \
        volatile uint32_t _n = (SYSTEM_CORE_CLOCK / 7000) * (ms);                                  \
        while (_n--)                                                                               \
            __NOP();                                                                               \
    } while (0)

#endif // DEFINES_H