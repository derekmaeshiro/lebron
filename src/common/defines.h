#ifndef DEFINES_H
#define DEFINES_H

#include <stdint.h>
#include <stm32f4xx.h> // for __NOP()

#ifdef ROBOTIC_ARM
#define PCLK1 42000000U // (Correct for 168MHz core, APB1/4)
#define PCLK2 84000000U // (Correct for 168MHz core, APB2/2)
#define SYSTEM_CORE_CLOCK 168000000UL
#define BAUD 9600U
#elif defined(ARM_SLEEVE)
#define PCLK1 50000000U
#define PCLK2 100000000U
#define SYSTEM_CORE_CLOCK 100000000UL
#define BAUD 9600U
#else
#error "No board defined! Define ROBOTIC_ARM or ARM_SLEEVE."
#endif

#define I2C_SPEED 100000U // in kHz
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