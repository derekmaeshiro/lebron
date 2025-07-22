#ifndef DEFINES_H
#define DEFINES_H

#include <stdint.h>

// In defines.h
#ifdef ROBOTIC_ARM
#define SYSTEM_CORE_CLOCK 180000000u
#elif defined(ARM_SLEEVE)
#define SYSTEM_CORE_CLOCK 100000000u
#else
#define SYSTEM_CORE_CLOCK 16000000u
#endif

#define UNUSED(x) (void)(x)
#define SUPPRESS_UNUSED __attribute__((unused))
#define ARRAY_SIZE(array) (sizeof(array) / sizeof(array[0]))

#define BUSY_WAIT_ms(ms)                                                                           \
    do {                                                                                           \
        volatile uint32_t _n = (SYSTEM_CORE_CLOCK / 100000) * (ms);                                \
        while (_n--)                                                                               \
            __NOP();                                                                               \
    } while (0)

#endif // DEFINES_H