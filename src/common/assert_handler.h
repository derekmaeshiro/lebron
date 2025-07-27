#ifndef ASSERT_HANDLER_H
#define ASSERT_HANDLER_H

#include <stdint.h>

// Assert implementation suitable for a microcontroller

#define ASSERT(expression)                                                                         \
    do {                                                                                           \
        if (!(expression)) {                                                                       \
            uint32_t pc;                                                                           \
            __asm volatile("mov %0, pc" : "=r"(pc));                                               \
            assert_handler(pc);                                                                    \
        }                                                                                          \
    } while (0)

#define ASSERT_INTERRUPT(expression)                                                               \
    do {                                                                                           \
        if (!(expression)) {                                                                       \
            while (1)                                                                              \
                ;                                                                                  \
        }                                                                                          \
    } while (0)

void assert_handler(uint32_t program_counter);

#endif // ASSERT_HANDLER_H
