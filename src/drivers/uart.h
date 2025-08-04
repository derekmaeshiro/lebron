#ifndef UART_H
#define UART_H

#include <stdbool.h>

void uart_init(void);
void uart_putchar_polling(char c);
void uart_print_polling(const char *string);
void _putchar(char c);
void uart_print_interrupt(const char *string);

// These functions should ONLY be called by assert_handler
void uart_init_assert(void);
void uart_trace_assert(const char *string);

#endif // UART_H