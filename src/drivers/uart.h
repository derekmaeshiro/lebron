#ifndef UART_H
#define UART_H

#include <stdbool.h>

void uart_init(void);
void uart_putchar_polling(char c);
void _putchar(char c);
void uart_print_interrupt(const char *string);

#endif // UART_H