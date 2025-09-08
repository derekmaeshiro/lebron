#include "potentiometer.h"
#include <stdbool.h>
#include <stdint.h>

#ifndef UART_H
#define UART_H

extern struct ring_buffer rx_buffer;
extern struct ring_buffer tx_buffer;

void uart_init(void);
void uart_putchar_polling(char c);
void uart_print_polling(const char *string);
void _putchar(char c);
void uart_print_interrupt(const char *string);

struct potentiometer_reading
{
    joint_e potentiometer_board;
    uint16_t angle;
};

void uart_send_potentiometer_readings(const struct potentiometer_reading *readings, uint8_t count);
void deserialize_potentiometer_reading(const char *data, struct potentiometer_reading *reading);

// These functions should ONLY be called by assert_handler
void uart_init_assert(void);
void uart_trace_assert(const char *string);

#endif // UART_H