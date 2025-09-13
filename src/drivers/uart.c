#include "uart.h"
#include "potentiometer.h"
#include "../common/assert_handler.h"
#include "../common/defines.h"
#include "../common/ring_buffer.h"
#include "../common/trace.h"
#include <stdint.h>
#include <stdio.h>
#include <stm32f4xx.h>

#define UART_TX_BUFFER_SIZE (17)
#define UART_RX_BUFFER_SIZE (NUM_OF_JOINTS)
uint8_t tx_buffer_data[UART_TX_BUFFER_SIZE];
uint8_t rx_buffer_data[UART_RX_BUFFER_SIZE];

struct ring_buffer tx_buffer = { .buffer = tx_buffer_data, .size = sizeof(tx_buffer_data) };
struct ring_buffer rx_buffer = { .buffer = rx_buffer_data, .size = sizeof(rx_buffer_data) };

static inline void uart_tx_clear_interrupt(void)
{
    // STM32 doesn't need to clear interrupt
}

static inline void uart_tx_enable_interrupt(void)
{
    USART1->CR1 |= USART_CR1_TXEIE;
    USART1->CR1 |= USART_CR1_RXNEIE;
}

static inline void uart_tx_disable_interrupt(void)
{
    USART1->CR1 &= ~USART_CR1_TXEIE;
}

static void uart_tx_start(void)
{
    if (!ring_buffer_empty(&tx_buffer)) {
        USART1->DR = (uint8_t)ring_buffer_peek(&tx_buffer);
    }
}

static void uart_rx_start(void)
{
    if (USART1->SR & USART_SR_RXNE) {
        uint8_t data = USART1->DR;
        ring_buffer_put(&rx_buffer, data);
    }
}

void usart1_handler(void)
{
    /* RX Implementation */
    uart_rx_start();

    if (ring_buffer_empty(&tx_buffer)) {
        uart_tx_disable_interrupt(); // Instead of while(1)
        return;
    }

    ring_buffer_get(&tx_buffer); // Remove byte that was just sent

    // Optionally clear any interrupt flags if needed (STM32 doesn't require it)

    if (!ring_buffer_empty(&tx_buffer)) {
        uart_tx_start(); // Send next byte (your function)
    } else {
        uart_tx_disable_interrupt(); // Done transmitting
    }
}

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t pclk, uint32_t baud)
{
    // For OVER8=0 (default)
    uint32_t usartdiv_x16 = (pclk + (baud / 2)) / baud;
    uint32_t mantissa = usartdiv_x16 / 16;
    uint32_t fraction = usartdiv_x16 % 16;

    USARTx->BRR = (mantissa << 4) | (fraction & 0xF);
}

static void uart_configure(void)
{
    // 1. Enable the UART Peripheral Clock
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // 2. Configure word length and parity (8N1)
    USART1->CR1 &= ~USART_CR1_M; // 8 data bits
    USART1->CR1 &= ~USART_CR1_PCE; // No parity

    // 3. Configure number of stop bits (1 stop bit)
    USART1->CR2 &= ~USART_CR2_STOP; // 1 stop bit

    // 4. Set baud rate
    uart_set_baudrate(USART1, PCLK2, BAUD);

    // 5. (OPTIONAL) Enable DMA transmitter (remove if not used)
    // USART1->CR3 |= USART_CR3_DMAT;

    // 6. Enable Transmitter/Receiver
    USART1->CR1 |= USART_CR1_TE;
    USART1->CR1 |= USART_CR1_RE;

    // 7. Enable USART (last)
    USART1->CR1 |= USART_CR1_UE;

    // 8. Enable NVIC interrupt for USART1
    NVIC_SetPriority(USART1_IRQn, 1);
    NVIC_EnableIRQ(USART1_IRQn);
}

static bool initialized = false;

/* For UART, we'll be following a 10 bit configuration that consists of 1 start bit, 8 data bits, no
 * parity bit, and 1 stop bit. */

void uart_init(void)
{
    ASSERT(!initialized);
    uart_configure();

    // Interrupts triggers when TX buffer is empty, which it is after boot, so clear it here.
    uart_tx_clear_interrupt();

    // Enable TX interrupt
    uart_tx_enable_interrupt();
}

void uart_putchar_polling(char c)
{
    // Wait until the transmit data register is empty
    while (!(USART1->SR & USART_SR_TXE))
        ;
    USART1->DR = (uint8_t)c;

    // If LF is sent, send CR before it for CRLF (Windows-style line endings)
    if (c == '\n') {
        uart_putchar_polling('\r'); // CR before LF
    }
}

void uart_print_polling(const char *string)
{
    int i = 0;
    while (string[i] != '\0') {
        uart_putchar_polling(string[i]);
        i++;
    }
}

void _putchar(char c)
{
    while (ring_buffer_full(&tx_buffer)) { }

    uart_tx_disable_interrupt();
    const bool tx_ongoing = !ring_buffer_empty(&tx_buffer);
    ring_buffer_put(&tx_buffer, c);

    if (!tx_ongoing) {
        uart_tx_start();
    }

    uart_tx_enable_interrupt();

    if (c == '\n') {
        _putchar('\r');
    }
}

void uart_print_interrupt(const char *string)
{
    int i = 0;
    while (string[i] != '\0') {
        _putchar(string[i]);
        i++;
    }
}

static uint8_t utoa_uint(char *buf, uint32_t value)
{
    char tmp[10];
    uint8_t i = 0;
    do {
        tmp[i++] = (value % 10) + '0';
        value /= 10;
    } while (value > 0);

    // reverse into buf
    for (uint8_t j = 0; j < i; j++) {
        buf[j] = tmp[i - j - 1];
    }
    return i; // number of chars written
}

static uint32_t atou_uint(const char *str, uint8_t *pos)
{
    uint32_t value = 0;
    while (str[*pos] >= '0' && str[*pos] <= '9') {
        value = value * 10 + (str[*pos] - '0');
        (*pos)++;
    }
    return value;
}

void serialize_potentiometer_reading(const struct potentiometer_reading *reading,
                                     char *potentiometer_buffer, uint8_t potentiometer_buffer_size)
{
    uint8_t pos = 0;

    // board
    pos += utoa_uint(potentiometer_buffer + pos, reading->potentiometer_board);

    // comma
    if (pos < potentiometer_buffer_size) {
        potentiometer_buffer[pos++] = ',';
    }

    // angle
    pos += utoa_uint(potentiometer_buffer + pos, reading->angle);

    // newline
    if (pos < potentiometer_buffer_size) {
        potentiometer_buffer[pos++] = '\n';
    }

    // null terminator
    if (pos < potentiometer_buffer_size) {
        potentiometer_buffer[pos] = '\0';
    }
}

void deserialize_potentiometer_reading(const char *data, struct potentiometer_reading *reading)
{
    uint8_t idx = 0;

    // Parse board
    uint32_t board_val = atou_uint(data, &idx);

    // Expect comma
    if (data[idx] == ',') {
        idx++;
    } else {
        reading->potentiometer_board = 0; // default
        reading->angle = 0;
        return;
    }

    // Parse angle
    uint32_t angle_val = atou_uint(data, &idx);

    // Assign values (with simple bounds check)
    reading->potentiometer_board = (uint8_t)board_val;
    reading->angle = (uint16_t)angle_val;
}

void uart_send_potentiometer_readings(const struct potentiometer_reading *readings,
                                      uint8_t reading_count)
{
    //(void)readings;
    uint8_t MAX_READING_BUFFER_SIZE = 64;
    for (uint8_t i = 0; i < reading_count; i++) {
        char readings_buffer[MAX_READING_BUFFER_SIZE];
        serialize_potentiometer_reading(&readings[i], readings_buffer, MAX_READING_BUFFER_SIZE);
        uart_print_interrupt(readings_buffer);
    }
}

void uart_init_assert(void)
{
    uart_tx_disable_interrupt();
    uart_configure();
}

void uart_trace_assert(const char *string)
{
    int i = 0;
    while (string[i] != '\0') {
        uart_putchar_polling(string[i]);
        i++;
    }
}
