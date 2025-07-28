#include "uart.h"
#include "../common/assert_handler.h"
#include "../common/defines.h"
#include "../common/ring_buffer.h"
#include <stm32f4xx.h>

int count = 0;

#define UART_BUFFER_SIZE (17)
static uint8_t buffer[UART_BUFFER_SIZE];
static struct ring_buffer tx_buffer = { .buffer = buffer, .size = sizeof(buffer) };

static inline void uart_tx_clear_interrupt(void)
{
    // STM32 doesn't need to clear interrupt
}

static inline void uart_tx_enable_interrupt(void)
{
    USART1->CR1 |= USART_CR1_TXEIE;
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

void usart1_handler(void)
{
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

#define PCLK1 42000000U
#define PCLK2 84000000U
#define BAUD 115200U

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

void _putchar(char c)
{
    count++;

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
