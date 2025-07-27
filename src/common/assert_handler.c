#include "assert_handler.h"
#include "defines.h"
#include "../drivers/uart.h"
#include "../external/printf/printf.h"

#include <stm32f4xx.h>

/* STM32 provides BKPT as an instruction to add breakpoints. The CPU will halt and the debugger will
 * catch the breakpoint in GDB/VSCode. */
#define BREAKPOINT __asm volatile("BKPT #0");

/* Use correct test led ports and pins */
#if defined(ROBOTIC_ARM)
#define TEST_LED_PORT GPIOA
#define TEST_LED_PIN 5
#elif defined(ARM_SLEEVE)
#define TEST_LED_PORT GPIOC
#define TEST_LED_PIN 13
#else
#error "Board type not defined"
#endif

// Text + Program Counter + Null Terminator
#define ASSERT_STRING_MAX_SIZE (15u + 6u + 1u)

static void assert_trace(uint32_t program_counter)
{
    // UART Tx (PA9)
    GPIOA->MODER &= ~(0b11 << (2 * 9));
    GPIOA->MODER |= (0b10 << (2 * 9));
    GPIOA->AFR[1] &= ~(0xF << (4 * 1));
    GPIOA->AFR[1] |= (0x7 << (4 * 1));

    uart_init_assert();
    char assert_string[ASSERT_STRING_MAX_SIZE];
    snprintf(assert_string, sizeof(assert_string), "ASSERT 0x%x\n", (unsigned long)program_counter);
    uart_trace_assert(assert_string);
}

static void assert_blink_led(void)
{
    // Configure pin as output
    TEST_LED_PORT->MODER &= ~(0x3u << (TEST_LED_PIN * 2)); // Clear mode bits
    TEST_LED_PORT->MODER |= (0x1u << (TEST_LED_PIN * 2)); // Set as output (binary 01)
    TEST_LED_PORT->PUPDR &= ~(0x3u << (TEST_LED_PIN * 2)); // No pull-up/pull-down
    TEST_LED_PORT->PUPDR |=
        (0x0u << (TEST_LED_PIN * 2)); // (Optional, explicitly sets to 00: no pull)
    TEST_LED_PORT->ODR &= ~(0x1u << TEST_LED_PIN); // Set output low

    while (1) {
        TEST_LED_PORT->ODR ^= (0x1u << TEST_LED_PIN); // Blink indefinitely
        BUSY_WAIT_ms(250); // Delay
    };
}

void assert_handler(uint32_t program_counter)
{
    // TODO: Turn off motors / servos ("safe state")

    /* STM32 makes breakpoints stop all running code because the code is run in debugger mode. As a
    result, only use BREAKPOINT when tracing back code. Otherwise, leave this commented out since
    the LED won't be able to blink with this line before it. */

    // BREAKPOINT
    assert_trace(program_counter);
    assert_blink_led();
}