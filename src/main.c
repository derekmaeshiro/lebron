#include "common/assert_handler.h"
#include "common/defines.h"
#include <stm32f4xx.h>

void delay(volatile uint32_t count)
{
    while (count--)
        ;
}

SUPPRESS_UNUSED
static void test_pc_13(void)
{
    // Enable the clock for GPIOC
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    // Clear mode for PC13 and set it to output mode
    GPIOC->MODER &= ~(0x3 << (13 * 2)); // Clear mode for PC13
    GPIOC->MODER |= (0x01 << (13 * 2)); // Set to output mode (01)

    while (1) {
        // Toggle the LED
        GPIOC->ODR ^= (1 << 13); // Toggle PC13
        delay(1000000); // Call a simple delay
    }
}

int main(void)
{
    test_pc_13();
    ASSERT(0);
    return 0;
}