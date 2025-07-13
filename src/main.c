#include "../include/stm32f4xx.h"
// #include "../include/stm32f446xx.h" // Include the device-specific header
#include "led.h"

#define LED_PIN 5

int main(void)
{
    // Enable the GPIOB clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable clock for GPIO port B
    // Set pin PB0 as output
    GPIOA->MODER &= ~(3U << (LED_PIN * 2)); // Clear mode bits for PB0
    led_init();
    // GPIOA->MODER |= (1U << (LED_PIN * 2));

    while (1) {
        led_toggle();
        // GPIOA->ODR ^= (1U << LED_PIN);
        for (volatile int i = 0; i < 100000; i++)
            ; // Busy wait
    }
}