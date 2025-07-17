#include <stm32f4xx.h>
// #include "../include/stm32f446xx.h"
#include "led.h"
// A5 for nucleo, C13 for black pill
#define LED_PIN 13

void led_init(void)
{
    GPIOC->MODER |= (1U << (LED_PIN * 2)); // Set PB0 to output mode (01)
}

void led_toggle(void)
{
    GPIOC->ODR ^= (1U << LED_PIN); // Toggle PB0
}