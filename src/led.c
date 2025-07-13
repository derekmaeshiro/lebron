#include "../include/stm32f4xx.h"
// #include "../include/stm32f446xx.h"
#include "led.h"

#define LED_PIN 5

void led_init(void)
{
    GPIOA->MODER |= (1U << (LED_PIN * 2)); // Set PB0 to output mode (01)
}

void led_toggle(void)
{
    GPIOA->ODR ^= (1U << LED_PIN); // Toggle PB0
}