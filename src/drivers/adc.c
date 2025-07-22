#include <stdint.h>
#include <stdbool.h>

#include "adc.h"
#include "io.h"

static uint16_t *adc_pins;

// Update based on number of pins being used
static uint16_t adc_pin_cnt = 1;

static const struct io_config default_adc_pin_config = {
    .select = IO_SELECT_ANALOG,
    .io_alt_function = IO_ALT_FUNCTION_0,
    .resistor = IO_PULL_DOWN_ENABLED,
    .out = IO_OUT_LOW
};

static bool initialized = false;
void adc_init(void){
    // Uncomment when assert is complete
    // Ensure ADC is initialized only once
    // ASSERT(initialized);

    // Retrieve list of ADC pins in io.c
    adc_pins = io_adc_pins(&adc_pin_cnt);

     // Set IO config on each pin
    for (uint8_t i = 0; i < adc_pin_cnt; i++) {
        io_e pin = adc_pins[i];
        io_configure(pin, &default_adc_pin_config);
    }

     // Enable ADC1 clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // CONTROL REGISTER 2
    // For single conversion mode
    ADC1->CR2 = 0;
    // // For continuous conversion mode
    // ADC1->CR2 |= ADC_CR2_CONT;

    // CONTROL REGISTER 1
    ADC1->CR1 = 0; // 12-bit resolution default

    // Maximum sample time for all channels
    ADC1->SMPR2 = 0xFFFFFFFF;

    // Enable ADC1
    ADC1->CR2 |= ADC_CR2_ADON;

    initialized = true;
}

/*
    Helper function to reset ADC
    Use when changing conversion modes or running into problems
*/
void adc_reset(ADC_TypeDef *adc) {
    // 1. Disable ADC (turn it off)
    adc->CR2 &= ~ADC_CR2_ADON;

    // 2. Wait for any ongoing conversion to finish
    // Optionally, wait until ADON bit clears (some delay might be needed)
    while (adc->CR2 & ADC_CR2_ADON) {
        // Wait for ADC to power down
    }

    // 3. Clear status flags (EOC, OVR, etc.)
    adc->SR = 0;

    // 4. Reset control registers to default state (optional)
    adc->CR1 = 0;
    adc->CR2 = 0;
    adc->SMPR1 = 0;
    adc->SMPR2 = 0;
}
