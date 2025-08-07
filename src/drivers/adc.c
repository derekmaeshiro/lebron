#include <stdint.h>
#include <stdbool.h>
#include <stm32f4xx.h>

#include "adc.h"
#include "io.h"
#include "led.h"
#include "../common/assert_handler.h"
#include "../common/trace.h"

static const io_e *adc_pins = 0;
static uint8_t adc_pin_cnt = 0;

struct io_config default_adc_pin_config = { .select = IO_SELECT_ANALOG,
                                            .io_alt_function = IO_ALT_FUNCTION_0,
                                            .resistor = IO_PULL_DOWN_ENABLED,
                                            .out = IO_OUT_LOW };

static bool initialized = false;

void adc_init(void)
{
    ASSERT(!initialized);

    // Retrieve list of ADC pins in io.c
    adc_pins = io_adc_pins(&adc_pin_cnt);

    // Set IO config on each pin
    for (uint8_t i = 0; i < adc_pin_cnt; i++) {
        io_e pin = adc_pins[i];
        io_configure(pin, &default_adc_pin_config);
    }

    // Enable ADC1 clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // For single conversion mode
    ADC1->CR1 = ADC_CR1_SCAN;
    ADC1->CR2 = ADC_CR2_EOCS; // EOC after each conversion

    // // For continuous conversion mode
    // ADC1-> CR2 = ADC_CR2_CONT;
    // ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_DDS;

    // Max sample time for all channels
    ADC1->SMPR1 = 0x3FFFFFFF;
    ADC1->SMPR2 = 0x3FFFFFFF;

    // Fill conversion sequence starting from first conversion (SQR3 first slots)
    ADC1->SQR1 = ((adc_pin_cnt - 1) << 20);
    ADC1->SQR3 = 0;
    for(uint8_t i = 0; i < adc_pin_cnt; i++) {
        uint8_t ch = io_to_adc_idx(adc_pins[i]);
        ADC1->SQR3 |= (ch << (i * 5)); // 5 bits per channel in SQR3
    }

    initialized = true;
}

void adc_get_channel_values(adc_channel_values_t values)
{
    if (!initialized) return;

    __disable_irq();  // Disable global interrupts

    ADC1->CR2 |= ADC_CR2_ADON;     // Power on ADC
    ADC1->CR2 |= ADC_CR2_SWSTART;  // Start conversion sequence

    for (uint8_t i = 0; i < adc_pin_cnt; i++) {
        while (!(ADC1->SR & ADC_SR_EOC)); // Wait for each conversion
        uint8_t ch = io_to_adc_idx(adc_pins[i]);
        values[ch] = ADC1->DR; // Store by ADC channel index
    }

    __enable_irq();  // Re-enable interrupts
}

/*
    Helper function to reset ADC
    Use when changing conversion modes or running into problems
*/
void adc_reset(void)
{
    // 1. Disable ADC1
    ADC1->CR2 &= ~ADC_CR2_ADON;

    // 2. Wait for ADC1 to power down
    while (ADC1->CR2 & ADC_CR2_ADON) {
        // wait
    }

    // 3. Clear flags
    ADC1->SR = 0;

    // 4. Reset registers
    ADC1->CR1 = 0;
    ADC1->CR2 = 0;
    ADC1->SMPR1 = 0;
    ADC1->SMPR2 = 0;
}