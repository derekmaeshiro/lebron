#include <stdint.h>
#include <stdbool.h>
#include <stm32f4xx.h>

#include "adc.h"
#include "io.h"
#include "led.h"
#include "../common/assert_handler.h"
#include "../common/trace.h"

static const io_e *adc_pins;
// Update based on number of pins being used
static uint8_t adc_pin_cnt = 1;

static volatile adc_channel_values_t adc_dtc_block;
static volatile adc_channel_values_t adc_dtc_block_cache;

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
    ADC1->CR1 = (adc_pin_cnt > 1) ? ADC_CR1_SCAN : 0;
    ADC1->CR2 = 0;

    // // For continuous conversion mode
    // ADC1-> CR2 = ADC_CR2_CONT;

    ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_DDS;

    // Maximum sample time for all channels
    ADC1->SMPR2 = 0xFFFFFFFF;

    // Max sample time for all channels
    ADC1->SMPR1 = 0x3FFFFFFF;
    ADC1->SMPR2 = 0x3FFFFFFF;

    // Sequence length
    ADC1->SQR1 = ((adc_pin_cnt - 1) << 20);

    initialized = true;
}

void adc_get_channel_values(adc_channel_values_t values)
{
    __disable_irq(); // Disable global interrupts

    if (!initialized)
        return;

    for (uint8_t i = 0; i < adc_pin_cnt; i++) {
        uint8_t channel_idx = io_to_adc_idx(adc_pins[i]);

        // Select this channel in SQR3 (1 conversion in the sequence)
        ADC1->SQR3 = channel_idx;

        // Enable ADC
        ADC1->CR2 |= ADC_CR2_ADON;

        // Start conversion
        ADC1->CR2 |= ADC_CR2_SWSTART;

        // Wait for end of conversion
        while (!(ADC1->SR & ADC_SR_EOC))
            ;

        // Read the result
        values[channel_idx] = ADC1->DR;
    }

    __enable_irq(); // Re-enable interrupts
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