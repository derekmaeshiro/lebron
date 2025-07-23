#include <stdint.h>
#include <stdbool.h>
#include <stm32f4xx.h>
#include <stm32f446xx.h>

#include "adc.h"
#include "io.h"

static const io_e *adc_pins;
// Update based on number of pins being used
static uint8_t adc_pin_cnt = 1;

static volatile adc_channel_values_t adc_dtc_block;
static volatile adc_channel_values_t adc_dtc_block_cache;

struct io_config default_adc_pin_config = {
    .select = IO_SELECT_ANALOG,
    .io_alt_function = IO_ALT_FUNCTION_0,
    .resistor = IO_PULL_DOWN_ENABLED,
    .out = IO_OUT_LOW
};

static bool initialized = false;

// Setup DMA stream to write ADC values into adc_dtc_block
static void adc_dma_init(void)
{
    // Enable DMA2 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // Disable stream before config
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream0->CR & DMA_SxCR_EN);

    // Clear interrupt flags for Stream 0
    DMA2->LIFCR |= DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 |
                   DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;

    // Peripheral address (ADC data register)
    DMA2_Stream0->PAR  = (uint32_t)&ADC1->DR;

    // Memory address (our buffer)
    DMA2_Stream0->M0AR = (uint32_t)adc_dtc_block;

    // Number of conversions
    DMA2_Stream0->NDTR = adc_pin_cnt;

    // DMA config
    DMA2_Stream0->CR =
        (0 << DMA_SxCR_CHSEL_Pos) |     // Channel 0
        DMA_SxCR_PL_1            |      // High priority
        DMA_SxCR_MINC            |      // Memory increment
        DMA_SxCR_MSIZE_0         |      // 16-bit memory
        DMA_SxCR_PSIZE_0         |      // 16-bit peripheral
        DMA_SxCR_TCIE            |      // Transfer complete interrupt
        DMA_SxCR_CIRC            |      // Circular mode
        DMA_SxCR_DIR_0;                 // Peripheral to memory

    // Enable DMA IRQ
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    // Enable stream
    DMA2_Stream0->CR |= DMA_SxCR_EN;
}

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

    adc_dma_init();

    // CONTROL REGISTER 2
    // For single conversion mode
    ADC1->CR2 = 0;
    // // For continuous conversion mode
    // ADC1->CR2 |= ADC_CR2_CONT;

    // CONTROL REGISTER 1
    ADC1->CR1 = 0; // 12-bit resolution default

    // Enable scan mode
    // ADC1->CR1 |= ADC_CR1_SCAN;

    // Enable DMA
    ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_DDS;

    // Maximum sample time for all channels
    ADC1->SMPR2 = 0xFFFFFFFF;

    // Configure channel sampling
    ADC1->SQR1 = ((adc_pin_cnt - 1) << 20);
    for (uint8_t i = 0; i < adc_pin_cnt; i++) {
        uint8_t ch = io_to_adc_idx(adc_pins[i]);
        if (i < 6)
            ADC1->SQR3 |= (ch << (5 * i));
        else if (i < 12)
            ADC1->SQR2 |= (ch << (5 * (i - 6)));
        else
            ADC1->SQR1 |= (ch << (5 * (i - 12)));
    }

    // Enable ADC1
    ADC1->CR2 |= ADC_CR2_ADON;

    // Start conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;

    initialized = true;
}

/*
    Nsumo video had a similar interrupt function
    TODO: Call function automatically when DMA scan finishes 
    This copies values from adc_dtc_block into a safe place (the cache)
*/
void DMA2_Stream0_IRQHandler(void)
{
    // Check if transfer complete flag is set
    if (DMA2->LISR & DMA_LISR_TCIF0) {
        // Clear the flag
        DMA2->LIFCR |= DMA_LIFCR_CTCIF0;

        // Copy the ADC results from the DMA buffer to the cache
        for (uint8_t i = 0; i < adc_pin_cnt; i++) {
            adc_dtc_block_cache[i] = adc_dtc_block[i];
        }

        // Optionally: restart conversion manually if you're not in circular mode
        // ADC1->CR2 |= ADC_CR2_SWSTART;
    }
}



void adc_get_channel_values(adc_channel_values_t values)
{
    // TODO: Include interrupts
    // __disable_irq();  // Disable global interrupts

    for (uint8_t i = 0; i < adc_pin_cnt; i++) {
        uint8_t channel_idx = io_to_adc_idx(adc_pins[i]);
        values[channel_idx] = adc_dtc_block_cache[i];
    }

    // __enable_irq();  // Re-enable interrupts
}

/*
    Helper function to reset ADC
    Use when changing conversion modes or running into problems
*/
void adc_reset(void) {
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

