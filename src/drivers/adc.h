#include <stdint.h>
#ifndef ADC_H
#define ADC_H

#define ADC_CHANNEL_COUNT (16u)

// Channel values are 12-bit integers
typedef uint16_t adc_channel_values_t[ADC_CHANNEL_COUNT];

// Add additional adc pin configurations if necessary
extern struct io_config default_adc_pin_config;

void adc_init(void);
void adc_get_channel_values(adc_channel_values_t values);

#endif
