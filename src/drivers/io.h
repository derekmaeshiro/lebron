#ifndef IO_H
#define IO_H
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    // ---- Port A ----
    IO_PA0, // PA0: A0, ADC1_IN0, TIM2_CH1, TIM5_CH1
    IO_PA1, // PA1: A1, ADC1_IN1, TIM2_CH2, TIM5_CH2
    IO_PA2, // PA2: D1, USART2_TX, TIM2_CH3, ADC1_IN2
    IO_PA3, // PA3: D0, USART2_RX, TIM2_CH4, ADC1_IN3
    IO_PA4, // PA4: A2, ADC1_IN4, SPI1_NSS, DAC1_OUT1
    IO_PA5, // PA5: D13, SPI1_SCK, ADC1_IN5, DAC1_OUT2, onboard LED
    IO_PA6, // PA6: D12, SPI1_MISO, ADC1_IN6, TIM3_CH1
    IO_PA7, // PA7: D11, SPI1_MOSI, ADC1_IN7, TIM3_CH2
    IO_PA8, // PA8: D7, TIM1_CH1, MCO1
    IO_PA9, // PA9: D8, USART1_TX, TIM1_CH2
    IO_PA10, // PA10: D2, USART1_RX, TIM1_CH3
    IO_PA11, // PA11: USB_DM, CAN1_RX, TIM1_CH4
    IO_PA12, // PA12: USB_DP, CAN1_TX, TIM1_ETR
    IO_PA13, // PA13: SWDIO (debug only, not for normal GPIO)
    IO_PA14, // PA14: SWCLK (debug only, not for normal GPIO)
    IO_PA15, // PA15: JTDI, SPI1_NSS

    // ---- Port B ----
    IO_PB0, // PB0: A3, ADC1_IN8, TIM3_CH3
    IO_PB1, // PB1: A4, ADC1_IN9, TIM3_CH4
    IO_PB2, // PB2: BOOT1
    IO_PB3, // PB3: D3, SPI1_SCK, JTDO, TIM2_CH2
    IO_PB4, // PB4: D5, SPI1_MISO, NJTRST, TIM3_CH1
    IO_PB5, // PB5: D4, CAN2_RX, TIM3_CH2, I2C1_SMBA
    IO_PB6, // PB6: D10, I2C1_SCL, USART1_TX, TIM4_CH1
    IO_PB7, // PB7: D9, I2C1_SDA, USART1_RX, TIM4_CH2
    IO_PB8, // PB8: D15, I2C1_SCL, TIM4_CH3
    IO_PB9, // PB9: D14, I2C1_SDA, TIM4_CH4
    IO_PB10, // PB10: D6, I2C2_SCL, USART3_TX, TIM2_CH3
    IO_PB11, // PB11: USART3_RX, I2C2_SDA, TIM2_CH4
    IO_PB12, // PB12: D5, SPI2_NSS, I2C2_SMBALERT, UART5_RX
    IO_PB13, // PB13: D3, SPI2_SCK, TIM1_CH1N
    IO_PB14, // PB14: SPI2_MISO, TIM1_CH2N
    IO_PB15, // PB15: SPI2_MOSI, RTC_REFIN

    // ---- Port C ----
    IO_PC0, // PC0: A5, ADC1_IN10
    IO_PC1, // PC1: A4, ADC1_IN11
    IO_PC2, // PC2: ADC1_IN12, SPI2_MISO
    IO_PC3, // PC3: ADC1_IN13, SPI2_MOSI
    IO_PC4, // PC4: ADC1_IN14, I2C2_SDA
    IO_PC5, // PC5: ADC1_IN15, I2C2_SMBA
    IO_PC6, // PC6: TIM3_CH1, USART6_TX
    IO_PC7, // PC7: TIM3_CH2, USART6_RX
    IO_PC8, // PC8: TIM3_CH3
    IO_PC9, // PC9: TIM3_CH4, I2C3_SDA
    IO_PC10, // PC10: UART4_TX, USART3_TX
    IO_PC11, // PC11: UART4_RX, USART3_RX
    IO_PC12, // PC12: UART5_TX, USART3_CK
    IO_PC13, // PC13: USER_BUTTON (blue button), RTC_AF1
    IO_PC14, // PC14: OSC32_IN (for LSE crystal, not header)
    IO_PC15, // PC15: OSC32_OUT (for LSE crystal, not header)

    // ---- Port D ----
    IO_PD2, // PD2: SDIO_CMD

    IO_PIN_MAX // Marker
} io_pin_e;

typedef enum {
#if defined(ARM_SLEEVE) // STM32F411 (Black Pill)
    IO_I2C_SCL = IO_PB8, // IMU sensor drivers
    IO_I2C_SDA = IO_PB9, // IMU sensor drivers
    IO_UART_RXD = IO_PA10, // Bluetooth Communication
    IO_UART_TXD = IO_PA9, // Bluetooth Communication
    IO_TEST_LED = IO_PC13, // Testing Blinky / Debugging
    IO_ANALOG_MUX_S0 = IO_PB4, // Analog Mux Bit Configuration
    IO_ANALOG_MUX_S1 = IO_PB5, // Analog Mux Bit Configuration
    IO_ANALOG_MUX_S2 = IO_PB3, // Analog Mux Bit Configuration
    IO_ANALOG_MUX_S3 = IO_PA15, // Analog Mux Bit Configuration
    IO_ANALOG_MUX_ENABLE_1 = IO_PB12, // Analog Mux Enable Pin
    IO_ANALOG_MUX_ENABLE_2 = IO_PB13, // Analog Mux Enable Pin
    IO_ANALOG_MUX_COM_1 = IO_PA0, // Analog Mux Output Pin
    IO_ANALOG_MUX_COM_2 = IO_PA1, // Analog Mux Output Pin

#elif defined(ROBOTIC_ARM) // STM32F446RE
    IO_I2C_SCL = IO_PB8, // Servo drivers
    IO_I2C_SDA = IO_PB9, // Servo drivers
    IO_UART_RXD = IO_PA10, // UART
    IO_UART_TXD = IO_PA9, // UART
    IO_TEST_LED = IO_PA5, // Testing Blinky / Debugging
    IO_ANALOG_MUX_S0 = IO_PB4, // Analog Mux Bit Configuration
    IO_ANALOG_MUX_S1 = IO_PB5, // Analog Mux Bit Configuration
    IO_ANALOG_MUX_S2 = IO_PB3, // Analog Mux Bit Configuration
    IO_ANALOG_MUX_S3 = IO_PA15, // Analog Mux Bit Configuration
    IO_ANALOG_MUX_ENABLE_1 = IO_PB12, // Analog Mux Enable Pin
    IO_ANALOG_MUX_ENABLE_2 = IO_PB13, // Analog Mux Enable Pin
    IO_ANALOG_MUX_COM_1 = IO_PA0, // Analog Mux Output Pin
    IO_ANALOG_MUX_COM_2 = IO_PA1, // Analog Mux Output Pin
    IO_PWM_DISTAL_INTERPHALANGEAL_JOINT = IO_PA6, // DIP (top) (orientation #1)
    IO_PWM_PROXIMAL_INTERPHALANGEAL_JOINT = IO_PA7, // PIP (middle) (orientation #1)
    IO_PWM_METACARPOPHALANGEAL_JOINT_1 = IO_PC6, // MCP (knuckles) (orientation #1)
    IO_PWM_METACARPOPHALANGEAL_JOINT_2 = IO_PB10, // MCP (knuckles) (orientation #2)
#endif
} io_e;

/* Select Registers (MODER) and (AFR[0], AFR[1]) */
/* Final Selection */
typedef enum {
    IO_SELECT_INPUT,
    IO_SELECT_OUTPUT,
    IO_SELECT_ALT,
    IO_SELECT_ANALOG,
} io_select_e;

typedef enum {
    IO_RESISTOR_DISABLED,
    IO_PULL_UP_ENABLED,
    IO_PULL_DOWN_ENABLED,
    IO_RESERVED,
} io_resistor_e;

typedef enum {
    IO_IN_LOW,
    IO_IN_HIGH,
} io_in_e;

typedef enum {
    IO_ALT_FUNCTION_0,
    IO_ALT_FUNCTION_1,
    IO_ALT_FUNCTION_2,
    IO_ALT_FUNCTION_3,
    IO_ALT_FUNCTION_4,
    IO_ALT_FUNCTION_5,
    IO_ALT_FUNCTION_6,
    IO_ALT_FUNCTION_7,
    IO_ALT_FUNCTION_8,
    IO_ALT_FUNCTION_9,
    IO_ALT_FUNCTION_10,
    IO_ALT_FUNCTION_11,
    IO_ALT_FUNCTION_12,
    IO_ALT_FUNCTION_13,
    IO_ALT_FUNCTION_14,
    IO_ALT_FUNCTION_15,
} io_alt_function_e;

typedef enum {
    IO_OUT_LOW,
    IO_OUT_HIGH,
} io_out_e;

typedef enum {
    IO_TRIGGER_RISING, // 0 -> 1
    IO_TRIGGER_FALLING, // 1 -> 0
    IO_TRIGGER_BOTH_EDGES, // 0 -> 1 and 1 -> 0
} io_trigger_e;

struct io_config
{
    bool used;
    io_select_e select;
    io_alt_function_e io_alt_function;
    io_resistor_e resistor;
    io_out_e out;
};

// TODO: functions
void io_init(void);
void io_configure(io_e io, const struct io_config *config);
void io_get_current_config(io_e io, struct io_config *current_config);
bool io_config_compare(const struct io_config *cfg1, const struct io_config *cfg2);
void io_set_select(io_e io, io_select_e select, io_alt_function_e alt_function);
void io_set_resistor(io_e io, io_resistor_e resistor); // only applicable if pin is set to input
void io_set_out(io_e io, io_out_e out);

// ADC functions
const io_e *io_adc_pins(uint8_t *cnt);
bool io_supports_adc(io_e pin);
uint8_t io_to_adc_idx(io_e io);

io_in_e io_get_input(io_e io);

typedef void (*isr_function)(void);
void io_configure_interrupt(io_e io, io_trigger_e trigger, isr_function isr);
void io_deconfigure_interrupt(io_e io);
void io_enable_interrupt(io_e io);
void io_disable_interrupt(io_e io);

#endif // IO_H