#include "io.h"
#include "../common/defines.h"
#include "../common/assert_handler.h"
#include "../common/trace.h"
#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <stddef.h>

/* Extracing pin and port using bitwise operations. STM32 uses 16 pins per port
 * Enum value can be seen as:
 * [ Zeros (10 bits) | Port (2 bits) | pin (4 bits) ] */
#define IO_PORT_OFFSET (4u)
#define IO_PORT_MASK (0x3u << IO_PORT_OFFSET)
#define IO_PIN_MASK (0xFu)
#define IO_PORT_CNT 4
#define IO_PIN_CNT_PER_PORT 16

static_assert(sizeof(io_pin_e) == 1);

/* MODER is 32 bits long for each port. AFR is 64 bits long, but is split into 2 groups: AFR[0] and
   AFR[1]. AFR[0] controls pins 0-7 and AFR[1] controls pins 8-15. Each pin gets 4 bits to choose
   its alt function (0 - 15). */

// Example: PA5
static inline uint8_t io_port(io_e io) // gives Port A (0) --> Port B (1)
{
    return (io & IO_PORT_MASK) >> IO_PORT_OFFSET;
}

static inline uint8_t io_pin_idx(io_e io) // gives 5 (0 - 15)
{
    return io & IO_PIN_MASK;
}

static inline uint8_t io_pin_bit(io_e io) // gives 100000
{
    return 1 << io_pin_idx(io);
}

/* STM32 peripherals are off by default to save power. Before you use a GPIO port,
 * you have to enable its clock via the RCC (Reset and Clock Control) registers. */
static const uint32_t gpio_port_clk_masks[] = {
    RCC_AHB1ENR_GPIOAEN, // 0: A
    RCC_AHB1ENR_GPIOBEN, // 1: B
    RCC_AHB1ENR_GPIOCEN, // 2: C
    RCC_AHB1ENR_GPIODEN, // 3: D
};

static void io_enable_clock(io_e io)
{
    const uint8_t port = io_port(io);
    const uint32_t gpio_port_clk_mask = gpio_port_clk_masks[port];

    RCC->AHB1ENR |= gpio_port_clk_mask; // Enable GPIOx clock
}

/* GPIO Type Def Array */
static GPIO_TypeDef *const gpio_ports[] = { GPIOA, GPIOB, GPIOC, GPIOD };

/* In the STM32 series, ISR functions (interrupt handlers) are handled on a basis of pins per port.
You can only have one interrupt configured for one pin in every port at a time. As a result, if you
configure PA0 as an interrupt, you can't configure PB0, PC0, or PD0 as interrupts. Furthermore,
interrupt handlers (what happens at an interrupt) are differentiated by the EXTI lines. EXTI0 to
EXTI4 can each hold 1 interrupt handler. However, EXTI5-9 and EXTI10-15, the last two handlers, each
only hold 1 interrupt handler. This means that the pins 5-9 and the pins 10-15 that are configured
as interrupts must each share the same interrupt handler. As a result, there are, at a maximum, 7
possible interrupt handlers and 16 possible configurable interrupt pins. */
static isr_function isr_functions[7] = { NULL };

/* In the STM32 series, ISR functions (interrupt handlers) are handled on a basis of pins per port.
You can only have one interrupt configured for one pin in every port at a time. As a result, if you
configure PA0 as an interrupt, you can't configure PB0, PC0, or PD0 as interrupts. Furthermore,
interrupt handlers (what happens at an interrupt) are differentiated by the EXTI lines. EXTI0 to
EXTI4 can each hold 1 interrupt handler. However, EXTI5-9 and EXTI10-15, the last two handlers, each
only hold 1 interrupt handler. This means that the pins 5-9 and the pins 10-15 that are configured
as interrupts must each share the same interrupt handler. As a result, there are, at a maximum, 7
possible interrupt handlers and 16 possible configurable interrupt pins. */

// TODO: This had an error. I commented for now.
// static isr_function isr_functions[7] = { NULL };

/* Macro for ADC pin configuration */
#define ADC_CONFIG { true, IO_SELECT_INPUT, IO_ALT_FUNCTION_0, IO_PULL_UP_ENABLED, IO_OUT_LOW }

/* This array holds the initial configs of all IO pins. */
/* Never: PA13, PA14 (debug) - don't touch */
static const struct io_config io_initial_configs[IO_PORT_CNT * IO_PIN_CNT_PER_PORT] = {
#if defined(ROBOTIC_ARM)
    // Application Pins
    [IO_TEST_LED] = { true, IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                      IO_OUT_LOW }, // PA5 OUTPUT
    [IO_I2C_SCL] = { true, IO_SELECT_ALT, IO_ALT_FUNCTION_4, IO_PULL_UP_ENABLED,
                     IO_OUT_HIGH }, // PB8 I2C1_SCL
    [IO_I2C_SDA] = { true, IO_SELECT_ALT, IO_ALT_FUNCTION_4, IO_PULL_UP_ENABLED,
                     IO_OUT_HIGH }, // PB9 I2C1_SDA
    [IO_UART_RXD] = { true, IO_SELECT_ALT, IO_ALT_FUNCTION_7, IO_RESISTOR_DISABLED,
                      IO_OUT_LOW }, // PA10 USART1_RX
    [IO_UART_TXD] = { true, IO_SELECT_ALT, IO_ALT_FUNCTION_7, IO_RESISTOR_DISABLED,
                      IO_OUT_LOW }, // PA9 USART1_TX
    [IO_ANALOG_MUX_1_S0] = { true, IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                             IO_OUT_LOW }, // PB4 Output etc
    [IO_ANALOG_MUX_1_S1] = { true, IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                             IO_OUT_LOW }, // PB5
    [IO_ANALOG_MUX_1_S2] = { true, IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                             IO_OUT_LOW }, // PB3
    [IO_ANALOG_MUX_1_S3] = { true, IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                             IO_OUT_LOW }, // PA15
    [IO_ANALOG_MUX_2_S0] = { true, IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                             IO_OUT_LOW }, // PB0
    [IO_ANALOG_MUX_2_S1] = { true, IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                             IO_OUT_LOW }, // PC1
    [IO_ANALOG_MUX_2_S2] = { true, IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                             IO_OUT_LOW }, // PA4
    [IO_ANALOG_MUX_2_S3] = { true, IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                             IO_OUT_LOW }, // PA8
    [IO_ANALOG_MUX_ENABLE_1] = { true, IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                                 IO_OUT_LOW }, // PB12
    [IO_ANALOG_MUX_ENABLE_2] = { true, IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                                 IO_OUT_LOW }, // PB13
    [IO_ANALOG_MUX_COM_1] = ADC_CONFIG, // PA0
    [IO_ANALOG_MUX_COM_2] = ADC_CONFIG, // PA1
    [IO_PWM_L_WRIST_NAE_NAE] = { true, IO_SELECT_ALT, IO_ALT_FUNCTION_2, IO_RESISTOR_DISABLED,
                                 IO_OUT_LOW }, // PA6 TIM?
    [IO_PWM_L_ELBOW] = { true, IO_SELECT_ALT, IO_ALT_FUNCTION_2, IO_RESISTOR_DISABLED,
                         IO_OUT_LOW }, // PA7
    [IO_PWM_L_BICEP] = { true, IO_SELECT_ALT, IO_ALT_FUNCTION_3, IO_RESISTOR_DISABLED,
                         IO_OUT_LOW }, // PC6?
    [IO_PWM_L_SHOULDER_FRONT_RAISE] = { true, IO_SELECT_ALT, IO_ALT_FUNCTION_1,
                                        IO_RESISTOR_DISABLED, IO_OUT_LOW }, // PB10
    [IO_PWM_L_SHOULDER_LAT_RAISE] = { true, IO_SELECT_ALT, IO_ALT_FUNCTION_2, IO_RESISTOR_DISABLED,
                                      IO_OUT_LOW }, // PB6
    [IO_PWM_R_WRIST_NAE_NAE] = { true, IO_SELECT_ALT, IO_ALT_FUNCTION_2, IO_RESISTOR_DISABLED,
                                 IO_OUT_LOW }, // PB7, TIM4_CH2, AF2
    [IO_PWM_R_ELBOW] = { true, IO_SELECT_ALT, IO_ALT_FUNCTION_2, IO_RESISTOR_DISABLED,
                         IO_OUT_LOW }, // PC7, TIM3_CH2, AF2
    [IO_PWM_R_BICEP] = { true, IO_SELECT_ALT, IO_ALT_FUNCTION_1, IO_RESISTOR_DISABLED,
                         IO_OUT_LOW }, // PA2, TIM2_CH2, AF1
    [IO_PWM_R_SHOULDER_FRONT_RAISE] = { true, IO_SELECT_ALT, IO_ALT_FUNCTION_2,
                                        IO_RESISTOR_DISABLED, IO_OUT_LOW }, // PB1, TIM3_CH4, AF2
    [IO_PWM_R_SHOULDER_LAT_RAISE] = { true, IO_SELECT_ALT, IO_ALT_FUNCTION_1, IO_RESISTOR_DISABLED,
                                      IO_OUT_LOW }, // PA3, TIM2_CH4, AF1

    // Board-specific
    [IO_PC13] = { true, IO_SELECT_INPUT, IO_ALT_FUNCTION_0, IO_PULL_UP_ENABLED,
                  IO_OUT_LOW }, // User Button
    [IO_PC14] = { true, IO_SELECT_ANALOG, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                  IO_OUT_LOW }, // LSE Crystal
    [IO_PC15] = { true, IO_SELECT_ANALOG, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                  IO_OUT_LOW }, // LSE Crystal
#elif defined(ARM_SLEEVE)
    // Application Pins
    [IO_I2C_SCL] = { true, IO_SELECT_ALT, IO_ALT_FUNCTION_4, IO_PULL_UP_ENABLED,
                     IO_OUT_HIGH }, // PB8, I2C1_SCL
    [IO_I2C_SDA] = { true, IO_SELECT_ALT, IO_ALT_FUNCTION_4, IO_PULL_UP_ENABLED,
                     IO_OUT_HIGH }, // PB9, I2C1_SDA
    [IO_UART_TXD] = { true, IO_SELECT_ALT, IO_ALT_FUNCTION_7, IO_RESISTOR_DISABLED,
                      IO_OUT_LOW }, // PA9, USART1_TX
    [IO_UART_RXD] = { true, IO_SELECT_ALT, IO_ALT_FUNCTION_7, IO_RESISTOR_DISABLED,
                      IO_OUT_LOW }, // PA10, USART1_RX
    [IO_TEST_LED] = { true, IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                      IO_OUT_LOW }, // PC13, User LED
    [IO_ANALOG_MUX_1_S0] = { true, IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                             IO_OUT_LOW }, // PB4 Output etc
    [IO_ANALOG_MUX_1_S1] = { true, IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                             IO_OUT_LOW }, // PB5
    [IO_ANALOG_MUX_1_S2] = { true, IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                             IO_OUT_LOW }, // PB3
    [IO_ANALOG_MUX_1_S3] = { true, IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                             IO_OUT_LOW }, // PA15
    [IO_ANALOG_MUX_2_S0] = { true, IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                             IO_OUT_LOW }, // PB0
    [IO_ANALOG_MUX_2_S1] = { true, IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                             IO_OUT_LOW }, // PC1
    [IO_ANALOG_MUX_2_S2] = { true, IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                             IO_OUT_LOW }, // PA4
    [IO_ANALOG_MUX_2_S3] = { true, IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                             IO_OUT_LOW }, // PA8
    [IO_ANALOG_MUX_ENABLE_1] = { true, IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                                 IO_OUT_LOW }, // PB12
    [IO_ANALOG_MUX_ENABLE_2] = { true, IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                                 IO_OUT_LOW }, // PB13
    [IO_ANALOG_MUX_COM_1] = ADC_CONFIG, // PA0
    [IO_ANALOG_MUX_COM_2] = ADC_CONFIG, // PA1
#endif
};

// TODO: Assign pins here that will be reserved as ADC pins
// Temporarily assigned one

static const io_e io_adc_pins_arr[] = { IO_ANALOG_MUX_COM_1, IO_ANALOG_MUX_COM_2 };

// Default config for all *other* pins
const struct io_config io_default_unused = { false, IO_SELECT_ANALOG, IO_ALT_FUNCTION_0,
                                             IO_RESISTOR_DISABLED, IO_OUT_LOW };

typedef enum {
    HW_TYPE_ROBOTIC_ARM,
    HW_TYPE_ARM_SLEEVE,
    HW_TYPE_UNKNOWN
} hw_type_e;

/* STM32 provides a register that tells you what model their microcontroller is.
 * By accessing it, we can get the ID and the hardware type. */
#define STM32F411_DEVICE_ID 0x431
#define STM32F446_DEVICE_ID 0x421

static hw_type_e io_detect_hw_type(void)
{
    uint32_t dev_id = (*(volatile uint32_t *)0xE0042000) & 0xFFF;

    if (dev_id == STM32F411_DEVICE_ID) {
        return HW_TYPE_ARM_SLEEVE; // F411
    } else if (dev_id == STM32F446_DEVICE_ID) {
        return HW_TYPE_ROBOTIC_ARM; // F446
    }
    return HW_TYPE_UNKNOWN;
}

void io_init(void)
{
#if defined(ARM_SLEEVE)
    // TODO: Assert
    if (io_detect_hw_type() != HW_TYPE_ARM_SLEEVE) {
        while (1) { }
    }
#elif defined(ROBOTIC_ARM)
    // TODO: Assert
    if (io_detect_hw_type() != HW_TYPE_ROBOTIC_ARM) {
        while (1) { }
    }
#else
    // TODO: Assert
    while (1) { }
#endif
    for (int io = 0; io < IO_PIN_MAX; io++) {
        const struct io_config *cfg =
            (io_initial_configs[io].used != false) ? &io_initial_configs[io] : &io_default_unused;
        if (io != IO_PA13 && io != IO_PA14) {
            io_configure(io, cfg);
        }
    }
}

void io_configure(io_e io, const struct io_config *config)
{
    // Forbid configuring PA13 or PA14 (SWD/JTAG debug pins)
    ASSERT(io != (io_e)IO_PA13 && io != (io_e)IO_PA14);

    io_enable_clock(io);
    io_set_select(io, config->select, config->io_alt_function);
    io_set_resistor(io, config->resistor);
    io_set_out(io, config->out);
}

void io_get_current_config(io_e io, struct io_config *current_config)
{
    const uint8_t port = io_port(io); // 0
    const uint8_t pin = io_pin_idx(io); // 5
    GPIO_TypeDef *gpio = gpio_ports[port]; // gpioa
    uint8_t afr_n = pin / 8; // 0 or 1
    uint8_t afr_shift = (pin % 8) * 4; // number of shift indices

    current_config->select = (io_select_e)(gpio->MODER >> (pin * 2)) & (0x3);
    current_config->io_alt_function = (io_alt_function_e)(gpio->AFR[afr_n] >> (afr_shift)) & (0xFu);
    current_config->resistor = (io_resistor_e)(gpio->PUPDR >> (pin * 2)) & (0x3);
    current_config->out = (io_out_e)(gpio->ODR >> pin) & (0x1);
}

bool io_config_compare(const struct io_config *cfg1, const struct io_config *cfg2)
{
    return (cfg1->select == cfg2->select) && (cfg1->io_alt_function == cfg2->io_alt_function)
        && (cfg1->resistor == cfg2->resistor) && (cfg1->out == cfg2->out);
}

void io_set_select(io_e io, io_select_e select, io_alt_function_e alt_function)
{
    const uint8_t port = io_port(io); // 0
    const uint8_t pin = io_pin_idx(io); // 5

    GPIO_TypeDef *gpio = gpio_ports[port]; // gpioa
    gpio->MODER &= ~(0x3u << (pin * 2)); // clear bits in moder for pa5
    gpio->MODER |= ((select & 0x3u) << (pin * 2)); // set select type in moder

    // afr[0] for pins 0-7 and afr[1] for pins 8-15
    uint8_t afr_n = pin / 8; // 0 or 1
    // each pin gets 4 bits
    uint8_t afr_shift = (pin % 8) * 4; // number of shift indices
    // clear afr bits
    gpio->AFR[afr_n] &= ~(0xFu << afr_shift); // & with 0000

    // if alt function:
    if (select == IO_SELECT_ALT) {
        gpio->AFR[afr_n] |= ((alt_function & 0xFu) << afr_shift); // set alt function in afr
    }
}

void io_set_resistor(io_e io, io_resistor_e resistor)
// only applicable if pin is set to input
{
    const uint8_t port = io_port(io); // 0
    const uint8_t pin = io_pin_idx(io); // 5
    GPIO_TypeDef *gpio = gpio_ports[port]; // gpioa

    gpio->PUPDR &= ~(0x3u << (pin * 2)); // clear output
    gpio->PUPDR |= ((resistor & 0x3u) << (pin * 2));
}

void io_set_out(io_e io, io_out_e out)
{
    const uint8_t port = io_port(io); // 0
    const uint8_t pin = io_pin_idx(io); // 5
    GPIO_TypeDef *gpio = gpio_ports[port]; // gpioa

    gpio->ODR &= ~(0x1u << (pin)); // clear output
    gpio->ODR |= ((out & 0x1u) << (pin));
}

io_in_e io_get_input(io_e io)
{
    const uint8_t port = io_port(io); // 0
    const uint8_t pin = io_pin_idx(io); // 5
    const GPIO_TypeDef *gpio = gpio_ports[port]; // gpioa

    io_in_e state = (gpio->IDR & (0x1u << pin)) ? 1 : 0; // 1 if high, 0 if low
    return state;
}

static void io_clear_interrupt(io_e io)
{
    const uint8_t pin = io_pin_idx(io); // 5
    EXTI->PR |= (1 << pin);
}

/* This function disables interrupts before selecting the edge becuase it might trigger an interrupt
 * if you don't. */
static void io_set_interrupt_trigger(io_e io, io_trigger_e trigger)
{
    const uint8_t pin = io_pin_idx(io); // 5

    // // Disable interrupts
    io_disable_interrupt(io);

    // // Unmask line
    EXTI->IMR |= (1 << pin);

    switch (trigger) {
    case IO_TRIGGER_RISING:
        EXTI->RTSR |= (1 << pin);
        break;
    case IO_TRIGGER_FALLING:
        EXTI->FTSR |= (1 << pin);
        break;
    case IO_TRIGGER_BOTH_EDGES:
        EXTI->RTSR |= (1 << pin);
        EXTI->FTSR |= (1 << pin);
        break;
    }

    /* Clear interrupt because even if an interrupt is disabled, the flag is still set. */
    io_clear_interrupt(io);
}

static void io_register_isr(io_e io, isr_function isr)
{
    uint8_t pin = io_pin_idx(io); // 0-15
    uint8_t isr_idx = pin;
    // pins 5-9 share isr_functions[5], pins 10-15 share isr_functions[6]
    if (pin >= 5 && pin <= 9) {
        isr_idx = 5;
    } else if (pin >= 10 && pin <= 15) {
        isr_idx = 6;
    }

    ASSERT(isr_functions[isr_idx] == NULL);
    isr_functions[isr_idx] = isr;
}

void io_configure_exti_mapping(io_e io)
{
    uint8_t pin = io_pin_idx(io); // e.g., 0, 1, ...
    uint8_t port = io_port(io); // 0 for A, 1 for B, 2 for C, etc.
    uint8_t reg = pin / 4;
    uint8_t shift = (pin % 4) * 4;
    uint32_t mask = 0xF << shift;

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[reg] = (SYSCFG->EXTICR[reg] & ~mask) | (port << shift);
}

static io_e exti_to_io[16];

void io_configure_interrupt(io_e io, io_trigger_e trigger, isr_function isr)
{
    uint8_t pin = io_pin_idx(io);
    io_configure_exti_mapping(io);
    exti_to_io[pin] = io;
    io_set_interrupt_trigger(io, trigger);
    io_register_isr(io, isr);
}

static inline void io_unregister_isr(io_e io)
{
    uint8_t pin = io_pin_idx(io); // 0-15
    uint8_t isr_idx = pin;
    // pins 5-9 share isr_functions[5], pins 10-15 share isr_functions[6]
    if (pin >= 5 && pin <= 9) {
        isr_idx = 5;
    } else if (pin >= 10 && pin <= 15) {
        isr_idx = 6;
    }

    isr_functions[isr_idx] = NULL;
}

void io_deconfigure_interrupt(io_e io)
{
    io_unregister_isr(io);
    io_disable_interrupt(io);
}

void io_enable_interrupt(io_e io)
{
    const uint8_t pin = io_pin_idx(io); // 5
    EXTI->IMR |= (1 << pin);
    if (pin <= 4)
        NVIC_EnableIRQ((IRQn_Type)(EXTI0_IRQn + pin));
    else if (pin <= 9)
        NVIC_EnableIRQ(EXTI9_5_IRQn);
    else
        NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void io_disable_interrupt(io_e io)
{
    const uint8_t pin = io_pin_idx(io); // 5
    EXTI->IMR &= ~(1 << pin);
    if (pin <= 4)
        NVIC_DisableIRQ((IRQn_Type)(EXTI0_IRQn + pin));
    else if (pin <= 9)
        NVIC_DisableIRQ(EXTI9_5_IRQn);
    else
        NVIC_DisableIRQ(EXTI15_10_IRQn);
}

static void io_isr(io_e io)
{
    const uint8_t pin = io_pin_idx(io);
    uint8_t isr_idx = pin;
    if (pin >= 5 && pin <= 9)
        isr_idx = 5;
    else if (pin >= 10 && pin <= 15)
        isr_idx = 6;
    if (EXTI->PR & (1 << pin)) {
        if (isr_functions[isr_idx] != NULL) {
            isr_functions[isr_idx]();
        }
        io_clear_interrupt(io);
    }
}

io_e io_mapped_to_exti(uint8_t exti_line)
{
    return exti_to_io[exti_line];
}

void exti0_handler(void)
{
    io_isr(io_mapped_to_exti(0));
}

void exti1_handler(void)
{
    io_isr(io_mapped_to_exti(1));
}

void exti2_handler(void)
{
    io_isr(io_mapped_to_exti(2));
}

void exti3_handler(void)
{
    io_isr(io_mapped_to_exti(3));
}

void exti4_handler(void)
{
    io_isr(io_mapped_to_exti(4));
}

void exti9_5_handler(void)
{
    for (uint8_t pin = 5; pin <= 9; ++pin) {
        if (EXTI->PR & (1 << pin)) {
            io_isr(io_mapped_to_exti(pin));
        }
    }
}

void exti15_10_handler(void)
{
    for (uint8_t pin = 10; pin <= 15; ++pin) {
        if (EXTI->PR & (1 << pin)) {
            io_isr(io_mapped_to_exti(pin));
        }
    }
}

// Helper method to retrieve pointer to array of ADC pins
const io_e *io_adc_pins(uint8_t *cnt)
{
    *cnt = ARRAY_SIZE(io_adc_pins_arr);
    return io_adc_pins_arr;
}

// Check if a pin is compatible with ADC
bool io_supports_adc(io_e pin)
{
    switch (pin) {
    case IO_ANALOG_MUX_COM_1:
        return true;
    case IO_ANALOG_MUX_COM_2:
        return true;

    default:
        return false;
    }
}

// Helper method to retrieve ADC channel index given provided pin
uint8_t io_to_adc_idx(io_e io)
{
    ASSERT(io_supports_adc(io));
    return io_pin_idx(io);
}