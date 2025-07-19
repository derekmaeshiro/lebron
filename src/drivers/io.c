#include "io.h"
#include "../common/defines.h"
#include <stm32f4xx.h>
#include <stdint.h>

/* Extracing pin and port using bitwise operations. STM32 uses 16 pins per port
 * Enum value can be seen as:
 * [ Zeros (10 bits) | Port (2 bits) | pin (4 bits) ] */
#define IO_PORT_OFFSET (4u)
#define IO_PORT_MASK (0x3u << IO_PORT_OFFSET)
#define IO_PIN_MASK (0xFu)
#define IO_PORT_CNT 4
#define IO_PIN_CNT_PER_PORT 16

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

/* This array holds the initial configs of all IO pins. */
/* Never: PA13, PA14 (debug) - don't touch */
static const struct io_config io_initial_configs[IO_PORT_CNT * IO_PIN_CNT_PER_PORT] = {
#if defined(ROBOTIC_ARM)
    // Application Pins
    [IO_TEST_LED] = { IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                      IO_OUT_LOW }, // PA5 OUTPUT
    [IO_I2C_SCL] = { IO_SELECT_ALT, IO_ALT_FUNCTION_4, IO_PULL_UP_ENABLED,
                     IO_OUT_HIGH }, // PB8 I2C1_SCL
    [IO_I2C_SDA] = { IO_SELECT_ALT, IO_ALT_FUNCTION_4, IO_PULL_UP_ENABLED,
                     IO_OUT_HIGH }, // PB9 I2C1_SDA
    [IO_UART_RXD] = { IO_SELECT_ALT, IO_ALT_FUNCTION_7, IO_RESISTOR_DISABLED,
                      IO_OUT_LOW }, // PA10 USART1_RX
    [IO_UART_TXD] = { IO_SELECT_ALT, IO_ALT_FUNCTION_7, IO_RESISTOR_DISABLED,
                      IO_OUT_LOW }, // PA9 USART1_TX
    [IO_ANALOG_MUX_S0] = { IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                           IO_OUT_LOW }, // PB4 Output etc
    [IO_ANALOG_MUX_S1] = { IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                           IO_OUT_LOW }, // PB5
    [IO_ANALOG_MUX_S2] = { IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                           IO_OUT_LOW }, // PB3
    [IO_ANALOG_MUX_S3] = { IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                           IO_OUT_LOW }, // PA15
    [IO_ANALOG_MUX_ENABLE_1] = { IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                                 IO_OUT_LOW }, // PB12
    [IO_ANALOG_MUX_ENABLE_2] = { IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                                 IO_OUT_LOW }, // PB13
    [IO_ANALOG_MUX_COM_1] = { IO_SELECT_INPUT, IO_ALT_FUNCTION_0, IO_PULL_UP_ENABLED,
                              IO_OUT_LOW }, // PA0
    [IO_ANALOG_MUX_COM_2] = { IO_SELECT_INPUT, IO_ALT_FUNCTION_0, IO_PULL_UP_ENABLED,
                              IO_OUT_LOW }, // PA1
    [IO_PWM_DISTAL_INTERPHALANGEAL_JOINT] = { IO_SELECT_ALT, IO_ALT_FUNCTION_2,
                                              IO_RESISTOR_DISABLED, IO_OUT_LOW }, // PA6 TIM?
    [IO_PWM_PROXIMAL_INTERPHALANGEAL_JOINT] = { IO_SELECT_ALT, IO_ALT_FUNCTION_2,
                                                IO_RESISTOR_DISABLED, IO_OUT_LOW }, // PA7
    [IO_PWM_METACARPOPHALANGEAL_JOINT_1] = { IO_SELECT_ALT, IO_ALT_FUNCTION_2, IO_RESISTOR_DISABLED,
                                             IO_OUT_LOW }, // PC6?
    [IO_PWM_METACARPOPHALANGEAL_JOINT_2] = { IO_SELECT_ALT, IO_ALT_FUNCTION_1, IO_RESISTOR_DISABLED,
                                             IO_OUT_LOW }, // PB10

    // Board-specific
    [IO_PA2] = { IO_SELECT_ALT, IO_ALT_FUNCTION_7, IO_RESISTOR_DISABLED, IO_OUT_LOW }, // USART2_TX
    [IO_PA3] = { IO_SELECT_ALT, IO_ALT_FUNCTION_7, IO_RESISTOR_DISABLED, IO_OUT_LOW }, // USART2_RX
    [IO_PC13] = { IO_SELECT_INPUT, IO_ALT_FUNCTION_0, IO_PULL_UP_ENABLED,
                  IO_OUT_LOW }, // User Button
    [IO_PC14] = { IO_SELECT_ANALOG, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                  IO_OUT_LOW }, // LSE Crystal
    [IO_PC15] = { IO_SELECT_ANALOG, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                  IO_OUT_LOW }, // LSE Crystal
#elif defined(ARM_SLEEVE)
    // Application Pins
    [IO_I2C_SCL] = { IO_SELECT_ALT, IO_ALT_FUNCTION_4, IO_PULL_UP_ENABLED,
                     IO_OUT_HIGH }, // PB8, I2C1_SCL
    [IO_I2C_SDA] = { IO_SELECT_ALT, IO_ALT_FUNCTION_4, IO_PULL_UP_ENABLED,
                     IO_OUT_HIGH }, // PB9, I2C1_SDA
    [IO_UART_TXD] = { IO_SELECT_ALT, IO_ALT_FUNCTION_7, IO_RESISTOR_DISABLED,
                      IO_OUT_LOW }, // PA9, USART1_TX
    [IO_UART_RXD] = { IO_SELECT_ALT, IO_ALT_FUNCTION_7, IO_RESISTOR_DISABLED,
                      IO_OUT_LOW }, // PA10, USART1_RX
    [IO_TEST_LED] = { IO_SELECT_OUTPUT, IO_ALT_FUNCTION_0, IO_RESISTOR_DISABLED,
                      IO_OUT_LOW }, // PC13, User LED
#endif
};

// Default config for all *other* pins
const struct io_config io_default_unused = { IO_SELECT_ANALOG, IO_ALT_FUNCTION_0,
                                             IO_RESISTOR_DISABLED, IO_OUT_LOW };

typedef enum {
    HW_TYPE_ROBOTIC_ARM,
    HW_TYPE_ARM_SLEEVE,
    HW_TYPE_UNKNOWN
} hw_type_e;

/* STM32 provides a register that tells you what model their microcontroller is.
 * By accessing it, we can get the ID and the hardware type. */
#define STM32F411_DEVICE_ID 0x1B1
#define STM32F446_DEVICE_ID 0x1C9

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
            (io_initial_configs[io].select != 0) ? &io_initial_configs[io] : &io_default_unused;
        io_configure(io, cfg);
    }
}

void io_configure(io_e io, const struct io_config *config)
{
    io_enable_clock(io);
    io_set_select(io, config->select, config->io_alt_function);
    io_set_resistor(io, config->resistor);
    io_set_out(io, config->out);
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