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