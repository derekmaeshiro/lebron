#include <stm32f4xx.h>
// #include "../include/stm32f446xx.h" // Include the device-specific header
#include "drivers/led.h"
#include "drivers/io.h"
#include "drivers/mcu_init.h"

#define LED_PIN 13

static void test_setup(void)
{
    mcu_init();
}

// static void test_blink_led(void)
// {
//     test_setup();

//     const struct io_config led_config =
//     {
//         .select = IO_SELECT_OUTPUT,
//         .io_alt_function = IO_ALT_FUNCTION_0,
//         .resistor = IO_RESISTOR_DISABLED,
//     };

//     io_configure(IO_TEST_LED, &led_config);
//     io_out_e out = IO_OUT_LOW;

//     while (1) {
//         out = (out == IO_OUT_LOW) ? IO_OUT_HIGH : IO_OUT_LOW;
//         io_set_out(IO_TEST_LED, out);
//         for (volatile int i = 0; i < 500000; i++) {}
//     }
// }

// Configure all pins as output, toggle them in a loop. Verify with logic analyzer.
// static void test_nucleo_io_pins_output(void)
// {
//     test_setup();
//     const struct io_config output_config = { .select = IO_SELECT_OUTPUT,
//                                              .io_alt_function = IO_ALT_FUNCTION_0,
//                                              .resistor = IO_RESISTOR_DISABLED,
//                                              .out = IO_OUT_LOW };

//     // Configure all pins as output
//     for (io_pin_e io = IO_PA0; io <= IO_PD2; io++) {
//         io_configure( (io_e) io, &output_config);
//     }

//     while (1) {
//         for (io_pin_e io = IO_PA0; io <= IO_PD2; io++) {
//             io_set_out( (io_e) io, IO_OUT_HIGH);
//             for (volatile int i = 0; i < 10000; i++) {}
//             io_set_out( (io_e) io, IO_OUT_LOW);
//         }
//     }
// }

/* Configure all the pins except PA5 as input with internal pull-up resistors.
   Expected behavior: */
static void test_nucleo_io_pins_input(void)
{
    test_setup();
    const struct io_config input_config = { .select = IO_SELECT_INPUT,
                                            .resistor = IO_PULL_UP_ENABLED,
                                            .out = IO_OUT_HIGH };

    const struct io_config led_config = { .select = IO_SELECT_OUTPUT,
                                          .io_alt_function = IO_ALT_FUNCTION_0,
                                          .resistor = IO_RESISTOR_DISABLED,
                                          .out = IO_OUT_LOW };

    const io_pin_e io_led = IO_PA5;

    // Configure all pins as input
    for (io_pin_e io = IO_PA0; io <= IO_PD2; io++) {
        io_configure((io_e)io, &input_config);
    }

    // Configure led pin as output
    io_configure((io_e)io_led, &led_config);

    for (io_pin_e io = IO_PA0; io <= IO_PD2; io++) {
        if (io == io_led) {
            continue;
        }

        io_set_out((io_e)io_led, IO_OUT_HIGH);

        // Wait for the operator to pull the pin low
        while (io_get_input((io_e)io) == IO_IN_HIGH) {
            for (volatile int i = 0; i < 10000; i++) { }
        }

        io_set_out((io_e)io_led, IO_OUT_LOW);

        // Wait for user to disconnect
        while (io_get_input((io_e)io) == IO_IN_LOW) {
            for (volatile int i = 0; i < 10000; i++) { }
        }
    }

    // Blink LED when test is done
    while (1) {
        io_set_out((io_e)io_led, IO_OUT_HIGH);
        for (volatile int i = 0; i < 500000; i++) { }
        io_set_out((io_e)io_led, IO_OUT_LOW);
        for (volatile int i = 0; i < 500000; i++) { }
    }
}

int main(void)
{
    test_nucleo_io_pins_input();
    // test_nucleo_io_pins_output();
    // test_blink_led();
}

// int main(void)
// {
//     // Enable the GPIOB clock
//     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // Enable clock for GPIO port B
//     // Set pin PB0 as output
//     GPIOC->MODER &= ~(3U << (LED_PIN * 2)); // Clear mode bits for PB0
//     led_init();
//     // GPIOA->MODER |= (1U << (LED_PIN * 2));

//     while (1) {
//         led_toggle();
//         // GPIOA->ODR ^= (1U << LED_PIN);
//         for (volatile int i = 0; i < 100000; i++)
//             ; // Busy wait
//     }
// }