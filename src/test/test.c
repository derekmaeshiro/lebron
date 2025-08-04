#include <stm32f4xx.h>
// #include "../include/stm32f446xx.h" // Include the device-specific header
#include "../drivers/led.h"
#include "../drivers/io.h"
#include "../drivers/pwm.h"
#include "../drivers/mcu_init.h"
#include "../drivers/uart.h"
#include "../drivers/adc.h"
#include "../common/assert_handler.h"
#include "../common/defines.h"
#include "../common/trace.h"

// For PWM test
#define PWM_GPIO_PORT GPIOA
#define PWM_GPIO_PIN 6
#define PWM_AF 2

SUPPRESS_UNUSED
static void test_setup(void)
{
    mcu_init();
}

SUPPRESS_UNUSED
static void test_assert(void)
{
    test_setup();
    ASSERT(0);
}

SUPPRESS_UNUSED
static void test_blink_led(void)
{
    test_setup();
    io_init();
    led_init();
    led_state_e led_state = LED_STATE_OFF;

    while (1) {
        led_state = (led_state == LED_STATE_OFF) ? LED_STATE_ON : LED_STATE_OFF;
        led_set(LED_TEST, led_state);
        BUSY_WAIT_ms(1000);
    }
}

SUPPRESS_UNUSED
// Configure all pins as output, toggle them in a loop. Verify with logic analyzer.
static void test_nucleo_io_pins_output(void)
{
    test_setup();
    const struct io_config output_config = { .select = IO_SELECT_OUTPUT,
                                             .io_alt_function = IO_ALT_FUNCTION_0,
                                             .resistor = IO_RESISTOR_DISABLED,
                                             .out = IO_OUT_LOW };

    // Configure all pins as output
    for (io_pin_e io = IO_PA0; io <= IO_PD2; io++) {
        if (io == IO_PA13 || io == IO_PA14) {
            continue;
        }

        io_configure((io_e)io, &output_config);
    }

    while (1) {
        for (io_pin_e io = IO_PA0; io <= IO_PD2; io++) {
            if (io == IO_PA13 || io == IO_PA14) {
                continue;
            }

            io_set_out((io_e)io, IO_OUT_HIGH);
            for (volatile int i = 0; i < 10000; i++) { }
            io_set_out((io_e)io, IO_OUT_LOW);
        }
    }
}

/* Configure all the pins except PA5 as input with internal pull-up resistors.
 * Expected behavior: Driving a pin to ground will turn off the led until you un-drive the pin.
 * Some pins are, by default, configured to be UART, SPI, etc. so when they're set to pull-up,
 * they don't work as intended. For example, PA2 and PA3 won't work as they're UART. */
SUPPRESS_UNUSED
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
        if (io == IO_PA13 || io == IO_PA14) {
            continue;
        }

        io_configure((io_e)io, &input_config);
    }

    // Configure led pin as output
    io_configure((io_e)io_led, &led_config);

    for (io_pin_e io = IO_PA0; io <= IO_PD2; io++) {
        if (io == io_led) {
            continue;
        }
        if (io == IO_PA13 || io == IO_PA14) {
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

SUPPRESS_UNUSED
static void pa_1_isr(void)
{
    led_set(LED_TEST, LED_STATE_ON);
}

SUPPRESS_UNUSED
static void pa_10_isr(void)
{
    led_set(LED_TEST, LED_STATE_OFF);
}

SUPPRESS_UNUSED
static void test_io_interrupt(void)
{
    test_setup();
    const struct io_config input_config = {
        .select = IO_SELECT_INPUT,
        .io_alt_function = IO_ALT_FUNCTION_0,
        .resistor = IO_PULL_UP_ENABLED,
        .out = IO_OUT_HIGH,
    };

    io_configure((io_e)IO_PA1, &input_config);
    io_configure((io_e)IO_PA10, &input_config);
    led_init();
    io_configure_interrupt((io_e)IO_PA1, IO_TRIGGER_FALLING, pa_1_isr);
    io_configure_interrupt((io_e)IO_PA10, IO_TRIGGER_FALLING, pa_10_isr);
    io_enable_interrupt((io_e)IO_PA1);
    io_enable_interrupt((io_e)IO_PA10);
    while (1)
        ;
}

SUPPRESS_UNUSED
static void test_uart_put_char_polling(void)
{
    test_setup();
    mcu_init();
    uart_init();

    uart_putchar_polling('a');
    uart_putchar_polling('p');
    uart_putchar_polling('p');
    uart_putchar_polling('l');
    uart_putchar_polling('e');
    uart_putchar_polling('a');
    uart_putchar_polling('p');
    uart_putchar_polling('p');
    uart_putchar_polling('l');
    uart_putchar_polling('e');
    uart_putchar_polling('a');
    uart_putchar_polling('p');
    uart_putchar_polling('p');
    uart_putchar_polling('l');
    uart_putchar_polling('e');
    uart_putchar_polling('a');
    uart_putchar_polling('p');
    uart_putchar_polling('p');
    uart_putchar_polling('l');
    uart_putchar_polling('e');
    uart_putchar_polling('\n');
}

SUPPRESS_UNUSED
static void test_uart_put_char_interrupt(void)
{
    test_setup();
    mcu_init();
    uart_init();

    // uart_putchar_interrupt('a');
    // uart_putchar_interrupt('b');
    // uart_putchar_interrupt('c');
    // uart_putchar_interrupt('d');
    // uart_putchar_interrupt('e');

    // const char *msg = "apples\n";
    // while (*msg) {
    //     uart_putchar_interrupt(*msg++);
    // }

    while (1) {
        // uart_putchar_interrupt('H');
        // uart_putchar_interrupt('e');
        // uart_putchar_interrupt('l');
        // uart_putchar_interrupt('l');
        // uart_putchar_interrupt('o');
        // uart_putchar_interrupt(' ');
        // uart_putchar_interrupt('W');
        // uart_putchar_interrupt('o');
        // uart_putchar_interrupt('r');
        // uart_putchar_interrupt('l');
        // uart_putchar_interrupt('d');
        // uart_putchar_interrupt('\n');
    }

    // while (1) {}

    // const char *msg = "apples\n";
    // while (*msg) {
    //     uart_putchar_interrupt(*msg++);
    // }
    // while (*msg) {
    //     uart_putchar_interrupt(*msg++);
    // }
    // while (*msg) {
    //     uart_putchar_interrupt(*msg++);
    // }

    // while (*msg) {
    //     uart_putchar_interrupt(*msg++);
    // }

    while (1) { }
}

SUPPRESS_UNUSED
static void test_uart_put_string(void)
{
    test_setup();
    mcu_init();
    uart_init();

    while (1) {
        uart_print_interrupt("ANgry brids\n");
        BUSY_WAIT_ms(250);
    }
}

SUPPRESS_UNUSED
static void test_uart(void)
{
    test_setup();
    uart_init();
    while (1) {
        _putchar('D');
        _putchar('E');
        _putchar('R');
        _putchar('E');
        _putchar('K');
        _putchar('!');
        _putchar('\n');
        BUSY_WAIT_ms(100);
    }
}

SUPPRESS_UNUSED
static void test_trace(void)
{
    test_setup();
    trace_init();

    while (1) {
        // printf("derek maeshiro %d\n", 2025);
        TRACE("Artful bytes %d", 2025);
        BUSY_WAIT_ms(1000);
    }
}

SUPPRESS_UNUSED
static void test_pwm(void)
{
    test_setup();
    trace_init();
    led_init();
#if defined ROBOTIC_ARM
    pwm_init();
    pwm_e pwms[] = { PWM_DISTAL_INTERPHALANGEAL_JOINT,
                             PWM_PROXIMAL_INTERPHALANGEAL_JOINT,
                             PWM_METACARPOPHALANGEAL_JOINT_1,
                             PWM_METACARPOPHALANGEAL_JOINT_2 };
    const int duty_cycles[] = { 0, 20, 40, 60, 80, 100 };
    const uint16_t wait_time = 3000;
    while (1) {
        for (uint8_t i = 0; i < 6; i++) {
            TRACE("Set duty cycle to %d for %d ms", duty_cycles[i], wait_time);
            led_set(LED_TEST, LED_STATE_ON);
            for(uint8_t j=0; j<ARRAY_SIZE(pwms); j++) {
                pwm_set_duty_cycle(pwms[j], duty_cycles[i]);
            }
            BUSY_WAIT_ms(wait_time);

            // Turn off the PWM channel and toggle off LED
            TRACE("Turning off PWM and waiting for %d ms", wait_time);
            led_set(LED_TEST, LED_STATE_OFF);
            for(uint8_t j=0; j<ARRAY_SIZE(pwms); j++) {
                pwm_set_duty_cycle(pwms[j], 0);
            }
            BUSY_WAIT_ms(wait_time);
        }
    }
#endif
}
SUPPRESS_UNUSED
static void test_adc(void)
{
    test_setup();
    // Uncomment once UART is complete
    //trace_init();
    adc_init();
    while(1){
        adc_channel_values_t values;
        adc_get_channel_values(values);
        for(uint8_t i=0; i<ADC_CHANNEL_COUNT; i++){
            // TODO: Uncomment once UART is complete
            // TRACE("ADC ch %u: %u", i, values[i]);
        }
        BUSY_WAIT_ms(1000);
    }
}

int main(void)
{
    TEST();
    while (1) { }

    ASSERT(0);
}
