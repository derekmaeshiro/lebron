#include <stm32f4xx.h>
// #include "../include/stm32f446xx.h" // Include the device-specific header
#include "../drivers/led.h"
#include "../drivers/io.h"
#include "../drivers/pwm.h"
#include "../drivers/mcu_init.h"
#include "../common/assert_handler.h"
#include "../common/defines.h"

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

        io_configure( (io_e) io, &output_config);
    }

    while (1) {
        for (io_pin_e io = IO_PA0; io <= IO_PD2; io++) {
            if (io == IO_PA13 || io == IO_PA14) {
                continue;
            }

            io_set_out( (io_e) io, IO_OUT_HIGH);
            for (volatile int i = 0; i < 10000; i++) {}
            io_set_out( (io_e) io, IO_OUT_LOW);
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

    io_configure((io_e) IO_PA1, &input_config);
    io_configure((io_e) IO_PA10, &input_config);
    led_init();
    io_configure_interrupt((io_e) IO_PA1, IO_TRIGGER_FALLING, pa_1_isr);
    io_configure_interrupt((io_e) IO_PA10, IO_TRIGGER_FALLING, pa_10_isr);
    io_enable_interrupt((io_e) IO_PA1);
    io_enable_interrupt((io_e) IO_PA10);
    while (1);
}

SUPPRESS_UNUSED
static void test_pwm(void){
    test_setup();
    // Uncomment when merged with uart branch
    // trace_init();
    pwm_init();
    int duty_cycles[] = {100, 28, 54, 16, 22, 88};
    while(1){
        for(uint8_t i=0; i<ARRAY_SIZE(duty_cycles); i++){
            pwm_set_duty_cycle(PWM_DISTAL_INTERPHALANGEAL_JOINT, duty_cycles[i]);
            pwm_set_duty_cycle(PWM_PROXIMAL_INTERPHALANGEAL_JOINT, duty_cycles[i]);
            pwm_set_duty_cycle(PWM_METACARPOPHALANGEAL_JOINT_1, duty_cycles[i]);
            pwm_set_duty_cycle(PWM_METACARPOPHALANGEAL_JOINT_2, duty_cycles[i]);
            BUSY_WAIT_ms(3000);
        }
    }
}

int main(void)
{
    TEST();
    ASSERT(0);
}
