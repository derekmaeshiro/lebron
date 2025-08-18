#include <stm32f4xx.h>
// #include "../include/stm32f446xx.h" // Include the device-specific header
#include "../drivers/led.h"
#include "../drivers/io.h"
#include "../drivers/pwm.h"
#include "../drivers/potentiometer.h"
#include "../drivers/mcu_init.h"
#include "../drivers/uart.h"
#include "../drivers/adc.h"
#include "../drivers/i2c.h"
#include "../drivers/servo_driver.h"
#include "../common/assert_handler.h"
#include "../common/defines.h"
#include "../common/trace.h"

// For PWM test
#define PWM_GPIO_PORT GPIOA
#define PWM_GPIO_PIN 6
#define PWM_AF 2

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

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
        BUSY_WAIT_ms(5000);
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
    pwm_e pwms[] = { PWM_DISTAL_INTERPHALANGEAL_JOINT, PWM_PROXIMAL_INTERPHALANGEAL_JOINT,
                     PWM_METACARPOPHALANGEAL_JOINT_1, PWM_METACARPOPHALANGEAL_JOINT_2 };
    const int duty_cycles[] = { 0, 20, 40, 60, 80, 100 };
    const uint16_t wait_time = 3000;
    while (1) {
        for (uint8_t i = 0; i < 6; i++) {
            TRACE("Set duty cycle to %d for %d ms", duty_cycles[i], wait_time);
            led_set(LED_TEST, LED_STATE_ON);
            for (uint8_t j = 0; j < ARRAY_SIZE(pwms); j++) {
                pwm_set_duty_cycle(pwms[j], duty_cycles[i]);
            }
            BUSY_WAIT_ms(wait_time);

            // Turn off the PWM channel and toggle off LED
            TRACE("Turning off PWM and waiting for %d ms", wait_time);
            led_set(LED_TEST, LED_STATE_OFF);
            for (uint8_t j = 0; j < ARRAY_SIZE(pwms); j++) {
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
    trace_init();
    led_init();
    adc_init();
    TRACE("Testing adc...");
    while (1) {
        uint8_t channel = 0;
        uint16_t adc_value = adc_read_single(channel);
        TRACE("ADC ch %u: %u", channel, adc_value);
        BUSY_WAIT_ms(2000);
        // adc_channel_values_t values;
        // adc_get_channel_values(values);
        // for (uint8_t i = 0; i < ARRAY_SIZE(values); i++) {
        //     TRACE("ADC ch %u: %u", i, values[i]);
        // }
        // TRACE("\n");
        // BUSY_WAIT_ms(2000);
    }
}

/* TESTING I2C */

volatile uint8_t g_pca9685_last_read = 0;

void pca9685_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t payload[2] = { reg, val };
    i2c_write(0x40, payload, 2, NULL);
    while (i2c_is_busy())
        ; // wait for write to finish
}

void pca9685_read_reg(uint8_t reg)
{
    i2c_write(0x40, &reg, 1, NULL); // write reg address
    while (i2c_is_busy())
        ; // wait for write to finish
    i2c_read(0x40, (uint8_t *)&g_pca9685_last_read, 1, NULL); // read value
    while (i2c_is_busy())
        ; // wait for read to finish
}

SUPPRESS_UNUSED
static void test_pca9685_readwrite(void)
{
    test_setup();
    trace_init();
    i2c_init();

    // Step 1: Write MODE1 to 0x10 (sleep mode)
    pca9685_write_reg(0x00, 0x10);
    TRACE("Write complete.\n");

    // Step 2: Read it back
    pca9685_read_reg(0x00);
    TRACE("Read MODE1 = 0x%x\n", g_pca9685_last_read);
}

// Just write some bytes to a device and confirm no error
bool test_i2c_write_only(uint8_t addr)
{
    uint8_t data[3] = { 0x00, 0x55, 0xAA }; // example payload
    if (!i2c_write(addr, data, 3, NULL)) {
        TRACE("I2C busy, write failed\n");
        return false;
    }
    while (i2c_is_busy())
        ;
    TRACE("I2C write only test complete\n");
    return true;
}

// This assumes device auto-increments or defaults to register 0 on read
bool test_i2c_read_only(uint8_t addr, uint8_t *val)
{
    if (!i2c_read(addr, val, 1, NULL)) {
        TRACE("I2C busy, read failed\n");
        return false;
    }
    while (i2c_is_busy())
        ;
    TRACE("I2C read only test value = 0x%x\n", *val);
    return true;
}

// Write register address, then read 1 byte back (blocking, simple)
bool test_i2c_write_then_read(uint8_t addr, uint8_t reg, uint8_t *val)
{
    if (!i2c_write(addr, &reg, 1, NULL))
        return false;
    while (i2c_is_busy())
        ;
    if (!i2c_read(addr, val, 1, NULL))
        return false;
    while (i2c_is_busy())
        ;
    TRACE("I2C write-then-read test reg=0x%x val=0x%x\n", reg, *val);
    return true;
}

void test_i2c_loop_read(uint8_t addr, uint8_t reg)
{
    uint8_t val = 0;
    while (1) {
        if (test_i2c_write_then_read(addr, reg, &val)) {
            TRACE("Loop read reg 0x%x = 0x%x\n", reg, val);
        } else {
            TRACE("I2C loop read failed\n");
        }
        BUSY_WAIT_ms(1000); // your delay function
    }
}

// Example usage in your main or test function
void test_i2c_simple(void)
{
    test_setup();
    trace_init();
    i2c_init();

    uint8_t val = 0xAA;

    // Write 0xAA to register 0x06
    uint8_t payload[2] = { 0x06, val };
    bool ok1 = i2c_write(0x40, payload, 2, NULL);
    while (i2c_is_busy())
        ;

    // Read it back
    val = 0;
    bool ok2 = test_i2c_write_then_read(0x40, 0x06, &val);
    while (i2c_is_busy())
        ;

    if (ok1 && ok2) {
        TRACE("LED0_ON_L = 0x%02X\n", val);
    } else {
        TRACE("I2C write or read failed\n");
    }
}

void pca9685_init(void)
{
    // 1. Sleep
    uint8_t sleep[] = { 0x00, 0x10 }; // MODE1 = sleep
    i2c_write(0x40, sleep, 2, NULL);
    while (i2c_is_busy())
        ;

    // 2. Set prescaler for 50Hz PWM
    uint8_t prescale[] = { 0xFE, 121 }; // PRESCALE = 121
    i2c_write(0x40, prescale, 2, NULL);
    while (i2c_is_busy())
        ;

    // 3. Wake up with auto-increment
    uint8_t wake[] = { 0x00, 0x20 }; // MODE1 = auto-increment
    i2c_write(0x40, wake, 2, NULL);
    while (i2c_is_busy())
        ;
}

// Send PWM pulse to channel 0 (LED0_ON_L = 0x06)
void pca9685_set_pwm_ch0(uint16_t on, uint16_t off)
{
    uint8_t payload[5];
    payload[0] = 0x06; // LED0_ON_L register
    payload[1] = on & 0xFF;
    payload[2] = on >> 8;
    payload[3] = off & 0xFF;
    payload[4] = off >> 8;

    i2c_write(0x40, payload, 5, NULL);
    while (i2c_is_busy())
        ;
}

void test_servo_simple(void)
{
    test_setup();
    trace_init();
    i2c_init();

    pca9685_init();

    //while (1) { }

    while (1) { //175 for stm32f446re
        // Sweep left
        pca9685_set_pwm_ch0(0, 102); // 0.5ms pulse
        TRACE("Left\n");
        BUSY_WAIT_ms(1000);

        // Sweep center
        pca9685_set_pwm_ch0(0, 307); // 1.5ms pulse
        TRACE("Center\n");
        BUSY_WAIT_ms(1000);

        // Sweep right
        pca9685_set_pwm_ch0(0, 512); // 2.5ms pulse
        TRACE("Right\n");
        BUSY_WAIT_ms(1000);
    }
}

void test_multiple_servos(void)
{
    test_setup();
    i2c_init();
    servo_driver_t driver = { 0 };
    servo_driver_init(&driver, 0x40); // Includes set-all to 90 deg
    // Next line guarantees channel 0 gets set again:
    // servo_driver_set_servo_angle(&driver, 0, 90);
    while (1) { }
}

SUPPRESS_UNUSED
void test_analog_mux(void){
    test_setup();
    trace_init();

    #if defined ROBOTIC_ARM
    analog_mux_init();

    TRACE("Testing analog mux...");
    while(1){
        for(uint8_t i=0; i<3; i++){
            TRACE("Toggling to pin %u", i);
            toggle_analog_mux(MUX_BOARD_1, i);
            BUSY_WAIT_ms(5000);
        }
    }
    #endif
}

SUPPRESS_UNUSED
void test_potentiometer(void)
{
    test_setup();
    trace_init();
    adc_init();

    #if defined ROBOTIC_ARM
    potentiometer_init();

    TRACE("Testing potentiometer...");
    while (1) {
        uint8_t current_potentiometer = 0;
        potentiometer_e potentiometers[] = { POTENTIOMETER_1, POTENTIOMETER_2 };
        uint16_t angle_value = potentiometer_read(potentiometers[current_potentiometer]);
        TRACE("POTENTIOMETER %u READING: %u", current_potentiometer+1, angle_value);
        // for (uint8_t i = 0; i < ARRAY_SIZE(potentiometers); i++) {
        //     uint16_t angle_value = potentiometer_read(potentiometers[i]);
        //     TRACE("POTENTIOMETER %u READING: %u", i + 1, angle_value);
        // }
        BUSY_WAIT_ms(500);
    }
    #endif
}
SUPPRESS_UNUSED
void test_read_all_potentiometers(void)
{
    test_setup();
    trace_init();
    adc_init();

    #if defined ROBOTIC_ARM
    potentiometer_init();

    TRACE("Testing potentiometer...");
    while (1) {
        uint8_t i = 0;
        uint16_t angles[2];
        read_all_potentiometers(angles);
        TRACE("POTENTIOMETER %u READING: %u", i, angles[i]);
        BUSY_WAIT_ms(500);
    }
    #endif
}

void delay(volatile uint32_t count) {
    while (count--);
}

SUPPRESS_UNUSED
static void test_pc_13(void)
{
    // Enable the clock for GPIOC
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; 

    // Clear mode for PC13 and set it to output mode
    GPIOC->MODER &= ~(0x3 << (13 * 2)); // Clear mode for PC13
    GPIOC->MODER |= (0x01 << (13 * 2)); // Set to output mode (01)

    while (1) {
        // Toggle the LED
        GPIOC->ODR ^= (1 << 13); // Toggle PC13
        delay(3000000); // Call a simple delay
    }
}

// PA9 = USART1_TX
void uart_init_simple(void)
{
    // Enable GPIOA/USART1 clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // Set PA9 to AF7 (USART1)
    GPIOA->MODER &= ~(0b11 << (9*2));
    GPIOA->MODER |=  (0b10 << (9*2));
    GPIOA->AFR[1] &= ~(0xF << ((9-8)*4));
    GPIOA->AFR[1] |=  (0x7 << ((9-8)*4));

    // Set baud: 100 MHz / 9600 = 10417 -> mantissa=651, frac=1. 6511 = 0x197F
    USART1->BRR = (100000000 + 4800)/9600;   // rounding for ~9600
        
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE;
}

void uart_putc(char c)
{
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = c;
}

SUPPRESS_UNUSED
void test_polling_check(void)
{
    test_setup();
    uart_init_simple();
    // blink here if you want

    while (1) {
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = 'U';
    for (volatile int i = 0; i < 200000; ++i) {}
}
}

SUPPRESS_UNUSED
void test_uart_potentiometer_readings(void){
    test_setup();
    trace_init();
    adc_init();
    #if defined ROBOTIC_ARM
    potentiometer_init();
    // const struct potentiometer_reading dummy_readings[] = {
    //     { .potentiometer_board = POTENTIOMETER_1, .angle = 45 },
    //     { .potentiometer_board = POTENTIOMETER_2, .angle = 90 },
    // };
    uint16_t angles[2];
    read_all_potentiometers(angles);
    struct potentiometer_reading potentiometer_readings[2];
    for(int i=0; i<2; i++){
        potentiometer_readings[i].potentiometer_board = (potentiometer_e)i;
        potentiometer_readings[i].angle = angles[i];
        TRACE("Potentiometer %u angle: %u", i, angles[i]);
    }
    while(1){
        uart_send_potentiometer_readings(potentiometer_readings, 2);
        BUSY_WAIT_ms(2000);
    }
    #endif
}

SUPPRESS_UNUSED
void test_uart_potentiometers_deserialize(void){
    #if defined ROBOTIC_ARM
    test_setup();
    uart_init();
    const char *test_strings[] = {
        "0,45\n",
        "1,90\n",
        "0,360\n",
        "1,180\n",
    };
    struct potentiometer_reading reading;
    for(uint8_t i=0; i<ARRAY_SIZE(test_strings); i++){
        deserialize_potentiometer_reading(test_strings[i], &reading);
        TRACE("Deserialized: Board=%u Angle=%u from \"%s\"", reading.potentiometer_board, reading.angle, test_strings[i]);
    }
    while(1) {}
    #endif
}

int main(void)
{
    TEST();
    while (1) { }

    ASSERT(0);
}
