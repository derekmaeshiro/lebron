#include <stm32f4xx.h>
// #include "../include/stm32f446xx.h" // Include the device-specific header
#include "../drivers/led.h"
#include "../drivers/io.h"
#include "../drivers/pwm.h"
#include "../drivers/potentiometer.h"
#include "../drivers/potentiometer_workflow.h"
#include "../drivers/mcu_init.h"
#include "../drivers/uart.h"
#include "../drivers/adc.h"
#include "../drivers/i2c.h"
#include "../drivers/servo_driver.h"
#include "../drivers/imu_driver.h"
#include "../common/assert_handler.h"
#include "../common/defines.h"
#include "../common/trace.h"
#include "../common/ring_buffer.h"

#include <math.h>

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

    while (1) {
        uart_putchar_polling('a');
        uart_putchar_polling('p');
        uart_putchar_polling('p');
        uart_putchar_polling('l');
        uart_putchar_polling('e');
    }

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

// SUPPRESS_UNUSED
// static void test_pwm(void)
// {
//     test_setup();
//     trace_init();
//     led_init();
// #if defined ROBOTIC_ARM
//     pwm_init();
//     pwm_e pwms[] = { PWM_DISTAL_INTERPHALANGEAL_JOINT, PWM_PROXIMAL_INTERPHALANGEAL_JOINT,
//                      PWM_METACARPOPHALANGEAL_JOINT_1, PWM_METACARPOPHALANGEAL_JOINT_2 };
//     const int duty_cycles[] = { 0, 20, 40, 60, 80, 100 };
//     const uint16_t wait_time = 3000;
//     while (1) {
//         for (uint8_t i = 0; i < 6; i++) {
//             TRACE("Set duty cycle to %d for %d ms", duty_cycles[i], wait_time);
//             led_set(LED_TEST, LED_STATE_ON);
//             for (uint8_t j = 0; j < ARRAY_SIZE(pwms); j++) {
//                 pwm_set_duty_cycle(pwms[j], duty_cycles[i]);
//             }
//             BUSY_WAIT_ms(wait_time);

//             // Turn off the PWM channel and toggle off LED
//             TRACE("Turning off PWM and waiting for %d ms", wait_time);
//             led_set(LED_TEST, LED_STATE_OFF);
//             for (uint8_t j = 0; j < ARRAY_SIZE(pwms); j++) {
//                 pwm_set_duty_cycle(pwms[j], 0);
//             }
//             BUSY_WAIT_ms(wait_time);
//         }
//     }
// #endif
// }

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

    // #if defined ROBOTIC_ARM
    potentiometer_init();

    TRACE("Testing potentiometer...");
    while (1) {
        uint8_t current_potentiometer = 0;
        joint_e potentiometers[] = { THUMB_DISTAL, THUMB_MIDDLE };
        uint16_t angle_value = potentiometer_read(potentiometers[current_potentiometer]);
        TRACE("POTENTIOMETER %u READING: %u", current_potentiometer+1, angle_value);
        // for (uint8_t i = 0; i < ARRAY_SIZE(potentiometers); i++) {
        //     uint16_t angle_value = potentiometer_read(potentiometers[i]);
        //     TRACE("POTENTIOMETER %u READING: %u", i + 1, angle_value);
        // }
        BUSY_WAIT_ms(500);
    }
    // #endif
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

#define MPU_ADDR 0x68 // address without shifting

void test_mpu6050(void)
{
    test_setup();
    trace_init();
    adc_init();
    potentiometer_init();
    // const struct potentiometer_reading dummy_readings[] = {
    //     { .potentiometer_board = POTENTIOMETER_1, .angle = 45 },
    //     { .potentiometer_board = POTENTIOMETER_2, .angle = 90 },
    // };
    uint16_t angles[2];
    read_all_potentiometers(angles);
    struct potentiometer_reading potentiometer_readings[2];
    for(int i=0; i<2; i++){
        potentiometer_readings[i].potentiometer_board = (joint_e)i;
        potentiometer_readings[i].angle = angles[i];
        TRACE("Potentiometer %u angle: %u", i, angles[i]);
    }
    while(1){
        uart_send_potentiometer_readings(potentiometer_readings, 2);
        BUSY_WAIT_ms(2000);
    }
}

void test_mpu6050_reg(void)
{
    test_setup();
    trace_init();
    i2c_init();
   
    uint8_t reg_only[] = {0x6B}; // Just register address
    i2c_write(MPU_ADDR, reg_only, 1, NULL);
    while (i2c_is_busy());
}

#define MUX_ADDR 0x70

void test_mux_simple(void)
{
    test_setup();
    trace_init();
    i2c_init();

    uint8_t select = 0x01; // for example, channel 1
    i2c_write(0x70, &select, 1, NULL);
    while (i2c_is_busy());
}

void test_mux_write_channel0(void)
{
    test_setup();
    trace_init();
    i2c_init();
        
    uint8_t select = 0x01;
    TRACE("Selecting MUX channel 0");
    i2c_write(MUX_ADDR, &select, 1, NULL);
    while (i2c_is_busy());
    TRACE("MUX channel 0 selected");
}

#define PCA9548A_ADDR   0x70       // Your I2C mux address (default is often 0x70)
#define MPU6050_ADDR    0x68  // Your MPU6050 address (default is 0x68)
#define MPU6050_WHOAMI  0x75           // WHOAMI register address

void test_mux_channel0_read(void)
{
    test_setup();
    trace_init();
    i2c_init();

    uint8_t mux_control = 0x01;    // Channel 0 = bit 0 = 1
    uint8_t whoami_reg = MPU6050_WHOAMI;
    uint8_t whoami_val = 0;

    // 1. Select channel 0 on PCA9548A:
    i2c_write(0x70, &mux_control, 1, NULL);
    while (i2c_is_busy());

    // 2. Point to WHOAMI register:
    i2c_write(0x68, &whoami_reg, 1, NULL);
    while (i2c_is_busy());

    // 3. Read WHOAMI register:
    i2c_read(0x68, &whoami_val, 1, NULL);
    while (i2c_is_busy());

    // 4. Print result:
    TRACE("MPU6050 WHOAMI (channel 0): 0x%02X", whoami_val);
    // Should print 0x68 if MPU6050 is responding
}

void test_imu_driver_init(void)
{
    test_setup();
    trace_init();
    i2c_init();

    imu_driver_t imu_driver;
    imu_driver_init(&imu_driver, I2C_MUX);

    imu_driver_select_channel(&imu_driver, 0);

    uint8_t whoami_reg = 0x75;
    uint8_t whoami_val = 0;
    
    i2c_write(MPU_6050_ADDR, &whoami_reg, 1, NULL);
    while (i2c_is_busy());    
    i2c_read(MPU_6050_ADDR, &whoami_val, 1, NULL);
    while (i2c_is_busy());    
    TRACE("WHOAMI: 0x%02X", whoami_val);   // Should be 0x68

    int16_t ax0, ay0, az0, ax1, ay1, az1, gx, gy, gz;

    while (1) {
        imu_driver_read_all(&imu_driver, 0, &ax0, &ay0, &az0, &gx, &gy, &gz);
        TRACE("CHANNEL %d: ax=%d, ay=%d, az=%d, gx=%d, gy=%d, gz=%d\n", 0, ax0, ay0, az0, gx, gy, gz);
        imu_driver_read_all(&imu_driver, 1, &ax1, &ay1, &az1, &gx, &gy, &gz);
        TRACE("CHANNEL %d: ax=%d, ay=%d, az=%d, gx=%d, gy=%d, gz=%d\n", 1, ax1, ay1, az1, gx, gy, gz);

        // float g0 = sqrt(ax0*ax0 + ay0*ay0 + az0*az0);
        // float g1 = sqrt(ax1*ax1 + ay1*ay1 + az1*az1);

        // float angle0 = acos(az0 / g0) * (180.0f / 3.14159f);
        // float angle1 = acos(az1 / g1) * (180.0f / 3.14159f);

        // TRACE("angle0=%d angle1=%d joint_angle=%d\n", (uint16_t) angle1, (uint16_t) angle0);
        BUSY_WAIT_ms(500);
    }

    // uint16_t imu_angles[3];
    // update_joint_angles(&imu_driver, imu_angles);
    // TRACE("imu_angles[0] is %d and imu_angles[1] is %d\n and imu_angles[2] is %d\n", imu_angles[0], imu_angles[1], imu_angles[2]);

}

void test_accel_data(void)
{
    test_setup();
    trace_init();
    i2c_init();

    imu_driver_t imu_driver;
    imu_driver_init(&imu_driver, I2C_MUX);

    int16_t ax, ay, az, gx, gy, gz;

    while (1) {
        imu_driver_read_all(&imu_driver, 4, &ax, &ay, &az, &gx, &gy, &gz);
        TRACE("ax=%d ay=%d az=%d gx=%d gy=%d gz=%d\n", ax, ay, az, gx, gy, gz);
        BUSY_WAIT_ms(500);
    }
}

void test_joint_angles(void)
{
    test_setup();
    trace_init();
    i2c_init();

    TRACE("Going to start initializing...\n");

    imu_driver_t imu_driver;
    imu_driver_init(&imu_driver, I2C_MUX);
    calibrate_gyro_biases(&imu_driver);
    
    servo_driver_t driver = {0};
    servo_driver_init(&driver, 0x40);

    TRACE("Done initializing...\n");

    float imu_angles[NUM_OF_JOINTS] = {0};  // <-- CHANGE HERE

    // Let Madgwick settle by updating all IMUs for 1 second before calibrating
    #define SETTLE_CYCLES 100
    #define UPDATE_PERIOD_MS 5
    for(int i=0;i<SETTLE_CYCLES;i++){
        for(int j=0;j<NUM_OF_IMU_SENSORS;j++){
            update_single_imu(&imu_driver, j, UPDATE_PERIOD_MS/1000.0f);
        }
        BUSY_WAIT_ms(2);
    }
    calibrate_joint_zero_pose(&imu_driver);

    TRACE("Starting...\n");

    uint32_t next_tick = get_ms() + UPDATE_PERIOD_MS;
    int current_imu = 0;

    while (1) {
        uint32_t now = get_ms();
        if (now < next_tick) continue;
        next_tick += UPDATE_PERIOD_MS;

        float dt = UPDATE_PERIOD_MS / 1000.0f; // dt is always 5ms in this example

        // Update just ONE IMU per tick
        update_single_imu(&imu_driver, current_imu, dt);
        current_imu++;

        if (current_imu >= NUM_OF_IMU_SENSORS) {
            current_imu = 0;
            // After updating ALL IMUs, now update the joint angles
            update_joint_angles(&imu_driver, imu_angles);

            // Example: Use joint for WRIST_WAVE
            float joint_angle = imu_angles[WRIST_WAVE]; // Or whichever joint you want

            // Clamp to range [-90, +90] for safety
            if(joint_angle < -90) joint_angle = -90;
            if(joint_angle >  90) joint_angle =  90;

            float servo_angle = joint_angle + 90; // map [-90,90] -> [0,180]
            servo_driver_set_servo_angle(&driver, 0, (uint8_t)servo_angle);

            // Or, iterate all IMU angles you're interested in:
            for (size_t i = 0; i < NUM_IMU_ANGLE_JOINTS; ++i) {
                joint_e j = imu_angle_joints[i];
                joint_angle = imu_angles[j];
                // Do something with joint_angle!
            }
        }
    }
}

void test_gpio_blink(void)
{
    test_setup();

    // GPIO Toggle test, blink every 1000 get_ms
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER |= (0x1 << (13 * 2));

    uint32_t last = get_ms();
    while (1) {
        if (get_ms() - last >= 1000) {
            last += 1000;
            GPIOC->ODR ^= (1 << 13);
        }
    }
}

#define PCA9548A_ADDR  0x70   // For example, check your jumpers/schematic!
#define CTRL_BYTE      0x01   // The control byte you want to write

void test_mux_scratch(void)
{

    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;         // Enable GPIOB clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;          // Enable I2C1 clock

    // Setup PB8/PB9 for AF4 Open Drain
    GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    GPIOB->MODER |= (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1); // AF mode

    GPIOB->OTYPER |= GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9; // open drain

    GPIOB->AFR[1] |= (4 << ((8-8)*4)) | (4 << ((9-8)*4)); // AF4

    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9); // no pullup

    // I2C peripheral config
    I2C1->CR1 = I2C_CR1_SWRST;      // Reset
    I2C1->CR1 = 0;                  // End reset

    I2C1->CR2   = 42;               // PCLK1 = 42MHz
    I2C1->CCR   = 210;              // (42MHz)/(2*100kHz)=210 for 100kHz
    I2C1->TRISE = 43;               // 42 + 1

    I2C1->CR1 |= I2C_CR1_PE;        // Enable peripheral

    // 1. Generate Start
I2C1->CR1 |= I2C_CR1_START;

// 2. Wait for Start bit sent (SB in SR1)
while (!(I2C1->SR1 & I2C_SR1_SB));

// 3. Send slave address (write: last bit 0)
I2C1->DR = (PCA9548A_ADDR << 1); // 0x70 << 1 == 0xE0

// 4. Wait for address acknowledged (ADDR in SR1 *or* NACK, AF in SR1)
//   - If slave ACKs, ADDR will be set.
//   - If NACK, AF will be set.
while (
    !(I2C1->SR1 & I2C_SR1_ADDR) &&  // Address acknowledged
    !(I2C1->SR1 & I2C_SR1_AF)       // NACK error
);

// 5. Did we get an ACK or NACK?
if (I2C1->SR1 & I2C_SR1_AF)
{
    // NACK! Slave didn't acknowledge.
    // Clean up: reset the AF bit by writing 0 and maybe send STOP.
    I2C1->SR1 &= ~I2C_SR1_AF;
    I2C1->CR1 |= I2C_CR1_STOP;
    // Do some error indication here (blink LED, etc)
}
else
{
    // ACK: Address phase was accepted.
    // Need to clear ADDR bit: read SR1/SR2
    (void)I2C1->SR2;

    // 6. Send your data byte (CTRL_BYTE)
    I2C1->DR = CTRL_BYTE;

    // 7. Wait for data register empty (TXE in SR1)
    while (!(I2C1->SR1 & I2C_SR1_TXE));

    // 8. Generate STOP condition
    I2C1->CR1 |= I2C_CR1_STOP;
    // Done!
}
}

// SUPPRESS_UNUSED
// void test_uart_potentiometer_readings(void){
//     test_setup();
//     trace_init();
//     adc_init();
//     //#if defined ROBOTIC_ARM
//     potentiometer_init();
//     // const struct potentiometer_reading dummy_readings[] = {
//     //     { .potentiometer_board = POTENTIOMETER_1, .angle = 45 },
//     //     { .potentiometer_board = POTENTIOMETER_2, .angle = 90 },
//     // };
//     uint16_t angles[1];
//     read_all_potentiometers(angles);
//     struct potentiometer_reading potentiometer_readings[1];
//     for(int i=0; i<1; i++){
//         potentiometer_readings[i].potentiometer_board = (joint_e)i;
//         potentiometer_readings[i].angle = angles[i];
//         TRACE("Potentiometer %u angle: %u", i, angles[i]);
//     }
//     while(1){
//         uart_send_potentiometer_readings(potentiometer_readings, 2);
//         BUSY_WAIT_ms(2000);
//     }
//     //#endif
// }

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

SUPPRESS_UNUSED
void test_potentiometer_to_servo(void){
    #if defined ROBOTIC_ARM
        test_setup();
        uart_init();
        trace_init();
        i2c_init();
        TRACE("Testing potentiometer to servo...");

        adc_init();
        potentiometer_init();
        uint8_t current_potentiometer = 0;
        joint_e potentiometers[] = { THUMB_DISTAL, THUMB_MIDDLE };

        servo_driver_t driver = { 0 };
        servo_channel_t channel = 0;
        servo_driver_init(&driver, 0x40);

        while(1){
            uint16_t angle_value = potentiometer_read(potentiometers[current_potentiometer]);
            TRACE("POTENTIOMETER %u READING: %u", current_potentiometer+1, angle_value);
            servo_driver_set_servo_angle(&driver, channel, angle_value);
            BUSY_WAIT_ms(100);
        }
    #endif
}

void test_uart_rx_buffer(void) 
{
    test_setup();
    uart_init();
    trace_init();

    TRACE("UART RX test ready!\n");
    while (1) {
        while (!ring_buffer_empty(&rx_buffer)) {
            uint8_t c = ring_buffer_get(&rx_buffer);
            _putchar(c);
        }
    }
}

SUPPRESS_UNUSED
void test_potentiometer_workflow(void)
{
    test_setup();
    trace_init();

    #if defined ARM_SLEEVE

    potentiometer_workflow_init();
    potentiometer_workflow_enable();
    //TRACE("Testing potentiometer_workflow for ARM_SLEEVE...");
    while(1){
        potentiometer_workflow_run();
        BUSY_WAIT_ms(1000);
    }
    #endif

    #if defined ROBOTIC_ARM

    potentiometer_workflow_init();
    potentiometer_workflow_enable();
    TRACE("Testing potentiometer_workflow for ROBOTIC_ARM...");
    while(1){
        potentiometer_workflow_run();
    }

    #endif
}

void test_get_ms(void)
{
    test_setup();

    while (1) {
        static uint32_t last = 0;
        if (get_ms() != last) {
            last = get_ms();
            GPIOC->ODR ^= (1 << 13); // Or whichever pin you have connected to your scope
        }
    }
}

void test_pwm_init(void)
{
    test_setup();
    trace_init();
    led_init();

    pwm_init();

    for (size_t i = 0; i < NUM_PWM_CHANNELS; ++i) {
        pwm_set_duty_cycle(pwm_joints[i], 50); // Set to 50% duty cycle
    }
}

void test_pwm(void)
{
    #define WAIT_TIME_MS 3000
    test_setup();
    trace_init();
    led_init();

    pwm_init();

    // Use pwm_joints from pwm.h
    const int duty_cycles[] = { 0, 20, 40, 60, 80, 100 };

    while (1) {
        for (uint8_t i = 0; i < sizeof(duty_cycles)/sizeof(duty_cycles[0]); i++) {
            TRACE("Set duty cycle to %d for %d ms\n", duty_cycles[i], WAIT_TIME_MS);
            led_set(LED_TEST, LED_STATE_ON);
            // set duty cycle for each joint
            for (uint8_t j = 0; j < NUM_PWM_CHANNELS; j++) {
                pwm_set_duty_cycle(pwm_joints[j], duty_cycles[i]);
            }
            BUSY_WAIT_ms(WAIT_TIME_MS);

            TRACE("Turning off PWM and waiting for %d ms\n", WAIT_TIME_MS);
            led_set(LED_TEST, LED_STATE_OFF);
            for (uint8_t j = 0; j < NUM_PWM_CHANNELS; j++) {
                pwm_set_duty_cycle(pwm_joints[j], 0);
            }
            BUSY_WAIT_ms(WAIT_TIME_MS);
        }
    }
}

int main(void)
{
    TEST();
    while (1) { }

    ASSERT(0);
}
