#include "i2c.h"
#include "potentiometer.h"
#include "potentiometer_workflow.h"
#include "servo_driver.h"
#include "uart.h"
#include "../common/assert_handler.h"
#include "../common/trace.h"
#include <stdbool.h>

#if defined ROBOTIC_ARM
static servo_driver_t driver = { 0 };
#endif

bool workflow_enabled = false;
static bool initialized = false;
void potentiometer_workflow_init(void)
{
    ASSERT(!initialized);

    trace_init();
// Initialize the dependent drivers
#if defined ARM_SLEEVE
    adc_init();
    i2c_init();
    potentiometer_init();
    uart_init();
#endif
#if defined ROBOTIC_ARM
    i2c_init();
    potentiometer_init();
    servo_driver_init(&driver, 0x40);
    uart_init();
#endif

    initialized = true;
}

void potentiometer_workflow_enable(void)
{
    ASSERT(!workflow_enabled);
    workflow_enabled = true;
}

void potentiometer_workflow_disable(void)
{
    ASSERT(workflow_enabled);

    workflow_enabled = false;
}

#if defined ARM_SLEEVE
void potentiometer_workflow_run(void)
{
    if (!workflow_enabled) {
        return;
    }
    uint16_t angles[NUM_OF_POTENTIOMETERS];
    read_all_potentiometers(angles);

    struct potentiometer_reading potentiometer_readings[NUM_OF_POTENTIOMETERS];
    for (uint8_t i = 0; i < NUM_OF_POTENTIOMETERS; i++) {
        potentiometer_readings[i] =
            (struct potentiometer_reading) { .potentiometer_board = (potentiometer_e)i,
                                             .angle = angles[i] };
    }
    uart_send_potentiometer_readings(potentiometer_readings, NUM_OF_POTENTIOMETERS);
}
#endif

#if defined ROBOTIC_ARM
void potentiometer_workflow_run(void)
{
    if (!workflow_enabled) {
        return;
    }
    // TODO: Replace the dummy readings with actual readings
    const char *sent_readings[] = {
        "0,45\n", "1,90\n", "2,70\n",   "3,180\n",  "4,30\n",  "5,60\n",  "6,120\n", "7,150\n",
        "8,15\n", "9,75\n", "10,135\n", "11,165\n", "12,25\n", "13,55\n", "14,95\n", "15,105\n",
    };

    // Deserialize the readings and place in received_potentiometer_readings array
    struct potentiometer_reading received_potentiometer_readings[NUM_OF_POTENTIOMETERS];
    for (int i = 0; i < NUM_OF_POTENTIOMETERS; i++) {
        deserialize_potentiometer_reading(sent_readings[i], &received_potentiometer_readings[i]);
    }

    // Set the servo angles
    for (int i = 0; i < NUM_OF_POTENTIOMETERS; i++) {
        struct potentiometer_reading reading = received_potentiometer_readings[i];
        servo_channel_name_t channel_name = (servo_channel_name_t)reading.potentiometer_board;
        servo_channel_t channel = (servo_channel_t)channel_name;
        servo_driver_set_servo_angle(&driver, channel, reading.angle);
    }
}
#endif