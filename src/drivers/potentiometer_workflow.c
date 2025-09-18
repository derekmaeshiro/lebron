#include "uart.h"
#include "i2c.h"
#include "potentiometer.h"
#include "potentiometer_workflow.h"
#include "servo_driver.h"
#include "imu_driver.h"
#include "../common/defines.h"
#include "../common/assert_handler.h"
#include "../common/trace.h"
#include "../common/ring_buffer.h"
#include <stdbool.h>
#include <stddef.h> // for size_t

bool workflow_enabled = false;
static bool initialized = false;

#if defined ARM_SLEEVE
// static imu_driver_t imu_driver;
#endif

#if defined ROBOTIC_ARM

// static char line_buf[LINE_BUF_SIZE];
// static size_t line_pos = 0;
// static struct potentiometer_reading all_readings[SERVO_DRIVER_MAX_CHANNELS];
// static bool updated_channels[SERVO_DRIVER_MAX_CHANNELS];
// static size_t updated_count = 0;

static servo_driver_t driver = { 0 };
#endif

void potentiometer_workflow_init(void)
{
    ASSERT(!initialized);

    adc_init();
    uart_init();
    i2c_init();
    potentiometer_init();

#if defined ARM_SLEEVE
    calibrate_potentiometers();

    // imu_driver_init(&imu_driver, I2C_MUX);
    // calibrate_joint_zero_pose(&imu_driver);
#endif

#if defined ROBOTIC_ARM
    calibrate_potentiometers();

    servo_driver_init(&driver, 0x40);
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

    uint16_t angles[NUM_OF_JOINTS];
    read_all_potentiometers(angles);

    // float imu_angles[NUM_OF_JOINTS];
    // update_joint_angles(&imu_driver, imu_angles); //

    struct potentiometer_reading potentiometer_readings[NUM_OF_JOINTS];
    for (uint8_t i = 0; i < NUM_OF_JOINTS; i++) {
        potentiometer_readings[i] =
            (struct potentiometer_reading) { .potentiometer_board = (joint_e)i,
                                             .angle = angles[i] };
    }

    uart_send_potentiometer_readings(potentiometer_readings, NUM_OF_JOINTS);
}
#endif

#if defined ROBOTIC_ARM

#define POT_BATCH_SIZE NUM_OF_JOINTS
#define SERVO_MOVE_COUNT SERVO_DRIVER_MAX_CHANNELS

void potentiometer_workflow_run(void)
{
    if (!workflow_enabled)
        return;

    static struct potentiometer_reading batch[POT_BATCH_SIZE];
    static size_t batch_pos = 0;

    static char line_buf[LINE_BUF_SIZE];
    static size_t line_pos = 0;

    while (!ring_buffer_empty(&rx_buffer)) {
        uint8_t c = ring_buffer_get(&rx_buffer);

        if (c == '\r')
            continue;

        if (c == '\n') {
            line_buf[line_pos] = '\0';

            struct potentiometer_reading reading;
            deserialize_potentiometer_reading(line_buf, &reading);
            line_pos = 0;

            if (batch_pos < POT_BATCH_SIZE) {
                batch[batch_pos++] = reading;
            }

            // When we hit a full batch, move all servos
            if (batch_pos == POT_BATCH_SIZE) {
                for (size_t i = 0; i < SERVO_MOVE_COUNT; i++) {
                    // Be sure that batch[i] actually corresponds to the correct servo channel if
                    // ordering isn't guaranteed
                    servo_driver_set_servo_angle(&driver, (servo_channel_t)i, batch[i].angle);
                }
                batch_pos = 0; // Reset for next batch
            }

        } else {
            if (line_pos < LINE_BUF_SIZE - 1)
                line_buf[line_pos++] = c;
            else
                line_pos = 0; // Overflow: reset buffer
        }
    }
}

#endif