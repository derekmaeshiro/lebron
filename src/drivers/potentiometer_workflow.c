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

#define LINE_BUF_SIZE 16
char readings_storage[NUM_OF_JOINTS][LINE_BUF_SIZE] = { { 0 } };
// // TODO: Edit back to not const
char *sent_readings[NUM_OF_JOINTS] = { 0 };
char line_buf[LINE_BUF_SIZE];
size_t line_pos = 0;
size_t count = 0;

static void poll_serial_and_store(void)
{
    while (!ring_buffer_empty(&rx_buffer)) {
        uint8_t c = ring_buffer_get(&rx_buffer);
        // Optional: strip carriage-return (for Windows-style lines)
        if (c == '\r') {
            // ignore carriage return
            continue;
        }
        if (c == '\n') {
            line_buf[line_pos] = '\0'; // null-terminate
            // Copy line_buf into next slot in readings_storage
            size_t index = count % NUM_OF_JOINTS;
            for (size_t i = 0; i < line_pos + 1 && i < LINE_BUF_SIZE; ++i) {
                readings_storage[index][i] = line_buf[i];
            }
            sent_readings[index] = readings_storage[index];
            count++;
            line_pos = 0; // reset for next line
        } else {
            if (line_pos < (LINE_BUF_SIZE - 1))
                line_buf[line_pos++] = c;
            // else: ignore character (buffer full)
        }
    }
}

void potentiometer_workflow_run(void)
{
    if (!workflow_enabled) {
        return;
    }

    poll_serial_and_store();

    // ---- Use Dummy Data for All Channels ----

    // 26 joints (19 potentiometer, 7 IMU-derived) expected.
    // The slave code only cares about NUM_OF_JOINTS count (==26).

    // const char *dummy_lines[NUM_OF_JOINTS] = {
    //     "0,0\n",  "1,100\n", "2,150\n", "3,45\n",   "4,100\n", "5,150\n", "6,45\n",
    //     "7,100\n",  "8,150\n", "9,45\n",  "10,100\n", "11,150\n", "12,45\n", "13,100\n",
    //     "14,150\n", "15,45\n", "16,100\n", "17,150\n", "18,45\n", // up to 18: potentiometers
    //     "19,75\n",  "20,85\n", "21,95\n", "22,105\n",  "23,115\n", "24,125\n", "25,135\n"
    //     // 19-25: stand-ins for IMU data, just example values
    // };
    // for (size_t i = 0; i < NUM_OF_JOINTS; i++) {
    //     sent_readings[i] = dummy_lines[i];
    // }

    struct potentiometer_reading received_potentiometer_readings[NUM_OF_JOINTS];

    for (size_t i = 0; i < NUM_OF_JOINTS; i++) {
        if (sent_readings[i] != NULL) {
            deserialize_potentiometer_reading(sent_readings[i],
                                              &received_potentiometer_readings[i]);
        } else {
            // Handle missing readings: set to zero, skip, etc.
            received_potentiometer_readings[i].potentiometer_board = (joint_e)i;
            received_potentiometer_readings[i].angle = 0;
        }
    }

    // Set the servo angles as received
    for (size_t i = 0; i < NUM_OF_JOINTS; i++) {
        struct potentiometer_reading reading = received_potentiometer_readings[i];
        servo_channel_name_t channel_name = (servo_channel_name_t)reading.potentiometer_board;
        servo_channel_t channel = (servo_channel_t)channel_name;
        servo_driver_set_servo_angle(&driver, channel, reading.angle);
    }
}

// void potentiometer_workflow_run(void)
// {
//     if (!workflow_enabled) {
//         return;
//     }

//     // TODO: Uncomment out eventually
//     // // Make sure we've received enough readings
//     // if (count < NUM_OF_JOINTS) {
//     //     // Not enough readings yet, handle as you wish
//     //     return;
//     // }

//     // poll_serial_and_store();

//     const char *dummy_lines[NUM_OF_JOINTS] = {
//         "0,135\n",
//         "1,100\n",
//         "2,150\n",
//         "3,45\n",
//         "4,100\n",
//         "5,150\n",
//         "6,45\n",
//         "7,100\n",
//         "8,150\n",
//         "9,45\n",
//         "10,100\n",
//         "11,150\n",
//         "12,45\n",
//         "13,100\n",
//         "14,150\n",
//         "15,45\n",
//         "16,100\n",
//         "17,150\n",
//         "18,45\n",
//         "19,45\n",
//         "20,150\n",
//         "21,150\n",
//         "22,150\n",
//         "23,150\n",
//         "24,150\n",
//         "25,150\n",
//     };
//     for (size_t i = 0; i < NUM_OF_JOINTS; i++) {
//         sent_readings[i] = dummy_lines[i];
//     }

//     struct potentiometer_reading received_potentiometer_readings[NUM_OF_JOINTS];

//     for (size_t i = 0; i < NUM_OF_JOINTS; i++) {
//         if (sent_readings[i] != NULL) {
//             deserialize_potentiometer_reading(sent_readings[i],
//                                               &received_potentiometer_readings[i]);
//         } else {
//             // Handle missing readings: set to zero, skip, etc.
//             received_potentiometer_readings[i].potentiometer_board = (joint_e)i;
//             received_potentiometer_readings[i].angle = 0;
//         }
//     }

//     // Set the servo angles as received
//     for (size_t i = 0; i < NUM_OF_JOINTS; i++) {
//         struct potentiometer_reading reading = received_potentiometer_readings[i];
//         servo_channel_name_t channel_name = (servo_channel_name_t)reading.potentiometer_board;
//         servo_channel_t channel = (servo_channel_t)channel_name;
//         servo_driver_set_servo_angle(&driver, channel, reading.angle);
//     }
// }
#endif