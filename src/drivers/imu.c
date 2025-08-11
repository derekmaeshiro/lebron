#include "imu.h"
#include "io.h"
#include "i2c.h"
#include "led.h"
#include <stm32f4xx.h>
#include "../common/assert_handler.h"
#include "../common/defines.h"
#include "../common/trace.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43

// Blocking I2C write to 1 register
static void imu_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t data[2] = { reg, val };
    TRACE("Writing 0x%02X to reg 0x%02X\n", val, reg);
    i2c_write(MPU6050_ADDR << 1, data, 2, NULL);
    led_init();
    while (i2c_is_busy()) {
        led_set(LED_TEST, LED_STATE_ON);
        BUSY_WAIT_ms(2000);
        led_set(LED_TEST, LED_STATE_OFF);
        BUSY_WAIT_ms(2000);
    }
}

// Blocking I2C read from 1 register
// static uint8_t imu_read_reg(uint8_t reg) {
//     uint8_t val;
//     i2c_write(MPU6050_ADDR << 1, &reg, 1, NULL);
//     while (i2c_is_busy()) { }

//     i2c_read(MPU6050_ADDR << 1, &val, 1, NULL);
//     while (i2c_is_busy()) { }

//     return val;
// }

// Blocking I2C read from 2 consecutive registers (e.g., accel x)
static uint16_t imu_read_reg16(uint8_t reg)
{
    uint8_t buf[2];
    i2c_write(MPU6050_ADDR << 1, &reg, 1, NULL);
    while (i2c_is_busy()) { }

    i2c_read(MPU6050_ADDR << 1, buf, 2, NULL);
    while (i2c_is_busy()) { }

    return (buf[0] << 8) | buf[1];
}

static bool initialized = false;
void imu_init(void)
{
    ASSERT(!initialized);

    // Wake up the MPU6050 (clear sleep bit)
    uint8_t payload[2] = { MPU6050_PWR_MGMT_1, 0x00 };
    i2c_write(MPU6050_ADDR, payload, 2, NULL);

    volatile int crude_delay = 100000;
    TRACE("Waiting for IMU init...\n");
    while (i2c_is_busy()) {
        crude_delay--;
        if (crude_delay < 0) {
            break;
        }
    }

    initialized = true;
}

uint16_t read_imu(imu_e imu)
{
    imu_write_reg(MPU6050_PWR_MGMT_1, 0x00);
    switch (imu) {
    case IMU_SCL: // ACCEL_X
        return imu_read_reg16(MPU6050_ACCEL_XOUT_H);
    case IMU_SDA: // GYRO_X
        return imu_read_reg16(MPU6050_GYRO_XOUT_H);
    default:
        return 0;
    }
}