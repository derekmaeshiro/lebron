#include "imu_driver.h"
#include "uart.h"
#include "../common/trace.h"
#include "../common/assert_handler.h"
#include "../common/defines.h"
#include "i2c.h"
#include "led.h"
#include "MadgwickAHRS.h"

#include <math.h>

static void mpu6050_init(uint8_t slave_address)
{
    // Wake up MPU6050 (write 0x00 to register 0x6B)
    uint8_t buf[2] = { 0x6B, 0x00 };
    i2c_write(slave_address, buf, 2, NULL);
    while (i2c_is_busy()) { }
    // Additional configuration as needed
}

void imu_driver_init(imu_driver_t *imu_driver, uint8_t slave_address)
{
    if (!imu_driver)
        return; // Null pointer check

    imu_driver->slave_address = slave_address; // Set I2C mux address

    // Initialize all sensors to default values (you could also use memset)
    for (int i = IMU_PALM; i < IMU_DRIVER_MAX_CHANNELS; i++) {

        imu_driver->imu_sensors[i].channel_number = (imu_channel_t)i;
        imu_driver->imu_sensors[i].roll = 0;
        imu_driver->imu_sensors[i].pitch = 0;
        imu_driver->imu_sensors[i].yaw = 0;
        imu_driver->imu_sensors[i].q0 = 1.0f;
        imu_driver->imu_sensors[i].q1 = 0.0f;
        imu_driver->imu_sensors[i].q2 = 0.0f;
        imu_driver->imu_sensors[i].q3 = 0.0f;

        imu_driver_select_channel(imu_driver, i);
        mpu6050_init(MPU_6050_ADDR);
    }
}

void imu_driver_select_channel(imu_driver_t *imu_driver, imu_channel_t channel)
{
    uint8_t slave_address = imu_driver->slave_address;
    uint8_t control = 1 << channel;
    i2c_write(slave_address, &control, 1, NULL);
    while (i2c_is_busy()) { }
}

// Get data from IMU
void imu_driver_read_all(imu_driver_t *imu_driver, imu_channel_t channel, int16_t *ax, int16_t *ay,
                         int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    imu_driver_select_channel(imu_driver, channel);

    uint8_t reg = 0x3B;
    uint8_t data[14] = { 0 };

    i2c_write(MPU_6050_ADDR, &reg, 1, NULL);
    while (i2c_is_busy()) { }

    i2c_read(MPU_6050_ADDR, data, 14, NULL);
    while (i2c_is_busy()) { }

    *ax = (int16_t)((data[0] << 8) | data[1]);
    *ay = (int16_t)((data[2] << 8) | data[3]);
    *az = (int16_t)((data[4] << 8) | data[5]);
    *gx = (int16_t)((data[8] << 8) | data[9]);
    *gy = (int16_t)((data[10] << 8) | data[11]);
    *gz = (int16_t)((data[12] << 8) | data[13]);
}

int16_t gx_bias[IMU_DRIVER_MAX_CHANNELS] = { 0, 0 }, gy_bias[IMU_DRIVER_MAX_CHANNELS] = { 0, 0 },
        gz_bias[IMU_DRIVER_MAX_CHANNELS] = { 0, 0 };

void calibrate_imus(imu_driver_t *imu_driver)
{
    for (int channel = 0; channel < 2; channel++) {
        int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
        int n_samples = 200; // do 200 samples
        for (int i = 0; i < n_samples; i++) {
            int16_t ax, ay, az, gx, gy, gz;
            imu_driver_read_all(imu_driver, channel, &ax, &ay, &az, &gx, &gy, &gz);
            gx_sum += gx;
            gy_sum += gy;
            gz_sum += gz;
            BUSY_WAIT_ms(2);
        }
        gx_bias[channel] = gx_sum / n_samples;
        gy_bias[channel] = gy_sum / n_samples;
        gz_bias[channel] = gz_sum / n_samples;

        // TRACE("Bias CH%d: gx=%d gy=%d gz=%d\n", channel, gx_bias[channel], gy_bias[channel],
        // gz_bias[channel]);
    }
}

#define MPU6050_ACCEL_SENS_2G 16384.0f
#define MPU6050_GYRO_SENS_250 131.0f
#define DEG_TO_RAD (3.14159265359f / 180.0f)

void update_quaternion(imu_driver_t *imu_driver, imu_channel_t channel)
{
    int16_t ax, ay, az, gx, gy, gz;
    imu_driver_read_all(imu_driver, channel, &ax, &ay, &az, &gx, &gy, &gz);

    // TRACE("BEFORE | Accel CH%d: ax=%d ay=%d az=%d gx=%d gy=%d gz=%d\n", channel, ax, ay, az, gx,
    // gy, gz);

    gx -= gx_bias[channel];
    gy -= gy_bias[channel];
    gz -= gz_bias[channel];

    // TRACE("AFTER | Accel CH%d: ax=%d ay=%d az=%d gx=%d gy=%d gz=%d\n", channel, ax, ay, az, gx,
    // gy, gz);

    float ax_g = (float)ax / MPU6050_ACCEL_SENS_2G;
    float ay_g = (float)ay / MPU6050_ACCEL_SENS_2G;
    float az_g = (float)az / MPU6050_ACCEL_SENS_2G;

    float gx_rs = ((float)gx / MPU6050_GYRO_SENS_250) * DEG_TO_RAD;
    float gy_rs = ((float)gy / MPU6050_GYRO_SENS_250) * DEG_TO_RAD;
    float gz_rs = ((float)gz / MPU6050_GYRO_SENS_250) * DEG_TO_RAD;

    // TRACE("Accel CH%d: ax_g=%d ay_g=%d az_g=%d gx_g=%d gy_g=%d gz_g=%d\n", channel, ax, ay, az,
    // gx, gy, gz);

    float q0 = imu_driver->imu_sensors[channel].q0;
    float q1 = imu_driver->imu_sensors[channel].q1;
    float q2 = imu_driver->imu_sensors[channel].q2;
    float q3 = imu_driver->imu_sensors[channel].q3;

    // TRACE("Accel CH%d: q0=%d q1=%d q2=%d q3=%d\n", channel, (int)(q0 * 1000), (int)(q1 * 1000),
    // (int)(q2 * 1000), (int) (q3 * 1000));

    MadgwickAHRSupdate(gx_rs, gy_rs, gz_rs, ax_g, ay_g, az_g, 0.0, 0.0, 0.0, &q0, &q1, &q2, &q3);

    // // 1. Enable GPIOC clock
    // RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    // // 2. Set PC13 as output (clear MODER bits to 00, then set to 01)
    // GPIOC->MODER &= ~(0x3 << (13 * 2)); // Clear old mode
    // GPIOC->MODER |=  (0x1 << (13 * 2)); // Set output mode

    // // Optional: Set output speed, type, no pull-up/down
    // GPIOC->OTYPER &= ~(1 << 13);             // Push-pull
    // GPIOC->OSPEEDR |= (0x3 << (13 * 2));     // High speed
    // GPIOC->PUPDR &= ~(0x3 << (13 * 2));      // No pull, no up/down

    // GPIOC->ODR ^= (1 << 13);

    imu_driver->imu_sensors[channel].q0 = q0;
    imu_driver->imu_sensors[channel].q1 = q1;
    imu_driver->imu_sensors[channel].q2 = q2;
    imu_driver->imu_sensors[channel].q3 = q3;
}

void convert_quaternion_to_euler(imu_driver_t *imu_driver, imu_channel_t channel)
{
    float q0 = imu_driver->imu_sensors[channel].q0;
    float q1 = imu_driver->imu_sensors[channel].q1;
    float q2 = imu_driver->imu_sensors[channel].q2;
    float q3 = imu_driver->imu_sensors[channel].q3;
    // TRACE("Q: %d, %d, %d, %d\n",
    //   (int)(q0 * 1000), (int)(q1 * 1000), (int)(q2 * 1000), (int)(q3 * 1000));

    // Convert quaternion to Euler angles (in radians)
    float pitch = (float)atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
    float roll = (float)asin(2 * (q0 * q2 - q3 * q1));
    float yaw = (float)atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));

    // If you want degrees:
    pitch *= (180.0f / 3.14159265f);
    roll *= (180.0f / 3.14159265f);
    yaw *= (180.0f / 3.14159265f);

    // TRACE("Euler: pitch=%d roll=%d yaw=%d\n",
    //   (int)(pitch*1000), (int)(roll*1000), (int)(yaw*1000));

    // Write to your imu_driver struct, or global, etc:
    imu_driver->imu_sensors[channel].pitch = pitch;
    imu_driver->imu_sensors[channel].roll = roll;
    imu_driver->imu_sensors[channel].yaw = yaw;
}

// static float get_joint_angle(float imu_angle_1, float imu_angle_2)
// {
//     return imu_angle_2 - imu_angle_1; // signed difference
// }

static float get_joint_angle(float imu_angle_1, float imu_angle_2)
{
    float ROM_ori = imu_angle_2 - imu_angle_1;
    if (ROM_ori < 0)
        ROM_ori += 360; // Ensure positive angle

    float ROM_ij;
    if (ROM_ori > 180)
        ROM_ij = 360 - ROM_ori;
    else
        ROM_ij = ROM_ori;

    return ROM_ij;
}

void update_joint_angles(imu_driver_t *imu_driver, float *imu_angles)
{
    for (int channel = IMU_PALM; channel < IMU_DRIVER_MAX_CHANNELS; channel++) {
        update_quaternion(imu_driver, channel);
        convert_quaternion_to_euler(imu_driver, channel);
    }

    float pitch_palm = imu_driver->imu_sensors[IMU_PALM].pitch;
    float pitch_lower_wrist = imu_driver->imu_sensors[IMU_LOWER_WRIST].pitch;
    float roll_palm = imu_driver->imu_sensors[IMU_PALM].roll;
    float roll_lower_wrist = imu_driver->imu_sensors[IMU_LOWER_WRIST].roll;
    float yaw_palm = imu_driver->imu_sensors[IMU_PALM].yaw;
    float yaw_lower_wrist = imu_driver->imu_sensors[IMU_LOWER_WRIST].yaw;

    // float pitch_upper_elbow = imu_driver->imu_sensors[IMU_UPPER_ELBOW].pitch;
    // float pitch_lower_elbow = imu_driver->imu_sensors[IMU_LOWER_ELBOW].pitch;
    // float roll_upper_elbow = imu_driver->imu_sensors[IMU_UPPER_ELBOW].roll;
    // float roll_lower_elbow = imu_driver->imu_sensors[IMU_LOWER_ELBOW].roll;
    // float yaw_upper_elbow = imu_driver->imu_sensors[IMU_UPPER_ELBOW].yaw;
    // float yaw_lower_elbow = imu_driver->imu_sensors[IMU_LOWER_ELBOW].yaw;

    // float pitch_lower_shoulder = imu_driver->imu_sensors[IMU_LOWER_SHOULDER].pitch;
    // float pitch_upper_shoulder = imu_driver->imu_sensors[IMU_UPPER_SHOULDER].pitch;
    // float roll_lower_shoulder = imu_driver->imu_sensors[IMU_LOWER_SHOULDER].roll;
    // float roll_upper_shoulder = imu_driver->imu_sensors[IMU_UPPER_SHOULDER].roll;
    // float yaw_lower_shoulder = imu_driver->imu_sensors[IMU_LOWER_SHOULDER].yaw;
    // float yaw_upper_shoulder = imu_driver->imu_sensors[IMU_UPPER_SHOULDER].yaw;

    imu_angles[0] = get_joint_angle(pitch_palm, pitch_lower_wrist);
    imu_angles[1] = get_joint_angle(roll_palm, roll_lower_wrist);
    imu_angles[2] = get_joint_angle(yaw_palm, yaw_lower_wrist);

    // TRACE("Joint angles: pitch=%d roll=%d yaw=%d\n",
    //   (int)(imu_angles[0] * 1000),
    //   (int)(imu_angles[1] * 1000),
    //   (int)(imu_angles[2] * 1000));
    // imu_angles[3] = (uint16_t)get_joint_angle(pitch_upper_elbow, pitch_lower_elbow);
    // imu_angles[4] = (uint16_t)get_joint_angle(roll_upper_elbow, roll_lower_elbow);
    // imu_angles[5] = (uint16_t)get_joint_angle(yaw_upper_elbow, yaw_lower_elbow);
    // imu_angles[6] = (uint16_t)get_joint_angle(pitch_lower_shoulder, pitch_upper_shoulder);
    // imu_angles[7] = (uint16_t)get_joint_angle(roll_lower_shoulder, roll_upper_shoulder);
    // imu_angles[8] = (uint16_t)get_joint_angle(yaw_lower_shoulder, yaw_upper_shoulder);
}