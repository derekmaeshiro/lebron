#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdint.h>

#define I2C_MUX 0x70 // A0, A1, A2 on GND pins
#define MPU_6050_ADDR 0x68 // AD0 on GND pin
#define IMU_DRIVER_MAX_CHANNELS 2

typedef enum {
    IMU_PALM = 0,
    IMU_LOWER_WRIST,
    IMU_UPPER_ELBOW,
    IMU_LOWER_ELBOW,
    IMU_LOWER_SHOULDER,
    IMU_UPPER_SHOULDER,
} imu_channel_t;

typedef enum {
    WRIST_PITCH,
    WRIST_ROLL,
    WRIST_YAW,
    ELBOW_PITCH,
    ELBOW_ROLL,
    ELBOW_YAW,
    SHOULDER_PITCH,
    SHOULDER_ROLL,
    SHOULDER_YAW,
} joint_t;

typedef struct
{
    imu_channel_t channel_number;
    float roll;
    float pitch;
    float yaw;
    float q0;
    float q1;
    float q2;
    float q3;
} imu_sensor_t;

typedef struct
{
    uint8_t slave_address;
    imu_sensor_t imu_sensors[IMU_DRIVER_MAX_CHANNELS];

} imu_driver_t;

void imu_driver_init(imu_driver_t *imu_driver, uint8_t slave_address);
void imu_driver_select_channel(const imu_driver_t *imu_driver, imu_channel_t channel);

// Raw Data
void imu_driver_read_all(const imu_driver_t *imu_driver, imu_channel_t channel, int16_t *ax, int16_t *ay,
                         int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);

// Angle Calculation
void calibrate_imus(const imu_driver_t *imu_driver);
void update_quaternion(imu_driver_t *imu_driver, imu_channel_t channel);
void convert_quaternion_to_euler(imu_driver_t *imu_driver, imu_channel_t channel);
void update_joint_angles(imu_driver_t *imu_driver, float *imu_angles);

#endif // IMU_DRIVER_H