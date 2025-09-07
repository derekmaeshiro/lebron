#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdint.h>

#define I2C_MUX 0x70 // A0, A1, A2 on GND pins
#define MPU_6050_ADDR 0x68 // AD0 on GND pin
#define IMU_DRIVER_MAX_CHANNELS 8

typedef enum {
    IMU_PALM = 0,
    IMU_LOWER_WRIST,
    IMU_UPPER_ELBOW,
    IMU_LOWER_ELBOW,
    IMU_LOWER_SHOULDER,
    IMU_UPPER_SHOULDER,
    NUM_OF_IMU_SENSORS,
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

typedef enum {
    WRIST_WAVE,
    WRIST_FLICK,
    WRIST_NAE_NAE,
    ELBOW,
    BICEP,
    SHOULDER_FRONT_RAISE,
    SHOULDER_LAT_RAISE,
    NUM_OF_IMU_ANGLES,
} imu_angle_t;

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
    imu_sensor_t imu_sensors[NUM_OF_IMU_SENSORS];
    float zero_pose[NUM_OF_IMU_ANGLES];
} imu_driver_t;

void imu_driver_init(imu_driver_t *imu_driver, uint8_t slave_address);
void imu_driver_select_channel(const imu_driver_t *imu_driver, imu_channel_t channel);

// Raw Data
void imu_driver_read_all(const imu_driver_t *imu_driver, imu_channel_t channel, int16_t *ax,
                         int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);

// Angle Calculation
void calibrate_gyro_biases(const imu_driver_t *imu_driver);
void calibrate_joint_zero_pose(imu_driver_t *imu_driver);
void update_quaternion(imu_driver_t *imu_driver, imu_channel_t channel, float dt);
void convert_quaternion_to_euler(imu_driver_t *imu_driver, imu_channel_t channel);
void update_single_imu(imu_driver_t *imu_driver, int channel, float dt);
float get_joint_angle_quaternion(imu_sensor_t *imu_proximal, // e.g., upper arm
                                 imu_sensor_t *imu_distal, // e.g., forearm
                                 char axis // 'p'/pitch, 'r'/roll, 'y'/yaw
);
void update_joint_angles(imu_driver_t *imu_driver, float *imu_angles);

#endif // IMU_DRIVER_H