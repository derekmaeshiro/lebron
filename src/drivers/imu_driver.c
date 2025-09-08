#include "imu_driver.h"
// #include "uart.h"
#include "../common/trace.h"
#include "../common/assert_handler.h"
#include "../common/defines.h"
#include "i2c.h"
#include "led.h"
#include "MadgwickAHRS.h"

#include <math.h>

const joint_e imu_angle_joints[] = {
    WRIST_WAVE,
    WRIST_FLICK,
    WRIST_NAE_NAE,
    ELBOW,
    BICEP,
    SHOULDER_FRONT_RAISE,
    SHOULDER_LAT_RAISE,
};

const size_t NUM_IMU_ANGLE_JOINTS = sizeof(imu_angle_joints) / sizeof(joint_e);

static void mpu6050_init(uint8_t slave_address)
{
    // Wake up MPU6050 (write 0x00 to register 0x6B)
    const uint8_t buf[2] = { 0x6B, 0x00 };
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
    for (int i = IMU_PALM; i < NUM_OF_IMU_SENSORS; i++) {

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

void imu_driver_select_channel(const imu_driver_t *imu_driver, imu_channel_t channel)
{
    uint8_t slave_address = imu_driver->slave_address;
    uint8_t control = 1 << channel;
    i2c_write(slave_address, &control, 1, NULL);
    while (i2c_is_busy()) { }
}

// Get data from IMU
void imu_driver_read_all(const imu_driver_t *imu_driver, imu_channel_t channel, int16_t *ax,
                         int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
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

int16_t gx_bias[NUM_OF_IMU_SENSORS] = { 0, 0, 0, 0, 0, 0 },
        gy_bias[NUM_OF_IMU_SENSORS] = { 0, 0, 0, 0, 0, 0 },
        gz_bias[NUM_OF_IMU_SENSORS] = { 0, 0, 0, 0, 0, 0 };

#define NUM_GYRO_CAL_SAMPLES 100

void calibrate_gyro_biases(const imu_driver_t *imu_driver)
{
    for (int channel = 0; channel < NUM_OF_IMU_SENSORS; ++channel) {
        int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
        for (int i = 0; i < NUM_GYRO_CAL_SAMPLES; ++i) {
            int16_t ax, ay, az, gx, gy, gz;
            imu_driver_read_all(imu_driver, channel, &ax, &ay, &az, &gx, &gy, &gz);
            gx_sum += gx;
            gy_sum += gy;
            gz_sum += gz;
            BUSY_WAIT_ms(2); // let sensor settle
        }
        gx_bias[channel] = gx_sum / NUM_GYRO_CAL_SAMPLES;
        gy_bias[channel] = gy_sum / NUM_GYRO_CAL_SAMPLES;
        gz_bias[channel] = gz_sum / NUM_GYRO_CAL_SAMPLES;
    }
}

#define MPU6050_ACCEL_SENS_2G 16384.0f
#define MPU6050_GYRO_SENS_250 131.0f
#define DEG_TO_RAD (3.14159265359f / 180.0f)

void update_quaternion(imu_driver_t *imu_driver, imu_channel_t channel, float dt)
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

    MadgwickAHRSupdate(gx_rs, gy_rs, gz_rs, ax_g, ay_g, az_g, 0.0, 0.0, 0.0, &q0, &q1, &q2, &q3,
                       dt);

    GPIOC->ODR ^= (1 << 14);

    imu_driver->imu_sensors[channel].q0 = q0;
    imu_driver->imu_sensors[channel].q1 = q1;
    imu_driver->imu_sensors[channel].q2 = q2;
    imu_driver->imu_sensors[channel].q3 = q3;
}

// q1: proximal segment quaternion (e.g. upper arm), q2: distal (e.g. forearm)
static void quaternion_conjugate(const float *q, float *qc)
{
    qc[0] = q[0];
    qc[1] = -q[1];
    qc[2] = -q[2];
    qc[3] = -q[3];
}

// quaternion multiplication: q_out = q1 * q2
static void quaternion_multiply(const float *q1, const float *q2, float *q_out)
{
    q_out[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    q_out[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    q_out[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    q_out[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
}

float get_joint_angle_quaternion(const imu_sensor_t *imu_proximal, // e.g., upper arm
                                 const imu_sensor_t *imu_distal, // e.g., forearm
                                 char axis // 'p'/pitch, 'r'/roll, 'y'/yaw
)
{
    const float qA[4] = { imu_proximal->q0, imu_proximal->q1, imu_proximal->q2, imu_proximal->q3 };
    const float qB[4] = { imu_distal->q0, imu_distal->q1, imu_distal->q2, imu_distal->q3 };
    float qA_conj[4], q_rel[4];
    quaternion_conjugate(qA, qA_conj);
    quaternion_multiply(qB, qA_conj, q_rel);

    // Convert q_rel to Euler
    float roll = atan2(2 * (q_rel[0] * q_rel[1] + q_rel[2] * q_rel[3]),
                       1 - 2 * (q_rel[1] * q_rel[1] + q_rel[2] * q_rel[2]));
    float pitch = asin(2 * (q_rel[0] * q_rel[2] - q_rel[3] * q_rel[1]));
    float yaw = atan2(2 * (q_rel[0] * q_rel[3] + q_rel[1] * q_rel[2]),
                      1 - 2 * (q_rel[2] * q_rel[2] + q_rel[3] * q_rel[3]));

    pitch *= (180.0f / 3.14159265f);
    roll *= (180.0f / 3.14159265f);
    yaw *= (180.0f / 3.14159265f);

    switch (axis) {
    case 'p':
        return pitch; // elbow flexion/extension, shoulder flexion
    case 'r':
        return roll; // wrist wave, etc.
    case 'y':
        return yaw; // pronation/supination, shoulder rotation
    default:
        return pitch;
    }
}

void calibrate_joint_zero_pose(imu_driver_t *imu_driver)
{
    // Capture current joint angles as reference
    imu_driver->zero_pose[WRIST_WAVE] = get_joint_angle_quaternion(
        &imu_driver->imu_sensors[IMU_PALM], &imu_driver->imu_sensors[IMU_LOWER_WRIST], 'p');
    imu_driver->zero_pose[WRIST_FLICK] = get_joint_angle_quaternion(
        &imu_driver->imu_sensors[IMU_PALM], &imu_driver->imu_sensors[IMU_LOWER_WRIST], 'y');
    imu_driver->zero_pose[WRIST_NAE_NAE] = get_joint_angle_quaternion(
        &imu_driver->imu_sensors[IMU_PALM], &imu_driver->imu_sensors[IMU_LOWER_WRIST], 'r');
    imu_driver->zero_pose[ELBOW] = get_joint_angle_quaternion(
        &imu_driver->imu_sensors[IMU_UPPER_ELBOW], &imu_driver->imu_sensors[IMU_LOWER_ELBOW], 'p');
    imu_driver->zero_pose[BICEP] = get_joint_angle_quaternion(
        &imu_driver->imu_sensors[IMU_UPPER_ELBOW], &imu_driver->imu_sensors[IMU_LOWER_ELBOW], 'r');
    imu_driver->zero_pose[SHOULDER_FRONT_RAISE] =
        get_joint_angle_quaternion(&imu_driver->imu_sensors[IMU_LOWER_SHOULDER],
                                   &imu_driver->imu_sensors[IMU_UPPER_SHOULDER], 'y');
    imu_driver->zero_pose[SHOULDER_LAT_RAISE] =
        get_joint_angle_quaternion(&imu_driver->imu_sensors[IMU_LOWER_SHOULDER],
                                   &imu_driver->imu_sensors[IMU_UPPER_SHOULDER], 'p');
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
    float roll = (float)atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
    float pitch = (float)asin(2 * (q0 * q2 - q3 * q1));
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

// static float get_joint_angle(float imu_angle_1, float imu_angle_2)
// {
//     float ROM_ori = imu_angle_2 - imu_angle_1;
//     if (ROM_ori < 0)
//         ROM_ori += 360; // Ensure positive angle

//     float ROM_ij;
//     if (ROM_ori > 180)
//         ROM_ij = 360 - ROM_ori;
//     else
//         ROM_ij = ROM_ori;

//     return ROM_ij;
// }

// Wrap angle to [-180,180]
float normalize_angle(float angle)
{
    while (angle < -180.0f)
        angle += 360.0f;
    while (angle > 180.0f)
        angle -= 360.0f;
    return angle;
}

void update_single_imu(imu_driver_t *imu_driver, int channel, float dt)
{
    update_quaternion(imu_driver, channel, dt);
    convert_quaternion_to_euler(imu_driver, channel);
    // Optionally blink LED to show which one updated
    GPIOC->ODR ^= (1 << 13);
}

void update_joint_angles(imu_driver_t *imu_driver, float *imu_angles)
{
    imu_angles[WRIST_WAVE] =
        normalize_angle(get_joint_angle_quaternion(&imu_driver->imu_sensors[IMU_PALM],
                                                   &imu_driver->imu_sensors[IMU_LOWER_WRIST], 'p')
                        - imu_driver->zero_pose[WRIST_WAVE]);

    imu_angles[WRIST_FLICK] =
        normalize_angle(get_joint_angle_quaternion(&imu_driver->imu_sensors[IMU_PALM],
                                                   &imu_driver->imu_sensors[IMU_LOWER_WRIST], 'y')
                        - imu_driver->zero_pose[WRIST_FLICK]);

    imu_angles[WRIST_NAE_NAE] =
        normalize_angle(get_joint_angle_quaternion(&imu_driver->imu_sensors[IMU_PALM],
                                                   &imu_driver->imu_sensors[IMU_LOWER_WRIST], 'r')
                        - imu_driver->zero_pose[WRIST_NAE_NAE]);

    imu_angles[ELBOW] =
        normalize_angle(get_joint_angle_quaternion(&imu_driver->imu_sensors[IMU_UPPER_ELBOW],
                                                   &imu_driver->imu_sensors[IMU_LOWER_ELBOW], 'p')
                        - imu_driver->zero_pose[ELBOW]);

    imu_angles[BICEP] =
        normalize_angle(get_joint_angle_quaternion(&imu_driver->imu_sensors[IMU_UPPER_ELBOW],
                                                   &imu_driver->imu_sensors[IMU_LOWER_ELBOW], 'r')
                        - imu_driver->zero_pose[BICEP]);

    imu_angles[SHOULDER_FRONT_RAISE] = normalize_angle(
        get_joint_angle_quaternion(&imu_driver->imu_sensors[IMU_LOWER_SHOULDER],
                                   &imu_driver->imu_sensors[IMU_UPPER_SHOULDER], 'y')
        - imu_driver->zero_pose[SHOULDER_FRONT_RAISE]);

    imu_angles[SHOULDER_LAT_RAISE] = normalize_angle(
        get_joint_angle_quaternion(&imu_driver->imu_sensors[IMU_LOWER_SHOULDER],
                                   &imu_driver->imu_sensors[IMU_UPPER_SHOULDER], 'p')
        - imu_driver->zero_pose[SHOULDER_LAT_RAISE]);
}