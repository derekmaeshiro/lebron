#include <stdint.h>

typedef enum {
    IMU_SCL,
    IMU_SDA
} imu_e;

void imu_init(void);
uint16_t read_imu(imu_e imu);