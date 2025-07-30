#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stdbool.h>

typedef void (*i2c_callback_t)(int result); // 0 = success, -1 = error

void i2c_init(void);
bool i2c_is_busy(void);
bool i2c_write(uint8_t addr, const uint8_t *data, uint8_t len, i2c_callback_t cb);
bool i2c_read(uint8_t addr, uint8_t *data, uint8_t len, i2c_callback_t cb);

// Convenient wrapper functions
// i2c_result_e i2c_read_addr8_data8(uint8_t addr, uint8_t *data, i2c_callback_t callback);
// i2c_result_e i2c_read_addr8_data16(uint8_t addr, uint16_t *data, i2c_callback_t callback);
// i2c_result_e i2c_read_addr8_data32(uint8_t addr, uint32_t *data, i2c_callback_t callback);
// i2c_result_e i2c_write_addr8_data8(uint8_t addr, uint8_t data, i2c_callback_t callback);

#endif // I2C_H