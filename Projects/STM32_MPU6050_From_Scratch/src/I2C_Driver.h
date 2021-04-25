#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#define MPU6050_ADDRESS 0x68

uint8_t mpu6050_raw_data[14];

void I2C_init(void);

void I2C_write(uint8_t device_address, uint8_t register_start_address, uint8_t data);

#endif //I2C_DRIVER_H