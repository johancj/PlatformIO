#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#define MPU6050_ADDRESS 0xD0 // 8 bit write device address (7bit address << 1)

extern uint8_t mpu6050_raw_data[2];

void I2C_init(void);

//void I2C_write(uint8_t device_address, uint8_t register_start_address, uint8_t data);

//void I2C_read(uint8_t device_address, uint8_t register_start_address);

#endif //I2C_DRIVER_H