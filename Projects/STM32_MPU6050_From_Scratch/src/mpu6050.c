#include <stm32f1xx.h>
#include <stdio.h>

#include "mpu6050.h"
#include "I2C_Driver.h"


void mpu6050_init(void){
    I2C_init();

    // Initialisating MPU6050
    I2C_write(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x00);    // Power on
    I2C_write(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0x08);   // Max 500 deg/s
    I2C_write(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x08);  // Max 4 g

    // Set gyroscope sample rate 1kHz (8kHz with averaging should be considered)

    // Digital low pass filter with latency is 0.98ms for 256Hz cutoff frequency. 
    I2C_write(MPU6050_ADDRESS, MPU6050_RA_CONFIG, 0x00);

    // Interrupt configuration: Active high, push-pull, INT high untill cleared, INT status cleared by reading INT_STATUS, 
    I2C_write(MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 0b00100000);
    I2C_write(MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 0x01); // Enable interrupt on data ready

    // Consider need for zero-rate compensation.

    // Temperature compensation (Yes, important when not using DMP!)?

    // Use DMP (Digital Motion Processor) do whats mentioned above (If latency and refresh rate not too bad). DMP has very bad documentation..

    // Interrupt from DMP task complete?

}

