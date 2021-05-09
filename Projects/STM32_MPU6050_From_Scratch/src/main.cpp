#include <Arduino.h>

#include "I2C_Driver.h"
#include "mpu6050.h"

int16_t x_val = 0;




void setup() {


  Serial1.begin(115200);
  delay(2500);
  Serial1.println("I2C_init()");
  I2C_init();
  
  Serial1.println("Turn on mpu6050...");
  
  I2C_write(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x00);
  I2C_write(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0x08); // max 500 deg/s
  I2C_write(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x08); // max 4 g
  delay(500);

  Serial1.println("Reading the gyro...");
  delay(2);

}

void loop() {
  I2C_read(MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, 2);
  x_val = (mpu6050_raw_data[0] << 8) | mpu6050_raw_data[1];
  Serial1.print("x_val = "); Serial1.println(x_val);
  delay(250);
}

