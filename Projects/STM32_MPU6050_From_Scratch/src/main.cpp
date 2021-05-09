#include <Arduino.h>

extern "C"{
#include "I2C_Driver.h"
#include "mpu6050.h"
}


/*
This code works. However, mpu6050 lib needs some refinement.
*/


int16_t z_val = 0;


void setup() {


  Serial1.begin(115200);
  delay(2500);
  Serial1.println("MPU6050_init()");
  
  mpu6050_init();
  
  Serial1.println("Reading the gyro...");
  delay(2);

}

void loop() {
  I2C_read(MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, 2);
  z_val = (mpu6050_raw_data[0] << 8) | mpu6050_raw_data[1];
  Serial1.print("z_raw_val = "); Serial1.println(z_val);
  delay(250);
}

