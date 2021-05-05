#include <Arduino.h>

#include "I2C_Driver.h"

int16_t x_val = 0;

void I2C_write(uint8_t device_address, uint8_t register_start_address, uint8_t data){
    
    uint32_t garbage = 0;
    //Start condition
    I2C1->CR1 |= I2C_CR1_START;
    while(!(I2C1->SR1 & I2C_SR1_SB_Msk)){} // Check for sent start condition
    Serial1.println("1...");
    delay(1);

    //Send device adress
    I2C1->DR = device_address << 1;
    while(!(I2C1->SR1 & I2C_SR1_ADDR_Msk)){} // Wait for sent device address
    //I2C1->SR2;
    garbage = I2C1->SR2;
    Serial1.println("2...");
    delay(1);

    //send internal adress
    I2C1->DR = register_start_address;
    while(!(I2C1->SR1 & I2C_SR1_TXE)){} // Wait for byte transfer complete
    Serial1.println("3...");
    delay(1);

    //write data
    I2C1->DR = data;
    while(!(I2C1->SR1 & I2C_SR1_TXE)){} // Wait for byte transfer complete
    Serial1.println("4...");
    delay(1);

    //Stop condition
    I2C1->CR1 |= I2C_CR1_STOP;
}

void setup() {


  Serial1.begin(115200);
  I2C_init();
  delay(2500);
  Serial1.println("Kontakter gyroen...");
  I2C_write(MPU6050_ADDRESS, 0x6B, 0x00);
  delay(500);
  Serial1.println("Leser gyroen...");
  delay(5);
  I2C_read(MPU6050_ADDRESS, 0x3B);
  x_val = (mpu6050_raw_data[0] << 8) | mpu6050_raw_data[1];
  Serial1.println(x_val);
}

void loop() {
  // put your main code here, to run repeatedly:
}

