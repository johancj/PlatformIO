#include <Arduino.h>

#include "I2C_Driver.h"
#include "mpu6050.h"

int16_t x_val = 0;

void I2C_write(uint8_t device_address, uint8_t register_start_address, uint8_t data){
    
    //Serial1.println("0...");
    delay(1);
    
    //Start condition
    I2C1->CR1 |= I2C_CR1_START;
    while(!(I2C1->SR1 & I2C_SR1_SB_Msk)){} // Check for sent start condition
    //Serial1.println("1...");
    delay(1);

    //Send device adress
    I2C1->DR = device_address;
    while(!(I2C1->SR1 & I2C_SR1_ADDR_Msk)){} // Wait for sent device address
    //I2C1->SR2;
    I2C1->SR2;
    //Serial1.println("2...");
    delay(1);

    //send internal adress
    I2C1->DR = register_start_address;
    while(!(I2C1->SR1 & I2C_SR1_TXE)){} // Wait for byte transfer complete
    //Serial1.println("3...");
    delay(1);

    //write data
    I2C1->DR = data;
    while(!(I2C1->SR1 & I2C_SR1_TXE)){} // Wait for byte transfer complete
    //Serial1.println("4...");
    delay(1);

    //Stop condition
    I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C_write(uint8_t device_address, uint8_t register_start_address){
    
    Serial1.println("0...");
    delay(1);
    
    //Start condition
    I2C1->CR1 |= I2C_CR1_START;
    while(!(I2C1->SR1 & I2C_SR1_SB_Msk)){} // Check for sent start condition
    Serial1.println("1...");
    delay(1);

    //Send device adress
    I2C1->DR = device_address;
    while(!(I2C1->SR1 & I2C_SR1_ADDR_Msk)){} // Wait for sent device address
    I2C1->SR2;
    Serial1.println("2...");
    delay(1);

    //send internal adress
    I2C1->DR = register_start_address;
    while(!(I2C1->SR1 & I2C_SR1_TXE)){} // Wait for byte transfer complete
    Serial1.println("3...");
    delay(1);

    //Stop condition
    I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C_read(uint8_t device_address, uint8_t register_start_address, uint8_t data_length){
    
    // Write to set the internal device pointer at correct place
    I2C_write(device_address, register_start_address);
    

    // DMA things
    I2C1->CR2 |= I2C_CR2_DMAEN;
    I2C1->CR1 |= I2C_CR1_ACK;
    
    DMA1_Channel7->CPAR = (uint32_t)&I2C1->DR;
    DMA1_Channel7->CMAR = (uint32_t)&mpu6050_raw_data;
    DMA1_Channel7->CNDTR = data_length;
    DMA1_Channel7->CCR |= DMA_CCR_MINC | DMA_CCR_TCIE | DMA_CCR_EN;
    
    Serial1.println("r0...");
    delay(1);

    //Start condition
    I2C1->CR1 |= I2C_CR1_START;
    while(!(I2C1->SR1 & I2C_SR1_SB_Msk)){} // Check for sent start condition
    Serial1.println("r1...");
    delay(1);

    //Send device adress
    I2C1->DR = device_address + 1; // +1 because the least significant bit is the data direction bit. 1 is read.
    while(!(I2C1->SR1 & I2C_SR1_ADDR_Msk)){} // Wait for sent device address
    I2C1->SR2;
    Serial1.println("r2...");
    delay(1);
    
    while((DMA1->ISR & DMA_ISR_TCIF7) == 0);
    Serial1.println("r4...");
    delay(1);
    I2C1->CR1 |= I2C_CR1_STOP;
    DMA1_Channel7->CCR &= ~DMA_CCR_EN; //Disable DMA for I2C1.
} 


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
  delay(1);
  I2C_read(MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_L, 2);
  
  delay(1);
  x_val = (mpu6050_raw_data[0] << 8) | mpu6050_raw_data[1];
  Serial1.print("x_val = "); Serial1.println(x_val);
  delay(250);
  I2C_read(MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_L, 2);
  delay(1);
  x_val = (mpu6050_raw_data[0] << 8) | mpu6050_raw_data[1];
  Serial1.print("x_val = "); Serial1.println(x_val);
  delay(250);
}

void loop() {
  
}

