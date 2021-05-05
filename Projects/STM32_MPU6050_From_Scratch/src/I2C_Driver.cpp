#include <stm32f1xx.h>
#include <stdio.h>


#include "I2C_Driver.h"

uint8_t mpu6050_raw_data[2];

void I2C_init(void){
    //Initiating I2C1

    I2C1->CR1 = 0UL;
    I2C1->CR2 = 0UL;
    //Enable I2C pheripheral clock
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; //Enable I2C1 clock


    //DMA init
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;  //Enable DMA clock
    

    I2C1->CR2 |= (36 << I2C_CR2_FREQ_Pos); //Defines the I2C clock = APB1 buss clk = 36MHz

    /*
    Calculating the I2C clock control in fast mode (fm, 400kHz)
    T_pclk1 = 1/APB1_clk = 1/36MHz = 27.7778ns 
    1/400kHz = 2500ns (total)
    1/3*2500ns = 833.3ns (t_high)
    2/3*2500ns = 1666.67ns (T_low)
        
    ->    CCR = t_high / T_pclk1 = 833.3 / 27.778 ~= 30

    PE needs to be 0 for the following to work.
    */
    I2C1->CCR = 0;
    I2C1->CCR |= (I2C_CCR_FS) | (30 << I2C_CCR_CCR_Pos);
    I2C1->TRISE = 37; // 300ns/27.7778ns + 1 = 11.79 = 12. (300ns is max t_rise for MPU6050. generic from MCU datasheet: 1000us gives 37.)

    I2C1->CR1 |= I2C_CR1_ACK; // Enable acknowlage

    //Strech mode enabled by default

    // 7 bit addressing mode by default

    //GPIO config for I2C1 PB6 and PB7 alernate function open drain.
    GPIOB->CRL |= GPIO_CRL_CNF6 | GPIO_CRL_CNF7 | GPIO_CRL_MODE6 | GPIO_CRL_MODE7;

    I2C1->CR1 |= I2C_CR1_PE; //Enables I2C1 pheripheral. Needs to be done at last.


}

/* void I2C_write(uint8_t device_address, uint8_t register_start_address, uint8_t data){
    
    uint32_t garbage = 0;
    //Start condition
    I2C1->CR1 |= I2C_CR1_START;
    while(!(I2C1->SR1 & I2C_SR1_SB_Msk)){} // Check for sent start condition
    
    //Send device adress
    I2C1->DR = device_address;
    while(!(I2C1->SR1 & I2C_SR1_ADDR_Msk)){} // Wait for sent device address
    I2C1->SR2;
    garbage = I2C1->SR2;
    
    //send internal adress
    I2C1->DR = register_start_address;
    while(!(I2C1->SR1 & I2C_SR1_TXE)){} // Wait for byte transfer complete

    //write data
    I2C1->DR = data;
    while(!(I2C1->SR1 & I2C_SR1_TXE)){} // Wait for byte transfer complete

    //Stop condition
    I2C1->CR1 |= I2C_CR1_STOP;
} */


void I2C_read(uint8_t device_address, uint8_t register_start_address){
    
    //uint32_t garbage = 0;

    // DMA things
    I2C1->CR2 |= I2C_CR2_DMAEN;
    DMA1_Channel7->CNDTR = 2;
    DMA1_Channel7->CPAR = (uint32_t)&I2C1->DR;
    DMA1_Channel7->CMAR = (uint32_t)&mpu6050_raw_data;
    DMA1_Channel7->CCR |= DMA_CCR_MINC | DMA_CCR_TCIE | DMA_CCR_EN;

    //Start condition
    I2C1->CR1 |= I2C_CR1_START;
    while(!(I2C1->SR1 & I2C_SR1_SB_Msk)){} // Check for sent start condition
    
    //Send device adress
    I2C1->DR = (device_address << 1) + 1; // +1 because the least significant bit is the data direction bit. 1 is read.
    while(!(I2C1->SR1 & I2C_SR1_ADDR_Msk)){} // Wait for sent device address
    I2C1->SR2;
    //garbage = I2C1->SR2;

    //send internal adress
    I2C1->DR = register_start_address;
    while(!(I2C1->SR1 & I2C_SR1_TXE)){} // Wait for byte transfer complete
    
    while((DMA1->ISR & DMA_ISR_TCIF7) == 0);

    I2C1->CR1 |= I2C_CR1_STOP;
}

/* void DMA1_Channel7_IRQHandler(void){
    I2C1->CR1 |= I2C_CR1_STOP;
    DMA1_Channel7->CCR &= ~DMA_CCR_EN; //Disable DMA for I2C1.
} */