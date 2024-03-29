#include <stm32f1xx.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "USART.h"

void USART_init(void){
    //USART1; TX: PA9, RX: PA10
    
    // Pheripheral Enable
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN; 
    GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9;
    GPIOA->CRH &= ~(GPIO_CRH_CNF9_0);

    // 72MHz / (9600 (BAUD_RATE) * 16) - 1 = 467.75
    USART1->BRR = 0x1D4C;
    USART1->CR1 |= USART_CR1_UE | USART_CR1_TE; //Usart enable, transmitter enable
}

void USART_transmitt(unsigned char data){
    while(!(USART1->SR & USART_SR_TXE));
    USART1->DR = data;
}


