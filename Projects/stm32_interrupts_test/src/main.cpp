#include <Arduino.h>
#include "stm32f1xx.h"


volatile uint32_t count_interrupts_PID = 0;



void PID_timer_init(void){ 
	//Example: https://www.youtube.com/watch?v=2YSYWR309Y4&list=PLmY3zqJJdVeNIZ8z_yw7Db9ej3FVG0iLy&index=15&ab_channel=EddieAmaya
	
	
	// Using TIM2 to trigger an interrupt every sample time T

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 pheripheral clock

	TIM2->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS); // Sets TIM2 as up counter
	TIM2->CR1 |= TIM_CR1_URS; // Only counter overflow generates interrupt
	
	TIM2->CNT = 0; // Clear counter register
	TIM2->PSC = 720 - 1; // Provides the counter clock frequency CK_CNT; fCK_PSC / (PSC[15:0] + 1) = 1MHz counter on TIM2.
	TIM2->ARR = 10000 - 1; //Overflow with 1MHz/(ARR + 1) = 100Hz 			

	TIM2->EGR |= TIM_EGR_UG; //UG: update generation ////////////Mulig denne må settes senere i initialiseringen. ////////////
	TIM2->DIER |= TIM_DIER_UIE; // Update interrupt enable

	NVIC_EnableIRQ(TIM2_IRQn); 

	TIM2->CR1 |= TIM_CR1_CEN; // Counter enable


}

void setup() {
	Serial1.begin(115200);
	//__enable_irq();
	delay(5);
	Serial1.println("Starting up...");
	delay(50);
	PID_timer_init();
}

void loop() {
	Serial1.println(TIM2->CNT);
	delay(5);
	if (TIM2->SR != 0){
		Serial1.println(TIM2->SR);
	}
}


void TIM2_IRQHandler(void){ //TIM2 global handler
	// This function is never called, but the interrupt flag is set and thus the code crashes. My theory is that Arduino lib is using it and this function never reaced.
	Serial1.printf("TIM2_IRQHandler #%d\n\r", count_interrupts_PID++);
	delay(200);
	TIM2->SR &= ~TIM_SR_UIF;
}