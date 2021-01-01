#include <stdio.h>

#include "PID.h"
#include "USART.h"

#include "stm32f1xx.h"

volatile uint8_t count_interrupts = 0;
static uint8_t PID_updated_flag = 0;

void PID_timer_init(void){ // Using TIM2 to trigger an interrupt every sample time T

	//Example: https://www.youtube.com/watch?v=2YSYWR309Y4&list=PLmY3zqJJdVeNIZ8z_yw7Db9ej3FVG0iLy&index=15&ab_channel=EddieAmaya
	
	// Enable peripheral clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 pheripheral clock

	TIM2->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS); // Sets TIM2 as up counter
	TIM2->CR1 |= TIM_CR1_URS; // Only counter overflow generates interrupt
	
	TIM2->CNT = 0; // Clear counter register
	TIM2->PSC = 72 - 1; // Provides the counter clock frequency CK_CNT; fCK_PSC / (PSC[15:0] + 1) = 1MHz counter on TIM2.
	TIM2->ARR = 10000 - 1; //Overflow with 1MHz/(ARR + 1) = 100Hz 			

	TIM2->EGR |= TIM_EGR_UG; //UG: update generation ////////////Mulig denne må settes senere i initialiseringen. ////////////
	TIM2->DIER |= TIM_DIER_UIE; // Update interrupt enable

	NVIC_EnableIRQ(TIM2_IRQn); 

	TIM2->CR1 |= TIM_CR1_CEN; // Counter enable

}

void PID_init(PID_t* pid){
    // PID controller gains
    pid->Kp = 1.0f;
    pid->Ki = 0.0f;
    pid->Kd = 0.0f;

    // Sample time
    pid->T = 0.03f; // seconds

    // output static clamping
	pid->lim_max_output = 40.0f;
	pid->lim_min_output = - pid->lim_max_output;

    // Integrator static clamping
	pid->lim_max_int = pid->lim_max_output;
	pid->lim_min_int = pid->lim_min_output;
	
	// clear memory
	pid->prev_error = 0.0f;
    pid->prev_reference = 0.0f;
	pid->i_term = 0.0f;
	pid->d_term = 0.0f;
	pid->PID_updated_flag = 0;
	
	pid->u = 0.0f;

	PID_timer_init();
}

void PID_update(PID_t* pid, int16_t reference, int16_t target){  //reference (MCP6050) and target (controller) are rotation rate
    
    float error = target - reference;
	
	// Calculate P-term
	float p_term = pid->Kp * error;

	
	// Calculate I-term
	pid->i_term = pid->i_term + 0.5f * pid->Ki * pid->T * (error + pid->prev_error);
	
	//Dynamically adjust i-term clamping limit
	if ((pid->lim_max_output - p_term) > 0){
		pid->lim_max_int = pid->lim_max_output - p_term;
	}
	else{
		pid->lim_max_int = 0;
	}
	if ((pid->lim_min_output - p_term) < 0){
		pid->lim_min_int = pid->lim_min_output - p_term;
	}
	else{
		pid->lim_min_int = 0;
	}
	
	// Anti i-term wind-up using dynamic clamping
	if(pid->i_term > pid->lim_max_int){
		pid->i_term = pid->lim_max_int;
	}
	else if(pid->i_term < pid->lim_min_int){
		pid->i_term = pid->lim_min_int;
	}
	
	
	// Calculate derivative term
	pid->d_term = pid->Kd / pid->T * (error - pid->prev_error);
	//pid->d_term = pid->Kd / pid->T * (reference - pid->prev_reference); // Using reference for PID to reduce step function effect from target
	
	// PID output
	pid->u = (p_term + pid->i_term + pid->d_term);
	
	// Update memory
	pid->prev_error = error;
    pid->prev_reference = reference;
	
	
 /*	if (pid->u > 0){
		PIOD->PIO_CODR |= PIO_SODR_P10; //Motor direction:0
	}
	else{
		PIOD->PIO_SODR |= PIO_SODR_P10; //Motor direction:1
	}
	
	// PID output and clamping
	if (pid->u > pid->lim_max_output){ 
		DACC->DACC_CDR = abs(pid->lim_max_output);
	}
	else if (pid->u < pid->lim_min_output){
		DACC->DACC_CDR = abs(pid->lim_min_output);
	}
	else{
		DACC->DACC_CDR = abs(pid->u);
	} */

}

void TIM2_IRQHandler(void){ //PID timer, TIM2 global handler
	myprintf("TIM2_IRQHandler #%d\n\r", count_interrupts++);
	TIM2->SR &= ~TIM_SR_UIF;
} 