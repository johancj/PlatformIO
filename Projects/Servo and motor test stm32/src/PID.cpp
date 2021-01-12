#include <stdio.h>
//#include <stdlib.h> // for abs()

#include "PID.h"
#include "USART.h"

#include "stm32f1xx.h"




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

	pid->roll.target = 0.0f; 
	pid->pitch.target = 0.0f; 
	pid->yaw.target = 0.0f; 


    // PID controller gains for all axis
    pid->roll.Kp = 1.0f;
    pid->roll.Ki = 0.0f;
    pid->roll.Kd = 0.0f;

	pid->pitch.Kp = 1.0f;
    pid->pitch.Ki = 0.0f;
    pid->pitch.Kd = 0.0f;

	pid->yaw.Kp = 1.0f;
    pid->yaw.Ki = 0.0f;
    pid->yaw.Kd = 0.0f;

    // Sample time
    pid->T = 0.01f; // seconds

    // Output static clamping
	pid->lim_max_output = 300.0f;
	pid->lim_min_output = - pid->lim_max_output;

    // Integrator static clamping
	pid->lim_max_int = pid->lim_max_output;
	pid->lim_min_int = pid->lim_min_output;
	
	// clear memory for all axis
	pid->roll.prev_error = 0.0f; 
	pid->roll.prev_reference = 0.0f; 
	pid->roll.i_term = 0.0f; 

	pid->pitch.prev_error = 0.0f; 
	pid->pitch.prev_reference = 0.0f; 
	pid->pitch.i_term = 0.0f; 

	pid->yaw.prev_error = 0.0f;
    pid->yaw.prev_reference = 0.0f;
	pid->yaw.i_term = 0.0f; 

	// The PID controller has not been run yet
	pid->pid_updated_flag = 0;
	
	pid->roll.output = 0.0f; 
	pid->pitch.output = 0.0f; 
	pid->yaw.output = 0.0f;

	PID_timer_init();
}

void PID_update_axis(axis_t* axis, PID_t* pid){  //////////////////// PID controller for one of the axis /////////////////////
    //reference (MCP6050) and target (controller) are rotation rate in degrees per second
	
    float error = axis->target - axis->reference;
	
	// Calculate P-term
	float p_term = axis->Kp * error;

	
	// Calculate I-term
	axis->i_term = axis->i_term + 0.5f * axis->Ki * pid->T * (error + axis->prev_error);
	
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
	if(axis->i_term > pid->lim_max_int){
		axis->i_term = pid->lim_max_int;
	}
	else if(axis->i_term < pid->lim_min_int){
		axis->i_term = pid->lim_min_int;
	}
	
	
	// Calculate derivative term
	//float d_term = axis->Kd / pid->T * (error - axis->prev_error);
	float d_term = axis->Kd / pid->T * (axis->reference - axis->prev_reference); // Using reference for PID to reduce step function effect from change in target
	
	// PID output
	axis->output = (p_term + axis->i_term + d_term);
	
	// Update memory
	axis->prev_error = error;
    axis->prev_reference = axis->reference;
	
	
	// PID output and clamping
	if (axis->output > pid->lim_max_output){ 
		axis->output = pid->lim_max_output;
	}
	else if (axis->output < pid->lim_min_output){
		axis->output = pid->lim_min_output;
	}

	pid->pid_updated_flag = 1;
} 

void PID_update(PID_t* pid){  //reference (MCP6050) and target (controller) are rotation rate
	PID_update_axis(&pid->roll, pid);
	PID_update_axis(&pid->pitch, pid);
	PID_update_axis(&pid->yaw, pid);

}

