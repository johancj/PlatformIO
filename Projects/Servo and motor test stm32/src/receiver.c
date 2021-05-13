#include "stm32f1xx.h"
#include "receiver.h"

void receiver_timer_init(void){ // Using TIM4 to trigger interrupts to read receiver and battery voltage

	//Example: https://www.youtube.com/watch?v=2YSYWR309Y4&list=PLmY3zqJJdVeNIZ8z_yw7Db9ej3FVG0iLy&index=15&ab_channel=EddieAmaya
	
	// Enable peripheral clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // Enable TIM4 pheripheral clock

	TIM4->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS); // Sets TIM4 as up counter
	TIM4->CR1 |= TIM_CR1_URS; // Only counter overflow generates interrupt
	
	TIM4->CNT = 0; // Clear counter register
	TIM4->PSC = 72 - 1; // Provides the counter clock frequency CK_CNT; fCK_PSC / (PSC[15:0] + 1) = 1MHz counter on TIM4.
    TIM4->ARR = 40000 - 1; //Overflow with 1MHz/(ARR + 1) = 25Hz 			

	TIM4->EGR |= TIM_EGR_UG; //UG: update generation                        //////////// Mulig denne må settes senere i initialiseringen. ////////////
	TIM4->DIER |= TIM_DIER_UIE; // Update interrupt enable

	NVIC_EnableIRQ(TIM4_IRQn); 

	TIM4->CR1 |= TIM_CR1_CEN; // Counter enable

}

void receiver_init(receiver_t* receiver){
    receiver->throttle = 0.0f; receiver->roll = 1500.0f; receiver->pitch = 1500.0f; receiver->yaw = 1500.0f;
    receiver_timer_init();
}

void update_target_vals(PID_t* pid, receiver_t* receiver){
    // Linear rates with "max_xxxx_rates" as the maximum rotaton rate of the drone in degrees per second
    float max_roll_rate = 300.0f;
    float max_pitch_rate = 300.0f;
    float max_yaw_rate = 200.0f;

    pid->roll.target = (receiver->roll - 1500.0f)*max_roll_rate/500.0f;
    pid->pitch.target = (receiver->pitch - 1500.0f)*max_pitch_rate/500.0f;
    pid->yaw.target = (receiver->yaw - 1500.0f)*max_yaw_rate/500.0f;

}