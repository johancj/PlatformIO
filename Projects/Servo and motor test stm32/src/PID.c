#include "PID.h"

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
	
	pid->u = 0.0f;
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