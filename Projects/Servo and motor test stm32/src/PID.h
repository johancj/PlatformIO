#ifndef PID_H_
#define PID_H_

#include <stdint.h>

typedef struct
{
    // PID controller gains
	float Kp; //Proportional factor
	float Ki; //Integral factor
	float Kd; //Derivative factor

	// Sample time
	float T; // In seconds
	
	// Integrator limits
	float lim_max_int;
	float lim_min_int;
	
	// memory
	float prev_error;
    float prev_reference;
	float i_term;
	float d_term;
	uint8_t PID_updated_flag;
	
	// output clamping
	float lim_max_output;
	float lim_min_output;
	
	float u;

} PID_t;


void PID_init(PID_t* pid);

void PID_update(PID_t* pid, int16_t reference, int16_t target);

#endif //PID.h