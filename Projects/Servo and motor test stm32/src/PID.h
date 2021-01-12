#ifndef PID_H_
#define PID_H_

#include <stdint.h>

typedef struct{ 
	

	float target; // Target rotation-rate in degrees per second
	float reference; // Reference rotation-rate in degrees per second
	
	// PID controller gains for a single axis
	float Kp; //Proportional factor
	float Ki; //Integral factor
	float Kd; //Derivative factor
	
	// memory
	float prev_error;
    float prev_reference;
	float i_term;

	float output;
} axis_t; // Independent values for each axis

typedef struct
{
	axis_t roll; 
	axis_t pitch; 
	axis_t yaw;
	
	float T; // Sample time in seconds
	
	float lim_max_int; // Integrator max limit
	float lim_min_int; // Integrator min limit
	
	float lim_max_output; // output max clamping
	float lim_min_output; // output min clamping

	uint8_t pid_updated_flag;

} PID_t; // All values for needed for the PID controler, for all axis


void PID_init(PID_t* pid);

void PID_update(PID_t* pid);

#endif //PID.h