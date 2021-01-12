#include "receiver.h"


void receiver_init(receiver_t* receiver){
    receiver->throttle = 0.0f; receiver->roll = 1500.0f; receiver->pitch = 1500.0f; receiver->yaw = 1500.0f;
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