#ifndef RECEIVER_H_
#define RECEIVER_H_

#include <stdint.h>
#include "PID.h"

typedef struct{
    uint16_t throttle;
    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
} receiver_t;

void receiver_init(receiver_t* receiver);

void update_target_vals(PID_t* pid, receiver_t* receiver);

#endif // RECEIVER_H_