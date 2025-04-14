#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stdint.h"

typedef struct
{
    float pos;
    float vel;
    float acc;
} Motor;

void vrep_motor_init(void);
void vrep_motor_deinit(void);
void motor_init(void);
void send_motors_target_pos(float* motors_target_pos);
void leg_get_motors_current_pos(uint8_t leg_idx, float motors_current_pos[3]);
void motors_recv_update(void);
#endif

