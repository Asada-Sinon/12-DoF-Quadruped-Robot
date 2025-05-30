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
void send_motors_target_force_pos_vel(float *motors_target_force, float *motors_target_pos, float *motors_target_vel);
void leg_get_motors_current_pos(uint8_t leg_idx, float motors_current_pos[3]);
void leg_get_motors_current_vel(uint8_t leg_idx, float motors_current_vel[3]);
void motors_recv_update(void);
void leg_set_motor_kp_kd(uint8_t leg_idx, float kp_hip, float kd_hip, float kp_thigh, float kd_thigh, float kp_calf, float kd_calf);
#endif

