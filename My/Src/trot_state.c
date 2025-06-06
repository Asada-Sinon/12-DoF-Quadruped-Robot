#include "fsm.h"
#include "dog.h"
#include "timer.h"
#include "gait.h"
#include "stdio.h"
#include "motor.h"
#include "string.h"
#include "ANO_TC.h"
#include "force_calculate.h"

GaitState trot_state;    

typedef struct pd{
    float kp;
    float kd;
}pd;

 pd swing_hip[4] = { {120 , 3}, {120, 2}, {120, 2}, {120, 2} };
 pd swing_thigh[4] = { {40 , 2}, {50, 2}, {50, 2}, {50, 2} };
 pd swing_calf[4] = { {40 , 2}, {50, 2}, {50, 2}, {50, 2} };

 pd stance_hip[4] = { {120 , 3}, {120, 2}, {120, 2}, {120, 2} };
 pd stance_thigh[4] = { {70 , 5}, {80 , 3}, {80 , 3}, {80 , 3} };
 pd stance_calf[4] = { {70 , 5}, {80 , 3}, {80 , 3}, {80 , 3} };

 pd stance_force_hip[4] = { {1 , 1}, {1, 1}, {1, 1}, {1, 1} };
 pd stance_force_thigh[4] = { {1 , 1}, {1 , 1}, {1 , 1}, {1 , 1} };
 pd stance_force_calf[4] = { {1 , 1}, {1 , 1}, {1 , 1}, {1 , 1} };

// pd swing_hip[4] = { {0 , 0}, {0, 0}, {0, 0}, {0, 0} };
// pd swing_thigh[4] = { {0 , 0}, {0, 0}, {0, 0}, {0, 0} };
// pd swing_calf[4] = { {0 , 0}, {0, 0}, {0, 0}, {0, 0} };

// pd stance_hip[4] = { {0 , 0}, {0, 0}, {0, 0}, {0, 0} };
// pd stance_thigh[4] = { {0 , 0}, {0, 0}, {0, 0}, {0, 0} };
// pd stance_calf[4] = { {0 , 0}, {0, 0}, {0, 0}, {0, 0} };

//pd swing_hip[4] = { {10 , 3}, {10, 3}, {10, 3}, {10, 3} };
//pd swing_thigh[4] = { {10 , 3}, {10, 3}, {10, 3}, {10, 3} };
//pd swing_calf[4] = { {10 , 3}, {10, 3}, {10, 3}, {10, 3} };

//pd stance_hip[4] = { {10 , 3}, {10, 3}, {10, 3}, {10, 3} };
//pd stance_thigh[4] = { {10 , 3}, {10, 3}, {10, 3}, {10, 3} };
//pd stance_calf[4] = { {10 , 3}, {10, 3}, {10, 3}, {10, 3} };

float trot_foot_current_force[4][3] = {0};
float trot_foot_current_vel[4][3] = {0};
float trot_foot_current_pos[4][3] = {0};

float trot_foot_target_force[4][3] = {0};
float trot_foot_target_vel[4][3] = {0};
float trot_foot_target_pos[4][3] = {0};

/* 对角步态状态的处理函数 */
static void trot_enter(void) {
    // printf("进入对角步态状态\n");
    for(uint8_t i = 0; i < 4; i ++)
    {
        leg_set_motor_kp_kd(i, stance_hip[0].kp, stance_hip[0].kd, stance_thigh[0].kp, stance_thigh[0].kd, stance_calf[0].kp, stance_calf[0].kd);
        trot_state.contact[i] = 1;
    }
        
    trot_state.time_start = getTime();   
    
}

float phase[4] = {0};
// 从当前位置经历trot_T秒到达目标位置
static void trot_run(void) {
    
    float motor_target_pos[4][3];
    float motor_target_vel[4][3];
    float motor_target_force[4][3];
    float s_body_vel[3];
    
    dog_get_body_vel_without_cog(s_body_vel); // 获取当前机体速度
    if (s_body_vel[X_IDX] > 0) // 前进时使用前进的重心补偿
    {
        get_dog_params()->posture.center_of_gravity.translation[X_IDX] = get_dog_params()->posture.center_of_gravity.trot_cog_forward_offset[X_IDX];
    }
    else if(s_body_vel[X_IDX] < 0)// 后退时使用后退的重心补偿
    {
        get_dog_params()->posture.center_of_gravity.translation[X_IDX] = get_dog_params()->posture.center_of_gravity.trot_cog_backward_offset[X_IDX];
    }
//    else // 站立时重心
//    {
//        get_dog_params()->posture.center_of_gravity.translation[X_IDX] = get_dog_params()->posture.center_of_gravity.stand_cog_offset[X_IDX];
//    }
    
    phase_wave_generator(get_trot_params(), trot_state.wave_status, trot_state.time_start, trot_state.phase, trot_state.contact);
//    phase_wave_generator_with_force(get_trot_params(), trot_state.wave_status, trot_state.time_start, trot_state.phase, trot_state.contact);
    gait_generator(get_trot_params(), trot_state.phase, trot_state.contact, trot_state.foot_target_pos, trot_state.foot_target_vel);
    
    for(int i = 0; i < 4; i++){
        if (trot_state.contact[i] == 0) // 摆动腿
        {
            memset(trot_state.foot_target_force[i], 0, sizeof(trot_state.foot_target_force[i]));
            leg_set_motor_kp_kd(i, swing_hip[0].kp, swing_hip[0].kd, swing_thigh[0].kp, swing_thigh[0].kd, swing_calf[0].kp, swing_calf[0].kd);
        }
            
        else // 支撑腿
        {
            // 力控
            if (vmc_get_force_control_state() == 1)
            {
                vmc_get_foot_target_force(i, trot_state.foot_target_force[i]);
                leg_set_motor_kp_kd(i, stance_force_hip[i].kp, stance_force_hip[i].kd, stance_force_thigh[i].kp, stance_force_thigh[i].kd, stance_force_calf[i].kp, stance_force_calf[i].kd);
            }
            else // 位控
            {
                memset(trot_state.foot_target_force[i], 0, sizeof(trot_state.foot_target_force[i]));
                leg_set_motor_kp_kd(i, stance_hip[0].kp, stance_hip[0].kd, stance_thigh[0].kp, stance_thigh[0].kd, stance_calf[0].kp, stance_calf[0].kd);
            }
                
        }
            
        
        // leg_foot_to_motor(i, trot_state.foot_target_pos[i], motor_target_pos[i]);
        // leg_set_motor_pos(i, motor_target_pos[i]);
        // 调试用
//        memset(trot_state.foot_target_force[i], 0, sizeof(trot_state.foot_target_force[i]));
        
        leg_set_target_foot_pos(i, trot_state.foot_target_pos[i]);
        leg_foot_to_motor_force_pos_vel(i, trot_state.foot_target_force[i], trot_state.foot_target_pos[i], trot_state.foot_target_vel[i], motor_target_force[i], motor_target_pos[i], motor_target_vel[i]);
        // leg_set_motor_pos_vel(i, motor_target_pos[i], motor_target_vel[i]);
        leg_set_motor_force_pos_vel(i, motor_target_force[i], motor_target_pos[i], motor_target_vel[i]);
    
    // 获取足端力，调试用
        leg_get_current_foot_force_pos_vel(i, trot_foot_current_force[i], trot_foot_current_pos[i], trot_foot_current_vel[i]);
        set_debug_data(2, trot_foot_current_force[0][2]);
        set_debug_data(3, trot_foot_current_force[1][2]);
        set_debug_data(4, trot_foot_current_force[2][2]);
        set_debug_data(5, trot_foot_current_force[3][2]);
        
        set_debug_data(6, trot_foot_current_vel[0][2]);
        set_debug_data(7, trot_foot_current_vel[1][2]);
        set_debug_data(8, trot_foot_current_vel[2][2]);
        set_debug_data(9, trot_foot_current_vel[3][2]);
        
        set_debug_data(10, trot_state.contact[0]);
//        set_debug_data(7, trot_state.contact[1]);
//        set_debug_data(8, trot_state.contact[2]);
//        set_debug_data(9, trot_state.contact[3]);
//    
    
    }
    
    
}

static void trot_exit(void) {
    // printf("退出对角步态状态\n");
    // 离开对角步态状态的处理
    // ...
}

/* 对角步态状态的转换检查 */
static bool trot_check_transition(StateName next) {
    // 从对角步态状态可以转到任何状态
    if (next == STATE_STAND) {
        if(trot_state.contact[0] == 1 && trot_state.contact[1] == 1 && trot_state.contact[2] == 1 && trot_state.contact[3] == 1)
        {
            return true;
        }
    }
    if (next == STATE_PASSIVE)
        return true;
    return false;
}

/* 注册站立状态 */
void register_trot_state(void) {
    fsm_register_state(STATE_TROT,
                     trot_enter,
                     trot_run,
                     trot_exit,
                     trot_check_transition);
}
