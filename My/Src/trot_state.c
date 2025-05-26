#include "fsm.h"
#include "dog.h"
#include "timer.h"
#include "gait.h"
#include "stdio.h"
#include "motor.h"


GaitState trot_state;    

typedef struct pd{
    float kp;
    float kd;
}pd;

pd swing_hip[4] = { {120 , 0.5}, {120, 2}, {120, 2}, {120, 2} };
pd swing_thigh[4] = { {100 , 3}, {150, 3}, {150, 3}, {150, 3} };
pd swing_calf[4] = { {100 , 3}, {120, 4}, {120, 4}, {120, 4} };

pd stance_hip[4] = { {120 , 2}, {120, 2}, {120, 2}, {120, 2} };
pd stance_thigh[4] = { {140 , 5}, {150, 3}, {150, 3}, {150, 3} };
pd stance_calf[4] = { {100 , 5}, {120, 4}, {120, 4}, {120, 4} };

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

/* 对角步态状态的处理函数 */
static void trot_enter(void) {
    // printf("进入对角步态状态\n");
    trot_state.time_start = getTime();   
}

// 从当前位置经历trot_T秒到达目标位置
static void trot_run(void) {
    float phase[4] = {0};
    float motor_target_pos[4][3];
    float motor_target_vel[4][3];
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
    else // 站立时重心
    {
        get_dog_params()->posture.center_of_gravity.translation[X_IDX] = get_dog_params()->posture.center_of_gravity.stand_cog_offset[X_IDX];
    }
    
    phase_wave_generator(get_trot_params(), trot_state.wave_status, trot_state.time_start, phase, trot_state.contact);
    gait_generator(get_trot_params(), phase, trot_state.contact, trot_state.foot_target_pos, trot_state.foot_target_vel);
    
    for(int i = 0; i < 4; i++){
        if (trot_state.contact[i] == 0) // 摆动腿
            leg_set_motor_kp_kd(i, swing_hip[0].kp, swing_hip[0].kd, swing_thigh[0].kp, swing_thigh[0].kd, swing_calf[0].kp, swing_calf[0].kd);
        else // 支撑腿
            leg_set_motor_kp_kd(i, stance_hip[0].kp, stance_hip[0].kd, stance_thigh[0].kp, stance_thigh[0].kd, stance_calf[0].kp, stance_calf[0].kd);

        leg_set_target_foot_pos(i, trot_state.foot_target_pos[i]); // 调试用
        
        // leg_foot_to_motor(i, trot_state.foot_target_pos[i], motor_target_pos[i]);
        // leg_set_motor_pos(i, motor_target_pos[i]);

        leg_foot_to_motor_pos_vel(i, trot_state.foot_target_pos[i], trot_state.foot_target_vel[i], motor_target_pos[i], motor_target_vel[i]);
        leg_set_motor_pos_vel(i, motor_target_pos[i], motor_target_vel[i]);
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
