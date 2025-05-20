#include "fsm.h"
#include "dog.h"
#include "timer.h"
#include "gait.h"
#include "stdio.h"

float trot_forward_cog_offset = 0.01;
float trot_backward_cog_offset = 0.005;

GaitState trot_state;    
/* 对角步态状态的处理函数 */
static void trot_enter(void) {
    // printf("进入对角步态状态\n");
    trot_state.time_start = getTime();   
}

// 从当前位置经历trot_T秒到达目标位置
static void trot_run(void) {
    float phase[4] = {0};
    float motor_target_pos[4][3];
    float s_body_vel[3];
    dog_get_body_vel_without_cog(s_body_vel); // 获取当前机体速度
    if (s_body_vel[X_IDX] >= 0) // 前进时使用前进的重心补偿
    {
        get_robot_params()->posture.center_of_gravity.translation[X_IDX] = trot_forward_cog_offset;
    }
    else // 后退时使用后退的重心补偿
    {
        get_robot_params()->posture.center_of_gravity.translation[X_IDX] = trot_backward_cog_offset;
    }
    phase_wave_generator(get_trot_params(), trot_state.wave_status, trot_state.time_start, phase, trot_state.contact);
    gait_generator(get_trot_params(), phase, trot_state.contact, trot_state.foot_target_pos);
    
    for(int i = 0; i < 4; i++){
        leg_set_target_foot_pos(i, trot_state.foot_target_pos[i]); // 调试用
        leg_foot_to_motor(i, trot_state.foot_target_pos[i], motor_target_pos[i]);
        leg_set_motor_pos(i, motor_target_pos[i]);
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
