#include "fsm.h"
#include "dog.h"
#include "timer.h"
#include "stdio.h"
#include "string.h"
#include "motor.h"
#include "force_calculate.h"
#include "ANO_TC.h"
#include "estimator.h"

static float stand_T = 2.0f; // 站立时间基准
static float T = 0; // 根据距离计算的站立时间
static float time_start = 0;
static float foot_start_pos[4][3];
static float foot_target_pos[4][3];
static float foot_target_vel[4][3];

static float p = 0;
static float t = 0;
static float motor_current_pos[4][3];
float stand_test_foot_start_pos[4][3];
float stand_test_foot_target_pos[4][3];

float stand_test_foot_current_force[4][3] = {0};
float stand_test_foot_current_pos[4][3] = {0};
float stand_test_foot_current_vel[4][3] = {0};

float stand_test_foot_target_force[4][3];

typedef struct pd{
    float kp;
    float kd;
}pd;
 pd pos_hip[4] = { {120 , 2}, {120, 2}, {120, 2}, {120, 2} };
 pd pos_thigh[4] = { {100 , 5}, {100 , 5}, {100 , 5}, {100 , 5} };
 pd pos_calf[4] = { {80 , 5}, {80, 5}, {80, 5}, {80, 5} };

//pd pos_hip[4] = { {0 , 0}, {0, 0}, {0, 0}, {0, 0} };
//pd pos_thigh[4] = { {0 , 0}, {0, 0}, {0, 0}, {0, 0} };
//pd pos_calf[4] = { {0 , 0}, {0, 0}, {0, 0}, {0, 0} };

//pd pos_hip[4] = { {5 , 1}, {5, 1}, {5, 1}, {5, 1} };
//pd pos_thigh[4] = { {5 , 1}, {5, 1}, {5, 1}, {5, 1} };
//pd pos_calf[4] = { {5 , 1}, {5, 1}, {5, 1}, {5, 1} };

pd force_hip[4] = { {0 , 0}, {0, 0}, {0, 0}, {0, 0} };
pd force_thigh[4] = { {0 , 0}, {0, 0}, {0, 0}, {0, 0} };
pd force_calf[4] = { {0 , 0}, {0, 0}, {0, 0}, {0, 0} };

//pd force_hip[4] = { {1 , 0.5}, {1, 0.5}, {1, 0.5}, {1, 0.5} };
//pd force_thigh[4] = { {1 , 0.5}, {1, 0.5}, {1, 0.5}, {1, 0.5} };
//pd force_calf[4] = { {1 , 1}, {1, 1}, {1, 1}, {1, 1} };


/* 站立状态的处理函数 */
static void stand_enter(void) {
    // printf("进入站立状态\n");
    RobotParams* params = get_dog_params();
    time_start = getTime();
    for (int i = 0; i < 4; i++){
        if (fsm_get_previous_state() == STATE_TROT) // 上一个状态是对角步态，仿真中把上一时刻的目标值当做起始值，防止电机回传延迟。实机正常把当前值做起始值就行。
        {
//            leg_get_target_foot_pos(i, foot_start_pos[i]);
            leg_get_current_foot_pos(i, foot_start_pos[i]);
            // printf("站立状态起始位置[%d]: %f %f %f\n", i, foot_start_pos[i][X_IDX], foot_start_pos[i][Y_IDX], foot_start_pos[i][Z_IDX]);
        }
        else
        {
            leg_get_current_foot_pos(i, foot_start_pos[i]);
            leg_get_motors_current_pos(i, motor_current_pos[i]);
            leg_set_motor_pos(i, motor_current_pos[i]);
        }
        

        // 设置为全支撑、相位为0.5
        get_dog_params()->posture.contact[i] = 1;
        get_dog_params()->posture.phase[i] = 0.5;
    }
            
     
    memcpy(stand_test_foot_start_pos, foot_start_pos, sizeof(foot_start_pos));
    for (int i = 0; i < 4; i++)
    {
        leg_set_motor_kp_kd(i, pos_hip[i].kp, pos_hip[i].kd, pos_thigh[i].kp, pos_thigh[i].kd, pos_calf[i].kp, pos_calf[i].kd);
    }
}

float motor_target_force[4][3] = {0};
// 从当前位置经历stand_T秒到达目标位置
static void stand_run(void) {
    t = getTime() - time_start;
    float stand_height = dog_get_stand_height();
    float neutral_pos[4][3]; 
    RobotParams* params = get_dog_params();
    float motor_target_pos[4][3];
    float motor_target_vel[4][3] = {0};
    

    for (int i = 0; i < 4; i++)
    {
        leg_get_neutral_current_pos(i, neutral_pos[i]);
        leg_thigh_to_hip(i, neutral_pos[i], neutral_pos[i]);
    }
    for (int i = 0; i < 4; i++)
    {
        float xf = neutral_pos[i][X_IDX];
        float yf = neutral_pos[i][Y_IDX];
        float zf = neutral_pos[i][Z_IDX];

        float x0 = foot_start_pos[i][X_IDX];
        float y0 = foot_start_pos[i][Y_IDX];
        float z0 = foot_start_pos[i][Z_IDX];

        float length = sqrtf(powf(xf - x0, 2) + powf(yf - y0, 2) + powf(zf - z0, 2));
        T = length / stand_height * stand_T;

        p = t/T;
        if (p > 1)
            p = 1;

        float x = x0 + (xf - x0) * (1 - cosf(MY_PI*p)) / 2.0f;
        float y = y0 + (yf - y0) * (1 - cosf(MY_PI*p)) / 2.0f;
        float z = z0 + (zf - z0) * (1 - cosf(MY_PI*p)) / 2.0f;

        foot_target_pos[i][X_IDX] = x;
        foot_target_pos[i][Y_IDX] = y;
        foot_target_pos[i][Z_IDX] = z;
        
        // 力控，如果是从对角步态切换过来，为了保持稳定性直接用力控
        if (vmc_get_force_control_state() == 1 && fsm_get_previous_state() == STATE_TROT)
        {
            vmc_get_foot_target_force(i, stand_test_foot_target_force[i]);
            leg_set_motor_kp_kd(i, force_hip[i].kp, force_hip[i].kd, force_thigh[i].kp, force_thigh[i].kd, force_calf[i].kp, force_calf[i].kd);
        }
            
        else // 位控，因为力控没写位置约束，所以站立要用位控
        {
            memset(stand_test_foot_target_force[i], 0, sizeof(stand_test_foot_target_force[i]));
            leg_set_motor_kp_kd(i, pos_hip[i].kp, pos_hip[i].kd, pos_thigh[i].kp, pos_thigh[i].kd, pos_calf[i].kp, pos_calf[i].kd);
        }
        // 调试用
//        memset(stand_test_foot_target_force[i], 0, sizeof(stand_test_foot_target_force[i]));
        
        // 必须加这个set_target_foot_pos
        leg_set_target_foot_pos(i, foot_target_pos[i]);
        leg_foot_to_motor_force_pos_vel(i, stand_test_foot_target_force[i], foot_target_pos[i], foot_target_vel[i], motor_target_force[i], motor_target_pos[i], motor_target_vel[i]);
        leg_set_motor_force_pos_vel(i, motor_target_force[i], motor_target_pos[i], motor_target_vel[i]);

        // 获取足端力，调试用
        leg_get_current_foot_force_pos_vel(i, stand_test_foot_current_force[i], stand_test_foot_current_pos[i], stand_test_foot_current_vel[i]);
        set_debug_data(2, stand_test_foot_current_force[0][2]);
        set_debug_data(3, stand_test_foot_current_force[1][2]);
        set_debug_data(4, stand_test_foot_current_force[2][2]);
        set_debug_data(5, stand_test_foot_current_force[3][2]);
//        
//        set_debug_data(6, stand_test_foot_current_force[0][2]);
//        set_debug_data(7, stand_test_foot_current_force[1][2]);
//        set_debug_data(8, stand_test_foot_current_force[2][2]);
//        set_debug_data(9, stand_test_foot_current_force[3][2]);
        // leg_foot_to_motor(i, foot_target_pos[i], motor_target_pos[i]);
        // leg_set_motor_pos_vel(i, motor_target_pos[i], motor_target_vel[i]);
    }
    memcpy(stand_test_foot_target_pos, foot_target_pos, sizeof(foot_target_pos));
}

static void stand_exit(void) {
    // printf("退出站立状态\n");
}

/* 站立状态的转换检查 */
static bool stand_check_transition(StateName next) {
    // 从站立状态可以转到任何状态
    if (next == STATE_TROT)
        if(t >= T) // 确保完成站立后再进入对角步态
        {
            return true;
        }
    if (next == STATE_PASSIVE)
        return true;

    return false;
}

/* 注册站立状态 */
void register_stand_state(void) {
    fsm_register_state(STATE_STAND,
                     stand_enter,
                     stand_run,
                     stand_exit,
                     stand_check_transition);
}
