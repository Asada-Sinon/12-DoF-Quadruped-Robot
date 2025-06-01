#include "force_calculate.h"
#include "matrix.h"
#include "dog.h"
#include "estimator.h"
#include "force_calculate.h"
#include "ANO_TC.h"

float body_target_force[6] = {0}; // 机体目标力 [x, y, z, roll, pitch, yaw]
float foot_force[4][3] = {0}; // 足端目标力 [x, y, z]

float foot_pos[3] = {0};
float leg_height[4] = {0};
float leg_neutral_pos[4][3] = {0};
    
float leg_current_force[4][3] = {0};
float leg_current_pos[4][3] = {0};
float leg_current_vel[4][3] = {0};

float kxy = 0.0001;
float kpz = 100; //100
float kdz = 20;
float kroll = 0.1;
float kpitch = 0.1;
float kyaw = 0.1;

uint8_t force_control = 0;

s_LPFilter lpf_body_height_vel;
uint8_t vmc_init_flag = 0;

// 根据机体目标速度与位置计算机体的目标力
void vmc_force_calculate() // 放在定时器里
{
    if(!vmc_init_flag)
    {
        vmc_init_flag = 1;
        LPFilter_init(&lpf_body_height_vel, 0.002, 1.2);
    }
    float body_vel[3] = {0};
    dog_get_body_vel(body_vel);
    // 计算机体目标力
    body_target_force[0] = kxy * (body_vel[0] - est_get_body_vel(0)); // xy方向只进行速度控制
    body_target_force[1] = kxy * (body_vel[1] - est_get_body_vel(1));
    body_target_force[5] = kyaw * (body_vel[2] - est_get_body_vel(5));
    // 计算机体当前高度

    for(int i = 0; i < 4; i++)
    {
        leg_get_current_foot_pos(i, foot_pos);
        leg_height[i] = foot_pos[2];
        leg_get_neutral_current_pos(i, leg_neutral_pos[i]);
        leg_get_current_foot_force_pos_vel(i, leg_current_force[i], leg_current_pos[i], leg_current_vel[i]);
    }
//    float body_height = -(leg_height[0] + leg_height[1] + leg_height[2] + leg_height[3]) / 4.0f;
//    float body_target_height = -(leg_neutral_pos[0][2] + leg_neutral_pos[1][2] + leg_neutral_pos[2][2] + leg_neutral_pos[3][2]) / 4.0f;
    
    float body_height = -leg_height[0];
    float body_target_height = -leg_neutral_pos[0][2];
    float body_current_vz = -leg_current_vel[0][2];
    LPFilter(&lpf_body_height_vel, body_current_vz);
    body_target_force[2] = kpz * (body_target_height - body_height) + kdz * (0 - LPF_get_value(&lpf_body_height_vel)); // 高度上用pd控制位置
    set_debug_data(8, LPF_get_value(&lpf_body_height_vel));
    set_debug_data(9, body_current_vz);
    // 分配到足端力上
    for(int i = 0; i < 4; i++)
    {
        if(leg_get_contact_state(i) == 1) // 位于支撑相
        {
            foot_force[i][0] = -body_target_force[0]; // 暂时只有vx vy z， w和俯仰与横滚角控制以后再加
            foot_force[i][1] = -body_target_force[1];
            foot_force[i][2] = -body_target_force[2];
        }
        else
        {
            memset(foot_force[i], 0, sizeof(foot_force[i])); // 摆动相足端力为0
        }
    }
}

void vmc_get_foot_target_force(uint8_t leg_id, float foot_target_force[3])
{
    memcpy(foot_target_force, foot_force[leg_id], sizeof(foot_force[leg_id]));
}

void vmc_set_force_control(uint8_t x)
{
    if (x == 0 || x == 1)
        force_control = x;
}

uint8_t vmc_get_force_control_state()
{
    return force_control;
}
