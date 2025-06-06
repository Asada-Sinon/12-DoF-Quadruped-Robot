#include "force_calculate.h"
#include "matrix.h"
#include "dog.h"
#include "estimator.h"
#include "force_calculate.h"
#include "ANO_TC.h"
#include "timer.h"

float body_target_force[6] = {0}; // 机体目标力 [x, y, z, roll, pitch, yaw]
float leg_force_z[4] = {0};  // 腿目标力 [z]
float foot_force[4][3] = {0}; // 足端目标力 [x, y, z]

float foot_pos[3] = {0};
float leg_height[4] = {0};
float leg_neutral_pos[4][3] = {0};
    
float leg_current_force[4][3] = {0};
float leg_current_pos[4][3] = {0};
float leg_current_vel[4][3] = {0};


float ky = 100;
float kx = 150;

float kpz[4] = {500, 500, 520, 500}; //150
float kdz[4] = {25, 25, 25, 25};  //25
float kroll = 0.1;
float kpitch = 0.1;
float kyaw = 0.1;

float test_time_force_cal = 0;
float _time = 0;

uint8_t force_control = 0;

s_LPFilter lpf_body_height_vel[4];
uint8_t vmc_init_flag = 0;

// 根据机体目标速度与位置计算机体的目标力
void vmc_force_calculate() // 放在定时器里
{
    _time = getTime();
    if(!vmc_init_flag)
    {
        vmc_init_flag = 1;
        // 踏步时扰动大，滤波新值权重给的较小，有延迟但能接受       
        LPFilter_init(&lpf_body_height_vel[0], 0.003, 2); //1.2
        LPFilter_init(&lpf_body_height_vel[1], 0.003, 2);
        LPFilter_init(&lpf_body_height_vel[2], 0.003, 2);
        LPFilter_init(&lpf_body_height_vel[3], 0.003, 2);
    }
    float body_vel[3] = {0};
    dog_get_body_vel(body_vel);
    // 计算机体xy方向的目标力
    body_target_force[0] = kx * (body_vel[0] - est_get_body_vel(0)); // xy方向只进行速度控制
    body_target_force[1] = ky * (body_vel[1] - est_get_body_vel(1));
    // 计算机体自转方向的目标力
    body_target_force[5] = kyaw * (body_vel[2] - est_get_body_vel(5));
    // 计算机体z方向的目标力
    
    for(int i = 0; i < 4; i++)
    {
        leg_get_current_foot_pos(i, foot_pos);
        leg_height[i] = foot_pos[2];
        leg_get_neutral_current_pos(i, leg_neutral_pos[i]);
        
        leg_get_current_foot_force_pos_vel(i, leg_current_force[i], leg_current_pos[i], leg_current_vel[i]);
        
        float body_height = -leg_height[i];
        float body_target_height = -leg_neutral_pos[i][2];
        float body_current_vz = -leg_current_vel[i][2];
        LPFilter(&lpf_body_height_vel[i], body_current_vz);
        leg_force_z[i] = kpz[i] * (body_target_height - body_height) + kdz[i] * (0 - LPF_get_value(&lpf_body_height_vel[i])); // 高度上用pd控制位置
    }
    
//    // 高度
//    set_debug_data(8, LPF_get_value(&lpf_body_height_vel[1]));
//    set_debug_data(9, -leg_current_vel[1][2]);

    // 分配到足端力上
    for(int i = 0; i < 4; i++)
    {
        if(leg_get_contact_state(i) == 1) // 位于支撑相
        {
            foot_force[i][0] = -body_target_force[0]; // 足端x方向力
            foot_force[i][1] = -body_target_force[1]; // 足端y方向力
            foot_force[i][2] = -leg_force_z[i]; // 足端z方向力
        }
        else
        {
            memset(foot_force[i], 0, sizeof(foot_force[i])); // 摆动相足端力为0
        }
    }
     // x方向
//    set_debug_data(8, foot_force[0][0]);
//    set_debug_data(9, foot_force[0][1]);
//    set_debug_data(10, foot_force[0][2]);
    test_time_force_cal = (getTime() - _time) * 1000;
}

float CONTACT_FORCE_THRESHOLD = 5;
uint8_t contact_judge_with_force(uint8_t leg_id)
{
    float foot_force[3];
    float foot_pos[3];
    float foot_vel[3];
    
    // 获取足端力和位置信息
    leg_get_current_foot_force_pos_vel(leg_id, foot_force, foot_pos, foot_vel);
    
    // 判断z轴力是否超过阈值
    if(foot_force[2] > CONTACT_FORCE_THRESHOLD)
    {
        return 1;  // 触地
    }
    else
    {
        return 0;  // 未触地
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
