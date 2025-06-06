#include "force_calculate.h"
#include "matrix.h"
#include "dog.h"
#include "estimator.h"
#include "force_calculate.h"
#include "ANO_TC.h"
#include "timer.h"

float body_target_force[6] = {0}; // ����Ŀ���� [x, y, z, roll, pitch, yaw]
float leg_force_z[4] = {0};  // ��Ŀ���� [z]
float foot_force[4][3] = {0}; // ���Ŀ���� [x, y, z]

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

// ���ݻ���Ŀ���ٶ���λ�ü�������Ŀ����
void vmc_force_calculate() // ���ڶ�ʱ����
{
    _time = getTime();
    if(!vmc_init_flag)
    {
        vmc_init_flag = 1;
        // ̤��ʱ�Ŷ����˲���ֵȨ�ظ��Ľ�С�����ӳٵ��ܽ���       
        LPFilter_init(&lpf_body_height_vel[0], 0.003, 2); //1.2
        LPFilter_init(&lpf_body_height_vel[1], 0.003, 2);
        LPFilter_init(&lpf_body_height_vel[2], 0.003, 2);
        LPFilter_init(&lpf_body_height_vel[3], 0.003, 2);
    }
    float body_vel[3] = {0};
    dog_get_body_vel(body_vel);
    // �������xy�����Ŀ����
    body_target_force[0] = kx * (body_vel[0] - est_get_body_vel(0)); // xy����ֻ�����ٶȿ���
    body_target_force[1] = ky * (body_vel[1] - est_get_body_vel(1));
    // ���������ת�����Ŀ����
    body_target_force[5] = kyaw * (body_vel[2] - est_get_body_vel(5));
    // �������z�����Ŀ����
    
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
        leg_force_z[i] = kpz[i] * (body_target_height - body_height) + kdz[i] * (0 - LPF_get_value(&lpf_body_height_vel[i])); // �߶�����pd����λ��
    }
    
//    // �߶�
//    set_debug_data(8, LPF_get_value(&lpf_body_height_vel[1]));
//    set_debug_data(9, -leg_current_vel[1][2]);

    // ���䵽�������
    for(int i = 0; i < 4; i++)
    {
        if(leg_get_contact_state(i) == 1) // λ��֧����
        {
            foot_force[i][0] = -body_target_force[0]; // ���x������
            foot_force[i][1] = -body_target_force[1]; // ���y������
            foot_force[i][2] = -leg_force_z[i]; // ���z������
        }
        else
        {
            memset(foot_force[i], 0, sizeof(foot_force[i])); // �ڶ��������Ϊ0
        }
    }
     // x����
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
    
    // ��ȡ�������λ����Ϣ
    leg_get_current_foot_force_pos_vel(leg_id, foot_force, foot_pos, foot_vel);
    
    // �ж�z�����Ƿ񳬹���ֵ
    if(foot_force[2] > CONTACT_FORCE_THRESHOLD)
    {
        return 1;  // ����
    }
    else
    {
        return 0;  // δ����
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
