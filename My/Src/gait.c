#include "gait.h"
#include "timer.h"
#include "stdint.h"
#include "dog.h"
#include "robot_params.h"
#include "stdio.h"

// 获取步态参数
GaitParams* get_trot_params() {
    return (GaitParams*)&get_robot_params()->trot_gait;
}

// 贝塞尔曲线未移植完成，部分移植完成的代码在4.10号上传的第一版中，现已删除

/**
 * @brief 计算足端目标位置（Raibert落足点算法）
 * @discription 没有观测器反馈，所以先假设v = vd，即期望速度与反馈速度相等
 * @param leg_id 腿的ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param body_vel 机体目标速度 [x速度, y速度, 角速度]
 * @param stance_duration 支撑相持续时间
 * @param end_pos 输出参数，足端位置 [x, y, z]
 * @return 步长
 */
float cal_foot_end_pos(int leg_id, const float body_vel[3], float stance_duration, float end_pos[3])
{
    // 提取速度分量
    float vx = body_vel[X_IDX];
    float vy = body_vel[Y_IDX];
    float vw = body_vel[Z_IDX];

    // w逆时针为正，根据腿坐标系方向确定w分解到xy轴的符号
    float VW_SIGNS[4][2] = {
        {-1.0f, +1.0f},  // FL: x负, y正
        {+1.0f, +1.0f},  // FR: x正, y正
        {-1.0f, -1.0f},  // HL: x负, y负
        {+1.0f, -1.0f}   // HR: x正, y负
    };

    // 将角速度分解到腿坐标系
    vx += VW_SIGNS[leg_id][0] * vw * sinf(HIP_ANGLE);
    vy += VW_SIGNS[leg_id][1] * vw * cosf(HIP_ANGLE);

    // 使用Raibert启发式方法将速度转换为步长
    // 步长 = (支撑相时间/2) * 速度
    end_pos[X_IDX] = (stance_duration / 2.0f) * vx;
    end_pos[Y_IDX] = (stance_duration / 2.0f) * vy;
    end_pos[Z_IDX] = 0.0f;  // 默认高度为0

    return 2*sqrtf(end_pos[X_IDX]*end_pos[X_IDX] + end_pos[Y_IDX]*end_pos[Y_IDX]);
}

/**
 * @brief 生成步态相位和接触状态
 * 
 * @param gait 步态参数
 * @param status 步态状态：正常步态、全摆动、全支撑
 * @param phase 输出参数，各腿相位值数组[4]，范围0~1
 * @param contact 输出参数，各腿接触状态数组[4]，1=支撑相，0=摆动相
 */
void phase_wave_generator(GaitParams *gait, WaveStatus status, float start_time, float *phase, int *contact)
{
    // 全部腿的默认相位值
    const float default_phase = 0.5f;
    // 根据步态状态生成相应的相位和接触状态
    switch(status)
    {
        case WAVE_ALL:  // 正常步态模式
        {
            // 计算从开始时间到现在的时间（秒）
            float t = getTime() - start_time;
            // 计算每条腿的相位和接触状态
            for (int i = 0; i < 4; i++)
            {
                // 计算归一化时间，考虑相位偏移
                float normalized_time = fmod(t + gait->T - gait->T * gait->phase[i], gait->T) / gait->T;
                // 判断是否在支撑相
                if (normalized_time < gait->stance_ratio)
                {
                    // 支撑相
                    contact[i] = 1;
                    phase[i] = normalized_time / gait->stance_ratio;
                }
                else
                {
                    // 摆动相
                    contact[i] = 0;
                    phase[i] = (normalized_time - gait->stance_ratio) / (1 - gait->stance_ratio);
                }
            }
            break;
        }
        case SWING_ALL:  // 全摆动模式
            // 所有腿都处于摆动相
            for (int i = 0; i < 4; i++) 
            {
                contact[i] = 0;
                phase[i] = default_phase;
            }
            break;
        case STANCE_ALL:  // 全支撑模式
            // 所有腿都处于支撑相
            for (int i = 0; i < 4; i++) 
            {
                contact[i] = 1;
                phase[i] = default_phase;
            }
            break;
    }
}

void gait_generator(GaitParams *gait, float *phase, int *contact, float foot_target_pos[4][3])
{
    float stand_height = dog_get_stand_height();
    float stance_duration = gait->stance_ratio * gait->T;
    
    float foot_target_pos_thigh[4][3];
    // 获取各腿足端中性点位置
    float neutral_pos[4][3]; 
    float end_P[4][3];
    float body_vel[3];
    
    float x = 0.0f;  
    float y = 0.0f;  
    float z = 0.0f;  
    
    // 获取当前速度和中性点位置
    dog_get_body_vel(body_vel);
    
    // 获取中性点位置 - 修改为针对单腿获取
    for (int i = 0; i < 4; i++) {
        leg_get_neutral_pos(i, neutral_pos[i]);
    }

    for(int i = 0; i < 4; ++i){
        if(contact[i] == 1){ // 处于支撑相   
            // 计算支撑相落足点位置，后续用力代替
            cal_foot_end_pos(i, body_vel, stance_duration, end_P[i]);   
            // 计算支撑相中的水平位置：从step_length/2到-step_length/2线性变化
            x = end_P[i][X_IDX] * (1 - (2 * phase[i]));
            y = end_P[i][Y_IDX] * (1 - (2 * phase[i]));
            // 支撑相中足端略微下压，提供更好的地面接触
            z = -gait->stance_depth * cosf(PI * (phase[i] - 0.5f));
        }
        else{ // 处于摆动相 
            // 规划落足点位置 
            cal_foot_end_pos(i, body_vel, stance_duration, end_P[i]);

            float fai = 2 * PI * phase[i];
            x = end_P[i][X_IDX] * (fai - sinf(fai)) / PI - end_P[i][X_IDX];
            y = end_P[i][Y_IDX] * (fai - sinf(fai)) / PI - end_P[i][Y_IDX];
            z = gait->swing_height * (1 - cosf(fai)) / 2.0f;
        }
        // xy转换到thigh坐标系下
        // 调试用重心补偿
        //if (i == LEG_HL || i == LEG_HR)
        //{
        //    foot_target_pos_thigh[i].z -= 0.00f;
        //}
        foot_target_pos_thigh[i][X_IDX] = x + neutral_pos[i][X_IDX];
        foot_target_pos_thigh[i][Y_IDX] = y + neutral_pos[i][Y_IDX];
        foot_target_pos_thigh[i][Z_IDX] = z + neutral_pos[i][Z_IDX];

        // xyz转换到hip坐标系下
        leg_thigh_to_hip(i, foot_target_pos_thigh[i], foot_target_pos[i]);
    }
}


