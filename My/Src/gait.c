#include "gait.h"
#include "timer.h"
#include "stdint.h"
#include "dog.h"
#include "robot_params.h"
#include "stdio.h"
#include "estimator.h"

static void limit(float *value, float min, float max)
{
    if (*value < min) *value = min;
    if (*value > max) *value = max;
}

// 获取步态参数
GaitParams* get_trot_params() {
    return (GaitParams*)&get_dog_params()->trot_gait;
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
float vbx = 0;
float vby = 0;
float x_adjust = 0;
float y_adjust = 0;
float cal_foot_end_pos(int leg_id, GaitParams *gait, const float body_vel[3], float end_pos[3])
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

    vbx = est_get_body_vel(0);
    vby = est_get_body_vel(1);
    float phase = get_dog_params()->posture.phase[leg_id];
    float Tswing = gait->T * gait->swing_ratio;
    float Tstance = gait->T * gait->stance_ratio;
    float kx = gait->kx;
    float ky = gait->ky;
    // tudo kx*(vbx - vx)加范围,调大kx
    x_adjust = kx*(vbx - vx);
    y_adjust = ky*(vby - vy);
    limit(&x_adjust, -0.05f, 0.05f);
    limit(&y_adjust, -0.05f, 0.05f);

    end_pos[X_IDX] = vbx*(1-phase)*Tswing + vbx*Tstance/2.0f + x_adjust;
    end_pos[Y_IDX] = vby*(1-phase)*Tswing + vby*Tstance/2.0f + y_adjust;
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
    for(int i = 0; i < 4; ++i){
        // 实时更新机器人的步态状态
        get_dog_params()->posture.contact[i] = contact[i];
        get_dog_params()->posture.phase[i] = phase[i];
    }
}

float p0[4][3] = {0};
float pf[4][3] = {0};
float ratio = 1.8; // 如果支撑相滑动，减小ratio
void gait_generator(GaitParams *gait, float *phase, int *contact, float foot_target_pos[4][3], float foot_target_vel[4][3])
{
    float foot_target_pos_thigh[4][3];
    // 获取各腿足端中性点位置
    float neutral_pos[4][3]; 
    float body_vel[3];
    
    // static float p0[4][3] = {0};
    float h = gait->swing_height;
    float T = gait->T;

    float x = 0.0f;  
    float y = 0.0f;  
    float z = 0.0f;  

    float vx = 0.0f;
    float vy = 0.0f;
    float vz = 0.0f;
    
    // 获取当前速度和中性点位置
    dog_get_body_vel(body_vel);
    
    // 获取中性点位置 - 修改为针对单腿获取
    for (int i = 0; i < 4; i++) {
        leg_get_neutral_current_pos(i, neutral_pos[i]);
    }

    for(int i = 0; i < 4; ++i){
        if(contact[i] == 1){ // 处于支撑相   
            // 支撑相应尽量保持世界坐标系下足端无滑动
            // 如果处于支撑相的开始，更新足端起始位置
            if (phase[i] <= 0.01f)
                leg_get_current_foot_pos(i, p0[i]);
            x = p0[i][X_IDX] - body_vel[X_IDX] * T * gait->stance_ratio * phase[i] * ratio;
            y = p0[i][Y_IDX] - body_vel[Y_IDX] * T * gait->stance_ratio * phase[i] * ratio;
            z = 0;

            vx = 0;
            vy = 0;
            vz = 0;
        }
        else{ // 处于摆动相 
            // 规划落足点位置 
            cal_foot_end_pos(i, gait, body_vel, pf[i]);
            leg_thigh_to_hip(i, pf[i], pf[i]);
            // 如果处于摆动相的开始，更新足端起始位置
            if (phase[i] <= 0.01f)
                leg_get_current_foot_pos(i, p0[i]);
            // 摆线轨迹
            float fai = 2 * MY_PI * phase[i];
            x = (pf[i][X_IDX] - p0[i][X_IDX]) * (fai - sinf(fai)) / (2*MY_PI) + p0[i][X_IDX];
            y = (pf[i][Y_IDX] - p0[i][Y_IDX]) * (fai - sinf(fai)) / (2*MY_PI) + p0[i][Y_IDX];
            z = h * (1 - cosf(fai)) / 2.0f;
            // 摆线速度
//            vx = (pf[i][X_IDX] - p0[i][X_IDX]) / T * (1 - cosf(fai));
//            vy = (pf[i][Y_IDX] - p0[i][Y_IDX]) / T * (1 - cosf(fai));
//            vz = MY_PI * h / T * sinf(fai);
            vx = 0;
            vy = 0;
            vz = 0;
        }
        // xy转换到thigh坐标系下
        // 调试用重心补偿
        //if (i == LEG_HL || i == LEG_HR)
        //{
        //    foot_target_pos_thigh[i].z -= 0.00f;
        //}
        foot_target_pos[i][X_IDX] = x + neutral_pos[i][X_IDX];
        foot_target_pos[i][Y_IDX] = y + neutral_pos[i][Y_IDX];
        foot_target_pos[i][Z_IDX] = z + neutral_pos[i][Z_IDX];

        foot_target_vel[i][X_IDX] = vx;
        foot_target_vel[i][Y_IDX] = vy;
        foot_target_vel[i][Z_IDX] = vz;
    }
}


