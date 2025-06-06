#ifndef GAIT_H
#define GAIT_H
#include "robot_params.h"
#include "dog.h"

typedef enum {
    WAVE_ALL,    // 正常步态波形，四条腿按照预设的相位差交替运动
    SWING_ALL,   // 所有腿同时处于摆动相
    STANCE_ALL   // 所有腿同时处于支撑相
} WaveStatus;

// 当前步态状态结构体
typedef struct {
    float time_start;
    WaveStatus wave_status;
    float foot_start_pos[4][3];
    float foot_target_pos[4][3];
    float foot_target_vel[4][3];
    float foot_target_force[4][3];
    int contact[4];
    float phase[4];
} GaitState;

// 获取步态参数
GaitParams* get_trot_params(void);

//// 获取步态状态
GaitState* get_gait_state(void);


/**
 * @brief 生成步态相位和接触状态
 * @param gait 步态参数
 * @param status 步态状态：正常步态、全摆动、全支撑
 * @param phase 输出参数，各腿相位值数组[4]，范围0~1
 * @param contact 输出参数，各腿接触状态数组[4]，1=支撑相，0=摆动相
 */
void phase_wave_generator(GaitParams *gait, WaveStatus status, float start_time, float *phase, int *contact);

/**
 * @brief 根据接触力生成步态相位和接触状态
 * @param gait 步态参数
 * @param status 步态状态：正常步态、全摆动、全支撑
 * @param phase 输出参数，各腿相位值数组[4]，范围0~1
 * @param contact 输出参数，各腿接触状态数组[4]，1=支撑相，0=摆动相
 */
void phase_wave_generator_with_force(GaitParams *gait, WaveStatus status, float start_time, float *phase, int *contact);

/**
 * @brief 生成步态
 * @param gait 步态参数
 * @param phase 各腿相位值数组[4]
 * @param contact 各腿接触状态数组[4]
 * @param foot_target_pos 输出参数，各腿目标位置 [4][3]
 * @param foot_target_vel 输出参数，各腿目标速度 [4][3]
 */
void gait_generator(GaitParams *gait, float *phase, int *contact, float foot_target_pos[4][3], float foot_target_vel[4][3]);

/**
 * @brief 设置步态相位状态
 * @param wave_status 步态相位状态
 */
void set_wave_status(WaveStatus wave_status);

/**
 * @brief 启动小跑步态
 */
void start_gait_trot(void);

/**
 * @brief 设置步态相位状态
 * @param status 步态相位状态
 */
void gait_set_wave_status(WaveStatus status);

float* gait_get_phase(void);

int* gait_get_contact(void);

float* gait_get_foot_target_pos(int leg_idx);

//根据速度调整步频
void adjust_gait_frequency(void);
#endif

