#include "timer.h"
#include "tim.h"
#include "dog.h"
#include "j60.h"
#include "fsm.h"
#include "motor.h"
#include "estimator.h"
#include "gamepad.h"
#include "ANO_TC.h"

// 获取系统时间
// long long getSystemTime() {
//     clock_t now = clock();
//     return (long long)(now * 1000000LL / CLOCKS_PER_SEC);
// }

// 获取当前时间（秒）
double getTime() {
    return (double)(HAL_GetTick() * 1e-3);
}

void timer_init()
{
    HAL_TIM_Base_Start_IT(&htim1); // 1ms处理can队列信息
    HAL_TIM_Base_Start_IT(&htim2); // 4ms发送电机信息
    HAL_TIM_Base_Start_IT(&htim3); // 2ms状态机
    HAL_TIM_Base_Start_IT(&htim4); // 3ms状态观测器
    HAL_TIM_Base_Start_IT(&htim5); // 10ms上位机
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        // 1ms处理can队列信息
        J60_ProcessCanQueues();
    }
    if (htim->Instance == TIM2) {
        // 4ms发送电机信息
        dog_send_motors();
    }
    if (htim->Instance == TIM3) {
        // 全局调整重心
        dog_smooth_cog(0.0007);
        // 手柄
        gamepad_control();
        // 2ms状态机
        fsm_update();
    }
    if (htim->Instance == TIM4) {
        // 状态观测器
        estimation_run();
    }
    if (htim->Instance == TIM5) {
        // 上位机
        send_debug_data();
    }
}
