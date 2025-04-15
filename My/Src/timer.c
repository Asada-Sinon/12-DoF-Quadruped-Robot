#include "timer.h"
#include "tim.h"
#include "dog.h"
#include "j60.h"
#include "fsm.h"
#include "motor.h"

// ��ȡϵͳʱ��
// long long getSystemTime() {
//     clock_t now = clock();
//     return (long long)(now * 1000000LL / CLOCKS_PER_SEC);
// }

// ��ȡ��ǰʱ�䣨�룩
double getTime() {
    return (double)(HAL_GetTick() * 1e-3);
}

void timer_init()
{
    HAL_TIM_Base_Start_IT(&htim1); // 1ms����can������Ϣ
    HAL_TIM_Base_Start_IT(&htim2); // 2ms���͵����Ϣ
    HAL_TIM_Base_Start_IT(&htim3); // 2ms״̬��
}

float motors_current_pos[4][3];
float joints_current_pos[4][3];
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        // 1ms����can������Ϣ
        J60_ProcessCanQueues();
    }
    if (htim->Instance == TIM2) {
        // 2ms���͵����Ϣ
        dog_send_motors();
        for (int i = 0; i < 4; i++) {
            leg_get_motors_current_pos(i, motors_current_pos[i]);
//            leg_motor_to_joint(i, motors_current_pos[i], joints_current_pos[i]);
        }

    }
    if (htim->Instance == TIM3) {
        // 2ms״̬��
        fsm_update();
    }
}
