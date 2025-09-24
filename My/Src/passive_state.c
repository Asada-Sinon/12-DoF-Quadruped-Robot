#include "fsm.h"
#include "dog.h"
#include "timer.h"
#include "gait.h"
#include "stdio.h"
#include "motor.h"
GaitState passive_state;    
/* 阻尼状态的处理函数 */
static void passive_enter(void) {
    // printf("进入阻尼状态\n");
    
}

static void passive_run(void) {
    for(int i = 0; i < 4; i++){
        leg_set_motor_force_vel_zero(i); // 清空电机目标力矩与速度
        leg_set_motor_kp_kd(i, 0, 9, 0, 9, 0, 9); // 设置电机为阻尼模式
    }
}

static void passive_exit(void) {
    // printf("退出阻尼状态\n");

}

/* 阻尼状态的转换检查 */
static bool passive_check_transition(StateName next) {
    if (next == STATE_STAND) // 阻尼状态只能切换到站立状态
        return true;
    return false;
}

/* 注册站立状态 */
void register_passive_state(void) {
    fsm_register_state(STATE_PASSIVE,
                     passive_enter,
                     passive_run,
                     passive_exit,
                     passive_check_transition);
}
