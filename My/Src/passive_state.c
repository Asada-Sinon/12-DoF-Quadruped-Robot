#include "fsm.h"
#include "dog.h"
#include "timer.h"
#include "gait.h"
#include "stdio.h"
#include "motor.h"
GaitState passive_state;    
/* 对角步态状态的处理函数 */
static void passive_enter(void) {
    printf("进入阻尼状态\n");
    for(int i = 0; i < 4; i++){
        leg_set_motor_kp_kd(i, 0, 8, 0, 8, 0, 8);
    }
}


static void passive_run(void) {

}

static void passive_exit(void) {
    printf("退出阻尼状态\n");

}

/* 对角步态状态的转换检查 */
static bool passive_check_transition(StateName next) {
    return true;
}

/* 注册站立状态 */
void register_passive_state(void) {
    fsm_register_state(STATE_PASSIVE,
                     passive_enter,
                     passive_run,
                     passive_exit,
                     passive_check_transition);
}
