#include "fsm.h"
#include "dog.h"
#include "timer.h"
#include "gait.h"
#include "stdio.h"
#include "motor.h"
GaitState passive_state;    
/* �Խǲ�̬״̬�Ĵ����� */
static void passive_enter(void) {
    printf("��������״̬\n");
    for(int i = 0; i < 4; i++){
        leg_set_motor_kp_kd(i, 0, 8, 0, 8, 0, 8);
    }
}


static void passive_run(void) {

}

static void passive_exit(void) {
    printf("�˳�����״̬\n");

}

/* �Խǲ�̬״̬��ת����� */
static bool passive_check_transition(StateName next) {
    return true;
}

/* ע��վ��״̬ */
void register_passive_state(void) {
    fsm_register_state(STATE_PASSIVE,
                     passive_enter,
                     passive_run,
                     passive_exit,
                     passive_check_transition);
}
