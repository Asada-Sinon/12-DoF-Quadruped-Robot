#include "fsm.h"
#include "dog.h"
#include "timer.h"
#include "gait.h"
#include "stdio.h"
#include "motor.h"
GaitState passive_state;    
/* ����״̬�Ĵ����� */
static void passive_enter(void) {
    // printf("��������״̬\n");
    
}

static void passive_run(void) {
    for(int i = 0; i < 4; i++){
        leg_set_motor_force_vel_zero(i); // ��յ��Ŀ���������ٶ�
        leg_set_motor_kp_kd(i, 0, 9, 0, 9, 0, 9); // ���õ��Ϊ����ģʽ
    }
}

static void passive_exit(void) {
    // printf("�˳�����״̬\n");

}

/* ����״̬��ת����� */
static bool passive_check_transition(StateName next) {
    if (next == STATE_STAND) // ����״ֻ̬���л���վ��״̬
        return true;
    return false;
}

/* ע��վ��״̬ */
void register_passive_state(void) {
    fsm_register_state(STATE_PASSIVE,
                     passive_enter,
                     passive_run,
                     passive_exit,
                     passive_check_transition);
}
