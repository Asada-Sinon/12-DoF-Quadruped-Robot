#include "fsm.h"
#include "dog.h"
#include "timer.h"
#include "gait.h"
#include "stdio.h"

GaitState trot_state;    
/* �Խǲ�̬״̬�Ĵ����� */
static void trot_enter(void) {
    printf("����Խǲ�̬״̬\n");
    trot_state.time_start = getTime();   
}

// �ӵ�ǰλ�þ���trot_T�뵽��Ŀ��λ��
static void trot_run(void) {
    float phase[4] = {0};
    float motor_target_pos[4][3];

    phase_wave_generator(get_trot_params(), trot_state.wave_status, trot_state.time_start, phase, trot_state.contact);
    gait_generator(get_trot_params(), phase, trot_state.contact, trot_state.foot_target_pos);
    
    for(int i = 0; i < 4; i++){
        leg_set_target_foot_pos(i, trot_state.foot_target_pos[i]); // ������
        leg_foot_to_motor(i, trot_state.foot_target_pos[i], motor_target_pos[i]);
        leg_set_motor_pos(i, motor_target_pos[i]);
    }
}

static void trot_exit(void) {
    printf("�˳��Խǲ�̬״̬\n");
    // �뿪�Խǲ�̬״̬�Ĵ���
    // ...
}

/* �Խǲ�̬״̬��ת����� */
static bool trot_check_transition(StateName next) {
    // �ӶԽǲ�̬״̬����ת���κ�״̬
    if (next == STATE_STAND) {
        // if(first_run == 1)
        // {
        //     first_run = 0;
        //     printf("first_run = 0\n");
        //     get_gait_state()->end_T[0] = 1;
        //     get_gait_state()->end_T[1] = 1;
        //     get_gait_state()->end_T[2] = 1;
        //     get_gait_state()->end_T[3] = 1;
        // }
        // if(get_gait_state()->end_T[0] == 0 && get_gait_state()->end_T[1] == 0 && get_gait_state()->end_T[2] == 0 && get_gait_state()->end_T[3] == 0)
        // {
        //     for (int i = 0; i < 4; i++) {
        //         printf("�˳��Խǲ�̬ʱ���Ŀ��λ��[%d]: %f %f %f\n", i, foot_target_pos[i][X_IDX], foot_target_pos[i][Y_IDX], foot_target_pos[i][Z_IDX]);
        //     }
        //     return true;

        // }
        if(trot_state.contact[0] == 1 && trot_state.contact[1] == 1 && trot_state.contact[2] == 1 && trot_state.contact[3] == 1)
        {
            return true;
        }
    }
    return false;
}

/* ע��վ��״̬ */
void register_trot_state(void) {
    fsm_register_state(STATE_TROT,
                     trot_enter,
                     trot_run,
                     trot_exit,
                     trot_check_transition);
}
