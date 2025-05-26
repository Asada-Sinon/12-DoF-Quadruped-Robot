#include "fsm.h"
#include "dog.h"
#include "timer.h"
#include "gait.h"
#include "stdio.h"
#include "motor.h"


GaitState trot_state;    

typedef struct pd{
    float kp;
    float kd;
}pd;

pd swing_hip[4] = { {120 , 0.5}, {120, 2}, {120, 2}, {120, 2} };
pd swing_thigh[4] = { {100 , 3}, {150, 3}, {150, 3}, {150, 3} };
pd swing_calf[4] = { {100 , 3}, {120, 4}, {120, 4}, {120, 4} };

pd stance_hip[4] = { {120 , 2}, {120, 2}, {120, 2}, {120, 2} };
pd stance_thigh[4] = { {140 , 5}, {150, 3}, {150, 3}, {150, 3} };
pd stance_calf[4] = { {100 , 5}, {120, 4}, {120, 4}, {120, 4} };

// pd swing_hip[4] = { {0 , 0}, {0, 0}, {0, 0}, {0, 0} };
// pd swing_thigh[4] = { {0 , 0}, {0, 0}, {0, 0}, {0, 0} };
// pd swing_calf[4] = { {0 , 0}, {0, 0}, {0, 0}, {0, 0} };

// pd stance_hip[4] = { {0 , 0}, {0, 0}, {0, 0}, {0, 0} };
// pd stance_thigh[4] = { {0 , 0}, {0, 0}, {0, 0}, {0, 0} };
// pd stance_calf[4] = { {0 , 0}, {0, 0}, {0, 0}, {0, 0} };

//pd swing_hip[4] = { {10 , 3}, {10, 3}, {10, 3}, {10, 3} };
//pd swing_thigh[4] = { {10 , 3}, {10, 3}, {10, 3}, {10, 3} };
//pd swing_calf[4] = { {10 , 3}, {10, 3}, {10, 3}, {10, 3} };

//pd stance_hip[4] = { {10 , 3}, {10, 3}, {10, 3}, {10, 3} };
//pd stance_thigh[4] = { {10 , 3}, {10, 3}, {10, 3}, {10, 3} };
//pd stance_calf[4] = { {10 , 3}, {10, 3}, {10, 3}, {10, 3} };

/* �Խǲ�̬״̬�Ĵ����� */
static void trot_enter(void) {
    // printf("����Խǲ�̬״̬\n");
    trot_state.time_start = getTime();   
}

// �ӵ�ǰλ�þ���trot_T�뵽��Ŀ��λ��
static void trot_run(void) {
    float phase[4] = {0};
    float motor_target_pos[4][3];
    float motor_target_vel[4][3];
    float s_body_vel[3];
    dog_get_body_vel_without_cog(s_body_vel); // ��ȡ��ǰ�����ٶ�
    if (s_body_vel[X_IDX] > 0) // ǰ��ʱʹ��ǰ�������Ĳ���
    {
        get_dog_params()->posture.center_of_gravity.translation[X_IDX] = get_dog_params()->posture.center_of_gravity.trot_cog_forward_offset[X_IDX];
    }
    else if(s_body_vel[X_IDX] < 0)// ����ʱʹ�ú��˵����Ĳ���
    {
        get_dog_params()->posture.center_of_gravity.translation[X_IDX] = get_dog_params()->posture.center_of_gravity.trot_cog_backward_offset[X_IDX];
    }
    else // վ��ʱ����
    {
        get_dog_params()->posture.center_of_gravity.translation[X_IDX] = get_dog_params()->posture.center_of_gravity.stand_cog_offset[X_IDX];
    }
    
    phase_wave_generator(get_trot_params(), trot_state.wave_status, trot_state.time_start, phase, trot_state.contact);
    gait_generator(get_trot_params(), phase, trot_state.contact, trot_state.foot_target_pos, trot_state.foot_target_vel);
    
    for(int i = 0; i < 4; i++){
        if (trot_state.contact[i] == 0) // �ڶ���
            leg_set_motor_kp_kd(i, swing_hip[0].kp, swing_hip[0].kd, swing_thigh[0].kp, swing_thigh[0].kd, swing_calf[0].kp, swing_calf[0].kd);
        else // ֧����
            leg_set_motor_kp_kd(i, stance_hip[0].kp, stance_hip[0].kd, stance_thigh[0].kp, stance_thigh[0].kd, stance_calf[0].kp, stance_calf[0].kd);

        leg_set_target_foot_pos(i, trot_state.foot_target_pos[i]); // ������
        
        // leg_foot_to_motor(i, trot_state.foot_target_pos[i], motor_target_pos[i]);
        // leg_set_motor_pos(i, motor_target_pos[i]);

        leg_foot_to_motor_pos_vel(i, trot_state.foot_target_pos[i], trot_state.foot_target_vel[i], motor_target_pos[i], motor_target_vel[i]);
        leg_set_motor_pos_vel(i, motor_target_pos[i], motor_target_vel[i]);
    }
}

static void trot_exit(void) {
    // printf("�˳��Խǲ�̬״̬\n");
    // �뿪�Խǲ�̬״̬�Ĵ���
    // ...
}

/* �Խǲ�̬״̬��ת����� */
static bool trot_check_transition(StateName next) {
    // �ӶԽǲ�̬״̬����ת���κ�״̬
    if (next == STATE_STAND) {
        if(trot_state.contact[0] == 1 && trot_state.contact[1] == 1 && trot_state.contact[2] == 1 && trot_state.contact[3] == 1)
        {
            return true;
        }
    }
    if (next == STATE_PASSIVE)
        return true;
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
