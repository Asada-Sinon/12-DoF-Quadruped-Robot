#include "fsm.h"
#include "dog.h"
#include "timer.h"
#include "stdio.h"
#include "string.h"
#include "motor.h"

static float stand_T = 2.0f; // վ��ʱ���׼
static float T = 0; // ���ݾ�������վ��ʱ��
static float time_start = 0;
static float foot_start_pos[4][3];
static float foot_target_pos[4][3];
static float p = 0;
static float t = 0;
static float motor_current_pos[4][3];
float stand_test_foot_start_pos[4][3];
float stand_test_foot_target_pos[4][3];

float hip_kp = 70;
float hip_kd = 0.5;
float thigh_kp = 80;
float thigh_kd = 0.5;
float calf_kp = 90;
float calf_kd = 0.5;

/* վ��״̬�Ĵ����� */
static void stand_enter(void) {
    printf("����վ��״̬\n");
    RobotParams* params = get_robot_params();
    time_start = getTime();
    for (int i = 0; i < 4; i++){
        if (fsm_get_previous_state() == STATE_TROT) // ��һ��״̬�ǶԽǲ�̬������һʱ�̵�Ŀ��ֵ������ʼֵ����ֹ����ش��ӳ�
        {
//            leg_get_target_foot_pos(i, foot_start_pos[i]);
            leg_get_current_foot_pos(i, foot_start_pos[i]);
            printf("վ��״̬��ʼλ��[%d]: %f %f %f\n", i, foot_start_pos[i][X_IDX], foot_start_pos[i][Y_IDX], foot_start_pos[i][Z_IDX]);
        }
        else
        {
            leg_get_current_foot_pos(i, foot_start_pos[i]);
            leg_get_motors_current_pos(i, motor_current_pos[i]);
            leg_set_motor_pos(i, motor_current_pos[i]);
        }
            
    }   
    memcpy(stand_test_foot_start_pos, foot_start_pos, sizeof(foot_start_pos));
    for (int i = 0; i < 4; i++)
    {
        leg_set_motor_kp_kd(i, hip_kp, hip_kd, thigh_kp, thigh_kd, calf_kp, calf_kd);
    }
}

// �ӵ�ǰλ�þ���stand_T�뵽��Ŀ��λ��
static void stand_run(void) {
    // if (fsm_get_previous_state() != STATE_INVALID && fsm_get_current_state() != STATE_PASSIVE) {
    //     return;
    // }
    t = getTime() - time_start;
    float stand_height = dog_get_stand_height();
    float neutral_pos[4][3]; 
    
    float motor_target_pos[4][3];

    for (int i = 0; i < 4; i++)
    {
        leg_get_neutral_pos(i, neutral_pos[i]);
        leg_thigh_to_hip(i, neutral_pos[i], neutral_pos[i]);
    }
    for (int i = 0; i < 4; i++)
    {
        float xf = neutral_pos[i][X_IDX];
        float yf = neutral_pos[i][Y_IDX];
        float zf = neutral_pos[i][Z_IDX];

        float x0 = foot_start_pos[i][X_IDX];
        float y0 = foot_start_pos[i][Y_IDX];
        float z0 = foot_start_pos[i][Z_IDX];

        float length = sqrtf(powf(xf - x0, 2) + powf(yf - y0, 2) + powf(zf - z0, 2));
        T = length / stand_height * stand_T;

        p = t/T;
        if (p > 1)
            p = 1;

        float x = x0 + (xf - x0) * (1 - cosf(PI*p)) / 2.0f;
        float y = y0 + (yf - y0) * (1 - cosf(PI*p)) / 2.0f;
        float z = z0 + (zf - z0) * (1 - cosf(PI*p)) / 2.0f;

        foot_target_pos[i][X_IDX] = x;
        foot_target_pos[i][Y_IDX] = y;
        foot_target_pos[i][Z_IDX] = z;

        leg_foot_to_motor(i, foot_target_pos[i], motor_target_pos[i]);
        leg_set_motor_pos(i, motor_target_pos[i]);
    }
    memcpy(stand_test_foot_target_pos, foot_target_pos, sizeof(foot_target_pos));
}

static void stand_exit(void) {
    printf("�˳�վ��״̬\n");
}

/* վ��״̬��ת����� */
static bool stand_check_transition(StateName next) {
    // ��վ��״̬����ת���κ�״̬
    if (next == STATE_TROT)
        if(t >= T) // վ��T���ٿ�ʼ�Խǲ�̬
        {
            return true;
        }
    if (next == STATE_PASSIVE)
        return true;

    return false;
}

/* ע��վ��״̬ */
void register_stand_state(void) {
    fsm_register_state(STATE_STAND,
                     stand_enter,
                     stand_run,
                     stand_exit,
                     stand_check_transition);
}
