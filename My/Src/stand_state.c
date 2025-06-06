#include "fsm.h"
#include "dog.h"
#include "timer.h"
#include "stdio.h"
#include "string.h"
#include "motor.h"
#include "force_calculate.h"
#include "ANO_TC.h"
#include "estimator.h"

static float stand_T = 2.0f; // վ��ʱ���׼
static float T = 0; // ���ݾ�������վ��ʱ��
static float time_start = 0;
static float foot_start_pos[4][3];
static float foot_target_pos[4][3];
static float foot_target_vel[4][3];

static float p = 0;
static float t = 0;
static float motor_current_pos[4][3];
float stand_test_foot_start_pos[4][3];
float stand_test_foot_target_pos[4][3];

float stand_test_foot_current_force[4][3] = {0};
float stand_test_foot_current_pos[4][3] = {0};
float stand_test_foot_current_vel[4][3] = {0};

float stand_test_foot_target_force[4][3];

typedef struct pd{
    float kp;
    float kd;
}pd;
 pd pos_hip[4] = { {120 , 2}, {120, 2}, {120, 2}, {120, 2} };
 pd pos_thigh[4] = { {100 , 5}, {100 , 5}, {100 , 5}, {100 , 5} };
 pd pos_calf[4] = { {80 , 5}, {80, 5}, {80, 5}, {80, 5} };

//pd pos_hip[4] = { {0 , 0}, {0, 0}, {0, 0}, {0, 0} };
//pd pos_thigh[4] = { {0 , 0}, {0, 0}, {0, 0}, {0, 0} };
//pd pos_calf[4] = { {0 , 0}, {0, 0}, {0, 0}, {0, 0} };

//pd pos_hip[4] = { {5 , 1}, {5, 1}, {5, 1}, {5, 1} };
//pd pos_thigh[4] = { {5 , 1}, {5, 1}, {5, 1}, {5, 1} };
//pd pos_calf[4] = { {5 , 1}, {5, 1}, {5, 1}, {5, 1} };

pd force_hip[4] = { {0 , 0}, {0, 0}, {0, 0}, {0, 0} };
pd force_thigh[4] = { {0 , 0}, {0, 0}, {0, 0}, {0, 0} };
pd force_calf[4] = { {0 , 0}, {0, 0}, {0, 0}, {0, 0} };

//pd force_hip[4] = { {1 , 0.5}, {1, 0.5}, {1, 0.5}, {1, 0.5} };
//pd force_thigh[4] = { {1 , 0.5}, {1, 0.5}, {1, 0.5}, {1, 0.5} };
//pd force_calf[4] = { {1 , 1}, {1, 1}, {1, 1}, {1, 1} };


/* վ��״̬�Ĵ����� */
static void stand_enter(void) {
    // printf("����վ��״̬\n");
    RobotParams* params = get_dog_params();
    time_start = getTime();
    for (int i = 0; i < 4; i++){
        if (fsm_get_previous_state() == STATE_TROT) // ��һ��״̬�ǶԽǲ�̬�������а���һʱ�̵�Ŀ��ֵ������ʼֵ����ֹ����ش��ӳ١�ʵ�������ѵ�ǰֵ����ʼֵ���С�
        {
//            leg_get_target_foot_pos(i, foot_start_pos[i]);
            leg_get_current_foot_pos(i, foot_start_pos[i]);
            // printf("վ��״̬��ʼλ��[%d]: %f %f %f\n", i, foot_start_pos[i][X_IDX], foot_start_pos[i][Y_IDX], foot_start_pos[i][Z_IDX]);
        }
        else
        {
            leg_get_current_foot_pos(i, foot_start_pos[i]);
            leg_get_motors_current_pos(i, motor_current_pos[i]);
            leg_set_motor_pos(i, motor_current_pos[i]);
        }
        

        // ����Ϊȫ֧�š���λΪ0.5
        get_dog_params()->posture.contact[i] = 1;
        get_dog_params()->posture.phase[i] = 0.5;
    }
            
     
    memcpy(stand_test_foot_start_pos, foot_start_pos, sizeof(foot_start_pos));
    for (int i = 0; i < 4; i++)
    {
        leg_set_motor_kp_kd(i, pos_hip[i].kp, pos_hip[i].kd, pos_thigh[i].kp, pos_thigh[i].kd, pos_calf[i].kp, pos_calf[i].kd);
    }
}

float motor_target_force[4][3] = {0};
// �ӵ�ǰλ�þ���stand_T�뵽��Ŀ��λ��
static void stand_run(void) {
    t = getTime() - time_start;
    float stand_height = dog_get_stand_height();
    float neutral_pos[4][3]; 
    RobotParams* params = get_dog_params();
    float motor_target_pos[4][3];
    float motor_target_vel[4][3] = {0};
    

    for (int i = 0; i < 4; i++)
    {
        leg_get_neutral_current_pos(i, neutral_pos[i]);
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

        float x = x0 + (xf - x0) * (1 - cosf(MY_PI*p)) / 2.0f;
        float y = y0 + (yf - y0) * (1 - cosf(MY_PI*p)) / 2.0f;
        float z = z0 + (zf - z0) * (1 - cosf(MY_PI*p)) / 2.0f;

        foot_target_pos[i][X_IDX] = x;
        foot_target_pos[i][Y_IDX] = y;
        foot_target_pos[i][Z_IDX] = z;
        
        // ���أ�����ǴӶԽǲ�̬�л�������Ϊ�˱����ȶ���ֱ��������
        if (vmc_get_force_control_state() == 1 && fsm_get_previous_state() == STATE_TROT)
        {
            vmc_get_foot_target_force(i, stand_test_foot_target_force[i]);
            leg_set_motor_kp_kd(i, force_hip[i].kp, force_hip[i].kd, force_thigh[i].kp, force_thigh[i].kd, force_calf[i].kp, force_calf[i].kd);
        }
            
        else // λ�أ���Ϊ����ûдλ��Լ��������վ��Ҫ��λ��
        {
            memset(stand_test_foot_target_force[i], 0, sizeof(stand_test_foot_target_force[i]));
            leg_set_motor_kp_kd(i, pos_hip[i].kp, pos_hip[i].kd, pos_thigh[i].kp, pos_thigh[i].kd, pos_calf[i].kp, pos_calf[i].kd);
        }
        // ������
//        memset(stand_test_foot_target_force[i], 0, sizeof(stand_test_foot_target_force[i]));
        
        // ��������set_target_foot_pos
        leg_set_target_foot_pos(i, foot_target_pos[i]);
        leg_foot_to_motor_force_pos_vel(i, stand_test_foot_target_force[i], foot_target_pos[i], foot_target_vel[i], motor_target_force[i], motor_target_pos[i], motor_target_vel[i]);
        leg_set_motor_force_pos_vel(i, motor_target_force[i], motor_target_pos[i], motor_target_vel[i]);

        // ��ȡ�������������
        leg_get_current_foot_force_pos_vel(i, stand_test_foot_current_force[i], stand_test_foot_current_pos[i], stand_test_foot_current_vel[i]);
        set_debug_data(2, stand_test_foot_current_force[0][2]);
        set_debug_data(3, stand_test_foot_current_force[1][2]);
        set_debug_data(4, stand_test_foot_current_force[2][2]);
        set_debug_data(5, stand_test_foot_current_force[3][2]);
//        
//        set_debug_data(6, stand_test_foot_current_force[0][2]);
//        set_debug_data(7, stand_test_foot_current_force[1][2]);
//        set_debug_data(8, stand_test_foot_current_force[2][2]);
//        set_debug_data(9, stand_test_foot_current_force[3][2]);
        // leg_foot_to_motor(i, foot_target_pos[i], motor_target_pos[i]);
        // leg_set_motor_pos_vel(i, motor_target_pos[i], motor_target_vel[i]);
    }
    memcpy(stand_test_foot_target_pos, foot_target_pos, sizeof(foot_target_pos));
}

static void stand_exit(void) {
    // printf("�˳�վ��״̬\n");
}

/* վ��״̬��ת����� */
static bool stand_check_transition(StateName next) {
    // ��վ��״̬����ת���κ�״̬
    if (next == STATE_TROT)
        if(t >= T) // ȷ�����վ�����ٽ���Խǲ�̬
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
