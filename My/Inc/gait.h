#ifndef GAIT_H
#define GAIT_H
#include "robot_params.h"
#include "dog.h"

typedef enum {
    WAVE_ALL,    // ������̬���Σ������Ȱ���Ԥ�����λ����˶�
    SWING_ALL,   // ������ͬʱ���ڰڶ���
    STANCE_ALL   // ������ͬʱ����֧����
} WaveStatus;

// ��ǰ��̬״̬�ṹ��
typedef struct {
    float time_start;
    WaveStatus wave_status;
    float foot_start_pos[4][3];
    float foot_target_pos[4][3];
    int contact[4];
} GaitState;

// ��ȡ��̬����
GaitParams* get_trot_params(void);

// ��ȡ��̬״̬
GaitState* get_gait_state(void);

/**
 * @brief �������Ŀ��λ�ã�Raibert������㷨��
 * @param leg_id �ȵ�ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param body_vel ����Ŀ���ٶ� [x�ٶ�, y�ٶ�, ���ٶ�]
 * @param stance_duration ֧�������ʱ��
 * @param end_pos ������������λ�� [x, y, z]
 * @return ����
 */
float cal_foot_end_pos(int leg_id, const float body_vel[3], float stance_duration, float end_pos[3]);

/**
 * @brief ���ɲ�̬��λ�ͽӴ�״̬
 * @param gait ��̬����
 * @param status ��̬״̬��������̬��ȫ�ڶ���ȫ֧��
 * @param phase ���������������λֵ����[4]����Χ0~1
 * @param contact ������������ȽӴ�״̬����[4]��1=֧���࣬0=�ڶ���
 */
void phase_wave_generator(GaitParams *gait, WaveStatus status, float start_time, float *phase, int *contact);

/**
 * @brief ���ɲ�̬
 * @param gait ��̬����
 * @param phase ������λֵ����[4]
 * @param contact ���ȽӴ�״̬����[4]
 * @param foot_target_pos �������������Ŀ��λ�� [4][3]
 */
void gait_generator(GaitParams *gait, float *phase, int *contact, float foot_target_pos[4][3]);

/**
 * @brief ���ò�̬��λ״̬
 * @param wave_status ��̬��λ״̬
 */
void set_wave_status(WaveStatus wave_status);

/**
 * @brief ����С�ܲ�̬
 */
void start_gait_trot(void);

/**
 * @brief ���ò�̬��λ״̬
 * @param status ��̬��λ״̬
 */
void gait_set_wave_status(WaveStatus status);

float* gait_get_phase(void);

int* gait_get_contact(void);

float* gait_get_foot_target_pos(int leg_idx);


#endif

