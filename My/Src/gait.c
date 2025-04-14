#include "gait.h"
#include "timer.h"
#include "stdint.h"
#include "dog.h"
#include "robot_params.h"
#include "stdio.h"

// ��ȡ��̬����
GaitParams* get_trot_params() {
    return (GaitParams*)&get_robot_params()->trot_gait;
}

// ����������δ��ֲ��ɣ�������ֲ��ɵĴ�����4.10���ϴ��ĵ�һ���У�����ɾ��

/**
 * @brief �������Ŀ��λ�ã�Raibert������㷨��
 * @discription û�й۲��������������ȼ���v = vd���������ٶ��뷴���ٶ����
 * @param leg_id �ȵ�ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param body_vel ����Ŀ���ٶ� [x�ٶ�, y�ٶ�, ���ٶ�]
 * @param stance_duration ֧�������ʱ��
 * @param end_pos ������������λ�� [x, y, z]
 * @return ����
 */
float cal_foot_end_pos(int leg_id, const float body_vel[3], float stance_duration, float end_pos[3])
{
    // ��ȡ�ٶȷ���
    float vx = body_vel[X_IDX];
    float vy = body_vel[Y_IDX];
    float vw = body_vel[Z_IDX];

    // w��ʱ��Ϊ��������������ϵ����ȷ��w�ֽ⵽xy��ķ���
    float VW_SIGNS[4][2] = {
        {-1.0f, +1.0f},  // FL: x��, y��
        {+1.0f, +1.0f},  // FR: x��, y��
        {-1.0f, -1.0f},  // HL: x��, y��
        {+1.0f, -1.0f}   // HR: x��, y��
    };

    // �����ٶȷֽ⵽������ϵ
    vx += VW_SIGNS[leg_id][0] * vw * sinf(HIP_ANGLE);
    vy += VW_SIGNS[leg_id][1] * vw * cosf(HIP_ANGLE);

    // ʹ��Raibert����ʽ�������ٶ�ת��Ϊ����
    // ���� = (֧����ʱ��/2) * �ٶ�
    end_pos[X_IDX] = (stance_duration / 2.0f) * vx;
    end_pos[Y_IDX] = (stance_duration / 2.0f) * vy;
    end_pos[Z_IDX] = 0.0f;  // Ĭ�ϸ߶�Ϊ0

    return 2*sqrtf(end_pos[X_IDX]*end_pos[X_IDX] + end_pos[Y_IDX]*end_pos[Y_IDX]);
}

/**
 * @brief ���ɲ�̬��λ�ͽӴ�״̬
 * 
 * @param gait ��̬����
 * @param status ��̬״̬��������̬��ȫ�ڶ���ȫ֧��
 * @param phase ���������������λֵ����[4]����Χ0~1
 * @param contact ������������ȽӴ�״̬����[4]��1=֧���࣬0=�ڶ���
 */
void phase_wave_generator(GaitParams *gait, WaveStatus status, float start_time, float *phase, int *contact)
{
    // ȫ���ȵ�Ĭ����λֵ
    const float default_phase = 0.5f;
    // ���ݲ�̬״̬������Ӧ����λ�ͽӴ�״̬
    switch(status)
    {
        case WAVE_ALL:  // ������̬ģʽ
        {
            // ����ӿ�ʼʱ�䵽���ڵ�ʱ�䣨�룩
            float t = getTime() - start_time;
            // ����ÿ���ȵ���λ�ͽӴ�״̬
            for (int i = 0; i < 4; i++)
            {
                // �����һ��ʱ�䣬������λƫ��
                float normalized_time = fmod(t + gait->T - gait->T * gait->phase[i], gait->T) / gait->T;
                // �ж��Ƿ���֧����
                if (normalized_time < gait->stance_ratio)
                {
                    // ֧����
                    contact[i] = 1;
                    phase[i] = normalized_time / gait->stance_ratio;
                }
                else
                {
                    // �ڶ���
                    contact[i] = 0;
                    phase[i] = (normalized_time - gait->stance_ratio) / (1 - gait->stance_ratio);
                }
            }
            break;
        }
        case SWING_ALL:  // ȫ�ڶ�ģʽ
            // �����ȶ����ڰڶ���
            for (int i = 0; i < 4; i++) 
            {
                contact[i] = 0;
                phase[i] = default_phase;
            }
            break;
        case STANCE_ALL:  // ȫ֧��ģʽ
            // �����ȶ�����֧����
            for (int i = 0; i < 4; i++) 
            {
                contact[i] = 1;
                phase[i] = default_phase;
            }
            break;
    }
}

void gait_generator(GaitParams *gait, float *phase, int *contact, float foot_target_pos[4][3])
{
    float stand_height = dog_get_stand_height();
    float stance_duration = gait->stance_ratio * gait->T;
    
    float foot_target_pos_thigh[4][3];
    // ��ȡ����������Ե�λ��
    float neutral_pos[4][3]; 
    float end_P[4][3];
    float body_vel[3];
    
    float x = 0.0f;  
    float y = 0.0f;  
    float z = 0.0f;  
    
    // ��ȡ��ǰ�ٶȺ����Ե�λ��
    dog_get_body_vel(body_vel);
    
    // ��ȡ���Ե�λ�� - �޸�Ϊ��Ե��Ȼ�ȡ
    for (int i = 0; i < 4; i++) {
        leg_get_neutral_pos(i, neutral_pos[i]);
    }

    for(int i = 0; i < 4; ++i){
        if(contact[i] == 1){ // ����֧����   
            // ����֧���������λ�ã�������������
            cal_foot_end_pos(i, body_vel, stance_duration, end_P[i]);   
            // ����֧�����е�ˮƽλ�ã���step_length/2��-step_length/2���Ա仯
            x = end_P[i][X_IDX] * (1 - (2 * phase[i]));
            y = end_P[i][Y_IDX] * (1 - (2 * phase[i]));
            // ֧�����������΢��ѹ���ṩ���õĵ���Ӵ�
            z = -gait->stance_depth * cosf(PI * (phase[i] - 0.5f));
        }
        else{ // ���ڰڶ��� 
            // �滮�����λ�� 
            cal_foot_end_pos(i, body_vel, stance_duration, end_P[i]);

            float fai = 2 * PI * phase[i];
            x = end_P[i][X_IDX] * (fai - sinf(fai)) / PI - end_P[i][X_IDX];
            y = end_P[i][Y_IDX] * (fai - sinf(fai)) / PI - end_P[i][Y_IDX];
            z = gait->swing_height * (1 - cosf(fai)) / 2.0f;
        }
        // xyת����thigh����ϵ��
        // ���������Ĳ���
        //if (i == LEG_HL || i == LEG_HR)
        //{
        //    foot_target_pos_thigh[i].z -= 0.00f;
        //}
        foot_target_pos_thigh[i][X_IDX] = x + neutral_pos[i][X_IDX];
        foot_target_pos_thigh[i][Y_IDX] = y + neutral_pos[i][Y_IDX];
        foot_target_pos_thigh[i][Z_IDX] = z + neutral_pos[i][Z_IDX];

        // xyzת����hip����ϵ��
        leg_thigh_to_hip(i, foot_target_pos_thigh[i], foot_target_pos[i]);
    }
}


