#include "gait.h"
#include "timer.h"
#include "stdint.h"
#include "dog.h"
#include "robot_params.h"
#include "stdio.h"
#include "estimator.h"
#include "ANO_TC.h"
#include "force_calculate.h"

static void limit(float *value, float min, float max)
{
    if (*value < min) *value = min;
    if (*value > max) *value = max;
}

// ��ȡ��̬����
GaitParams* get_trot_params() {
    return (GaitParams*)&get_dog_params()->trot_gait;
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
float vbx = 0;
float vby = 0;
float x_adjust = 0;
float y_adjust = 0;
float cal_foot_end_pos(int leg_id, GaitParams *gait, const float body_vel[3], float end_pos[3])
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
    // vx += VW_SIGNS[leg_id][0] * vw * sinf(HIP_ANGLE);
    // vy += VW_SIGNS[leg_id][1] * vw * cosf(HIP_ANGLE);

    // ʹ��Raibert����ʽ�������ٶ�ת��Ϊ����
    // ���� = (֧����ʱ��/2) * �ٶ�

    vbx = est_get_body_vel(0);
    vby = est_get_body_vel(1);
    float phase = get_dog_params()->posture.phase[leg_id];
    float Tswing = gait->T * gait->swing_ratio;
    float Tstance = gait->T * gait->stance_ratio;
    float kx = gait->kx;
    float ky = gait->ky;
    // tudo kx*(vbx - vx)�ӷ�Χ,����kx
    x_adjust = kx*(vbx - vx);
    y_adjust = ky*(vby - vy);
    limit(&x_adjust, -0.04f, 0.04f);
    limit(&y_adjust, -0.08f, 0.08f);
    
//    end_pos[X_IDX] = vbx*(1-phase)*Tswing + vbx*Tstance/2.0f + x_adjust;
    end_pos[X_IDX] = vbx*(1-phase)*Tswing + (vbx + VW_SIGNS[leg_id][0] * vw * sinf(HIP_ANGLE))*Tstance/2.0f + x_adjust;
//    end_pos[X_IDX] = (vx + VW_SIGNS[leg_id][0] * vw * sinf(HIP_ANGLE))*Tstance/2.0f + x_adjust;
    // end_pos[X_IDX] = (vx + VW_SIGNS[leg_id][0] * vw * sinf(HIP_ANGLE))*Tstance/2.0f;
    end_pos[Y_IDX] = (vy + VW_SIGNS[leg_id][1] * vw * cosf(HIP_ANGLE))*Tstance/2.0f;
//    end_pos[Y_IDX] = (vy + VW_SIGNS[leg_id][1] * vw * cosf(HIP_ANGLE))*Tstance/2.0f + y_adjust;
    // end_pos[Y_IDX] = (vy + VW_SIGNS[leg_id][1] * vw * cosf(HIP_ANGLE))*Tstance/2.0f;
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
                // ʱ�������ж�
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
    for(int i = 0; i < 4; ++i){
        // ʵʱ���»����˵Ĳ�̬״̬
        get_dog_params()->posture.contact[i] = contact[i];
        get_dog_params()->posture.phase[i] = phase[i];
    }
}

uint8_t swing_leg[4] = {1, 0, 0, 1}; // ��ǰ�ڶ���
uint8_t all_legs_contact = 1;
/**
 * @brief ���ɲ�̬��λ�ͽӴ�״̬
 * 
 * @param gait ��̬����
 * @param status ��̬״̬��������̬��ȫ�ڶ���ȫ֧��
 * @param phase ���������������λֵ����[4]����Χ0~1
 * @param contact ������������ȽӴ�״̬����[4]��1=֧���࣬0=�ڶ���
 */
void phase_wave_generator_with_force(GaitParams *gait, WaveStatus status, float start_time, float *phase, int *contact)
{
    static uint8_t all_contact_flag = 0;  // ��ֹ��������
    static float last_cycle_time = 0;     // ��һ�����ڵĿ�ʼʱ��
    
    
    // ȫ���ȵ�Ĭ����λֵ
    const float default_phase = 0.5f;
    
    // ���ݲ�̬״̬������Ӧ����λ�ͽӴ�״̬
    switch(status)
    {
        case WAVE_ALL:  // ������̬ģʽ
        {
            // ����ӿ�ʼʱ�䵽���ڵ�ʱ�䣨�룩
            float t = getTime() - last_cycle_time - start_time;
            if (t < 0)
                t = 0;
            // ����Ƿ�ȫ������
            all_legs_contact = 1;
            for (int i = 0; i < 4; i++)
            {
                if (contact_judge_with_force(i) == 1)
                {
                    contact[i] = 1;
                }
                else
                {
                    contact[i] = 0;
                    all_legs_contact = 0;
                }
            }
            // ���ȫ�����أ�������һ���Խ��ߵ���ֻ��
            if (all_legs_contact && !all_contact_flag)
            {
                all_contact_flag = 1;
                last_cycle_time = getTime() - start_time;
                t = 0;
                for(int i = 0; i < 4; ++i){
                    swing_leg[i] = !swing_leg[i];
                }
            }
            else if (!all_legs_contact)
            {
                all_contact_flag = 0;
            }
            
            // ����ÿ���ȵ���λ
            for (int i = 0; i < 4; i++)
            {
                // float normalized_time = fmod(t + gait->T - gait->T * gait->phase[i], gait->T) / gait->T;
                if (contact[i] == 1)
                {
                    phase[i] = t / (gait->stance_ratio * gait->T);
                }
                else
                {
                    phase[i] = t / ((1 - gait->stance_ratio) * gait->T);
                }
                if (phase[i] < 0)
                    phase[i] = 0;
                if (phase[i] > 1)
                    phase[i] = 1;
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
    
    for(int i = 0; i < 4; ++i){
        // ʵʱ���»����˵Ĳ�̬״̬
        get_dog_params()->posture.contact[i] = contact[i];
        get_dog_params()->posture.phase[i] = phase[i];
    }
}

float p0[4][3] = {0};
float pf[4][3] = {0};
float ratio = 1; // ���֧���໬������Сratio
uint8_t prev_contact[4] = {1, 1, 1, 1};
uint8_t first_run[4] = {1, 1, 1, 1};
void gait_generator(GaitParams *gait, float *phase, int *contact, float foot_target_pos[4][3], float foot_target_vel[4][3])
{
    float foot_target_pos_thigh[4][3];
    // ��ȡ����������Ե�λ��
    float neutral_pos[4][3]; 
    float body_vel[3];
    
    // static float p0[4][3] = {0};
    float h = gait->swing_height;
    float T = gait->T;

    float x = 0.0f;  
    float y = 0.0f;  
    float z = 0.0f;  

    float vx = 0.0f;
    float vy = 0.0f;
    float vz = 0.0f;
    
    float VW_SIGNS[4][2] = {
        {-1.0f, +1.0f},  // FL: x��, y��
        {+1.0f, +1.0f},  // FR: x��, y��
        {-1.0f, -1.0f},  // HL: x��, y��
        {+1.0f, -1.0f}   // HR: x��, y��
    };
    
    // ��ȡ��ǰ�ٶȺ����Ե�λ��
    dog_get_body_vel(body_vel);

    // �����ٶȵ�����Ƶ ֱ���޸Ļᵼ����˶������ȶ�
    // adjust_gait_frequency();
    
    // ��ȡ���Ե�λ�� - �޸�Ϊ��Ե��Ȼ�ȡ
    for (int i = 0; i < 4; i++) {
        leg_get_neutral_current_pos(i, neutral_pos[i]);
    }

    for(int i = 0; i < 4; ++i){
        if(contact[i] == 1){ // ����֧����   
            // ֧����Ӧ����������������ϵ������޻���
            // �������֧����Ŀ�ʼ�����������ʼλ��
            if (prev_contact[i] == 0 || first_run[i] == 1) // �Ӱڶ�����ȵ�֧����
            {
                // p0Ϊ�ڶ���Ŀ��λ��
                leg_get_target_foot_pos(i, p0[i]);
                // leg_get_current_foot_pos(i, p0[i]);
                prev_contact[i] = 1;
                first_run[i] = 0;
            }
            
            x = p0[i][X_IDX] - (body_vel[X_IDX] + VW_SIGNS[i][0] * body_vel[2] * sinf(HIP_ANGLE)) * T * gait->stance_ratio * phase[i] * ratio;
            y = p0[i][Y_IDX] - (body_vel[Y_IDX] + VW_SIGNS[i][1] * body_vel[2] * cosf(HIP_ANGLE)) * T * gait->stance_ratio * phase[i] * ratio;
            z = 0;

            vx = 0;
            vy = 0;
            vz = 0;
        }
        else{ // ���ڰڶ��� 
            // �滮�����λ�� 
            cal_foot_end_pos(i, gait, body_vel, pf[i]);
            leg_thigh_to_hip(i, pf[i], pf[i]);
            // ������ڰڶ���Ŀ�ʼ�����������ʼλ��
            if (prev_contact[i] == 1 || first_run[i] == 1) // ��֧������ȵ��ڶ���
            {
                leg_get_current_foot_pos(i, p0[i]);
                prev_contact[i] = 0;
                first_run[i] = 0;
            }
            // ���߹켣
            float fai = 2 * MY_PI * phase[i];
            x = (pf[i][X_IDX] - p0[i][X_IDX]) * (fai - sinf(fai)) / (2*MY_PI) + p0[i][X_IDX];
            y = (pf[i][Y_IDX] - p0[i][Y_IDX]) * (fai - sinf(fai)) / (2*MY_PI) + p0[i][Y_IDX];
            z = h * (1 - cosf(fai)) / 2.0f;
            // �����ٶ�
           vx = (pf[i][X_IDX] - p0[i][X_IDX]) / T * (1 - cosf(fai));
           vy = (pf[i][Y_IDX] - p0[i][Y_IDX]) / T * (1 - cosf(fai));
           vz = MY_PI * h / T * sinf(fai);
//            vx = 0;
//            vy = 0;
//            vz = 0;
        }
        // xyת����thigh����ϵ��
        // ���������Ĳ���
        //if (i == LEG_HL || i == LEG_HR)
        //{
        //    foot_target_pos_thigh[i].z -= 0.00f;
        //}
        foot_target_pos[i][X_IDX] = x + neutral_pos[i][X_IDX];
        foot_target_pos[i][Y_IDX] = y + neutral_pos[i][Y_IDX];
        foot_target_pos[i][Z_IDX] = z + neutral_pos[i][Z_IDX];

        foot_target_vel[i][X_IDX] = vx;
        foot_target_vel[i][Y_IDX] = vy;
        foot_target_vel[i][Z_IDX] = vz;
    }
}

float max_T = 0.5; 
float min_T = 0.38;
float max_vel = 0.5;
float T_inc = 0.0001;
//�����ٶȵ�����Ƶ
void adjust_gait_frequency()
{
    // ����Ŀ�경Ƶ
    float target_body_vel[3] = {0};
    dog_get_body_vel(target_body_vel);
    // �ٶ�Խ�󣬲�ƵԽ��
    float current_velocity = sqrt(target_body_vel[X_IDX] * target_body_vel[X_IDX] + target_body_vel[Y_IDX] * target_body_vel[Y_IDX]);
    
    float velocity_factor = current_velocity / max_vel;
    if (velocity_factor > 1)
        velocity_factor = 1;

    float target_T = max_T - velocity_factor * (max_T - min_T);
    float now_T = get_dog_params()->trot_gait.T;
    float next_T = smooth(now_T, target_T, T_inc);

    get_dog_params()->trot_gait.T = next_T;
}
