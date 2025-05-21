#include "motor.h"
#include "robot_params.h"
#include "stdio.h"
#include "j60.h"
#include "dog.h"
// ��ʼ������ش�

void motor_init()
{
    
    J60_Can_Init();
    
    // FL
    J60_ConfigMotor(LEG_FL*3 + JOINT_HIP, MOTOR_FL_HIP_CAN_BUS, MOTOR_FL_HIP_ID);   
    J60_ConfigMotor(LEG_FL*3 + JOINT_THIGH, MOTOR_FL_THIGH_CAN_BUS, MOTOR_FL_THIGH_ID);   
    J60_ConfigMotor(LEG_FL*3 + JOINT_CALF, MOTOR_FL_CALF_CAN_BUS, MOTOR_FL_CALF_ID); 

    // FR
    J60_ConfigMotor(LEG_FR*3 + JOINT_HIP, MOTOR_FR_HIP_CAN_BUS, MOTOR_FR_HIP_ID);   
    J60_ConfigMotor(LEG_FR*3 + JOINT_THIGH, MOTOR_FR_THIGH_CAN_BUS, MOTOR_FR_THIGH_ID);   
    J60_ConfigMotor(LEG_FR*3 + JOINT_CALF, MOTOR_FR_CALF_CAN_BUS, MOTOR_FR_CALF_ID);  
    
    // HL
    J60_ConfigMotor(LEG_HL*3 + JOINT_HIP, MOTOR_HL_HIP_CAN_BUS, MOTOR_HL_HIP_ID);  
    J60_ConfigMotor(LEG_HL*3 + JOINT_THIGH, MOTOR_HL_THIGH_CAN_BUS, MOTOR_HL_THIGH_ID);  
    J60_ConfigMotor(LEG_HL*3 + JOINT_CALF, MOTOR_HL_CALF_CAN_BUS, MOTOR_HL_CALF_ID); 

    // HR
    J60_ConfigMotor(LEG_HR*3 + JOINT_HIP, MOTOR_HR_HIP_CAN_BUS, MOTOR_HR_HIP_ID);  
    J60_ConfigMotor(LEG_HR*3 + JOINT_THIGH, MOTOR_HR_THIGH_CAN_BUS, MOTOR_HR_THIGH_ID);  
    J60_ConfigMotor(LEG_HR*3 + JOINT_CALF, MOTOR_HR_CALF_CAN_BUS, MOTOR_HR_CALF_ID);  


   for (uint8_t i = 0; i < 12; i++) { // ��ʼ��ʹ��״̬��1ʹ��ʧ�ܣ�0ʹ�ܳɹ�
       J60_GetMotor(i)->is_enable = 1;
   }

    while(J60_GetMotor(0)->is_enable == 1 || J60_GetMotor(1)->is_enable == 1 || J60_GetMotor(2)->is_enable == 1 || J60_GetMotor(3)->is_enable == 1 || \
    J60_GetMotor(4)->is_enable == 1 || J60_GetMotor(5)->is_enable == 1 || J60_GetMotor(6)->is_enable == 1 || J60_GetMotor(7)->is_enable == 1 || \
    J60_GetMotor(8)->is_enable == 1 || J60_GetMotor(9)->is_enable == 1 || J60_GetMotor(10)->is_enable == 1 || J60_GetMotor(11)->is_enable == 1)
    {
        for (int i = 0; i < 12; i++) 
        {
            if (J60_GetMotor(i)->is_enable == 1)
            {
                J60_EnableMotor(i);
                HAL_Delay(10);
//                printf("δ���յ����[%d]�ش�����\n", i);
            }
        }
    }
    // ��յ�����Ʋ�����ֻ���յ���ش�����
    for (int i = 0; i < 12; i++) {
        J60_GetMotor(i)->kp = 0;
        J60_GetMotor(i)->kd = 0;
    }
    // printf("����ش���ʼ���ɹ�\n");
}

/**
 * @brief ��鲢���Ƶ���Ƕ��ڰ�ȫ��Χ��
 * 
 * @param motor_target Ŀ�����Ƕ����� [12] (4���� x 3���ؽ�)
 */
void check_motor_limit(float *motor_target_pos)
{
    // ��ȡ�����˲���
    const RobotParams* params = get_dog_params();
    
    // �ؽ��������ƣ����ھ�����Ϣ
    const char* joint_names[] = {"hip", "thigh", "calf"};
    
    // ����������
    for (int leg_idx = 0; leg_idx < 4; leg_idx++) 
    {
        // ��ȡ��ǰ�ȵĵ�����Ʋ���
        const LegAngleLimits* leg_limits = &params->motor_param.motor_limits[leg_idx];
        
        // ����ÿ���ȵ�3���ؽ�
        for (int joint_idx = 0; joint_idx < 3; joint_idx++) 
        {
            // ���㵱ǰ�ؽ��������е�����
            int motor_idx = leg_idx * 3 + joint_idx;
            float angle = motor_target_pos[motor_idx];
            
            // ���ݹؽ����ͻ�ȡ��Ӧ�ĽǶ�����
            float max_angle = 0.0f;
            float min_angle = 0.0f;
            
            switch (joint_idx) 
            {
                case JOINT_HIP:
                    max_angle = leg_limits->max_hip_angle;
                    min_angle = leg_limits->min_hip_angle;
                    break;
                case JOINT_THIGH:
                    max_angle = leg_limits->max_thigh_angle;
                    min_angle = leg_limits->min_thigh_angle;
                    break;
                case JOINT_CALF:
                    max_angle = leg_limits->max_calf_angle;
                    min_angle = leg_limits->min_calf_angle;
                    break;
            }
            
            // �������
            if (angle > max_angle) 
            {
//                printf("���棺��%d��%s�ؽڽǶȳ�������! ��ǰֵ��%.3f�����ޣ�%.3f\n", 
//                       leg_idx, joint_names[joint_idx], angle, max_angle);
                motor_target_pos[motor_idx] = max_angle;
            }
            
            // �������
            if (angle < min_angle) 
            {
//                printf("���棺��%d��%s�ؽڽǶȳ�������! ��ǰֵ��%.3f�����ޣ�%.3f\n", 
//                       leg_idx, joint_names[joint_idx], angle, min_angle);
                motor_target_pos[motor_idx] = min_angle;
            }
        }
    }
}

// ���͵��Ŀ��λ��
void send_motors_target_pos(float *motors_target_pos)
{
//    check_motor_limit(motors_target_pos);  // �ȼ����λ
    
    for (int i = 0; i < 12; i++) {
        J60_MotorControl(i, motors_target_pos[i], 0, J60_GetMotor(i)->kp, J60_GetMotor(i)->kd, 0);
    } 
}   


// ��ȡ�����ǰλ��
void leg_get_motors_current_pos(uint8_t leg_idx, float motors_current_pos[3])
{
    for (int i = 0; i < 3; i++) {
        motors_current_pos[i] = J60_GetMotor(leg_idx*3+i)->position;
    }
}

// ��ȡ�����ǰ�ٶ�
void leg_get_motors_current_vel(uint8_t leg_idx, float motors_current_vel[3])
{
    for (int i = 0; i < 3; i++) {
        motors_current_vel[i] = J60_GetMotor(leg_idx*3+i)->velocity;
    }
}

void leg_set_motor_kp_kd(uint8_t leg_idx, float kp_hip, float kd_hip, float kp_thigh, float kd_thigh, float kp_calf, float kd_calf)
{
    J60_GetMotor(leg_idx*3+JOINT_HIP)->kp = kp_hip;
    J60_GetMotor(leg_idx*3+JOINT_HIP)->kd = kd_hip;
    J60_GetMotor(leg_idx*3+JOINT_THIGH)->kp = kp_thigh;
    J60_GetMotor(leg_idx*3+JOINT_THIGH)->kd = kd_thigh;
    J60_GetMotor(leg_idx*3+JOINT_CALF)->kp = kp_calf;
    J60_GetMotor(leg_idx*3+JOINT_CALF)->kd = kd_calf;
}
