/**
 * @file dog.h
 * @brief ��������˶���ͽӿ�����
 */

#ifndef _DOG_H
#define _DOG_H

/*========================= ͷ�ļ����� =========================*/
#include "stdint.h"
#include "robot_params.h"

// ��������
#define X_IDX 0
#define Y_IDX 1
#define Z_IDX 2

// �ؽ�����
#define HIP_IDX   0
#define THIGH_IDX 1
#define CALF_IDX  2

// �Ȳ�����
#define FL_IDX 0
#define FR_IDX 1
#define HL_IDX 2
#define HR_IDX 3


/*========================= �������Ͷ��� =========================*/
/**
 * @brief �����Ƚṹ��
 */
typedef struct
{
    uint8_t leg_id;              // �ȵ�ID (0:FL, 1:FR, 2:HL, 3:HR)
    float join_target_pos[3];    // Ŀ��ؽڽǶ� [hip, thigh, calf]
    float motor_target_pos[3];   // Ŀ�����Ƕ� [hip, thigh, calf]
    float foot_target_pos[3];    // ���Ŀ��λ�� [x, y, z]
    float foot_neutral_pos[3];   // ������Ե�λ�� [x, y, z]
} Leg;

/**
 * @brief ��������˽ṹ��
 */
typedef struct
{
    Leg leg[4];          // ������ [FL, FR, HL, HR]
    RobotParams params;  // �����˲�������
} Dog;

/*========================= �˶�ѧ�������� =========================*/
/**
 * @brief ��ξ������˶�ѧ���⣬����ӹؽڽǶȵ����λ�õ�ӳ��
 * @param leg_id �ȵ�ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param joint_pos ����ĹؽڽǶ����� [hip, thigh, calf]
 * @param foot_pos ��������λ��
 */
void leg_forward_kinematics(uint8_t leg_id, const float joint_pos[3], float foot_pos[3]);

/**
 * @brief ���η����˶�ѧ��⣬��������λ�õ��ؽڽǶȵ�ӳ��
 * @param leg_id �ȵ�ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param foot_pos ��������λ��
 * @param joint_pos ����ĹؽڽǶ����� [hip, thigh, calf]
 */
void leg_inverse_kinematics(uint8_t leg_id, const float foot_pos[3], float joint_pos[3]);

/*========================= ����ϵת���������� =========================*/
/**
 * @brief ���ȹؽ�����ϵת�������ϵ
 * @param leg_id �ȵ�ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param joint_pos �ؽڽǶ����� [3]
 * @param motor_pos ����Ƕ����� [3]
 */
void leg_joint_to_motor(uint8_t leg_id, const float *joint_pos, float *motor_pos);

/**
 * @brief ���ȵ������ϵת�ؽ�����ϵ
 * @param leg_id �ȵ�ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param motor_pos ����Ƕ����� [3]
 * @param joint_pos �ؽڽǶ����� [3]
 */
void leg_motor_to_joint(uint8_t leg_id, const float *motor_pos, float *joint_pos);

/**
 * @brief ��������ϵת�����Źؽ�����ϵ
 * @param leg_id �ȵ�ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param thigh_pos ��������ϵλ��
 * @param hip_pos �Źؽ�����ϵλ��
 */
void leg_thigh_to_hip(uint8_t leg_id, const float thigh_pos[3], float hip_pos[3]);

/**
 * @brief ���λ��ֱ��ת��Ϊ����Ƕ�
 * @param leg_id �ȵ�ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param foot_pos ���λ��
 * @param motor_pos ����ĵ���Ƕ����� [3]
 */
void leg_foot_to_motor(uint8_t leg_id, const float foot_pos[3], float motor_pos[3]);

/*========================= �����˳�ʼ������ =========================*/
/**
 * @brief ��ʼ��������������״̬
 * @param init_params ��ʼ��������NULLʱʹ��Ĭ�ϲ���
 */
void dog_init(const RobotParams* init_params);

/**
 * @brief ���»���������
 */
void dog_data_update(void);

/*========================= set�������� =========================*/
/**
 * @brief ����ָ���ȵĵ��Ŀ��λ��
 * @param leg_id �ȵ�ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param motor_pos ���Ŀ��λ������ [3]
 */
void leg_set_motor_pos(uint8_t leg_id, const float motor_pos[3]);

/**
 * @brief ����ָ���ȵĹؽ�Ŀ��λ��
 * @param leg_id �ȵ�ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param joint_pos �ؽ�Ŀ��λ������ [3]
 */
void leg_set_joint_pos(uint8_t leg_id, const float joint_pos[3]);

/**
 * @brief ����ָ���ȵ����Ŀ��λ��
 * @param leg_id �ȵ�ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param foot_pos ���Ŀ��λ������ [3]
 */
void leg_set_target_foot_pos(uint8_t leg_id, const float foot_pos[3]);

/**
 * @brief ���û����ٶ�
 * @param vx ����ǰ���ٶ�
 * @param vy ��������ٶ�
 * @param w ������ת�ٶ�
 */
void dog_set_body_vel(float vx, float vy, float w);

/**
 * @brief ����վ���߶�
 * @param height վ���߶�
 */
void dog_set_stand_height(float height);

/*========================= get�������� =========================*/
/**
 * @brief ��ȡָ���ȵĵ�ǰ�ؽ�λ��
 * @param leg_idx �ȵ�ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param joint_current_pos �ؽڵ�ǰλ������ [3]
 */
void leg_get_current_joints(uint8_t leg_idx, float joint_current_pos[3]);

/**
 * @brief ��ȡָ���ȵĵ�ǰ���λ��
 * @param leg_idx �ȵ�ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param foot_pos ���λ������ [3]
 */
void leg_get_current_foot_pos(uint8_t leg_idx, float foot_pos[3]);

/**
 * @brief ��ȡָ���ȵ����Ե�λ��
 * @param leg_idx �ȵ�ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param neutral_pos ���Ե�λ������ [3]
 */
void leg_get_neutral_pos(uint8_t leg_idx, float neutral_pos[3]);

/**
 * @brief ��ȡָ���ȵĹؽ�λ��
 * @param leg_idx �ȵ�ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param joint_pos �ؽ�λ������ [3]
 */
void leg_get_joint_pos(uint8_t leg_idx, float joint_pos[3]);

/**
 * @brief ��ȡָ���ȵ����Ŀ��λ��
 * @param leg_idx �ȵ�ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param foot_pos ���Ŀ��λ������ [3]
 */
void leg_get_target_foot_pos(uint8_t leg_idx, float foot_pos[3]);

/**
 * @brief Al��ȡ�����ٶ�
 * @param velocity �����ٶ����� [3]
 */
void dog_get_body_vel(float velocity[3]);

/**
 * @brief ��ȡվ���߶�
 * @return float վ���߶�
 */
float dog_get_stand_height(void);

/*========================= send�������� =========================*/
/**
 * @brief ���͵��Ŀ��λ��
 */
void dog_send_motors(void);

/**
 * @brief �ѹؽڽǶ�ת��Ϊ����ǶȲ�����
 */
void dog_send_motors_from_joints(void);

/*========================= ���ƺ������� =========================*/
/**
 * @brief �����˿�����������ִ���˶�����̬�Ͳ�̬����
 */
void robot_control(void);

/*========================= ���Ŀ��ƺ������� =========================*/
/**
 * @brief ͨ������������Ե�λ�������ƻ���������
 * 
 * @param cog_translation ƽ������ƫ���� [3]
 * @param cog_rotation ��ת����ƫ���� [3]
 * @param foot_points �����������Ե�λ������ [4][3]
 */
void dog_cog_to_foot(const float cog_translation[3], const float cog_rotation[3], float foot_points[4][3]);

/**
 * @brief ƽ�����ɵ�ָ������λ��
 * 
 * @param center_of_gravity ����������ƫ���� [3]
 * @param foot_now ��ǰ���λ�� [4][3]
 * @param step_increment ��������
 * @param foot_next ��һ�����λ�� [4][3]
 */
void dog_smooth_cog(const float center_of_gravity[3], const float foot_now[4][3], float step_increment, float foot_next[4][3]);

float smooth(float current, float target, float step_increment);

#endif /* _DOG_H */
