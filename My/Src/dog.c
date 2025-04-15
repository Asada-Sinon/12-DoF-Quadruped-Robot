/**
 * @file dog.c
 * @brief ����������˶�ѧ�Ϳ���ʵ��
 */

/*========================= ͷ�ļ����� =========================*/
/* ��׼�� */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

/* �Զ���ͷ�ļ� */
#include "dog.h"
#include "motor.h"
#include "gait.h"

/*========================= ȫ�ֱ��� =========================*/
Dog dog;                    // ��ʵ��
float body_vel[3] = {0, 0, 0}; // �����ٶ�

/*========================= ���Ͷ��� =========================*/
// ����ϵת������ö��
typedef enum {
    JOINT_TO_MOTOR = 0,  // �ؽ�����ϵת��Ϊ�������ϵ
    MOTOR_TO_JOINT = 1   // �������ϵת��Ϊ�ؽ�����ϵ
} TransformDirection;


/*========================= �˶�ѧ���� =========================*/
/* https://github.com/unitreerobotics/unitree_guide.git 
 * ������ϵ���壨����ϵ��:
 *
 *            �� z
 *            |
 *            |
 *  x ����������������|
 *            /
 *           /
 *          /
 *         y
 *  
 * ע��ԭ���ڴ��ȹؽ��������Źؽ����ߵĽ����ϣ������ڻ����ϡ�x����ͷ����z�����ϣ������ֶ���ȷ��y�᷽��
 * ע�������FL��HL ��y����������ࣻ�Ҳ���FR��HR��y���������ڲࡣ
 */

void leg_forward_kinematics(uint8_t leg_id, const float joint_pos[3], float foot_pos[3])
{
    const LegLinkParams *leg_links = &dog.params.leg_links;
    int sign = 1;
    if (leg_id == LEG_FR || leg_id == LEG_HR) {
        sign = -1;
    }
    
    // ת�����˳��ȣ���Ӧ����ϵ����
    float l1 = sign * leg_links->hip_length;
    float l2 = -leg_links->thigh_length;
    float l3 = -leg_links->calf_length;

    // �������Ǻ���ֵ
    float s1 = sinf(joint_pos[HIP_IDX]);
    float s2 = sinf(joint_pos[THIGH_IDX]);
    float s3 = sinf(joint_pos[CALF_IDX]);

    float c1 = cosf(joint_pos[HIP_IDX]);
    float c2 = cosf(joint_pos[THIGH_IDX]);
    float c3 = cosf(joint_pos[CALF_IDX]);

    // ���㸴�Ͻǵ����Ǻ���ֵ
    float c23 = c2 * c3 - s2 * s3;
    float s23 = s2 * c3 + c2 * s3;

    // Ӧ���˶�ѧ���ⷽ��
    foot_pos[X_IDX] = l3 * s23 + l2 * s2;
    foot_pos[Y_IDX] = -l3 * s1 * c23 + l1 * c1 - l2 * c2 * s1;
    foot_pos[Z_IDX] =  l3 * c1 * c23 + l1 * s1 + l2 * c1 * c2;

    printf("foot_pos[%d]: %f %f %f\n", leg_id, foot_pos[X_IDX], foot_pos[Y_IDX], foot_pos[Z_IDX]);

}

void leg_inverse_kinematics(uint8_t leg_id, const float foot_pos[3], float joint_pos[3])
{
    const LegLinkParams *leg_links = &dog.params.leg_links;
    
    // ȷ���Źؽ�Y������ţ����Ⱥ����ȷ����෴��
    int y_sign = (leg_id == LEG_FL || leg_id == LEG_HL) ? 1 : -1;
    
    // ��ȡ���λ������
    float px = foot_pos[X_IDX];
    float py = foot_pos[Y_IDX];
    float pz = foot_pos[Z_IDX];
    
    // ת�����˳��ȣ���Ӧ����ϵ����
    float l1 = y_sign * leg_links->hip_length;  // �Źؽ�ƫ�ƣ��������ҷ���
    float l2 = -leg_links->thigh_length;        // ���ȳ���
    float l3 = -leg_links->calf_length;         // С�ȳ���

    /*�����ŹؽڽǶ�*/
    // �������˼�С����������������ϵ��yzƽ���ͶӰ����
    float L1 = sqrtf(py*py + pz*pz - l1*l1); 
    // �ŹؽڽǶ� - ʹ��atan2ȷ����ȷ������
    float hip_angle = atan2f(pz*l1+py*L1, py*l1-pz*L1);
    
    /*����С�ȹؽڽǶ�*/
    // �����������ȹؽڵľ���
    float L2 = sqrtf(px*px + py*py + pz*pz - l1*l1);  
    // Ӧ�����Ҷ���ȷ��������arccos�Ķ�����[-1,1]��
    float temp = (l2*l2 + l3*l3 - L2*L2)/(2*fabs(l2*l3));
    if(temp>1) temp = 1;
    if(temp<-1) temp = -1;
    // С�ȹؽڽǶ� - ���ȼ���Ϊ[0,��]��Χ��Ȼ��ת�����ؽ�Լ����Χ[-��,0]
    float calf_angle = acosf(temp); //[0, PI]
    calf_angle = -(PI - calf_angle); //[-PI, 0]

    /*������ȹؽڽǶ�*/
    // ͨ�����ι�ϵ�������
    float a1 = py*sin(hip_angle) - pz*cos(hip_angle);
    float a2 = px;
    float m1 = l3*sin(calf_angle);
    float m2 = l2 + l3*cos(calf_angle);
    // ���ȹؽڽǶ� - ʹ��atan2ȷ����ȷ������
    float thigh_angle = atan2f(m1*a1+m2*a2, m1*a2-m2*a1);
    
    // �洢������
    joint_pos[HIP_IDX] = hip_angle;
    joint_pos[THIGH_IDX] = thigh_angle;
    joint_pos[CALF_IDX] = calf_angle;
}

/*========================= ����ϵת������ =========================*/

/**
 * @brief ���ȹؽ�����ϵ��������ϵ����ת��
 * 
 * @param leg_id �ȵ�ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param input ����Ƕ����� [3]
 * @param output ����Ƕ����� [3]
 * @param direction ת������ (JOINT_TO_MOTOR��MOTOR_TO_JOINT)
 */
void leg_transform(uint8_t leg_id, const float *input, float *output, TransformDirection direction)
{
    const MotorJointDirConfig *motor_dir = &dog.params.motor_param.motor_dir[leg_id];
    const MotorZeroPos *motor_zero_pos = &dog.params.motor_param.motor_zero_pos[leg_id];
    for (int joint_idx = 0; joint_idx < 3; joint_idx++) {
        int8_t dir = 1;
        float zero_pos = 0;
        // ���ݹؽ�����ѡ���Ӧ�ķ�������
        switch (joint_idx) {
            case JOINT_HIP:
                dir = motor_dir->hip_dir;
                zero_pos = motor_zero_pos->hip_zero_pos;
                break;
            case JOINT_THIGH:
                dir = motor_dir->thigh_dir;
                zero_pos = motor_zero_pos->thigh_zero_pos;
                break;
            case JOINT_CALF:
                dir = motor_dir->calf_dir;
                zero_pos = motor_zero_pos->calf_zero_pos;
                break;
        }
        // ����ת������Ӧ�ñ任
        if (direction == JOINT_TO_MOTOR) {
            output[joint_idx] = dir * input[joint_idx] + zero_pos;  // �ؽ���ת�ĽǶ���������ת������ϵ����λ�Ƕ�
        } else {  // MOTOR_TO_JOINT
            output[joint_idx] = dir * (input[joint_idx] - zero_pos);  // �����ת�ĽǶ���������ת������Ϲؽ���λ�Ƕȣ�0�ȣ�
        }
    }
}

void leg_joint_to_motor(uint8_t leg_id, const float *joint_pos, float *motor_pos)
{
    leg_transform(leg_id, joint_pos, motor_pos, JOINT_TO_MOTOR);
}

void leg_motor_to_joint(uint8_t leg_id, const float *motor_pos, float *joint_pos)
{
    leg_transform(leg_id, motor_pos, joint_pos, MOTOR_TO_JOINT);
}

void leg_thigh_to_hip(uint8_t leg_id, const float thigh_pos[3], float hip_pos[3])
{
    const LegLinkParams *leg_links = &dog.params.leg_links;
    if (leg_id == LEG_FL || leg_id == LEG_HL) {
        hip_pos[Y_IDX] = thigh_pos[Y_IDX] + leg_links->hip_length;
    } else {
        hip_pos[Y_IDX] = thigh_pos[Y_IDX] - leg_links->hip_length;
    }
    hip_pos[X_IDX] = thigh_pos[X_IDX];
    hip_pos[Z_IDX] = thigh_pos[Z_IDX];
}

void leg_foot_to_motor(uint8_t leg_id, const float foot_pos[3], float motor_pos[3])
{
    float joint_pos[3];
    leg_inverse_kinematics(leg_id, foot_pos, joint_pos);
    leg_joint_to_motor(leg_id, joint_pos, motor_pos);
}

/*========================= �����˿��ƺ��� =========================*/


/*========================= ���Ŀ��ƺ��� =========================*/

/**
 * @brief ͨ������������Ե�λ�������ƻ���������
 * 
 * @param center_of_gravity ����ƫ�������������������ϵ��ͬ
 * @param foot_neutral_points �Źؽ�����ϵ�£������������Ե�λ������
 */
void dog_cog_to_foot(const float cog_translation[3], const float cog_rotation[3], float foot_points[4][3])
{
    float stand_height = dog.params.posture.stand_height;
    CenterOfGravityLimits *cog_limit = &dog.params.posture.center_of_gravity_limit;
    // ��������ƫ��������������Ե�λ��(cog_xyz�ڵ�������ϵ:�������������ϵ��ͬ��������ԭ��Ϊ���������ڵ����ͶӰ)
    float cog_x = cog_translation[X_IDX];
    float cog_y = cog_translation[Y_IDX];
    float cog_z = cog_translation[Z_IDX];
    // ��ȫ���ƣ���ֹ����ƫ�ƹ����²��ȶ�
    float cog_x_max = cog_limit->cog_x_max;
    float cog_x_min = cog_limit->cog_x_min;
    float cog_y_max = cog_limit->cog_y_max;
    float cog_y_min = cog_limit->cog_y_min;
    float cog_z_max = cog_limit->cog_z_max;
    float cog_z_min = cog_limit->cog_z_min;

    // �޷�
    if (cog_x >  cog_x_max) cog_x =  cog_x_max;
    if (cog_x <  cog_x_min) cog_x =  cog_x_min;
    if (cog_y >  cog_y_max) cog_y =  cog_y_max;
    if (cog_y <  cog_y_min) cog_y =  cog_y_min;
    if (cog_z >  cog_z_max) cog_z =  cog_z_max;
    if (cog_z <  cog_z_min) cog_z =  cog_z_min;
    // ��������ƫ�Ƶ�������������Ե�(�Źؽ�����ϵ)
    for (int i = 0; i < 4; i++) {
        // ���򲹳�����ƫ�ƣ����������ƶ�����ǰ����������Ե���Ҫ���Һ��ƶ���
        foot_points[i][X_IDX] = -cog_x;
        foot_points[i][Y_IDX] = -cog_y;
        foot_points[i][Z_IDX] = -stand_height-cog_z;
    }
}

// ƽ�����ɺ���
float smooth(float current, float target, float step_increment) {
    float diff = target - current;
    if (fabs(diff) <= step_increment) {
        return target; // ����С�ڲ�����ֱ�ӵ���Ŀ��
    }
    return current + (diff > 0 ? step_increment : -step_increment); // ��Ŀ�귽���ƶ�һ��
}

/**
 * @brief ƽ�����ɵ�ָ������λ��
 * 
 * @param center_of_gravity ��������ϵ������������λ��
 * @param step_increment ��������(m)
 */
void dog_smooth_cog(const float center_of_gravity[3], const float foot_now[4][3], float step_increment, float foot_next[4][3])
{
    // ����Ŀ��������Ե�λ��
    float target_neutral_points[4][3];
    dog_cog_to_foot(center_of_gravity, /* rotation */ NULL, target_neutral_points);
    // ����ÿ������
    for (int i = 0; i < 4; i++) {
        foot_next[i][X_IDX] = smooth(foot_now[i][X_IDX], target_neutral_points[i][X_IDX], step_increment);
        foot_next[i][Y_IDX] = smooth(foot_now[i][Y_IDX], target_neutral_points[i][Y_IDX], step_increment);
        foot_next[i][Z_IDX] = smooth(foot_now[i][Z_IDX], target_neutral_points[i][Z_IDX], step_increment);
    }
}

void leg_set_motor_pos(uint8_t leg_id, const float motor_pos[3])
{
    memcpy(dog.leg[leg_id].motor_target_pos, motor_pos, 3 * sizeof(float));
}

void leg_set_joint_pos(uint8_t leg_id, const float joint_pos[3])
{
    memcpy(dog.leg[leg_id].join_target_pos, joint_pos, 3 * sizeof(float));
}

void leg_set_target_foot_pos(uint8_t leg_id, const float foot_pos[3])
{
    memcpy(dog.leg[leg_id].foot_target_pos, foot_pos, 3 * sizeof(float));
}



void leg_get_current_joints(uint8_t leg_idx, float joint_current_pos[3])
{
    float motor_current_pos[3];
    leg_get_motors_current_pos(leg_idx, motor_current_pos);
    leg_motor_to_joint(leg_idx, motor_current_pos, joint_current_pos);
}

void leg_get_current_foot_pos(uint8_t leg_idx, float foot_pos[3])
{
    float joint_current_pos[3];
    leg_get_current_joints(leg_idx, joint_current_pos);
    leg_forward_kinematics(leg_idx, joint_current_pos, foot_pos);
}

void leg_get_target_foot_pos(uint8_t leg_idx, float foot_pos[3])
{
    memcpy(foot_pos, dog.leg[leg_idx].foot_target_pos, 3 * sizeof(float));
}


void leg_get_neutral_pos(uint8_t leg_idx, float neutral_pos[3])
{
    memcpy(neutral_pos, dog.leg[leg_idx].foot_neutral_pos, 3 * sizeof(float));
}

void leg_get_joint_pos(uint8_t leg_idx, float joint_pos[3])
{
    memcpy(joint_pos, dog.leg[leg_idx].join_target_pos, 3 * sizeof(float));
}

void dog_get_body_vel(float velocity[3])
{
    memcpy(velocity, body_vel, 3 * sizeof(float));
}

void dog_set_body_vel(float vx, float vy, float w)
{
    body_vel[X_IDX] = vx;
    body_vel[Y_IDX] = vy;
    body_vel[Z_IDX] = w;
}

float dog_get_stand_height()
{
    return dog.params.posture.stand_height;
}

void dog_set_stand_height(float height)
{
    dog.params.posture.stand_height = height;
}



/*========================= send���� =========================*/
void dog_send_motors()
{
    float motors_target_pos[12];
    for (int leg_idx = 0; leg_idx < 4; leg_idx++) {
        memcpy(motors_target_pos + leg_idx * 3, 
               dog.leg[leg_idx].motor_target_pos, 
               3 * sizeof(float));
    }
    send_motors_target_pos(motors_target_pos);
}

void dog_send_motors_from_joints()
{
    // ����һ����ʱ�������洢���е��Ŀ��λ��
    float motors_target_pos[12];
    for (int leg_idx = 0; leg_idx < 4; leg_idx++) {
        leg_joint_to_motor(leg_idx, dog.leg[leg_idx].join_target_pos, dog.leg[leg_idx].motor_target_pos);
        memcpy(motors_target_pos + leg_idx * 3, 
               dog.leg[leg_idx].motor_target_pos, 
               3 * sizeof(float));
    }
    // ���õ���ķ��ͺ���
    send_motors_target_pos(motors_target_pos);
}
     
/**
 * @brief ���Ŀ��Ʋ���ʾ��
 */
// void dog_cog_test(void)
// {
//     // ��ʼ������λ�ã�Ĭ���ڼ������ĵ㣩
//     Point center_of_gravity = {0.0f, 0.0f, 0.0f};
    
//     // վ���߶�
//     float stand_height = -0.33f;
    
//     // ǰ����б��ʾ
//     printf("ǰ����б����...\n");
//     center_of_gravity.x = 0.05f;  // ��ǰƫ��5cm
//     dog_smooth_cog(&center_of_gravity, 1.0f);
//     extApi_sleepMs(1000);
    
//     // ���Ҳ���б��ʾ
//     printf("���Ҳ���б����...\n");
//     center_of_gravity.x = 0.0f;
//     center_of_gravity.y = -0.05f;  // ����ƫ��5cm
//     dog_smooth_cog(&center_of_gravity, 1.0f);
//     extApi_sleepMs(1000);
    
//     // ����ǰ����б��ʾ
//     printf("����ǰ����б����...\n");
//     center_of_gravity.x = 0.05f;   // ��ǰƫ��5cm
//     center_of_gravity.y = 0.05f;   // ����ƫ��5cm
//     dog_smooth_cog(&center_of_gravity, 1.0f);
//     extApi_sleepMs(1000);
    
//     // ��������λ��
//     printf("��������λ��...\n");
//     center_of_gravity.x = 0.0f;
//     center_of_gravity.y = 0.0f;
//     center_of_gravity.z = 0.0f;
//     dog_smooth_cog(&center_of_gravity, 1.0f);
// }
/*========================= ��ʼ������ =========================*/
void dog_init(const RobotParams* init_params)
{
    // ����ṩ�˳�ʼ������ʹ���ṩ�Ĳ���������ʹ��Ĭ�ϲ���
    if (init_params != NULL) {
        memcpy(&dog.params, init_params, sizeof(RobotParams));
    } else {
        // ����Ĭ�ϲ���
        memcpy(&dog.params, get_robot_params(), sizeof(RobotParams));
    }
    // ��ʼ����ID
    dog.leg[0].leg_id = LEG_FL;
    dog.leg[1].leg_id = LEG_FR;
    dog.leg[2].leg_id = LEG_HL;
    dog.leg[3].leg_id = LEG_HR;

    // ��ʼ��������Ե�λ��
    float foot_neutral_points[4][3];
    dog_cog_to_foot(dog.params.posture.center_of_gravity.translation, 
                     dog.params.posture.center_of_gravity.rotation, 
                     foot_neutral_points);
    
    for (int leg_idx = 0; leg_idx < 4; leg_idx++) {
        memcpy(dog.leg[leg_idx].foot_neutral_pos, foot_neutral_points[leg_idx], 3 * sizeof(float));
    }


    // ��ʼ���ؽڽǶ�(�����ϵ�ʱ�Ƕ�)
    float motor_current_pos[4][3];
    
    for (int leg_idx = 0; leg_idx < 4; leg_idx++) {
        leg_get_motors_current_pos(leg_idx, motor_current_pos[leg_idx]);
        leg_motor_to_joint(leg_idx, motor_current_pos[leg_idx], dog.leg[leg_idx].join_target_pos);
        leg_set_motor_pos(leg_idx, motor_current_pos[leg_idx]);
    }

    
    // ��ʼ������ǶȺ󣬸��������Ʋ���
    for (int i = 0; i < 12; i++) {
//        if (i == 8 || i == 7)
//        {
            J60_GetMotor(i)->kp = 0;
            J60_GetMotor(i)->kd = 0;
//        }
//        if (i == 9 || i==10 ||i==11)
//        {
//            J60_GetMotor(i)->kp = dog.params.motor_param.motor_control_params[i].kp;
//            J60_GetMotor(i)->kd = dog.params.motor_param.motor_control_params[i].kd;
//        }
        
    }
}

void dog_data_update()
{
    float motor_current_pos[4][3];
    for (int leg_idx = 0; leg_idx < 4; leg_idx++) {
        leg_get_motors_current_pos(leg_idx, motor_current_pos[leg_idx]);
        leg_set_motor_pos(leg_idx, motor_current_pos[leg_idx]);
    }
}

/*========================= ������������� =========================*/
// �����õ�Ŀ��ؽڽǶ�
float joint_target_pos_test[4][3] = {
    {0.3, 0.67, -1.3}, // FL
    {0.3, 0.67, -1.3}, // FR
    {-0.5, 0.3, -1},   // HL
    {-0.5, 0.3, -1}    // HR
};

// �����õ�Ŀ����λ��
float motor_target_pos_test[12];

// �����õ����λ��
float foot_pos_test[4][3];
float foot_pos_test_inverse[4][3] = {
    {-0.006368, 0.174142, -0.279386},
    {-0.006368, 0.014028, -0.328915},
    {0.069739, -0.091398, -0.342096},
    {0.069739, -0.238481, -0.261744}
};

// �����õĹؽڽǶ�
float joint_pos_test_inverse[4][3];

float cog_test[3] = {0.1f, 0.0f, 0.0f};
float foot_neutral_points_test_now[4][3] = {0};
float foot_neutral_points_test_next[4][3] = {0};

uint8_t first_run = 1;
// ��ȡ����������Ե�λ��
float neutral_pos[4][3]; 
float foot_target_pos[4][3];
int t = 0;
void test_robot_control()
{
    
}
