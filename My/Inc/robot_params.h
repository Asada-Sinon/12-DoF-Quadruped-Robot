#ifndef _ROBOT_PARAMS_H
#define _ROBOT_PARAMS_H

#include "stdint.h"
#include "math.h"
#include "j60.h"
// ��.h�ļ�����Ϊ�̶������������ˡ�����Ⱥ���Ҫ�޸ģ�һ��ֻ��Ҫ�޸�һ��
// ���ಽ̬���������Ʋ����ڴ��ļ���Ӧ��.c�ļ����޸�

/************************************************************
 * ������������
 ************************************************************/
// ��λת��ϵ��
#define RADIAN_TO_DEGREE 57.295779513f
#define DEGREE_TO_RADIAN 0.0174533f
#define MY_PI 3.1415926f
// ����������
//  0   1
//
//
//  2   3
#define LEG_FL 0
#define LEG_FR 1
#define LEG_HL 2
#define LEG_HR 3

// �ؽ���������
#define JOINT_HIP   0
#define JOINT_THIGH 1
#define JOINT_CALF  2

/*-------------------------�����ˡ�����Ⱥ���Ҫ�޸����в���-------------------------*/

/************************************************************
 * ��е����
 ************************************************************/
// ���˳��ȣ��ף�
#define LINK_LENGTH_HIP   0.0693f
#define LINK_LENGTH_THIGH 0.255f
#define LINK_LENGTH_CALF  0.265f

// ����ߴ�(�������ĵ����ȹؽڵľ��룬���ڷ�����ת���ٶȵ�xyƽ��)
#define BODY_LENGTH 0.459f 
#define BODY_WIDTH 0.3686f

// ��������ϵԭ����������ϵԭ�����
#define BODY_CENTER_TO_LEG_X (BODY_LENGTH/2.0f)
#define BODY_CENTER_TO_LEG_Y (BODY_WIDTH/2.0f - LINK_LENGTH_HIP)


// ���ȹؽ�������������߽Ƕ�
#define HIP_ANGLE atan2f(BODY_LENGTH, BODY_WIDTH)
#define CENTER_TO_THIGH (0.5f * sqrtf(BODY_LENGTH*BODY_LENGTH + BODY_WIDTH*BODY_WIDTH))
/************************************************************
 * Ĭ����̬����
 ************************************************************/
// վ���߶ȣ��ף�
#define DEFAULT_STAND_HEIGHT 0.33f //0.3

// վ��ʱ����λ��
#define DEFAULT_CENTER_OF_GRAVITY_TRANSLATION_X 0.01f
#define DEFAULT_CENTER_OF_GRAVITY_TRANSLATION_Y 0.001f
#define DEFAULT_CENTER_OF_GRAVITY_TRANSLATION_Z 0.0f

#define DEFAULT_CENTER_OF_GRAVITY_ROTATION_X 0.0f
#define DEFAULT_CENTER_OF_GRAVITY_ROTATION_Y 0.0f
#define DEFAULT_CENTER_OF_GRAVITY_ROTATION_Z 0.0f

// �ٶȲ�����ԭ��̤��ʱ����ǰ��������ת�Ļ������ֵ
#define DEFAULT_CENTER_OF_GRAVITY_VX 0.11f
#define DEFAULT_CENTER_OF_GRAVITY_VY -0.02f
#define DEFAULT_CENTER_OF_GRAVITY_VW -0.01f

// ����ƫ��������
#define DEFAULT_CENTER_OF_GRAVITY_MAX_X 0.2f
#define DEFAULT_CENTER_OF_GRAVITY_MIN_X -0.2f
#define DEFAULT_CENTER_OF_GRAVITY_MAX_Y 0.2f
#define DEFAULT_CENTER_OF_GRAVITY_MIN_Y -0.2f
#define DEFAULT_CENTER_OF_GRAVITY_MAX_Z 0.2f
#define DEFAULT_CENTER_OF_GRAVITY_MIN_Z -0.2f


// Ĭ�ϹؽڽǶȣ����ȣ� # ������
#define DEFAULT_JOINT_ANGLE_HIP    0.0f
#define DEFAULT_JOINT_ANGLE_THIGH  0.67f
#define DEFAULT_JOINT_ANGLE_CALF  -1.3f


/*-------------------------�����������-------------------------*/
/************************************************************
 * ���id��ؽ�id��Ӧ��ϵ
 ************************************************************/
#define MOTOR_FL_HIP_ID 3
#define MOTOR_FL_THIGH_ID 2
#define MOTOR_FL_CALF_ID 1

#define MOTOR_FR_HIP_ID 6
#define MOTOR_FR_THIGH_ID 5
#define MOTOR_FR_CALF_ID 4

#define MOTOR_HL_HIP_ID 12
#define MOTOR_HL_THIGH_ID 11
#define MOTOR_HL_CALF_ID 10

#define MOTOR_HR_HIP_ID 9
#define MOTOR_HR_THIGH_ID 8
#define MOTOR_HR_CALF_ID 7
/************************************************************
 * ���CAN��������
 ************************************************************/
#define MOTOR_FL_HIP_CAN_BUS CAN_BUS_1
#define MOTOR_FL_THIGH_CAN_BUS CAN_BUS_1
#define MOTOR_FL_CALF_CAN_BUS CAN_BUS_1

#define MOTOR_FR_HIP_CAN_BUS CAN_BUS_2
#define MOTOR_FR_THIGH_CAN_BUS CAN_BUS_2
#define MOTOR_FR_CALF_CAN_BUS CAN_BUS_2

#define MOTOR_HL_HIP_CAN_BUS CAN_BUS_1
#define MOTOR_HL_THIGH_CAN_BUS CAN_BUS_3
#define MOTOR_HL_CALF_CAN_BUS CAN_BUS_2

#define MOTOR_HR_HIP_CAN_BUS CAN_BUS_3
#define MOTOR_HR_THIGH_CAN_BUS CAN_BUS_3
#define MOTOR_HR_CALF_CAN_BUS CAN_BUS_3

/************************************************************
 * �������ϵ��أ�����ȷ���������ϵ��ؽ�����ϵ�Ĺ�ϵ��
 ************************************************************/
#define MOTOR_FL_HIP_DIRECTION -1 // ��ǰ���Źؽڵ����ת������ؽ�����ϵ��ת������ͬΪ1���෴Ϊ-1��
#define MOTOR_FL_THIGH_DIRECTION -1 // ��ǰ�ȴ��ȵ����ת������ؽ�����ϵ��ת������ͬΪ1���෴Ϊ-1��
#define MOTOR_FL_CALF_DIRECTION -1 // ��ǰ��С�ȵ����ת������ؽ�����ϵ��ת������ͬΪ1���෴Ϊ-1��

#define MOTOR_FR_HIP_DIRECTION -1 // ��ǰ���Źؽڵ����ת������ؽ�����ϵ��ת������ͬΪ1���෴Ϊ-1��
#define MOTOR_FR_THIGH_DIRECTION 1 // ��ǰ�ȴ��ȵ����ת������ؽ�����ϵ��ת������ͬΪ1���෴Ϊ-1��
#define MOTOR_FR_CALF_DIRECTION 1 // ��ǰ��С�ȵ����ת������ؽ�����ϵ��ת������ͬΪ1���෴Ϊ-1��

#define MOTOR_HL_HIP_DIRECTION 1 // ������Źؽڵ����ת������ؽ�����ϵ��ת������ͬΪ1���෴Ϊ-1��
#define MOTOR_HL_THIGH_DIRECTION -1 // ����ȴ��ȵ����ת������ؽ�����ϵ��ת������ͬΪ1���෴Ϊ-1��
#define MOTOR_HL_CALF_DIRECTION -1 // �����С�ȵ����ת������ؽ�����ϵ��ת������ͬΪ1���෴Ϊ-1��

#define MOTOR_HR_HIP_DIRECTION 1 // �Һ����Źؽڵ����ת������ؽ�����ϵ��ת������ͬΪ1���෴Ϊ-1��
#define MOTOR_HR_THIGH_DIRECTION 1 // �Һ��ȴ��ȵ����ת������ؽ�����ϵ��ת������ͬΪ1���෴Ϊ-1��
#define MOTOR_HR_CALF_DIRECTION 1 // �Һ���С�ȵ����ת������ؽ�����ϵ��ת������ͬΪ1���෴Ϊ-1��

// �ض�λ�õĹؽ�����ϵ�Ƕȣ�һ���ɽ�ģ�����ֱ�Ӳ����õ���
#define JOINT_FL_HIP_ANGLE_SPECIAL_POS 0.0f
#define JOINT_FL_THIGH_ANGLE_SPECIAL_POS 2.4622f * DEGREE_TO_RADIAN
#define JOINT_FL_CALF_ANGLE_SPECIAL_POS -31.753f * DEGREE_TO_RADIAN

#define JOINT_FR_HIP_ANGLE_SPECIAL_POS 0.0f
#define JOINT_FR_THIGH_ANGLE_SPECIAL_POS 2.4622f * DEGREE_TO_RADIAN
#define JOINT_FR_CALF_ANGLE_SPECIAL_POS -31.753f * DEGREE_TO_RADIAN

#define JOINT_HL_HIP_ANGLE_SPECIAL_POS 0.0f
#define JOINT_HL_THIGH_ANGLE_SPECIAL_POS 2.4622f * DEGREE_TO_RADIAN
#define JOINT_HL_CALF_ANGLE_SPECIAL_POS -31.753f * DEGREE_TO_RADIAN

#define JOINT_HR_HIP_ANGLE_SPECIAL_POS 0.0f
#define JOINT_HR_THIGH_ANGLE_SPECIAL_POS 2.4622f * DEGREE_TO_RADIAN
#define JOINT_HR_CALF_ANGLE_SPECIAL_POS -31.753f * DEGREE_TO_RADIAN

// ��Ӧ�ĵ������ϵ�Ƕȣ��ںùؽ�λ�ú�ӵ���ش����ݶ�ȡ����
#define MOTOR_FL_HIP_ANGLE_SPECIAL_POS 0.313f
#define MOTOR_FL_THIGH_ANGLE_SPECIAL_POS 1.54f
#define MOTOR_FL_CALF_ANGLE_SPECIAL_POS -2.23f

#define MOTOR_FR_HIP_ANGLE_SPECIAL_POS -0.517f
#define MOTOR_FR_THIGH_ANGLE_SPECIAL_POS -1.08f
#define MOTOR_FR_CALF_ANGLE_SPECIAL_POS 2.064f

#define MOTOR_HL_HIP_ANGLE_SPECIAL_POS -0.394f
#define MOTOR_HL_THIGH_ANGLE_SPECIAL_POS 1.04f
#define MOTOR_HL_CALF_ANGLE_SPECIAL_POS -2.168f

#define MOTOR_HR_HIP_ANGLE_SPECIAL_POS 0.179f
#define MOTOR_HR_THIGH_ANGLE_SPECIAL_POS -1.05f
#define MOTOR_HR_CALF_ANGLE_SPECIAL_POS 2.326f

// ����������ϵ��ؽ�����ϵ��λ�غ�ʱ����ĵ�ǰ�Ƕ�
#define MOTOR_FL_HIP_ANGLE_ZERO_POS (MOTOR_FL_HIP_ANGLE_SPECIAL_POS - MOTOR_FL_HIP_DIRECTION*JOINT_FL_HIP_ANGLE_SPECIAL_POS)
#define MOTOR_FL_THIGH_ANGLE_ZERO_POS (MOTOR_FL_THIGH_ANGLE_SPECIAL_POS - MOTOR_FL_THIGH_DIRECTION*JOINT_FL_THIGH_ANGLE_SPECIAL_POS)
#define MOTOR_FL_CALF_ANGLE_ZERO_POS (MOTOR_FL_CALF_ANGLE_SPECIAL_POS - MOTOR_FL_CALF_DIRECTION*JOINT_FL_CALF_ANGLE_SPECIAL_POS)

#define MOTOR_FR_HIP_ANGLE_ZERO_POS (MOTOR_FR_HIP_ANGLE_SPECIAL_POS - MOTOR_FR_HIP_DIRECTION*JOINT_FR_HIP_ANGLE_SPECIAL_POS)
#define MOTOR_FR_THIGH_ANGLE_ZERO_POS (MOTOR_FR_THIGH_ANGLE_SPECIAL_POS - MOTOR_FR_THIGH_DIRECTION*JOINT_FR_THIGH_ANGLE_SPECIAL_POS)
#define MOTOR_FR_CALF_ANGLE_ZERO_POS (MOTOR_FR_CALF_ANGLE_SPECIAL_POS - MOTOR_FR_CALF_DIRECTION*JOINT_FR_CALF_ANGLE_SPECIAL_POS)

#define MOTOR_HL_HIP_ANGLE_ZERO_POS (MOTOR_HL_HIP_ANGLE_SPECIAL_POS - MOTOR_HL_HIP_DIRECTION*JOINT_HL_HIP_ANGLE_SPECIAL_POS)
#define MOTOR_HL_THIGH_ANGLE_ZERO_POS (MOTOR_HL_THIGH_ANGLE_SPECIAL_POS - MOTOR_HL_THIGH_DIRECTION*JOINT_HL_THIGH_ANGLE_SPECIAL_POS)
#define MOTOR_HL_CALF_ANGLE_ZERO_POS (MOTOR_HL_CALF_ANGLE_SPECIAL_POS - MOTOR_HL_CALF_DIRECTION*JOINT_HL_CALF_ANGLE_SPECIAL_POS)

#define MOTOR_HR_HIP_ANGLE_ZERO_POS (MOTOR_HR_HIP_ANGLE_SPECIAL_POS - MOTOR_HR_HIP_DIRECTION*JOINT_HR_HIP_ANGLE_SPECIAL_POS)
#define MOTOR_HR_THIGH_ANGLE_ZERO_POS (MOTOR_HR_THIGH_ANGLE_SPECIAL_POS - MOTOR_HR_THIGH_DIRECTION*JOINT_HR_THIGH_ANGLE_SPECIAL_POS)
#define MOTOR_HR_CALF_ANGLE_ZERO_POS (MOTOR_HR_CALF_ANGLE_SPECIAL_POS - MOTOR_HR_CALF_DIRECTION*JOINT_HR_CALF_ANGLE_SPECIAL_POS)

/************************************************************
 * ����Ƕ����ƣ����ȣ�
 ************************************************************/
// FL�ȵ������
#define MOTOR_FL_HIP_MAX   0.65f
#define MOTOR_FL_HIP_MIN   -0.6f
#define MOTOR_FL_THIGH_MAX 2.166f
#define MOTOR_FL_THIGH_MIN 0.5f
#define MOTOR_FL_CALF_MAX  -0.26f
#define MOTOR_FL_CALF_MIN  -1.8f

// FR�ȵ������
#define MOTOR_FR_HIP_MAX    0.28f
#define MOTOR_FR_HIP_MIN   -0.8f
#define MOTOR_FR_THIGH_MAX -0.35f
#define MOTOR_FR_THIGH_MIN -2.0f
#define MOTOR_FR_CALF_MAX  1.7f
#define MOTOR_FR_CALF_MIN  0.13f

// HL�ȵ������
#define MOTOR_HL_HIP_MAX    0.4f
#define MOTOR_HL_HIP_MIN   -0.7f
#define MOTOR_HL_THIGH_MAX -0.18f
#define MOTOR_HL_THIGH_MIN 1.48f
#define MOTOR_HL_CALF_MAX  -0.48f
#define MOTOR_HL_CALF_MIN  -0.2f

// HR�ȵ������
#define MOTOR_HR_HIP_MAX    1.0f
#define MOTOR_HR_HIP_MIN   -0.15f
#define MOTOR_HR_THIGH_MAX -0.1f
#define MOTOR_HR_THIGH_MIN -1.68f
#define MOTOR_HR_CALF_MAX  1.9f
#define MOTOR_HR_CALF_MIN  0.25f


/************************************************************
 * �ṹ�嶨��
 ************************************************************/

// ���˲����ṹ��
typedef struct {
    float hip_length;     // �Źؽ����˳���
    float thigh_length;   // �������˳���
    float calf_length;    // С�����˳���
} LegLinkParams;

// �ؽڽǶ����ƽṹ��
typedef struct {
    float max_hip_angle;    // �Źؽ����Ƕ�
    float min_hip_angle;    // �Źؽ���С�Ƕ�
    float max_thigh_angle;  // ���ȹؽ����Ƕ�
    float min_thigh_angle;  // ���ȹؽ���С�Ƕ�
    float max_calf_angle;   // С�ȹؽ����Ƕ�
    float min_calf_angle;   // С�ȹؽ���С�Ƕ�
} LegAngleLimits;

// ����
typedef struct {
    float translation[3];
    float rotation[3];
    float velocity[3];
} CenterOfGravity;


// ����ƫ��������
typedef struct {
    float cog_x_max;    // ����x��������ƫ�������ֵ
    float cog_x_min;    // ����x��������ƫ������Сֵ
    float cog_y_max;    // ����y��������ƫ�������ֵ
    float cog_y_min;    // ����y��������ƫ������Сֵ
    float cog_z_max;    // ����z��������ƫ�������ֵ
    float cog_z_min;    // ����z��������ƫ������Сֵ
} CenterOfGravityLimits;

typedef struct {
    uint8_t hip_id;
    uint8_t thigh_id;
    uint8_t calf_id;
} MotorIDConfig;

typedef struct {
    int8_t hip_dir;
    int8_t thigh_dir;
    int8_t calf_dir;
} MotorJointDirConfig;

typedef struct {
    float hip_zero_pos;
    float thigh_zero_pos;
    float calf_zero_pos;
} MotorZeroPos;

typedef struct {
    float kp;
    float kd;
} MotorControlParams;

// �����ؽڶ�Ӧ��ϵ���ýṹ��
typedef struct {
    MotorIDConfig motor_id[4];
    LegAngleLimits motor_limits[4];
    MotorJointDirConfig motor_dir[4];
    MotorZeroPos motor_zero_pos[4];
    MotorControlParams motor_control_params[12];
} MotorParams; 

// ��̬�����ṹ��
typedef struct {
    float stand_height;    // վ���߶�
    CenterOfGravityLimits center_of_gravity_limit; // ����ƫ��������
    CenterOfGravity center_of_gravity; // ����λ��
    float default_joint_pos[4][3];  // ÿ���ȵ�Ĭ�ϹؽڽǶ� [leg][joint]
    int contact[4]; // ÿ���ȵĽӴ�״̬
    float phase[4]; // ÿ���ȵ�ǰ������λ
} PostureParams;

typedef struct {
    float phase[4];        // ��̬��λ
    float T;               // ��̬����(s)
    float stance_ratio;    // ֧����ʱ��ռ���ڱ���
    float swing_ratio;     // �ڶ���ʱ��ռ���ڱ���
    float step_length;     // ����
    float swing_height;    // �ڶ��߶�
    float stance_depth;    // ֧�����
    float stand_height;   // վ���߶�
} GaitParams;

// PID���Ʋ����ṹ��
typedef struct {
    float kp;  // ����ϵ��
    float ki;  // ����ϵ��
    float kd;  // ΢��ϵ��
} PIDParams;

// �����˲����ܽṹ��
typedef struct {
    LegLinkParams leg_links;       // �Ȳ����˲���
    LegAngleLimits joint_limits;   // �ؽڽǶ�����
    MotorParams motor_param;       // �������
    PostureParams posture;         // ��̬����

    GaitParams trot_gait;
    PIDParams position_control;    // λ�ÿ���PID����
    PIDParams velocity_control;    // �ٶȿ���PID����
} RobotParams;

/************************************************************
 * ��������
 ************************************************************/
RobotParams* get_robot_params(void);

#endif /* _ROBOT_PARAMS_H */
