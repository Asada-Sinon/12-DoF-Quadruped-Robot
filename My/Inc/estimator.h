/**
 * @file estimator.h
 * @brief ״̬������ͷ�ļ� - ����Cʵ��
 */

#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "matrix.h"

#define STATE_DIM 18  // ״̬����ά��: ����λ��(3) + �����ٶ�(3) + 4�����λ��(12)
#define MEAS_DIM 15   // ��������ά��: IMU����(6) + ���λ�ò���(12�������Ӵ�ʱ��Ч)
#define MAX_VALID_MEAS MEAS_DIM
#define OUTPUT_DIM 28 // �������ά��: ������λ��(12) + �������ٶ�(12) + ��˸߶�(4)
#define CTRL_DIM 3    // ��������ά��: ���ٶȹ���(3)
#define LEG_NUM 4     // �ȵ�����

// ���������ݽṹ
typedef struct {
    float acc[3];      // ���ٶȼ� (x,y,z)
    float gyro[3];     // ������ (roll,pitch,yaw)
    float leg_odom[3]; // �Ȳ���̼ƹ��Ƶ�λ��
    float contacts[4]; // ��˽Ӵ�״̬ (0/1)
} SensorData;

// ��չ�Ĵ��������ݽṹ
typedef struct {
    float acc[3];            // ���ٶȼ� (x,y,z)
    float gyro[3];           // ������ (roll,pitch,yaw)
    float quaternion[4];     // ��Ԫ����̬ (w,x,y,z)
    float foot_pos[12];      // �ĸ��������������ϵ�µ�λ��
    float joint_angles[12];  // 12���ؽڽǶ�
    int contact[4];          // ��˽Ӵ�״̬ (0=�޽Ӵ�, 1=�Ӵ�)
    float prev_foot_pos[12]; // ��һʱ�����λ��(���ڼ����ٶ�)
    float dt;                // ʱ�䲽��
} ExtendedSensorData;

// �������˲����ṹ�� - ����Cʵ��
typedef struct {
    // ״̬����
    float x[STATE_DIM];                 // ״̬����
    float P[STATE_DIM][STATE_DIM];      // ״̬Э�������

    // ϵͳģ�Ͳ���
    float F[STATE_DIM][STATE_DIM];      // ״̬ת�ƾ���A
    float B[STATE_DIM][CTRL_DIM];       // �����������B
    float Q[STATE_DIM][STATE_DIM];      // ��������Э�������Q
    float Q_init[STATE_DIM][STATE_DIM]; // ��ʼ��������Э�������Q

    // ����ģ�Ͳ��� (��̬����)
    float H[OUTPUT_DIM][STATE_DIM];       // �۲����C
    float R[OUTPUT_DIM][OUTPUT_DIM];        // ��������Э�������R
    float R_init[OUTPUT_DIM][OUTPUT_DIM]; // ��ʼ��������Э�������R

    // �������
    float y[OUTPUT_DIM];                // �������
    
    // ʱ�䲽��
    float dt;

    // ��һʱ��״̬(���ڼ����ٶ�)
    float prev_x[STATE_DIM];
    float prev_time;
} KalmanFilter;

// ��������

/**
 * @brief ��ʼ���������˲���
 * @param kf �������˲���ָ��
 * @param dt ʱ�䲽��
 */
void init_kalman_filter(KalmanFilter* kf, float dt);


/**
 * @brief ��ȡ28ά�������
 * @param kf �������˲���ָ��
 * @param output �����������[28]
 */
void kf_get_output_vector(KalmanFilter* kf, float output[]);

/**
 * @brief ��ȡ������λ������
 * @param kf �������˲���ָ��
 * @param psfB ������λ���������[4][3]
 */
void kf_get_foot_relative_positions(KalmanFilter* kf, float psfB[][3]);

/**
 * @brief ��ȡ�������ٶ�����
 * @param kf �������˲���ָ��
 * @param vsfB �������ٶ��������[4][3]
 */
void kf_get_foot_relative_velocities(KalmanFilter* kf, float vsfB[][3]);

void estimation_init(void);
void estimation_start(void);
void estimation_run(void);
#endif /* ESTIMATOR_H */


