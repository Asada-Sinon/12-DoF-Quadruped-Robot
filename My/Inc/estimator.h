/**
 * @file estimator.h
 * @brief ״̬������ͷ�ļ� - ����Cʵ��
 */

#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#define STATE_DIM 18  // ״̬����ά��: ����λ��(3) + �����ٶ�(3) + 4�����λ��(12)
#define MEAS_DIM 15   // ��������ά��: IMU����(6) + ���λ�ò���(12�������Ӵ�ʱ��Ч)
#define MAX_VALID_MEAS MEAS_DIM  // �����Ч��������
#define MAX_MATRIX_DIM STATE_DIM  // ���ھ�����������ά��
#define OUTPUT_DIM 28 // �������ά��: ������λ��(12) + �������ٶ�(12) + ��˸߶�(4)
#define CTRL_DIM 3    // ��������ά��: ���ٶȹ���(3)
#define LEG_NUM 4     // �ȵ�����
#define MAT_SUCCESS 0     // �����ɹ�
#define MAT_ERROR -1      // ����ʧ��

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
    float F[STATE_DIM][STATE_DIM];      // ״̬ת�ƾ���
    float B[STATE_DIM][CTRL_DIM];       // �����������
    float Q[STATE_DIM][STATE_DIM];      // ��������Э�������

    // ����ģ�Ͳ��� (��̬����)
    float H[MEAS_DIM][STATE_DIM];       // �۲����
    float R[MEAS_DIM][MEAS_DIM];        // ��������Э�������

    int valid_measurements;             // ��Ч��������

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

/**
 * @brief �������� (��ά����汾)
 * @param n ����ά��
 * @param A �������
 * @param Ainv ��������
 * @return 0�ɹ�, -1ʧ��
 */
int mat_inverse(int n, float A[][MAX_MATRIX_DIM], float Ainv[][MAX_MATRIX_DIM]);

/**
 * @brief �������� (ָ��汾)
 * @param n ����ά��
 * @param A ������� (һά���鰴�д洢)
 * @param Ainv �������� (һά���鰴�д洢)
 * @return 0�ɹ�, -1ʧ��
 */
int mat_inverse_ptr(int n, const float *A, float *Ainv);

/**
 * @brief ר�Ŵ���3x3��������溯�� (��ά����汾)
 * @param A ����3x3����
 * @param Ainv ���3x3�����
 * @return 0�ɹ�, -1ʧ��
 */
int mat_inverse_3x3(float A[3][3], float Ainv[3][3]);

/**
 * @brief ר�Ŵ���3x3��������溯�� (ָ��汾)
 * @param A ����3x3���� (һά���鰴�д洢������9)
 * @param Ainv ���3x3����� (һά���鰴�д洢������9)
 * @return 0�ɹ�, -1ʧ��
 */
int mat_inverse_3x3_ptr(const float *A, float *Ainv);

/**
 * @brief ��ά����ֱ�Ӳ��
 * @param a ��������a������Ϊ3
 * @param b ��������b������Ϊ3
 * @param c �������c = a �� b������Ϊ3
 */
void vec3_cross(const float a[3], const float b[3], float c[3]);

/**
 * @brief ��ά������� (ָ��汾)
 * @param a ��������a������Ϊ3
 * @param b ��������b������Ϊ3
 * @param c �������c = a �� b������Ϊ3
 */
void vec3_cross_ptr(const float *a, const float *b, float *c);

#endif /* ESTIMATOR_H */


