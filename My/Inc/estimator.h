/**
 * @file estimator.h
 * @brief 状态估计器头文件 - 基础C实现
 */

#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#define STATE_DIM 18  // 状态向量维度: 机身位置(3) + 机身速度(3) + 4个足端位置(12)
#define MEAS_DIM 15   // 测量向量维度: IMU数据(6) + 足端位置测量(12，仅当接触时有效)
#define MAX_VALID_MEAS MEAS_DIM  // 最大有效测量数量
#define MAX_MATRIX_DIM STATE_DIM  // 用于矩阵操作的最大维度
#define OUTPUT_DIM 28 // 输出向量维度: 足端相对位置(12) + 足端相对速度(12) + 足端高度(4)
#define CTRL_DIM 3    // 控制输入维度: 加速度估计(3)
#define LEG_NUM 4     // 腿的数量
#define MAT_SUCCESS 0     // 操作成功
#define MAT_ERROR -1      // 操作失败

// 传感器数据结构
typedef struct {
    float acc[3];      // 加速度计 (x,y,z)
    float gyro[3];     // 陀螺仪 (roll,pitch,yaw)
    float leg_odom[3]; // 腿部里程计估计的位置
    float contacts[4]; // 足端接触状态 (0/1)
} SensorData;

// 扩展的传感器数据结构
typedef struct {
    float acc[3];            // 加速度计 (x,y,z)
    float gyro[3];           // 陀螺仪 (roll,pitch,yaw)
    float quaternion[4];     // 四元数姿态 (w,x,y,z)
    float foot_pos[12];      // 四个足端在世界坐标系下的位置
    float joint_angles[12];  // 12个关节角度
    int contact[4];          // 足端接触状态 (0=无接触, 1=接触)
    float prev_foot_pos[12]; // 上一时刻足端位置(用于计算速度)
    float dt;                // 时间步长
} ExtendedSensorData;

// 卡尔曼滤波器结构体 - 基础C实现
typedef struct {
    // 状态变量
    float x[STATE_DIM];                 // 状态向量
    float P[STATE_DIM][STATE_DIM];      // 状态协方差矩阵

    // 系统模型参数
    float F[STATE_DIM][STATE_DIM];      // 状态转移矩阵
    float B[STATE_DIM][CTRL_DIM];       // 控制输入矩阵
    float Q[STATE_DIM][STATE_DIM];      // 过程噪声协方差矩阵

    // 测量模型参数 (动态调整)
    float H[MEAS_DIM][STATE_DIM];       // 观测矩阵
    float R[MEAS_DIM][MEAS_DIM];        // 测量噪声协方差矩阵

    int valid_measurements;             // 有效测量数量

    // 输出变量
    float y[OUTPUT_DIM];                // 输出向量

    // 时间步长
    float dt;

    // 上一时刻状态(用于计算速度)
    float prev_x[STATE_DIM];
    float prev_time;
} KalmanFilter;

// 函数声明

/**
 * @brief 初始化卡尔曼滤波器
 * @param kf 卡尔曼滤波器指针
 * @param dt 时间步长
 */
void init_kalman_filter(KalmanFilter* kf, float dt);


/**
 * @brief 获取28维输出向量
 * @param kf 卡尔曼滤波器指针
 * @param output 输出向量数组[28]
 */
void kf_get_output_vector(KalmanFilter* kf, float output[]);

/**
 * @brief 获取足端相对位置向量
 * @param kf 卡尔曼滤波器指针
 * @param psfB 足端相对位置输出数组[4][3]
 */
void kf_get_foot_relative_positions(KalmanFilter* kf, float psfB[][3]);

/**
 * @brief 获取足端相对速度向量
 * @param kf 卡尔曼滤波器指针
 * @param vsfB 足端相对速度输出数组[4][3]
 */
void kf_get_foot_relative_velocities(KalmanFilter* kf, float vsfB[][3]);

/**
 * @brief 矩阵求逆 (二维数组版本)
 * @param n 矩阵维度
 * @param A 输入矩阵
 * @param Ainv 输出逆矩阵
 * @return 0成功, -1失败
 */
int mat_inverse(int n, float A[][MAX_MATRIX_DIM], float Ainv[][MAX_MATRIX_DIM]);

/**
 * @brief 矩阵求逆 (指针版本)
 * @param n 矩阵维度
 * @param A 输入矩阵 (一维数组按行存储)
 * @param Ainv 输出逆矩阵 (一维数组按行存储)
 * @return 0成功, -1失败
 */
int mat_inverse_ptr(int n, const float *A, float *Ainv);

/**
 * @brief 专门处理3x3矩阵的求逆函数 (二维数组版本)
 * @param A 输入3x3矩阵
 * @param Ainv 输出3x3逆矩阵
 * @return 0成功, -1失败
 */
int mat_inverse_3x3(float A[3][3], float Ainv[3][3]);

/**
 * @brief 专门处理3x3矩阵的求逆函数 (指针版本)
 * @param A 输入3x3矩阵 (一维数组按行存储，长度9)
 * @param Ainv 输出3x3逆矩阵 (一维数组按行存储，长度9)
 * @return 0成功, -1失败
 */
int mat_inverse_3x3_ptr(const float *A, float *Ainv);

/**
 * @brief 三维向量直接叉乘
 * @param a 输入向量a，长度为3
 * @param b 输入向量b，长度为3
 * @param c 输出向量c = a × b，长度为3
 */
void vec3_cross(const float a[3], const float b[3], float c[3]);

/**
 * @brief 三维向量叉乘 (指针版本)
 * @param a 输入向量a，长度为3
 * @param b 输入向量b，长度为3
 * @param c 输出向量c = a × b，长度为3
 */
void vec3_cross_ptr(const float *a, const float *b, float *c);

#endif /* ESTIMATOR_H */


