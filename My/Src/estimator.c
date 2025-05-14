/************************************************************
 * 四足机器人卡尔曼滤波器状态估计器(18维状态向量) - 基础C实现
 * 状态向量x: [pb_x, pb_y, pb_z, vb_x, vb_y, vb_z, 
 *            p0_x, p0_y, p0_z, p1_x, p1_y, p1_z, 
 *            p2_x, p2_y, p2_z, p3_x, p3_y, p3_z]
 * 输出向量y: [psfB_0x, psfB_0y, psfB_0z, ..., psfB_3z,    // 12维足端到机身位置
 *            vsfB_0x, vsfB_0y, vsfB_0z, ..., vsfB_3z,     // 12维足端到机身速度
 *            psz_0, psz_1, psz_2, psz_3]                  // 4维足端高度
 * 输入: IMU数据、足端接触传感器、关节编码器数据
 * 输出: 估计的机身位置、速度、足端位置及相对量
 ************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "estimator.h"
#include "dog.h"  // 引入dog.h以使用腿部运动学函数

KalmanFilter kf;
ExtendedSensorData sensor_data;
// 自定义矩阵操作函数

/**
 * 矩阵乘法: C = A * B (使用指针形式)
 * @param[in] m A矩阵的行数
 * @param[in] n A矩阵的列数和B矩阵的行数
 * @param[in] p B矩阵的列数
 * @param[in] A 输入矩阵A，大小为m×n，按行存储
 * @param[in] B 输入矩阵B，大小为n×p，按行存储
 * @param[out] C 输出矩阵C，大小为m×p，按行存储
 */
void mat_mult_ptr(int m, int n, int p, const float *A, const float *B, float *C) {
    int i, j, k;
    for (i = 0; i < m; i++) {
        for (j = 0; j < p; j++) {
            C[i*p + j] = 0.0f;
            for (k = 0; k < n; k++) {
                C[i*p + j] += A[i*n + k] * B[k*p + j];
            }
        }
    }
}

/**
 * 矩阵乘以向量: y = A * x (使用指针形式)
 * @param[in] m A矩阵的行数
 * @param[in] n A矩阵的列数和x向量的维度
 * @param[in] A 输入矩阵A，大小为m×n，按行存储
 * @param[in] x 输入向量x，大小为n
 * @param[out] y 输出向量y，大小为m
 */
void mat_mult_vec_ptr(int m, int n, const float *A, const float *x, float *y) {
    int i, j;
    for (i = 0; i < m; i++) {
        y[i] = 0.0f;
        for (j = 0; j < n; j++) {
            y[i] += A[i*n + j] * x[j];
        }
    }
}

/**
 * 矩阵加法: C = A + B (使用指针形式)
 * @param[in] m 矩阵的行数
 * @param[in] n 矩阵的列数
 * @param[in] A 输入矩阵A，大小为m×n，按行存储
 * @param[in] B 输入矩阵B，大小为m×n，按行存储
 * @param[out] C 输出矩阵C，大小为m×n，按行存储
 */
void mat_add_ptr(int m, int n, const float *A, const float *B, float *C) {
    int i, j, idx;
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            idx = i*n + j;
            C[idx] = A[idx] + B[idx];
        }
    }
}

/**
 * 矩阵减法: C = A - B (使用指针形式)
 * @param[in] m 矩阵的行数
 * @param[in] n 矩阵的列数
 * @param[in] A 输入矩阵A，大小为m×n，按行存储
 * @param[in] B 输入矩阵B，大小为m×n，按行存储
 * @param[out] C 输出矩阵C，大小为m×n，按行存储
 */
void mat_sub_ptr(int m, int n, const float *A, const float *B, float *C) {
    int i, j, idx;
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            idx = i*n + j;
            C[idx] = A[idx] - B[idx];
        }
    }
}

/**
 * 矩阵转置: B = A^T (使用指针形式)
 * @param[in] m A矩阵的行数
 * @param[in] n A矩阵的列数
 * @param[in] A 输入矩阵，大小为m×n，按行存储
 * @param[out] B 输出转置矩阵，大小为n×m，按行存储
 */
void mat_transpose_ptr(int m, int n, const float *A, float *B) {
    int i, j;
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            B[j*m + i] = A[i*n + j];
        }
    }
}

/**
 * 向量减法: c = a - b (使用指针形式)
 * @param[in] n 向量的维度
 * @param[in] a 输入向量a，大小为n
 * @param[in] b 输入向量b，大小为n
 * @param[out] c 输出向量c，大小为n
 */
void vec_sub_ptr(int n, const float *a, const float *b, float *c) {
    int i;
    for (i = 0; i < n; i++) {
        c[i] = a[i] - b[i];
    }
}

/**
 * 向量加法: c = a + b (使用指针形式)
 * @param[in] n 向量的维度
 * @param[in] a 输入向量a，大小为n
 * @param[in] b 输入向量b，大小为n
 * @param[out] c 输出向量c，大小为n
 */
void vec_add_ptr(int n, const float *a, const float *b, float *c) {
    int i;
    for (i = 0; i < n; i++) {
        c[i] = a[i] + b[i];
    }
}

/**
 * 三维向量直接叉乘
 * @param[in] a 输入向量a，长度为3
 * @param[in] b 输入向量b，长度为3
 * @param[out] c 输出向量c = a × b，长度为3
 */
void vec3_cross(const float a[3], const float b[3], float c[3]) {
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

/**
 * 三维向量叉乘 (指针版本)
 * @param[in] a 输入向量a，长度为3
 * @param[in] b 输入向量b，长度为3
 * @param[out] c 输出向量c = a × b，长度为3
 */
void vec3_cross_ptr(const float *a, const float *b, float *c) {
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

// 矩阵求逆的指针版本 (使用LU分解法)
// 返回值: 0成功, -1失败
// A和Ainv是按行存储的一维数组形式
int mat_inverse_ptr(int n, const float *A, float *Ainv) {
    int i, j, k;
    float L[MAX_MATRIX_DIM * MAX_MATRIX_DIM] = {0}; // 下三角矩阵
    float U[MAX_MATRIX_DIM * MAX_MATRIX_DIM] = {0}; // 上三角矩阵
    float sum;
    int P[MAX_MATRIX_DIM];    // 行交换记录(整数索引)
    int temp_idx;
    float temp_val;
    float y[MAX_MATRIX_DIM];    // 中间解向量
    float b[MAX_MATRIX_DIM];    // 右侧向量
    
    printf("A(指针版本):\n");
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            printf("%f ", A[i*n + j]);
        }
        printf("\n");
    }
    
    if (n > MAX_MATRIX_DIM) {
        return MAT_ERROR; // 矩阵维度超出限制
    }
    
    // 初始化L和U矩阵
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            if (i == j) {
                L[i*n + j] = 1.0f; // L对角线初始化为1
            } else {
                L[i*n + j] = 0.0f;
            }
            U[i*n + j] = 0.0f;
        }
        P[i] = i; // 初始无行交换，使用整数索引
    }
    
    // 计算行列式值，检查是否可逆
    float det = 1.0f;
    for (i = 0; i < n; i++) {
        det *= A[i*n + i];
    }
    if (fabs(det) < 1e-5f) {
        // 快速检查是否可能不可逆
        // 进行更详细的检查
    }
    
    // LU分解带部分主元
    for (i = 0; i < n; i++) {
        // 查找主元
        int pivot_row = i;
        float pivot_val = 0.0f;
        for (j = i; j < n; j++) {
            if (fabs(A[P[j]*n + i]) > pivot_val) {
                pivot_val = fabs(A[P[j]*n + i]);
                pivot_row = j;
            }
        }
        
        // 检查矩阵是否可逆 - 提高阈值
        if (pivot_val < 1e-5f) {
            return MAT_ERROR; // 矩阵接近奇异，不可逆
        }
        
        // 交换行索引
        if (pivot_row != i) {
            temp_idx = P[i];
            P[i] = P[pivot_row];
            P[pivot_row] = temp_idx;
        }
        
        // 计算U的第i行
        for (j = i; j < n; j++) {
            sum = 0.0f;
            for (k = 0; k < i; k++) {
                sum += L[P[i]*n + k] * U[k*n + j];
            }
            U[i*n + j] = A[P[i]*n + j] - sum;
        }
        
        // 计算L的第i列
        for (j = i+1; j < n; j++) { // 从i+1开始，因为对角线元素已设为1
            sum = 0.0f;
            for (k = 0; k < i; k++) {
                sum += L[P[j]*n + k] * U[k*n + i];
            }
            // 避免除以非常小的数
            if (fabs(U[i*n + i]) < 1e-5f) {
                return MAT_ERROR; // 主元太小，不可逆
            }
            L[P[j]*n + i] = (A[P[j]*n + i] - sum) / U[i*n + i];
        }
    }
    
    // 通过求解方程AX=I来获取逆矩阵
    // 对每一列分别求解
    for (j = 0; j < n; j++) {
        // 构造单位向量
        for (i = 0; i < n; i++) {
            b[i] = (i == j) ? 1.0f : 0.0f;
        }
        
        // 前向替换求解Ly = b (考虑行交换)
        for (i = 0; i < n; i++) {
            y[i] = b[P[i]]; // 应用行交换
            for (k = 0; k < i; k++) {
                y[i] -= L[P[i]*n + k] * y[k];
            }
        }
        
        // 后向替换求解Ux = y
        for (i = n - 1; i >= 0; i--) {
            Ainv[i*n + j] = y[i];
            for (k = i + 1; k < n; k++) {
                Ainv[i*n + j] -= U[i*n + k] * Ainv[k*n + j];
            }
            // 再次检查数值稳定性
            if (fabs(U[i*n + i]) < 1e-5f) {
                return MAT_ERROR; // 主元太小，可能导致数值不稳定
            }
            Ainv[i*n + j] /= U[i*n + i];
        }
    }
    
    return MAT_SUCCESS;
}

// 专门处理3x3矩阵的求逆函数的指针版本
int mat_inverse_3x3_ptr(const float *A, float *Ainv) {
    // 计算行列式
    float det = A[0] * (A[4] * A[8] - A[5] * A[7])
              - A[1] * (A[3] * A[8] - A[5] * A[6])
              + A[2] * (A[3] * A[7] - A[4] * A[6]);
    
    // 判断是否可逆
    if (fabs(det) < 1e-5f) {
        printf("矩阵奇异，行列式接近于0: %f\n", det);
        return MAT_ERROR; // 矩阵不可逆
    }
    
    // 计算伴随矩阵的转置除以行列式
    float inv_det = 1.0f / det;
    
    Ainv[0] = (A[4] * A[8] - A[5] * A[7]) * inv_det;
    Ainv[1] = (A[2] * A[7] - A[1] * A[8]) * inv_det;
    Ainv[2] = (A[1] * A[5] - A[2] * A[4]) * inv_det;
    
    Ainv[3] = (A[5] * A[6] - A[3] * A[8]) * inv_det;
    Ainv[4] = (A[0] * A[8] - A[2] * A[6]) * inv_det;
    Ainv[5] = (A[2] * A[3] - A[0] * A[5]) * inv_det;
    
    Ainv[6] = (A[3] * A[7] - A[4] * A[6]) * inv_det;
    Ainv[7] = (A[1] * A[6] - A[0] * A[7]) * inv_det;
    Ainv[8] = (A[0] * A[4] - A[1] * A[3]) * inv_det;
    
    return MAT_SUCCESS;
}

// 初始化卡尔曼滤波器
void init_kalman_filter(KalmanFilter* kf, float dt) {
    int i, j;
    
    // 设置时间步长
    kf->dt = dt;
    kf->prev_time = 0;
    
    // 初始化状态向量和上一时刻状态为0
    memset(kf->x, 0, sizeof(kf->x));
    memset(kf->prev_x, 0, sizeof(kf->prev_x));
    
    // 初始化输出向量为0
    memset(kf->y, 0, sizeof(kf->y));
    
    // 初始化状态协方差矩阵为对角阵 * 初始不确定度
    memset(kf->P, 0, sizeof(kf->P));
    for (i = 0; i < STATE_DIM; i++) {
        // 机身位置与速度的初始不确定度较大
        if (i < 6) {
            kf->P[i][i] = 1.0f;
        } else {
            // 足端位置的初始不确定度较小(假设机器人初始站立姿态已知)
            kf->P[i][i] = 0.1f;
        }
    }
    
    // 初始化状态转移矩阵 F (离散时间系统)
    memset(kf->F, 0, sizeof(kf->F));
    for (i = 0; i < STATE_DIM; i++) {
        kf->F[i][i] = 1.0f;  // 单位矩阵部分
    }
    // 位置与速度的关系: pb(k+1) = pb(k) + vb(k)*dt
    kf->F[0][3] = kf->dt;  // pb_x与vb_x
    kf->F[1][4] = kf->dt;  // pb_y与vb_y
    kf->F[2][5] = kf->dt;  // pb_z与vb_z
    
    // 初始化控制输入矩阵 B
    memset(kf->B, 0, sizeof(kf->B));
    // 加速度对速度的影响: vb(k+1) = vb(k) + a(k)*dt
    kf->B[3][0] = kf->dt;  // vb_x与ax
    kf->B[4][1] = kf->dt;  // vb_y与ay
    kf->B[5][2] = kf->dt;  // vb_z与az
    
    // 初始化过程噪声协方差矩阵 Q
    memset(kf->Q, 0, sizeof(kf->Q));
    // 设置适当的过程噪声值 (基于系统特性调整)
    // 机身位置噪声
    for (i = 0; i < 3; i++) 
        kf->Q[i][i] = 0.01f;
    // 机身速度噪声
    for (i = 3; i < 6; i++) 
        kf->Q[i][i] = 0.1f;
    // 足端位置噪声(取决于是否接触地面)
    // 在update步骤中根据接触状态动态调整
    for (i = 6; i < STATE_DIM; i++) 
        kf->Q[i][i] = 0.05f;
    
    // 初始化观测矩阵 H 和测量噪声协方差矩阵 R
    memset(kf->H, 0, sizeof(kf->H));
    memset(kf->R, 0, sizeof(kf->R));
    
    // 初始有效测量数量
    kf->valid_measurements = 0;
}

// 根据接触状态更新观测模型
void update_observation_model(KalmanFilter* kf, ExtendedSensorData* sensor_data) {
    int i, j;
    int meas_idx = 0;
    
    // 重置观测矩阵和测量噪声协方差
    memset(kf->H, 0, sizeof(float) * MEAS_DIM * STATE_DIM);
    memset(kf->R, 0, sizeof(float) * MEAS_DIM * MEAS_DIM);
    
    // 1. IMU加速度数据用于速度估计(需要考虑姿态换算到世界坐标系)
    // 这部分需要从局部坐标系转换到世界坐标系，这里简化处理
    for (i = 0; i < 3; i++) {
        kf->H[meas_idx][i+3] = 1.0f;  // 速度观测
        kf->R[meas_idx][meas_idx] = 0.1f;  // 速度测量噪声
        meas_idx++;
    }
    
    // 2. 处理足端接触信息
    // 当足接触地面时，该足的位置在世界坐标系中变化很小(假设不打滑)
    for (i = 0; i < LEG_NUM; i++) {
        if (sensor_data->contact[i]) {
            // 该足接触地面，将其位置测量添加到观测
            for (j = 0; j < 3; j++) {
                // 足端位置在状态向量中的索引: 6+i*3+j
                kf->H[meas_idx][6+i*3+j] = 1.0f;
                // 接触点位置噪声小，因为地面提供了约束
                kf->R[meas_idx][meas_idx] = 0.01f;
                meas_idx++;
            }
        }
    }
    
    // 3. 使用运动学模型作为机身位置的间接测量
    // 对于每个接触地面的足，可以根据关节角度计算机身相对于足的位置
    for (i = 0; i < LEG_NUM; i++) {
        if (sensor_data->contact[i]) {
            // 添加机身位置与足端位置的关系约束
            for (j = 0; j < 3; j++) {
                // 机身位置 = 足端位置 - 从足端到机身的相对位置(由正向运动学提供)
                kf->H[meas_idx][j] = 1.0f;  // 机身位置
                kf->H[meas_idx][6+i*3+j] = -1.0f;  // 足端位置的负值
                kf->R[meas_idx][meas_idx] = 0.05f;  // 运动学约束噪声
                meas_idx++;
            }
        }
    }
    
    // 记录有效测量数量
    kf->valid_measurements = meas_idx;
}

// 准备测量向量
void prepare_measurement_vector(KalmanFilter* kf, ExtendedSensorData* sensor_data, float z[MEAS_DIM]) {
    int i, j;
    int meas_idx = 0;
    
    // 1. IMU加速度积分得到的速度估计(简化处理)
    for (i = 0; i < 3; i++) {
        // 这里假设已经将局部加速度转换到世界坐标系并考虑了重力补偿
        // 实际应用中需要使用四元数旋转矩阵进行坐标转换
        z[meas_idx++] = kf->x[i+3] + sensor_data->acc[i] * kf->dt;
    }
    
    // 2. 接触足端的位置测量
    for (i = 0; i < LEG_NUM; i++) {
        if (sensor_data->contact[i]) {
            for (j = 0; j < 3; j++) {
                z[meas_idx++] = sensor_data->foot_pos[i*3+j];
            }
        }
    }
    
    // 3. 机身位置的间接测量(通过运动学)
    float relative_pos[3];
    for (i = 0; i < LEG_NUM; i++) {
        if (sensor_data->contact[i]) {
            // 计算从足端到机身的相对位置(由正向运动学提供)
            // 这里使用已有的运动学函数计算从关节角度到足端位置的映射
            float joint_pos[3];
            float foot_to_body[3];
            
            // 从关节数组中提取当前腿的关节角度
            memcpy(joint_pos, &sensor_data->joint_angles[i*3], 3 * sizeof(float));
            
            // 计算足端相对于髋关节的位置
            leg_forward_kinematics(i, joint_pos, foot_to_body);
            
            // 计算从足端到机身的相对位置(假设髋关节位于机身坐标系原点)
            relative_pos[0] = -foot_to_body[0];
            relative_pos[1] = -foot_to_body[1];
            relative_pos[2] = -foot_to_body[2];
            
            // 机身位置 = 足端位置 - 相对位置
            for (j = 0; j < 3; j++) {
                z[meas_idx++] = sensor_data->foot_pos[i*3+j] - relative_pos[j];
            }
        }
    }
}

// 调整过程噪声协方差，考虑足端是否接触地面
void adjust_process_noise(KalmanFilter* kf, ExtendedSensorData* sensor_data) {
    int i, j;
    
    // 调整足端位置的过程噪声
    for (i = 0; i < LEG_NUM; i++) {
        // 接触地面的足端位置变化更小(噪声小)，悬空的足端位置变化大(噪声大)
        float foot_noise = sensor_data->contact[i] ? 0.001f : 0.1f;
        for (j = 0; j < 3; j++) {
            kf->Q[6+i*3+j][6+i*3+j] = foot_noise;
        }
    }
}

// 卡尔曼滤波器预测步骤
void kf_predict(KalmanFilter* kf, float u[CTRL_DIM]) {
    int i, j, k;
    float temp_x[MAX_MATRIX_DIM] = {0};
    float temp_matrix[MAX_MATRIX_DIM][MAX_MATRIX_DIM] = {0};
    float Fx[MAX_MATRIX_DIM] = {0};
    float Bu[MAX_MATRIX_DIM] = {0};
    float FP[MAX_MATRIX_DIM][MAX_MATRIX_DIM] = {0};
    float F_transpose[MAX_MATRIX_DIM][MAX_MATRIX_DIM] = {0};
    
    // 保存上一时刻状态(用于计算速度)
    memcpy(kf->prev_x, kf->x, sizeof(kf->x));
    kf->prev_time += kf->dt;
    
    // 1. 预测状态: x(k|k-1) = F * x(k-1|k-1) + B * u(k)
    
    // 计算 Fx
    mat_mult_vec_ptr(STATE_DIM, STATE_DIM, &kf->F[0][0], kf->x, Fx);
    
    // 如果有控制输入u，计算 Bu
    if (u != NULL) {
        mat_mult_vec_ptr(STATE_DIM, CTRL_DIM, &kf->B[0][0], u, Bu);
        
        // 计算 x = Fx + Bu
        vec_add_ptr(STATE_DIM, Fx, Bu, kf->x);
    } else {
        // 如果没有控制输入，直接使用 x = Fx
        memcpy(kf->x, Fx, sizeof(Fx));
    }
    
    // 2. 预测误差协方差: P(k|k-1) = F * P(k-1|k-1) * F^T + Q
    
    // 计算 FP = F * P
    mat_mult_ptr(STATE_DIM, STATE_DIM, STATE_DIM, &kf->F[0][0], &kf->P[0][0], &FP[0][0]);
    
    // 计算 F 的转置
    mat_transpose_ptr(STATE_DIM, STATE_DIM, &kf->F[0][0], &F_transpose[0][0]);
    
    // 计算 FPFT = FP * F^T
    mat_mult_ptr(STATE_DIM, STATE_DIM, STATE_DIM, &FP[0][0], &F_transpose[0][0], &temp_matrix[0][0]);
    
    // 计算 P = FPFT + Q
    mat_add_ptr(STATE_DIM, STATE_DIM, &temp_matrix[0][0], &kf->Q[0][0], &kf->P[0][0]);
}

// 卡尔曼滤波器更新步骤
void kf_update(KalmanFilter* kf, float z[MEAS_DIM]) {
    int i, j, k;
    int valid_meas = kf->valid_measurements;
    
    if (valid_meas <= 0) return; // 没有有效测量，跳过更新
    
    // 临时变量
    float Hx[MAX_VALID_MEAS] = {0};
    float innovation[MAX_VALID_MEAS] = {0};
    float HP[MAX_VALID_MEAS][MAX_MATRIX_DIM] = {0};
    float H_transpose[MAX_MATRIX_DIM][MAX_VALID_MEAS] = {0};
    float HPHT[MAX_VALID_MEAS][MAX_VALID_MEAS] = {0};
    float S[MAX_VALID_MEAS][MAX_VALID_MEAS] = {0};
    float S_inv[MAX_VALID_MEAS][MAX_VALID_MEAS] = {0};
    float PHT[MAX_MATRIX_DIM][MAX_VALID_MEAS] = {0};
    float K[MAX_MATRIX_DIM][MAX_VALID_MEAS] = {0};
    float Ky[MAX_MATRIX_DIM] = {0};
    float KH[MAX_MATRIX_DIM][MAX_MATRIX_DIM] = {0};
    float IKH[MAX_MATRIX_DIM][MAX_MATRIX_DIM] = {0};
    float temp_P[MAX_MATRIX_DIM][MAX_MATRIX_DIM] = {0};
    
    // 临时向量和矩阵 (为了适应变化的有效测量数量)
    float H_valid[MAX_VALID_MEAS][MAX_MATRIX_DIM] = {0};
    float R_valid[MAX_VALID_MEAS][MAX_VALID_MEAS] = {0};
    float z_valid[MAX_VALID_MEAS] = {0};
    
    // 提取有效的H, R和z
    for (i = 0; i < valid_meas; i++) {
        z_valid[i] = z[i];
        for (j = 0; j < valid_meas; j++) {
            R_valid[i][j] = kf->R[i][j];
        }
        for (j = 0; j < STATE_DIM; j++) {
            H_valid[i][j] = kf->H[i][j];
        }
    }
    
    // 0. 计算测量残差: innovation = z - H*x
    
    // 计算 H * x
    mat_mult_vec_ptr(valid_meas, STATE_DIM, &H_valid[0][0], kf->x, Hx);
    
    // 计算 innovation = z - Hx
    vec_sub_ptr(valid_meas, z_valid, Hx, innovation);
    
    // 1. 计算残差协方差: S = H * P * H^T + R
    
    // 计算 HP = H * P
    mat_mult_ptr(valid_meas, STATE_DIM, STATE_DIM, &H_valid[0][0], &kf->P[0][0], &HP[0][0]);
    
    // 计算 H 的转置
    mat_transpose_ptr(valid_meas, STATE_DIM, &H_valid[0][0], &H_transpose[0][0]);
    
    // 计算 HPHT = HP * H^T
    mat_mult_ptr(valid_meas, STATE_DIM, valid_meas, &HP[0][0], &H_transpose[0][0], &HPHT[0][0]);
    
    // 计算 S = HPHT + R
    mat_add_ptr(valid_meas, valid_meas, &HPHT[0][0], &R_valid[0][0], &S[0][0]);
    
    // 2. 计算卡尔曼增益: K = P * H^T * S^(-1)
    
    // 计算 S 的逆矩阵
    if (mat_inverse_ptr(valid_meas, &S[0][0], &S_inv[0][0]) != MAT_SUCCESS) {
        printf("警告: 矩阵求逆失败\n");
        return;
    }
    
    // 计算 P * H^T
    mat_mult_ptr(STATE_DIM, STATE_DIM, valid_meas, &kf->P[0][0], &H_transpose[0][0], &PHT[0][0]);
    
    // 计算 K = PHT * S_inv
    mat_mult_ptr(STATE_DIM, valid_meas, valid_meas, &PHT[0][0], &S_inv[0][0], &K[0][0]);
    
    // 3. 更新状态估计: x(k|k) = x(k|k-1) + K * innovation
    
    // 计算 K * innovation
    mat_mult_vec_ptr(STATE_DIM, valid_meas, &K[0][0], innovation, Ky);
    
    // 更新状态向量: x = x + Ky
    vec_add_ptr(STATE_DIM, kf->x, Ky, kf->x);
    
    // 4. 更新误差协方差: P(k|k) = (I - K*H) * P(k|k-1)
    
    // 计算 KH = K * H
    mat_mult_ptr(STATE_DIM, valid_meas, STATE_DIM, &K[0][0], &H_valid[0][0], &KH[0][0]);
    
    // 计算 I - KH
    for (i = 0; i < STATE_DIM; i++) {
        for (j = 0; j < STATE_DIM; j++) {
            IKH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];
        }
    }
    
    // 计算 (I - KH) * P
    mat_mult_ptr(STATE_DIM, STATE_DIM, STATE_DIM, &IKH[0][0], &kf->P[0][0], &temp_P[0][0]);
        
    // 更新误差协方差
    memcpy(kf->P, temp_P, sizeof(temp_P));
}

// 计算28维输出向量
void compute_output_vector(KalmanFilter* kf) {
    int i, j, idx = 0;
    
    // 计算足端相对位置向量 psfB (12维)
    for (i = 0; i < LEG_NUM; i++) {
        for (j = 0; j < 3; j++) {
            // 足端位置减去机身位置
            kf->y[idx++] = kf->x[6 + i*3 + j] - kf->x[j];
        }
    }
    
    // 计算足端相对速度向量 vsfB (12维)
    for (i = 0; i < LEG_NUM; i++) {
        for (j = 0; j < 3; j++) {
            // 计算足端位置变化率减去机身速度
            // (当前足端位置 - 上一时刻足端位置)/dt - 机身速度
            float foot_vel = (kf->x[6 + i*3 + j] - kf->prev_x[6 + i*3 + j]) / kf->dt;
            kf->y[12 + idx++] = foot_vel - kf->x[3 + j];
        }
    }
    
    // 计算足端高度 psz (4维)
    for (i = 0; i < LEG_NUM; i++) {
        // 足端z坐标就是高度
        kf->y[24 + i] = kf->x[6 + i*3 + 2];
    }
}


// 获取28维输出向量
void kf_get_output_vector(KalmanFilter* kf, float output[OUTPUT_DIM]) {
    memcpy(output, kf->y, OUTPUT_DIM * sizeof(float));
}

// 获取足端相对位置向量
void kf_get_foot_relative_positions(KalmanFilter* kf, float psfB[LEG_NUM][3]) {
    for (int i = 0; i < LEG_NUM; i++) {
        memcpy(psfB[i], &kf->y[i*3], 3 * sizeof(float));
    }
}

// 获取足端相对速度向量
void kf_get_foot_relative_velocities(KalmanFilter* kf, float vsfB[LEG_NUM][3]) {
    for (int i = 0; i < LEG_NUM; i++) {
        memcpy(vsfB[i], &kf->y[12 + i*3], 3 * sizeof(float));
    }
}

/**
 * 从四元数计算ZYX旋转矩阵
 * 输入: 四元数 q[4] = [w, x, y, z]
 * 输出: 3x3旋转矩阵 R，使用ZYX旋转顺序(先yaw，再pitch，最后roll)
 */
void quaternion_to_rotation_matrix_zyx(const float q[4], float R[3][3]) {
    float w = q[0];
    float x = q[1];
    float y = q[2];
    float z = q[3];
    
    // 归一化四元数以确保单位长度
    float norm = sqrtf(w*w + x*x + y*y + z*z);
    if (norm < 1e-6f) {
        // 如果四元数接近零，返回单位矩阵
        R[0][0] = 1.0f; R[0][1] = 0.0f; R[0][2] = 0.0f;
        R[1][0] = 0.0f; R[1][1] = 1.0f; R[1][2] = 0.0f;
        R[2][0] = 0.0f; R[2][1] = 0.0f; R[2][2] = 1.0f;
        return;
    }
    
    w /= norm;
    x /= norm;
    y /= norm;
    z /= norm;
    
    // 计算四元数的平方项
    float w2 = w*w;
    float x2 = x*x;
    float y2 = y*y;
    float z2 = z*z;
    
    // 计算乘积项
    float xy = x*y;
    float xz = x*z;
    float yz = y*z;
    float wx = w*x;
    float wy = w*y;
    float wz = w*z;
    
    // 构建旋转矩阵
    R[0][0] = w2 + x2 - y2 - z2;  // 1 - 2*(y2 + z2)
    R[0][1] = 2.0f * (xy - wz);
    R[0][2] = 2.0f * (xz + wy);
    
    R[1][0] = 2.0f * (xy + wz);
    R[1][1] = w2 - x2 + y2 - z2;  // 1 - 2*(x2 + z2)
    R[1][2] = 2.0f * (yz - wx);
    
    R[2][0] = 2.0f * (xz - wy);
    R[2][1] = 2.0f * (yz + wx);
    R[2][2] = w2 - x2 - y2 + z2;  // 1 - 2*(x2 + y2)
}


/**
 * 将加速度从机身坐标系转换到世界坐标系
 * 使用IMU提供的四元数
 */
void transform_acceleration_to_world_frame(const float q[4], const float acc_body[3], float acc_world[3]) {

    float R[3][3]; // 旋转矩阵
    quaternion_to_rotation_matrix_zyx(q, R);

    // 应用旋转变换
    mat_mult_vec_ptr(3, 3, &R[0][0], acc_body, acc_world);  // 使用指针版本
    
    // 补偿重力加速度 (假设世界坐标系Z轴向上)
    acc_world[2] += 9.81f;
}

//计算世界坐标系下足端相对于机身的位置
void world_foot_to_body_position(uint8_t leg_id, const float R[3][3], float PsfBi[3]) {
    float PbfBi[3];
    leg_get_current_foot_pos_body(leg_id, PbfBi);
    mat_mult_vec_ptr(3, 3, &R[0][0], PbfBi, PsfBi);  // 添加&R[0][0]以获取指针
}

// 计算世界坐标系下足端相对于机身的速度
void world_foot_to_body_velocity(uint8_t leg_id, const float R[3][3], float VsfBi[3]) {
    float PbfBi[3];
    float WbxPbfBi[3];
    float VbfBi[3];
    float Wb_add_Vb[3];
    float joint_pos[3];
    float joint_vel[3];
    leg_get_current_foot_pos_body(leg_id, PbfBi);
    vec3_cross(sensor_data.gyro, PbfBi, WbxPbfBi);  // 使用vec3_cross替代vec_cross
    leg_get_current_joint_pos(leg_id, joint_pos);
    leg_get_current_joint_vel(leg_id, joint_vel);
    leg_forward_kinematics_vel(leg_id, joint_pos, joint_vel, VbfBi);
    vec_add_ptr(3, WbxPbfBi, VbfBi, Wb_add_Vb);
    mat_mult_vec_ptr(3, 3, &R[0][0], Wb_add_Vb, VsfBi);  // 添加&R[0][0]以获取指针
}

// 卡尔曼滤波器完整处理步骤
void process_quadruped_state_estimation() {
    // 1. 根据接触状态调整过程噪声
    adjust_process_noise(&kf, &sensor_data);
    
    // 2. 更新观测模型
    update_observation_model(&kf, &sensor_data);
    
    // 3. 准备控制输入(IMU加速度在世界坐标系下)
    float u[CTRL_DIM];
    // 将加速度从机身坐标系转换到世界坐标系 
    // 使用四元数旋转，这里简化处理
    transform_acceleration_to_world_frame(sensor_data.quaternion, sensor_data.acc, u);
    // 4. 预测步骤
    kf_predict(&kf, u);
    
    // 5. 准备测量向量
    float z[MEAS_DIM];
    prepare_measurement_vector(&kf, &sensor_data, z);
    
    // 6. 更新步骤
    kf_update(&kf, z);
    
    // 7. 计算输出向量
    compute_output_vector(&kf);
}