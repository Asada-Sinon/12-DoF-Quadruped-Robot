/************************************************************
 * 四足机器人卡尔曼滤波器状态估计器(18维状态向量) - 基础C实现
 * 状态向量x: [pb_x, pb_y, pb_z, vb_x, vb_y, vb_z, 
 *            p0_x, p0_y, p0_z, p1_x, p1_y, p1_z, 
 *            p2_x, p2_y, p2_z, p3_x, p3_y, p3_z]
 * 测量向量y: [psfB_0x, psfB_0y, psfB_0z, ..., psfB_3z,    // 12维世界坐标系下足端到机身位置
 *            vsfB_0x, vsfB_0y, vsfB_0z, ..., vsfB_3z,     // 12维世界坐标系下足端到机身速度
 *            psz_0, psz_1, psz_2, psz_3]                  // 4维足端高度，防止误差积累，给0
 * 控制向量u: [asb, asy, asz + g]
 ************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "estimator.h"
#include "dog.h"  // 引入dog.h以使用腿部运动学函数
#include "matrix.h"
#include "imu.h"
#include "timer.h"

KalmanFilter kf;

float kf_start = 0;

// 一些常量矩阵
float dt = 0.002; // 离散化采样时间，即计算间隔s
float largeVariance = 100; // 大数，用于代替无穷
float I3dt_f32[3][3] = {0};
float I3_f32[3][3] = {0};
float _I3_f32[3][3] = {0};
float I12_f32[12][12] = {0};
float I18_f32[18][18] = {0};

struct LPFilter{
    float weight;
    uint8_t start;
    float pastValue;
} lpf;

// 低通滤波器
void LPFilter_init(float samplePeriod, float cutFrequency){
    lpf.weight = 1.0f / ( 1.0f + 1.0f/(2.0f * PI * samplePeriod * cutFrequency) );
    lpf.start  = 0;
}

void LPFilter(float newValue){
    if(!lpf.start){
        lpf.start = 1;
        lpf.pastValue = newValue;
    }
    lpf.pastValue = lpf.weight*newValue + (1-lpf.weight)*lpf.pastValue;
}

float LPF_get_value(){
    return lpf.pastValue;
}

void LPF_clear(){
    lpf.start = 0;
    lpf.pastValue = 0;
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
void body_to_world_acc(float R[3][3], const float acc_body[3], float acc_world[3]) {
    // 应用旋转变换
    mat_mult_vec_ptr(3, 3, &R[0][0], acc_body, acc_world);  // 使用指针版本
    // 补偿重力加速度 (假设世界坐标系Z轴向上)
    acc_world[2] += 9.81f;
}

//计算世界坐标系下足端相对于机身的位置
void world_foot_to_body_pos(uint8_t leg_id, float R[3][3], float PsfBi[3]) {
    float PbfBi[3];
    leg_get_current_foot_pos_body(leg_id, PbfBi);
    mat_mult_vec_ptr(3, 3, &R[0][0], PbfBi, PsfBi);  // 添加&R[0][0]以获取指针
}

// 计算世界坐标系下足端相对于机身的速度
void world_foot_to_body_vel(uint8_t leg_id, float R[3][3], float VsfBi[3]) {
    float PbfBi[3];
    float WbxPbfBi[3];
    float VbfBi[3];
    float Wb_add_Vb[3];
    float joint_pos[3];
    float joint_vel[3];
    leg_get_current_foot_pos_body(leg_id, PbfBi);
    vec3_cross(imu_get_data()->gyro, PbfBi, WbxPbfBi);  // 使用vec3_cross替代vec_cross
    leg_get_current_joint_pos(leg_id, joint_pos);
    leg_get_current_joint_vel(leg_id, joint_vel);
    leg_forward_kinematics_vel(leg_id, joint_pos, joint_vel, VbfBi);
    vec_add_ptr(3, WbxPbfBi, VbfBi, Wb_add_Vb);
    mat_mult_vec_ptr(3, 3, &R[0][0], Wb_add_Vb, VsfBi);  // 添加&R[0][0]以获取指针
}

void estimation_init() {
    // 初始化常量矩阵
    for (int i = 0; i < 3; i++) {
        I3_f32[i][i] = 1;
        I3dt_f32[i][i] = dt;
        _I3_f32[i][i] = -1;
    }
    for (int i = 0; i < 12; i++) {
        I12_f32[i][i] = 1;
    }
    for (int i = 0; i < 18; i++) {
        I18_f32[i][i] = 1;
    }
    // 构造离散化状态转移矩阵F = I + dt*A
    //[I3 I3dt 0]
    //[    I3   ]
    //[      I12] 18x18
    memset(kf.F, 0, sizeof(kf.F));
    mat_add_block_ptr(0, 0, 3, 3, 18, &kf.F[0][0], &I3_f32[0][0], &kf.F[0][0]);
    mat_add_block_ptr(0, 3, 3, 3, 18, &kf.F[0][0], &I3dt_f32[0][0], &kf.F[0][0]);
    mat_add_block_ptr(3, 3, 3, 3, 18, &kf.F[0][0], &I3_f32[0][0], &kf.F[0][0]);
    mat_add_block_ptr(6, 6, 12, 12, 18, &kf.F[0][0], &I12_f32[0][0], &kf.F[0][0]);
    // 构造离散化控制矩阵B = dt*B
    memset(kf.B, 0, sizeof(kf.B));
    mat_add_block_ptr(0, 3, 3, 3, 18, &kf.B[0][0], &I3dt_f32[0][0], &kf.B[0][0]);
    // 构造离散化观测矩阵H = C
    // [-I3 0 I3 0 0 0]
    // [-I3 0 0 I3 0 0]
    // [-I3 0 0 0 I3 0]
    // [-I3 0 0 0 0 I3]
    // [0 -I3 0 0 0 0]
    // [0 -I3 0 0 0 0]
    // [0 -I3 0 0 0 0]
    // [0 -I3 0 0 0 0]
    // [0 0 [0 0 1] 0 0 0]
    // [0 0 0 [0 0 1] 0 0]
    // [0 0 0 0 [0 0 1] 0]
    // [0 0 0 0 0 [0 0 1]] 28x18
    memset(kf.H, 0, sizeof(kf.H));
    mat_add_block_ptr(0, 0, 3, 3, 18, &kf.H[0][0], &_I3_f32[0][0], &kf.H[0][0]);
    mat_add_block_ptr(3, 0, 3, 3, 18, &kf.H[0][0], &_I3_f32[0][0], &kf.H[0][0]);
    mat_add_block_ptr(6, 0, 3, 3, 18, &kf.H[0][0], &_I3_f32[0][0], &kf.H[0][0]);
    mat_add_block_ptr(9, 0, 3, 3, 18, &kf.H[0][0], &_I3_f32[0][0], &kf.H[0][0]);
    mat_add_block_ptr(12, 3, 3, 3, 18, &kf.H[0][0], &_I3_f32[0][0], &kf.H[0][0]);
    mat_add_block_ptr(15, 3, 3, 3, 18, &kf.H[0][0], &_I3_f32[0][0], &kf.H[0][0]);
    mat_add_block_ptr(18, 3, 3, 3, 18, &kf.H[0][0], &_I3_f32[0][0], &kf.H[0][0]);
    mat_add_block_ptr(21, 3, 3, 3, 18, &kf.H[0][0], &_I3_f32[0][0], &kf.H[0][0]);
    mat_add_block_ptr(0, 6, 12, 12, 18, &kf.H[0][0], &I12_f32[0][0], &kf.H[0][0]);
    kf.H[24][8] = 1;
    kf.H[25][11] = 1;
    kf.H[26][14] = 1;
    kf.H[27][17] = 1;
    // 控制矩阵的噪声协方差矩阵，一般直接测出来。这里用的宇树程序里的值，如果效果不好需要自己重测
    float Cu[3][3] = {
        268.573,  -43.819, -147.211,
        -43.819 ,  92.949 ,  58.082,
        -147.211,   58.082,  302.120
    };
    // 状态变量的噪声协方差，前六个方差较小，后12个因为足端触底有抖动，因此方差较大
    float Qdig[18][18] = {0};
    for (int i = 0; i < 18; i++) {
        if (i < 3) {
            Qdig[i][i] = 0.0003;
        } else if (i < 6) {
            Qdig[i][i] = 0.0003;
        } else {
            Qdig[i][i] = 0.01;
        }
    }
    // 构造过程噪声协方差矩阵Q = Qdig + B * Cu * B^T
    float BCu[18][18] = {0};
    float BT[3][18] = {0};
    mat_mult_ptr(18, 3, 3, &kf.B[0][0], &Cu[0][0], &BCu[0][0]);
    mat_transpose_ptr(18, 3, &kf.B[0][0], &BT[0][0]);
    mat_mult_ptr(18, 3, 18, &BCu[0][0], &BT[0][0], &kf.Q_init[0][0]);
    mat_add_ptr(18, 18, &Qdig[0][0], &kf.Q_init[0][0], &kf.Q_init[0][0]);
    memcpy(kf.Q, kf.Q_init, sizeof(kf.Q));
    // 测量噪声协方差矩阵R，直接用的宇树的
    float R[28][28] = {
        {0.008, 0.012, -0.000, -0.009, 0.012, 0.000, 0.009, -0.009, -0.000, -0.009, -0.009, 0.000, -0.000, 0.000, -0.000, 0.000, -0.000, -0.001, -0.002, 0.000, -0.000, -0.003, -0.000, -0.001, 0.000, 0.000, 0.000, 0.000},
        {0.012, 0.019, -0.001, -0.014, 0.018, -0.000, 0.014, -0.013, -0.000, -0.014, -0.014, 0.001, -0.001, 0.001, -0.001, 0.000, 0.000, -0.001, -0.003, 0.000, -0.001, -0.004, -0.000, -0.001, 0.000, 0.000, 0.000, 0.000},
        {-0.000, -0.001, 0.001, 0.001, -0.001, 0.000, -0.000, 0.000, -0.000, 0.001, 0.000, -0.000, 0.000, -0.000, 0.000, 0.000, -0.000, -0.000, 0.000, -0.000, -0.000, -0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000},
        {-0.009, -0.014, 0.001, 0.010, -0.013, 0.000, -0.010, 0.010, 0.000, 0.010, 0.010, -0.000, 0.001, 0.000, 0.000, 0.001, -0.000, 0.001, 0.002, -0.000, 0.000, 0.003, 0.000, 0.001, 0.000, 0.000, 0.000, 0.000},
        {0.012, 0.018, -0.001, -0.013, 0.018, -0.000, 0.013, -0.013, -0.000, -0.013, -0.013, 0.001, -0.001, 0.000, -0.001, 0.000, 0.001, -0.001, -0.003, 0.000, -0.001, -0.004, -0.000, -0.001, 0.000, 0.000, 0.000, 0.000},
        {0.000, -0.000, 0.000, 0.000, -0.000, 0.001, 0.000, 0.000, -0.000, 0.000, 0.000, -0.000, -0.000, 0.000, -0.000, 0.000, 0.000, 0.000, -0.000, -0.000, -0.000, -0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000},
        {0.009, 0.014, -0.000, -0.010, 0.013, 0.000, 0.010, -0.010, -0.000, -0.010, -0.010, 0.000, -0.001, 0.000, -0.001, 0.000, -0.000, -0.001, -0.001, 0.000, -0.000, -0.003, -0.000, -0.001, 0.000, 0.000, 0.000, 0.000},
        {-0.009, -0.013, 0.000, 0.010, -0.013, 0.000, -0.010, 0.009, 0.000, 0.010, 0.010, -0.000, 0.001, -0.000, 0.000, -0.000, 0.000, 0.001, 0.002, 0.000, 0.000, 0.003, 0.000, 0.001, 0.000, 0.000, 0.000, 0.000},
        {-0.000, -0.000, -0.000, 0.000, -0.000, -0.000, -0.000, 0.000, 0.001, 0.000, 0.000, 0.000, 0.000, -0.000, 0.000, -0.000, 0.000, -0.000, 0.000, -0.000, 0.000, 0.000, -0.000, -0.000, 0.000, 0.000, 0.000, 0.000},
        {-0.009, -0.014, 0.001, 0.010, -0.013, 0.000, -0.010, 0.010, 0.000, 0.010, 0.010, -0.000, 0.001, 0.000, 0.000, -0.000, -0.000, 0.001, 0.002, -0.000, 0.000, 0.003, 0.000, 0.001, 0.000, 0.000, 0.000, 0.000},
        {-0.009, -0.014, 0.000, 0.010, -0.013, 0.000, -0.010, 0.010, 0.000, 0.010, 0.010, -0.000, 0.001, -0.000, 0.000, -0.000, 0.000, 0.001, 0.002, -0.000, 0.000, 0.003, 0.001, 0.001, 0.000, 0.000, 0.000, 0.000},
        {0.000, 0.001, -0.000, -0.000, 0.001, -0.000, 0.000, -0.000, 0.000, -0.000, -0.000, 0.001, 0.000, -0.000, -0.000, -0.000, 0.000, 0.000, -0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000},
        {-0.000, -0.001, 0.000, 0.001, -0.001, -0.000, -0.001, 0.001, 0.000, 0.001, 0.001, 0.000, 1.708, 0.048, 0.784, 0.062, 0.042, 0.053, 0.077, 0.001, -0.061, 0.046, -0.019, -0.029, 0.000, 0.000, 0.000, 0.000},
        {0.000, 0.001, -0.000, 0.000, 0.000, 0.000, 0.000, -0.000, -0.000, 0.000, -0.000, -0.000, 0.048, 5.001, -1.631, -0.036, 0.144, 0.040, 0.036, 0.016, -0.051, -0.067, -0.024, -0.005, 0.000, 0.000, 0.000, 0.000},
        {-0.000, -0.001, 0.000, 0.000, -0.001, -0.000, -0.001, 0.000, 0.000, 0.000, 0.000, -0.000, 0.784, -1.631, 1.242, 0.057, -0.037, 0.018, 0.034, -0.017, -0.015, 0.058, -0.021, -0.029, 0.000, 0.000, 0.000, 0.000},
        {0.000, 0.000, 0.000, 0.001, 0.000, 0.000, 0.000, -0.000, -0.000, -0.000, -0.000, -0.000, 0.062, -0.036, 0.057, 6.228, -0.014, 0.932, 0.059, 0.053, -0.069, 0.148, 0.015, -0.031, 0.000, 0.000, 0.000, 0.000},
        {-0.000, 0.000, -0.000, -0.000, 0.001, 0.000, -0.000, 0.000, 0.000, -0.000, 0.000, 0.000, 0.042, 0.144, -0.037, -0.014, 3.011, 0.986, 0.076, 0.030, -0.052, -0.027, 0.057, 0.051, 0.000, 0.000, 0.000, 0.000},
        {-0.001, -0.001, -0.000, 0.001, -0.001, 0.000, -0.001, 0.001, -0.000, 0.001, 0.001, 0.000, 0.053, 0.040, 0.018, 0.932, 0.986, 0.885, 0.090, 0.044, -0.055, 0.057, 0.051, -0.003, 0.000, 0.000, 0.000, 0.000},
        {-0.002, -0.003, 0.000, 0.002, -0.003, -0.000, -0.001, 0.002, 0.000, 0.002, 0.002, -0.000, 0.077, 0.036, 0.034, 0.059, 0.076, 0.090, 6.230, 0.139, 0.763, 0.013, -0.019, -0.024, 0.000, 0.000, 0.000, 0.000},
        {0.000, 0.000, -0.000, -0.000, 0.000, -0.000, 0.000, 0.000, -0.000, -0.000, -0.000, 0.000, 0.001, 0.016, -0.017, 0.053, 0.030, 0.044, 0.139, 3.130, -1.128, -0.010, 0.131, 0.018, 0.000, 0.000, 0.000, 0.000},
        {-0.000, -0.001, -0.000, 0.000, -0.001, -0.000, -0.000, 0.000, 0.000, 0.000, 0.000, 0.000, -0.061, -0.051, -0.015, -0.069, -0.052, -0.055, 0.763, -1.128, 0.866, -0.022, -0.053, 0.007, 0.000, 0.000, 0.000, 0.000},
        {-0.003, -0.004, -0.000, 0.003, -0.004, -0.000, -0.003, 0.003, 0.000, 0.003, 0.003, 0.000, 0.046, -0.067, 0.058, 0.148, -0.027, 0.057, 0.013, -0.010, -0.022, 2.437, -0.102, 0.938, 0.000, 0.000, 0.000, 0.000},
        {-0.000, -0.000, 0.000, 0.000, -0.000, 0.000, -0.000, 0.000, -0.000, 0.000, 0.001, 0.000, -0.019, -0.024, -0.021, 0.015, 0.057, 0.051, -0.019, 0.131, -0.053, -0.102, 4.944, 1.724, 0.000, 0.000, 0.000, 0.000},
        {-0.001, -0.001, 0.000, 0.001, -0.001, 0.000, -0.001, 0.001, -0.000, 0.001, 0.001, 0.000, -0.029, -0.005, -0.029, -0.031, 0.051, -0.003, -0.024, 0.018, 0.007, 0.938, 1.724, 1.569, 0.000, 0.000, 0.000, 0.000},
        {0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 1.0, 0.000, 0.000, 0.000},
        {0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 1.0, 0.000, 0.000},
        {0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 1.0, 0.000},
        {0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 1.0}
    };
    memcpy(kf.R_init, R, sizeof(kf.R_init));
    memcpy(kf.R, kf.R_init, sizeof(kf.R));
    
    // 初始化协方差矩阵P
    memset(kf.P, 0, sizeof(kf.P));
    for (int i = 0; i < 18; i++) {
        kf.P[i][i] = largeVariance;
    }
    // 初始化状态变量x
    memset(kf.x, 0, sizeof(kf.x));
    // 初始化低通滤波器
    LPFilter_init(dt, 3.0);
}

// 窗口函数，用于降低触底抖动对状态估计的影响
float windowFunc(float x, float windowRatio){
    float xRange = 1;
    float yRange = 1;
    if((x < 0)||(x > xRange)){
        printf("The x=%f, which should between [0, xRange]\n", x);
    }
    if((windowRatio <= 0)||(windowRatio >= 0.5f)){
        printf("The windowRatio=%f, which should between [0, 0.5]\n", windowRatio);
    }
    if(x/xRange < windowRatio){
        return x * yRange / (xRange * windowRatio);
    }
    else if(x/xRange > 1 - windowRatio){
        return yRange * (xRange - x)/(xRange * windowRatio);
    }
    else{
        return yRange;
    }
}

/* -----------中间变量------------- */
float Rot_f32[3][3] = {0};
float u_f32[3] = {0};
float feetPos_f32[4][3] = {0};
float feetVel_f32[4][3] = {0};
float feetZ_f32[4] = {0};
float Ax_f32[STATE_DIM] = {0};
float Bu_f32[STATE_DIM] = {0};
float xhat_f32[STATE_DIM] = {0};
float yhat_f32[OUTPUT_DIM] = {0};
float Ppri_f32[STATE_DIM][STATE_DIM] = {0};
float PpriT_f32[STATE_DIM][STATE_DIM] = {0};
float AP_f32[STATE_DIM][STATE_DIM] = {0};
float AT_f32[STATE_DIM][STATE_DIM] = {0};
float APAT_f32[STATE_DIM][STATE_DIM] = {0};
float CT_f32[STATE_DIM][OUTPUT_DIM] = {0};
float CP_f32[OUTPUT_DIM][STATE_DIM] = {0};
float S_f32[OUTPUT_DIM][OUTPUT_DIM] = {0};
float ST_f32[OUTPUT_DIM][OUTPUT_DIM] = {0};
float SL_f32[OUTPUT_DIM][OUTPUT_DIM] = {0};
float SU_f32[OUTPUT_DIM][OUTPUT_DIM] = {0};
int LU_P_f32[OUTPUT_DIM] = {0};
float invSy_f32[OUTPUT_DIM] = {0};
float y_yhat_f32[OUTPUT_DIM] = {0};
float invSC_f32[OUTPUT_DIM][STATE_DIM] = {0};
float invSR_f32[OUTPUT_DIM][OUTPUT_DIM] = {0};
float invSTC_f32[STATE_DIM][OUTPUT_DIM] = {0};
float KC_f32[STATE_DIM][STATE_DIM] = {0};
float I_KC_f32[STATE_DIM][STATE_DIM] = {0};
float PCT_f32[STATE_DIM][OUTPUT_DIM] = {0};
float I_KCT_f32[STATE_DIM][STATE_DIM] = {0};
float I_KCPI_KCT_f32[STATE_DIM][STATE_DIM] = {0};
float KR_f32[STATE_DIM][OUTPUT_DIM] = {0};
float KRKT_f32[STATE_DIM][STATE_DIM] = {0};

float start_time[5];
float use_time[5]; 
float use_time_all = 0;
void estimation_run() {
    if(kf_start == 0){
        return;
    }
    start_time[0] = getTime();
    static arm_matrix_instance_f32 I18; 
    static arm_matrix_instance_f32 A; 
    static arm_matrix_instance_f32 x; 
    static arm_matrix_instance_f32 B; 
    static arm_matrix_instance_f32 u;
    static arm_matrix_instance_f32 C;
    static arm_matrix_instance_f32 y;
    static arm_matrix_instance_f32 xhat;
    static arm_matrix_instance_f32 yhat;
    static arm_matrix_instance_f32 Ax;
    static arm_matrix_instance_f32 Bu;
    static arm_matrix_instance_f32 Ppri;
    static arm_matrix_instance_f32 PpriT;
    static arm_matrix_instance_f32 AP;
    static arm_matrix_instance_f32 AT;
    static arm_matrix_instance_f32 APAT;
    static arm_matrix_instance_f32 P;
    static arm_matrix_instance_f32 Q;
    static arm_matrix_instance_f32 CP;
    static arm_matrix_instance_f32 CT;
    static arm_matrix_instance_f32 PCT;
    static arm_matrix_instance_f32 R;
    static arm_matrix_instance_f32 S;
    static arm_matrix_instance_f32 ST;
    static arm_matrix_instance_f32 y_yhat;
    static arm_matrix_instance_f32 invSy;
    static arm_matrix_instance_f32 invSC;
    static arm_matrix_instance_f32 invSR;
    static arm_matrix_instance_f32 invSTC;
    static arm_matrix_instance_f32 KC;
    static arm_matrix_instance_f32 I_KC;
    static arm_matrix_instance_f32 I_KCT;
    static arm_matrix_instance_f32 I_KCPI_KCT;
    static arm_matrix_instance_f32 KR;
    static arm_matrix_instance_f32 KRKT;
    
    /* -----------根据触底状态更新协方差矩阵------------- */
    for(int i = 0; i < 4; ++i){
        if(leg_get_contact_state(i) == 0){
            for (int j = 0; j < 3; j ++)
            {
                kf.Q[6+3*i+j][6+3*i+j] = largeVariance;
                kf.R[12+3*i+j][12+3*i+j] = largeVariance;
            }
            kf.R[24+i][24+i] = largeVariance;
        }
        else{
            float trust = windowFunc(leg_get_phase(i), 0.2);
            for (int j = 0; j < 3; j ++)
            {
                kf.Q[6+3*i+j][6+3*i+j] = (1 + (1-trust)*largeVariance) * kf.Q_init[6+3*i+j][6+3*i+j];
                kf.R[12+3*i+j][12+3*i+j] = (1 + (1-trust)*largeVariance) * kf.R_init[12+3*i+j][12+3*i+j];
            }
            kf.R[24+i][24+i] = (1 + (1-trust)*largeVariance) * kf.R_init[24+i][24+i];
        }
    }
    use_time[0] = (getTime() - start_time[0]) * 1000;
    /* -----------预测部分------------- */
    // R
    quaternion_to_rotation_matrix_zyx(imu_get_data()->quaternion, Rot_f32);
    // u = R * acc + g
    body_to_world_acc(Rot_f32, imu_get_data()->acc, u_f32);
    // xhat = A * x + B * u
    arm_mat_init_f32(&A, STATE_DIM, STATE_DIM, (float32_t*)kf.F);
    arm_mat_init_f32(&x, STATE_DIM, 1, (float32_t*)kf.x);
    arm_mat_init_f32(&B, STATE_DIM, CTRL_DIM, (float32_t*)kf.B);
    arm_mat_init_f32(&u, CTRL_DIM, 1, (float32_t*)u_f32);
    arm_mat_init_f32(&xhat, STATE_DIM, 1, (float32_t*)xhat_f32);
    arm_mat_init_f32(&Ax, STATE_DIM, 1, (float32_t*)Ax_f32);
    arm_mat_init_f32(&Bu, STATE_DIM, 1, (float32_t*)Bu_f32);
    
    arm_mat_mult_f32(&A, &x, &Ax);
    arm_mat_mult_f32(&B, &u, &Bu);
    arm_mat_add_f32(&Ax, &Bu, &xhat);
    
    // yhat = C * x
    arm_mat_init_f32(&C, OUTPUT_DIM, STATE_DIM, (float32_t*)kf.H);
    arm_mat_init_f32(&yhat, OUTPUT_DIM, 1, (float32_t*)yhat_f32);
    arm_mat_mult_f32(&C, &x, &yhat);
    
    // Ppri = A * P * A^T + Q
    arm_mat_init_f32(&Ppri, STATE_DIM, STATE_DIM, (float32_t*)Ppri_f32);
    arm_mat_init_f32(&P, STATE_DIM, STATE_DIM, (float32_t*)kf.P);
    arm_mat_init_f32(&AP, STATE_DIM, STATE_DIM, (float32_t*)AP_f32);
    arm_mat_init_f32(&AT, STATE_DIM, STATE_DIM, (float32_t*)AT_f32);
    arm_mat_init_f32(&APAT, STATE_DIM, STATE_DIM, (float32_t*)APAT_f32);
    arm_mat_init_f32(&Q, STATE_DIM, STATE_DIM, (float32_t*)kf.Q);
    
    
    arm_mat_mult_f32(&A, &P, &AP);
    arm_mat_trans_f32(&A, &AT);
    start_time[1] = getTime();
    // arm_mat_mult_f32(&AP, &AT, &APAT);
    mat_mult_18x18_optimized(AP.pData, AT.pData, APAT.pData);
//    mat_mult_ptr(STATE_DIM, STATE_DIM, STATE_DIM, AP.pData, AT.pData, APAT.pData);
    use_time[1] = (getTime() - start_time[1]) * 1000;
    arm_mat_add_f32(&APAT, &Q, &Ppri);
    
    /* -----------更新部分------------- */
    start_time[2] = getTime();
    // y 28x1
    for(int i = 0; i < 4; i ++){
        world_foot_to_body_pos(i, Rot_f32, feetPos_f32[i]);
        world_foot_to_body_vel(i, Rot_f32, feetVel_f32[i]);
        feetZ_f32[i] = 0;
        for(int j = 0; j < 3; j ++){
            kf.y[3*i+j] = feetPos_f32[i][j];
            kf.y[3*i+12+j] = feetVel_f32[i][j];
        }
        kf.y[24+i] = feetZ_f32[i];
    }
    
    // S = R + C * Ppri * C^T 28x28
    arm_mat_init_f32(&CP, OUTPUT_DIM, STATE_DIM, (float32_t*)CP_f32);
    arm_mat_init_f32(&CT, STATE_DIM, OUTPUT_DIM, (float32_t*)CT_f32);
    arm_mat_init_f32(&R, OUTPUT_DIM, OUTPUT_DIM, (float32_t*)kf.R);
    arm_mat_init_f32(&S, OUTPUT_DIM, OUTPUT_DIM, (float32_t*)S_f32);

    arm_mat_mult_f32(&C, &Ppri, &CP);
    arm_mat_trans_f32(&C, &CT);
    arm_mat_mult_f32(&CP, &CT, &S);
    arm_mat_add_f32(&S, &R, &S);
    use_time[2] = (getTime() - start_time[2]) * 1000;
    // S^-1(y - yhat) 28x1
    start_time[3] = getTime();
    arm_mat_init_f32(&y, OUTPUT_DIM, 1, (float32_t*)kf.y);
    arm_mat_init_f32(&y_yhat, OUTPUT_DIM, 1, (float32_t*)y_yhat_f32);
    arm_mat_init_f32(&invSy, OUTPUT_DIM, 1, (float32_t*)invSy_f32);

    arm_mat_sub_f32(&y, &yhat, &y_yhat);
    mat_lu_decomp_ptr(OUTPUT_DIM, S.pData, (float*)SL_f32, (float*)SU_f32, LU_P_f32);
    mat_solve_with_lu_ptr(OUTPUT_DIM, (float*)SL_f32, (float*)SU_f32, LU_P_f32, 1, y_yhat.pData, invSy.pData);
    // S^-1 * C 28x18
    arm_mat_init_f32(&invSC, OUTPUT_DIM, STATE_DIM, (float32_t*)invSC_f32);
    mat_solve_with_lu_ptr(OUTPUT_DIM, (float*)SL_f32, (float*)SU_f32, LU_P_f32, STATE_DIM, C.pData, invSC.pData);
    // S^-1 * R 28x28
    arm_mat_init_f32(&invSR, OUTPUT_DIM, OUTPUT_DIM, (float32_t*)invSR_f32);
    mat_solve_with_lu_ptr(OUTPUT_DIM, (float*)SL_f32, (float*)SU_f32, LU_P_f32, OUTPUT_DIM, R.pData, invSR.pData);
    // (S^T)^-1 * C 28x18
    arm_mat_init_f32(&ST, OUTPUT_DIM, OUTPUT_DIM, (float32_t*)ST_f32);
    arm_mat_init_f32(&invSTC, STATE_DIM, OUTPUT_DIM, (float32_t*)invSTC_f32);
    arm_mat_trans_f32(&S, &ST);
    mat_solve_with_lu_ptr(OUTPUT_DIM, (float*)SL_f32, (float*)SU_f32, LU_P_f32, STATE_DIM, C.pData, invSTC.pData);
    use_time[3] = (getTime() - start_time[3]) * 1000;
    // I - Ppri * C^T * S^-1 * C 18x18
    start_time[4] = getTime();
    arm_mat_init_f32(&PCT, STATE_DIM, OUTPUT_DIM, (float32_t*)PCT_f32);
    arm_mat_init_f32(&KC, STATE_DIM, STATE_DIM, (float32_t*)KC_f32);
    arm_mat_init_f32(&I_KC, STATE_DIM, STATE_DIM, (float32_t*)I_KC_f32);
    arm_mat_init_f32(&I18, STATE_DIM, STATE_DIM, (float32_t*)I18_f32);
    arm_mat_mult_f32(&Ppri, &CT, &PCT);
    arm_mat_mult_f32(&PCT, &invSC, &KC);
    arm_mat_sub_f32(&I18, &KC, &I_KC);
    // x = xhat + Ppri * C^T * S^-1 * (y - yhat) 18x1
    arm_mat_mult_f32(&PCT, &invSy, &x);
    arm_mat_add_f32(&x, &xhat, &x);
    memcpy(kf.x, x.pData, sizeof(kf.x));
    // P = (I-KC) * Ppri * (I-KC)^T + Ppri * C^T * S^-1 * R * (S^T)^-1 * C * Ppri^T 18x18
    arm_mat_init_f32(&I_KCPI_KCT, STATE_DIM, STATE_DIM, (float32_t*)I_KCPI_KCT_f32);
    arm_mat_init_f32(&I_KCT, STATE_DIM, STATE_DIM, (float32_t*)I_KCT_f32);
    arm_mat_init_f32(&KR, STATE_DIM, OUTPUT_DIM, (float32_t*)KR_f32);
    arm_mat_init_f32(&KRKT, STATE_DIM, STATE_DIM, (float32_t*)KRKT_f32);
    arm_mat_init_f32(&PpriT, STATE_DIM, STATE_DIM, (float32_t*)PpriT_f32);

    arm_mat_mult_f32(&I_KC, &Ppri, &I_KCPI_KCT);
    arm_mat_trans_f32(&I_KC, &I_KCT);
    arm_mat_mult_f32(&I_KCPI_KCT, &I_KCT, &I_KCPI_KCT);

    arm_mat_mult_f32(&PCT, &invSR, &KR);
    arm_mat_mult_f32(&KR, &invSTC, &KRKT);
    arm_mat_trans_f32(&Ppri, &PpriT);
    arm_mat_mult_f32(&KRKT, &PpriT, &KRKT);
    arm_mat_add_f32(&I_KCPI_KCT, &KRKT, &P);
    memcpy(kf.P, P.pData, sizeof(kf.P));

    // 对速度进行低通滤波
    LPFilter(kf.x[3]);
    LPFilter(kf.x[4]);
    LPFilter(kf.x[5]);
    use_time[4] = (getTime() - start_time[4]) * 1000;
    use_time_all = (getTime() - start_time[0]) * 1000;
}

void estimation_start(void)
{
    kf_start = 1;
}
