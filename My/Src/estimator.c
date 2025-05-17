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
KalmanFilter kf;
ExtendedSensorData sensor_data;

// 一些常量矩阵
float dt = 0.002; // 离散化采样时间，即计算间隔s
float largeVariance = 100; // 大数，用于代替无穷
float I3dt[3][3];
float I3[3][3];
float _I3[3][3];
float I12[12][12];
float I18[18][18];

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
    vec3_cross(sensor_data.gyro, PbfBi, WbxPbfBi);  // 使用vec3_cross替代vec_cross
    leg_get_current_joint_pos(leg_id, joint_pos);
    leg_get_current_joint_vel(leg_id, joint_vel);
    leg_forward_kinematics_vel(leg_id, joint_pos, joint_vel, VbfBi);
    vec_add_ptr(3, WbxPbfBi, VbfBi, Wb_add_Vb);
    mat_mult_vec_ptr(3, 3, &R[0][0], Wb_add_Vb, VsfBi);  // 添加&R[0][0]以获取指针
}

void estimation_init() {
    // 初始化常量矩阵
    for (int i = 0; i < 3; i++) {
        I3[i][i] = 1;
        I3dt[i][i] = dt;
        _I3[i][i] = -1;
    }
    for (int i = 0; i < 12; i++) {
        I12[i][i] = 1;
    }
    for (int i = 0; i < 18; i++) {
        I18[i][i] = 1;
    }
    // 构造离散化状态转移矩阵F = I + dt*A
    //[I3 I3dt 0]
    //[    I3   ]
    //[      I12] 18x18
    memset(kf.F, 0, sizeof(kf.F));
    mat_add_block_ptr(0, 0, 3, 3, 18, &kf.F[0][0], &I3[0][0], &kf.F[0][0]);
    mat_add_block_ptr(0, 3, 3, 3, 18, &kf.F[0][0], &I3dt[0][0], &kf.F[0][0]);
    mat_add_block_ptr(3, 3, 3, 3, 18, &kf.F[0][0], &I3[0][0], &kf.F[0][0]);
    mat_add_block_ptr(6, 6, 12, 12, 18, &kf.F[0][0], &I12[0][0], &kf.F[0][0]);
    // 构造离散化控制矩阵B = dt*B
    memset(kf.B, 0, sizeof(kf.B));
    mat_add_block_ptr(0, 3, 3, 3, 18, &kf.B[0][0], &I3dt[0][0], &kf.B[0][0]);
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
    mat_add_block_ptr(0, 0, 3, 3, 18, &kf.H[0][0], &_I3[0][0], &kf.H[0][0]);
    mat_add_block_ptr(3, 0, 3, 3, 18, &kf.H[0][0], &_I3[0][0], &kf.H[0][0]);
    mat_add_block_ptr(6, 0, 3, 3, 18, &kf.H[0][0], &_I3[0][0], &kf.H[0][0]);
    mat_add_block_ptr(9, 0, 3, 3, 18, &kf.H[0][0], &_I3[0][0], &kf.H[0][0]);
    mat_add_block_ptr(12, 3, 3, 3, 18, &kf.H[0][0], &_I3[0][0], &kf.H[0][0]);
    mat_add_block_ptr(15, 3, 3, 3, 18, &kf.H[0][0], &_I3[0][0], &kf.H[0][0]);
    mat_add_block_ptr(18, 3, 3, 3, 18, &kf.H[0][0], &_I3[0][0], &kf.H[0][0]);
    mat_add_block_ptr(21, 3, 3, 3, 18, &kf.H[0][0], &_I3[0][0], &kf.H[0][0]);
    mat_add_block_ptr(0, 6, 12, 12, 18, &kf.H[0][0], &I12[0][0], &kf.H[0][0]);
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
    mat_mult_ptr(18, 3, 18, &BCu[0][0], &BT[0][0], &kf.Q[0][0]);
    mat_add_ptr(18, 18, &Qdig[0][0], &kf.Q[0][0], &kf.Q[0][0]);
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
    memcpy(kf.R, R, sizeof(kf.R));
    
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
    float Rot[3][3] = {0};
    float u[3] = {0};
    float feetPos[4][3] = {0};
    float feetVel[4][3] = {0};
    float feetZ[4] = {0};
    float xhat[STATE_DIM] = {0};
    float yhat[OUTPUT_DIM] = {0};
    float Ppri[STATE_DIM][STATE_DIM] = {0};
    float PpriT[STATE_DIM][STATE_DIM] = {0};
    float AP[STATE_DIM][STATE_DIM] = {0};
    float AT[STATE_DIM][STATE_DIM] = {0};
    float CT[STATE_DIM][OUTPUT_DIM] = {0};
    float CP[OUTPUT_DIM][STATE_DIM] = {0};
    float S[OUTPUT_DIM][OUTPUT_DIM] = {0};
    float ST[OUTPUT_DIM][OUTPUT_DIM] = {0};
    float SL[OUTPUT_DIM][OUTPUT_DIM] = {0};
    float SU[OUTPUT_DIM][OUTPUT_DIM] = {0};
    int LU_P[OUTPUT_DIM] = {0};
    float invSy[OUTPUT_DIM] = {0};
    float y_yhat[OUTPUT_DIM] = {0};
    float invSC[OUTPUT_DIM][STATE_DIM] = {0};
    float invSR[OUTPUT_DIM][OUTPUT_DIM] = {0};
    float invSTC[STATE_DIM][OUTPUT_DIM] = {0};
    float KC[STATE_DIM][STATE_DIM] = {0};
    float I_KC[STATE_DIM][STATE_DIM] = {0};
    float PCT[STATE_DIM][OUTPUT_DIM] = {0};
    float I_KCT[STATE_DIM][STATE_DIM] = {0};
    float I_KCPI_KCT[STATE_DIM][STATE_DIM] = {0};
    float KR[STATE_DIM][OUTPUT_DIM] = {0};
    float KRKT[STATE_DIM][STATE_DIM] = {0};
void estimation_run() {
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
                kf.Q[6+3*i+j][6+3*i+j] = (1 + (1-trust)*largeVariance) * kf.Q[6+3*i+j][6+3*i+j];
                kf.R[12+3*i+j][12+3*i+j] = (1 + (1-trust)*largeVariance) * kf.R[12+3*i+j][12+3*i+j];
            }
            kf.R[24+i][24+i] = (1 + (1-trust)*largeVariance) * kf.R[24+i][24+i];
        }
    }
    /* -----------预测部分------------- */
    // R
    quaternion_to_rotation_matrix_zyx(sensor_data.quaternion, Rot);
    // u = R * acc + g
    body_to_world_acc(Rot, sensor_data.acc, u);
    // xhat = A * x + B * u
    mat_mult_vec_ptr(STATE_DIM, STATE_DIM, &kf.F[0][0], kf.x, xhat);
    vec_add_ptr(STATE_DIM, xhat, u, xhat);
    // yhat = C * x
    mat_mult_vec_ptr(OUTPUT_DIM, STATE_DIM, &kf.H[0][0], kf.x, yhat);
    // Ppri = A * P * A^T + Q
    mat_mult_ptr(STATE_DIM, STATE_DIM, STATE_DIM, &kf.F[0][0], &kf.P[0][0], &AP[0][0]);
    mat_transpose_ptr(STATE_DIM, STATE_DIM, &kf.F[0][0], &AT[0][0]);
    mat_mult_ptr(STATE_DIM, STATE_DIM, STATE_DIM, &AP[0][0], &AT[0][0], &Ppri[0][0]);
    mat_add_ptr(STATE_DIM, STATE_DIM, &Ppri[0][0], &kf.Q[0][0], &Ppri[0][0]);

    /* -----------更新部分------------- */
    // y 28x1
    for(int i = 0; i < 4; i ++){
        world_foot_to_body_pos(i, Rot, feetPos[i]);
        world_foot_to_body_vel(i, Rot, feetVel[i]);
        feetZ[i] = 0;
        for(int j = 0; j < 3; j ++){
            kf.y[3*i+j] = feetPos[i][j];
            kf.y[3*i+12+j] = feetVel[i][j];
        }
        kf.y[24+i] = feetZ[i];
    }
    // S = R + C * Ppri * C^T 28x28
    mat_mult_ptr(OUTPUT_DIM, STATE_DIM, STATE_DIM, &kf.H[0][0], &Ppri[0][0], &CP[0][0]);
    mat_transpose_ptr(OUTPUT_DIM, STATE_DIM, &kf.H[0][0], &CT[0][0]);
    mat_mult_ptr(OUTPUT_DIM, STATE_DIM, OUTPUT_DIM, &CP[0][0], &CT[0][0], &S[0][0]);
    mat_add_ptr(OUTPUT_DIM, OUTPUT_DIM, &S[0][0], &kf.R[0][0], &S[0][0]);
    // S^-1(y - yhat) 28x1
    vec_sub_ptr(OUTPUT_DIM, kf.y, yhat, y_yhat);
    mat_lu_decomp_ptr(OUTPUT_DIM, &S[0][0], &SL[0][0], &SU[0][0], LU_P);
    mat_solve_with_lu_ptr(OUTPUT_DIM, &SL[0][0], &SU[0][0], LU_P, 1, y_yhat, invSy);
    // S^-1 * C 28x18
    mat_solve_with_lu_ptr(OUTPUT_DIM, &SL[0][0], &SU[0][0], LU_P, STATE_DIM, &kf.H[0][0], &invSC[0][0]);
    // S^-1 * R 28x28
    mat_solve_with_lu_ptr(OUTPUT_DIM, &SL[0][0], &SU[0][0], LU_P, OUTPUT_DIM, &kf.R[0][0], &invSR[0][0]);
    // (S^T)^-1 * C 28x18
    mat_transpose_ptr(OUTPUT_DIM, OUTPUT_DIM, &S[0][0], &ST[0][0]);
    mat_solve_with_lu_ptr(OUTPUT_DIM, &SL[0][0], &SU[0][0], LU_P, STATE_DIM, &kf.H[0][0], &invSTC[0][0]);

    // I - Ppri * C^T * S^-1 * C 18x18
    mat_mult_ptr(STATE_DIM, STATE_DIM, OUTPUT_DIM, &Ppri[0][0], &CT[0][0], &PCT[0][0]);
    mat_mult_ptr(STATE_DIM, OUTPUT_DIM, STATE_DIM, &PCT[0][0], &invSC[0][0], &KC[0][0]);
    mat_sub_ptr(STATE_DIM, STATE_DIM, &I18[0][0], &KC[0][0], &I_KC[0][0]);
    // x = xhat + Ppri * C^T * S^-1 * (y - yhat) 18x1
    mat_mult_ptr(STATE_DIM, OUTPUT_DIM, 1, &PCT[0][0], &invSy[0], &kf.x[0]);
    vec_add_ptr(STATE_DIM, xhat, &kf.x[0], &kf.x[0]);
    // P = (I-KC) * Ppri * (I-KC)^T + Ppri * C^T * S^-1 * R * (S^T)^-1 * C * Ppri^T 18x18
    mat_mult_ptr(STATE_DIM, STATE_DIM, STATE_DIM, &I_KC[0][0], &Ppri[0][0], &I_KCPI_KCT[0][0]);
    mat_mult_ptr(STATE_DIM, STATE_DIM, STATE_DIM, &I_KCPI_KCT[0][0], &I_KCT[0][0], &I_KCPI_KCT[0][0]);
    mat_mult_ptr(STATE_DIM, OUTPUT_DIM, OUTPUT_DIM, &PCT[0][0], &invSR[0][0], &KR[0][0]);
    mat_mult_ptr(STATE_DIM, OUTPUT_DIM, STATE_DIM, &KR[0][0], &invSTC[0][0], &KRKT[0][0]);
    mat_transpose_ptr(STATE_DIM, STATE_DIM, &Ppri[0][0], &PpriT[0][0]);
    mat_mult_ptr(STATE_DIM, STATE_DIM, STATE_DIM, &KRKT[0][0], &PpriT[0][0], &KRKT[0][0]);
    mat_add_ptr(STATE_DIM, STATE_DIM, &I_KCPI_KCT[0][0], &KRKT[0][0], &kf.P[0][0]);

    // 对速度进行低通滤波
    LPFilter(xhat[3]);
    LPFilter(xhat[4]);
    LPFilter(xhat[5]);
}
