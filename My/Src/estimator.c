/************************************************************
 * 四足机器人卡尔曼滤波器状态估计器-只考虑机身xy速度的极致简化版本，放在单片机上可以运行
 * 状态向量x: [pb_x, pb_y, vb_x, vb_y]            // 4维机身位置和速度
 * 测量向量y: [vsfB_0x, vsfB_0y ..., vsfB_3y]     // 8维世界坐标系下足端到机身速度
 * 控制向量u: [asb, asy]                          // 3维机身加速度和重力加速度
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
#include "ANO_TC.h"
KalmanFilter kf;

float kf_start = 0;

// 一些常量矩阵
float dt = 0.003; // 离散化采样时间，即计算间隔s
float largeVariance = 100; // 大数，用于代替无穷
float I3dt_f32[3][3] = {0};
float I3_f32[3][3] = {0};
float _I3_f32[3][3] = {0};
float I12_f32[12][12] = {0};
float I18_f32[18][18] = {0};

s_LPFilter lpf_x, lpf_y;

// 低通滤波器
void LPFilter_init(s_LPFilter* lpf, float samplePeriod, float cutFrequency){
    lpf->weight = 1.0f / ( 1.0f + 1.0f/(2.0f * PI * samplePeriod * cutFrequency) );
    lpf->start  = 0;
}

void LPFilter(s_LPFilter* lpf, float newValue){
    if(!lpf->start){
        lpf->start = 1;
        lpf->pastValue = newValue;
    }
    lpf->pastValue = lpf->weight*newValue + (1-lpf->weight)*lpf->pastValue;
}

float LPF_get_value(s_LPFilter* lpf){
    return lpf->pastValue;
}

void LPF_clear(s_LPFilter* lpf){
    lpf->start = 0;
    lpf->pastValue = 0;
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

float time_cal = 0;
float time_cal_start = 0;
// 计算世界坐标系下足端相对于机身的速度
void world_foot_to_body_vel(uint8_t leg_id, float R[3][3], float VsfBi[3]) {
    float PbfBi[3];
    float WbxPbfBi[3];
    float VbfBi[3];
    float Wb_add_Vb[3];
    float joint_pos[3];
    float joint_vel[3];
    
    time_cal_start = getTime();
    leg_get_current_foot_pos_body(leg_id, PbfBi);
    time_cal = (getTime() - time_cal_start) * 1000;
    vec3_cross(imu_get_data()->gyro, PbfBi, WbxPbfBi);  // 使用vec3_cross替代vec_cross
    
    leg_get_current_joint_pos(leg_id, joint_pos);
    leg_get_current_joint_vel(leg_id, joint_vel);
    
    leg_forward_kinematics_vel(leg_id, joint_pos, joint_vel, VbfBi);
    //不考虑旋转角速度
    memcpy(VsfBi, VbfBi, sizeof(VbfBi));
//    vec_add_ptr(3, WbxPbfBi, VbfBi, Wb_add_Vb);
//    
//    mat_mult_vec_ptr(3, 3, &R[0][0], Wb_add_Vb, VsfBi);  // 添加&R[0][0]以获取指针
    
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
    //[I2 I2dt]
    //[0   I2 ] 4x4
    memset(kf.F, 0, sizeof(kf.F));
    for(int i = 0; i < 4; i ++){
        kf.F[i][i] = 1;
    }
    kf.F[0][2] = dt;
    kf.F[1][3] = dt;
    // 构造离散化控制矩阵B = dt*B
    //[ 0  ]
    //[I2dt] 4x2
    memset(kf.B, 0, sizeof(kf.B));
    kf.B[2][0] = dt;
    kf.B[3][1] = dt;
    // 构造离散化观测矩阵H = C
    // [0 -I2]
    // [0 -I2]
    // [0 -I2]
    // [0 -I2] 8x4
    memset(kf.H, 0, sizeof(kf.H));
    for(int i = 0; i < 4; i ++){
        kf.H[i*2][2] = -1;
        kf.H[i*2+1][3] = -1;
    }
    // 控制矩阵的噪声协方差矩阵，一般直接测出来。这里用的宇树程序里的值，如果效果不好需要自己重测
    float Cu[CTRL_DIM][CTRL_DIM] = {
        268.573,  -43.819,
        -43.819 ,  92.949 
    };
    // 状态变量的噪声协方差，前六个方差较小，后12个因为足端触底有抖动，因此方差较大
    float Qdig[STATE_DIM][STATE_DIM] = {0};
    for (int i = 0; i < STATE_DIM; i++) {
        if (i < 2) // Q越大速度抖动越大，Q越小越平滑越滞后
            Qdig[i][i] = 0.5; // 位置估计协方差，没用
        else if(i == 2) // vx
            Qdig[i][i] = 2;
        else if(i == 3) // vy，发现vy收敛比vx慢，所以vy对应的协方差的大一点
            Qdig[i][i] = 4;
            
    }
    // 构造过程噪声协方差矩阵Q = Qdig + B * Cu * B^T
    float BCu[STATE_DIM][CTRL_DIM] = {0};
    float BT[CTRL_DIM][STATE_DIM] = {0};
    mat_mult_ptr(STATE_DIM, CTRL_DIM, CTRL_DIM, &kf.B[0][0], &Cu[0][0], &BCu[0][0]);
    mat_transpose_ptr(STATE_DIM, CTRL_DIM, &kf.B[0][0], &BT[0][0]);
    mat_mult_ptr(STATE_DIM, CTRL_DIM, STATE_DIM, &BCu[0][0], &BT[0][0], &kf.Q_init[0][0]);
    mat_add_ptr(STATE_DIM, STATE_DIM, &Qdig[0][0], &kf.Q_init[0][0], &kf.Q_init[0][0]);
    memcpy(kf.Q, kf.Q_init, sizeof(kf.Q));
    // 测量噪声协方差矩阵R - 简化版只包含平面速度，用的宇树的数据，效果不好可能需要自己测量
    float R[OUTPUT_DIM][OUTPUT_DIM] = {
        // VsfB0_x, VsfB0_y, VsfB1_x, VsfB1_y, VsfB2_x, VsfB2_y, VsfB3_x, VsfB3_y
        {1.708, 0.048, 0.062, 0.042, 0.077, 0.001, 0.046, -0.019}, // VsfB0_x
        {0.048, 5.001, -0.036, 0.144, 0.036, 0.016, -0.067, -0.024}, // VsfB0_y
        {0.062, -0.036, 6.228, -0.014, 0.059, 0.053, 0.148, 0.015}, // VsfB1_x
        {0.042, 0.144, -0.014, 3.011, 0.076, 0.030, -0.027, 0.057}, // VsfB1_y
        {0.077, 0.036, 0.059, 0.076, 6.230, 0.139, 0.013, -0.019}, // VsfB2_x
        {0.001, 0.016, 0.053, 0.030, 0.139, 3.130, -0.010, 0.131}, // VsfB2_y
        {0.046, -0.067, 0.148, -0.027, 0.013, -0.010, 2.437, -0.102}, // VsfB3_x
        {-0.019, -0.024, 0.015, 0.057, -0.019, 0.131, -0.102, 4.944}  // VsfB3_y
    };
    memcpy(kf.R_init, R, sizeof(kf.R_init));
    memcpy(kf.R, kf.R_init, sizeof(kf.R));
    
    // 初始化协方差矩阵P
    memset(kf.P, 0, sizeof(kf.P));
    for (int i = 0; i < STATE_DIM; i++) {
        kf.P[i][i] = largeVariance;
    }
    // 初始化状态变量x
    memset(kf.x, 0, sizeof(kf.x));
    // 初始化低通滤波器
    LPFilter_init(&lpf_x, dt, 3.0);
    LPFilter_init(&lpf_y, dt, 3.0);
}

// 窗口函数，用于降低触底抖动对状态估计的影响
float windowFunc(float x, float windowRatio){
    float xRange = 1;
    float yRange = 1;
    if((x < 0)||(x > xRange)){
        // printf("The x=%f, which should between [0, xRange]\n", x);
    }
    if((windowRatio <= 0)||(windowRatio >= 0.5f)){
        // printf("The windowRatio=%f, which should between [0, 0.5]\n", windowRatio);
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
float CPCT_f32[OUTPUT_DIM][OUTPUT_DIM] = {0};
float RCPCT_f32[OUTPUT_DIM][OUTPUT_DIM] = {0};
float invRCPCT_f32[OUTPUT_DIM][OUTPUT_DIM] = {0};
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
float K_f32[STATE_DIM][OUTPUT_DIM] = {0};
float KT_f32[OUTPUT_DIM][STATE_DIM] = {0};
float KC_f32[STATE_DIM][STATE_DIM] = {0};
float I_KC_f32[STATE_DIM][STATE_DIM] = {0};
float PCT_f32[STATE_DIM][OUTPUT_DIM] = {0};
float I_KCT_f32[STATE_DIM][STATE_DIM] = {0};
float I_KCPI_KCT_f32[STATE_DIM][STATE_DIM] = {0};
float KR_f32[STATE_DIM][OUTPUT_DIM] = {0};
float KRKT_f32[STATE_DIM][STATE_DIM] = {0};

arm_matrix_instance_f32 I18; 
arm_matrix_instance_f32 A; 
arm_matrix_instance_f32 x; 
arm_matrix_instance_f32 B; 
arm_matrix_instance_f32 u;
arm_matrix_instance_f32 C;
arm_matrix_instance_f32 y;
arm_matrix_instance_f32 xhat;
arm_matrix_instance_f32 yhat;
arm_matrix_instance_f32 Ax;
arm_matrix_instance_f32 Bu;
arm_matrix_instance_f32 Ppri;
arm_matrix_instance_f32 PpriT;
arm_matrix_instance_f32 AP;
arm_matrix_instance_f32 AT;
arm_matrix_instance_f32 APAT;
arm_matrix_instance_f32 P;
arm_matrix_instance_f32 Q;
arm_matrix_instance_f32 CP;
arm_matrix_instance_f32 CT;
arm_matrix_instance_f32 PCT;
arm_matrix_instance_f32 CPCT;
arm_matrix_instance_f32 RCPCT;
arm_matrix_instance_f32 invRCPCT;
arm_matrix_instance_f32 R;
arm_matrix_instance_f32 S;
arm_matrix_instance_f32 ST;
arm_matrix_instance_f32 y_yhat;
arm_matrix_instance_f32 invSy;
arm_matrix_instance_f32 invSC;
arm_matrix_instance_f32 invSR;
arm_matrix_instance_f32 invSTC;
arm_matrix_instance_f32 K;
arm_matrix_instance_f32 KT;
arm_matrix_instance_f32 KC;
arm_matrix_instance_f32 I_KC;
arm_matrix_instance_f32 I_KCT;
arm_matrix_instance_f32 I_KCPI_KCT;
arm_matrix_instance_f32 KR;
arm_matrix_instance_f32 KRKT;

float start_time[5];
float use_time[5]; 
float use_time_all = 0;
float use_time_greater_than_3 = 0;
arm_status status;
void estimation_run() {
    if(kf_start == 0){
        return;
    }
    start_time[0] = getTime();
    
    /* -----------根据触底状态更新协方差矩阵------------- */
    for(int i = 0; i < 4; ++i){
        if(leg_get_contact_state(i) == 0){
            for (int j = 0; j < 2; j ++) // 足端悬空时不信任测量值
            {
                kf.R[2*i+j][2*i+j] = largeVariance;
            }
        }
        else{ //完全触地时信任测量值
            float trust = windowFunc(leg_get_phase(i), 0.2); // 使用窗口函数降低触地时的噪声
            if (i == 0)
                set_debug_data(3, trust);
            for (int j = 0; j < 2; j ++)
            {
                kf.R[2*i+j][2*i+j] = (1 + (1-trust)*largeVariance) * kf.R_init[2*i+j][2*i+j];
            }
        }
//        if (i == 0)
            // ano_data[2] = leg_get_contact_state(i);
    }
    use_time[0] = (getTime() - start_time[0]) * 1000;
    /* -----------预测部分------------- */
    // R
    // quaternion_to_rotation_matrix_zyx(imu_get_data()->quaternion, Rot_f32);
    // 狗运动时角速度波动较大，转换到世界坐标系波动很大，这里直接使用单位阵代替旋转矩阵，计算出的vxvy为机体坐标系下的速度
    memset(Rot_f32, 0, sizeof(Rot_f32));
    for(int i = 0; i < 3; i ++){
        Rot_f32[i][i] = 1;
    }
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
    arm_mat_mult_f32(&AP, &AT, &APAT);
    //mat_mult_18x18_optimized(AP.pData, AT.pData, APAT.pData);
//    mat_mult_ptr(STATE_DIM, STATE_DIM, STATE_DIM, AP.pData, AT.pData, APAT.pData);
    use_time[1] = (getTime() - start_time[1]) * 1000;
    arm_mat_add_f32(&APAT, &Q, &Ppri);
    
    /* -----------更新部分------------- */
    
    // y 28x1
    for(int i = 0; i < 4; i ++){
        start_time[2] = getTime();
        world_foot_to_body_vel(i, Rot_f32, feetVel_f32[i]);
        use_time[2] = (getTime() - start_time[2]) * 1000;
        for(int j = 0; j < 2; j ++){
            kf.y[2*i+j] = feetVel_f32[i][j];
        }
    }
    set_debug_data(0, kf.y[0]);
    set_debug_data(1, kf.y[1]);

    set_debug_data(2, imu_get_data()->gyro[2]);
    start_time[3] = getTime();
    // 直接求R+CPC^T的逆
    // (R + CPC^T)^-1
    arm_mat_init_f32(&CP, OUTPUT_DIM, STATE_DIM, (float32_t*)CP_f32);
    arm_mat_init_f32(&CT, STATE_DIM, OUTPUT_DIM, (float32_t*)CT_f32);
    arm_mat_init_f32(&CPCT, OUTPUT_DIM, OUTPUT_DIM, (float32_t*)CPCT_f32);
    arm_mat_init_f32(&R, OUTPUT_DIM, OUTPUT_DIM, (float32_t*)kf.R);
    arm_mat_init_f32(&invRCPCT, OUTPUT_DIM, OUTPUT_DIM, (float32_t*)invRCPCT_f32);
    arm_mat_init_f32(&RCPCT, OUTPUT_DIM, OUTPUT_DIM, (float32_t*)RCPCT_f32);

    arm_mat_mult_f32(&C, &Ppri, &CP);
    arm_mat_trans_f32(&C, &CT);
    arm_mat_mult_f32(&CP, &CT, &CPCT);
    arm_mat_add_f32(&CPCT, &R, &RCPCT);
    status = arm_mat_inverse_f32(&RCPCT, &invRCPCT);
    // K = PC^T * (R + CPC^T)^-1
    arm_mat_init_f32(&PCT, STATE_DIM, OUTPUT_DIM, (float32_t*)PCT_f32);
    arm_mat_init_f32(&K, STATE_DIM, OUTPUT_DIM, (float32_t*)K_f32);

    arm_mat_mult_f32(&Ppri, &CT, &PCT);
    arm_mat_mult_f32(&PCT, &invRCPCT, &K);
    
    use_time[3] = (getTime() - start_time[3]) * 1000;
    
    start_time[4] = getTime();
    // x = xhat + K * (y - yhat)
    arm_mat_init_f32(&y, OUTPUT_DIM, 1, (float32_t*)kf.y);
    arm_mat_init_f32(&yhat, OUTPUT_DIM, 1, (float32_t*)yhat_f32);
    arm_mat_init_f32(&y_yhat, OUTPUT_DIM, 1, (float32_t*)y_yhat_f32);

    arm_mat_sub_f32(&y, &yhat, &y_yhat);
    arm_mat_mult_f32(&K, &y_yhat, &x);
    arm_mat_add_f32(&x, &xhat, &x);

    if (x.pData[0] > 100) // 算出来nan
    {
        memset(x.pData, 0, sizeof(x.pData));
        memset(kf.x, 0, sizeof(kf.x));
    }
    else
        memcpy(kf.x, x.pData, sizeof(kf.x));

   
    
    // 测量噪声小的情况下，可以考虑使用简化版本的更新P = (I - KC) * Ppri，保证P的对称性即可
    // P = (I - KC) * Ppri * (I - KC)^T + K * R * K^T
    arm_mat_init_f32(&I_KC, STATE_DIM, STATE_DIM, (float32_t*)I_KC_f32);
    arm_mat_init_f32(&KC, STATE_DIM, STATE_DIM, (float32_t*)KC_f32);
    arm_mat_init_f32(&KR, STATE_DIM, OUTPUT_DIM, (float32_t*)KR_f32);
    arm_mat_init_f32(&KRKT, STATE_DIM, STATE_DIM, (float32_t*)KRKT_f32);
    arm_mat_init_f32(&I18, STATE_DIM, STATE_DIM, (float32_t*)I18_f32);
    arm_mat_init_f32(&I_KCPI_KCT, STATE_DIM, STATE_DIM, (float32_t*)I_KCPI_KCT_f32);
    arm_mat_init_f32(&I_KCT, STATE_DIM, STATE_DIM, (float32_t*)I_KCT_f32);
    arm_mat_init_f32(&KT, OUTPUT_DIM, STATE_DIM, (float32_t*)KT_f32);

    arm_mat_mult_f32(&K, &C, &KC);
    arm_mat_sub_f32(&I18, &KC, &I_KC);
    arm_mat_mult_f32(&I_KC, &Ppri, &I_KCPI_KCT);
    arm_mat_trans_f32(&I_KC, &I_KCT);
    arm_mat_mult_f32(&I_KCPI_KCT, &I_KCT, &I_KCPI_KCT);
    
    arm_mat_mult_f32(&K, &R, &KR);
    arm_mat_trans_f32(&K, &KT);
    arm_mat_mult_f32(&KR, &KT, &KRKT);
    arm_mat_add_f32(&I_KCPI_KCT, &KRKT, &P);
    memcpy(kf.P, P.pData, sizeof(kf.P));
//    // S = R + C * Ppri * C^T 28x28
//    arm_mat_init_f32(&CP, OUTPUT_DIM, STATE_DIM, (float32_t*)CP_f32);
//    arm_mat_init_f32(&CT, STATE_DIM, OUTPUT_DIM, (float32_t*)CT_f32);
//    arm_mat_init_f32(&R, OUTPUT_DIM, OUTPUT_DIM, (float32_t*)kf.R);
//    arm_mat_init_f32(&S, OUTPUT_DIM, OUTPUT_DIM, (float32_t*)S_f32);

//    arm_mat_mult_f32(&C, &Ppri, &CP);
//    arm_mat_trans_f32(&C, &CT);
//    arm_mat_mult_f32(&CP, &CT, &S);
//    arm_mat_add_f32(&S, &R, &S);
//    
//    // S^-1(y - yhat) 28x1
//    start_time[3] = getTime();
//    arm_mat_init_f32(&y, OUTPUT_DIM, 1, (float32_t*)kf.y);
//    arm_mat_init_f32(&y_yhat, OUTPUT_DIM, 1, (float32_t*)y_yhat_f32);
//    arm_mat_init_f32(&invSy, OUTPUT_DIM, 1, (float32_t*)invSy_f32);

//    arm_mat_sub_f32(&y, &yhat, &y_yhat);
//    mat_lu_decomp_ptr(OUTPUT_DIM, S.pData, (float*)SL_f32, (float*)SU_f32, LU_P_f32);
//    mat_solve_with_lu_ptr(OUTPUT_DIM, (float*)SL_f32, (float*)SU_f32, LU_P_f32, 1, y_yhat.pData, invSy.pData);
    
    
//    // S^-1 * C 28x18
//    arm_mat_init_f32(&invSC, OUTPUT_DIM, STATE_DIM, (float32_t*)invSC_f32);
//    mat_solve_with_lu_ptr(OUTPUT_DIM, (float*)SL_f32, (float*)SU_f32, LU_P_f32, STATE_DIM, C.pData, invSC.pData);
//    // S^-1 * R 28x28
//    arm_mat_init_f32(&invSR, OUTPUT_DIM, OUTPUT_DIM, (float32_t*)invSR_f32);
//    mat_solve_with_lu_ptr(OUTPUT_DIM, (float*)SL_f32, (float*)SU_f32, LU_P_f32, OUTPUT_DIM, R.pData, invSR.pData);
//    // (S^T)^-1 * C 28x18
//    arm_mat_init_f32(&ST, OUTPUT_DIM, OUTPUT_DIM, (float32_t*)ST_f32);
//    arm_mat_init_f32(&invSTC, STATE_DIM, OUTPUT_DIM, (float32_t*)invSTC_f32);
//    arm_mat_trans_f32(&S, &ST);
//    mat_solve_with_lu_ptr(OUTPUT_DIM, (float*)SL_f32, (float*)SU_f32, LU_P_f32, STATE_DIM, C.pData, invSTC.pData);
//    use_time[3] = (getTime() - start_time[3]) * 1000;
//    // I - Ppri * C^T * S^-1 * C 18x18
//    start_time[4] = getTime();
//    arm_mat_init_f32(&PCT, STATE_DIM, OUTPUT_DIM, (float32_t*)PCT_f32);
//    arm_mat_init_f32(&KC, STATE_DIM, STATE_DIM, (float32_t*)KC_f32);
//    arm_mat_init_f32(&I_KC, STATE_DIM, STATE_DIM, (float32_t*)I_KC_f32);
//    arm_mat_init_f32(&I18, STATE_DIM, STATE_DIM, (float32_t*)I18_f32);
//    arm_mat_mult_f32(&Ppri, &CT, &PCT);
//    arm_mat_mult_f32(&PCT, &invSC, &KC);
//    arm_mat_sub_f32(&I18, &KC, &I_KC);
//    // x = xhat + Ppri * C^T * S^-1 * (y - yhat) 18x1
//    arm_mat_mult_f32(&PCT, &invSy, &x);
//    arm_mat_add_f32(&x, &xhat, &x);
//    memcpy(kf.x, x.pData, sizeof(kf.x));
//    // P = (I-KC) * Ppri * (I-KC)^T + Ppri * C^T * S^-1 * R * (S^T)^-1 * C * Ppri^T 18x18
//    arm_mat_init_f32(&I_KCPI_KCT, STATE_DIM, STATE_DIM, (float32_t*)I_KCPI_KCT_f32);
//    arm_mat_init_f32(&I_KCT, STATE_DIM, STATE_DIM, (float32_t*)I_KCT_f32);
//    arm_mat_init_f32(&KR, STATE_DIM, OUTPUT_DIM, (float32_t*)KR_f32);
//    arm_mat_init_f32(&KRKT, STATE_DIM, STATE_DIM, (float32_t*)KRKT_f32);
//    arm_mat_init_f32(&PpriT, STATE_DIM, STATE_DIM, (float32_t*)PpriT_f32);

//    arm_mat_mult_f32(&I_KC, &Ppri, &I_KCPI_KCT);
//    arm_mat_trans_f32(&I_KC, &I_KCT);
//    arm_mat_mult_f32(&I_KCPI_KCT, &I_KCT, &I_KCPI_KCT);

//    arm_mat_mult_f32(&PCT, &invSR, &KR);
//    arm_mat_mult_f32(&KR, &invSTC, &KRKT);
//    arm_mat_trans_f32(&Ppri, &PpriT);
//    arm_mat_mult_f32(&KRKT, &PpriT, &KRKT);
//    arm_mat_add_f32(&I_KCPI_KCT, &KRKT, &P);
//    memcpy(kf.P, P.pData, sizeof(kf.P));

    // 对速度进行低通滤波
    LPFilter(&lpf_x, kf.x[2]);
    LPFilter(&lpf_y, kf.x[3]);
    use_time[4] = (getTime() - start_time[4]) * 1000;
    use_time_all = (getTime() - start_time[0]) * 1000;
    if (use_time_all > 3)
        use_time_greater_than_3 = 1;
    set_debug_data(4, LPF_get_value(&lpf_x));
    set_debug_data(5, LPF_get_value(&lpf_y));
}

void estimation_start(void)
{
    kf_start = 1;
}
