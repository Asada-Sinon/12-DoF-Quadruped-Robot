/**
 * @file dog.c
 * @brief 四足机器人运动学和控制实现
 */

/*========================= 头文件包含 =========================*/
/* 标准库 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

/* 自定义头文件 */
#include "dog.h"
#include "motor.h"
#include "gait.h"
#include "estimator.h"
#include "matrix.h"
#include "timer.h"
/*========================= 全局变量 =========================*/
Dog dog;                    // 狗实例
float body_vel[3] = {0, 0, 0}; // 机体速度

/*========================= 类型定义 =========================*/
// 坐标系转换方向枚举
typedef enum {
    JOINT_TO_MOTOR = 0,  // 关节坐标系转换为电机坐标系
    MOTOR_TO_JOINT = 1   // 电机坐标系转换为关节坐标系
} TransformDirection;


/*========================= 运动学函数 =========================*/
/* https://github.com/unitreerobotics/unitree_guide.git 
 * 腿坐标系定义（右手系）:
 *
 *            ↑ z
 *            |
 *            |
 *  x ←———————|
 *            /
 *           /
 *          /
 *         y
 *  
 * 注：原点在大腿关节轴线与髋关节轴线的交点上，固连在机体上。x轴向狗头方向，z轴向上，按右手定则确定y轴方向。
 * 注：左侧腿FL，HL 的y轴向身体外侧；右侧腿FR，HR的y轴向身体内侧。
 */

float sin_time_start = 0;
float sin_time = 0;

void leg_forward_kinematics(uint8_t leg_id, const float joint_pos[3], float foot_pos[3])
{
    const LegLinkParams *leg_links = &dog.params.leg_links;
    int sign = 1;
    if (leg_id == LEG_FR || leg_id == LEG_HR) {
        sign = -1;
    }
    
    // 转换连杆长度，适应坐标系方向
    float l1 = sign * leg_links->hip_length;
    float l2 = -leg_links->thigh_length;
    float l3 = -leg_links->calf_length;

    // 计算三角函数值
    
    float s1 = sinf(joint_pos[HIP_IDX]);
    float s2 = sinf(joint_pos[THIGH_IDX]);
    float s3 = sinf(joint_pos[CALF_IDX]);

    float c1 = cosf(joint_pos[HIP_IDX]);
    float c2 = cosf(joint_pos[THIGH_IDX]);
    float c3 = cosf(joint_pos[CALF_IDX]);
    
    // 计算复合角的三角函数值
    float c23 = c2 * c3 - s2 * s3;
    float s23 = s2 * c3 + c2 * s3;

    // 应用运动学正解方程
    foot_pos[X_IDX] = l3 * s23 + l2 * s2;
    foot_pos[Y_IDX] = -l3 * s1 * c23 + l1 * c1 - l2 * c2 * s1;
    foot_pos[Z_IDX] =  l3 * c1 * c23 + l1 * s1 + l2 * c1 * c2;
    sin_time_start = getTime();
    // printf("foot_pos[%d]: %f %f %f\n", leg_id, foot_pos[X_IDX], foot_pos[Y_IDX], foot_pos[Z_IDX]);
    sin_time = (getTime() - sin_time_start ) * 1000;
}

void leg_inverse_kinematics(uint8_t leg_id, const float foot_pos[3], float joint_pos[3])
{
    const LegLinkParams *leg_links = &dog.params.leg_links;
    
    // 确定髋关节Y方向符号（左腿和右腿方向相反）
    int y_sign = (leg_id == LEG_FL || leg_id == LEG_HL) ? 1 : -1;
    
    // 提取足端位置坐标
    float px = foot_pos[X_IDX];
    float py = foot_pos[Y_IDX];
    float pz = foot_pos[Z_IDX];
    
    // 转换连杆长度，适应坐标系方向
    float l1 = y_sign * leg_links->hip_length;  // 髋关节偏移（考虑左右方向）
    float l2 = -leg_links->thigh_length;        // 大腿长度
    float l3 = -leg_links->calf_length;         // 小腿长度

    /*计算髋关节角度*/
    // 大腿连杆加小腿连杆在髋腿坐标系下yz平面的投影长度
    float L1 = sqrtf(py*py + pz*pz - l1*l1); 
    // 髋关节角度 - 使用atan2确保正确的象限
    float hip_angle = atan2f(pz*l1+py*L1, py*l1-pz*L1);
    
    /*计算小腿关节角度*/
    // 计算足端与大腿关节的距离
    float L2 = sqrtf(px*px + py*py + pz*pz - l1*l1);  
    // 应用余弦定理，确保参数在arccos的定义域[-1,1]内
    float temp = (l2*l2 + l3*l3 - L2*L2)/(2*fabs(l2*l3));
    if(temp>1) temp = 1;
    if(temp<-1) temp = -1;
    // 小腿关节角度 - 首先计算为[0,π]范围，然后转换到关节约束范围[-π,0]
    float calf_angle = acosf(temp); //[0, PI]
    calf_angle = -(PI - calf_angle); //[-PI, 0]

    /*计算大腿关节角度*/
    // 通过几何关系计算参数
    float a1 = py*sin(hip_angle) - pz*cos(hip_angle);
    float a2 = px;
    float m1 = l3*sin(calf_angle);
    float m2 = l2 + l3*cos(calf_angle);
    // 大腿关节角度 - 使用atan2确保正确的象限
    float thigh_angle = atan2f(m1*a1+m2*a2, m1*a2-m2*a1);
    
    // 存储计算结果
    joint_pos[HIP_IDX] = hip_angle;
    joint_pos[THIGH_IDX] = thigh_angle;
    joint_pos[CALF_IDX] = calf_angle;
}

// 计算雅克比矩阵
void leg_jacobian(uint8_t leg_id, const float joint_pos[3], float jacobian[3][3])
{
    const LegLinkParams *leg_links = &dog.params.leg_links;

    // 确定髋关节Y方向符号（左腿和右腿方向相反）
    int y_sign = (leg_id == LEG_FL || leg_id == LEG_HL) ? 1 : -1;
    // 转换连杆长度，适应坐标系方向
    float l1 = y_sign * leg_links->hip_length;  // 髋关节偏移（考虑左右方向）
    float l2 = -leg_links->thigh_length;        // 大腿长度
    float l3 = -leg_links->calf_length;         // 小腿长度

    float s1 = sinf(joint_pos[HIP_IDX]);
    float s2 = sinf(joint_pos[THIGH_IDX]);
    float s3 = sinf(joint_pos[CALF_IDX]);

    float c1 = cosf(joint_pos[HIP_IDX]);
    float c2 = cosf(joint_pos[THIGH_IDX]);
    float c3 = cosf(joint_pos[CALF_IDX]);

    float c23 = c2 * c3 - s2 * s3;
    float s23 = s2 * c3 + c2 * s3;
    jacobian[0][0] = 0;
    jacobian[1][0] = -l3 * c1 * c23 - l2 * c1 * c2 - l1 * s1;
    jacobian[2][0] = -l3 * s1 * c23 - l2 * c2 * s1 + l1 * c1;
    jacobian[0][1] = l3 * c23 + l2 * c2;
    jacobian[1][1] = l3 * s1 * s23 + l2 * s1 * s2;
    jacobian[2][1] = -l3 * c1 * s23 - l2 * c1 * s2;
    jacobian[0][2] = l3 * c23;
    jacobian[1][2] = l3 * s1 * s23;
    jacobian[2][2] = -l3 * c1 * s23;
}

void leg_forward_kinematics_vel(uint8_t leg_id, const float joint_pos[3], const float joint_vel[3], float foot_vel[3])
{
    float jacobian[3][3];
    leg_jacobian(leg_id, joint_pos, jacobian);
    for (int i = 0; i < 3; i++) {
        foot_vel[i] = 0;
        for (int j = 0; j < 3; j++) {
            foot_vel[i] += jacobian[i][j] * joint_vel[j];
        }
    }
}

// 传入当前关节力、位置、速度，输出当前足端力、位置、速度
void leg_forward_kinematics_force_pos_vel(uint8_t leg_id, const float joint_force[3], const float joint_pos[3], const float joint_vel[3], float foot_force[3], float foot_pos[3], float foot_vel[3])
{
    float jacobian[3][3];
    float jacobian_T[3][3];
    float jacobian_T_inv[3][3];
    leg_jacobian(leg_id, joint_pos, jacobian);
    leg_forward_kinematics(leg_id, joint_pos, foot_pos);
    mat_transpose_ptr(3, 3, (float *)jacobian, (float *)jacobian_T);
    mat_inverse_ptr(3, &jacobian_T[0][0], &jacobian_T_inv[0][0]);

    for (int i = 0; i < 3; i++) {
        foot_vel[i] = 0;
        foot_force[i] = 0;
        for (int j = 0; j < 3; j++) {
            foot_vel[i] += jacobian[i][j] * joint_vel[j];
            foot_force[i] += jacobian_T_inv[i][j] * joint_force[j];
        }
    }
}

// 传入当前关节力、位置、速度，输出当前足端力、位置、速度

float test_jacobian[3][3];
float test_joint_force[3];
// 传入目标足端力、位置、速度，输出目标关节力、位置、速度
void leg_inverse_kinematics_force_pos_vel(uint8_t leg_id,const float foot_force[3], const float foot_pos[3], const float foot_vel[3],float joint_force[3], float joint_pos[3], float joint_vel[3])
{
    float jacobian[3][3];
    float jacobian_inv[3][3];
    float jacobian_T[3][3];
    float joint_current_pos[3] = {0};
    
    leg_inverse_kinematics(leg_id, foot_pos, joint_pos);
    leg_get_current_joint_pos(leg_id, joint_current_pos);
    leg_jacobian(leg_id, joint_current_pos, jacobian);
    if (leg_id == 0)
        memcpy(test_jacobian, jacobian, sizeof(test_jacobian));
    mat_transpose_ptr(3, 3, (float *)jacobian, (float *)jacobian_T);
    mat_inverse_3x3_ptr((float *)jacobian, (float *)jacobian_inv);
    // if(mat_inverse_ptr(3, &jacobian[0][0], &jacobian_inv[0][0]) == MAT_ERROR) {
    //     // printf("jacobian_inv error.\n");
    // }
//    else {
        for (int i = 0; i < 3; i++) {
            joint_vel[i] = 0;
            joint_force[i] = 0;
            for (int j = 0; j < 3; j++) {
                joint_vel[i] += jacobian_inv[i][j] * foot_vel[j];
                joint_force[i] += jacobian_T[i][j] * foot_force[j];
            }
        }
//    }
    if (leg_id == 0)
        memcpy(test_joint_force, joint_force, sizeof(test_joint_force));
}


/*========================= 坐标系转换函数 =========================*/

/**
 * @brief 单腿关节坐标系与电机坐标系互相转换
 * 
 * @param leg_id 腿的ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param input 输入角度数组 [3]
 * @param output 输出角度数组 [3]
 * @param direction 转换方向 (JOINT_TO_MOTOR或MOTOR_TO_JOINT)
 */
void leg_transform(uint8_t leg_id, const float *input, float *output, TransformDirection direction)
{
    const MotorJointDirConfig *motor_dir = &dog.params.motor_param.motor_dir[leg_id];
    const MotorZeroPos *motor_zero_pos = &dog.params.motor_param.motor_zero_pos[leg_id];
    for (int joint_idx = 0; joint_idx < 3; joint_idx++) {
        int8_t dir = 1;
        float zero_pos = 0;
        // 根据关节类型选择对应的方向配置
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
        // 根据转换方向应用变换
        if (direction == JOINT_TO_MOTOR) {
            output[joint_idx] = dir * input[joint_idx] + zero_pos;  // 关节旋转的角度增量乘旋转方向加上电机零位角度
        } else {  // MOTOR_TO_JOINT
            output[joint_idx] = dir * (input[joint_idx] - zero_pos);  // 电机旋转的角度增量乘旋转方向加上关节零位角度（0度）
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

void leg_motor_to_joint_vel(uint8_t leg_id, const float *motor_vel, float *joint_vel)
{
    const MotorJointDirConfig *dir = &dog.params.motor_param.motor_dir[leg_id];
    joint_vel[HIP_IDX] = dir->hip_dir * motor_vel[HIP_IDX];
    joint_vel[THIGH_IDX] = dir->thigh_dir * motor_vel[THIGH_IDX];
    joint_vel[CALF_IDX] = dir->calf_dir * motor_vel[CALF_IDX];
}

void leg_joint_to_motor_vel(uint8_t leg_id, const float *joint_vel, float *motor_vel)
{
    const MotorJointDirConfig *dir = &dog.params.motor_param.motor_dir[leg_id];
    motor_vel[HIP_IDX] = dir->hip_dir * joint_vel[HIP_IDX];
    motor_vel[THIGH_IDX] = dir->thigh_dir * joint_vel[THIGH_IDX];
    motor_vel[CALF_IDX] = dir->calf_dir * joint_vel[CALF_IDX];
}

void leg_joint_to_motor_force(uint8_t leg_id, const float *joint_force, float *motor_force)
{
    const MotorJointDirConfig *dir = &dog.params.motor_param.motor_dir[leg_id];
    motor_force[HIP_IDX] = dir->hip_dir * joint_force[HIP_IDX];
    motor_force[THIGH_IDX] = dir->thigh_dir * joint_force[THIGH_IDX];
    motor_force[CALF_IDX] = dir->calf_dir * joint_force[CALF_IDX];
}

void leg_motor_to_joint_force(uint8_t leg_id, const float *motor_force, float *joint_force)
{
    const MotorJointDirConfig *dir = &dog.params.motor_param.motor_dir[leg_id];
    joint_force[HIP_IDX] = dir->hip_dir * motor_force[HIP_IDX];
    joint_force[THIGH_IDX] = dir->thigh_dir * motor_force[THIGH_IDX];
    joint_force[CALF_IDX] = dir->calf_dir * motor_force[CALF_IDX];
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

void leg_foot_to_motor_force_pos_vel(uint8_t leg_id, const float foot_force[3], const float foot_pos[3], const float foot_vel[3], float motor_force[3], float motor_pos[3], float motor_vel[3])
{
    float joint_pos[3];
    float joint_vel[3];
    float joint_force[3];
    leg_inverse_kinematics_force_pos_vel(leg_id, foot_force, foot_pos, foot_vel, joint_force, joint_pos, joint_vel);
    leg_joint_to_motor_vel(leg_id, joint_vel, motor_vel);
    leg_joint_to_motor(leg_id, joint_pos, motor_pos);
    leg_joint_to_motor_force(leg_id, joint_force, motor_force);
}

// 传入当前电机力、位置、速度，输出当前足端力、位置、速度
void leg_motor_to_foot_force_pos_vel(uint8_t leg_id, const float motor_force[3], const float motor_pos[3], const float motor_vel[3], float foot_force[3], float foot_pos[3], float foot_vel[3])
{
    float joint_pos[3];
    float joint_vel[3];
    float joint_force[3];
    leg_motor_to_joint(leg_id, motor_pos, joint_pos);
    leg_motor_to_joint_vel(leg_id, motor_vel, joint_vel);
    leg_motor_to_joint_force(leg_id, motor_force, joint_force);
    leg_forward_kinematics_force_pos_vel(leg_id, joint_force, joint_pos, joint_vel, foot_force, foot_pos, foot_vel);

}

/*========================= 机器人控制函数 =========================*/


/*========================= 重心控制函数 =========================*/

/**
 * @brief 通过调整足端中性点位置来控制机器人重心
 * 
 * @param center_of_gravity 重心偏移量，方向与机体坐标系相同
 * @param foot_neutral_points 髋关节坐标系下，输出的足端中性点位置数组
 */
void dog_cog_to_foot(const float cog_translation[3], const float cog_rotation[3], const float cog_foot_offset[2], float foot_points[4][3])
{
    float stand_height = dog.params.posture.stand_height;
    CenterOfGravityLimits *cog_limit = &dog.params.posture.center_of_gravity_limit;
    // 根据重心偏移量调整足端中性点位置(cog_xyz在地面坐标系:方向与机体坐标系相同，坐标轴原点为机体中心在地面的投影)
    float cog_x = cog_translation[X_IDX];
    float cog_y = cog_translation[Y_IDX];
    float cog_z = cog_translation[Z_IDX];
    float pitch_length = cog_rotation[X_IDX];
    float roll_length = cog_rotation[Z_IDX];
    
    // 安全限制，防止重心偏移过大导致不稳定
    float cog_x_max = cog_limit->cog_x_max;
    float cog_x_min = cog_limit->cog_x_min;
    float cog_y_max = cog_limit->cog_y_max;
    float cog_y_min = cog_limit->cog_y_min;
    float cog_z_max = cog_limit->cog_z_max;
    float cog_z_min = cog_limit->cog_z_min;
    
    // 根据重心偏移调整所有足端中性点(髋关节坐标系)
    for (int i = 0; i < 4; i++) {
        // 反向补偿重心偏移（机体重心移动到左前方，足端中性点需要向右后方移动）
        if (i == LEG_FL)
        {
            cog_x = cog_translation[X_IDX] - cog_foot_offset[X_IDX];
            cog_y = cog_translation[Y_IDX] - cog_foot_offset[Y_IDX];
            cog_z = cog_translation[Z_IDX] + pitch_length + roll_length;
        }
        else if (i == LEG_FR)
        {
            cog_x = cog_translation[X_IDX] - cog_foot_offset[X_IDX];
            cog_y = cog_translation[Y_IDX] + cog_foot_offset[Y_IDX];
            cog_z = cog_translation[Z_IDX] + pitch_length - roll_length;
        }
        else if (i == LEG_HL)
        {
            cog_x = cog_translation[X_IDX] + cog_foot_offset[X_IDX];
            cog_y = cog_translation[Y_IDX] - cog_foot_offset[Y_IDX];
            cog_z = cog_translation[Z_IDX] - pitch_length + roll_length;
        }
        else if (i == LEG_HR)
        {
            cog_x = cog_translation[X_IDX] + cog_foot_offset[X_IDX];
            cog_y = cog_translation[Y_IDX] + cog_foot_offset[Y_IDX];
            cog_z = cog_translation[Z_IDX] - pitch_length - roll_length;
        }
        // 限幅
        if (cog_x >  cog_x_max) cog_x =  cog_x_max;
        if (cog_x <  cog_x_min) cog_x =  cog_x_min;
        if (cog_y >  cog_y_max) cog_y =  cog_y_max;
        if (cog_y <  cog_y_min) cog_y =  cog_y_min;
        if (cog_z >  cog_z_max) cog_z =  cog_z_max;
        if (cog_z <  cog_z_min) cog_z =  cog_z_min;
        foot_points[i][X_IDX] = -cog_x;
        foot_points[i][Y_IDX] = -cog_y;
        foot_points[i][Z_IDX] = -stand_height - cog_z;
    }
}

// 平滑过渡函数
float smooth(float current, float target, float step_increment) {
    float diff = target - current;
    if (fabs(diff) <= step_increment) {
        return target; // 距离小于步长，直接到达目标
    }
    return current + (diff > 0 ? step_increment : -step_increment); // 向目标方向移动一步
}

/**
 * @brief 平滑过渡到指定重心位置
 * 
 * @param center_of_gravity 地面坐标系下期望的重心位置
 * @param step_increment 步长增量(m)
 */
void dog_smooth_cog(float step_increment)
{
    // 计算目标足端中性点位置
    float target_neutral_points[4][3];
    dog_cog_to_foot(get_dog_params()->posture.center_of_gravity.translation, get_dog_params()->posture.center_of_gravity.rotation, get_dog_params()->posture.center_of_gravity.foot_offset, target_neutral_points);
    // 计算每步增量 
    for (int i = 0; i < 4; i++) {
        dog.leg[i].foot_neutral_pos[X_IDX] = smooth(dog.leg[i].foot_neutral_pos[X_IDX], target_neutral_points[i][X_IDX], step_increment);
        dog.leg[i].foot_neutral_pos[Y_IDX] = smooth(dog.leg[i].foot_neutral_pos[Y_IDX], target_neutral_points[i][Y_IDX], step_increment);
        dog.leg[i].foot_neutral_pos[Z_IDX] = smooth(dog.leg[i].foot_neutral_pos[Z_IDX], target_neutral_points[i][Z_IDX], step_increment);
    }
}

void leg_set_motor_pos(uint8_t leg_id, const float motor_pos[3])
{
    memcpy(dog.leg[leg_id].motor_target_pos, motor_pos, 3 * sizeof(float));
}

void leg_set_motor_pos_vel(uint8_t leg_id, const float motor_pos[3], const float motor_vel[3])
{
    memcpy(dog.leg[leg_id].motor_target_pos, motor_pos, 3 * sizeof(float));
    memcpy(dog.leg[leg_id].motor_target_vel, motor_vel, 3 * sizeof(float));
}

void leg_set_motor_force_pos_vel(uint8_t leg_id, const float motor_force[3], const float motor_pos[3], const float motor_vel[3])
{
    memcpy(dog.leg[leg_id].motor_target_force, motor_force, 3 * sizeof(float));
    memcpy(dog.leg[leg_id].motor_target_pos, motor_pos, 3 * sizeof(float));
    memcpy(dog.leg[leg_id].motor_target_vel, motor_vel, 3 * sizeof(float));
}

void leg_set_joint_pos(uint8_t leg_id, const float joint_pos[3])
{
    memcpy(dog.leg[leg_id].join_target_pos, joint_pos, 3 * sizeof(float));
}

void leg_set_target_foot_pos(uint8_t leg_id, const float foot_pos[3])
{
    memcpy(dog.leg[leg_id].foot_target_pos, foot_pos, 3 * sizeof(float));
}

void leg_set_target_foot_force(uint8_t leg_id, const float foot_force[3])
{
    memcpy(dog.leg[leg_id].foot_target_force, foot_force, 3 * sizeof(float));
}

void leg_get_current_joint_pos(uint8_t leg_idx, float joint_current_pos[3])
{
    float motor_current_pos[3];
    leg_get_motors_current_pos(leg_idx, motor_current_pos);
    leg_motor_to_joint(leg_idx, motor_current_pos, joint_current_pos);
}

void leg_get_current_foot_pos(uint8_t leg_idx, float foot_pos[3])
{
    float joint_current_pos[3];
    leg_get_current_joint_pos(leg_idx, joint_current_pos);
    leg_forward_kinematics(leg_idx, joint_current_pos, foot_pos);
}

void leg_get_current_joint_vel(uint8_t leg_idx, float joint_current_vel[3])
{
    float motor_current_vel[3];
    leg_get_motors_current_vel(leg_idx, motor_current_vel);
    leg_motor_to_joint_vel(leg_idx, motor_current_vel, joint_current_vel);
}

void leg_get_current_foot_force_pos_vel(uint8_t leg_idx, float foot_force[3], float foot_pos[3], float foot_vel[3])
{
    float motor_current_force[3];
    float motor_current_pos[3];
    float motor_current_vel[3];
    leg_get_motors_current_force(leg_idx, motor_current_force);
    leg_get_motors_current_pos(leg_idx, motor_current_pos);
    leg_get_motors_current_vel(leg_idx, motor_current_vel);
    leg_motor_to_foot_force_pos_vel(leg_idx, motor_current_force, motor_current_pos, motor_current_vel, foot_force, foot_pos, foot_vel);
}

// 机身坐标系原点为机体中心，x轴正方向为前进方向，y轴正方向为左，z轴正方向为上
void leg_get_current_foot_pos_body(uint8_t leg_idx, float foot_pos_body[3])
{
    float foot_pos[3];
    leg_get_current_foot_pos(leg_idx, foot_pos);
    switch (leg_idx) {
        case LEG_FL:
            foot_pos_body[X_IDX] = foot_pos[X_IDX] + BODY_CENTER_TO_LEG_X;
            foot_pos_body[Y_IDX] = foot_pos[Y_IDX] + BODY_CENTER_TO_LEG_Y;
            foot_pos_body[Z_IDX] = foot_pos[Z_IDX];
            break;
        case LEG_FR:
            foot_pos_body[X_IDX] = foot_pos[X_IDX] + BODY_CENTER_TO_LEG_X;
            foot_pos_body[Y_IDX] = foot_pos[Y_IDX] - BODY_CENTER_TO_LEG_Y;
            foot_pos_body[Z_IDX] = foot_pos[Z_IDX];
            break;
        case LEG_HL:
            foot_pos_body[X_IDX] = foot_pos[X_IDX] - BODY_CENTER_TO_LEG_X;
            foot_pos_body[Y_IDX] = foot_pos[Y_IDX] + BODY_CENTER_TO_LEG_Y;
            foot_pos_body[Z_IDX] = foot_pos[Z_IDX];
            break;
        case LEG_HR:
            foot_pos_body[X_IDX] = foot_pos[X_IDX] - BODY_CENTER_TO_LEG_X;
            foot_pos_body[Y_IDX] = foot_pos[Y_IDX] - BODY_CENTER_TO_LEG_Y;
            foot_pos_body[Z_IDX] = foot_pos[Z_IDX];
            break;
    }
}

void leg_get_target_foot_pos(uint8_t leg_idx, float foot_pos[3])
{
    memcpy(foot_pos, dog.leg[leg_idx].foot_target_pos, 3 * sizeof(float));
}


void leg_get_neutral_current_pos(uint8_t leg_idx, float neutral_pos[3])
{
    memcpy(neutral_pos, dog.leg[leg_idx].foot_neutral_pos, 3 * sizeof(float));
}

void leg_get_target_joint_pos(uint8_t leg_idx, float joint_pos[3])
{
    memcpy(joint_pos, dog.leg[leg_idx].join_target_pos, 3 * sizeof(float));
}

void dog_get_body_vel(float velocity[3])
{
    velocity[X_IDX] = body_vel[X_IDX];
    velocity[Y_IDX] = body_vel[Y_IDX];
    velocity[Z_IDX] = body_vel[Z_IDX];
}

void dog_get_body_vel_without_cog(float velocity[3])
{
    velocity[X_IDX] = body_vel[X_IDX] - get_dog_params()->posture.center_of_gravity.velocity[X_IDX];
    velocity[Y_IDX] = body_vel[Y_IDX] - get_dog_params()->posture.center_of_gravity.velocity[Y_IDX];
    velocity[Z_IDX] = body_vel[Z_IDX] - get_dog_params()->posture.center_of_gravity.velocity[Z_IDX];
}

int leg_get_contact_state(uint8_t leg_idx)
{
    return get_dog_params()->posture.contact[leg_idx];
}

float leg_get_phase(uint8_t leg_idx)
{
    return get_dog_params()->posture.phase[leg_idx];
}

void dog_set_body_vel(float vx, float vy, float w)
{
    // 速度加上重心补偿速度
    body_vel[X_IDX] = vx + get_dog_params()->posture.center_of_gravity.velocity[X_IDX];
    body_vel[Y_IDX] = vy + get_dog_params()->posture.center_of_gravity.velocity[Y_IDX];
    body_vel[Z_IDX] = w + get_dog_params()->posture.center_of_gravity.velocity[Z_IDX];
}

float dog_get_stand_height()
{
    return dog.params.posture.stand_height;
}

void dog_set_stand_height(float height)
{
    dog.params.posture.stand_height = height;
}



/*========================= send函数 =========================*/
float motors_target_force[12];
void dog_send_motors()
{
    float motors_target_pos[12];
    float motors_target_vel[12];
    
    for (int leg_idx = 0; leg_idx < 4; leg_idx++) {
        memcpy(motors_target_pos + leg_idx * 3, 
               dog.leg[leg_idx].motor_target_pos, 
               3 * sizeof(float));
        memcpy(motors_target_vel + leg_idx * 3, 
               dog.leg[leg_idx].motor_target_vel, 
               3 * sizeof(float));
        memcpy(motors_target_force + leg_idx * 3, 
               dog.leg[leg_idx].motor_target_force, 
               3 * sizeof(float));
    }
    send_motors_target_force_pos_vel(motors_target_force, motors_target_pos, motors_target_vel);
}

//void dog_send_motors_from_joints()
//{
//    // 创建一个临时数组来存储所有电机目标位置
//    float motors_target_pos[12];
//    for (int leg_idx = 0; leg_idx < 4; leg_idx++) {
//        leg_joint_to_motor(leg_idx, dog.leg[leg_idx].join_target_pos, dog.leg[leg_idx].motor_target_pos);
//        memcpy(motors_target_pos + leg_idx * 3, 
//               dog.leg[leg_idx].motor_target_pos, 
//               3 * sizeof(float));
//    }
//    // 调用电机的发送函数
//    send_motors_target_pos(motors_target_pos);
//}
     
/*========================= 初始化函数 =========================*/
void dog_init(const RobotParams* init_params)
{
    // 如果提供了初始参数，使用提供的参数；否则使用默认参数
    if (init_params != NULL) {
        memcpy(&dog.params, init_params, sizeof(RobotParams));
    } else {
        // 加载默认参数
        memcpy(&dog.params, get_robot_params(), sizeof(RobotParams));
    }
    // 初始化腿ID
    dog.leg[0].leg_id = LEG_FL;
    dog.leg[1].leg_id = LEG_FR;
    dog.leg[2].leg_id = LEG_HL;
    dog.leg[3].leg_id = LEG_HR;

    // 初始化足端中性点位置
    float foot_neutral_points[4][3];
    dog_cog_to_foot(dog.params.posture.center_of_gravity.translation, 
                     dog.params.posture.center_of_gravity.rotation, 
                     dog.params.posture.center_of_gravity.foot_offset, foot_neutral_points);
    
    for (int leg_idx = 0; leg_idx < 4; leg_idx++) {
        memcpy(dog.leg[leg_idx].foot_neutral_pos, foot_neutral_points[leg_idx], 3 * sizeof(float));
    }


    // 初始化关节角度(保持上电时角度)
    float motor_current_pos[4][3];
    
    for (int leg_idx = 0; leg_idx < 4; leg_idx++) {
        leg_get_motors_current_pos(leg_idx, motor_current_pos[leg_idx]);
        leg_motor_to_joint(leg_idx, motor_current_pos[leg_idx], dog.leg[leg_idx].join_target_pos);
        leg_set_motor_pos(leg_idx, motor_current_pos[leg_idx]);
    }

    
    // 初始化电机角度后，赋予电机控制参数
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

/* 获取狗参数 */
RobotParams* get_dog_params(void) {
    return &dog.params;
}
