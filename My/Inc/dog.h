/**
 * @file dog.h
 * @brief 四足机器人定义和接口声明
 */

#ifndef _DOG_H
#define _DOG_H

/*========================= 头文件包含 =========================*/
#include "stdint.h"
#include "robot_params.h"

// 坐标索引
#define X_IDX 0
#define Y_IDX 1
#define Z_IDX 2

// 关节索引
#define HIP_IDX   0
#define THIGH_IDX 1
#define CALF_IDX  2

// 腿部索引
#define FL_IDX 0
#define FR_IDX 1
#define HL_IDX 2
#define HR_IDX 3


/*========================= 数据类型定义 =========================*/
/**
 * @brief 单条腿结构体
 */
typedef struct
{
    uint8_t leg_id;              // 腿的ID (0:FL, 1:FR, 2:HL, 3:HR)
    float join_target_pos[3];    // 目标关节角度 [hip, thigh, calf]
    float motor_target_pos[3];   // 目标电机角度 [hip, thigh, calf]
    float foot_target_pos[3];    // 足端目标位置 [x, y, z]
    float foot_neutral_pos[3];   // 足端中性点位置 [x, y, z]
} Leg;

/**
 * @brief 四足机器人结构体
 */
typedef struct
{
    Leg leg[4];          // 四条腿 [FL, FR, HL, HR]
    RobotParams params;  // 机器人参数配置
} Dog;

/*========================= 运动学函数声明 =========================*/
/**
 * @brief 其次矩阵求运动学正解，计算从关节角度到足端位置的映射
 * @param leg_id 腿的ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param joint_pos 输入的关节角度数组 [hip, thigh, calf]
 * @param foot_pos 输出的足端位置
 */
void leg_forward_kinematics(uint8_t leg_id, const float joint_pos[3], float foot_pos[3]);

/**
 * @brief 几何法求运动学逆解，计算从足端位置到关节角度的映射
 * @param leg_id 腿的ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param foot_pos 输入的足端位置
 * @param joint_pos 输出的关节角度数组 [hip, thigh, calf]
 */
void leg_inverse_kinematics(uint8_t leg_id, const float foot_pos[3], float joint_pos[3]);

/**
 * @brief 几何法求运动学逆解，计算从足端位置到关节角度的映射
 * @param leg_id 腿的ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param joint_pos 输入的关节角度数组 [hip, thigh, calf]
 * @param joint_vel 输入的关节速度数组 [hip, thigh, calf]
 * @param foot_vel 输出的足端速度数组 [x, y, z]
 */
void leg_forward_kinematics_vel(uint8_t leg_id, const float joint_pos[3], const float joint_vel[3], float foot_vel[3]);


/*========================= 坐标系转换函数声明 =========================*/
/**
 * @brief 单腿关节坐标系转电机坐标系
 * @param leg_id 腿的ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param joint_pos 关节角度数组 [3]
 * @param motor_pos 电机角度数组 [3]
 */
void leg_joint_to_motor(uint8_t leg_id, const float *joint_pos, float *motor_pos);

/**
 * @brief 单腿电机坐标系转关节坐标系
 * @param leg_id 腿的ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param motor_pos 电机角度数组 [3]
 * @param joint_pos 关节角度数组 [3]
 */
void leg_motor_to_joint(uint8_t leg_id, const float *motor_pos, float *joint_pos);

/**
 * @brief 大腿坐标系转换到髋关节坐标系
 * @param leg_id 腿的ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param thigh_pos 大腿坐标系位置
 * @param hip_pos 髋关节坐标系位置
 */
void leg_thigh_to_hip(uint8_t leg_id, const float thigh_pos[3], float hip_pos[3]);

/**
 * @brief 足端位置直接转换为电机角度
 * @param leg_id 腿的ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param foot_pos 足端位置
 * @param motor_pos 输出的电机角度数组 [3]
 */
void leg_foot_to_motor(uint8_t leg_id, const float foot_pos[3], float motor_pos[3]);

/*========================= 机器人初始化函数 =========================*/
/**
 * @brief 初始化机器狗参数和状态
 * @param init_params 初始化参数，NULL时使用默认参数
 */
void dog_init(const RobotParams* init_params);

/**
 * @brief 更新机器人数据
 */
void dog_data_update(void);

/*========================= set函数声明 =========================*/
/**
 * @brief 设置指定腿的电机目标位置
 * @param leg_id 腿的ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param motor_pos 电机目标位置数组 [3]
 */
void leg_set_motor_pos(uint8_t leg_id, const float motor_pos[3]);

/**
 * @brief 设置指定腿的关节目标位置
 * @param leg_id 腿的ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param joint_pos 关节目标位置数组 [3]
 */
void leg_set_joint_pos(uint8_t leg_id, const float joint_pos[3]);

/**
 * @brief 设置指定腿的足端目标位置
 * @param leg_id 腿的ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param foot_pos 足端目标位置数组 [3]
 */
void leg_set_target_foot_pos(uint8_t leg_id, const float foot_pos[3]);

/**
 * @brief 设置机体速度
 * @param vx 机体前进速度
 * @param vy 机体侧移速度
 * @param w 机体自转速度
 */
void dog_set_body_vel(float vx, float vy, float w);


/**
 * @brief 设置站立高度
 * @param height 站立高度
 */
void dog_set_stand_height(float height);

/*========================= get函数声明 =========================*/
/**
 * @brief 获取指定腿的当前关节位置
 * @param leg_idx 腿的ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param joint_current_pos 关节当前位置数组 [3]
 */
void leg_get_current_joint_pos(uint8_t leg_idx, float joint_current_pos[3]);

/**
 * @brief 获取指定腿的当前关节速度
 * @param leg_idx 腿的ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param joint_current_vel 关节当前速度数组 [3]
 */
void leg_get_current_joint_vel(uint8_t leg_idx, float joint_current_vel[3]);

/**
 * @brief 获取指定腿的当前足端位置
 * @param leg_idx 腿的ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param foot_pos 足端位置数组 [3]
 */
void leg_get_current_foot_pos(uint8_t leg_idx, float foot_pos[3]);

/**
 * @brief 获取指定腿的当前足端位置(机身坐标系下)
 * @description 机身坐标系原点为机体中心，x轴正方向为前进方向，y轴正方向为左，z轴正方向为上
 * @param leg_idx 腿的ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param foot_pos_body 足端位置数组 [3]
 */
void leg_get_current_foot_pos_body(uint8_t leg_idx, float foot_pos_body[3]);

/**
 * @brief 获取指定腿的中性点位置
 * @param leg_idx 腿的ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param neutral_pos 中性点位置数组 [3]
 */
void leg_get_neutral_current_pos(uint8_t leg_idx, float neutral_pos[3]);

/**
 * @brief 获取指定腿的关节位置
 * @param leg_idx 腿的ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param joint_pos 关节位置数组 [3]
 */
void leg_get_joint_pos(uint8_t leg_idx, float joint_pos[3]);

/**
 * @brief 获取指定腿的足端目标位置
 * @param leg_idx 腿的ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @param foot_pos 足端目标位置数组 [3]
 */
void leg_get_target_foot_pos(uint8_t leg_idx, float foot_pos[3]);

/**
 * @brief Al获取机体速度
 * @param velocity 机体速度数组 [3]
 */
void dog_get_body_vel(float velocity[3]);

/**
 * @brief 获取机体速度（不包含重心速度）
 * @param velocity 机体速度数组 [3]
 */
void dog_get_body_vel_without_cog(float velocity[3]);

/**
 * @brief 获取站立高度
 * @return float 站立高度
 */
float dog_get_stand_height(void);

/**
 * @brief 获取狗参数
 * @return RobotParams* 狗参数结构体
 */
RobotParams* get_dog_params(void);
/**
 * @brief 获取指定腿的接触状态
 * @param leg_idx 腿的ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @return int 接触状态 (0: 不接触, 1: 接触)
 */
int leg_get_contact_state(uint8_t leg_idx);

/**
 * @brief 获取指定腿的步态相位
 * @param leg_idx 腿的ID (0:FL, 1:FR, 2:HL, 3:HR)
 * @return float 步态相位
 */
float leg_get_phase(uint8_t leg_idx);

/*========================= send函数声明 =========================*/
/**
 * @brief 发送电机目标位置
 */
void dog_send_motors(void);

/**
 * @brief 把关节角度转换为电机角度并发送
 */
void dog_send_motors_from_joints(void);

/*========================= 控制函数声明 =========================*/
/**
 * @brief 机器人控制主函数，执行运动、姿态和步态控制
 */
void robot_control(void);

/*========================= 重心控制函数声明 =========================*/
/**
 * @brief 平滑过渡到指定重心位置
 * @param step_increment 单步增量
 */
void dog_smooth_cog(float step_increment);


float smooth(float current, float target, float step_increment);

#endif /* _DOG_H */
