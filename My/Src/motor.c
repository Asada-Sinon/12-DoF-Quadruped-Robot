#include "motor.h"
#include "robot_params.h"
#include "stdio.h"
#include "j60.h"
#include "dog.h"
// 初始化电机回传

void motor_init()
{
    
    J60_Can_Init();
    
    // FL
    J60_ConfigMotor(LEG_FL*3 + JOINT_HIP, MOTOR_FL_HIP_CAN_BUS, MOTOR_FL_HIP_ID);   
    J60_ConfigMotor(LEG_FL*3 + JOINT_THIGH, MOTOR_FL_THIGH_CAN_BUS, MOTOR_FL_THIGH_ID);   
    J60_ConfigMotor(LEG_FL*3 + JOINT_CALF, MOTOR_FL_CALF_CAN_BUS, MOTOR_FL_CALF_ID); 

    // FR
    J60_ConfigMotor(LEG_FR*3 + JOINT_HIP, MOTOR_FR_HIP_CAN_BUS, MOTOR_FR_HIP_ID);   
    J60_ConfigMotor(LEG_FR*3 + JOINT_THIGH, MOTOR_FR_THIGH_CAN_BUS, MOTOR_FR_THIGH_ID);   
    J60_ConfigMotor(LEG_FR*3 + JOINT_CALF, MOTOR_FR_CALF_CAN_BUS, MOTOR_FR_CALF_ID);  
    
    // HL
    J60_ConfigMotor(LEG_HL*3 + JOINT_HIP, MOTOR_HL_HIP_CAN_BUS, MOTOR_HL_HIP_ID);  
    J60_ConfigMotor(LEG_HL*3 + JOINT_THIGH, MOTOR_HL_THIGH_CAN_BUS, MOTOR_HL_THIGH_ID);  
    J60_ConfigMotor(LEG_HL*3 + JOINT_CALF, MOTOR_HL_CALF_CAN_BUS, MOTOR_HL_CALF_ID); 

    // HR
    J60_ConfigMotor(LEG_HR*3 + JOINT_HIP, MOTOR_HR_HIP_CAN_BUS, MOTOR_HR_HIP_ID);  
    J60_ConfigMotor(LEG_HR*3 + JOINT_THIGH, MOTOR_HR_THIGH_CAN_BUS, MOTOR_HR_THIGH_ID);  
    J60_ConfigMotor(LEG_HR*3 + JOINT_CALF, MOTOR_HR_CALF_CAN_BUS, MOTOR_HR_CALF_ID);  


   for (uint8_t i = 0; i < 12; i++) { // 初始化使能状态，1使能失败，0使能成功
       J60_GetMotor(i)->is_enable = 1;
   }

    while(J60_GetMotor(0)->is_enable == 1 || J60_GetMotor(1)->is_enable == 1 || J60_GetMotor(2)->is_enable == 1 || J60_GetMotor(3)->is_enable == 1 || \
    J60_GetMotor(4)->is_enable == 1 || J60_GetMotor(5)->is_enable == 1 || J60_GetMotor(6)->is_enable == 1 || J60_GetMotor(7)->is_enable == 1 || \
    J60_GetMotor(8)->is_enable == 1 || J60_GetMotor(9)->is_enable == 1 || J60_GetMotor(10)->is_enable == 1 || J60_GetMotor(11)->is_enable == 1)
    {
        for (int i = 0; i < 12; i++) 
        {
            if (J60_GetMotor(i)->is_enable == 1)
            {
                J60_EnableMotor(i);
                HAL_Delay(10);
//                printf("未接收到电机[%d]回传数据\n", i);
            }
        }
    }
    // 清空电机控制参数，只接收电机回传数据
    for (int i = 0; i < 12; i++) {
        J60_GetMotor(i)->kp = 0;
        J60_GetMotor(i)->kd = 0;
    }
    // printf("电机回传初始化成功\n");
}

/**
 * @brief 检查并限制电机角度在安全范围内
 * 
 * @param motor_target 目标电机角度数组 [12] (4条腿 x 3个关节)
 */
void check_motor_limit(float *motor_target_pos)
{
    // 获取机器人参数
    const RobotParams* params = get_dog_params();
    
    // 关节类型名称，用于警告信息
    const char* joint_names[] = {"hip", "thigh", "calf"};
    
    // 遍历所有腿
    for (int leg_idx = 0; leg_idx < 4; leg_idx++) 
    {
        // 获取当前腿的电机限制参数
        const LegAngleLimits* leg_limits = &params->motor_param.motor_limits[leg_idx];
        
        // 遍历每条腿的3个关节
        for (int joint_idx = 0; joint_idx < 3; joint_idx++) 
        {
            // 计算当前关节在数组中的索引
            int motor_idx = leg_idx * 3 + joint_idx;
            float angle = motor_target_pos[motor_idx];
            
            // 根据关节类型获取对应的角度限制
            float max_angle = 0.0f;
            float min_angle = 0.0f;
            
            switch (joint_idx) 
            {
                case JOINT_HIP:
                    max_angle = leg_limits->max_hip_angle;
                    min_angle = leg_limits->min_hip_angle;
                    break;
                case JOINT_THIGH:
                    max_angle = leg_limits->max_thigh_angle;
                    min_angle = leg_limits->min_thigh_angle;
                    break;
                case JOINT_CALF:
                    max_angle = leg_limits->max_calf_angle;
                    min_angle = leg_limits->min_calf_angle;
                    break;
            }
            
            // 检查上限
            if (angle > max_angle) 
            {
//                printf("警告：腿%d的%s关节角度超出上限! 当前值：%.3f，上限：%.3f\n", 
//                       leg_idx, joint_names[joint_idx], angle, max_angle);
                motor_target_pos[motor_idx] = max_angle;
            }
            
            // 检查下限
            if (angle < min_angle) 
            {
//                printf("警告：腿%d的%s关节角度超出下限! 当前值：%.3f，下限：%.3f\n", 
//                       leg_idx, joint_names[joint_idx], angle, min_angle);
                motor_target_pos[motor_idx] = min_angle;
            }
        }
    }
}

// 发送电机目标位置
void send_motors_target_pos(float *motors_target_pos)
{
//    check_motor_limit(motors_target_pos);  // 先检查限位
    
    for (int i = 0; i < 12; i++) {
        J60_MotorControl(i, motors_target_pos[i], 0, J60_GetMotor(i)->kp, J60_GetMotor(i)->kd, 0);
    } 
}   


// 获取电机当前位置
void leg_get_motors_current_pos(uint8_t leg_idx, float motors_current_pos[3])
{
    for (int i = 0; i < 3; i++) {
        motors_current_pos[i] = J60_GetMotor(leg_idx*3+i)->position;
    }
}

// 获取电机当前速度
void leg_get_motors_current_vel(uint8_t leg_idx, float motors_current_vel[3])
{
    for (int i = 0; i < 3; i++) {
        motors_current_vel[i] = J60_GetMotor(leg_idx*3+i)->velocity;
    }
}

void leg_set_motor_kp_kd(uint8_t leg_idx, float kp_hip, float kd_hip, float kp_thigh, float kd_thigh, float kp_calf, float kd_calf)
{
    J60_GetMotor(leg_idx*3+JOINT_HIP)->kp = kp_hip;
    J60_GetMotor(leg_idx*3+JOINT_HIP)->kd = kd_hip;
    J60_GetMotor(leg_idx*3+JOINT_THIGH)->kp = kp_thigh;
    J60_GetMotor(leg_idx*3+JOINT_THIGH)->kd = kd_thigh;
    J60_GetMotor(leg_idx*3+JOINT_CALF)->kp = kp_calf;
    J60_GetMotor(leg_idx*3+JOINT_CALF)->kd = kd_calf;
}
