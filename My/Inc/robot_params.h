#ifndef _ROBOT_PARAMS_H
#define _ROBOT_PARAMS_H

#include "stdint.h"
#include "math.h"
#include "j60.h"
// 此.h文件参数为固定参数，换连杆、电机等后需要修改，一般只需要修改一次
// 其余步态参数、控制参数在此文件对应的.c文件中修改

/************************************************************
 * 基础参数定义
 ************************************************************/
// 单位转换系数
#define RADIAN_TO_DEGREE 57.295779513f
#define DEGREE_TO_RADIAN 0.0174533f
#define MY_PI 3.1415926f
// 腿索引定义
//  0   1
//
//
//  2   3
#define LEG_FL 0
#define LEG_FR 1
#define LEG_HL 2
#define LEG_HR 3

// 关节索引定义
#define JOINT_HIP   0
#define JOINT_THIGH 1
#define JOINT_CALF  2

/*-------------------------换连杆、电机等后需要修改下列参数-------------------------*/

/************************************************************
 * 机械参数
 ************************************************************/
// 连杆长度（米）
#define LINK_LENGTH_HIP   0.0693f
#define LINK_LENGTH_THIGH 0.255f
#define LINK_LENGTH_CALF  0.265f

// 机体尺寸(机身中心到大腿关节的距离，用于分配自转角速度到xy平面)
#define BODY_LENGTH 0.459f 
#define BODY_WIDTH 0.3686f

// 机身坐标系原点与腿坐标系原点距离
#define BODY_CENTER_TO_LEG_X (BODY_LENGTH/2.0f)
#define BODY_CENTER_TO_LEG_Y (BODY_WIDTH/2.0f - LINK_LENGTH_HIP)


// 大腿关节与机体中心连线角度
#define HIP_ANGLE atan2f(BODY_LENGTH, BODY_WIDTH)
#define CENTER_TO_THIGH (0.5f * sqrtf(BODY_LENGTH*BODY_LENGTH + BODY_WIDTH*BODY_WIDTH))
/************************************************************
 * 默认姿态参数
 ************************************************************/
// 站立高度（米）
#define DEFAULT_STAND_HEIGHT 0.33f //0.3

// 站立时重心位置
#define DEFAULT_CENTER_OF_GRAVITY_TRANSLATION_X 0.01f
#define DEFAULT_CENTER_OF_GRAVITY_TRANSLATION_Y 0.001f
#define DEFAULT_CENTER_OF_GRAVITY_TRANSLATION_Z 0.0f

#define DEFAULT_CENTER_OF_GRAVITY_ROTATION_X 0.0f
#define DEFAULT_CENTER_OF_GRAVITY_ROTATION_Y 0.0f
#define DEFAULT_CENTER_OF_GRAVITY_ROTATION_Z 0.0f

// 速度补偿，原地踏步时狗往前往后退自转的话改这个值
#define DEFAULT_CENTER_OF_GRAVITY_VX 0.11f
#define DEFAULT_CENTER_OF_GRAVITY_VY -0.02f
#define DEFAULT_CENTER_OF_GRAVITY_VW -0.01f

// 重心偏移量限制
#define DEFAULT_CENTER_OF_GRAVITY_MAX_X 0.2f
#define DEFAULT_CENTER_OF_GRAVITY_MIN_X -0.2f
#define DEFAULT_CENTER_OF_GRAVITY_MAX_Y 0.2f
#define DEFAULT_CENTER_OF_GRAVITY_MIN_Y -0.2f
#define DEFAULT_CENTER_OF_GRAVITY_MAX_Z 0.2f
#define DEFAULT_CENTER_OF_GRAVITY_MIN_Z -0.2f


// 默认关节角度（弧度） # 已弃用
#define DEFAULT_JOINT_ANGLE_HIP    0.0f
#define DEFAULT_JOINT_ANGLE_THIGH  0.67f
#define DEFAULT_JOINT_ANGLE_CALF  -1.3f


/*-------------------------电机参数配置-------------------------*/
/************************************************************
 * 电机id与关节id对应关系
 ************************************************************/
#define MOTOR_FL_HIP_ID 3
#define MOTOR_FL_THIGH_ID 2
#define MOTOR_FL_CALF_ID 1

#define MOTOR_FR_HIP_ID 6
#define MOTOR_FR_THIGH_ID 5
#define MOTOR_FR_CALF_ID 4

#define MOTOR_HL_HIP_ID 12
#define MOTOR_HL_THIGH_ID 11
#define MOTOR_HL_CALF_ID 10

#define MOTOR_HR_HIP_ID 9
#define MOTOR_HR_THIGH_ID 8
#define MOTOR_HR_CALF_ID 7
/************************************************************
 * 电机CAN总线配置
 ************************************************************/
#define MOTOR_FL_HIP_CAN_BUS CAN_BUS_1
#define MOTOR_FL_THIGH_CAN_BUS CAN_BUS_1
#define MOTOR_FL_CALF_CAN_BUS CAN_BUS_1

#define MOTOR_FR_HIP_CAN_BUS CAN_BUS_2
#define MOTOR_FR_THIGH_CAN_BUS CAN_BUS_2
#define MOTOR_FR_CALF_CAN_BUS CAN_BUS_2

#define MOTOR_HL_HIP_CAN_BUS CAN_BUS_1
#define MOTOR_HL_THIGH_CAN_BUS CAN_BUS_3
#define MOTOR_HL_CALF_CAN_BUS CAN_BUS_2

#define MOTOR_HR_HIP_CAN_BUS CAN_BUS_3
#define MOTOR_HR_THIGH_CAN_BUS CAN_BUS_3
#define MOTOR_HR_CALF_CAN_BUS CAN_BUS_3

/************************************************************
 * 电机坐标系相关（用于确定电机坐标系与关节坐标系的关系）
 ************************************************************/
#define MOTOR_FL_HIP_DIRECTION -1 // 左前腿髋关节电机旋转方向，与关节坐标系旋转方向相同为1，相反为-1，
#define MOTOR_FL_THIGH_DIRECTION -1 // 左前腿大腿电机旋转方向，与关节坐标系旋转方向相同为1，相反为-1，
#define MOTOR_FL_CALF_DIRECTION -1 // 左前腿小腿电机旋转方向，与关节坐标系旋转方向相同为1，相反为-1，

#define MOTOR_FR_HIP_DIRECTION -1 // 右前腿髋关节电机旋转方向，与关节坐标系旋转方向相同为1，相反为-1，
#define MOTOR_FR_THIGH_DIRECTION 1 // 右前腿大腿电机旋转方向，与关节坐标系旋转方向相同为1，相反为-1，
#define MOTOR_FR_CALF_DIRECTION 1 // 右前腿小腿电机旋转方向，与关节坐标系旋转方向相同为1，相反为-1，

#define MOTOR_HL_HIP_DIRECTION 1 // 左后腿髋关节电机旋转方向，与关节坐标系旋转方向相同为1，相反为-1，
#define MOTOR_HL_THIGH_DIRECTION -1 // 左后腿大腿电机旋转方向，与关节坐标系旋转方向相同为1，相反为-1，
#define MOTOR_HL_CALF_DIRECTION -1 // 左后腿小腿电机旋转方向，与关节坐标系旋转方向相同为1，相反为-1，

#define MOTOR_HR_HIP_DIRECTION 1 // 右后腿髋关节电机旋转方向，与关节坐标系旋转方向相同为1，相反为-1，
#define MOTOR_HR_THIGH_DIRECTION 1 // 右后腿大腿电机旋转方向，与关节坐标系旋转方向相同为1，相反为-1，
#define MOTOR_HR_CALF_DIRECTION 1 // 右后腿小腿电机旋转方向，与关节坐标系旋转方向相同为1，相反为-1，

// 特定位置的关节坐标系角度（一般由建模软件中直接测量得到）
#define JOINT_FL_HIP_ANGLE_SPECIAL_POS 0.0f
#define JOINT_FL_THIGH_ANGLE_SPECIAL_POS 2.4622f * DEGREE_TO_RADIAN
#define JOINT_FL_CALF_ANGLE_SPECIAL_POS -31.753f * DEGREE_TO_RADIAN

#define JOINT_FR_HIP_ANGLE_SPECIAL_POS 0.0f
#define JOINT_FR_THIGH_ANGLE_SPECIAL_POS 2.4622f * DEGREE_TO_RADIAN
#define JOINT_FR_CALF_ANGLE_SPECIAL_POS -31.753f * DEGREE_TO_RADIAN

#define JOINT_HL_HIP_ANGLE_SPECIAL_POS 0.0f
#define JOINT_HL_THIGH_ANGLE_SPECIAL_POS 2.4622f * DEGREE_TO_RADIAN
#define JOINT_HL_CALF_ANGLE_SPECIAL_POS -31.753f * DEGREE_TO_RADIAN

#define JOINT_HR_HIP_ANGLE_SPECIAL_POS 0.0f
#define JOINT_HR_THIGH_ANGLE_SPECIAL_POS 2.4622f * DEGREE_TO_RADIAN
#define JOINT_HR_CALF_ANGLE_SPECIAL_POS -31.753f * DEGREE_TO_RADIAN

// 对应的电机坐标系角度（摆好关节位置后从电机回传数据读取出）
#define MOTOR_FL_HIP_ANGLE_SPECIAL_POS 0.313f
#define MOTOR_FL_THIGH_ANGLE_SPECIAL_POS 1.54f
#define MOTOR_FL_CALF_ANGLE_SPECIAL_POS -2.23f

#define MOTOR_FR_HIP_ANGLE_SPECIAL_POS -0.517f
#define MOTOR_FR_THIGH_ANGLE_SPECIAL_POS -1.08f
#define MOTOR_FR_CALF_ANGLE_SPECIAL_POS 2.064f

#define MOTOR_HL_HIP_ANGLE_SPECIAL_POS -0.394f
#define MOTOR_HL_THIGH_ANGLE_SPECIAL_POS 1.04f
#define MOTOR_HL_CALF_ANGLE_SPECIAL_POS -2.168f

#define MOTOR_HR_HIP_ANGLE_SPECIAL_POS 0.179f
#define MOTOR_HR_THIGH_ANGLE_SPECIAL_POS -1.05f
#define MOTOR_HR_CALF_ANGLE_SPECIAL_POS 2.326f

// 计算电机坐标系与关节坐标系零位重合时电机的当前角度
#define MOTOR_FL_HIP_ANGLE_ZERO_POS (MOTOR_FL_HIP_ANGLE_SPECIAL_POS - MOTOR_FL_HIP_DIRECTION*JOINT_FL_HIP_ANGLE_SPECIAL_POS)
#define MOTOR_FL_THIGH_ANGLE_ZERO_POS (MOTOR_FL_THIGH_ANGLE_SPECIAL_POS - MOTOR_FL_THIGH_DIRECTION*JOINT_FL_THIGH_ANGLE_SPECIAL_POS)
#define MOTOR_FL_CALF_ANGLE_ZERO_POS (MOTOR_FL_CALF_ANGLE_SPECIAL_POS - MOTOR_FL_CALF_DIRECTION*JOINT_FL_CALF_ANGLE_SPECIAL_POS)

#define MOTOR_FR_HIP_ANGLE_ZERO_POS (MOTOR_FR_HIP_ANGLE_SPECIAL_POS - MOTOR_FR_HIP_DIRECTION*JOINT_FR_HIP_ANGLE_SPECIAL_POS)
#define MOTOR_FR_THIGH_ANGLE_ZERO_POS (MOTOR_FR_THIGH_ANGLE_SPECIAL_POS - MOTOR_FR_THIGH_DIRECTION*JOINT_FR_THIGH_ANGLE_SPECIAL_POS)
#define MOTOR_FR_CALF_ANGLE_ZERO_POS (MOTOR_FR_CALF_ANGLE_SPECIAL_POS - MOTOR_FR_CALF_DIRECTION*JOINT_FR_CALF_ANGLE_SPECIAL_POS)

#define MOTOR_HL_HIP_ANGLE_ZERO_POS (MOTOR_HL_HIP_ANGLE_SPECIAL_POS - MOTOR_HL_HIP_DIRECTION*JOINT_HL_HIP_ANGLE_SPECIAL_POS)
#define MOTOR_HL_THIGH_ANGLE_ZERO_POS (MOTOR_HL_THIGH_ANGLE_SPECIAL_POS - MOTOR_HL_THIGH_DIRECTION*JOINT_HL_THIGH_ANGLE_SPECIAL_POS)
#define MOTOR_HL_CALF_ANGLE_ZERO_POS (MOTOR_HL_CALF_ANGLE_SPECIAL_POS - MOTOR_HL_CALF_DIRECTION*JOINT_HL_CALF_ANGLE_SPECIAL_POS)

#define MOTOR_HR_HIP_ANGLE_ZERO_POS (MOTOR_HR_HIP_ANGLE_SPECIAL_POS - MOTOR_HR_HIP_DIRECTION*JOINT_HR_HIP_ANGLE_SPECIAL_POS)
#define MOTOR_HR_THIGH_ANGLE_ZERO_POS (MOTOR_HR_THIGH_ANGLE_SPECIAL_POS - MOTOR_HR_THIGH_DIRECTION*JOINT_HR_THIGH_ANGLE_SPECIAL_POS)
#define MOTOR_HR_CALF_ANGLE_ZERO_POS (MOTOR_HR_CALF_ANGLE_SPECIAL_POS - MOTOR_HR_CALF_DIRECTION*JOINT_HR_CALF_ANGLE_SPECIAL_POS)

/************************************************************
 * 电机角度限制（弧度）
 ************************************************************/
// FL腿电机限制
#define MOTOR_FL_HIP_MAX   0.65f
#define MOTOR_FL_HIP_MIN   -0.6f
#define MOTOR_FL_THIGH_MAX 2.166f
#define MOTOR_FL_THIGH_MIN 0.5f
#define MOTOR_FL_CALF_MAX  -0.26f
#define MOTOR_FL_CALF_MIN  -1.8f

// FR腿电机限制
#define MOTOR_FR_HIP_MAX    0.28f
#define MOTOR_FR_HIP_MIN   -0.8f
#define MOTOR_FR_THIGH_MAX -0.35f
#define MOTOR_FR_THIGH_MIN -2.0f
#define MOTOR_FR_CALF_MAX  1.7f
#define MOTOR_FR_CALF_MIN  0.13f

// HL腿电机限制
#define MOTOR_HL_HIP_MAX    0.4f
#define MOTOR_HL_HIP_MIN   -0.7f
#define MOTOR_HL_THIGH_MAX -0.18f
#define MOTOR_HL_THIGH_MIN 1.48f
#define MOTOR_HL_CALF_MAX  -0.48f
#define MOTOR_HL_CALF_MIN  -0.2f

// HR腿电机限制
#define MOTOR_HR_HIP_MAX    1.0f
#define MOTOR_HR_HIP_MIN   -0.15f
#define MOTOR_HR_THIGH_MAX -0.1f
#define MOTOR_HR_THIGH_MIN -1.68f
#define MOTOR_HR_CALF_MAX  1.9f
#define MOTOR_HR_CALF_MIN  0.25f


/************************************************************
 * 结构体定义
 ************************************************************/

// 连杆参数结构体
typedef struct {
    float hip_length;     // 髋关节连杆长度
    float thigh_length;   // 大腿连杆长度
    float calf_length;    // 小腿连杆长度
} LegLinkParams;

// 关节角度限制结构体
typedef struct {
    float max_hip_angle;    // 髋关节最大角度
    float min_hip_angle;    // 髋关节最小角度
    float max_thigh_angle;  // 大腿关节最大角度
    float min_thigh_angle;  // 大腿关节最小角度
    float max_calf_angle;   // 小腿关节最大角度
    float min_calf_angle;   // 小腿关节最小角度
} LegAngleLimits;

// 重心
typedef struct {
    float translation[3];
    float rotation[3];
    float velocity[3];
} CenterOfGravity;


// 重心偏移量限制
typedef struct {
    float cog_x_max;    // 机体x方向重心偏移量最大值
    float cog_x_min;    // 机体x方向重心偏移量最小值
    float cog_y_max;    // 机体y方向重心偏移量最大值
    float cog_y_min;    // 机体y方向重心偏移量最小值
    float cog_z_max;    // 机体z方向重心偏移量最大值
    float cog_z_min;    // 机体z方向重心偏移量最小值
} CenterOfGravityLimits;

typedef struct {
    uint8_t hip_id;
    uint8_t thigh_id;
    uint8_t calf_id;
} MotorIDConfig;

typedef struct {
    int8_t hip_dir;
    int8_t thigh_dir;
    int8_t calf_dir;
} MotorJointDirConfig;

typedef struct {
    float hip_zero_pos;
    float thigh_zero_pos;
    float calf_zero_pos;
} MotorZeroPos;

typedef struct {
    float kp;
    float kd;
} MotorControlParams;

// 电机与关节对应关系配置结构体
typedef struct {
    MotorIDConfig motor_id[4];
    LegAngleLimits motor_limits[4];
    MotorJointDirConfig motor_dir[4];
    MotorZeroPos motor_zero_pos[4];
    MotorControlParams motor_control_params[12];
} MotorParams; 

// 姿态参数结构体
typedef struct {
    float stand_height;    // 站立高度
    CenterOfGravityLimits center_of_gravity_limit; // 重心偏移量限制
    CenterOfGravity center_of_gravity; // 重心位置
    float default_joint_pos[4][3];  // 每条腿的默认关节角度 [leg][joint]
    int contact[4]; // 每条腿的接触状态
    float phase[4]; // 每条腿当前所在相位
} PostureParams;

typedef struct {
    float phase[4];        // 步态相位
    float T;               // 步态周期(s)
    float stance_ratio;    // 支撑相时间占周期比例
    float swing_ratio;     // 摆动相时间占周期比例
    float step_length;     // 步长
    float swing_height;    // 摆动高度
    float stance_depth;    // 支撑深度
    float stand_height;   // 站立高度
} GaitParams;

// PID控制参数结构体
typedef struct {
    float kp;  // 比例系数
    float ki;  // 积分系数
    float kd;  // 微分系数
} PIDParams;

// 机器人参数总结构体
typedef struct {
    LegLinkParams leg_links;       // 腿部连杆参数
    LegAngleLimits joint_limits;   // 关节角度限制
    MotorParams motor_param;       // 电机参数
    PostureParams posture;         // 姿态参数

    GaitParams trot_gait;
    PIDParams position_control;    // 位置控制PID参数
    PIDParams velocity_control;    // 速度控制PID参数
} RobotParams;

/************************************************************
 * 函数声明
 ************************************************************/
RobotParams* get_robot_params(void);

#endif /* _ROBOT_PARAMS_H */
