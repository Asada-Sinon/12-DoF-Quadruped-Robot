#include "path.h"
#include "dog.h"
#include "math.h"
#include "vision.h"
#include "estimator.h"
#include "imu.h"
#include "fsm.h"

// 路径规划参数
typedef struct {
    float start_x;    // 起点x坐标
    float start_y;    // 起点y坐标
    float target_x;   // 目标点x坐标
    float target_y;   // 目标点y坐标
    float current_x;  // 当前x坐标
    float current_y;  // 当前y坐标
    float angle;      // 路径线坐标系角度
    float yaw;        // 目标偏航角
    float distance;   // 到目标点的距离
    uint8_t is_moving;// 是否在运动
    
    // 速度控制参数
    float max_speed;      // 最大速度 (m/s)
    float current_speed;  // 当前速度 (m/s)
    float accel;         // 加速度 (m/s^2)
    float decel;         // 减速度 (m/s^2)
    
    // 路径跟踪参数
    float cross_track_error;  // 横向跟踪误差
    float kp_cross;          // 横向误差比例系数
    float kd_cross;          // 横向误差微分系数
    float kp_yaw;        // 航向误差比例系数
    float kd_yaw;        // 航向误差微分系数

    float dt; // 控制周期
} PathPlan;

PathPlan path = {0};

// 初始化路径规划器
void path_init(void) {
    // 设置速度和加速度参数
    path.max_speed = 0.3f;    // 最大速度0.3m/s
    path.accel = 0.2f;        // 加速度0.2m/s^2
    path.decel = 0.3f;        // 减速度0.3m/s^2
    
    // 设置控制参数
    path.kp_cross = 0.03;     // 横向误差修正系数
    path.kd_cross = 0.0f;
    path.kp_yaw = 0.015;
    path.kd_yaw = 0.001;
    path.dt = 0.002f;         // 控制周期2ms
}


// 设置目标点 
// x：世界坐标系下目标点x坐标
// y：世界坐标系下目标点y坐标
// yaw：世界坐标系下目标点偏航角
// keep_yaw：是否保持偏航角，为1时调整偏航角为yaw，为0时调整偏航角朝向线坐标系方向
void path_set_target(float x, float y, float yaw, uint8_t keep_yaw) {
    // 初始化路径参数
    path_init();
    // 获取当前位置作为起点
    path.start_x = vision_get_pos(0);
    path.start_y = vision_get_pos(1);
    
    // 设置目标点
    path.target_x = x;
    path.target_y = y;
    
    // 计算路径角度
    path.angle = atan2f(y - path.start_y, x - path.start_x);
    
    // 目标偏航角
    path.yaw = yaw;

    // 计算总距离
    path.distance = sqrtf(powf(x - path.start_x, 2) + powf(y - path.start_y, 2));
    
    // 初始化速度
    path.current_speed = 0.0f;
    
    // 启动运动
    path.is_moving = 1;
}

float accel_distance = 0;
float decel_distance = 0;
// 更新路径状态
void path_update(void) {
    if (!path.is_moving) {
        return;
    }
    
    // 获取当前机体在世界坐标系下坐标
    path.current_x = vision_get_pos(0);
    path.current_y = vision_get_pos(1);

    // 获取当前机体偏航角
    float current_yaw = imu_get_data()->angle[2]; // 单位deg
    float current_w = imu_get_data()->gyro[2]; // 单位deg/s

    // 机体在世界坐标系下坐标转换到线坐标系下
    float line_x = cosf(path.angle) * path.current_x + sinf(path.angle) * path.current_y;
    float line_y = -sinf(path.angle) * path.current_x + cosf(path.angle) * path.current_y;
    
    // 机体坐标系下速度转换到世界坐标系下
    float world_current_vx = cosf(current_yaw * DEGREE_TO_RADIAN) * est_get_body_vel(0) - sinf(current_yaw * DEGREE_TO_RADIAN) * est_get_body_vel(1);
    float world_current_vy = sinf(current_yaw * DEGREE_TO_RADIAN) * est_get_body_vel(0) + cosf(current_yaw * DEGREE_TO_RADIAN) * est_get_body_vel(1);

    // 世界坐标系下速度转换到线坐标系下
    float line_vx = cosf(path.angle) * world_current_vx + sinf(path.angle) * world_current_vy;
    float line_vy = -sinf(path.angle) * world_current_vx + cosf(path.angle) * world_current_vy;

    // 计算当前到目标点的距离
    float current_distance = sqrtf(powf(path.target_x - path.current_x, 2) + 
                                 powf(path.target_y - path.current_y, 2));
    
    // 计算横向跟踪误差
    path.cross_track_error = line_y;
    
    // 如果到达目标点附近，停止运动
    if (current_distance < 5.0f) { // 5cm误差范围
        path.is_moving = 0;
        path.current_speed = 0.0f;
        dog_set_body_vel(0, 0, 0);
        fsm_change_to(STATE_STAND);
        return;
    }
    
    // 计算加速和减速所需的距离
    accel_distance = 100 * (path.max_speed * path.max_speed) / (2.0f * path.accel);
    decel_distance = 100 * (path.current_speed * path.current_speed) / (2.0f * path.decel);
    
    // 速度规划
    if (current_distance > (accel_distance + decel_distance)) {
        // 如果剩余距离大于加速和减速所需的距离，进入或保持匀速阶段
        if (path.current_speed < path.max_speed) {
            // 加速阶段
            path.current_speed += path.accel * path.dt;
            // 匀速阶段
            if (path.current_speed > path.max_speed) {
                path.current_speed = path.max_speed;
            }
        }
    } else {
        // 减速阶段
        path.current_speed -= path.decel * path.dt;
    }
    
    // 限制最小速度
    if (path.current_speed < 0.05f) {
        path.current_speed = 0.05f;
    }
    
    // 线坐标系y方向与yaw角pd矫正
    float line_correction_y = path.kp_cross * (0 - path.cross_track_error) + path.kd_cross * (0 - line_vy);
    float correction_w = path.kp_yaw * (path.yaw - current_yaw) + path.kd_yaw * (0 - current_w);

    // 线坐标系转世界坐标系
    float world_vx = cosf(path.angle) * path.current_speed - sinf(path.angle) * line_correction_y;
    float world_vy = sinf(path.angle) * path.current_speed + cosf(path.angle) * line_correction_y;

    // 世界坐标系转机体坐标系
    float body_vx = cosf(current_yaw * DEGREE_TO_RADIAN) * world_vx + sinf(current_yaw * DEGREE_TO_RADIAN) * world_vy;
    float body_vy = -sinf(current_yaw * DEGREE_TO_RADIAN) * world_vx + cosf(current_yaw * DEGREE_TO_RADIAN) * world_vy;

    // 设置运动速度
    dog_set_body_vel(
        body_vx,
        body_vy,
        correction_w
    );
}

// 检查是否到达目标点
uint8_t path_is_finished(void) {
    return !path.is_moving;
}

// 获取当前路径状态
void path_get_state(float *speed, float *cross_error) {
    if (speed != NULL) {
        *speed = path.current_speed;
    }
    if (cross_error != NULL) {
        *cross_error = path.cross_track_error;
    }
}
