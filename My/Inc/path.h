#ifndef __PATH_H
#define __PATH_H

#include "stdint.h"

// 初始化路径规划器
void path_init(void);

// 设置目标点
void path_set_target(float x, float y, float yaw, uint8_t keep_yaw);

// 更新路径状态
void path_update(void);

// 检查是否到达目标点
uint8_t path_is_finished(void);

// 获取当前路径状态
void path_get_state(float *speed, float *cross_error);

#define PI 3.14159265358979323846f

#endif
