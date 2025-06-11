#ifndef __PATH_H
#define __PATH_H

#include "stdint.h"

// ��ʼ��·���滮��
void path_init(void);

// ����Ŀ���
void path_set_target(float x, float y, float yaw, uint8_t keep_yaw);

// ����·��״̬
void path_update(void);

// ����Ƿ񵽴�Ŀ���
uint8_t path_is_finished(void);

// ��ȡ��ǰ·��״̬
void path_get_state(float *speed, float *cross_error);

#define PI 3.14159265358979323846f

#endif
