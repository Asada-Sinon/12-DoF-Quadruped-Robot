#ifndef GAMEPAD_H
#define GAMEPAD_H
#include "stdint.h"
/**
 * @brief �ֱ����ƻ����ٶ�
 */
void gamepad_control(void);
void HT10A_process(uint8_t buffer[30]);
void gamepad_control_init(void);
float get_origin_target_body_vel(int idx);
#endif

