#include "gamepad.h"
#include "dog.h"
#include "fsm.h"
#include "stdlib.h"

int16_t _channels[16];
// 通道定义
#define LEFT_X_CH   3
#define LEFT_Y_CH   2
#define RIGHT_X_CH  0
#define RIGHT_Y_CH  1

#define SWITCH_CH1   4
#define SWITCH_CH2   5
#define SWITCH_CH3   6
#define SWITCH_CH4   7

#define CHANNEL_MIDDLE 992 

#define GAMEPAD_CONNECTED (_channels[0] || _channels[1] || _channels[2] || _channels[3] || _channels[4] || _channels[5] || _channels[6] || _channels[7] || _channels[8] || _channels[9] || _channels[10] || _channels[11] || _channels[12] || _channels[13] || _channels[14] || _channels[15])

// 拨杆位置阈值定义
#define SWITCH_UP_THRESHOLD      500
#define SWITCH_DOWN_THRESHOLD    1500

// 拨杆状态判断宏
#define SWITCH_UP(ch)          (_channels[ch] < SWITCH_UP_THRESHOLD)
#define SWITCH_DOWN(ch)        (_channels[ch] > SWITCH_DOWN_THRESHOLD)
#define SWITCH_MIDDLE(ch)      (_channels[ch] <= SWITCH_DOWN_THRESHOLD && _channels[ch] >= SWITCH_UP_THRESHOLD)


float vx = 0.0f;
float vy = 0.0f;
float w = 0.0f;

float vx_smooth = 0.0f;
float vy_smooth = 0.0f;
float w_smooth = 0.0f;

int dead_zone = 50;
int big_dead_zone = 200;
float v_inc = 0.005f;
float v_dead_zone = 0.05f;
float vx_scale = 0.005;
float vy_scale = 0.005;
float w_scale = 0.001;

// 重心调整步长
#define COG_ADJUST_STEP 0.01f

// 记录上一次摇杆位置
static int16_t prev_right_x = CHANNEL_MIDDLE;
static int16_t prev_right_y = CHANNEL_MIDDLE;

void HT10A_process(uint8_t buffer[30])
{
	_channels[0] = (buffer[1] | ((buffer[2] & 0x07) << 8)) & 0x07FF;
    // 通道1: bits 11-21 (byte2低5位 + byte3高6位)
    _channels[1] = ((buffer[2] >> 3) | (buffer[3] << 5)) & 0x07FF;   
    // 通道2: bits 22-32 (byte3低2位 + byte4全8位 + byte5高1位)
    _channels[2] = ((buffer[3] >> 6) | (buffer[4] << 2) | ((buffer[5] & 0x01) << 10)) & 0x07FF;  
    // 通道3: bits 33-43 (byte5低7位 + byte6高4位)
    _channels[3] = ((buffer[5] >> 1) | (buffer[6] << 7)) & 0x07FF;  
    // 通道4: bits 44-54 (byte6低4位 + byte7高7位)
    _channels[4] = ((buffer[6] >> 4) | (buffer[7] << 4)) & 0x07FF;    
    // 通道5: bits 55-65 (byte7低1位 + byte8全8位 + byte9高2位)
    _channels[5] = ((buffer[7] >> 7) | (buffer[8] << 1) | ((buffer[9] & 0x03) << 9)) & 0x07FF;    
    // 通道6: bits 66-76 (byte9低6位 + byte10高5位)
    _channels[6] = ((buffer[9] >> 2) | (buffer[10] << 6)) & 0x07FF;
    // 通道7: bits 77-87 (byte10低3位 + byte11全8位)
    _channels[7] = ((buffer[10] >> 5) | (buffer[11] << 3)) & 0x07FF;    
    // 通道8: bits 88-98 (byte12低8位 + byte13高3位)
    _channels[8] = (buffer[12] | ((buffer[13] & 0x07) << 8)) & 0x07FF;    
    // 通道9: bits 99-109 (byte13低5位 + byte14高6位)
    _channels[9] = ((buffer[13] >> 3) | (buffer[14] << 5)) & 0x07FF;    
    // 通道10: bits 110-120 (byte14低2位 + byte15全8位 + byte16高1位)
    _channels[10] = ((buffer[14] >> 6) | (buffer[15] << 2) | ((buffer[16] & 0x01) << 10)) & 0x07FF;    
    // 通道11: bits 121-131 (byte16低7位 + byte17高4位)
    _channels[11] = ((buffer[16] >> 1) | (buffer[17] << 7)) & 0x07FF;    
    // 通道12: bits 132-142 (byte17低4位 + byte18高7位)
    _channels[12] = ((buffer[17] >> 4) | (buffer[18] << 4)) & 0x07FF;    
    // 通道13: bits 143-153 (byte18低1位 + byte19全8位 + byte20高2位)
    _channels[13] = ((buffer[18] >> 7) | (buffer[19] << 1) | ((buffer[20] & 0x03) << 9)) & 0x07FF;    
    // 通道14: bits 154-164 (byte20低6位 + byte21高5位)
    _channels[14] = ((buffer[20] >> 2) | (buffer[21] << 6)) & 0x07FF;    
    // 通道15: bits 165-175 (byte21低3位 + byte22全8位)
    _channels[15] = ((buffer[21] >> 5) | (buffer[22] << 3)) & 0x07FF;
    
    if (_channels[LEFT_Y_CH] - CHANNEL_MIDDLE > dead_zone)
        vx = (_channels[LEFT_Y_CH] - CHANNEL_MIDDLE - dead_zone) * vx_scale;
    else if (_channels[LEFT_Y_CH] - CHANNEL_MIDDLE < -dead_zone)
        vx = (_channels[LEFT_Y_CH] - CHANNEL_MIDDLE + dead_zone) * vx_scale;
    else
        vx = 0;

    if (_channels[LEFT_X_CH] - CHANNEL_MIDDLE > dead_zone)
        vy = -(_channels[LEFT_X_CH] - CHANNEL_MIDDLE - dead_zone) * vy_scale;
    else if (_channels[LEFT_X_CH] - CHANNEL_MIDDLE < -dead_zone)
        vy = -(_channels[LEFT_X_CH] - CHANNEL_MIDDLE + dead_zone) * vy_scale;
    else
        vy = 0;
    
    if (_channels[RIGHT_X_CH] - CHANNEL_MIDDLE > dead_zone)
        w = -(_channels[RIGHT_X_CH] - CHANNEL_MIDDLE - dead_zone) * w_scale;
    else if (_channels[RIGHT_X_CH] - CHANNEL_MIDDLE < -dead_zone)
        w = -(_channels[RIGHT_X_CH] - CHANNEL_MIDDLE + dead_zone) * w_scale;
    else
        w = 0;
}

uint8_t start_gamepad_control = 0;
void gamepad_control_init()
{
    start_gamepad_control = 1;
}

// 手柄控制机体速度
void gamepad_control()
{
    if (!start_gamepad_control || !GAMEPAD_CONNECTED)
        return;
        
    // 当拨杆4在中间位置时调整前进后退的重心
    if (!SWITCH_UP(SWITCH_CH4)) {
        // 获取当前重心位置
        float *cog_forward_offset = get_dog_params()->posture.center_of_gravity.trot_cog_forward_offset;
        float *cog_backward_offset = get_dog_params()->posture.center_of_gravity.trot_cog_backward_offset;
        
        // 检测右摇杆Y轴（前后）变化，调整机体X方向重心
        if (abs(_channels[RIGHT_Y_CH] - CHANNEL_MIDDLE) > big_dead_zone && 
            abs(prev_right_y - CHANNEL_MIDDLE) <= big_dead_zone) {
            // 从中位开始推动摇杆的瞬间
            if (_channels[RIGHT_Y_CH] > CHANNEL_MIDDLE) {
                // 向前推
                if (SWITCH_MIDDLE(SWITCH_CH4)) { // 如果拨杆4在中间位置
                    cog_forward_offset[X_IDX] += COG_ADJUST_STEP;
                }
                else if(SWITCH_DOWN(SWITCH_CH4)) { // 如果拨杆4在向下位置
                    cog_backward_offset[X_IDX] += COG_ADJUST_STEP;
                }
            }   
            else {
                // 向后推
                if (SWITCH_MIDDLE(SWITCH_CH4)) { // 如果拨杆4在中间位置
                    cog_forward_offset[X_IDX] -= COG_ADJUST_STEP;
                }
                else if(SWITCH_DOWN(SWITCH_CH4)) { // 如果拨杆4在向下位置
                    cog_backward_offset[X_IDX] -= COG_ADJUST_STEP;
                }
            }
        }
        
        // 检测右摇杆X轴（左右）变化，调整Y方向重心
        if (abs(_channels[RIGHT_X_CH] - CHANNEL_MIDDLE) > big_dead_zone && 
            abs(prev_right_x - CHANNEL_MIDDLE) <= big_dead_zone) {
            // 从中位开始推动摇杆的瞬间
            if (_channels[RIGHT_X_CH] > CHANNEL_MIDDLE) {
                // 向右推
                if (SWITCH_MIDDLE(SWITCH_CH4)) { // 如果拨杆4在中间位置
                    cog_forward_offset[Y_IDX] -= COG_ADJUST_STEP;
                }
                else if(SWITCH_DOWN(SWITCH_CH4)) { // 如果拨杆4在向下位置
                    cog_backward_offset[Y_IDX] -= COG_ADJUST_STEP;
                }
            } else {
                // 向左推
                if (SWITCH_MIDDLE(SWITCH_CH4)) { // 如果拨杆4在中间位置
                    cog_forward_offset[Y_IDX] += COG_ADJUST_STEP;
                }
                else if(SWITCH_DOWN(SWITCH_CH4)) { // 如果拨杆4在向下位置
                    cog_backward_offset[Y_IDX] += COG_ADJUST_STEP;
                }
            }
        }
        // 更新上一次摇杆位置
        prev_right_x = _channels[RIGHT_X_CH];
        prev_right_y = _channels[RIGHT_Y_CH];
        w = 0;
    } else {
        // 不在重心调整模式时，重置摇杆位置记录
        prev_right_x = CHANNEL_MIDDLE;
        prev_right_y = CHANNEL_MIDDLE;
    }
    
    // 计算机体速度
    vx_smooth = smooth(vx_smooth, vx, v_inc);
    vy_smooth = smooth(vy_smooth, vy, v_inc);
    w_smooth = smooth(w_smooth, w, v_inc);
    dog_set_body_vel(vx_smooth, vy_smooth, w_smooth);
    
    if (SWITCH_DOWN(SWITCH_CH3)) {
        fsm_change_to(STATE_PASSIVE);
        return;
    }   
    
    if (fabs(vx_smooth) < v_dead_zone && 
        fabs(vy_smooth) < v_dead_zone && 
        fabs(w_smooth) < v_dead_zone && 
        SWITCH_UP(SWITCH_CH2)) {
        fsm_change_to(STATE_STAND);
    } else {
        fsm_change_to(STATE_TROT);
    } 
}
