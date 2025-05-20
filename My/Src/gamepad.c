#include "gamepad.h"
#include "dog.h"
#include "fsm.h"

float vx = 0.0f;
float vy = 0.0f;
float w = 0.0f;

float vx_smooth = 0.0f;
float vy_smooth = 0.0f;
float w_smooth = 0.0f;

float default_vx = 0.8f;
float default_vy = 0.8f;
float default_w = 0.5f;

int16_t _channels[16];

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
    
    int middle = 992;
    int dead_zone = 50;
    float vx_scale = 0.001;
    float vy_scale = 0.001;
    float w_scale = 0.001;
    if (_channels[2] - middle > dead_zone)
        vx = (_channels[2] - middle - dead_zone) * vx_scale;
    else if (_channels[2] - middle < -dead_zone)
        vx = (_channels[2] - middle + dead_zone) * vx_scale;
    else
        vx = 0;

    if (_channels[3] - middle > dead_zone)
        vy = -(_channels[3] - middle - dead_zone) * vy_scale;
    else if (_channels[3] - middle < -dead_zone)
        vy = -(_channels[3] - middle + dead_zone) * vy_scale;
    else
        vy = 0;
    
    if (_channels[0] - middle > dead_zone)
        w = -(_channels[0] - middle - dead_zone) * w_scale;
    else if (_channels[0] - middle < -dead_zone)
        w = -(_channels[0] - middle + dead_zone) * w_scale;
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
    if (!start_gamepad_control)
        return;
//    if (_kbhit())//检测键盘缓冲区中是否有数据
//    {
//        char ch = _getch();
//        printf("%c",ch);
//        switch (ch)
//        {
//        case 'w':
//            vx = default_vx;
//            break;
//        case 's':
//            vx = -default_vx;
//            break;
//        case 'a':
//            vy = default_vy;
//            break;
//        case 'd':
//            vy = -default_vy;
//            break;
//        case 'q':
//            w = default_w;
//            break;
//        case 'e':
//            w = -default_w;
//            break;
//        case 'z':
//            vx = vy = w = 0.0f;
//            break;
//        }
//    }
    vx_smooth = smooth(vx_smooth, vx, 0.005f);
    vy_smooth = smooth(vy_smooth, vy, 0.005f);
    w_smooth = smooth(w_smooth, w, 0.005f);
    dog_set_body_vel(vx_smooth, vy_smooth, w_smooth);
    if (_channels[6] > 1700)
    {
        fsm_change_to(STATE_PASSIVE);
        return;
    }   
    if (fabs(vx_smooth) < 0.05f && fabs(vy_smooth) < 0.05f && fabs(w_smooth) < 0.05f) {
        fsm_change_to(STATE_STAND);
    }
    else
    {
        fsm_change_to(STATE_TROT);
    }
    
}
