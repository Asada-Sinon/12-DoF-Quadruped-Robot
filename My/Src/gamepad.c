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
// 手柄控制机体速度
void gamepad_control()
{
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
    vx_smooth = smooth(vx_smooth, vx, 0.05f);
    vy_smooth = smooth(vy_smooth, vy, 0.05f);
    w_smooth = smooth(w_smooth, w, 0.05f);
    dog_set_body_vel(vx_smooth, vy_smooth, w_smooth);
    if (fabs(vx_smooth) < 0.05f && fabs(vy_smooth) < 0.05f && fabs(w_smooth) < 0.05f) {
        fsm_change_to(STATE_STAND);
    }
    else
    {
        fsm_change_to(STATE_TROT);
    }
}
