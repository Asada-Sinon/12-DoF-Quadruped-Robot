#include "vision.h"
#include "stdint.h"
#include "string.h"
VisionData vision_data;
VisionData vision_data_start; // ��ʼһ�ε����ߵ�ʶ�������������Ӿ�����

void vision_data_process(uint8_t *vision_recv)
{
    if (vision_recv[0] == 0x0a && vision_recv[1] == 0x0b && vision_recv[10] == 0x0c && vision_recv[11] == 0x0d) // �ж�֡ͷ֡β
    {
        memcpy(vision_data.recv, &vision_recv[2], 8);
    }
    if (vision_recv[0] == 0xcc && vision_recv[1] == 0xcc && vision_recv[10] == 0xcc && vision_recv[11] == 0xcc) // �ж�֡ͷ֡β
    {
        memcpy(vision_data_start.recv, &vision_recv[2], 8);
    }
}

float vision_get_pos(uint8_t axis)
{
    if (axis == 0)
        return vision_data.data.x;
    else if (axis == 1)
        return vision_data.data.y;
    else
        return 0;
}




