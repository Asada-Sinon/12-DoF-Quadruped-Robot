#include "vision.h"
#include "stdint.h"
#include "string.h"
VisionData vision_data;
VisionData vision_data_start; // 开始一段倒着走的识别启动区红线视觉数据

void vision_data_process(uint8_t *vision_recv)
{
    if (vision_recv[0] == 0x0a && vision_recv[1] == 0x0d && vision_recv[10] == 0x0d && vision_recv[11] == 0x0a) // 判断帧头帧尾
    {
        memcpy(vision_data.recv, &vision_recv[2], 8);
    }
    if (vision_recv[0] == 0xcc && vision_recv[1] == 0xcc && vision_recv[10] == 0xcc && vision_recv[11] == 0xcc) // 判断帧头帧尾
    {
        memcpy(vision_data_start.recv, &vision_recv[2], 8);
    }
}

float vision_get_pos_x()
{
    return vision_data.data.x;
}

float vision_get_pos_y()
{
    return vision_data.data.y;
}

float vision_get_pos_x_start()
{
    return vision_data_start.data.x;
}


