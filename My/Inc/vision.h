#ifndef _VISION_H
#define _VISION_H
#include "stdint.h"

#define VISION_ERROR 10000
typedef union
{
    uint8_t recv[8];
    struct
    {
        float x;
        float y;
    }data;
}VisionData;



void vision_data_process(uint8_t *vision_recv);

float vision_get_pos(uint8_t axis);

#endif 
