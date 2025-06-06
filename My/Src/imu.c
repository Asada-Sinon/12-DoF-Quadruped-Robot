#include "imu.h"
#include "robot_params.h"
#include "stdint.h"
#include "stm32f7xx.h"
#include "usart.h"
#include "string.h"
#include "estimator.h" // 低通滤波器
#include "ANO_TC.h"

IMU imu_recv;
/* 接收数据并使用0x91数据包结构定义来解释数据 */
__align(4) id0x91_t dat; /* struct must be 4 byte aligned */
float imu_angle[3];             /* eular angles:R/P/Y */
s_LPFilter lpf_w;
uint8_t lpf_init_flag = 0;
float lpf_weight = 5;
static float ABS(float a)
{
	return a>0?a:-a;
}


// crc校验
static void crc16_update(uint16_t *currect_crc, const uint8_t *src, uint32_t len)
{
    uint32_t crc = *currect_crc;
    uint32_t j;
    for (j=0; j < len; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    } 
    *currect_crc = crc;
}


void imu_data_process(uint8_t *receive)
{
    // 初始化低通滤波器
    if(!lpf_init_flag){
        LPFilter_init(&lpf_w, 0.01, lpf_weight);
        lpf_init_flag = 1;
    }
    // 调试用
//    lpf_w.weight = 1.0f / ( 1.0f + 1.0f/(2.0f * PI * 0.01 * lpf_weight) );
    uint16_t CRCReceived = 0;            /* CRC value received from a frame */
    uint16_t CRCCalculated = 0;          /* CRC value caluated from a frame */
    uint8_t payload_len = 0;
    static float imu_Z_last = 0; // 上一时刻z角度,用于累计出z轴旋转的总角度
    float imu_Z_now = 0;
    float imu_Z_temp_1 = 0;
    float imu_Z_temp_2 = 0;
    float imu_Z_temp = 0;
    if(receive[0] == 0x5A && receive[1] == 0xA5) // 帧头
    {
        /* CRC */
        CRCReceived = receive[4] + (receive[5] << 8);
        payload_len = receive[2] + (receive[3] << 8);
        /* calculate CRC */
        crc16_update(&CRCCalculated, receive, 4);
        crc16_update(&CRCCalculated, receive + 6, payload_len);
        /* CRC match */
        if(CRCCalculated == CRCReceived)
        {
            memcpy(&dat, &receive[6], sizeof(id0x91_t));
            /* 计算Z轴累加旋转角度 */
            imu_Z_now = dat.eul[2];
            if(imu_Z_last <= imu_Z_now) 
            {
                imu_Z_temp_1 = imu_Z_now - imu_Z_last;
                imu_Z_temp_2 = imu_Z_now - imu_Z_last - 360;
            }
            else
            {
                imu_Z_temp_1 = imu_Z_now - imu_Z_last;
                imu_Z_temp_2 = imu_Z_now - imu_Z_last + 360;
            }
            imu_Z_temp = (ABS(imu_Z_temp_1))<(ABS(imu_Z_temp_2))? imu_Z_temp_1:imu_Z_temp_2;
            imu_Z_last = imu_Z_now;
            
            imu_angle[0] = dat.eul[0];
            imu_angle[1] = dat.eul[1];
            imu_angle[2] = imu_angle[2] + imu_Z_temp;
            
            imu_recv.acc[0] = dat.acc[0];
            imu_recv.acc[1] = dat.acc[1];
            imu_recv.acc[2] = dat.acc[2];

            LPFilter(&lpf_w, dat.gyr[2]);
            imu_recv.gyro[0] = dat.gyr[0];
            imu_recv.gyro[1] = dat.gyr[1];
            imu_recv.gyro[2] = LPF_get_value(&lpf_w);
//            set_debug_data(2, imu_recv.gyro[2]);
            imu_recv.w_original = dat.gyr[2];
//            set_debug_data(3, imu_recv.w_original);

            imu_recv.angle[0] = imu_angle[0];
            imu_recv.angle[1] = imu_angle[1];
            imu_recv.angle[2] = imu_angle[2];

            imu_recv.quaternion[0] = dat.quat[0];
            imu_recv.quaternion[1] = dat.quat[1];
            imu_recv.quaternion[2] = dat.quat[2];
            imu_recv.quaternion[3] = dat.quat[3];
        }
    }

}

IMU* imu_get_data(void)
{
    return &imu_recv;
}
