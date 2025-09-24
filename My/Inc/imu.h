#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "stdint.h"

typedef struct
{
    float acc[3];
    float gyro[3];
    float angle[3];
    float quaternion[4];
    float w_original;
} IMU;

__packed typedef  struct
{
        uint8_t     tag;                /* 0x91 */
        uint8_t     id;
        uint8_t     rev[6];             /* reserved */
        uint32_t    ts;                 /* timestamp */
        float       acc[3];
        float       gyr[3];
        float       mag[3];
        float       eul[3];             /* eular angles:R/P/Y */
        float       quat[4];            /* quaternion */

} id0x91_t;

void imu_data_process(uint8_t *receive);

void vrep_imu_init(void);
void vrep_imu_deinit(void);
void imu_recv_update(void);
IMU* imu_get_data(void);
#endif

