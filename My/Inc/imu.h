#ifndef __MOTOR_H__
#define __MOTOR_H__


typedef struct
{
    float acc[3];
    float gyro[3];
    float quaternion[4];
} IMU;

void vrep_imu_init(void);
void vrep_imu_deinit(void);
void imu_recv_update(void);
#endif

