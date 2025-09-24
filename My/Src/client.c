#include "client.h"
#include "extApi.h"
#include "extApiPlatform.h"
#include "stdio.h"


//服务器
simxInt Port = 5000;
simxChar Adresse[] = "127.0.0.1";
simxInt clientID = -1;
// Vrep中控件句柄
Object object; 

// 获取vrep中关节句柄
void sim_get_handle(simxInt clientID)
{
    simxGetObjectHandle(clientID, "FL_hip_joint", &object.joint_id[0], simx_opmode_blocking);
    simxGetObjectHandle(clientID, "FL_thigh_joint", &object.joint_id[1], simx_opmode_blocking);
    simxGetObjectHandle(clientID, "FL_calf_joint", &object.joint_id[2], simx_opmode_blocking);

    simxGetObjectHandle(clientID, "FR_hip_joint", &object.joint_id[3], simx_opmode_blocking);
    simxGetObjectHandle(clientID, "FR_thigh_joint", &object.joint_id[4], simx_opmode_blocking);
    simxGetObjectHandle(clientID, "FR_calf_joint", &object.joint_id[5], simx_opmode_blocking);

    simxGetObjectHandle(clientID, "RL_hip_joint", &object.joint_id[6], simx_opmode_blocking);
    simxGetObjectHandle(clientID, "RL_thigh_joint", &object.joint_id[7], simx_opmode_blocking);
    simxGetObjectHandle(clientID, "RL_calf_joint", &object.joint_id[8], simx_opmode_blocking);

    simxGetObjectHandle(clientID, "RR_hip_joint", &object.joint_id[9], simx_opmode_blocking);
    simxGetObjectHandle(clientID, "RR_thigh_joint", &object.joint_id[10], simx_opmode_blocking);
    simxGetObjectHandle(clientID, "RR_calf_joint", &object.joint_id[11], simx_opmode_blocking);
    
    for (uint8_t i = 0; i < 12; i++)
    {
        if (object.joint_id[i] == 0)
            printf_s("\nJoint%d ERROR",i);
    }
}

void sim_start_client()
{
    clientID = simxStart(Adresse, Port, 1, 1, 2000, 5);
    if (clientID != -1)
    {
        printf("V-rep connected.");
        extApi_sleepMs(300);
    }
    else
    {
        printf("V-rep can't be connected.");
        extApi_sleepMs(300);
    }
}


simxInt sim_get_clientID()
{
    return clientID;
}

simxInt sim_get_joint_id(uint8_t index)  
{
    return object.joint_id[index];
}
