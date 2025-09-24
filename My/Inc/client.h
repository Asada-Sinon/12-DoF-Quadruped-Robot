#ifndef __CLIENT_H__
#define __CLIENT_H__

#include "extApi.h"
#include "extApiPlatform.h"
#include "simConst.h"

#define SIM_PORT 5000
#define SIM_ADDRESS "127.0.0.1"

typedef struct
{
    simxInt joint_id[12];
}Object;

void sim_start_client(void);
void sim_get_handle(simxInt clientID);
simxInt sim_get_clientID(void);
simxInt sim_get_joint_id(uint8_t index);

#endif

