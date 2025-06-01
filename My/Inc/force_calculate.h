#ifndef __FORCE_CALCULATE_H__
#define __FORCE_CALCULATE_H__
#include "stdint.h"

void vmc_get_foot_target_force(uint8_t leg_id, float foot_target_force[3]);
void vmc_force_calculate(void);
void vmc_set_force_control(uint8_t x);
uint8_t vmc_get_force_control_state(void);
#endif