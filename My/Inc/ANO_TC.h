#ifndef ANO_TC_H
#define ANO_TC_H

#include "stdint.h"
#include "usart.h"
#define BIT16		short
#define  LAN16            10	
#define BIT32		float
#define  LAN32            12	

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)	) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

void set_debug_data(uint8_t idx, float data);
void send_debug_data(void);

#endif
