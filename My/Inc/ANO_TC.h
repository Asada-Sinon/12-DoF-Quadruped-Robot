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

void ANO_TC_To_Send(void);
void ano_tc(float *data);
void Usart_Send_To_Show32(UART_HandleTypeDef* huart, float *data_ts);

#endif
