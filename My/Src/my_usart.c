#include "my_usart.h"
#include "usart.h"
#include "stdint.h"
#include "imu.h"
#include "gamepad.h"

uint8_t imu_recv_data[100];
uint8_t gamepad_buffer[30];
void usart1_start_recv() // imu
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, imu_recv_data, 100);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

void usart2_start_recv(void) // Ò£¿Ø
{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2,gamepad_buffer,30);
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
}

void my_usart_init(void)
{
    usart1_start_recv();
    usart2_start_recv();
}


uint16_t size;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    size = Size;
    if (huart->Instance == USART1) // imu
    {
        HAL_UART_DMAStop(huart);
        imu_data_process(imu_recv_data);
        usart1_start_recv();
    }
    if (huart->Instance == USART2) // Ò£¿Ø
    {
        HAL_UART_DMAStop(huart);
        HT10A_process(gamepad_buffer);
        usart2_start_recv();
    }
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        usart1_start_recv();
    }
    if (huart->Instance == USART2)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        usart2_start_recv();
    }
}
