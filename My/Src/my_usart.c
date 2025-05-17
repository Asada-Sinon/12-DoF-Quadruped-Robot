#include "my_usart.h"
#include "usart.h"
#include "stdint.h"
#include "imu.h"

uint8_t imu_recv_data[100];
void usart1_start_recv() // imu
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, imu_recv_data, 100);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

void my_usart_init(void)
{
    usart1_start_recv();
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
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        usart1_start_recv();
    }
}
