#include "usart.h"
#include "stm32f7xx.h"

int _write(int fd, char *pBuffer, int size)
{
    HAL_UART_Transmit(&huart5, pBuffer, size, 1000);
    return size;
}