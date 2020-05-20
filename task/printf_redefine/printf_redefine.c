#include "usart.h"
#include "stm32f7xx.h"

//重定向
#define PRINT_REDIRECT_UART huart5

//gcc编译器
#ifdef __GNUC__
int _write(int fd, unsigned char *pBuffer, int size)
{
    HAL_UART_Transmit(&PRINT_REDIRECT_UART, pBuffer, size, 1000);
    return size;
}
#endif

//ARM_CC编译器，及keil
#ifdef __CC_ARM
int fputc(int ch, FILE *f)
{
    /* 发送一个字节数据到串口DEBUG_USART */
    HAL_UART_Transmit(&PRINT_REDIRECT_UART, (uint8_t *)&ch, 1, 1000);
    return (ch);
}
#endif

//ICC_ARM编译器，即iar
#ifdef __ICCARM__
int fputc(int ch, FILE *f)
{
    /* 发送一个字节数据到串口DEBUG_USART */
    HAL_UART_Transmit(&PRINT_REDIRECT_UART, (uint8_t *)&ch, 1, 1000);
    return (ch);
}
#endif
