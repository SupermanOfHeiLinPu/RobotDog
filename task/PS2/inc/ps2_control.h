#ifndef __PS2_CONTROL_H__
#define __PS2_CONTROL_H__

#include "gpio.h"
#include "main.h"

#define PS2_Nop(x)              \
    for (int k = 0; k < x; k++) \
        __NOP();

#define DAT HAL_GPIO_ReadPin(PS2_DAT_GPIO_Port, PS2_DAT_Pin)
#define CMD_H HAL_GPIO_WritePin(PS2_CMD_GPIO_Port, PS2_CMD_Pin, GPIO_PIN_SET)
#define CMD_L HAL_GPIO_WritePin(PS2_CMD_GPIO_Port, PS2_CMD_Pin, GPIO_PIN_RESET)
#define CS_H HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_SET)
#define CS_L HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_RESET)
#define CLK_H HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_SET)
#define CLK_L HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_RESET)

extern int _PS2_Read_Data(unsigned char data[9]);

#endif
