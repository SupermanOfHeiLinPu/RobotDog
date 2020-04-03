#include "ps2_control.h"

#include <stdio.h>
int _PS2_Write_Byte(char data, unsigned char a[9])
{
    for (int i = 1; i <= 128; i <<= 1)
    {
        if (i & data)
        {
            CMD_H;
            PS2_Nop(500);
        }
        else
        {
            CMD_L;
            PS2_Nop(500);
        }
        CLK_H;
        PS2_Nop(500);
        CLK_L;
        PS2_Nop(500);
        CLK_H;
        if (DAT)
            a[1] = a[1] & i;
    }
    PS2_Nop(500);
    return 0;
}

int _PS2_Read_Data(unsigned char data[9])
{
    unsigned int i, j;
    CS_L;
    _PS2_Write_Byte(0X01, data);
    PS2_Nop(500);
    _PS2_Write_Byte(0X42, data);
    for (j = 2; j < 9; j++)
    {
        data[j] = 0;
        PS2_Nop(500);
        for (i = 1; i <= 128; i <<= 1)
        {
            CLK_H;
            PS2_Nop(500);
            CLK_L;
            PS2_Nop(500);
            CLK_H;
            if (DAT)
            {
                data[j] = i | data[j];
                //printf("1");
                PS2_Nop(500);
            }
            else
            {
                //printf("0");
                PS2_Nop(500);
            }
        }
        PS2_Nop(500);
        PS2_Nop(500);
        //printf("%d\n", j);
    }
    CS_H;
    return 0;
}
