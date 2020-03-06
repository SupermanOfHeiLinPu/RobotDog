#ifndef __I2C_H__
#define __I2C_H__

//cube生成的i2c
#include "i2c.h"

#include <stdint.h>
#include "stm32f7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

//mpu6050官方库相关
#define bsp_i2c_Delay vTaskDelay

extern int Sensors_I2C_WriteRegister(unsigned char slave_addr,
                                     unsigned char reg_addr,
                                     unsigned short len,
                                     unsigned char *data_ptr);
extern int Sensors_I2C_ReadRegister(unsigned char slave_addr,
                                    unsigned char reg_addr,
                                    unsigned short len,
                                    unsigned char *data_ptr);
extern int get_tick_count(unsigned long *count);

#endif
